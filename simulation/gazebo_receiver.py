"""Native wheel interlock configuration and status submission gates, not wheel feedback."""
from __future__ import annotations
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from ros2_ws.src.toponav_bridge.toponav_bridge.protocol import decode, identity, number, validate_command

ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT/'.simulators/toponav-receiver/libtoponav-receiver.so'
BUILD = BINARY.with_name('build.json')
SOURCES = ('simulation/native/receiver_lease.hh', 'simulation/native/receiver_envelope.hh',
           'simulation/native/receiver_status.hh',
           'simulation/native/gazebo_receiver_interlock.cc',
           'tools/build_gazebo_receiver.sh')
MAX_RECEIVER_FRAMES = 4096  # Must match ReceiverLease::kMaxCommandFrames.
MAX_RECEIVER_EVENTS = 65536  # Strict production history, never a rolling window.
MAX_HISTORY_NOT_RETAINED = 2**64-1  # Saturating diagnostic count, not an authority.
MAX_RECEIVER_CANDIDATES = 8192  # Sent, not accepted; native frame limit stays 4096.
MAX_RECEIVER_SUBMISSIONS = 16384  # Strict post-send ledger prefix, not a queue.
MAX_SUBMISSION_NOT_RETAINED = 2**64-1  # Saturating UNVALIDATED observer-call count.


def verify_binary():
    build = json.loads(BUILD.read_text())
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    if build['binary_sha256'] != sha(BINARY) or any(build['sources'].get(n) != sha(ROOT/n) for n in SOURCES):
        raise RuntimeError('Missing/stale receiver interlock; run tools/build_gazebo_receiver.sh')
    if build['gazebo_runtime_sha256'] != sha(ROOT/'.simulators/gazebo-harmonic/usr/lib/x86_64-linux-gnu/libgz-sim8.so.8.15.0'):
        raise RuntimeError('Receiver Gazebo ABI/runtime mismatch; rebuild against matched runtime')
    return build


def install_interlock(world_text, session):
    """Only add the final command interlock; never modify physical geometry."""
    if not identity(session) or not all(c.isascii() and (c.isalnum() or c in '-_') for c in session):
        raise ValueError('Invalid receiver session')
    tree = ET.fromstring(world_text)
    robot = tree.find("world/model[@name='robot']")
    drive = robot.find("plugin[@name='gz::sim::systems::DiffDrive']") if robot is not None else None
    if (drive is None or len(robot.findall("plugin[@name='gz::sim::systems::DiffDrive']")) != 1
            or robot.find("plugin[@name='toponav::ReceiverInterlock']") is not None):
        raise ValueError('Receiver requires exactly one original DiffDrive fixture')
    expected = dict(left_joint='left_wheel_joint', right_joint='right_wheel_joint',
                    wheel_separation='.30', wheel_radius='.08', topic='/toponav/cmd_vel')
    for field, value in expected.items():
        actual = drive.findtext(field)
        if actual is None or (float(actual) != float(value) if field.startswith('wheel_') else actual != value):
            raise ValueError('Unsupported receiver drive geometry/topic')
    if any(n.tag.endswith('system_priority') and n.text != '0' for n in drive):
        raise ValueError('Receiver requires original DiffDrive priority zero')
    plugin = ET.SubElement(robot, 'plugin', filename=str(BINARY), name='toponav::ReceiverInterlock')
    ET.SubElement(plugin, 'session_id').text = session
    ET.indent(tree)
    return ET.tostring(tree, encoding='unicode')


class ReceiverEvidence:
    """Transport status is not wheel feedback, braking or contact proof."""
    def __init__(self, session, *, bind_submissions=False, require_frame_echo=False):
        if require_frame_echo and not bind_submissions:
            raise ValueError('Frame echo requires actual submission binding')
        self.session = session
        self.events = []
        self.error = None
        self._last_ready_wall_s = None
        self.bind_submissions = bind_submissions
        self.require_frame_echo = require_frame_echo
        self._history_not_retained = 0
        self.submissions = []
        self._submission_not_retained = 0
        self._submitted_commands = {}
        self._candidate_capacity_exceeded = False
        self._submitted_seq = 0
        self._terminal_sent = False

    def record_submission(self, packet, sent_steady_s):
        """Called under the IO condition, AFTER atomic send, BEFORE ACK wait.

        Sent is not accepted or physically applied. Keep unconfirmed sends
        for failed-channel zero observation, without granting any new lease.
        """
        # AFTER send: never turn a full audit prefix into a channel failure
        # that prevents zero/STOP. Omitted calls are NOT validated records;
        # do not copy them, advance retained sequence state, or add candidates.
        if self.require_frame_echo and len(self.submissions) >= MAX_RECEIVER_SUBMISSIONS:
            self._submission_not_retained = min(self._submission_not_retained+1, MAX_SUBMISSION_NOT_RETAINED)
            if self.error is None:
                self.error = 'RuntimeError: Receiver submission history capacity exceeded'
            return
        from simulation.gazebo_actuator import LEASE_S
        try:
            packet = copy.deepcopy(packet)
            base = {'kind', 'session_id', 'request_id'}
            if (not self.bind_submissions or not isinstance(packet, dict)
                    or not base <= packet.keys() or packet['session_id'] != self.session
                    or type(packet['request_id']) is not int or packet['request_id'] != len(self.submissions)+1
                    or not number(sent_steady_s) or sent_steady_s < 0
                    or (self.submissions and sent_steady_s < self.submissions[-1]['sent_steady_s'])):
                raise ValueError('Invalid submission identity/order/clock')
            kind = packet['kind']
            if kind == 'command':
                if set(packet) != base | {'command', 'sim_s', 'issued_steady_s', 'expires_steady_s'}:
                    raise ValueError('Invalid submission fields')
                c = validate_command(packet['command'])
                issued, expires = packet['issued_steady_s'], packet['expires_steady_s']
                if (self._terminal_sent or c['session_id'] != self.session
                        or c['seq'] != self._submitted_seq+1 or c['seq'] > 2**64-1
                        or not all(number(v) for v in (issued, expires, packet['sim_s']))
                        or not 0 <= issued <= sent_steady_s < expires <= issued+LEASE_S+1e-9
                        or not 0 <= c['captured_sim_s'] <= c['decision_sim_s'] <= packet['sim_s']+1e-8):
                    raise ValueError('Invalid submitted command authority')
                self._submitted_seq = c['seq']
                # Terminal commands publish STOP, not a new native command.
                # ReceiverLease retains its PRIOR seq/deadline when stopping.
                self._terminal_sent = c['state'] in ('visual_stop', 'aborted')
                if not self._terminal_sent:
                    # This observer runs AFTER send. Exhaustion must not throw
                    # and close an otherwise healthy zero/STOP channel. Keep
                    # the full candidate prefix and actual submission ledger,
                    # but never grant motion/completion from incomplete binding.
                    if self.require_frame_echo and len(self._submitted_commands) >= MAX_RECEIVER_CANDIDATES:
                        self._candidate_capacity_exceeded = True
                        if self.error is None:
                            self.error = 'RuntimeError: Receiver command candidate capacity exceeded'
                    else:
                        self._submitted_commands[c['seq']] = dict(sent_steady_s=sent_steady_s,
                            deadline_steady_s=min(expires, issued+LEASE_S),
                            frame_id_hex=c['frame_id'].encode('utf-8').hex(),
                            captured_sim_s=c['captured_sim_s'], issued_steady_s=issued,
                            wheels=[c['v_m_s'], c['omega_rad_s']])
            elif kind not in ('idle', 'stop', 'shutdown') or set(packet) != base:
                raise ValueError('Invalid submission kind/fields')
            self.submissions.append(dict(packet=packet, sent_steady_s=sent_steady_s))
        except (ValueError, TypeError, KeyError, OverflowError, RecursionError) as exc:
            if self.error is None:
                self.error = f'{type(exc).__name__}: Invalid receiver submission ledger: {exc}'
            raise RuntimeError('Receiver submission recording failed: '+str(exc)) from exc

    def _validate_submission_binding(self, item):
        if not self.bind_submissions or item['command_seq'] == 0:
            return  # Initial disarmed/fault-zero states grant no command.
        expected = self._submitted_commands.get(item['command_seq'])
        if (expected is None or item['wall_s'] < expected['sent_steady_s']
                or item['deadline_steady_s'] != expected['deadline_steady_s']
                or (item['version'] == 2 and any(item[n] != expected[n] for n in
                    ('frame_id_hex', 'captured_sim_s', 'issued_steady_s')))
                or (not item['latched'] and item['wheels'] != expected['wheels'])):
            raise ValueError('Receiver status differs from actual submission')

    def _validate_frame_echo(self, item):
        if item['version'] == 1:
            if self.require_frame_echo:
                raise ValueError('Receiver frame echo required; v1 downgrade refused')
            return  # Explicit historical/offline compatibility, never production.
        fields = ('frame_id_hex', 'captured_sim_s', 'issued_steady_s')
        if item['command_seq'] == 0:
            if any(item[n] is not None for n in fields):
                raise ValueError('Uncommanded receiver has frame echo')
            return
        frame, captured, issued = (item[n] for n in fields)
        if (not isinstance(frame, str) or not 2 <= len(frame) <= 512 or len(frame) % 2
                or any(c not in '0123456789abcdef' for c in frame)
                or not number(captured) or captured < 0
                or not number(issued) or not 0 <= issued <= item['wall_s']
                or not issued < item['deadline_steady_s'] <= issued+.8):
            raise ValueError('Invalid receiver frame echo')

    @staticmethod
    def _validate_authority(item):
        """Only states the native lease can actually publish are evidence.

        Compare deadlines to the status's OWN wall stamp, not receipt time:
        transport delay is not a renewal and this is not a new heartbeat.
        """
        seq, deadline = item['command_seq'], item['deadline_steady_s']
        wall, reason = item['wall_s'], item['reason']
        if (seq > 2**64-1 or (seq == 0 and deadline != 0)
                or (seq > 0 and deadline <= 0)
                or (reason == 'disarmed' and (seq != 0 or item['wheels'] != [0.,0.]))
                or (reason == 'accepted' and (seq == 0 or not wall < deadline <= wall+.8))
                or (reason == 'receiver_capacity_exceeded' and seq != MAX_RECEIVER_FRAMES)
                or (reason == 'command_lease_expired' and (seq == 0 or wall < deadline))):
            raise ValueError('Invalid receiver authority state')

    def push(self, raw):
        # The strict production path keeps a finite PREFIX, never evicts old
        # evidence and never treats unretained input as a fresh status. Do not
        # parse further payloads after exhaustion; the original error stays
        # sticky and zero/STOP requests remain independent of this gate.
        if self.require_frame_echo and len(self.events) >= MAX_RECEIVER_EVENTS:
            self._history_not_retained = min(self._history_not_retained+1, MAX_HISTORY_NOT_RETAINED)
            if self.error is None:
                self.error = 'RuntimeError: Receiver status history capacity exceeded'
            return
        try:
            item = decode(raw)
            fields = {'version','session_id','wall_s','sim_s','latched','reason','command_seq','deadline_steady_s','wheels'}
            version = item.get('version')
            if type(version) is int and version == 2:
                fields |= {'frame_id_hex', 'captured_sim_s', 'issued_steady_s'}
            if (set(item) != fields
                    or type(version) is not int or version not in (1, 2) or item['session_id'] != self.session
                    or type(item['latched']) is not bool or not isinstance(item['reason'], str)
                    or item['reason'] not in {'disarmed','accepted','remote_stop','command_lease_expired',
                                              'invalid_receiver_envelope','steady_clock_regression',
                                              'receiver_capacity_exceeded'}
                    or (item['latched'] == (item['reason'] in {'disarmed','accepted'}))
                    or type(item['command_seq']) is not int or item['command_seq'] < 0
                    or not all(number(item[n]) and item[n] >= 0 for n in ('wall_s','sim_s','deadline_steady_s'))
                    or not isinstance(item['wheels'], list) or len(item['wheels']) != 2
                    or not all(number(v) for v in item['wheels']) or not 0 <= item['wheels'][0] <= .15
                    or abs(item['wheels'][1]) > .30 or (item['latched'] and item['wheels'] != [0.,0.])):
                raise ValueError('Invalid receiver status')
            self._validate_authority(item)
            self._validate_frame_echo(item)
            self._validate_submission_binding(item)
            if self.events:
                last = self.events[-1]
                if (item['wall_s'] < last['wall_s'] or item['sim_s'] < last['sim_s']
                        or item['version'] < last['version']
                        or item['command_seq'] < last['command_seq']
                        or (item['version'] == last['version'] == 2
                            and ((item['command_seq'] == last['command_seq'] and any(item[n] != last[n]
                                  for n in ('frame_id_hex', 'captured_sim_s', 'issued_steady_s')))
                                 or (item['command_seq'] > last['command_seq'] and last['command_seq'] > 0
                                     and item['captured_sim_s'] <= last['captured_sim_s'])))
                        or (item['command_seq'] == last['command_seq']
                            and (item['deadline_steady_s'] != last['deadline_steady_s']
                                 or (not item['latched'] and item['wheels'] != last['wheels'])))
                        or (last['latched'] and (not item['latched'] or item['reason'] != last['reason']
                                                or item['command_seq'] != last['command_seq']))):
                    raise ValueError('Receiver state regressed')
            self.events.append(item)
        except (ValueError, TypeError, KeyError, OverflowError, RecursionError) as exc:
            if self.error is None: self.error = f'{type(exc).__name__}: {exc}'

    def require_ready(self):
        """Structural readiness only, also used by offline/historical audits."""
        if self.error or not self.events or self.events[-1]['latched']:
            raise RuntimeError('Receiver interlock not ready or fault-latched: '+str(self.error or self.events[-1:]))

    def require_fresh(self, now_steady_s):
        """Runtime prerequisite for nonzero submission; failures are sticky.

        Same-host monotonic PUBLICATION time, never receipt time. A newer
        status cannot extend an accepted command's original absolute lease.
        Zero/stop submission must remain independent of this status gate.
        """
        from simulation.gazebo_actuator import LEASE_S
        try:
            self.require_ready()
            if not number(now_steady_s) or now_steady_s < 0:
                raise RuntimeError('Receiver interlock clock invalid')
            if self._last_ready_wall_s is not None and now_steady_s < self._last_ready_wall_s:
                raise RuntimeError('Receiver interlock clock regressed')
            final = self.events[-1]
            if not 0 <= now_steady_s-final['wall_s'] < LEASE_S:
                raise RuntimeError('Receiver interlock status stale or from the future')
            if final['reason'] == 'accepted' and now_steady_s >= final['deadline_steady_s']:
                raise RuntimeError('Receiver interlock authority expired')
            if self.bind_submissions and self._submitted_commands and final['command_seq'] == 0:
                raise RuntimeError('Receiver interlock lacks submitted command feedback')
            self._last_ready_wall_s = now_steady_s
        except RuntimeError as exc:
            if self.error is None:
                self.error = f'{type(exc).__name__}: {exc}'
            raise

    def summary(self, last_step_started, final_sim):
        final = self.events[-1] if self.events else None
        fresh = bool(final and number(last_step_started) and number(final_sim)
                     and last_step_started < final['sim_s'] <= final_sim+1e-6)
        fault = bool(self.error or not final or any(e['latched'] and e['reason'] != 'remote_stop' for e in self.events))
        result = dict(events=len(self.events), error=self.error, final=final, final_step_fresh=fresh,
            fault_latched=fault, final_zero=bool(final and final['latched'] and final['wheels'] == [0.,0.]),
            scope='Native final wheel-command authority only; NOT measured physical braking or hardware certification')
        if self.bind_submissions:
            latest = max(self._submitted_commands, default=0)
            bound = bool(final and final['command_seq'] == latest and not self._candidate_capacity_exceeded
                         and not self._submission_not_retained)
            result['submission_binding'] = dict(required=True, sent_requests=len(self.submissions),
                native_command_candidates=len(self._submitted_commands), latest_native_candidate_seq=latest,
                final_authority_bound=bound, frame_echo_required=self.require_frame_echo,
                frame_echo_verified=bool(bound and latest and final['version'] == 2 and not self.error),
                scope='Sent sequence/velocity/original deadline and v2 accepted frame/capture/issued echo; '
                      'sent is NOT accepted/applied; echo is NOT image content or source authentication')
            result['fault_latched'] = result['fault_latched'] or not bound
        if self.require_frame_echo:
            result['submission_history'] = dict(limit=MAX_RECEIVER_SUBMISSIONS, retained=len(self.submissions),
                capacity_exceeded=bool(self._submission_not_retained), not_retained=self._submission_not_retained,
                not_retained_count_is_lower_bound=self._submission_not_retained == MAX_SUBMISSION_NOT_RETAINED,
                sent_requests_field_is_retained_count=True,
                scope='Bounded post-send callback prefix only; omitted calls are NOT copied/validated/candidates; '
                      'not_retained counts UNVALIDATED observer calls, NOT valid sends or native rejections; '
                      'NOT pose/transport/disk/whole-process memory bounds or source authentication')
            result['candidate_history'] = dict(limit=MAX_RECEIVER_CANDIDATES,
                retained=len(self._submitted_commands), capacity_exceeded=self._candidate_capacity_exceeded,
                scope='Bounded sent nonterminal-command candidate prefix only; omitted candidates grant no binding; '
                      'NOT accepted commands, submission/pose/transport/whole-process memory bounds or source authentication')
            result['status_history'] = dict(limit=MAX_RECEIVER_EVENTS, retained=len(self.events),
                capacity_exceeded=bool(self._history_not_retained),
                not_retained=self._history_not_retained,
                not_retained_count_is_lower_bound=self._history_not_retained == MAX_HISTORY_NOT_RETAINED,
                scope='Bounded valid-status prefix only; later inputs are NOT parsed/validated/freshness evidence; '
                      'NOT submission/pose/transport/whole-process memory bounds or source authentication')
        return result
