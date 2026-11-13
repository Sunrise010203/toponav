"""Sole Gazebo velocity publisher, isolated from navigation/GIL/step helpers.

Same-host monotonic deadlines, fixed .8s lease, no heartbeat renewal or re-arm.
This is software containment while this worker/transport/OS are healthy, NOT
firmware braking, hard real-time scheduling or physical-stop confirmation.
"""
from __future__ import annotations

import argparse
from collections import deque
from contextlib import closing
import json
import os
from pathlib import Path
import select
import socket
import stat
import subprocess
import sys
import time

from ros2_ws.src.toponav_bridge.toponav_bridge.protocol import (
    MotorGate, decode, encode, identity, number, validate_command,
)

LEASE_S = .8
POLL_S = .02
MAX_PACKET = 16384
AUDIT_TIMEOUT_S = .4
MAX_AUDIT_PENDING = 64
# Per-session replay evidence, never evicted to admit another command.
# Covers current tasks (2200 ticks) and the existing 8193-command zero-IPC
# audit workload; distinct from the native receiver's 4096-frame budget.
MAX_ACTUATOR_FRAMES = 16384


def audit_writer(channel_fd, output_fd):
    """Storage may block here, never in the sole velocity publisher."""
    with socket.socket(fileno=channel_fd) as channel, os.fdopen(output_fd, 'w') as stream:
        channel.send(b'ready')
        while True:
            raw = channel.recv(MAX_PACKET+1)
            if not raw:
                return
            if len(raw) > MAX_PACKET:
                raise ValueError('Oversized audit record')
            item = decode(raw.decode('utf-8'))
            stream.write(encode(item['record'])+'\n')
            stream.flush()
            # Acknowledges flush to the OS, NOT fsync/durable storage.
            channel.send(str(item['id']).encode())


class AuditPipe:
    """Bounded nonblocking mailbox to a separate, non-publishing process.

    No thread/fork of Gazebo's native threads and no synchronous disk I/O in
    the lease loop. Missing flush acknowledgements latch failure, not renewal.
    """
    def __init__(self, stream):
        self.channel, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.pending = deque()
        self.sequence = 0
        self.failed = False
        self.failure = None  # First cause only; never an authority/health override.
        self.process = None
        try:
            self.process = subprocess.Popen([sys.executable, '-u', '-m', 'simulation.gazebo_actuator',
                '--audit-writer', str(child.fileno()), str(stream.fileno())],
                pass_fds=(child.fileno(), stream.fileno()))
            child.close()
            # Startup only, before any motion authority exists.
            if not select.select([self.channel], [], [], 5.)[0] or self.channel.recv(32) != b'ready':
                raise RuntimeError('Audit writer startup failed')
            self.channel.setblocking(False)
        except BaseException:
            child.close()
            self.channel.close()
            if self.process is not None:
                self.process.kill()
                self.process.wait(timeout=5)
            raise

    def _fail(self, reason, error_number=None):
        # Fixed schema, no raw ACK/record/exception strings or growing list.
        # Do not perform diagnostic I/O in the lease/publish loop.
        if getattr(self, 'failure', None) is None:
            self.failure = dict(version=1, reason=reason, pending=len(self.pending),
                errno=error_number if type(error_number) is int and 0 <= error_number <= 2**31-1 else None)
        self.failed = True
        return False

    def poll(self):
        if self.failed:
            return False
        try:
            if self.process.poll() is not None:
                return self._fail('writer_exited')
            if self.pending and time.monotonic()-self.pending[0][1] >= AUDIT_TIMEOUT_S:
                return self._fail('flush_ack_timeout')
            for _ in range(MAX_AUDIT_PENDING):
                try:
                    raw = self.channel.recv(64)
                except BlockingIOError:
                    break
                if not raw:
                    return self._fail('ack_channel_closed')
                if not self.pending or raw != str(self.pending[0][0]).encode():
                    return self._fail('invalid_ack')
                self.pending.popleft()
        except OSError as exc:
            return self._fail('ack_receive_error', exc.errno)
        return not self.failed

    def submit(self, record):
        if not self.poll():
            return False
        try:
            if len(self.pending) >= MAX_AUDIT_PENDING:
                return self._fail('mailbox_full')
            seq = self.sequence+1
            raw = encode(dict(id=seq, record=record)).encode()
            if len(raw) > MAX_PACKET:
                return self._fail('record_too_large')
            if self.channel.send(raw) != len(raw):
                return self._fail('partial_record_send')
            self.sequence = seq
            self.pending.append((seq, time.monotonic()))
        except BlockingIOError as exc:
            return self._fail('record_send_would_block', exc.errno)
        except OSError as exc:
            return self._fail('record_send_error', exc.errno)
        return not self.failed

    def close(self):
        # Caller has already latched/published zero. Never wait for blocked
        # storage indefinitely, including when the writer itself is SIGSTOP'd.
        end = time.monotonic()+AUDIT_TIMEOUT_S
        while self.pending and self.poll() and time.monotonic() < end:
            select.select([self.channel], [], [], min(POLL_S, max(0., end-time.monotonic())))
        self.channel.close()
        try:
            self.process.wait(timeout=AUDIT_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)


_UNBOUND_STATUS = object()
_STATUS_FIELDS = {'latched', 'reason', 'wheels', 'deadline_steady_s', 'command_seq'}
MAX_AUDIT_FAILURE_BYTES = 1024
MAX_AUDIT_BYTES = 16*1024*1024
MAX_AUDIT_RECORD_BYTES = MAX_PACKET
MAX_AUDIT_EVENTS = 65536


def _audit_lines(path, history):
    """Bounded post-exit LF records; no special-file reads or symlink following.

    Limit checks happen before UTF-8/JSON parsing. At most one byte beyond the
    total budget is read; record buffering and retained events are also capped.
    Regular-file OS I/O can still stall. This is not an active-loop operation.
    """
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            raise ValueError('Non-regular actuator audit file')
        buffer = bytearray()
        records = 0

        def read(size):
            raw = os.read(fd, min(size, MAX_AUDIT_BYTES+1-history['bytes_read']))
            history['bytes_read'] += len(raw)
            if history['bytes_read'] > MAX_AUDIT_BYTES:
                history['byte_limit_exceeded'] = True
                raise ValueError('Actuator audit byte limit exceeded')
            return raw

        while True:
            if records >= MAX_AUDIT_EVENTS:
                if buffer or read(1):
                    history['event_limit_exceeded'] = True
                    raise ValueError('Actuator audit event limit exceeded')
                return
            end = buffer.find(b'\n')
            if end >= 0:
                if end+1 > MAX_AUDIT_RECORD_BYTES:
                    history['record_limit_exceeded'] = True
                    raise ValueError('Actuator audit record limit exceeded')
                raw = bytes(buffer[:end+1])
                del buffer[:end+1]
                # Preserve the previous splitlines semantics, but only inside
                # one bounded LF block (including legacy Unicode separators).
                for line in raw.decode('utf-8').splitlines():
                    if records >= MAX_AUDIT_EVENTS:
                        history['event_limit_exceeded'] = True
                        raise ValueError('Actuator audit event limit exceeded')
                    records += 1
                    yield line
                continue
            if len(buffer) > MAX_AUDIT_RECORD_BYTES:
                history['record_limit_exceeded'] = True
                raise ValueError('Actuator audit record limit exceeded')
            raw = read(min(8192, MAX_AUDIT_RECORD_BYTES+1-len(buffer)))
            if not raw:
                if buffer or records == 0:
                    raise ValueError('Incomplete actuator audit line')
                return
            buffer.extend(raw)
    finally:
        os.close(fd)


def _audit_failure_diagnostic(path):
    """Post-exit, bounded diagnostic read; can ONLY veto, never grant health.

    Do not follow symlinks or wait for FIFO/device data. Regular-file storage
    can still stall in the OS: this is NOT called by the active lease loop.
    Absence is not proof that every failure was recorded.
    """
    result = dict(status='absent', failure=None, blocks_completion=False)
    sidecar = Path(path).with_name(Path(path).name+'.failure.json')
    try:
        fd = os.open(sidecar, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
    except FileNotFoundError:
        return result
    except OSError:
        return dict(status='unreadable', failure=None, blocks_completion=True)
    try:
        try:
            if not stat.S_ISREG(os.fstat(fd).st_mode):
                raise ValueError('Non-regular audit failure diagnostic')
            raw = os.read(fd, MAX_AUDIT_FAILURE_BYTES+1)
            if len(raw) > MAX_AUDIT_FAILURE_BYTES:
                raise ValueError('Oversized audit failure diagnostic')
            failure = decode(raw.decode('utf-8'))
            if (set(failure) != {'version', 'reason', 'pending', 'errno'}
                    or type(failure['version']) is not int or failure['version'] != 1
                    or not isinstance(failure['reason'], str) or failure['reason'] not in {
                        'writer_exited', 'flush_ack_timeout', 'ack_channel_closed', 'invalid_ack',
                        'ack_receive_error', 'mailbox_full', 'record_too_large', 'partial_record_send',
                        'record_send_would_block', 'record_send_error'}
                    or type(failure['pending']) is not int or not 0 <= failure['pending'] <= MAX_AUDIT_PENDING
                    or (failure['errno'] is not None and (type(failure['errno']) is not int
                        or not 0 <= failure['errno'] <= 2**31-1))):
                raise ValueError('Invalid audit failure diagnostic schema')
            return dict(status='present', failure=failure, blocks_completion=True)
        except OSError:
            return dict(status='unreadable', failure=None, blocks_completion=True)
        except (ValueError, TypeError, UnicodeError, KeyError, RecursionError):
            return dict(status='invalid', failure=None, blocks_completion=True)
    finally:
        os.close(fd)


def _validate_audit_status(status):
    if (not isinstance(status, dict) or set(status) != _STATUS_FIELDS
            or type(status['latched']) is not bool or not isinstance(status['reason'], str)
            or type(status['command_seq']) is not int or status['command_seq'] < 0
            or not isinstance(status['wheels'], list) or len(status['wheels']) != 2
            or not all(number(v) for v in status['wheels'])
            or not 0 <= status['wheels'][0] <= .15 or abs(status['wheels'][1]) > .30):
        raise ValueError('Invalid actuator audit status')
    seq, deadline = status['command_seq'], status['deadline_steady_s']
    if ((seq == 0 and (deadline is not None or status['wheels'] != [0., 0.]))
            or (seq > 0 and (not number(deadline) or deadline <= 0))
            or (status['latched'] and (status['wheels'] != [0., 0.]
                                      or status['reason'] in ('accepted', 'disarmed')))
            or (not status['latched'] and status['reason'] != ('accepted' if seq else 'disarmed'))):
        raise ValueError('Invalid actuator audit authority state')


def _validate_audit_history(events):
    previous = None
    for index, item in enumerate(events):
        if (set(item) != _STATUS_FIELDS | {'event', 'wall_s', 'publish_success'}
                or not isinstance(item['event'], str) or item['wall_s'] < 0
                or (item['event'] == 'ready' and index != 0)
                or (item['event'] == 'worker_exit' and index != len(events)-1)):
            raise ValueError('Invalid actuator audit history schema')
        status = {key: item[key] for key in _STATUS_FIELDS}
        _validate_audit_status(status)
        seq, deadline = item['command_seq'], item['deadline_steady_s']
        # wall_s is sampled AFTER native publish returns, not at gate.poll.
        # A blocked publication may return after expiry; do not call that a
        # malformed log or grant a fresh lease from its completion timestamp.
        if item['reason'] == 'accepted' and deadline > item['wall_s']+LEASE_S:
            raise ValueError('Invalid actuator audit lease bound')
        if item['event'] in ('stop', 'shutdown', 'worker_exit') and not item['latched']:
            raise ValueError('Unlatched actuator audit stop')
        if previous is None:
            if seq != 0 or deadline is not None:
                raise ValueError('Actuator audit missing initial authority state')
        else:
            old_seq = previous['command_seq']
            if item['wall_s'] < previous['wall_s'] or seq not in (old_seq, old_seq+1):
                raise ValueError('Actuator audit clock/command sequence gap')
            if seq != old_seq and (item['event'] != 'command' or previous['latched']):
                raise ValueError('Actuator audit command missing or rearmed')
            if seq == old_seq:
                if deadline != previous['deadline_steady_s']:
                    raise ValueError('Actuator audit deadline rewritten')
                if not item['latched'] and item['wheels'] != previous['wheels']:
                    raise ValueError('Actuator audit target rewritten')
                if item['event'] == 'command' and not item['latched']:
                    raise ValueError('Actuator audit command did not advance sequence')
            if previous['latched'] and status != {key: previous[key] for key in _STATUS_FIELDS}:
                raise ValueError('Actuator audit latch rewritten')
        previous = item


def summarize_audit(path, worker_pid, *, expected_status=_UNBOUND_STATUS):
    """Validate recorded authority history, optionally bound to the last IPC ACK.

    Legacy six-field synthetic logs retain their two-argument compatibility,
    explicitly labelled as unbound. Production always requests full validation
    and terminal binding; None is a failed binding, never a legacy fallback.
    This is not an authenticated/durable log or physical-stop certification.
    """
    events = []
    error = None
    binding_requested = expected_status is not _UNBOUND_STATUS
    bound = False
    strict = binding_requested
    history = dict(max_bytes=MAX_AUDIT_BYTES, max_record_bytes=MAX_AUDIT_RECORD_BYTES,
        max_events=MAX_AUDIT_EVENTS, read_attempted=False, bytes_read=0, byte_limit_exceeded=False,
        record_limit_exceeded=False, event_limit_exceeded=False)
    diagnostic = _audit_failure_diagnostic(path)
    try:
        # An invalid/unreadable veto cannot support any history claim. Do not
        # open another untrusted input after this definitive rejection.
        if diagnostic['status'] in ('invalid', 'unreadable'):
            raise ValueError('Actuator audit not read: failure sidecar invalid or unreadable')
        history['read_attempted'] = True
        with closing(_audit_lines(path, history)) as lines:
            for line in lines:
                item = decode(line)
                if (not isinstance(item, dict) or type(item.get('latched')) is not bool
                        or type(item.get('publish_success')) is not bool
                        or not isinstance(item.get('reason'), str)
                        or not number(item.get('wall_s'))
                        or not isinstance(item.get('wheels'), list) or len(item['wheels']) != 2
                        or not all(number(v) for v in item['wheels'])):
                    raise ValueError('Invalid actuator audit record')
                events.append(item)
                strict = strict or 'command_seq' in item or 'deadline_steady_s' in item
        if strict:
            _validate_audit_history(events)
        if binding_requested:
            _validate_audit_status(expected_status)
            if not events or expected_status != {key: events[-1][key] for key in _STATUS_FIELDS}:
                raise ValueError('Actuator audit terminal status differs from IPC acknowledgement')
            bound = True
    except (OSError, ValueError, TypeError, UnicodeError, KeyError, RecursionError) as exc:
        error = f'{type(exc).__name__}: {exc}'
    if diagnostic['blocks_completion'] and error is None:
        error = 'RuntimeError: Actuator audit failure sidecar present, invalid or unreadable'
    complete = (error is None and len(events) >= 2 and events[0].get('event') == 'ready'
                and events[-1].get('event') == 'worker_exit'
                and events[-1]['wheels'] == [0., 0.] and events[-1]['latched'])
    safe = {'disarmed', 'accepted', 'remote_stop', 'remote_visual_stop', 'remote_abort'}
    return dict(worker_pid=worker_pid, lease_s=LEASE_S,
        events_path='audit/actuator.jsonl', events=len(events), audit_complete=complete, read_error=error,
        validation_scope='full_authority_history' if strict else 'legacy_minimal_records',
        terminal_status_binding_requested=binding_requested, terminal_status_bound=bound,
        audit_history=dict(**history, retained=len(events),
            scope='Bounded LF audit prefix only; bytes_read may include unparsed read-ahead; retained is NOT total records. '
                  'Limits veto completion, never grant authority/physical stop. NOT writer/disk/transport/whole-process bounds or hard realtime'),
        failure_diagnostic=diagnostic,
        fault_latched=not complete or any(e['reason'] not in safe or not e['publish_success'] for e in events),
        final=events[-1] if events else None,
        scope='Independent process zero publication; not physical-stop or hardware watchdog certification')


class LeaseGuard:
    def __init__(self, session):
        self.gate = MotorGate(session, lease_s=LEASE_S)
        self.request_id = 0
        self.captured = -1.
        self.frames = set()
        self.frame_id = ''
        self.issued = -1.

    def publication(self):
        g = self.gate
        return dict(version='1', session_id=g.session_id,
            kind='stop' if g.latched else 'command' if g.seq else 'idle',
            command_seq=str(g.seq), frame_id=self.frame_id,
            captured_sim_s=repr(self.captured), issued_steady_s=repr(self.issued),
            expires_steady_s=repr(g.hold_deadline if g.hold_deadline is not None else -1.))

    def status(self):
        g = self.gate
        return dict(latched=g.latched, reason=g.reason, wheels=list(g.wheels),
                    deadline_steady_s=g.hold_deadline, command_seq=g.seq)

    def poll(self, now):
        return self.gate.poll(now)

    def handle(self, packet, now):
        g = self.gate
        self.poll(now)
        base = {'kind', 'session_id', 'request_id'}
        try:
            if not isinstance(packet, dict) or not base <= packet.keys():
                raise ValueError('Invalid packet')
            if packet['session_id'] != g.session_id or type(packet['request_id']) is not int or packet['request_id'] != self.request_id+1:
                raise ValueError('Invalid session/request sequence')
            self.request_id = packet['request_id']
            kind = packet['kind']
            if kind in ('stop', 'shutdown') and set(packet) == base:
                g.trip('remote_stop')
                return True
            if kind == 'idle' and set(packet) == base:
                if g.latched or g.seq or g.hold_deadline is not None:
                    raise ValueError('Idle cannot replace a bound command')
                return True  # No lease or motion authority is created.
            if kind != 'command' or set(packet) != base | {'command', 'sim_s', 'issued_steady_s', 'expires_steady_s'}:
                raise ValueError('Invalid command packet fields')
            if g.latched:
                return False
            c = validate_command(packet['command'])
            issued, expires = packet['issued_steady_s'], packet['expires_steady_s']
            if not all(number(v) for v in (issued, expires, packet['sim_s'])) or not 0 <= issued <= now < expires:
                raise ValueError('Expired/future/invalid steady deadline')
            # A float subtraction can round by a fraction of a nanosecond;
            # clamp to the fixed bound rather than granting an age grace.
            if expires > issued+LEASE_S+1e-9:
                raise ValueError('Lease extension refused')
            if c['captured_sim_s'] <= self.captured or c['frame_id'] in self.frames:
                raise ValueError('Reused frame cannot refresh a lease')
            # Before arming, accepting or inserting a new frame: preserve the
            # last authority/deadline/metadata, and latch zero on exhaustion.
            # This also rejects terminal command evidence, NOT safe stopping:
            # frame-free STOP/shutdown above remain available even when full.
            if len(self.frames) >= MAX_ACTUATOR_FRAMES:
                g.trip('actuator_capacity_exceeded')
                return False
            g.arm(c['frame_id'], c['captured_sim_s'], now)
            g.accept(c, packet['sim_s'], now)
            if g.reason not in ('accepted', 'remote_visual_stop', 'remote_abort'):
                return False
            g.hold_deadline = min(expires, issued+LEASE_S)
            self.captured = c['captured_sim_s']
            self.frame_id = c['frame_id']
            self.issued = issued
            self.frames.add(c['frame_id'])
            self.poll(now)
            return True
        except (ValueError, TypeError, KeyError, OverflowError):
            g.trip('invalid_actuator_packet')
            return False


def serve(channel, session, publish, audit, connected=lambda: False, publication_context=None):
    """Publish callback is native in production; injected only by pipe tests."""
    channel.setblocking(False)
    guard = LeaseGuard(session)
    peer = True
    lost_at = None
    audit = AuditPipe(audit)

    def emit(v, omega):
        if publication_context is not None:
            publication_context(guard.publication())
        publish(v, omega)

    def output(event=None):
        if not audit.poll():
            guard.gate.trip('audit_failure')
        guard.poll(time.monotonic())
        try:
            emit(*guard.gate.wheels)
            success = True
        except Exception:
            guard.gate.trip('publisher_failure')
            success = False
            try: emit(0., 0.)
            except Exception: pass
        record = dict(event=event, wall_s=time.monotonic(), publish_success=success, **guard.status())
        if event and not audit.submit(record):
            guard.gate.trip('audit_failure')
            try: emit(0., 0.)
            except Exception: pass
        return record

    def reply(message):
        nonlocal peer
        try: channel.send(encode(message).encode())
        except OSError:
            peer = False
            guard.gate.trip('peer_closed')
            output('peer_closed')

    try:
        record = output('ready')
        reply(dict(kind='ready', pid=os.getpid(), audit_pid=audit.process.pid, status=guard.status()))
        while True:
            was_latched = guard.gate.latched
            if not audit.poll():
                guard.gate.trip('audit_failure')
            guard.poll(time.monotonic())
            if guard.gate.latched and not was_latched:
                output(guard.gate.reason)
            # Repeat zero after latch. There is no periodic nonzero refresh.
            if guard.gate.latched:
                output()
            now = time.monotonic()
            deadline = guard.gate.hold_deadline
            wait = min(POLL_S, max(0., deadline-now)) if deadline is not None and not guard.gate.latched else POLL_S
            if not peer:
                # Keep an orphan fail-closed while the native actuator remains
                # connected; stop only after its world has disappeared.
                if connected(): lost_at = None
                elif lost_at is None: lost_at = now
                elif now-lost_at >= 2.: break
                time.sleep(POLL_S)
                continue
            ready, _, _ = select.select([channel], [], [], wait)
            if not ready: continue
            raw = channel.recv(MAX_PACKET+1)
            if not raw:
                peer = False
                guard.gate.trip('peer_closed')
                output('peer_closed')
                continue
            try:
                if len(raw) > MAX_PACKET: raise ValueError('Oversized packet')
                packet = decode(raw.decode('utf-8'))
            except (ValueError, UnicodeError):
                guard.gate.trip('invalid_actuator_packet')
                output('invalid_actuator_packet')
                continue
            accepted = guard.handle(packet, time.monotonic())
            record = output(packet.get('kind'))
            accepted = accepted and record['publish_success'] and not audit.failed
            reply(dict(kind='ack', request_id=packet.get('request_id'), accepted=accepted,
                       status=guard.status(), published_steady_s=record['wall_s']))
            if accepted and packet.get('kind') == 'shutdown': break
    finally:
        guard.gate.trip('worker_exit')
        output('worker_exit')
        channel.close()
        audit.close()
    return audit.failure


class ActuatorClient:
    def __init__(self, session, audit_path, *, submission_observer=None):
        if not identity(session): raise ValueError('Invalid actuator session')
        self.session = session
        self.request_id = 0
        self.failed = False
        self.status = None
        self.last_publication_s = None
        self.submission_observer = submission_observer
        self.channel, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.channel.setblocking(False)
        self.process = None
        try:
            self.process = subprocess.Popen([sys.executable, '-u', '-m', 'simulation.gazebo_actuator',
                '--fd', str(child.fileno()), '--session', session, '--audit', str(audit_path)],
                pass_fds=(child.fileno(),))
            child.close()
            message = self._receive(30.)
            if (not isinstance(message, dict) or set(message) != {'kind', 'pid', 'audit_pid', 'status'}
                    or message['kind'] != 'ready' or type(message['pid']) is not int
                    or message['pid'] != self.process.pid or type(message['audit_pid']) is not int
                    or message['audit_pid'] <= 0 or message['audit_pid'] == self.process.pid):
                raise RuntimeError('Invalid actuator startup handshake')
            _validate_audit_status(message['status'])
            if message['status'] != dict(latched=False, reason='disarmed', wheels=[0., 0.],
                                         deadline_steady_s=None, command_seq=0):
                raise RuntimeError('Invalid actuator startup authority')
            self.status = message['status']
            self.audit_pid = message['audit_pid']
        except BaseException:
            child.close()
            self.channel.close()
            if self.process is not None:
                self.process.kill()
                self.process.wait(timeout=5)
            raise

    def _receive(self, timeout):
        ready, _, _ = select.select([self.channel], [], [], timeout)
        if not ready: raise TimeoutError('Independent actuator acknowledgement timed out')
        raw = self.channel.recv(MAX_PACKET+1)
        if not raw or len(raw) > MAX_PACKET: raise RuntimeError('Independent actuator channel closed/invalid')
        return decode(raw.decode('utf-8'))

    def _validate_ack(self, reply, packet, sent, received):
        """Bind the IPC ACK to this request, never infer physical execution.

        All steady stamps are same-host monotonic. Receiving an old ACK does
        not create a new lease, and a later good ACK cannot clear a bad one.
        """
        if (not isinstance(reply, dict)
                or set(reply) != {'kind', 'request_id', 'accepted', 'status', 'published_steady_s'}
                or reply['kind'] != 'ack' or type(reply['request_id']) is not int
                or reply['request_id'] != packet['request_id'] or type(reply['accepted']) is not bool
                or not number(reply['published_steady_s'])
                or not sent <= reply['published_steady_s'] <= received
                or (self.last_publication_s is not None
                    and reply['published_steady_s'] < self.last_publication_s)):
            raise RuntimeError('Invalid or unbound actuator acknowledgement')
        status = reply['status']
        _validate_audit_status(status)
        if not reply['accepted']:
            return  # Preserve a well-formed rejection status, then fail below.
        kind = packet['kind']
        base = {'kind', 'session_id', 'request_id'}
        previous = self.status
        if kind == 'command':
            if set(packet) != base | {'command', 'sim_s', 'issued_steady_s', 'expires_steady_s'}:
                raise RuntimeError('Actuator acknowledged invalid command fields')
            command = validate_command(packet['command'])
            issued, expires = packet['issued_steady_s'], packet['expires_steady_s']
            if (previous['latched'] or command['session_id'] != self.session
                    or command['seq'] != previous['command_seq']+1
                    or not all(number(v) for v in (issued, expires, packet['sim_s']))
                    or not 0 <= issued <= reply['published_steady_s']
                    or not issued < expires <= issued+LEASE_S+1e-9
                    or not 0 <= command['captured_sim_s'] <= command['decision_sim_s'] <= packet['sim_s']+1e-8):
                raise RuntimeError('Actuator acknowledged invalid command authority')
            reason = {'visual_stop': 'remote_visual_stop', 'aborted': 'remote_abort'}.get(command['state'], 'accepted')
            latched = reason != 'accepted'
            expected = dict(latched=latched, reason=reason, command_seq=command['seq'],
                deadline_steady_s=min(expires, issued+LEASE_S),
                wheels=[0., 0.] if latched else [command['v_m_s'], command['omega_rad_s']])
            if status != expected:
                raise RuntimeError('Actuator acknowledgement differs from submitted command')
            if not latched and received >= expected['deadline_steady_s']:
                raise RuntimeError('Actuator acknowledgement arrived after command deadline')
        elif kind in ('idle', 'stop', 'shutdown') and set(packet) == base:
            if kind == 'idle':
                if status != dict(latched=False, reason='disarmed', command_seq=0,
                                  deadline_steady_s=None, wheels=[0., 0.]):
                    raise RuntimeError('Actuator idle acknowledgement granted authority')
                if previous != status:
                    raise RuntimeError('Actuator idle acknowledgement replaced authority')
            elif (not status['latched'] or status['command_seq'] != previous['command_seq']
                    or status['deadline_steady_s'] != previous['deadline_steady_s']
                    or (previous['latched'] and status != previous)):
                raise RuntimeError('Actuator stop acknowledgement changed authority')
        else:
            raise RuntimeError('Actuator acknowledged invalid request kind/fields')

    def request(self, kind, **values):
        if self.failed: raise RuntimeError('Independent actuator unavailable after prior failure')
        self.request_id += 1
        packet = dict(kind=kind, session_id=self.session, request_id=self.request_id, **values)
        try:
            raw = encode(packet).encode()
            if len(raw) > MAX_PACKET: raise ValueError('Oversized actuator packet')
            sent = time.monotonic()
            self.channel.send(raw)  # Atomic/nonblocking; never queue heartbeats.
            observer = getattr(self, 'submission_observer', None)
            if observer is not None:
                observer(decode(raw.decode('utf-8')), sent)
            reply = self._receive(LEASE_S)
            self._validate_ack(reply, packet, sent, time.monotonic())
            self.status = reply['status']
            self.last_publication_s = reply['published_steady_s']
            if reply.get('accepted') is not True:
                raise RuntimeError('Independent actuator rejected command: '+str(self.status))
            return reply
        except BaseException:
            self.failed = True
            self.channel.close()  # EOF independently latches zero in the worker.
            raise

    def close(self):
        try:
            if not self.failed and self.process.poll() is None:
                self.request('shutdown')
        finally:
            self.channel.close()
            try: self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                # Called after the owned Gazebo world is stopped. Never kill
                # the safety publisher while its world may still be moving.
                self.process.kill()
                self.process.wait(timeout=5)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] == '--audit-writer':
        parser = argparse.ArgumentParser(description='Isolated actuator audit storage')
        parser.add_argument('--audit-writer', type=int, required=True)
        parser.add_argument('output_fd', type=int)
        args = parser.parse_args(argv)
        audit_writer(args.audit_writer, args.output_fd)
        return 0
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fd', type=int, required=True)
    parser.add_argument('--session', required=True)
    parser.add_argument('--audit', type=Path, required=True)
    args = parser.parse_args(argv)
    from gz.transport13 import Node
    from gz.msgs10.twist_pb2 import Twist
    node = Node()
    publisher = node.advertise('/toponav/cmd_vel', Twist)
    metadata = {}
    def context(values):
        metadata.clear()
        metadata.update(values)
    def publish(v, omega):
        message = Twist()
        message.linear.x, message.angular.z = v, omega
        for key, value in metadata.items():
            item = message.header.data.add()
            item.key = key
            item.value.append(value)
        if publisher.publish(message) is False:
            raise RuntimeError('Native velocity publication failed')
    with args.audit.open('x') as audit:
        failure = serve(socket.socket(fileno=args.fd), args.session, publish, audit, publisher.has_connections, context)
    # AFTER zero/latch, command channel close and bounded writer reap. Never
    # wait on diagnostic storage in an active lease/publish loop. Missing or
    # unwritable sidecars remain unknown, not successful/complete audits.
    if failure is not None:
        with args.audit.with_name(args.audit.name+'.failure.json').open('x') as stream:
            stream.write(json.dumps(failure, indent=2, allow_nan=False)+'\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
