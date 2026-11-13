"""Opt-in zero-only timing/fault audit. No hardware, ROS or Gazebo transport.

Production client/serve/audit writer and compiled ReceiverLease are exercised;
the publisher and native receive loop are explicit isolated test doubles.
Injected sleeps are NOT actual controller computation or camera capture time.
"""
from contextlib import contextmanager
import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import time
from unittest.mock import patch

from simulation import gazebo_actuator as act
from ros2_ws.src.toponav_bridge.toponav_bridge.protocol import command_from_decision
from tools.actuator_ipc_diagnostics import ClientProbe

ROOT = Path(__file__).resolve().parents[1]
CASES = ('zero_quick', 'zero_slow_first', 'zero_blocked_producer', 'zero_stopped_publisher')
SESSION = 'timing-audit'
FAULT_DELAY_S = 1.1
ALLOWANCE_S = .15  # Observation criterion only; never supplied to a guard.


def write_json(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')


def read_rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines()]


def zero_packet(seq, issued):
    command = command_from_decision(SESSION, seq, dict(state='observing', v_m_s=0.,
        omega_rad_s=0., control_frame_id=f'diagnostic-{seq}',
        captured_sim_s=seq*.1, decision_sim_s=seq*.1))
    return dict(command=command, sim_s=seq*.1,
                issued_steady_s=issued, expires_steady_s=issued+act.LEASE_S)


def wire_metadata(meta, v, w):
    # Refuse even isolated nonzero; this tool never has movement authority.
    if v != 0 or w != 0:
        raise ValueError('Timing audit is zero-only')
    return ' '.join(str(x) for x in (meta['session_id'], meta['kind'], meta['frame_id'] or '_',
        meta['command_seq'], meta['captured_sim_s'], meta['issued_steady_s'],
        meta['expires_steady_s'], v, w)).encode('ascii')


class ReceiveProbe:
    """Timestamp socket.recv RETURN, not kernel arrival, decode or acceptance."""
    def __init__(self, channel, stream):
        self.channel, self.stream = channel, stream
        self.ordinal = 0
    def recv(self, *args):
        raw = self.channel.recv(*args)
        stamp = time.monotonic()
        if raw:
            self.ordinal += 1
            self.stream.write(json.dumps(dict(event='recv_return', wall_s=stamp, ordinal=self.ordinal))+'\n')
            self.stream.flush()
        return raw
    def __getattr__(self, name):
        return getattr(self.channel, name)


def isolated_worker(args):
    # No gz/ROS imports or external publisher. Only a private socket is used.
    metadata = {}
    with socket.socket(fileno=args.receiver_fd) as receiver, args.audit.open('x') as audit, \
            args.publications.open('x', buffering=1) as pubs, \
            args.receives.open('x', buffering=1) as receives:
        receiver.setblocking(False)
        def context(values):
            metadata.clear(); metadata.update(values)
        def publish(v, w):
            raw = wire_metadata(metadata, v, w)
            started = time.monotonic()
            if receiver.send(raw) != len(raw):
                raise RuntimeError('Partial private receiver send')
            pubs.write(json.dumps(dict(started_steady_s=started, finished_steady_s=time.monotonic(),
                v=v, w=w, metadata=metadata.copy()))+'\n')
        channel = ReceiveProbe(socket.socket(fileno=args.fd), receives)
        failure = act.serve(channel, SESSION, publish, audit, publication_context=context)
    if failure is not None:
        write_json(args.audit.with_name(args.audit.name+'.failure.json'), failure)


@contextmanager
def isolated_client(folder, receiver):
    """Route ONLY client startup to our private worker; restore Popen immediately.

    No production source/default is changed. AuditPipe in the child starts the
    ORIGINAL production audit writer, which does not inherit this scoped patch.
    """
    original = subprocess.Popen
    def spawn(argv, **kwargs):
        if argv[:4] != [sys.executable, '-u', '-m', 'simulation.gazebo_actuator']:
            raise RuntimeError('Unexpected isolated startup')
        replacement = [sys.executable, '-u', '-m', 'tools.straight_deadline_audit', '--isolated-worker',
            *argv[4:], '--receiver-fd', str(receiver.fileno()),
            '--publications', str(folder/'publications.jsonl'), '--receives', str(folder/'receives.jsonl')]
        kwargs['pass_fds'] = (*kwargs['pass_fds'], receiver.fileno())
        return original(replacement, **kwargs)
    with (folder/'worker.log').open('x') as worker_log:
        def logged_spawn(argv, **kwargs):
            return spawn(argv, stdout=worker_log, stderr=subprocess.STDOUT, **kwargs)
        with patch.object(act.subprocess, 'Popen', logged_spawn):
            client = act.ActuatorClient(SESSION, folder/'actuator.jsonl')
        try:
            yield client
        finally:
            # Only our isolated child is resumed; never signal external processes.
            if client.process.poll() is None:
                os.kill(client.process.pid, signal.SIGCONT)
            client.close()


def compile_receiver(folder):
    binary = folder/'isolated-receiver'
    command = ['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-I', str(ROOT),
               str(ROOT/'tools/isolated_receiver_timing.cc'), '-o', str(binary)]
    with (folder/'compile.log').open('x') as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=False)
    write_json(folder/'compile.json', dict(command=command, returncode=result.returncode,
        binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest() if binary.exists() else None))
    if result.returncode:
        raise RuntimeError('Isolated receiver compilation failed')
    return binary


def evaluate(case, result, audit, native):
    """Separate expected fault observations from safety/acceptance conclusions."""
    deadline = result.get('deadline_steady_s')
    checks = {}
    for name, rows in (('worker', audit), ('receiver', native)):
        hit = next((r for r in rows if r['reason'] == 'command_lease_expired'), None)
        result[name+'_expiry'] = hit
        lag = hit['wall_s']-deadline if hit is not None and deadline is not None else None
        result[name+'_expiry_lag_s'] = lag
        checks[name+'_expiry_observed'] = (lag is not None and 0 <= lag < ALLOWANCE_S
                                          and hit['wheels'] == [0., 0.])
    if case == 'zero_stopped_publisher':
        # Worker is deliberately frozen across its deadline. Never count its
        # late publication as timely containment; only native receiver can poll.
        checks.pop('worker_expiry_observed')
        hit = result['receiver_expiry']
        checks['receiver_latched_while_publisher_stopped'] = (hit is not None
            and result.get('publisher_stop_confirmed_s', float('inf')) <= hit['wall_s']
            < result.get('resume_requested_s', -1.))
        status = result.get('post_resume_status') or {}
        checks['publisher_still_latched_on_resume'] = (status.get('latched') is True
            and status.get('wheels') == [0., 0.] and status.get('deadline_steady_s') == deadline)
    if case == 'zero_blocked_producer':
        for name in ('worker', 'receiver'):
            hit = result[name+'_expiry']
            checks[name+'_latched_during_injected_block'] = (hit is not None
                and result.get('block_started_s', float('inf')) <= hit['wall_s']
                < result.get('block_finished_s', -1.))
        checks['late_fresh_zero_refused'] = result.get('late_error') is not None and result.get('client_failed') is True
    if case == 'zero_slow_first':
        checks['slow_first_zero_accepted'] = result.get('first_ack', {}).get('accepted') is True
        checks['injected_preissue_delay_exceeds_lease'] = (result.get('compute_finished_s', 0.)
            - result.get('compute_started_s', 0.) > act.LEASE_S)
    if result.get('error') is not None:
        checks['no_unexpected_error'] = False
    result['checks'] = checks
    result['expected_observations_met'] = bool(checks) and all(checks.values())
    result['physical_stop_measured'] = False
    result['capture_to_motor_latency_s'] = None
    result['wall_capture_freshness_proved'] = False
    return result


def run_one(case, folder, binary):
    if case not in CASES:
        raise ValueError('Unknown case')
    folder.mkdir()
    result = dict(case=case, error=None, clock='same-host CLOCK_MONOTONIC / time.monotonic',
        actual_capture_wall_s=None, captured_sim_s=.1, diagnostic_frame_available_s=time.monotonic(),
        compute_kind='injected_sleep_NOT_controller_computation', lease_s=act.LEASE_S,
        observation_allowance_s=ALLOWANCE_S)
    probe = ClientProbe()
    parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    process = None
    client = None
    with (folder/'receiver.jsonl').open('x') as receiver_log:
        try:
            process = subprocess.Popen([str(binary), str(child.fileno())], pass_fds=(child.fileno(),),
                stdout=receiver_log, stderr=subprocess.STDOUT)
            child.close()
            if not select.select([parent], [], [], 5.)[0] or parent.recv(32) != b'ready':
                raise RuntimeError('Isolated receiver startup failed')
            with isolated_client(folder, parent) as client, probe.attach(client):
                result.update(worker_pid=client.process.pid, audit_pid=client.audit_pid, receiver_pid=process.pid)
                result['compute_started_s'] = time.monotonic()
                time.sleep(FAULT_DELAY_S if case == 'zero_slow_first' else .02)
                result['compute_finished_s'] = time.monotonic()
                values = zero_packet(1, time.monotonic())
                result['issued_steady_s'] = values['issued_steady_s']
                result['deadline_steady_s'] = values['expires_steady_s']
                result['first_ack'] = client.request('command', **values)
                result['first_ack_returned_s'] = time.monotonic()
                if case == 'zero_stopped_publisher':
                    result['stop_requested_s'] = time.monotonic()
                    os.kill(client.process.pid, signal.SIGSTOP)
                    pid, status = os.waitpid(client.process.pid, os.WUNTRACED)
                    if pid != client.process.pid or not os.WIFSTOPPED(status):
                        raise RuntimeError('Owned publisher did not stop')
                    result['publisher_stop_confirmed_s'] = time.monotonic()
                result['block_started_s'] = time.monotonic()
                time.sleep(FAULT_DELAY_S)  # No polling, sends or heartbeat in producer.
                result['block_finished_s'] = time.monotonic()
                if case == 'zero_stopped_publisher':
                    result['resume_requested_s'] = time.monotonic()
                    os.kill(client.process.pid, signal.SIGCONT)
                if case == 'zero_blocked_producer':
                    try:
                        result['late_ack'] = client.request('command', **zero_packet(2, time.monotonic()))
                    except RuntimeError as exc:
                        result['late_error'] = str(exc)
                    result['client_failed'] = client.failed
                elif case == 'zero_stopped_publisher':
                    # SIGSTOP also freezes ACK consumption for AuditPipe. Its
                    # unchanged .4s storage guard may fail on resume. Preserve
                    # that fault; receiver containment must not require a
                    # healthy/responsive publisher or a complete worker audit.
                    try:
                        result['stop_ack'] = client.request('stop')
                    except RuntimeError as exc:
                        result['post_resume_stop_error'] = str(exc)
                    result['post_resume_status'] = client.status
                else:
                    result['stop_ack'] = client.request('stop')
        except Exception as exc:
            result['error'] = f'{type(exc).__name__}: {exc}'
        finally:
            child.close()
            # Worker context has closed/reaped before terminating the receiver.
            if process is not None:
                try:
                    parent.send(b'quit')
                    process.wait(timeout=5.)
                except (OSError, subprocess.TimeoutExpired):
                    process.kill(); process.wait(timeout=5.)
                result['receiver_returncode'] = process.returncode
            parent.close()
    # All worker/audit/native output streams are closed before parsing/hashing.
    result['worker_returncode'] = client.process.returncode if client is not None else None
    result['all_children_reaped'] = (process is not None and process.returncode is not None
        and client is not None and client.process.returncode is not None
        and not Path(f'/proc/{client.audit_pid}').exists())
    write_json(folder/'client-probe.json', probe.snapshot())
    if client is not None:
        write_json(folder/'audit-summary.json', act.summarize_audit(folder/'actuator.jsonl',
            client.process.pid, expected_status=client.status))
    audit = read_rows(folder/'actuator.jsonl') if (folder/'actuator.jsonl').exists() else []
    native = read_rows(folder/'receiver.jsonl')
    result = evaluate(case, result, audit, native)
    if not result['all_children_reaped'] or result.get('receiver_returncode') != 0 or result['worker_returncode'] != 0:
        result['expected_observations_met'] = False
    write_json(folder/'result.json', result)
    return result


def run(output, repeats=3):
    if type(repeats) is not int or repeats < 1:
        raise ValueError('Positive integer repeats required')
    output = Path(output).resolve(); output.mkdir(exist_ok=False)
    binary = compile_receiver(output)
    results = []
    for repeat in range(repeats):
        for case in (CASES if repeat % 2 == 0 else CASES[::-1]):
            results.append(run_one(case, output/f'{repeat}-{case}', binary))
    summary = dict(results=results, expected_observations_met=all(r['expected_observations_met'] for r in results),
        all_children_reaped=all(r['all_children_reaped'] for r in results),
        production_changes=False, real_capture_or_controller_compute_measured=False,
        external_transport_or_movement=False, physical_stop_proved=False,
        scope='Diagnostic observations, not latency/safety/navigation acceptance; retain every repeat')
    write_json(output/'summary.json', summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--isolated-worker', action='store_true')
    parser.add_argument('--fd', type=int)
    parser.add_argument('--session')
    parser.add_argument('--audit', type=Path)
    parser.add_argument('--receiver-fd', type=int)
    parser.add_argument('--publications', type=Path)
    parser.add_argument('--receives', type=Path)
    args = parser.parse_args()
    if args.isolated_worker:
        if args.session != SESSION:
            parser.error('Isolated session mismatch')
        isolated_worker(args)
        return 0
    if args.output is None:
        parser.error('--output is required')
    result = run(args.output, args.repeats)
    print(json.dumps(dict(expected_observations_met=result['expected_observations_met'],
        all_children_reaped=result['all_children_reaped'], cases=len(result['results']))))
    return 0 if result['expected_observations_met'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
