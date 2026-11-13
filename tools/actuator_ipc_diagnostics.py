"""Opt-in, bounded IPC phase diagnostics; NEVER loaded by the runtime.

Wrap an already-started client, forwarding the original methods/socket calls.
No timeout, acknowledgement, receiver, GC threshold or authority changes.
Instrumentation adds overhead: measurements are diagnostic, not acceptance.
"""
from collections import deque
from contextlib import contextmanager
import gc
import hashlib
import json
from pathlib import Path
import threading
import time

import pytest


TARGET = 'tests/test_review_receiver_candidates.py::test_real_ipc_full_original_budget_zero_commands_and_acks_remain_healthy'
CAPACITY = 64
MAX_COUNTER = 2**64 - 1
MAX_SNAPSHOT_BYTES = 256 * 1024
STAMPS = ('started', 'send_started', 'sent', 'wait_started', 'wait_done',
          'recv_done', 'decoded', 'validation_started', 'validated', 'finished', 'closed')


def scalar(value):
    # No user conversions, booleans, strings, nonfinite or enormous numbers.
    return value if type(value) in (int, float) and 0 <= value <= 1e12 else None


def increment(value):
    return min(MAX_COUNTER, value + 1)


class _SocketProbe:
    def __init__(self, channel, owner):
        self.channel, self.owner = channel, owner

    def fileno(self):
        return self.channel.fileno()

    def send(self, *args, **kwargs):
        self.owner.mark('send_started', 'send')
        result = self.channel.send(*args, **kwargs)
        self.owner.mark('sent', 'submission')
        return result

    def recv(self, *args, **kwargs):
        # Original select returned before this call; includes its Python gap.
        self.owner.mark('wait_done', 'recv')
        result = self.channel.recv(*args, **kwargs)
        self.owner.mark('recv_done', 'decode_ack')
        return result

    def close(self):
        # Original failure path closes FIRST, before diagnostic allocation.
        result = self.channel.close()
        self.owner.mark('closed', None)
        return result

    def __getattr__(self, name):
        return getattr(self.channel, name)


class ClientProbe:
    """One synchronous client/thread, fixed recent ring + one slowest record.

    Records client-local intervals, not kernel arrival times or a root cause.
    GC callbacks only observe the calling thread, without collecting/tuning.
    No packets, ACK contents, errors, image data or unbounded history retained.
    """
    def __init__(self, *, clock=time.monotonic):
        self.clock = clock
        self.recent = deque(maxlen=CAPACITY)
        self.slowest = None
        self.active = None
        self.total = self.dropped = 0
        self.thread = None
        self.gc_started = [None] * 3

    def mark(self, stamp, phase):
        if self.active is not None:
            self.active['stamps'][stamp] = scalar(self.clock())
            if phase is not None:
                self.active['phase'] = phase

    def gc_callback(self, phase, info):
        if self.active is None or threading.get_ident() != self.thread:
            return
        generation = info.get('generation')
        if type(generation) is not int or not 0 <= generation <= 2:
            return
        if phase == 'start':
            self.gc_started[generation] = (self.clock(), self.active['phase'])
        elif phase == 'stop' and self.gc_started[generation] is not None:
            start, stage = self.gc_started[generation]
            self.gc_started[generation] = None
            duration = max(0., self.clock() - start)
            item = self.active['gc'][generation]
            item['count'] = increment(item['count'])
            item['total_s'] = min(1e12, item['total_s'] + duration)
            if duration > item['longest_s']:
                item['longest_s'], item['longest_phase'] = min(1e12, duration), stage

    @contextmanager
    def attach(self, client):
        if isinstance(client.channel, _SocketProbe) or self.thread is not None:
            raise ValueError('Probe requires an uninstrumented synchronous client')
        self.thread = threading.get_ident()
        original = {name: client.__dict__.get(name) for name in ('request', '_receive', '_validate_ack', 'channel')}
        present = {name for name in original if name in client.__dict__}
        request, receive, validate = client.request, client._receive, client._validate_ack

        def wrapped_receive(timeout):
            self.mark('wait_started', 'wait_ack')
            result = receive(timeout)  # Original select/recv/decode, exact timeout.
            self.mark('decoded', 'validation_pending')
            return result

        def wrapped_validate(reply, packet, sent, received):
            # Preserve the ORIGINAL caller's receipt time, not our clock read.
            self.mark('validation_started', 'validate_ack')
            if self.active is not None:
                self.active['receipt_argument_s'] = scalar(received)
            result = validate(reply, packet, sent, received)
            self.mark('validated', 'commit_return')
            return result

        def wrapped_request(kind, **values):
            self.active = dict(kind=kind if type(kind) is str and kind in {
                'command', 'idle', 'stop', 'shutdown'} else 'invalid',
                request_id=None, phase='encode', outcome='raised',
                issued_s=scalar(values.get('issued_steady_s')),
                expires_s=scalar(values.get('expires_steady_s')),
                receipt_argument_s=None, stamps=dict.fromkeys(STAMPS),
                gc=[dict(count=0, total_s=0., longest_s=0., longest_phase=None) for _ in range(3)])
            self.gc_started = [None] * 3
            self.mark('started', 'encode')
            try:
                result = request(kind, **values)
                self.active['outcome'] = 'returned'
                return result
            finally:
                # Original request already failed/closed or returned unchanged.
                self.mark('finished', None)
                seq = getattr(client, 'request_id', None)
                self.active['request_id'] = seq if type(seq) is int and 0 <= seq <= MAX_COUNTER else None
                item = self.active
                self.active = None
                self.total = increment(self.total)
                if len(self.recent) == CAPACITY:
                    self.dropped = increment(self.dropped)
                self.recent.append(item)
                if self.slowest is None or elapsed(item) > elapsed(self.slowest):
                    self.slowest = item

        callback = self.gc_callback
        try:
            client.channel = _SocketProbe(client.channel, self)
            client.request, client._receive, client._validate_ack = wrapped_request, wrapped_receive, wrapped_validate
            gc.callbacks.append(callback)
            yield self
        finally:
            gc.callbacks.remove(callback)
            for name in original:
                if name in present:
                    setattr(client, name, original[name])
                else:
                    delattr(client, name)
            self.thread, self.active = None, None
            self.gc_started = [None] * 3

    def snapshot(self):
        # Detached fixed-schema data; callers cannot mutate retained evidence.
        result = dict(version=1, capacity=CAPACITY, total=self.total,
            dropped=self.dropped, counters_saturate_at=MAX_COUNTER,
            counts_are_lower_bounds=self.total == MAX_COUNTER or self.dropped == MAX_COUNTER,
            recent=list(self.recent), slowest=self.slowest,
            scope='instrumented client-local phases; not kernel arrival, root cause, runtime repair or acceptance')
        raw = json.dumps(result, allow_nan=False)
        if len(raw.encode()) > MAX_SNAPSHOT_BYTES:
            raise ValueError('Diagnostic snapshot exceeds fixed byte budget')
        return json.loads(raw)


def elapsed(item):
    start, end = (item['stamps'][name] for name in ('started', 'finished'))
    return end - start if start is not None and end is not None else 0.


class DiagnosticPlugin:
    def __init__(self, output):
        self.output = Path(output)
        self.probe = None
        self.phases = []
        self.gc_before = dict(enabled=gc.isenabled(), thresholds=gc.get_threshold())

    @pytest.hookimpl(wrapper=True)
    def pytest_runtest_call(self, item):
        if item.nodeid == TARGET:
            self.probe = ClientProbe()
            with self.probe.attach(item.funcargs['client'][0]):
                return (yield)
        return (yield)

    def pytest_runtest_logreport(self, report):
        if report.nodeid == TARGET:
            if len(self.phases) < 3:
                self.phases.append(dict(when=report.when, outcome=report.outcome))

    def pytest_sessionfinish(self, session, exitstatus):
        root = Path(__file__).resolve().parents[1]
        files = ('simulation/gazebo_actuator.py', 'simulation/gazebo_receiver.py',
                 TARGET.split('::')[0], 'tools/actuator_ipc_diagnostics.py')
        result = dict(target=TARGET, pytest_exit_code=int(exitstatus), phases=self.phases,
            trace=self.probe.snapshot() if self.probe is not None else None,
            files={p: hashlib.sha256((root/p).read_bytes()).hexdigest() for p in files},
            original_deadline_s=.8, commands_expected=8193, requests_expected=8195,
            gc_before=self.gc_before,
            gc_after=dict(enabled=gc.isenabled(), thresholds=gc.get_threshold()),
            runtime_source_modified=False, instrumentation_adds_overhead=True,
            prior_full_suite_failure_reclassified=False)
        with self.output.open('x') as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
            stream.write('\n')


def pytest_addoption(parser):
    parser.addoption('--actuator-ipc-output', help='NEW post-session diagnostic JSON path')


def pytest_configure(config):
    output = config.getoption('--actuator-ipc-output')
    if output is None or Path(output).exists() or not Path(output).parent.is_dir():
        raise pytest.UsageError('actuator_ipc_diagnostics requires a new --actuator-ipc-output path')
    config.pluginmanager.register(DiagnosticPlugin(output), 'actuator-ipc-diagnostic-recorder')


def main(argv=None):
    """Exactly one original zero-only test; no retries or simulated motion."""
    import argparse
    import os
    import subprocess
    import sys
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New output directory')
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parents[1]
    for name in ('PYTEST_ADDOPTS', 'PYTEST_PLUGINS'):
        if os.environ.get(name, '').strip():
            parser.error(f'Unset {name} for an explicit diagnostic run')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    command = [sys.executable, '-m', 'pytest', '-q', '-p', 'tools.actuator_ipc_diagnostics',
        '--actuator-ipc-output', str(output/'trace.json'), '--junitxml', str(output/'junit.xml'), TARGET]
    env = os.environ.copy()
    env['PYTHONPATH'] = os.pathsep.join([str(root), str(root/'ros2_ws/src/toponav_bridge'),
                                        *([env['PYTHONPATH']] if env.get('PYTHONPATH') else [])])
    files = [root/'simulation/gazebo_actuator.py', root/'simulation/gazebo_receiver.py',
             root/TARGET.split('::')[0], Path(__file__).resolve()]
    before = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    with (output/'command.json').open('x') as stream:
        json.dump(dict(command=command, cwd=str(root), files_before=before,
            run_policy='exactly once regardless of outcome; diagnostic not baseline full-suite replacement'), stream, indent=2)
    with (output/'pytest.log').open('xb') as stream:
        code = subprocess.run(command, cwd=root, env=env, stdout=stream, stderr=subprocess.STDOUT).returncode
    after = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    with (output/'summary.json').open('x') as stream:
        json.dump(dict(exit_code=code, source_unchanged=before == after,
            files_after=after, original_failure_retained=True, formal_navigation_acceptance=False), stream, indent=2)
    return code if code else 0 if before == after else 1


if __name__ == '__main__':
    raise SystemExit(main())
