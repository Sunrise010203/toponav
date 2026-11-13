"""Opt-in observational tooling; never tunes authority, lease or GC policy."""
import copy
import gc
import json
from types import SimpleNamespace as NS

import pytest

from simulation import gazebo_actuator as actuator
from tools import actuator_ipc_diagnostics as diagnostic
from tests.test_review_gazebo_watchdog import client, packet, SESSION


def fake(monkeypatch, *, delay=None, failure=None):
    clock = NS(now=10.)
    obj = actuator.ActuatorClient.__new__(actuator.ActuatorClient)
    obj.session, obj.request_id, obj.failed = SESSION, 0, False
    obj.status = dict(latched=False, reason='disarmed', wheels=[0., 0.],
                      command_seq=0, deadline_steady_s=None)
    obj.last_publication_s, obj.submission_observer = None, None
    operations = []
    answer = dict(kind='ack', request_id=1, accepted=True, published_steady_s=10.,
        status=dict(latched=False, reason='accepted', wheels=[0., 0.], command_seq=1, deadline_steady_s=10.8))

    def action(stage):
        operations.append(stage)
        if delay == stage:
            clock.now += .9
        if failure == stage:
            raise ValueError('original failure, not a diagnostic verdict')

    class Channel:
        closed = False
        def send(self, raw):
            action('send')
            return len(raw)
        def recv(self, size):
            assert size == actuator.MAX_PACKET + 1
            action('recv')
            return json.dumps(answer).encode()
        def fileno(self):
            return -1 if self.closed else 123
        def close(self):
            operations.append('close')
            self.closed = True

    def select(read, write, error, timeout):
        action('select')
        assert timeout == .8 and not write and not error
        return read, [], []

    original_decode = actuator.decode
    def decode(raw):
        result = original_decode(raw)
        if result.get('kind') == 'ack':
            action('decode')
        return result

    obj.channel = Channel()
    monkeypatch.setattr(actuator, 'time', NS(monotonic=lambda: clock.now))
    monkeypatch.setattr(actuator, 'select', NS(select=select))
    monkeypatch.setattr(actuator, 'decode', decode)
    if delay == 'observer' or failure == 'observer':
        obj.submission_observer = lambda *args: action('observer')
    values = packet(issued=10.)
    values['command'].update(state='observing', v_m_s=0.)
    values = {k: v for k, v in values.items() if k not in ('kind', 'session_id', 'request_id')}
    return obj, values, clock, operations


def invoke(obj, values):
    return obj.request('command', **values)


def test_success_forwards_same_reply_state_and_socket_calls(monkeypatch):
    obj, values, clock, operations = fake(monkeypatch)
    original_dict = obj.__dict__.copy()
    probe = diagnostic.ClientProbe(clock=lambda: clock.now)
    original_gc = list(gc.callbacks), gc.isenabled(), gc.get_threshold()
    with probe.attach(obj):
        result = invoke(obj, values)
        assert obj.status is result['status'] and obj.last_publication_s == 10.
    assert operations == ['send', 'select', 'recv', 'decode']
    assert obj.channel is original_dict['channel'] and obj.submission_observer is None
    assert all(name not in obj.__dict__ for name in ('request', '_receive', '_validate_ack'))
    assert (list(gc.callbacks), gc.isenabled(), gc.get_threshold()) == original_gc
    trace = probe.snapshot()
    assert trace['total'] == 1 and trace['dropped'] == 0
    row = trace['recent'][0]
    assert row['outcome'] == 'returned' and row['receipt_argument_s'] == 10.
    assert row['stamps']['closed'] is None and row['expires_s'] == 10.8
    assert not obj.failed


@pytest.mark.parametrize('stage,left,right', [
    ('observer', 'sent', 'wait_started'), ('select', 'wait_started', 'wait_done'),
    ('recv', 'wait_done', 'recv_done'), ('decode', 'recv_done', 'decoded')])
def test_delays_locate_intervals_without_blessing_late_ack(monkeypatch, stage, left, right):
    obj, values, clock, operations = fake(monkeypatch, delay=stage)
    before = copy.deepcopy(obj.status)
    probe = diagnostic.ClientProbe(clock=lambda: clock.now)
    with probe.attach(obj):
        with pytest.raises(RuntimeError, match='arrived after command deadline'):
            invoke(obj, values)
        assert obj.failed and obj.channel.fileno() == -1 and obj.status == before
    row = probe.snapshot()['recent'][0]
    stamps = row['stamps']
    assert stamps[right] - stamps[left] == pytest.approx(.9)
    assert row['receipt_argument_s'] == pytest.approx(10.9)
    assert row['phase'] == 'validate_ack' and row['outcome'] == 'raised'
    assert operations[-1] == 'close' and stamps['closed'] <= stamps['finished']
    with pytest.raises(RuntimeError, match='prior failure'):
        obj.request('stop')


@pytest.mark.parametrize('stage,phase', [('send', 'send'), ('observer', 'submission'),
    ('select', 'wait_ack'), ('recv', 'recv'), ('decode', 'decode_ack')])
def test_original_error_preserved_and_failure_closes_before_record(monkeypatch, stage, phase):
    obj, values, clock, operations = fake(monkeypatch, failure=stage)
    probe = diagnostic.ClientProbe(clock=lambda: clock.now)
    with probe.attach(obj):
        with pytest.raises(ValueError, match='original failure, not a diagnostic verdict'):
            invoke(obj, values)
    assert obj.failed and obj.channel.closed
    row = probe.snapshot()['recent'][0]
    assert row['phase'] == phase and row['stamps']['closed'] == 10.
    assert row['outcome'] == 'raised'
    raw = json.dumps(row)
    assert 'original failure' not in raw and 'command_seq' not in raw and 'frame_id' not in raw


def test_observer_and_validator_arguments_identity_are_preserved(monkeypatch):
    obj, values, clock, operations = fake(monkeypatch)
    observed = []
    obj.submission_observer = lambda raw, stamp: observed.append((raw, stamp))
    validate = obj._validate_ack
    accepted = []
    def record(reply, packet, sent, received):
        accepted.append((reply, packet, sent, received))
        return validate(reply, packet, sent, received)
    obj._validate_ack = record
    with diagnostic.ClientProbe(clock=lambda: clock.now).attach(obj):
        reply = invoke(obj, values)
    assert accepted[0][0] is reply and accepted[0][2:] == (10., 10.)
    assert observed[0][0] == accepted[0][1] and observed[0][1] == 10.
    assert obj._validate_ack is record


def test_manual_gc_events_are_bounded_observations_not_collection(monkeypatch):
    obj, values, clock, operations = fake(monkeypatch)
    probe = diagnostic.ClientProbe(clock=lambda: clock.now)
    def observe(*args):
        probe.gc_callback('start', {'generation': 2})
        clock.now += .3
        probe.gc_callback('stop', {'generation': 2})
        for invalid in (-1, 3, True, None):
            probe.gc_callback('start', {'generation': invalid})
    obj.submission_observer = observe
    before = gc.isenabled(), gc.get_threshold()
    with probe.attach(obj):
        invoke(obj, values)
    row = probe.snapshot()['recent'][0]['gc'][2]
    assert row['count'] == 1 and row['longest_s'] == pytest.approx(.3)
    assert row['longest_phase'] == 'submission'
    assert (gc.isenabled(), gc.get_threshold()) == before
    assert probe.gc_started == [None] * 3


def test_recent_ring_slowest_and_snapshot_detachment(monkeypatch):
    obj, _, clock, _ = fake(monkeypatch)
    def noop(kind, **values):
        obj.request_id += 1
        clock.now += 2. if obj.request_id == 1 else .001
        return obj.request_id
    obj.request = noop
    probe = diagnostic.ClientProbe(clock=lambda: clock.now)
    with probe.attach(obj):
        for i in range(1000):
            assert obj.request('idle') == i + 1
    assert obj.request is noop
    data = probe.snapshot()
    assert len(data['recent']) == 64 and data['total'] == 1000 and data['dropped'] == 936
    assert data['slowest']['request_id'] == 1 and data['recent'][0]['request_id'] == 937
    assert len(json.dumps(data).encode()) < diagnostic.MAX_SNAPSHOT_BYTES
    data['slowest']['phase'] = 'tampered'
    assert probe.snapshot()['slowest']['phase'] != 'tampered'


@pytest.mark.parametrize('value', [True, None, 'secret', float('nan'), float('inf'), -1, 10**500])
def test_untrusted_scalars_do_not_escape_fixed_schema(value):
    assert diagnostic.scalar(value) is None


def test_counters_saturate_instead_of_unbounded_integers():
    assert diagnostic.increment(diagnostic.MAX_COUNTER) == diagnostic.MAX_COUNTER


def test_nested_probe_refused_without_leaking_callbacks(monkeypatch):
    obj, values, clock, _ = fake(monkeypatch)
    before = list(gc.callbacks)
    with diagnostic.ClientProbe(clock=lambda: clock.now).attach(obj):
        with pytest.raises(ValueError, match='uninstrumented'):
            with diagnostic.ClientProbe().attach(obj):
                pytest.fail('nested instrumentation must be refused')
        invoke(obj, values)
    assert list(gc.callbacks) == before


def test_callback_ignores_other_threads_and_inactive_calls(monkeypatch):
    obj, values, clock, _ = fake(monkeypatch)
    probe = diagnostic.ClientProbe(clock=lambda: clock.now)
    probe.gc_callback('start', {'generation': 2})
    with probe.attach(obj):
        def observer(*args):
            thread = probe.thread
            probe.thread = -1
            probe.gc_callback('start', {'generation': 2})
            probe.thread = thread
        obj.submission_observer = observer
        invoke(obj, values)
    assert probe.snapshot()['recent'][0]['gc'][2]['count'] == 0


def test_real_zero_ipc_delayed_observer_still_fails_closed(client):
    import time
    obj, _, _ = client
    probe = diagnostic.ClientProbe()
    obj.submission_observer = lambda *args: time.sleep(.9)
    p = packet(issued=time.monotonic())
    p['command'].update(state='observing', v_m_s=0.)
    before = copy.deepcopy(obj.status)
    with probe.attach(obj):
        with pytest.raises(RuntimeError, match='arrived after command deadline'):
            obj.request('command', **{k: v for k, v in p.items() if k not in ('kind', 'session_id', 'request_id')})
    assert obj.failed and obj.channel.fileno() == -1 and obj.status == before
    row = probe.snapshot()['recent'][0]
    assert row['stamps']['wait_started'] - row['stamps']['sent'] >= .9
    assert row['outcome'] == 'raised' and row['phase'] == 'validate_ack'
    assert row['stamps']['closed'] is not None
    with pytest.raises(RuntimeError, match='prior failure'):
        obj.request('stop')
    # This is injected zero-only delay, NOT reproduction of the old root cause.


def test_plugin_never_writes_during_request_and_uses_new_output(tmp_path, monkeypatch):
    output = tmp_path/'trace.json'
    plugin = diagnostic.DiagnosticPlugin(output)
    plugin.probe = diagnostic.ClientProbe()
    assert not output.exists()
    plugin.pytest_sessionfinish(None, 1)
    result = json.loads(output.read_text())
    assert result['trace']['total'] == 0 and result['pytest_exit_code'] == 1
    assert result['original_deadline_s'] == .8 and result['commands_expected'] == 8193
    assert result['requests_expected'] == 8195
    assert result['gc_before'] == result['gc_after']
    with pytest.raises(FileExistsError):
        plugin.pytest_sessionfinish(None, 0)
