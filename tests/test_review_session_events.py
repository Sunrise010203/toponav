"""Temporal event faults with deterministic clocks; not physical stop acceptance."""
import copy
import json
from pathlib import Path

import pytest

from simulation import navigation_session as module
from simulation.structural_localization import map_exits

ROOT = Path(__file__).resolve().parents[1]
NOW = 1_800_000_000_000_000_000


@pytest.fixture
def rig():
    graph = json.loads((ROOT / 'configs/graph.json').read_text())
    clocks = dict(mono=0., wall=NOW)
    session = module.NavigationSession(graph, clock=lambda: clocks['mono'], wall_ns=lambda: clocks['wall'],
                                       session_id='event-review')
    return session, clocks


def observation(session, node='N01'):
    exits = map_exits(session.graph, dict(node_id=node, heading_deg=0))
    return dict(relative_open={d: d in exits for d in module.TURNS}, confidence=1.)


def frame(session, clocks, node='N01'):
    clocks['mono'] += .1
    clocks['wall'] += 100_000_000
    return dict(frame_id=f"frame/{clocks['wall']}", captured_at_ns=clocks['wall'],
                observation=observation(session, node))


def issue(session, clocks):
    for _ in range(3):
        result = session.observe(frame(session, clocks))
    assert result['action'] == 'straight'
    return result['command_id']


def feedback(command='unknown'):
    return dict(event_id='completion', command_id=command, feedback=dict(status='succeeded'))


def assert_failed_closed(session, commands):
    assert session.state == 'failed' and not session.window
    assert session.metrics['commands_sent'] == commands
    result = session.tick()
    assert result['state'] == 'failed' and result['action'] == 'stop' and result['command_id'] is None
    json.dumps(session.report(), allow_nan=False)


BAD_FRAMES = [None, {}, [], {'frame_id': 'x'},
              dict(frame_id='', captured_at_ns=NOW, observation={}),
              dict(frame_id='x', captured_at_ns=True, observation={}),
              dict(frame_id='x', captured_at_ns=10**400, observation={}),
              dict(frame_id='x', captured_at_ns=NOW, observation={'truth': 'N01'})]
BAD_FEEDBACK = [None, {}, [], dict(event_id='x', command_id='x', feedback=None),
                dict(event_id='x', command_id='x', feedback=dict(status=[])),
                dict(event_id='x', command_id='x', feedback=dict(status=True)),
                dict(event_id='', command_id='x', feedback=dict(status='succeeded'))]


@pytest.mark.parametrize('phase', ['confirming', 'pending'])
@pytest.mark.parametrize('payload', BAD_FRAMES)
def test_malformed_frame_latches_failure_and_discards_partial_votes(rig, phase, payload):
    s, c = rig
    if phase == 'pending':
        issue(s, c)
    else:
        for _ in range(2):
            s.observe(frame(s, c))
    count = s.metrics['commands_sent']
    with pytest.raises(ValueError):
        s.observe(copy.deepcopy(payload))
    assert_failed_closed(s, count)
    assert s.observe(frame(s, c))['action'] == 'stop'
    assert s.metrics['commands_sent'] == count


@pytest.mark.parametrize('phase', ['confirming', 'pending'])
@pytest.mark.parametrize('payload', BAD_FEEDBACK)
def test_malformed_feedback_cannot_leave_an_active_session_or_raise_type_error(rig, phase, payload):
    s, c = rig
    if phase == 'pending':
        issue(s, c)
    else:
        for _ in range(2):
            s.observe(frame(s, c))
    count = s.metrics['commands_sent']
    with pytest.raises(ValueError):
        s.feedback(copy.deepcopy(payload))
    assert_failed_closed(s, count)


@pytest.mark.parametrize('terminal', ['reached', 'failed'])
@pytest.mark.parametrize('kind,payload', [('frame', p) for p in BAD_FRAMES] + [('feedback', p) for p in BAD_FEEDBACK])
def test_terminal_session_ignores_bad_events_and_does_not_read_clocks(rig, terminal, kind, payload):
    s, c = rig
    if terminal == 'reached':
        s = module.NavigationSession(s.graph, start='N01', goal='N01', heading=0,
                                     clock=lambda: c['mono'], wall_ns=lambda: c['wall'])
        for _ in range(3):
            s.observe(frame(s, c))
        assert s.state == 'reached'
    else:
        s.abort('test_cancel')
    before = copy.deepcopy((s.state, s.reason, s.candidates, s.metrics, s.pending, s.commands))
    def broken():
        raise AssertionError('terminal event must not read clocks')
    s.clock = s.wall_ns = broken
    result = s.observe(payload) if kind == 'frame' else s.feedback(payload)
    assert result['state'] == terminal and result['action'] == 'stop' and result['command_id'] is None
    assert (s.state, s.reason, s.candidates, s.metrics, s.pending, s.commands) == before
    json.dumps(s.report(), allow_nan=False)


BAD_CLOCKS = [True, None, '0', float('nan'), float('inf'), -float('inf'),
              pytest.param(10**400, id='overflowing-integer')]


@pytest.mark.parametrize('kind', ['tick', 'observe', 'feedback'])
@pytest.mark.parametrize('bad', BAD_CLOCKS)
def test_bad_monotonic_clock_fails_closed_before_processing_event(rig, kind, bad):
    s, c = rig
    cmd = issue(s, c)
    pending = s.pending
    packet = frame(s, c)
    c['mono'] = bad
    with pytest.raises(ValueError):
        if kind == 'tick':
            s.tick()
        elif kind == 'observe':
            s.observe(packet)
        else:
            s.feedback(feedback(cmd))
    assert_failed_closed(s, 1)
    assert s.pending is pending and pending['completion'] is None
    assert s.metrics['transitions_applied'] == 0


@pytest.mark.parametrize('kind', ['observe', 'feedback'])
@pytest.mark.parametrize('bad', BAD_CLOCKS + [-1])
def test_bad_wall_clock_does_not_consume_completion_or_change_epoch(rig, kind, bad):
    s, c = rig
    cmd = issue(s, c)
    pending = s.pending
    epoch, candidates, stage = s.epoch_wall_ns, copy.deepcopy(s.candidates), s.stage_started
    packet = frame(s, c)
    c['wall'] = bad
    with pytest.raises(ValueError):
        s.observe(packet) if kind == 'observe' else s.feedback(feedback(cmd))
    assert_failed_closed(s, 1)
    assert s.pending is pending and pending['completion'] is None
    assert s.candidates == candidates and s.epoch_wall_ns == epoch and s.stage_started == stage
    assert s.metrics['transitions_applied'] == s.metrics['execution_failures'] == 0
    assert 'completion' not in s.feedback_events


@pytest.mark.parametrize('status', ['succeeded', 'failed'])
def test_wall_rollback_cannot_lower_completion_barrier_and_admit_inflight_images(rig, status):
    s, c = rig
    cmd = issue(s, c)
    epoch, pending = s.epoch_wall_ns, s.pending
    old_frame = frame(s, c, 'N02')  # Captured in flight, not after completion.
    c['wall'] = epoch - 100_000_000
    event = feedback(cmd)
    event['feedback']['status'] = status
    with pytest.raises(ValueError, match='clock|Clock'):
        s.feedback(event)
    assert_failed_closed(s, 1)
    assert s.epoch_wall_ns == epoch and s.pending is pending and pending['completion'] is None
    c['wall'] = epoch + 500_000_000
    for i in range(3):
        packet = copy.deepcopy(old_frame)
        packet['frame_id'] += f'/{i}'
        packet['captured_at_ns'] += i
        assert s.observe(packet)['action'] == 'stop'
    assert s.metrics['commands_sent'] == 1


def test_wall_rollback_during_confirmation_discards_votes(rig):
    s, c = rig
    for _ in range(2):
        s.observe(frame(s, c))
    packet = frame(s, c)
    c['wall'] -= 200_000_000
    with pytest.raises(ValueError, match='clock|Clock'):
        s.observe(packet)
    assert_failed_closed(s, 0)


@pytest.mark.parametrize('kind', ['observe', 'feedback', 'tick'])
def test_unexpected_clock_exception_is_latched_then_propagated(rig, kind):
    s, c = rig
    cmd = issue(s, c)
    packet = frame(s, c)
    def broken():
        raise RuntimeError('clock disconnected')
    s.clock = broken
    with pytest.raises(RuntimeError, match='clock disconnected'):
        s.observe(packet) if kind == 'observe' else s.feedback(feedback(cmd)) if kind == 'feedback' else s.tick()
    assert_failed_closed(s, 1)


@pytest.mark.parametrize('kind', ['observe', 'feedback'])
def test_unexpected_localizer_exception_cannot_leave_session_active(rig, monkeypatch, kind):
    s, c = rig
    cmd = issue(s, c)
    pending = s.pending
    def broken(*args, **kwargs):
        raise RuntimeError('localizer failure')
    monkeypatch.setattr(module, 'update_candidates', broken)
    with pytest.raises(RuntimeError, match='localizer failure'):
        s.observe(frame(s, c)) if kind == 'observe' else s.feedback(feedback(cmd))
    assert_failed_closed(s, 1)
    assert s.pending is pending and pending['completion'] is None
    assert s.metrics['transitions_applied'] == 0


@pytest.mark.parametrize('field', ['max_frame_age_s', 'stage_timeout_s', 'session_timeout_s'])
def test_overflowing_duration_configuration_raises_value_error(field):
    with pytest.raises(ValueError):
        module.SessionConfig(**{field: 10**400})


@pytest.mark.parametrize('kind', ['wall', 'mono'])
def test_overflowing_initial_clock_raises_value_error(rig, kind):
    s, c = rig
    c[kind] = 10**400
    with pytest.raises(ValueError):
        module.NavigationSession(s.graph, clock=lambda: c['mono'], wall_ns=lambda: c['wall'])


@pytest.mark.parametrize('status', ['succeeded', 'failed'])
def test_equal_wall_completion_is_valid_and_preserves_strict_capture_barrier(rig, status):
    s, c = rig
    cmd = issue(s, c)
    epoch = s.epoch_wall_ns
    event = feedback(cmd)
    event['feedback']['status'] = status
    result = s.feedback(event)
    assert result['action'] == 'stop' and s.pending is None and s.epoch_wall_ns == epoch
    packet = frame(s, c, 'N02' if status == 'succeeded' else 'N01')
    packet['captured_at_ns'] = epoch
    rejected = s.observe(packet)
    assert rejected['reason'] == 'previous_epoch_frame' and rejected['window_size'] == 0


def test_stale_frames_remain_recoverable_not_protocol_failures(rig):
    s, c = rig
    packet = frame(s, c)
    packet['captured_at_ns'] -= 2_000_000_000
    result = s.observe(packet)
    assert result['state'] == 'recovering' and result['reason'] == 'stale_frame'
    assert issue(s, c)
