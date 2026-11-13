"""Runtime feedback-age gates; no receipt-time renewal or physical-stop claim."""
import copy
import json
import math
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from simulation import gazebo_cross_validation as runner
from simulation.gazebo_receiver import ReceiverEvidence
from tests.test_review_gazebo_failure_capture import harness


def status(*, wall=10., seq=1, deadline=10.8, sim=.1, latched=False):
    return dict(version=1, session_id='test', wall_s=wall, sim_s=sim,
        latched=latched, reason='remote_stop' if latched else 'accepted' if seq else 'disarmed',
        command_seq=seq, deadline_steady_s=deadline if seq else 0.,
        wheels=[0., 0.] if latched or not seq else [.1, 0.])


def evidence(item=None):
    result = ReceiverEvidence('test')
    if item is not None:
        result.push(json.dumps(item))
        assert result.error is None
    return result


def adapter(monkeypatch, item, now=10.1):
    io = runner.GazeboIO.__new__(runner.GazeboIO)
    io.condition = threading.Condition()
    io.receiver_evidence = evidence(item)
    io.actuator = Mock()
    io.command_context = dict(command=dict(v_m_s=.1, omega_rad_s=0.), sim_s=.1,
                              issued_steady_s=now, expires_steady_s=now+.8)
    monkeypatch.setattr(runner, 'time', NS(monotonic=lambda: now))
    return io


def test_real_adapter_rejects_stale_disarmed_before_first_nonzero_submission(monkeypatch):
    io = adapter(monkeypatch, status(seq=0), now=10.81)
    with pytest.raises(RuntimeError, match='Receiver interlock.*stale'):
        io.command(.1, 0.)
    io.actuator.request.assert_not_called()
    assert io.command_context is None and io.receiver_evidence.error


@pytest.mark.parametrize('now', [10., 10.1, 10.79, math.nextafter(10.8, -math.inf)])
@pytest.mark.parametrize('seq', [0, 1])
def test_fresh_native_publication_is_ready_without_mutating_lease(now, seq):
    e = evidence(status(seq=seq)); before = copy.deepcopy(e.events)
    e.require_fresh(now)
    assert e.error is None and e.events == before
    assert not e.summary(0., .1)['fault_latched']


@pytest.mark.parametrize('now', [10.8, 10.800000001, 11., 100.])
def test_stale_disarmed_fails_even_without_any_previous_motion(now):
    e = evidence(status(seq=0))
    with pytest.raises(RuntimeError, match='stale'):
        e.require_fresh(now)
    assert e.summary(0., .1)['fault_latched']


@pytest.mark.parametrize('now', [10.25, 10.3, 10.7])
def test_live_status_age_does_not_extend_original_accepted_deadline(now):
    e = evidence(status(wall=10.2, deadline=10.25))
    with pytest.raises(RuntimeError, match='expired'):
        e.require_fresh(now)
    assert e.events[-1]['deadline_steady_s'] == 10.25


@pytest.mark.parametrize('now', [0., 9.9, math.nextafter(10., -math.inf)])
def test_future_status_never_grants_authority(now):
    e = evidence(status())
    with pytest.raises(RuntimeError, match='future'):
        e.require_fresh(now)
    assert e.error


@pytest.mark.parametrize('now', [None, True, False, '10.1', float('nan'), float('inf'),
                                 -float('inf'), -1., 10**1000])
def test_invalid_same_host_clock_latches_without_changing_events(now):
    e = evidence(status()); before = copy.deepcopy(e.events)
    with pytest.raises(RuntimeError, match='clock'):
        e.require_fresh(now)
    assert e.error and e.events == before


def test_clock_regression_is_detected_even_within_freshness_window():
    e = evidence(status())
    e.require_fresh(10.2)
    with pytest.raises(RuntimeError, match='regressed'):
        e.require_fresh(10.1)
    error = e.error
    with pytest.raises(RuntimeError):
        e.require_fresh(10.3)
    assert e.error == error


@pytest.mark.parametrize('kind', ['missing', 'latched', 'parse_error'])
def test_unready_feedback_fails_closed_and_later_valid_callback_cannot_recover(kind):
    e = evidence(None if kind == 'missing' else status(latched=kind == 'latched'))
    if kind == 'parse_error': e.push('{}')
    with pytest.raises(RuntimeError, match='Receiver interlock'):
        e.require_fresh(10.1)
    error = e.error
    e.push(json.dumps(status(wall=10.2, sim=.2)))
    with pytest.raises(RuntimeError): e.require_fresh(10.3)
    assert error and e.error == error


def test_stale_fault_survives_new_valid_commands_and_zero_evidence():
    e = evidence(status(seq=0))
    with pytest.raises(RuntimeError): e.require_fresh(11.)
    error = e.error
    e.push(json.dumps(status(wall=11., deadline=11.8, sim=.2)))
    e.push(json.dumps(status(wall=11.1, deadline=11.8, sim=.3, latched=True)))
    with pytest.raises(RuntimeError): e.require_ready()
    summary = e.summary(.2, .3)
    assert summary['final_step_fresh'] and summary['final_zero'] and summary['fault_latched']
    assert e.error == error and 'stationary' not in summary


def test_replayed_old_status_cannot_renew_freshness_or_deadline():
    e = evidence(status(seq=0))
    original = copy.deepcopy(e.events[0])
    for now in (10.1, 10.4, 10.7):
        e.push(json.dumps(original))
        e.require_fresh(now)
    e.push(json.dumps(original))
    with pytest.raises(RuntimeError, match='stale'): e.require_fresh(10.81)
    assert all(item == original for item in e.events)


def test_continuous_real_publication_not_receipt_renews_feedback_age_only():
    e = evidence(status(deadline=10.8))
    for stamp in (10.1, 10.3, 10.5, 10.7):
        e.push(json.dumps(status(wall=stamp, sim=round(stamp-10, 6))))
        e.require_fresh(stamp+.01)
    with pytest.raises(RuntimeError, match='expired'): e.require_fresh(10.8)
    assert {item['deadline_steady_s'] for item in e.events} == {10.8}


def test_no_argument_readiness_remains_offline_structural_check_only():
    # Frozen historical callers use synthetic wall stamps. They do not grant
    # runtime submission authority: the actual adapter must call require_fresh.
    e = evidence(status())
    e.require_ready()
    assert e.error is None


@pytest.mark.parametrize('item,now,reason', [
    (status(), 10.81, 'stale'),
    (status(wall=10.2, deadline=10.25), 10.3, 'expired'),
    (status(wall=10.2), 10.1, 'future'),
    (status(), float('nan'), 'clock'),
    (status(latched=True), 10.1, 'fault-latched'),
])
def test_adapter_rejection_consumes_context_but_never_submits_nonzero(monkeypatch, item, now, reason):
    io = adapter(monkeypatch, item, now)
    with pytest.raises(RuntimeError, match=reason): io.command(.1, 0.)
    io.actuator.request.assert_not_called()
    assert io.command_context is None and io.receiver_evidence.error
    io.command(0., 0.)
    io.actuator.request.assert_called_once_with('stop')


@pytest.mark.parametrize('kind', ['stop', 'idle', 'command'])
def test_zero_paths_bypass_even_sticky_feedback_fault_and_bad_clock(monkeypatch, kind):
    io = adapter(monkeypatch, status(), float('nan'))
    io.receiver_evidence.error = 'original feedback fault'
    if kind == 'stop': io.command_context = None
    elif kind == 'idle': io.command_context = {'idle': True}
    else: io.command_context['command'] = dict(v_m_s=0., omega_rad_s=0.)
    io.command(0., 0.)
    assert io.actuator.request.call_count == 1 and io.actuator.request.call_args.args == (kind,)
    assert io.receiver_evidence.error == 'original feedback fault'


@pytest.mark.parametrize('v,omega', [(.1, 0.), (0., .3), (0., -.3), (.15, .3)])
def test_all_nonzero_axes_use_freshness_gate(monkeypatch, v, omega):
    io = adapter(monkeypatch, status(seq=0), 10.81)
    io.command_context['command'] = dict(v_m_s=v, omega_rad_s=omega)
    with pytest.raises(RuntimeError, match='stale'): io.command(v, omega)
    io.actuator.request.assert_not_called()


def test_fresh_adapter_preserves_original_command_context(monkeypatch):
    io = adapter(monkeypatch, status())
    context = copy.deepcopy(io.command_context)
    io.command(.1, 0.)
    io.actuator.request.assert_called_once_with('command', **context)
    assert io.command_context is None and not io.receiver_evidence.error


def test_real_adapter_clock_regression_blocks_second_submission(monkeypatch):
    io = adapter(monkeypatch, status())
    context = copy.deepcopy(io.command_context)
    io.command(.1, 0.)
    io.command_context = context
    monkeypatch.setattr(runner.time, 'monotonic', lambda: 10.05)
    with pytest.raises(RuntimeError, match='regressed'): io.command(.1, 0.)
    io.actuator.request.assert_called_once_with('command', **context)


@pytest.mark.parametrize('kind', ['fresh', 'lost', 'replayed', 'latched', 'future'])
def test_disarmed_startup_wait_requires_real_new_fresh_feedback(monkeypatch, kind):
    io = adapter(monkeypatch, status(seq=0), now=20.)
    io.actuator.status = dict(latched=False, reason='disarmed', command_seq=0, wheels=[0., 0.])
    monkeypatch.setattr(runner.time, 'sleep', lambda _: None, raising=False)
    def step(count):
        if kind in ('lost', 'replayed'):
            if kind == 'replayed': io.receiver_evidence.push(json.dumps(status(seq=0)))
        else:
            io.receiver_evidence.push(json.dumps(status(wall=21. if kind == 'future' else 20.,
                seq=0, sim=.2, latched=kind == 'latched')))
    io.step = Mock(side_effect=step)
    if kind == 'fresh':
        io.refresh_disarmed_receiver()
        assert not io.receiver_evidence.error
    else:
        with pytest.raises(RuntimeError): io.refresh_disarmed_receiver()
        assert io.receiver_evidence.error
    io.step.assert_called_once_with(12)
    io.actuator.request.assert_not_called()


@pytest.mark.parametrize('side', ['native', 'ack'])
@pytest.mark.parametrize('kind', ['accepted', 'stopped', 'fault', 'previous_command'])
def test_startup_observation_cannot_refresh_active_or_previously_used_authority(monkeypatch, side, kind):
    io = adapter(monkeypatch, status(seq=0), now=20.)
    io.actuator.status = dict(latched=False, reason='disarmed', command_seq=0, wheels=[0., 0.])
    changed = status(latched=kind == 'stopped')
    if side == 'native':
        io.receiver_evidence = evidence(changed)
        if kind == 'fault': io.receiver_evidence.error = 'sticky status fault'
    else:
        io.actuator.status.update({key: changed[key] for key in io.actuator.status})
        if kind == 'fault': io.actuator.status.update(latched=True, reason='command_lease_expired', wheels=[0., 0.])
        if kind == 'previous_command': io.actuator.status.update(reason='disarmed', command_seq=1, wheels=[0., 0.])
    io.step = Mock()
    with pytest.raises(RuntimeError): io.refresh_disarmed_receiver()
    io.step.assert_not_called()
    io.actuator.request.assert_not_called()


def test_startup_unknown_step_result_never_retries_or_sends_commands(monkeypatch):
    io = adapter(monkeypatch, status(seq=0), now=20.)
    io.actuator.status = dict(latched=False, reason='disarmed', command_seq=0, wheels=[0., 0.])
    io.step = Mock(side_effect=TimeoutError('step outcome unknown'))
    with pytest.raises(TimeoutError): io.refresh_disarmed_receiver()
    io.step.assert_called_once_with(12)
    io.actuator.request.assert_not_called()


def test_gui_start_wait_calls_disarmed_initializer_before_first_motion(tmp_path, harness, monkeypatch):
    from simulation import gazebo_scenes
    control, _, calls = harness
    fake_io = runner.GazeboIO
    output = tmp_path/'capture'
    wall = NS(now=10.)
    monkeypatch.setattr(runner.time, 'monotonic', lambda: wall.now)
    original_update = control.update
    def update(frame, now):
        return dict(original_update(frame, now), visited_nodes=[], phase='test')
    control.update = update
    def display(*args):
        wall.now = 20.
        (output/'start.requested').write_text('explicit test start\n')
    monkeypatch.setattr(gazebo_scenes, 'write_display', display)
    server, gui = Mock(pid=123456), Mock(pid=123457)
    server.poll.return_value = None
    gui.poll.side_effect = [None, None, None, 0, 0]
    monkeypatch.setattr(runner.subprocess, 'Popen', Mock(side_effect=[server, gui]))
    instances = []
    class IO(fake_io):
        command = runner_module_command
        refresh_disarmed_receiver = runner_refresh_disarmed
        def __init__(self, **kw):
            super().__init__(**kw)
            self.receiver_evidence = evidence(status(seq=0))
            self.actuator = Mock()
            self.actuator.status = dict(latched=False, reason='disarmed', command_seq=0, wheels=[0., 0.])
            self.actuator.request.side_effect = lambda kind, **kw: calls.append(('submission', kind))
            instances.append(self)
        def step(self, iterations):
            super().step(iterations)
            self.receiver_evidence.push(json.dumps(status(wall=wall.now, seq=0, sim=self.now_ns/1e9)))
    monkeypatch.setattr(runner, 'GazeboIO', IO)
    monkeypatch.setattr('simulation.gazebo_actuator.summarize_audit', lambda *a, **kw: {})
    result = runner.run(output, scenario='vision', gui=True, wait_for_start=True, max_ticks=1)
    assert result['exception'] is None and not instances[0].receiver_evidence.error
    assert instances[0].steps == 17  # 5 startup + 1 disarmed refresh + 1 tick + 10 cleanup
    assert [('submission', 'idle'), ('submission', 'command')] == [c for c in calls if c[0] == 'submission'][:2]


@pytest.mark.parametrize('kind', ['lost', 'delayed', 'expired', 'future'])
def test_runner_fault_uses_real_adapter_gate_and_preserves_failed_tick(tmp_path, harness, monkeypatch, kind):
    control, _, calls = harness
    fake_io = runner.GazeboIO
    instances = []
    class IO(fake_io):
        command = runner_module_command
        def __init__(self, **kw):
            super().__init__(**kw)
            self.receiver_evidence = evidence(status(seq=0, wall=9.3 if kind in ('lost', 'delayed') else 10.))
            self.actuator = Mock()
            self.actuator.status = dict(command_seq=0)
            self.actuator.request.side_effect = lambda *a, **kw: calls.append(('submission', a, kw))
            instances.append(self)
        def next_frame(self, previous):
            frame = super().next_frame(previous)
            if control.calls:
                if kind == 'delayed': self.receiver_evidence.push(json.dumps(status(seq=0, wall=9.3)))
                elif kind == 'expired':
                    self.receiver_evidence.push(json.dumps(status(wall=10., deadline=10.000000001)))
                elif kind == 'future': self.receiver_evidence.push(json.dumps(status(wall=11., seq=0)))
                now = {'lost': 10.2, 'delayed': 10.2, 'expired': 10.1, 'future': 10.1}[kind]
                monkeypatch.setattr(runner.time, 'monotonic', lambda: now)
            return frame
    # Avoid relying on a fake adapter's permissive command implementation.
    monkeypatch.setattr(runner, 'GazeboIO', IO)
    monkeypatch.setattr('simulation.gazebo_actuator.summarize_audit', lambda *a, **kw: {})
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=3)
    assert not result['completed'] and 'Receiver interlock' in result['exception']
    assert control.state == 'aborted' and control.cancelled
    record = result['failure_record']
    assert record['pending_tick']['tick'] == 1 and record['pending_tick']['stage'] == 'command_submission'
    assert record['committed_decision_count'] == 1 and record['zero_command_submitted']
    assert record == json.loads((tmp_path/'capture/audit/failure.json').read_text())
    assert len((tmp_path/'capture/decisions.jsonl').read_text().splitlines()) == 1
    requests = instances[0].actuator.request.call_args_list
    assert [call.args[0] for call in requests] == ['command', 'stop', 'stop']
    assert result['cleanup_stop']['zero_command_submitted'] and result['cleanup_stop']['steps_completed'] == 10
    assert result['receiver_interlock']['fault_latched']
    assert result['independent_stop_observation'] is None


# Capture before harness monkeypatches the module's GazeboIO symbol.
runner_module_command = runner.GazeboIO.command
runner_refresh_disarmed = runner.GazeboIO.refresh_disarmed_receiver
