"""Failed-channel observation without reopening transport or masking failure."""
import copy
import json
import math
from types import SimpleNamespace as NS

import pytest

from simulation import gazebo_cross_validation as runner
from simulation.gazebo_receiver import ReceiverEvidence
from tests.test_review_gazebo_failure_capture import harness


@pytest.fixture
def observer(monkeypatch):
    clock = NS(wall=10.1, on_wait=None)
    class Condition:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def wait(self, timeout):
            clock.wall += timeout
            if clock.on_wait: clock.on_wait()
    def sleep(duration): clock.wall += duration
    monkeypatch.setattr(runner, 'time', NS(monotonic=lambda: clock.wall, sleep=sleep))
    evidence = ReceiverEvidence('stop-test')
    def status(*, latched=True, seq=1, wall=None, sim=1., reason=None):
        return dict(version=1, session_id='stop-test', wall_s=clock.wall if wall is None else wall,
            sim_s=sim, latched=latched, reason=reason or ('remote_stop' if latched else 'accepted'),
            command_seq=seq, deadline_steady_s=10.8 if seq else 0., wheels=[0., 0.] if latched else [.1, 0.])
    evidence.push(json.dumps(status(wall=10.)))
    def forbidden(*args, **kwargs): raise AssertionError('Observation must not publish or reopen actuator')
    io = NS(condition=Condition(), now_ns=1_000_000_000, poses=[], receiver_evidence=evidence,
        actuator=NS(failed=True, channel=NS(fileno=lambda: -1), request=forbidden,
                    status=dict(latched=False, reason='accepted', wheels=[.1, 0.], command_seq=1, deadline_steady_s=10.8)),
        _step_service=NS(failed=False, process=NS(poll=lambda: None)), calls=[], kind='valid')
    def step(iterations):
        io.calls.append(iterations)
        if io.kind == 'step_error':
            io._step_service.failed = True
            raise RuntimeError('step outcome unknown')
        io.now_ns += 100_000_000; clock.wall += .01
        stamp = io.now_ns/1e9
        pair = [dict(sim_s=t, position_m=[.5, 0., .16], yaw_deg=0.) for t in (stamp-.02, stamp-.01)]
        if io.kind == 'stale': pair = []
        elif io.kind == 'single': pair = pair[-1:]
        elif io.kind == 'duplicate': pair[0]['sim_s'] = pair[1]['sim_s']
        elif io.kind == 'reversed': pair.reverse()
        elif io.kind == 'future':
            for i, pose in enumerate(pair): pose['sim_s'] = stamp+.01+i*.01
        elif io.kind == 'moving': pair[-1]['position_m'][0] += .01
        elif io.kind == 'turning': pair[-1]['yaw_deg'] += 1.
        elif io.kind == 'yaw_wrap': pair[0]['yaw_deg'], pair[1]['yaw_deg'] = 179.999, -179.999
        elif io.kind == 'nan': pair[-1]['position_m'][0] = float('nan')
        elif io.kind == 'inf': pair[-1]['yaw_deg'] = float('inf')
        elif io.kind == 'bool': pair[-1]['sim_s'] = True
        elif io.kind == 'bad_shape': pair[-1]['position_m'] = [.5, 0.]
        elif io.kind == 'overflow': pair[0]['position_m'][0], pair[1]['position_m'][0] = -1e308, 1e308
        io.poses.extend(pair)
        if io.kind == 'receiver_error': io.receiver_evidence.push('{}')
        elif io.kind == 'receiver_loss': pass
        else: io.receiver_evidence.push(json.dumps(status(sim=stamp)))
    io.step = step
    return io, clock, status


def observe(observer): return runner._observe_failed_actuator_stop(observer[0])


def test_fresh_zero_then_ten_steps_observe_stop_without_ack_or_motion(observer):
    io, _, _ = observer; before = copy.deepcopy(io.actuator.status)
    result = observe(observer)
    assert result['authority_confirmed'] and result['observation_complete'] and result['stationary']
    assert result['error'] is None and result['steps_completed'] == 10 and io.calls == [12]*10
    assert result['post_stop_pose_pair_fresh'] and result['receiver_status']['final_step_fresh']
    assert result['final_speed_m_s'] == result['final_yaw_rate_rad_s'] == 0.
    assert io.actuator.failed and io.actuator.channel.fileno() == -1 and io.actuator.status == before
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('change', ['healthy_client', 'open_channel', 'failed_helper', 'dead_helper',
                                   'no_sequence', 'boolean_sequence', 'bad_receiver', 'future_status'])
def test_ineligible_or_untrusted_state_never_steps(observer, change):
    io, clock, status = observer
    if change == 'healthy_client': io.actuator.failed = False
    elif change == 'open_channel': io.actuator.channel.fileno = lambda: 7
    elif change == 'failed_helper': io._step_service.failed = True
    elif change == 'dead_helper': io._step_service.process.poll = lambda: 1
    elif change == 'no_sequence': io.actuator.status = {}
    elif change == 'boolean_sequence': io.actuator.status['command_seq'] = True
    elif change == 'bad_receiver': io.receiver_evidence.push('{}')
    elif change == 'future_status':
        io.receiver_evidence = ReceiverEvidence('stop-test')
        io.receiver_evidence.push(json.dumps(status(wall=clock.wall+1)))
    result = observe(observer)
    assert result['error'] and not result['stationary'] and not result['observation_complete']
    assert io.calls == []


@pytest.mark.parametrize('kind', ['empty', 'unlatched', 'stale_zero', 'sequence_behind'])
def test_no_fresh_zero_times_out_without_step_or_new_authority(observer, kind):
    io, clock, status = observer; start = clock.wall
    io.receiver_evidence = ReceiverEvidence('stop-test')
    if kind == 'unlatched': item = status(latched=False)
    elif kind == 'stale_zero': item = status(wall=clock.wall-.8)
    elif kind == 'sequence_behind': item = status(seq=0)
    else: item = None
    if item: io.receiver_evidence.push(json.dumps(item))
    result = observe(observer)
    assert result['error'].startswith('TimeoutError:') and not result['authority_confirmed']
    assert io.calls == [] and clock.wall-start <= .8+1e-12
    assert io.actuator.failed


def test_can_wait_for_real_zero_but_never_step_before_it(observer):
    io, clock, status = observer; io.receiver_evidence = ReceiverEvidence('stop-test')
    io.receiver_evidence.push(json.dumps(status(latched=False)))
    def arriving():
        assert not io.calls
        if clock.wall >= 10.2:
            io.receiver_evidence.push(json.dumps(status())); clock.on_wait = None
    clock.on_wait = arriving
    result = observe(observer)
    assert result['stationary'] and result['authority']['wall_s'] >= 10.2


def test_bad_feedback_while_waiting_is_not_cleared_by_later_zero(observer):
    io, clock, status = observer; io.receiver_evidence = ReceiverEvidence('stop-test')
    def bad():
        io.receiver_evidence.push('{}'); io.receiver_evidence.push(json.dumps(status()))
    clock.on_wait = bad
    result = observe(observer)
    assert 'receiver audit failed' in result['error'] and not io.calls


def test_wait_clock_regression_fails_closed(observer):
    io, clock, _ = observer; io.receiver_evidence = ReceiverEvidence('stop-test')
    clock.on_wait = lambda: setattr(clock, 'wall', 9.)
    result = observe(observer)
    assert 'clock regressed' in result['error'] and not io.calls


@pytest.mark.parametrize('wall', [float('nan'), float('inf'), True, -1.])
def test_invalid_wall_clock_is_not_serialized_as_valid_evidence(observer, wall):
    io, clock, _ = observer; clock.wall = wall
    result = observe(observer)
    assert result['error'] and not io.calls and not result['stationary']
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('stamp', [float('nan'), float('inf'), True, -1.])
def test_invalid_simulation_clock_never_queues_a_step(observer, stamp):
    io, _, _ = observer; io.now_ns = stamp
    result = observe(observer)
    assert result['error'] and not io.calls and not result['stationary']
    json.dumps(result, allow_nan=False)


def test_zero_at_wait_deadline_does_not_extend_observation_budget(observer):
    io, clock, status = observer; io.receiver_evidence = ReceiverEvidence('stop-test')
    end = clock.wall+.8
    def arriving():
        if clock.wall >= end: io.receiver_evidence.push(json.dumps(status()))
    clock.on_wait = arriving
    result = observe(observer)
    assert result['error'].startswith('TimeoutError:') and not result['authority_confirmed'] and not io.calls


def test_helper_dying_while_waiting_never_queues_physics(observer):
    io, clock, _ = observer; io.receiver_evidence = ReceiverEvidence('stop-test')
    clock.on_wait = lambda: setattr(io._step_service.process, 'poll', lambda: 1)
    result = observe(observer)
    assert 'helper unavailable' in result['error'] and not io.calls


@pytest.mark.parametrize('kind', ['stale', 'single', 'duplicate', 'reversed', 'future',
                                 'nan', 'inf', 'bool', 'bad_shape', 'overflow'])
def test_bad_or_old_pose_pair_cannot_prove_stop(observer, kind):
    io, _, _ = observer; io.kind = kind
    result = observe(observer)
    assert result['steps_completed'] == 10 and result['error']
    assert not result['stationary'] and not result['observation_complete']
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('kind', ['moving', 'turning'])
def test_complete_observation_does_not_mean_stationary(observer, kind):
    io, _, _ = observer; io.kind = kind
    result = observe(observer)
    assert result['observation_complete'] and result['post_stop_pose_pair_fresh']
    assert not result['stationary'] and result['error'] is None


def test_yaw_wrapping_matches_original_physical_audit(observer):
    io, _, _ = observer; io.kind = 'yaw_wrap'
    result = observe(observer)
    assert result['stationary'] and result['final_yaw_rate_rad_s'] < .02


@pytest.mark.parametrize('speed,yaw,stationary', [(.0099, 0., True), (.01, 0., False),
                                                (.0101, 0., False), (0., .0199, True), (0., .0201, False)])
def test_original_physical_limits_are_not_relaxed(observer, speed, yaw, stationary):
    io, clock, status = observer
    def step(iterations):
        io.calls.append(iterations); start = io.now_ns/1e9; io.now_ns += 1_000_000_000; clock.wall += .01
        io.poses.extend([dict(sim_s=start+.25, position_m=[0., 0., .16], yaw_deg=0.),
                         dict(sim_s=start+.75, position_m=[speed*.5, 0., .16], yaw_deg=math.degrees(yaw*.5))])
        io.receiver_evidence.push(json.dumps(status(sim=io.now_ns/1e9)))
    io.step = step
    result = observe(observer)
    assert result['observation_complete'] and result['stationary'] is stationary


def test_successful_helper_reply_without_new_poses_is_not_stop_proof(observer):
    io, _, _ = observer; io.step = lambda n: io.calls.append(n)
    result = observe(observer)
    assert result['steps_completed'] == 10 and result['error'] and not result['stationary']


def test_unknown_step_outcome_is_never_retried_or_reset(observer):
    io, _, _ = observer; io.kind = 'step_error'
    result = observe(observer)
    assert 'step outcome unknown' in result['error'] and not result['stationary']
    assert io.calls == [12] and io._step_service.failed and result['steps_completed'] == 0


@pytest.mark.parametrize('kind', ['receiver_error', 'receiver_loss'])
def test_bad_or_missing_later_feedback_does_not_certify_stop(observer, kind):
    io, _, _ = observer; io.kind = kind
    result = observe(observer)
    assert not result['observation_complete'] and not result['stationary'] and result['error']
    if kind == 'receiver_error': assert io.calls == [12]
    else: assert result['steps_completed'] == 10 and not result['receiver_status']['final_step_fresh']


@pytest.mark.parametrize('reason', ['command_lease_expired', 'invalid_receiver_envelope', 'steady_clock_regression'])
def test_a_fault_zero_may_be_observed_without_becoming_navigation_success(observer, reason):
    io, clock, status = observer; clock.wall = 11.1
    io.receiver_evidence = ReceiverEvidence('stop-test')
    io.receiver_evidence.push(json.dumps(status(reason=reason)))
    # Preserve the native first fault; do not relax the evidence validator.
    def step_fault(iterations):
        io.calls.append(iterations); io.now_ns += 100_000_000; clock.wall += .01
        stamp = io.now_ns/1e9
        io.poses.extend(dict(sim_s=t, position_m=[.5, 0., .16], yaw_deg=0.) for t in (stamp-.02, stamp-.01))
        io.receiver_evidence.push(json.dumps(status(sim=stamp, reason=reason)))
    io.step = step_fault
    result = observe(observer)
    assert result['stationary'] and result['receiver_status']['fault_latched']
    assert io.actuator.failed and result['authority']['reason'] == reason


def test_helper_death_during_observation_stops_remaining_steps(observer):
    io, _, _ = observer; original = io.step
    def step(iterations):
        original(iterations)
        if len(io.calls) == 2: io._step_service.process.poll = lambda: 1
    io.step = step
    result = observe(observer)
    assert result['steps_completed'] == 2 and len(io.calls) == 2
    assert not result['observation_complete'] and result['error']


@pytest.mark.parametrize('observed', [True, False])
def test_run_persists_observation_separately_and_keeps_original_failure(tmp_path, monkeypatch, harness, observed):
    _, fault, _ = harness; fault.kind = 'perception'
    original = runner.GazeboIO
    class FailedFlag:
        failed = True
        def close(self): pass
        process = NS(pid=1234)
        status = None
    class IO(original):
        def __init__(self, **kwargs):
            super().__init__(**kwargs); self.actuator = FailedFlag()
        def command(self, v, omega):
            if v == omega == 0: raise RuntimeError('closed stop channel')
            return super().command(v, omega)
    monkeypatch.setattr(runner, 'GazeboIO', IO)
    sample = dict(stationary=observed, observation_complete=observed, error=None if observed else 'observation failed', steps_completed=10)
    calls = []
    monkeypatch.setattr(runner, '_observe_failed_actuator_stop', lambda io: calls.append(io) or sample)
    from simulation import gazebo_actuator
    monkeypatch.setattr(gazebo_actuator, 'summarize_audit', lambda *a, **k: dict(fault_latched=True))
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=3)
    assert len(calls) == 1 and result['independent_stop_observation'] == sample
    assert json.loads((tmp_path/'capture/audit/independent-stop.json').read_text()) == sample
    assert 'backend failed' in result['exception'] and 'closed stop channel' in result['cleanup_stop']['error']
    assert result['cleanup_stop']['steps_completed'] == 0 and not result['cleanup_stop']['zero_command_submitted']
    assert not result['completed'] and not result['audit']['stationary']
    assert result['failure_record']['exception'] == result['exception']
