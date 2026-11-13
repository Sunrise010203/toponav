"""Bound native pose prefix; truncation cannot certify a stopped trajectory."""
import json
import threading
from types import SimpleNamespace as NS

import pytest

from simulation import gazebo_cross_validation as runner
from tests.test_review_receiver_freshness import adapter
from tests.test_review_gazebo_failure_capture import harness
from tests.test_review_independent_stop import observer
from tests.test_review_gazebo_watchdog import client

LIMIT = 65536
IO_CLASS = runner.GazeboIO


def message(stamp_ns=100_000_000, x=.5, name='robot'):
    sec, nsec = divmod(stamp_ns, 1_000_000_000)
    return NS(pose=[NS(name=name, header=NS(stamp=NS(sec=sec, nsec=nsec)),
        position=NS(x=x, y=0., z=.16), orientation=NS(x=0., y=0., z=0., w=1.))])


def io_at(poses=()):
    io = IO_CLASS.__new__(IO_CLASS)
    io.condition = threading.Condition(); io.poses = list(poses)
    io._pose_not_retained = 0; io.error = None
    return io


@pytest.fixture(scope='module')
def prefix():
    io = io_at(); msg = message()
    for _ in range(LIMIT): IO_CLASS.pose(io, msg)
    assert len(io.poses) == LIMIT and not io.error and not io._pose_not_retained
    return tuple(io.poses)


def exhausted(prefix):
    io = io_at(prefix); IO_CLASS.pose(io, message()); return io


def test_actual_callback_keeps_every_slot_without_preemptive_fault(prefix):
    io = io_at(prefix)
    assert runner.MAX_GAZEBO_POSES == LIMIT and len(io.poses) == LIMIT and not io.error
    assert io.poses[0] == io.poses[-1] == dict(sim_s=.1, position_m=[.5, 0., .16], yaw_deg=0.)
    assert not io._pose_not_retained


@pytest.mark.parametrize('entry', [NS(name='robot'), NS(name='robot', orientation=None),
                                  NS(name='robot', position='x'*2_000_001)])
def test_omitted_robot_entry_is_counted_without_parsing_or_overwriting(prefix, entry):
    io = io_at(prefix); before = list(io.poses)
    IO_CLASS.pose(io, NS(pose=[entry]))
    assert io.poses == before and io._pose_not_retained == 1
    assert io.error == 'Gazebo pose history capacity exceeded'


def test_last_slot_remains_usable_and_same_message_cannot_overflow_list(prefix):
    io = io_at(prefix[:-1]); msg = message(stamp_ns=200_000_000)
    msg.pose *= 4; IO_CLASS.pose(io, msg)
    assert len(io.poses) == LIMIT and io.poses[-1]['sim_s'] == .2
    assert io._pose_not_retained == 3 and io.error


def test_nonrobot_and_empty_callbacks_never_consume_pose_slots_or_omission_count(prefix):
    io = io_at(prefix)
    IO_CLASS.pose(io, NS(pose=[]))
    IO_CLASS.pose(io, NS(pose=[NS(name='wall'), NS(name='other')]))
    assert len(io.poses) == LIMIT and not io._pose_not_retained and not io.error


@pytest.mark.parametrize('original', ['camera format failed', 'earlier physical audit fault'])
def test_existing_io_error_is_not_replaced_by_capacity(prefix, original):
    io = io_at(prefix); io.error = original
    IO_CLASS.pose(io, message())
    assert io.error == original and io._pose_not_retained == 1


def test_omission_count_saturates_at_uint64_without_changing_prefix(prefix):
    io = io_at(prefix); maximum = runner.MAX_POSE_NOT_RETAINED
    assert maximum == 2**64-1
    io._pose_not_retained = maximum-1
    for _ in range(5): IO_CLASS.pose(io, message())
    assert io._pose_not_retained == maximum and len(io.poses) == LIMIT


def test_original_condition_serializes_concurrent_capacity_boundary(prefix):
    io = io_at(prefix[:-1]); msg = message(stamp_ns=200_000_000)
    threads = [threading.Thread(target=IO_CLASS.pose, args=(io, msg)) for _ in range(8)]
    for t in threads: t.start()
    for t in threads: t.join(2.); assert not t.is_alive()
    assert len(io.poses) == LIMIT and io._pose_not_retained == 7 and io.error


@pytest.mark.parametrize('velocity', [(.1, 0.), (0., .3)])
def test_real_adapter_rejects_nonzero_without_touching_receiver_or_actuator(prefix, monkeypatch, velocity):
    io = adapter(monkeypatch, None, now=10.2); io._pose_not_retained = exhausted(prefix)._pose_not_retained
    with pytest.raises(RuntimeError, match='pose history capacity'): io.command(*velocity)
    assert not io.receiver_evidence.error
    io.actuator.request.assert_not_called()
    io.command(0., 0.); io.actuator.request.assert_called_once_with('stop')


def test_camera_wait_reports_pose_fault_without_reusing_an_existing_frame(prefix):
    io = exhausted(prefix)
    with pytest.raises(RuntimeError, match='pose history capacity'): io.next_frame(-1)


def test_actual_adapter_stop_request_and_ipc_ack_remain_healthy_after_pose_fault(prefix, client, monkeypatch):
    obj, _, _ = client
    io = adapter(monkeypatch, None, now=10.2); io.actuator = obj
    io._pose_not_retained = exhausted(prefix)._pose_not_retained; io.command_context = None
    io.command(0., 0.)
    assert not obj.failed and obj.status['latched'] and obj.status['wheels'] == [0., 0.]
    assert obj.channel.fileno() >= 0


def test_failed_channel_observer_refuses_known_truncated_pose_history_before_any_step(observer):
    io, _, _ = observer; io._pose_not_retained = 1
    result = runner._observe_failed_actuator_stop(io)
    assert not io.calls and not result['stationary'] and not result['authority_confirmed']
    assert 'pose history capacity' in result['error']


@pytest.mark.parametrize('at_step', [1, 5, 10])
def test_failed_channel_observer_stops_when_capacity_is_lost_during_observation(observer, at_step):
    io, _, _ = observer; original = io.step
    def step(iterations):
        original(iterations)
        if len(io.calls) == at_step: io._pose_not_retained = 1
    io.step = step
    result = runner._observe_failed_actuator_stop(io)
    assert result['steps_completed'] == at_step and len(io.calls) == at_step
    assert not result['observation_complete'] and not result['stationary']
    assert 'pose history capacity' in result['error']


def test_runner_pose_fault_cancels_without_nonzero_and_preserves_cleanup_and_prefix(tmp_path, harness, prefix):
    control, _, calls = harness; original = runner.GazeboIO
    class IO(original):
        def __init__(self, **kw):
            super().__init__(**kw); self.poses = list(prefix); self._pose_not_retained = 1
        def command(self, v, omega):
            if v or omega: raise RuntimeError('Gazebo pose history capacity exceeded')
            return super().command(v, omega)
    runner.GazeboIO = IO
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=3)
    assert not result['completed'] and control.cancelled and result['controller_result']['state'] == 'aborted'
    assert result['failure_record']['pending_tick']['stage'] == 'command_submission'
    assert result['failure_record']['zero_command_submitted'] and result['cleanup_stop']['steps_completed'] == 10
    assert not result['audit']['stationary'] and not result['audit']['post_stop_pose_pair_fresh']
    assert result['audit']['pose_history']['capacity_exceeded']
    assert len(json.loads((tmp_path/'capture/audit/poses.json').read_text())) == LIMIT
    assert not any(c[0] == 'command' and (c[1] or c[2]) for c in calls)


@pytest.mark.parametrize('count', [1, 2**64-1])
def test_capacity_during_cleanup_invalidates_even_an_otherwise_fresh_stationary_pair(tmp_path, monkeypatch, harness, count):
    from tests.test_review_gazebo_post_stop import terminal_io
    terminal_io(monkeypatch, harness); original = runner.GazeboIO
    class IO(original):
        def step(self, iterations):
            super().step(iterations)
            if self.steps == 16: self._pose_not_retained = count
    monkeypatch.setattr(runner, 'GazeboIO', IO)
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=1)
    assert result['controller_result']['state'] == 'visual_stop' and result['audit']['normal_geometric_stop_pass']
    assert result['cleanup_stop']['steps_completed'] == 10 and result['cleanup_stop']['error'] is None
    assert result['audit']['final_speed_m_s'] == result['audit']['final_yaw_rate_rad_s'] == 0.
    assert not result['audit']['stationary'] and not result['audit']['post_stop_pose_pair_fresh']
    assert not result['completed'] and 'physical audit incomplete' in result['exception']
    history = result['audit']['pose_history']
    assert history['not_retained'] == count and history['not_retained_count_is_lower_bound'] == (count == 2**64-1)


def test_pose_capacity_in_cleanup_does_not_replace_primary_execution_exception(tmp_path, harness):
    _, fault, _ = harness; fault.kind = 'perception'; original = runner.GazeboIO
    class IO(original):
        def step(self, iterations):
            super().step(iterations)
            if self.steps >= 7: self._pose_not_retained = 1
    runner.GazeboIO = IO
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=3)
    assert 'backend failed' in result['exception'] and not result['completed']
    assert result['audit']['pose_history']['capacity_exceeded']


def test_budget_covers_declared_nominal_task_but_not_all_resources():
    assert LIMIT > 2200*12+5*12+10*12
    assert runner.MAX_POSE_NOT_RETAINED == 2**64-1
