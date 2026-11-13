"""Synthetic blank exceptions and sticky fault state, not actual OOM recovery."""
from unittest.mock import Mock

import pytest

from simulation import gazebo_cross_validation as runner
from tests.test_review_image_ingress import IO, adapter, deliver, message
from tests.test_review_gazebo_failure_capture import harness
from tests.test_review_gazebo_post_stop import terminal_io
from tests.test_review_gazebo_watchdog import client


@pytest.mark.parametrize('kind', ['front', 'up', 'overview'])
@pytest.mark.parametrize('exc,text', [(MemoryError(), 'MemoryError'),
                                     (RuntimeError(), 'RuntimeError'),
                                     (ValueError('decoder failed'), 'decoder failed')])
def test_callback_records_nonempty_first_fault_and_notifies(kind, exc, text):
    io = adapter(); io.pixels = Mock(side_effect=exc)
    io.condition.notify_all = Mock()
    deliver(io, kind, message())
    assert io.error == text and io.overview is None
    io.condition.notify_all.assert_called_once_with()
    with pytest.raises(RuntimeError) as caught: io.next_frame(-1, timeout=0.)
    assert str(caught.value) == text


@pytest.mark.parametrize('kind', ['front', 'overview'])
@pytest.mark.parametrize('prior', ['', 'earlier fault'])
def test_even_blank_prior_fault_is_not_replaced_by_later_callback(kind, prior):
    io = adapter(); io.error = prior; io.pixels = Mock(side_effect=MemoryError())
    deliver(io, kind, message()); assert io.error == prior


@pytest.mark.parametrize('fault', ['', 'MemoryError', 'decoder failed'])
def test_fault_blocks_valid_synchronized_frame_and_nonzero_but_not_stop(fault):
    io = adapter()
    for view in runner.VIEWS: io.image(view, message())
    io.error = fault
    with pytest.raises(RuntimeError) as caught: io.next_frame(-1, timeout=0.)
    assert str(caught.value) == fault
    io.command_context = dict(command={'v_m_s': .1, 'omega_rad_s': 0.})
    with pytest.raises(RuntimeError, match='I/O callback failure'): io.command(.1, 0.)
    assert io.command_context is None
    io.receiver_evidence.require_fresh.assert_not_called()
    io.actuator.request.assert_not_called()
    io.command(0., 0.); io.actuator.request.assert_called_once_with('stop')


@pytest.mark.parametrize('kind', ['front', 'overview'])
def test_later_valid_image_cannot_clear_blank_exception_fault(kind):
    io = adapter(); original = io.pixels; io.pixels = Mock(side_effect=MemoryError())
    deliver(io, kind, message()); io.pixels = original
    deliver(io, kind, message(384 if kind == 'overview' else 128))
    assert io.error == 'MemoryError'
    assert (io.overview is not None) if kind == 'overview' else bool(io.cameras.views[kind])
    with pytest.raises(RuntimeError, match='MemoryError'): io.next_frame(-1, timeout=0.)


@pytest.mark.parametrize('fault', ['', 'MemoryError'])
def test_real_worker_accepts_frame_free_stop_after_fault(client, fault):
    io = adapter(); io.actuator = client[0]; io.error = fault; io.command(0., 0.)
    assert io.actuator.status['latched'] and io.actuator.status['reason'] == 'remote_stop'
    assert io.actuator.status['wheels'] == [0., 0.] and not io.actuator.failed
    assert io.error == fault


@pytest.mark.parametrize('kind', ['direct_blank', 'front', 'overview'])
def test_late_fault_denies_synthetic_completion_without_erasing_stop(tmp_path, monkeypatch, harness, kind):
    terminal_io(monkeypatch, harness); original = runner.GazeboIO
    class Adapter(original):
        def step(self, iterations):
            super().step(iterations)
            if self.steps == 16:
                self.error = None
                if kind == 'direct_blank': self.error = ''
                else:
                    self.pixels = Mock(side_effect=MemoryError())
                    if kind == 'front': self.cameras = runner.CameraBuffer()
                    deliver(self, kind, message())
        image = IO.image
        overview_image = IO.overview_image
    monkeypatch.setattr(runner, 'GazeboIO', Adapter)
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=1)
    assert not result['completed'] and result['exception'] == 'Gazebo I/O callback failure: '+('' if kind == 'direct_blank' else 'MemoryError')
    assert result['controller_result']['state'] == 'visual_stop' and result['audit']['normal_geometric_stop_pass']
    assert result['cleanup_stop']['steps_completed'] == 10 and result['cleanup_stop']['error'] is None
    assert result['audit']['stationary'] and result['audit']['post_stop_pose_pair_fresh']


@pytest.mark.parametrize('fault', ['', 'MemoryError'])
def test_late_fault_never_overwrites_primary_exception(tmp_path, monkeypatch, harness, fault):
    _, failure, _ = harness; failure.kind = 'perception'; original = runner.GazeboIO
    class Adapter(original):
        def step(self, iterations):
            super().step(iterations)
            if self.steps >= 7: self.error = fault
    monkeypatch.setattr(runner, 'GazeboIO', Adapter)
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=3)
    assert 'backend failed' in result['exception'] and 'I/O callback failure' not in result['exception']
    assert not result['completed']


def test_no_fault_keeps_original_terminal_contract(tmp_path, monkeypatch, harness):
    terminal_io(monkeypatch, harness)
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=1)
    assert result['completed'] and result['exception'] is None and result['audit']['stationary']
