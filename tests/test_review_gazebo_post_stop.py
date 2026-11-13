"""Execution/audit contracts with synthetic terminal decisions, NOT navigation tests."""
import json

import pytest

from simulation import gazebo_cross_validation as runner
from test_review_gazebo_failure_capture import harness


def terminal_io(monkeypatch, harness, kind='fresh'):
    control, fault, calls = harness
    def update(frame, now):
        control.calls += 1
        control.state = 'visual_stop'
        return dict(state='visual_stop', reason='synthetic_terminal', v_m_s=0., omega_rad_s=0.,
                    control_frame_id=frame['frame_id'], captured_sim_s=frame['captured_sim_s'], decision_sim_s=now)
    control.update = update
    original = runner.GazeboIO
    class IO(original):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            self.poses = [self.pose_at(.4), self.pose_at(.5)]
        @staticmethod
        def pose_at(stamp):
            return dict(sim_s=stamp, position_m=[2.5, 1.2, .16], yaw_deg=90.)
        def command(self, v, omega):
            if kind == 'zero_failure' and self.steps >= 6:
                raise RuntimeError('cleanup zero failed')
            return super().command(v, omega)
        def step(self, iterations):
            if kind == 'step_failure' and self.steps >= 6:
                raise RuntimeError('cleanup step failed')
            super().step(iterations)
            if self.steps < 7 or kind == 'stale': return
            stamp = self.now_ns/1e9
            if kind == 'duplicate':
                stamps = [stamp, stamp]
            elif kind == 'reversed':
                stamps = [stamp, stamp-.001]
            elif kind == 'future':
                stamps = [stamp+.001, stamp+.002]
            elif kind == 'single':
                # Only one sample after the final step starts; the other is
                # still post-zero but insufficient for final velocity proof.
                stamps = [stamp]
            else:
                stamps = [stamp-.02, stamp-.01]
            self.poses.extend(self.pose_at(t) for t in stamps)
            if kind == 'moving': self.poses[-1]['position_m'][0] += .01
            if kind == 'turning': self.poses[-1]['yaw_deg'] += 1.
    monkeypatch.setattr(runner, 'GazeboIO', IO)
    return control, fault, calls


@pytest.mark.parametrize('kind', ['zero_failure', 'step_failure', 'stale', 'single', 'duplicate', 'reversed', 'future'])
def test_terminal_state_cannot_complete_without_final_stop_proof(tmp_path, monkeypatch, harness, kind):
    terminal_io(monkeypatch, harness, kind)
    result = runner.run(tmp_path/'run', scenario='left', max_ticks=1)
    assert result['controller_result']['state'] == 'visual_stop'
    assert result['audit']['normal_geometric_stop_pass']
    assert not result['completed'], 'Terminal latch and old poses must not certify final stop'
    assert not result['audit']['stationary']
    if kind in ('zero_failure', 'step_failure'):
        assert result['exception'] and 'cleanup' in result['exception']
        assert result['cleanup_stop']['error']
    else:
        assert result['exception'] is None
        assert not result['audit']['post_stop_pose_pair_fresh']


def test_fresh_final_pair_can_still_complete_synthetic_terminal_contract(tmp_path, monkeypatch, harness):
    terminal_io(monkeypatch, harness)
    result = runner.run(tmp_path/'run', scenario='left', max_ticks=1)
    assert result['completed'] and result['audit']['stationary']
    stop = result['cleanup_stop']
    assert stop['zero_command_submitted'] and stop['steps_completed'] == 10 and stop['error'] is None
    assert result['audit']['post_stop_pose_pair_fresh']
    assert json.loads((tmp_path/'run/audit/cleanup-stop.json').read_text()) == stop
    assert stop['last_step_started_sim_s'] < result['audit']['final']['sim_s'] <= stop['final_sim_s']


@pytest.mark.parametrize('kind', ['moving', 'turning'])
def test_fresh_samples_do_not_relax_stationary_limits(tmp_path, monkeypatch, harness, kind):
    terminal_io(monkeypatch, harness, kind)
    result = runner.run(tmp_path/'run', scenario='left', max_ticks=1)
    assert not result['completed'] and not result['audit']['stationary']
    assert result['audit']['post_stop_pose_pair_fresh']


def test_cleanup_error_does_not_replace_primary_exception(tmp_path, monkeypatch, harness):
    _, fault, _ = terminal_io(monkeypatch, harness, 'zero_failure')
    fault.kind = 'step'  # First execution step fails; cleanup zero also fails.
    result = runner.run(tmp_path/'run', scenario='left', max_ticks=1)
    assert 'step failed' in result['exception'] and 'cleanup zero failed' not in result['exception']
    assert 'cleanup zero failed' in result['cleanup_stop']['error']
    assert result['failure_record']['exception'] == result['exception']
    assert not result['completed']


def test_no_pose_stream_is_explicitly_unverified(tmp_path, harness):
    result = runner.run(tmp_path/'run', scenario='left', max_ticks=1)
    assert not result['completed']
    assert result['audit']['stationary'] is False
    assert result['audit']['post_stop_pose_pair_fresh'] is False
