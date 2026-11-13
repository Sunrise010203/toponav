"""Runner fault tests use fake transport/physics, but the real MotorGate."""
import hashlib
import json
import threading
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
from PIL import Image
import pytest

from simulation import gazebo_cross_validation as runner
from simulation import gazebo_scenes


@pytest.fixture
def harness(monkeypatch):
    clock = NS(wall=10.)
    monkeypatch.setattr(runner, 'time', NS(monotonic=lambda: clock.wall, sleep=lambda _: None))
    monkeypatch.setenv('DISPLAY', ':test-owned-fake')
    control = NS(state='observing', calls=0, cancelled=False)
    fault = NS(kind=None)
    calls = []
    def update(frame, now):
        control.calls += 1
        if fault.kind == 'perception' and control.calls == 2: raise RuntimeError('backend failed')
        control.state = 'moving'
        return dict(state='moving', reason='synthetic_motion', v_m_s=.12, omega_rad_s=0.,
                    control_frame_id=frame['frame_id'], captured_sim_s=frame['captured_sim_s'], decision_sim_s=now)
    def cancel(now):
        calls.append(('cancel', now))
        if control.state not in ('visual_stop', 'aborted'): control.state = 'aborted'
        control.cancelled = True
    control.update = update; control.cancel = cancel
    control.result = lambda: dict(state=control.state, reason='test', motion_commands=control.calls)
    monkeypatch.setattr(gazebo_scenes, 'make_controller', lambda *a, **kw: control)
    monkeypatch.setattr(gazebo_scenes, 'write_display', lambda *a: (_ for _ in ()).throw(RuntimeError('display failed')) if fault.kind == 'display' else None)
    process = Mock(pid=123456)
    process.poll.return_value = None
    def spawn(*args, **kwargs):
        kwargs['stdout'].write('gz::physics::dartsim::Plugin\n')
        return process
    monkeypatch.setattr(runner.subprocess, 'Popen', spawn)
    monkeypatch.setattr(runner.os, 'killpg', lambda *a: None)
    class IO:
        def __init__(self, **kw):
            self.condition = threading.Condition()
            self.now_ns = 0; self.clock_messages = 1; self.steps = 0
            self.cameras = NS(views={v: {0: True} for v in runner.VIEWS})
            self.node = NS(service_list=lambda: ['/world/toponav/control'])
            self.overview = None; self.poses = []; self.contacts = []; self.contact_messages = {}
            self.stepper = Mock()
        def step(self, iterations):
            self.steps += 1
            if self.steps == 6:
                if fault.kind == 'lease': clock.wall += 1.
                if fault.kind == 'step': raise RuntimeError('step failed')
            self.now_ns += 100_000_000
        def next_frame(self, previous):
            if fault.kind == 'camera' and control.calls: raise TimeoutError('camera stopped')
            rgb = np.random.default_rng(self.now_ns).integers(0, 255, (256, 256, 3), np.uint8)
            return self.now_ns, rgb, self.now_ns / 1e9
        def command(self, v, omega):
            calls.append(('command', v, omega))
            if fault.kind == 'publish' and control.calls == 2 and v: raise RuntimeError('publish failed')
    monkeypatch.setattr(runner, 'GazeboIO', IO)
    return control, fault, calls


@pytest.mark.parametrize('fault_kind', ['lease', 'perception', 'publish', 'camera', 'display', 'step'])
def test_runner_failure_cancels_active_controller_and_preserves_evidence(tmp_path, harness, fault_kind):
    control, fault, calls = harness; fault.kind = fault_kind
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=3)
    assert result['exception'] and not result['completed']
    assert result['controller_result']['state'] == 'aborted'
    assert control.cancelled
    failure = json.loads((tmp_path/'capture/audit/failure.json').read_text())
    assert failure['zero_command_submitted']
    assert failure['controller_terminal']
    cancel_index = next(i for i, c in enumerate(calls) if c[0] == 'cancel')
    assert calls[cancel_index - 1] == ('command', 0., 0.)
    if fault_kind in ('lease', 'perception', 'publish'):
        reference = failure['pending_tick']['input']
        pixels = np.array(Image.open(tmp_path/'capture/public'/reference['rgb_path']))
        assert hashlib.sha256(pixels.tobytes()).hexdigest() == reference['rgb_sha256']
        assert reference['frame_id'].endswith('/600000000')
    if fault_kind == 'lease':
        pending = failure['pending_tick']
        assert pending['decision']['v_m_s'] == .12
        assert pending['command']['v_m_s'] == .12
        assert pending['actuator_gate']['reason'] == 'command_lease_expired'
        assert pending['actuator_gate']['wheels'] == [0., 0.]


def test_failure_after_committed_tick_does_not_rewrite_successful_prefix(tmp_path, harness):
    control, fault, _ = harness; fault.kind = 'display'
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=3)
    rows = [json.loads(l) for l in (tmp_path/'capture/decisions.jsonl').read_text().splitlines()]
    assert len(rows) == 1 and rows[0]['decision']['state'] == 'moving'
    assert result['failure_record']['pending_tick'] is None
    assert result['failure_record']['committed_decision_count'] == 1


def test_zero_submission_failure_is_not_reported_as_confirmed_stop(tmp_path, harness):
    _, fault, _ = harness; fault.kind = 'perception'
    original = runner.GazeboIO.command
    def command(self, v, omega):
        if v == omega == 0: raise RuntimeError('zero transport unavailable')
        return original(self, v, omega)
    runner.GazeboIO.command = command
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=3)
    failure = result['failure_record']
    assert not failure['zero_command_submitted']
    assert 'zero transport unavailable' in failure['zero_command_error']
    assert 'backend failed' in result['exception']
    assert result['controller_result']['state'] == 'aborted'
