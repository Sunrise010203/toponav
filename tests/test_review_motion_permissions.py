"""Explicit permission propagation; tests never launch Gazebo or a robot."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
from http.server import ThreadingHTTPServer
from unittest.mock import Mock, patch
import urllib.request

import numpy as np
from PIL import Image
import pytest

from simulation.gazebo_scenes import make_controller
from tools.visualize_simulation import Simulation, make_handler

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('scenario', ['straight', 'blocked'])
@pytest.mark.parametrize('permission', ['no_motion', 'bounded_probe'])
def test_gazebo_factory_preserves_explicit_straight_permission(scenario, permission):
    c = make_controller(scenario, True, motion_permission=permission)
    assert c.motion_permission == permission
    assert make_controller(scenario, True).motion_permission == 'no_motion'


@pytest.mark.parametrize('scenario', ['left', 't-right', 'route', 'vision', 'topology'])
def test_probe_permission_cannot_arm_other_gazebo_modes(scenario):
    with pytest.raises(ValueError, match='only for straight'):
        make_controller(scenario, True, motion_permission='bounded_probe')


@pytest.mark.parametrize('scenario', ['straight', 'blocked'])
@pytest.mark.parametrize('permission', ['no_motion', 'bounded_probe'])
def test_browser_worker_passes_permission_without_implicitly_upgrading_it(tmp_path, scenario, permission):
    sim = Simulation(tmp_path)
    proc = Mock()
    proc.poll.return_value = 0
    with patch('tools.visualize_simulation.subprocess.Popen', return_value=proc) as spawn:
        sim.start(scenario, True, permission)
        args = spawn.call_args.args[0]
        assert args[args.index('--motion-permission')+1] == permission
        assert '--confirm-initial-clearance' in args
    sim.close()


def test_browser_clearance_confirmation_alone_never_grants_probe_permission(tmp_path):
    sim = Simulation(tmp_path)
    proc = Mock()
    proc.poll.return_value = 0
    with patch('tools.visualize_simulation.subprocess.Popen', return_value=proc) as spawn:
        sim.start('straight', True)
        args = spawn.call_args.args[0]
        assert args[args.index('--motion-permission')+1] == 'no_motion'
    sim.close()


@pytest.mark.parametrize('preset,confirmed,permission', [
    ('straight', False, 'bounded_probe'), ('left', True, 'bounded_probe'),
    ('straight', True, None), ('straight', True, True), ('straight', True, 'drive'),
])
def test_browser_rejects_invalid_authorization_before_creating_a_worker(tmp_path, preset, confirmed, permission):
    sim = Simulation(tmp_path)
    with pytest.raises(ValueError):
        sim.start(preset, confirmed, permission)
    assert sim.process is None and list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('scenario', ['straight', 'blocked'])
@pytest.mark.parametrize('permission', ['no_motion', 'bounded_probe'])
def test_gazebo_cli_passes_explicit_permission(monkeypatch, tmp_path, scenario, permission):
    from simulation import gazebo_cross_validation as module
    calls = []
    def run(*args, **kwargs):
        calls.append((args, kwargs))
        return {'completed': True, 'preview_loaded': False}
    monkeypatch.setattr(module, 'run', run)
    assert module.main(['--output', str(tmp_path/'unused'), '--scenario', scenario,
                        '--confirm-initial-clearance', '--motion-permission', permission]) == 0
    assert calls[0][0][3] is True
    assert calls[0][1]['motion_permission'] == permission


def test_replay_uses_and_checks_the_same_public_motion_permission(tmp_path):
    from simulation.gazebo_replay import replay
    public = tmp_path/'public'
    frames = public/'frames'
    frames.mkdir(parents=True)
    rgb = np.random.default_rng(123).integers(0, 255, (256, 256, 3), dtype=np.uint8)
    Image.fromarray(rgb).save(frames/'0000.png')
    frame = dict(frame_id='f1', captured_sim_s=.1, rgb=rgb)
    controller = make_controller('straight', True, motion_permission='bounded_probe')
    decision = controller.update(frame, .1)
    row = dict(tick=0, sim_s=.1, input=dict(frame_id='f1', captured_sim_s=.1,
               rgb_path='frames/0000.png', rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest()),
               decision=decision)
    (tmp_path/'decisions.jsonl').write_text(json.dumps(row)+'\n')
    task = dict(scenario='straight', motion_permission='bounded_probe', initial_clearance_confirmed=True)
    task_path = public/'motion-task.json'
    task_path.write_text(json.dumps(task))
    frozen = {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
              for name in ('roadvision/continuous_straight.py', 'simulation/gazebo_scenes.py')}
    frozen['public/motion-task.json'] = hashlib.sha256(task_path.read_bytes()).hexdigest()
    (tmp_path/'freeze.json').write_text(json.dumps(frozen))
    assert replay(tmp_path, 'left', True, 'straight', motion_permission='bounded_probe')['all_decisions_match']
    with pytest.raises(ValueError, match='replay authorization'):
        replay(tmp_path, 'left', True, 'straight')
    task_path.write_text(json.dumps(dict(task, motion_permission='no_motion')))
    with pytest.raises(ValueError, match='task hash'):
        replay(tmp_path, 'left', True, 'straight', motion_permission='bounded_probe')


def test_browser_api_propagates_explicit_permission_to_the_worker(tmp_path):
    sim = Simulation(tmp_path)
    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(sim))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    proc = Mock()
    proc.poll.return_value = 0
    try:
        with patch('tools.visualize_simulation.subprocess.Popen', return_value=proc) as spawn:
            request = urllib.request.Request(f'http://127.0.0.1:{server.server_port}/api/run',
                data=json.dumps(dict(preset='straight', confirmed=True, motion_permission='bounded_probe')).encode(),
                headers={'Content-Type': 'application/json'})
            with urllib.request.urlopen(request, timeout=5) as response:
                assert response.status == 201
            args = spawn.call_args.args[0]
            assert args[args.index('--motion-permission')+1] == 'bounded_probe'
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
        sim.close()


def test_native_chooser_requires_separate_probe_opt_in_and_clears_it_on_scene_change():
    # Exercise real Qt widgets and the exact QProcess arguments, without
    # starting Gazebo or opening a desktop window.
    code = '''
import tempfile
from unittest.mock import patch
from PyQt5.QtWidgets import QApplication
from tools.gazebo_scenarios_app import ScenarioWindow
app=QApplication([])
with tempfile.TemporaryDirectory() as directory:
 w=ScenarioWindow(directory,auto_open=False)
 assert w.probe.isHidden() and not w.probe.isChecked()
 w.scenario.setCurrentIndex(w.scenario.findData('straight'))
 assert not w.probe.isHidden() and w.probe.isEnabled() and not w.probe.isChecked()
 w.confirmed.setChecked(True)
 with patch.object(w.worker,'start') as spawn:
  w.launch()
  args=spawn.call_args.args[1]
  assert args[args.index('--motion-permission')+1]=='no_motion'
 w.refresh()
 w.probe.setChecked(True)
 with patch.object(w.worker,'start') as spawn:
  w.launch()
  args=spawn.call_args.args[1]
  assert args[args.index('--motion-permission')+1]=='bounded_probe'
 w.scenario.setCurrentIndex(w.scenario.findData('blocked'))
 assert not w.probe.isChecked()
 w.scenario.setCurrentIndex(w.scenario.findData('left'))
 assert w.probe.isHidden() and not w.probe.isChecked()
 w.close()
'''
    result = subprocess.run([sys.executable, '-c', code], cwd=ROOT,
        env=dict(os.environ, QT_QPA_PLATFORM='offscreen', PYTHONDONTWRITEBYTECODE='1'),
        capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout+result.stderr
