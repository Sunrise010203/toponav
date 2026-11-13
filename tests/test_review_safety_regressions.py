"""Review regressions: real controller/bridge methods, no DDS or physics claim."""
import ast
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from PIL import Image
import pytest

from roadvision import continuous_straight as control
from roadvision.contract import to_observation
from ros2_ws.src.toponav_bridge.toponav_bridge.protocol import (
    VERSION, MotorGate, command_from_decision, decode, encode,
)

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('age', [float('nan'), float('inf'), -float('inf'),
                                     None, True, False, '0', -1., 1.000001])
def test_invalid_frame_age_never_yields_an_actionable_observation(age):
    probabilities = [0.] * 17
    probabilities[8] = 1.
    out = to_observation(probabilities, age_s=age)
    assert out['confidence'] == 0.
    assert all(v is None for v in out['relative_open'].values())


@pytest.mark.parametrize('age', [0, 0., .25, 1.])
def test_valid_frame_age_boundaries_are_preserved(age):
    out = to_observation([1.] + [0.] * 16, age_s=age)
    assert out['confidence'] == 1.


@pytest.mark.parametrize('kind', ['straight', 'corner', 'junction', 'route'])
@pytest.mark.parametrize('state', ['visual_stop', 'aborted'])
@pytest.mark.parametrize('bad', [None, {}, {'unexpected': 'field'}])
def test_all_terminal_controllers_ignore_malformed_packets(monkeypatch, kind, state, bad):
    from test_continuous_straight import route_unit
    if kind == 'route':
        c, _, _ = route_unit(monkeypatch)
    elif kind == 'straight':
        c = control.StraightController()
    else:
        c = control.CornerController(direction='left', junction='t' if kind == 'junction' else 'l')
    c.state = state
    c.reason = 'latched_result'
    before = c.result()
    d = c.update(bad, float('nan'))
    assert d['v_m_s'] == d['omega_rad_s'] == 0.
    assert c.result() == before


@pytest.fixture
def bridge(tmp_path, monkeypatch):
    """Load unmodified Navigation methods with ROS transport constructors stubbed.

    This exercises the real exception handler, emit order and MotorGate, not
    a copied safety implementation. It is not a ROS/DDS integration test.
    """
    from test_continuous_straight import route_unit
    source = ROOT / 'ros2_ws/src/toponav_bridge/toponav_bridge/nodes.py'
    tree = ast.parse(source.read_text())
    selected = [n for n in tree.body if
                isinstance(n, ast.FunctionDef) and n.name in ('pub', 'append', 'save', 'seconds') or
                isinstance(n, ast.ClassDef) and n.name == 'Navigation']
    ns = dict(Node=object, String=SimpleNamespace, np=np, PNG=Image, Path=Path,
              hashlib=hashlib, json=json, time=time, VERSION=VERSION,
              command_from_decision=command_from_decision, encode=encode)
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(source), 'exec'), ns)
    nav = ns['Navigation'].__new__(ns['Navigation'])
    nav.controller, _, _ = route_unit(monkeypatch)
    nav.session = nav.controller.session_id
    nav.out = tmp_path
    (tmp_path / 'frames').mkdir()
    nav.seq = 1
    nav.image_count = 0
    nav.now = 1.
    nav.waiting = nav.wait_started = nav.bridge_reason = None
    nav.abort_sent = False
    nav.commands, nav.decisions, nav.processed = Mock(), Mock(), Mock()
    rgb = np.random.default_rng(123).integers(0, 255, (256, 256, 3), dtype=np.uint8)
    nav.queued = SimpleNamespace(encoding='rgb8', width=256, height=256, step=768,
        is_bigendian=0, data=rgb.tobytes(), header=SimpleNamespace(
            frame_id=nav.session+'/f1', stamp=SimpleNamespace(sec=1, nanosec=0)))
    return nav, ns


def test_ros_perception_exception_sends_zero_before_the_lease_expires(bridge, monkeypatch):
    nav, _ = bridge
    gate = MotorGate(nav.session)
    gate.arm('previous', .9, 10.)
    previous = dict(control_frame_id='previous', captured_sim_s=.9, decision_sim_s=.9,
                    v_m_s=.15, omega_rad_s=0., state='moving')
    assert gate.accept(command_from_decision(nav.session, 1, previous), .9, 10.) == (.15, 0.)
    gate.arm(nav.queued.header.frame_id, 1., 10.05)
    nav.commands.publish.side_effect = lambda msg: gate.accept(decode(msg.data), 1., 10.1)

    def fail(*args, **kwargs):
        raise RuntimeError('perception failure')
    monkeypatch.setattr(control, 'analyze_corner_rgb', fail)
    nav.drain()
    assert nav.controller.state == 'aborted'
    assert nav.commands.publish.call_count == 1
    command = decode(nav.commands.publish.call_args.args[0].data)
    assert command['state'] == 'aborted'
    assert command['v_m_s'] == command['omega_rad_s'] == 0.
    assert gate.latched and gate.reason == 'remote_abort' and gate.wheels == (0., 0.)
    assert nav.bridge_reason == 'image_or_perception_error:RuntimeError'
    assert nav.waiting is nav.queued is None
    nav.abort('duplicate_transport_error')
    assert nav.commands.publish.call_count == 1


@pytest.mark.parametrize('writer', ['append', 'save'])
@pytest.mark.parametrize('state', ['aborted', 'visual_stop', 'observing'])
def test_zero_commands_precede_even_failing_evidence_writes(bridge, writer, state):
    nav, ns = bridge
    def fail(*args, **kwargs):
        raise OSError('disk unavailable')
    ns[writer] = fail
    decision = dict(control_frame_id='f1', captured_sim_s=1., decision_sim_s=1.,
                    v_m_s=0., omega_rad_s=0., state=state)
    with pytest.raises(OSError):
        nav.emit(decision, 'test_zero')
    assert nav.commands.publish.call_count == 1
    command = decode(nav.commands.publish.call_args.args[0].data)
    assert command['v_m_s'] == command['omega_rad_s'] == 0.


def test_motion_is_not_published_before_its_evidence_write(bridge):
    nav, ns = bridge
    def fail(*args, **kwargs):
        raise OSError('disk unavailable')
    ns['append'] = fail
    decision = dict(control_frame_id='f1', captured_sim_s=1., decision_sim_s=1.,
                    v_m_s=.15, omega_rad_s=0., state='moving')
    with pytest.raises(OSError):
        nav.emit(decision, 'test_move')
    nav.commands.publish.assert_not_called()
