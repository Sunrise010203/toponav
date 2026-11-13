import ast
import hashlib
import json
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.gazebo_cross_validation import CameraBuffer, GazeboIO, VIEWS, build_world


@pytest.mark.parametrize("direction", ["left", "right"])
def test_world_has_native_physics_wheels_and_four_calibrated_cameras(direction):
    world = ET.fromstring(build_world(direction)).find("world")
    assert world.attrib["name"] == "toponav"
    assert float(world.find("physics/max_step_size").text) == pytest.approx(1/120)
    robot = world.find("model[@name='robot']")
    assert robot.find("pose").text == ".5 0 .16 0 0 0"
    for wheel in ("left_wheel", "right_wheel"):
        assert robot.find(f"joint[@name='{wheel}_joint']").attrib["type"] == "revolute"
    cameras = robot.findall("link[@name='base']/sensor[@type='camera']")
    assert [c.attrib["name"] for c in cameras] == list(VIEWS)
    assert all(c.find("camera/image/width").text == "128" for c in cameras)
    assert robot.find("plugin[@name='gz::sim::systems::DiffDrive']/wheel_radius").text == "0.08"
    assert len(robot.findall("link/sensor[@type='contact']")) == 5


def test_four_camera_frames_are_never_retimed_or_mixed():
    buffer = CameraBuffer()
    for index, view in enumerate(VIEWS):
        buffer.put(view, 10, np.full((128,128,3), index, np.uint8))
    buffer.put("front", 20, np.zeros((128,128,3), np.uint8))
    timestamp, rgb = buffer.latest()
    assert timestamp == 10
    assert [rgb[y,x,0] for y,x in ((0,0),(0,128),(128,0),(128,128))] == [0,1,2,3]
    assert buffer.latest(after=10) is None
    assert buffer.latest(before=9) is None


def test_camera_history_is_bounded():
    buffer = CameraBuffer()
    for t in range(20):
        buffer.put("front", t, np.zeros((128,128,3),np.uint8))
    assert len(buffer.views["front"]) == 8


def test_invalid_direction_rejected():
    with pytest.raises(ValueError):
        build_world("straight")


def test_adapter_does_not_use_pybullet_or_pose_steering():
    path = Path(__file__).resolve().parents[1]/"simulation/gazebo_cross_validation.py"
    source = path.read_text()
    tree = ast.parse(source)
    imports = [n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
    assert not any(i and (i.startswith("pybullet") or i.startswith("simulation.continuous_audit_world")) for i in imports)
    assert "set_pose" not in source and "resetBasePosition" not in source
    assert 'controller.update(frame, now)' in source


def test_pose_timestamp_uses_individual_pose_header_not_empty_envelope():
    io = GazeboIO.__new__(GazeboIO)
    io.condition, io.poses = threading.Condition(), []
    p = NS(name="robot", header=NS(stamp=NS(sec=2, nsec=500_000_000)),
           position=NS(x=1., y=2., z=.16), orientation=NS(w=1., x=0., y=0., z=0.))
    io.pose(NS(header=NS(stamp=NS(sec=0, nsec=0)), pose=[p]))
    assert io.poses == [dict(sim_s=2.5, position_m=[1., 2., .16], yaw_deg=0.)]


def test_clock_consumes_sim_not_wall_time_and_never_goes_backwards():
    io = GazeboIO.__new__(GazeboIO)
    io.condition, io.now_ns, io.clock_messages = threading.Condition(), 0, 0
    io.clock(NS(sim=NS(sec=3, nsec=42), real=NS(sec=999, nsec=0)))
    io.clock(NS(sim=NS(sec=2, nsec=0)))
    assert io.now_ns == 3_000_000_042
    assert io.clock_messages == 2


def test_command_limits_reject_before_publishing():
    io = GazeboIO.__new__(GazeboIO)
    for v, omega in ((-.1, 0), (.151, 0), (0, .301), (float("nan"), 0)):
        with pytest.raises(ValueError, match="limits"):
            io.command(v, omega)


def test_public_replay_matches_and_detects_tampered_pixels(tmp_path):
    from PIL import Image
    from roadvision.continuous_straight import CornerController, InitialClearancePrior
    from simulation.gazebo_replay import replay
    root = Path(__file__).resolve().parents[1]
    controller_hash = hashlib.sha256((root / "roadvision/continuous_straight.py").read_bytes()).hexdigest()
    (tmp_path / "freeze.json").write_text(json.dumps({"roadvision/continuous_straight.py": controller_hash}))
    frames = tmp_path / "public/frames"
    frames.mkdir(parents=True)
    rgb = np.zeros((256,256,3), np.uint8)
    Image.fromarray(rgb).save(frames / "0000.png")
    frame = dict(frame_id="test/100", captured_sim_s=.1, rgb=rgb)
    d = CornerController(direction="left", initial_clearance=InitialClearancePrior(confirmed=False)).update(frame, .1)
    row = dict(tick=0, sim_s=.1, input=dict(frame_id="test/100", captured_sim_s=.1,
        rgb_path="frames/0000.png", rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest()), decision=d)
    (tmp_path / "decisions.jsonl").write_text(json.dumps(row)+"\n")
    assert replay(tmp_path, "left")["all_decisions_match"]
    rgb[0,0] = 255
    Image.fromarray(rgb).save(frames / "0000.png")
    with pytest.raises(ValueError, match="RGB hash mismatch"):
        replay(tmp_path, "left")
