"""Evaluation-infrastructure tests: use only the old/default rendering anchor."""
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from simulation import corridor_motion as motion
from simulation.corridor_validation import (
    ValidationWorld, classify_run, load_suite, run_case, validate_case, verify_lock,
)


def anchor():
    return {"id": "infra_anchor", "group": "anchor", "scene": {}, "appearance": {}}


def test_default_adapter_preserves_physics_pixels_and_frame_protocol():
    with motion.CorridorWorld() as original, ValidationWorld() as adapter:
        for v, omega in ((0, 0), (.25, .10), (.25, -.10), (.20, 0)):
            original.drive(v, omega)
            adapter.drive(v, omega)
            original.step_control()
            adapter.step_control()
            a, b = original.capture(), adapter.capture()
            assert set(b) == {"frame_id", "captured_sim_s", "rgb"}
            assert a["frame_id"] == b["frame_id"] and a["captured_sim_s"] == b["captured_sim_s"]
            assert np.array_equal(a["rgb"], b["rgb"])
            assert original.audit_state() == adapter.audit_state()


@pytest.mark.parametrize("change", [
    {"floor_rgb": [-1, 0, 0]}, {"wall_rgb": [0, float("nan"), 0]}, {"floor_rgb": [0, 0]},
    {"lighting": {"unknown_light": 1}}, {"lighting": {"lightDirection": [0, 0, 0]}},
    {"lighting": {"lightAmbientCoeff": 2}}, {"lighting": {"shadow": True}},
    {"barrier_front_m": 4}, {"true_pose": [1, 2, 3]},
    {"paint": {"x_start": 3, "x_end": 2, "y_center": 0, "width_m": 1, "rgb": [0, 0, 0]}},
])
def test_bad_scene_appearance_rejected_before_running_physics(change):
    case = anchor()
    case["appearance"] = change
    with pytest.raises(ValueError):
        validate_case(case)


def test_validation_copies_input_and_forbids_control_overrides():
    case = anchor()
    case["appearance"] = {"floor_rgb": [.2, .29, .4]}
    normalized = validate_case(case)
    normalized["appearance"]["floor_rgb"][0] = 0
    assert case["appearance"]["floor_rgb"][0] == .2
    case["vision_config"] = {"max_speed_m_s": .1}
    with pytest.raises(ValueError):
        validate_case(case)


def test_suite_rejects_duplicate_ids_and_wrong_denominators(tmp_path):
    data = {"version": "corridor_validation_v1", "scope": "test", "case_count": 2,
            "novel_case_count": 0, "cases": [anchor(), anchor()]}
    path = tmp_path / "suite.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_suite(path)
    data["cases"] = [anchor()]
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_suite(path)


def record(state="visual_stop", y=0, contact=0, speed=0, distance=1.3):
    return {"state": state, "audit": {"camera_to_end_wall_m": distance,
            "wall_contact_physics_steps": contact,
            "final": {"position_m": [4.5, y, .16], "camera_position_m": [6-distance, y, .45],
                      "yaw_deg": 0, "speed_m_s": speed, "yaw_rate_rad_s": 0}},
            "scene_config": {"length_m": 6}}


@pytest.mark.parametrize("state,y,contact,speed,distance,outcome", [
    ("visual_stop", 0, 0, 0, 1.3, "end_wall_verified"),
    ("aborted", 0, 0, 0, 5, "safe_rejection"),
    ("visual_stop", 0, 0, 0, 4, "visual_stop_unverified"),
    ("visual_stop", .3, 0, 0, 1.3, "visual_stop_unverified"),
    ("visual_stop", 0, 1, 0, 1.3, "unsafe"),
    ("aborted", 0, 0, .1, 5, "unsafe"),
])
def test_outcomes_do_not_conflate_refusal_or_false_stop_with_success(state, y, contact, speed, distance, outcome):
    original = record(state, y, contact, speed, distance)
    before = copy.deepcopy(original)
    assert classify_run(original)["outcome"] == outcome
    assert original == before


def make_lock():
    root = Path(__file__).resolve().parents[1]
    names = ("roadvision/corridor_control.py", "simulation/corridor_motion.py", "simulation/corridor_validation.py")
    return {"files": {name: hashlib.sha256((root/name).read_bytes()).hexdigest() for name in names},
            "vision_config": {}}


def test_freeze_lock_detects_changed_file(tmp_path):
    path = tmp_path / "lock.json"
    lock = make_lock()
    lock["files"]["roadvision/corridor_control.py"] = "0" * 64
    path.write_text(json.dumps(lock))
    with pytest.raises(ValueError, match="Frozen file changed"):
        verify_lock(path)


def test_freeze_lock_detects_policy_change_independently(tmp_path):
    path = tmp_path / "lock.json"
    path.write_text(json.dumps(make_lock()))
    with pytest.raises(ValueError, match="policy defaults"):
        verify_lock(path)


def test_factory_substitution_is_scoped_and_uses_original_defaults(tmp_path, monkeypatch):
    original = motion.CorridorWorld
    def fake_run(config, output_dir):
        assert config == motion.CorridorConfig()
        with motion.CorridorWorld(config) as world:
            assert isinstance(world, ValidationWorld)
            assert set(world.capture()) == {"frame_id", "captured_sim_s", "rgb"}
        Path(output_dir).mkdir()
        return {"marker": "unchanged loop API"}
    monkeypatch.setattr(motion, "run_corridor", fake_run)
    assert run_case(validate_case(anchor()), tmp_path/"case")["marker"] == "unchanged loop API"
    assert motion.CorridorWorld is original


def test_factory_is_restored_after_execution_error(tmp_path, monkeypatch):
    original = motion.CorridorWorld
    def failing_run(*args, **kwargs):
        raise RuntimeError("infrastructure failure")
    monkeypatch.setattr(motion, "run_corridor", failing_run)
    with pytest.raises(RuntimeError):
        run_case(validate_case(anchor()), tmp_path/"case")
    assert motion.CorridorWorld is original
