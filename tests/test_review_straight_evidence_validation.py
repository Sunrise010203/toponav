"""Synthetic perception faults test zero/latch contracts, not RGB accuracy."""
import copy
from pathlib import Path

import numpy as np
from PIL import Image
import pytest

from roadvision import continuous_straight as control
from roadvision.calibrated_ground_motion import GroundCalibration
from roadvision.straight_permission import assess_startup_texture
from test_straight_task2_permissions import setup, frame


INVALID_MOTIONS = [
    {"translation_m": [float("nan"), 0.]},
    {"translation_m": [0., float("inf")]},
    {"translation_m": [float("-inf"), 0.]},
    {"translation_m": [None, 0.]},
    {"translation_m": [True, 0.]},
    {"translation_m": [False, 0.]},
    {"translation_m": ["0", 0.]},
    {"translation_m": []},
    {"translation_m": [0.]},
    {"translation_m": [0., 0., 0.]},
    {"translation_m": [[0.], [0.]]},
    {"translation_m": None},
    {"yaw_delta_rad": float("nan")},
    {"yaw_delta_rad": float("inf")},
    {"yaw_delta_rad": float("-inf")},
    {"yaw_delta_rad": None},
    {"yaw_delta_rad": True},
    {"yaw_delta_rad": False},
    {"yaw_delta_rad": "0"},
    {"yaw_delta_rad": [0.]},
]


def motion(changes):
    return {"status": "conditional_ground_motion", "translation_m": [0., 0.],
            "yaw_delta_rad": 0., **copy.deepcopy(changes)}


def assert_zero(decision):
    assert decision["v_m_s"] == decision["omega_rad_s"] == 0.


@pytest.mark.parametrize("changes", INVALID_MOTIONS)
def test_invalid_conditional_startup_motion_cannot_authorize_probe(monkeypatch, changes):
    c, _ = setup(monkeypatch)
    monkeypatch.setattr(c, "_local_stationary_motion", lambda *_: motion(changes))
    for i in range(3):
        decision = c.update(frame(i), i * .1)
        assert_zero(decision)
    assert decision["state"] == "aborted" and decision["reason"] == "invalid_initial_ground_motion"
    assert c.motion_commands == 0 and c.probe_started is None
    terminal = copy.deepcopy(c.result())
    assert_zero(c.update(None, float("nan")))
    assert c.result() == terminal


@pytest.mark.parametrize("changes", [
    {"translation_m": [.01, 0.]}, {"translation_m": [0., -.01]},
    {"yaw_delta_rad": .02}, {"yaw_delta_rad": -.02}, {},
])
def test_original_valid_stationary_boundaries_still_allow_explicit_probe(monkeypatch, changes):
    c, _ = setup(monkeypatch)
    monkeypatch.setattr(c, "_local_stationary_motion", lambda *_: motion(changes))
    for i in range(3):
        decision = c.update(frame(i), i * .1)
    assert decision["state"] == "bootstrapping" and decision["v_m_s"] > 0


@pytest.mark.parametrize("changes", [{"translation_m": [.01001, 0.]}, {"yaw_delta_rad": -.02001}])
def test_original_moving_startup_rejection_is_unchanged(monkeypatch, changes):
    c, _ = setup(monkeypatch)
    monkeypatch.setattr(c, "_local_stationary_motion", lambda *_: motion(changes))
    for i in range(3):
        decision = c.update(frame(i), i * .1)
    assert_zero(decision)
    assert decision["reason"] == "initial_vehicle_not_visually_stationary"


INVALID_INTERVALS = [
    None, [], [1.], [1., 2., 3.], [float("nan"), 1.5], [1.4, float("nan")],
    [float("inf"), float("inf")], [1.4, float("inf")], [float("-inf"), 1.5],
    [True, 1.5], [False, 1.5], [1., True], [-.1, 1.5], [1.5, 1.4],
    [None, 1.5], [1.4, None], ["1.4", 1.5], [1.4, "1.5"], [[1.4], [1.5]],
    {"lower": 1.4, "upper": 1.5},
]


def replace_front(monkeypatch, front):
    original = control.probe_straight_history

    def probe(*args, **kwargs):
        packet = original(*args, **kwargs)
        packet["front_surface"] = copy.deepcopy(front)
        return packet

    monkeypatch.setattr(control, "probe_straight_history", probe)


@pytest.mark.parametrize("interval", INVALID_INTERVALS)
def test_invalid_supported_front_interval_stops_after_partial_motion(monkeypatch, interval):
    c, _ = setup(monkeypatch)
    for i in range(9):
        decision = c.update(frame(i), i * .1)
    assert decision["state"] == "following" and decision["v_m_s"] > 0
    commands = c.motion_commands
    replace_front(monkeypatch, {"status": "conditional_front_surface", "range_interval_m": interval})
    decision = c.update(frame(9), .9)
    assert_zero(decision)
    assert decision["state"] == "aborted" and decision["reason"] == "invalid_front_surface_interval"
    assert c.motion_commands == commands
    assert c.result()["result_type"] == "aborted_motion_uncertain"
    terminal = copy.deepcopy(c.result())
    assert_zero(c.update(None, float("nan")))
    assert c.result() == terminal


def test_missing_supported_front_interval_is_rejected_without_exception(monkeypatch):
    c, _ = setup(monkeypatch)
    replace_front(monkeypatch, {"status": "conditional_front_surface"})
    for i in range(9):
        decision = c.update(frame(i), i * .1)
    assert_zero(decision)
    assert decision["reason"] == "invalid_front_surface_interval"


@pytest.mark.parametrize("interval,expected", [
    ([0., 0.], "braking"), ([1.3, 1.4], "braking"),
    ([1.30001, 1.4], "following"), ((1.9, 2.), "following"),
])
def test_valid_front_interval_keeps_existing_stop_and_slowdown_boundaries(monkeypatch, interval, expected):
    c, _ = setup(monkeypatch)
    replace_front(monkeypatch, {"status": "conditional_front_surface", "range_interval_m": interval})
    for i in range(9):
        decision = c.update(frame(i), i * .1)
    assert decision["state"] == expected
    if expected == "braking":
        assert_zero(decision)
    else:
        assert 0 < decision["v_m_s"] <= c.policy.max_speed_m_s


@pytest.mark.parametrize("changes", INVALID_MOTIONS)
def test_invalid_stop_motion_cannot_prove_stationarity_or_rearm(monkeypatch, changes):
    c, fits = setup(monkeypatch)
    pixels = frame(0)["rgb"]

    def step(i):
        pixels[255, 255, 0] = i
        return c.update({"frame_id": f"stop-{i}", "captured_sim_s": i * .1, "rgb": pixels}, i * .1)

    for i in range(9):
        step(i)
    fits["good"] = False
    assert_zero(step(9))
    fits["good"] = True
    monkeypatch.setattr(c, "_local_stationary_motion", lambda *_: motion(changes))
    for i in range(10, 21):
        decision = step(i)
        assert_zero(decision)
        assert decision["stationary_frames"] == 0
    assert decision["state"] == "aborted" and "recovery_timeout" in decision["reason"]


@pytest.mark.parametrize("changes", INVALID_MOTIONS[:3] + INVALID_MOTIONS[12:15])
def test_invalid_stop_motion_cannot_report_completed_visual_stop(monkeypatch, changes):
    c, _ = setup(monkeypatch)
    for i in range(9):
        c.update(frame(i), i * .1)
    replace_front(monkeypatch, {"status": "conditional_front_surface", "range_interval_m": [1.2, 1.3]})
    assert c.update(frame(9), .9)["state"] == "braking"
    monkeypatch.setattr(c, "_local_stationary_motion", lambda *_: motion(changes))
    pixels = frame(9)["rgb"]
    for i in range(10, 19):
        decision = c.update({"frame_id": f"brake-{i}", "captured_sim_s": i * .1, "rgb": pixels}, i * .1)
        assert_zero(decision)
        assert decision["state"] != "visual_stop" and decision["stationary_frames"] == 0


@pytest.mark.parametrize("name", ["center_a", "repeated", "floor_paint"])
def test_real_exposed_fixture_reports_exact_startup_failed_checks_without_new_permission(name):
    path = Path(__file__).parent / "fixtures/straight_startup" / f"{name}.png"
    result = assess_startup_texture(np.array(Image.open(path)), GroundCalibration())
    cells = result["cells"]
    native = sum(c["reference_accepted"] for c in cells)
    good = [c for c in cells if c["rectified_accepted"]]
    checks = {
        "native_reference_cells_unique": native == 9,
        "rectified_supported_cell_count": len(good) >= 6,
        "forward_bin_coverage": len({c["nominal_point_latest_camera_xy_m"][0] for c in good}) == 3,
        "lateral_bin_coverage": len({c["nominal_point_latest_camera_xy_m"][1] for c in good}) == 3,
    }
    assert result["checks"] == checks
    assert result["failed_checks"] == [key for key, accepted in checks.items() if not accepted]
    assert result["reference_supported_cells"] == native
    assert result["accepted"] is all(checks.values())
    assert result["motion_authorized"] is result["certified_clearance"] is False


def test_startup_records_first_failed_frame_even_when_latest_frame_passes(monkeypatch):
    c, _ = setup(monkeypatch)
    checks = iter([
        {"accepted": False, "reason": "reference_texture_ambiguous_or_unsupported",
         "failed_checks": ["rectified_supported_cell_count"], "reference_supported_cells": 9,
         "rectified_supported_cells": 5},
        {"accepted": True, "reason": "reference_texture_supported", "failed_checks": [],
         "reference_supported_cells": 9, "rectified_supported_cells": 6},
        {"accepted": True, "reason": "reference_texture_supported", "failed_checks": [],
         "reference_supported_cells": 9, "rectified_supported_cells": 7},
    ])
    monkeypatch.setattr(control, "assess_startup_texture", lambda *_: next(checks))
    for i in range(3):
        decision = c.update(frame(i), i * .1)
    assert_zero(decision)
    assert decision["reason"] == "startup_texture_unverified" and c.motion_commands == 0
    recorded = decision["permission"]["startup_texture"]
    assert recorded["latest"]["accepted"] and not recorded["accepted"]
    assert recorded["frames"][0]["failed_checks"] == ["rectified_supported_cell_count"]
    assert [item["rectified_supported_cells"] for item in recorded["frames"]] == [5, 6, 7]
