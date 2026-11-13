"""Unit geometry fixtures are diagnostics, never training or evaluation images."""
import inspect
import math

import numpy as np
import pytest

from roadvision.corridor_control import CorridorController, VisionConfig, analyze_panel
from simulation.corridor_motion import CorridorConfig, CorridorWorld, run_corridor


def panel(offset=0.0, yaw=0.0, distance=5.0):
    view = np.full((128, 128, 3), (150, 145, 140), dtype=np.uint8)
    for row in range(65, 128):
        if row < 64 + 0.45 * 64 / distance:
            continue
        center = 64 + 64 * math.tan(yaw) + offset / 0.45 * (row - 64)
        half = 0.9 / 0.45 * (row - 64)
        a, b = max(0, math.ceil(center - half)), min(128, math.floor(center + half) + 1)
        view[row, a:b] = (45, 65, 90)
    return np.tile(view, (2, 2, 1))


def packet(index, image=None, captured=None):
    return {"frame_id": f"f-{index}", "captured_sim_s": index * 0.1 if captured is None else captured,
            "rgb": panel() if image is None else image}


@pytest.mark.parametrize("offset,yaw", [(0, 0), (0.2, 0), (-0.2, 0), (0, 0.1), (0, -0.1)])
def test_rgb_geometry_recovers_signed_offsets_and_heading(offset, yaw):
    result = analyze_panel(panel(offset, yaw))
    assert result["status"] == "valid"
    assert result["lateral_error_m"] == pytest.approx(offset, abs=0.025)
    assert result["heading_error_rad"] == pytest.approx(yaw, abs=0.025)


def test_controller_steers_toward_center_using_only_image():
    assert CorridorController().update(packet(0, panel(offset=0.2)), 0)["omega_rad_s"] < 0
    assert CorridorController().update(packet(0, panel(offset=-0.2)), 0)["omega_rad_s"] > 0
    assert CorridorController().update(packet(0), 0)["v_m_s"] > 0


def test_partial_top_floor_row_at_end_wall_is_not_a_side_boundary():
    image = panel(distance=1.31)
    front = image[:128, :128]
    first = np.flatnonzero(front[:, 64, 2] > front[:, 64, 0])[0]
    front[first, 74:] = (150, 145, 140)
    geometry = analyze_panel(image)
    assert geometry["status"] == "valid"
    assert abs(geometry["heading_error_rad"]) < 0.025
    assert abs(geometry["lateral_error_m"]) < 0.025


@pytest.mark.parametrize("kind", ["missing", "occluded", "wrong_shape", "wrong_dtype"])
def test_invalid_images_request_zero_motion(kind):
    image = panel()
    if kind == "missing":
        image[:128, :128] = 0
    elif kind == "occluded":
        image[:] = (150, 145, 140)
    elif kind == "wrong_shape":
        image = image[:128]
    else:
        image = image.astype(float)
    result = CorridorController().update(packet(0, image), 0)
    assert result["state"] == "aborted" and result["v_m_s"] == result["omega_rad_s"] == 0


@pytest.mark.parametrize("kind", ["repeat", "stale", "future", "out_of_order"])
def test_bad_frame_events_abort_without_reusing_motion(kind):
    controller = CorridorController()
    controller.update(packet(0), 0)
    p = packet(1)
    if kind == "repeat":
        p["frame_id"] = "f-0"
    elif kind == "stale":
        p["captured_sim_s"] = 0
    elif kind == "future":
        p["captured_sim_s"] = 1.0
    else:
        p["captured_sim_s"] = -0.01
    result = controller.update(p, 0.5 if kind == "stale" else 0.1)
    assert result["state"] == "aborted" and result["v_m_s"] == 0


def test_stationary_stop_needs_new_frames_after_braking_not_one_close_image():
    controller = CorridorController()
    close = panel(distance=1.2)
    assert controller.update(packet(0, close), 0)["state"] == "braking"
    for index in (1, 2):
        assert controller.update(packet(index, close), index * 0.1)["state"] == "braking"
    assert controller.update(packet(3, close), 3 * 0.1)["state"] == "visual_stop"
    assert controller.update(packet(4), 0.4)["v_m_s"] == 0


def test_missing_camera_and_time_limit_stop():
    controller = CorridorController()
    controller.update(packet(0), 0)
    assert controller.missing(0.1)["state"] == "aborted"
    controller = CorridorController(config=VisionConfig(max_duration_s=0.5))
    controller.update(packet(0), 0)
    assert controller.update(packet(6), 0.6)["reason"] == "motion_timeout"


def test_fresh_but_out_of_order_capture_is_rejected():
    controller = CorridorController()
    controller.update(packet(1), 0.1)
    result = controller.update(packet(2, captured=0.05), 0.2)
    assert result["reason"] == "out_of_order_frame"
    assert result["v_m_s"] == result["omega_rad_s"] == 0


def test_unstable_images_cannot_confirm_stop_and_braking_is_bounded():
    controller = CorridorController(VisionConfig(brake_timeout_s=0.5))
    close = panel(distance=1.2)
    controller.update(packet(0, close), 0)
    for index in range(1, 5):
        changed = close.copy()
        changed[128:, :, :] = 255 if index % 2 else 80
        result = controller.update(packet(index, changed), index * 0.1)
        assert result["state"] == "braking" and result["stationary_frames"] == 0
        assert result["v_m_s"] == result["omega_rad_s"] == 0
    assert controller.update(packet(5, close), 0.5)["reason"] == "brake_confirmation_timeout"


@pytest.mark.parametrize("captured", [float("nan"), float("inf"), -1, True, "0"])
def test_invalid_capture_time_never_authorizes_motion(captured):
    result = CorridorController().update(packet(0, captured=captured), 0)
    assert result["reason"] == "invalid_capture_time"
    assert result["v_m_s"] == result["omega_rad_s"] == 0


def test_controller_cannot_read_truth_and_does_not_mutate_image():
    controller = CorridorController()
    p = packet(0)
    before = p["rgb"].copy()
    controller.update(p, 0)
    assert np.array_equal(before, p["rgb"])
    with pytest.raises(ValueError):
        controller.update({**packet(1), "true_pose": [1, 2, 3]}, 0.1)
    assert "pybullet" not in inspect.getsource(inspect.getmodule(CorridorController))


def test_wheel_motors_move_physics_continuously_and_change_rgb(monkeypatch):
    import pybullet as p
    def forbidden(*args, **kwargs):
        raise AssertionError("No base teleporting or forced base velocities")
    monkeypatch.setattr(p, "resetBasePositionAndOrientation", forbidden)
    monkeypatch.setattr(p, "resetBaseVelocity", forbidden)
    with CorridorWorld() as world:
        start = world.audit_state()
        first = world.capture()["rgb"]
        previous = start["position_m"]
        for _ in range(20):
            world.drive(0.3, 0)
            world.step_control()
            current = world.audit_state()["position_m"]
            assert np.linalg.norm(np.array(current) - previous) < 0.08
            previous = current
        last = world.audit_state()
        assert last["position_m"][0] > start["position_m"][0] + 0.3
        assert not np.array_equal(first, world.capture()["rgb"])
        assert last["wall_contacts"] == 0


def test_collision_audit_remembers_contacts_between_camera_ticks(monkeypatch):
    import pybullet as p
    with CorridorWorld() as world:
        baseline = world.wall_contact_physics_steps
        # Inject a transient physics contact; the next camera-tick audit is clear.
        monkeypatch.setattr(p, "getContactPoints", lambda *args, **kwargs: [("contact",)])
        world._step()
        monkeypatch.setattr(p, "getContactPoints", lambda *args, **kwargs: [])
        world._step()
        assert world.audit_state()["wall_contacts"] == 0
        assert world.wall_contact_physics_steps == baseline + 1


@pytest.mark.parametrize("offset,yaw", [(0, 0), (0.2, 0), (-0.2, 0), (0, 10), (0, -10),
                                       (0.2, 10), (-0.2, -10)])
def test_corridor_closed_loop_stops_at_end_without_contact(offset, yaw):
    result = run_corridor(CorridorConfig(initial_offset_m=offset, initial_yaw_deg=yaw))
    assert result["audit"]["end_wall_verified"], (result["state"], result["reason"], result["audit"])
    assert result["feedback"]["status"] == "completed_visual_stop"
    assert result["audit"]["distance_travelled_m"] > 2.0
    assert result["audit"]["wall_contact_frames"] == 0
    assert result["audit"]["wall_contact_physics_steps"] == 0
    assert result["audit"]["final_speed_m_s"] < 0.03
    assert abs(result["audit"]["final_yaw_deg"]) < 8


@pytest.mark.parametrize("fault", ["missing_frame", "missing_view", "occluded", "stale_frame", "future_frame", "replay_frame"])
def test_mid_motion_camera_fault_brakes_and_reports_uncertain_partial_motion(fault):
    result = run_corridor(fault=fault, fault_cycle=15)
    assert result["state"] == "aborted"
    assert result["feedback"]["status"] == "aborted_motion_uncertain"
    assert result["audit"]["distance_travelled_m"] > 0.1
    assert result["audit"]["final_speed_m_s"] < 0.03
    assert result["audit"]["wall_contact_frames"] == 0


def test_rejection_before_first_motion_is_not_atomic_failed_feedback():
    result = run_corridor(fault="missing_frame", fault_cycle=0)
    assert result["feedback"]["status"] == "rejected_no_motion"
    assert result["feedback"]["atomic_edge_compatible"] is False
    assert not result["audit"]["end_wall_verified"]


def test_replay_fault_requires_a_preceding_frame():
    with pytest.raises(ValueError, match="preceding frame"):
        run_corridor(fault="replay_frame", fault_cycle=0)


def test_timeout_does_not_claim_original_pose_or_full_edge_success():
    result = run_corridor(vision_config=VisionConfig(max_duration_s=1.0))
    assert result["reason"] == "motion_timeout"
    assert result["feedback"]["status"] == "aborted_motion_uncertain"
    assert not result["audit"]["end_wall_verified"]


def test_full_width_mid_corridor_barrier_does_not_pass_end_wall_truth_audit():
    result = run_corridor(CorridorConfig(obstacle="full_width"))
    assert result["audit"]["wall_contact_frames"] == 0
    assert not result["audit"]["end_wall_verified"]
    # A visually identical barrier cannot be assigned a target identity by RGB alone.
    if result["state"] == "visual_stop":
        assert result["audit"]["false_end_wall_claim"]


def test_narrow_obstacle_causes_safe_stop_not_collision():
    result = run_corridor(CorridorConfig(obstacle="narrow"))
    assert not result["audit"]["end_wall_verified"]
    assert result["audit"]["wall_contact_frames"] == 0


@pytest.mark.parametrize("field,value", [("max_speed_m_s", -1), ("max_duration_s", float("nan")),
                                         ("stop_confirmations", 0), ("max_speed_m_s", 0.51),
                                         ("max_omega_rad_s", 1.01)])
def test_invalid_vision_policy_is_rejected(field, value):
    with pytest.raises(ValueError):
        VisionConfig(**{field: value})
