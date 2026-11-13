"""Protocol fixtures, not perceptual or physical geometry evidence."""
import copy

import pytest

from roadvision.corridor_evidence import review_corridor_evidence, split_motion_feedback


CALIBRATION = "four_rgb_level_128_fov90_h045_v1"


def packet():
    return {
        "version": "corridor_evidence_v1", "evidence_id": "e-1", "calibration_id": CALIBRATION,
        "frames": [{"frame_id": "f-1", "captured_sim_s": .1, "rgb_sha256": "1" * 64},
                   {"frame_id": "f-2", "captured_sim_s": .2, "rgb_sha256": "2" * 64}],
        "appearance_boundary": {"state": "candidate", "method": "rgb_colour"},
        "corridor_boundary": {"state": "unknown", "method": "none", "views": [],
                              "offset_m": None, "heading_rad": None},
        "front_surface": {"state": "unknown", "method": "none", "views": [], "range_m": None},
    }


def geometry_packet(front_state="non_ground_candidate"):
    p = packet()
    p["corridor_boundary"] = {"state": "candidate", "method": "rgb_temporal_geometry",
                              "views": ["left", "right"], "offset_m": .02, "heading_rad": .01}
    p["front_surface"] = {"state": front_state, "method": "rgb_temporal_geometry", "views": ["front"],
                          "range_m": 1.3 if front_state == "non_ground_candidate" else None}
    return p


def review(p):
    return review_corridor_evidence(p, now_s=.2, calibration_id=CALIBRATION)


def test_colour_candidate_is_not_physical_corridor_or_wall():
    result = review(packet())
    assert result["corridor_geometry"] == "unknown"
    assert result["front_surface"] == "unknown"
    assert result["motion_authorized"] is False
    assert result["arrival_status"] == "unverified"


@pytest.mark.parametrize("part", ["corridor_boundary", "front_surface"])
def test_colour_provenance_cannot_be_promoted_into_geometry(part):
    p = geometry_packet()
    p[part]["method"] = "rgb_colour"
    with pytest.raises(ValueError):
        review(p)


def test_ground_transition_is_not_obstacle_or_arrival():
    result = review(geometry_packet("ground_consistent"))
    assert "ground_transition_is_not_wall" in result["review_notes"]
    assert not result["motion_authorized"] and result["arrival_status"] == "unverified"


def test_non_ground_candidate_does_not_establish_endpoint_identity():
    result = review(geometry_packet())
    assert result["corridor_geometry"] == "candidate"
    assert "surface_is_not_endpoint_identity" in result["review_notes"]
    assert result["arrival_status"] == "unverified" and not result["motion_authorized"]


@pytest.mark.parametrize("kind", ["one_frame", "replayed_id", "same_capture", "same_rgb", "no_left", "no_right"])
def test_temporal_geometry_claim_needs_declared_two_sided_and_temporal_support(kind):
    p = geometry_packet()
    if kind == "one_frame":
        p["frames"] = p["frames"][:1]
    elif kind == "replayed_id":
        p["frames"][1]["frame_id"] = "f-1"
    elif kind == "same_capture":
        p["frames"][1]["captured_sim_s"] = .1
    elif kind == "same_rgb":
        p["frames"][1]["rgb_sha256"] = "1" * 64
    else:
        p["corridor_boundary"]["views"] = ["right" if kind == "no_left" else "left"]
    with pytest.raises(ValueError):
        review(p)


@pytest.mark.parametrize("time", [-1, .3, float("nan"), float("inf"), True, "0.1"])
def test_invalid_or_future_capture_time_is_rejected(time):
    p = packet()
    p["frames"][0]["captured_sim_s"] = time
    with pytest.raises(ValueError):
        review(p)


def test_old_capture_wrong_calibration_and_clock_regression_are_rejected():
    with pytest.raises(ValueError):
        review_corridor_evidence(packet(), now_s=.6, calibration_id=CALIBRATION)
    with pytest.raises(ValueError):
        review_corridor_evidence(packet(), now_s=.2, calibration_id="different")
    p = packet()
    p["frames"].reverse()
    with pytest.raises(ValueError):
        review(p)


@pytest.mark.parametrize("field", ["true_pose", "node_id", "audit", "succeeded", "depth"])
def test_truth_topology_and_extra_sensor_fields_are_forbidden(field):
    p = packet()
    p[field] = 1
    with pytest.raises(ValueError):
        review(p)


@pytest.mark.parametrize("part,key,value", [
    ("corridor_boundary", "offset_m", 0), ("corridor_boundary", "heading_rad", 0),
    ("front_surface", "range_m", 0),
])
def test_unknown_metrics_must_be_null_not_fake_zero(part, key, value):
    p = packet()
    p[part][key] = value
    with pytest.raises(ValueError):
        review(p)


@pytest.mark.parametrize("range_m", [-1, 0, float("nan"), float("inf"), True])
def test_non_ground_candidate_cannot_have_invalid_metric_range(range_m):
    p = geometry_packet()
    p["front_surface"]["range_m"] = range_m
    with pytest.raises(ValueError):
        review(p)


@pytest.mark.parametrize("status,commanded,outcome", [
    ("completed_visual_stop", True, "visual_stop_supported"),
    ("completed_visual_stop", False, "visual_stop_supported"),
    ("aborted_motion_uncertain", True, "motion_aborted_uncertain"),
    ("rejected_no_motion", False, "not_commanded"),
])
def test_motion_feedback_never_becomes_node_arrival_or_legacy_feedback(status, commanded, outcome):
    result = split_motion_feedback({"status": status, "motion_was_commanded": commanded,
                                    "atomic_edge_compatible": False})
    assert result["motion_outcome"] == outcome
    assert result["pose_status"] == result["arrival_status"] == "unverified"
    assert result["legacy_feedback"] is None and result["atomic_edge_compatible"] is False


@pytest.mark.parametrize("change", [
    {"status": "failed"}, {"status": "succeeded"}, {"atomic_edge_compatible": True},
    {"atomic_edge_compatible": 0}, {"motion_was_commanded": 1}, {"audit": {"end_wall_verified": True}},
    {"status": "rejected_no_motion", "motion_was_commanded": True},
    {"status": "aborted_motion_uncertain", "motion_was_commanded": False},
])
def test_feedback_rejects_legacy_atomic_truth_and_contradictory_claims(change):
    feedback = {"status": "completed_visual_stop", "motion_was_commanded": True, "atomic_edge_compatible": False}
    feedback.update(change)
    with pytest.raises(ValueError):
        split_motion_feedback(feedback)


def test_review_does_not_mutate_input_and_requires_no_physics_or_model():
    p = geometry_packet()
    before = copy.deepcopy(p)
    review(p)
    assert p == before
    import inspect
    import roadvision.corridor_evidence as module
    source = inspect.getsource(module)
    assert "import pybullet" not in source and "import torch" not in source
