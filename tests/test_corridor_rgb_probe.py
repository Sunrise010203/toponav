"""Analytic/unit diagnostics only; no training or independent benchmark data."""
import copy
import inspect
import json

import cv2
import numpy as np
import pytest

from roadvision.corridor_rgb_probe import (
    CALIBRATION_ID, K, plane_hypotheses, probe_rgb_window,
)


def texture(seed=41):
    rng = np.random.default_rng(seed)
    gray = cv2.GaussianBlur(rng.integers(20, 235, (128, 128), dtype=np.uint8), (3, 3), .6)
    return np.repeat(gray[:, :, None], 3, axis=2)


def window(kind="translation", seed=41):
    image = texture(seed)
    frames = []
    for i in range(3):
        if kind == "rotation":
            matrix = cv2.Rodrigues(np.array([0., .025 * i, 0.]))[0]
        else:
            t = np.array([.03 * i, 0., .045 * i]) if kind == "translation" else np.array([.0001 * i, 0., 0.])
            matrix = np.eye(3) + np.outer(t, [0, 0, 1])
        h = K @ matrix @ np.linalg.inv(K)
        view = cv2.warpPerspective(image, h, (128, 128))
        frames.append({"frame_id": f"f-{i}", "captured_sim_s": i / 10,
                       "rgb": np.tile(view, (2, 2, 1))})
    return frames


def probe(frames):
    return probe_rgb_window(frames, now_s=.2, calibration_id=CALIBRATION_ID)


def assert_no_physical_claim(result):
    packet = result["evidence_packet"]
    assert packet["corridor_boundary"]["state"] == "unknown"
    assert packet["front_surface"]["state"] == "unknown"
    assert packet["corridor_boundary"]["offset_m"] is None
    assert packet["front_surface"]["range_m"] is None
    assert result["review"]["motion_authorized"] is False
    assert result["review"]["arrival_status"] == "unverified"


def test_textured_planar_flow_has_real_pixel_tracks_but_not_physical_identity():
    result = probe(window())
    assert result["diagnostic_status"] == "planar_flow_candidate"
    front = result["views"]["front"]
    assert front["tracked_count"] >= 12 and len(front["tracks"]) == front["tracked_count"]
    assert front["models"]["homography_inlier_ratio"] >= .8
    assert front["models"]["rotation_residual_median_px"] > 1
    assert front["models"]["plane_hypotheses"]
    assert_no_physical_claim(result)


@pytest.mark.parametrize("kind,reason", [("rotation", "rotation_or_low_parallax"),
                                          ("tiny", "insufficient_image_displacement")])
def test_pure_rotation_or_small_displacement_stays_unknown(kind, reason):
    result = probe(window(kind))
    assert result["diagnostic_status"] == "unknown"
    assert reason in result["views"]["front"]["reasons"]
    assert_no_physical_claim(result)


@pytest.mark.parametrize("kind", ["uniform", "static", "repetition", "brightness"])
def test_texture_degeneracy_and_changed_hashes_do_not_establish_parallax(kind):
    frames = window()
    if kind == "uniform":
        images = [np.full((256, 256, 3), 100, np.uint8)] * 3
    elif kind == "static":
        images = [frames[0]["rgb"]] * 3
    elif kind == "brightness":
        images = [np.clip(frames[0]["rgb"].astype(int) + 3 * i, 0, 255).astype(np.uint8) for i in range(3)]
    else:
        checker = ((np.indices((128, 128)).sum(axis=0) // 8) % 2 * 150 + 40).astype(np.uint8)
        # Axis-aligned repeated corners, not just a one-dimensional stripe.
        checker = (((np.indices((128, 128))[0] // 12 + np.indices((128, 128))[1] // 12) % 2) * 150 + 40).astype(np.uint8)
        images = [np.tile(np.repeat(np.roll(checker, i, axis=1)[:, :, None], 3, axis=2), (2, 2, 1)) for i in range(3)]
    for frame, image in zip(frames, images):
        frame["rgb"] = image
    result = probe(frames)
    assert result["diagnostic_status"] == "unknown"
    if kind == "repetition":
        assert result["views"]["front"]["ambiguous_corner_count"] > 0
    assert_no_physical_claim(result)


def test_unrelated_and_occluded_frames_do_not_become_geometry():
    frames = window()
    frames[1]["rgb"][:] = 0
    result = probe(frames)
    assert result["diagnostic_status"] == "unknown"
    assert_no_physical_claim(result)


@pytest.mark.parametrize("normal,label", [([0, 1, 0], "ground_compatible"),
                                           ([0, 0, 1], "vertical_compatible")])
def test_analytic_homographies_retain_ground_and_vertical_interpretations_without_selecting_truth(normal, label):
    h = K @ (np.eye(3) + np.outer([.04, .01, .09], normal)) @ np.linalg.inv(K)
    hypotheses = plane_hypotheses(h)
    assert len(hypotheses) >= 2  # sign and decomposition ambiguity retained
    assert label in {v["normal_family"] for v in hypotheses}
    assert all("translation_over_plane_distance" in v and "range_m" not in v for v in hypotheses)


def test_front_left_right_are_separate_not_stereo_correspondences():
    frames = window()
    for frame in frames:
        frame["rgb"][:128, 128:] = 100
        frame["rgb"][128:, :128] = 100
    result = probe(frames)
    assert result["views"]["front"]["status"] == "planar_flow_candidate"
    assert result["views"]["left"]["status"] == result["views"]["right"]["status"] == "unknown"
    assert_no_physical_claim(result)


@pytest.mark.parametrize("kind", ["few", "many", "extra_truth", "bad_dtype", "bad_shape", "bad_id",
                                  "replay", "order", "future", "stale", "nan", "bool_time"])
def test_input_whitelist_and_invalid_window_reject(kind):
    frames = window()
    if kind == "few":
        frames.pop()
    elif kind == "many":
        frames.append(copy.deepcopy(frames[-1]))
    elif kind == "extra_truth":
        frames[0]["audit"] = {"pose": [0, 0, 0]}
    elif kind == "bad_dtype":
        frames[0]["rgb"] = frames[0]["rgb"].astype(float)
    elif kind == "bad_shape":
        frames[0]["rgb"] = frames[0]["rgb"][:128]
    elif kind == "bad_id":
        frames[0]["frame_id"] = ""
    elif kind == "replay":
        frames[1]["frame_id"] = frames[0]["frame_id"]
    elif kind == "order":
        frames.reverse()
    elif kind == "future":
        frames[2]["captured_sim_s"] = .3
    elif kind == "stale":
        frames[0]["captured_sim_s"] = -1
    elif kind == "nan":
        frames[0]["captured_sim_s"] = float("nan")
    elif kind == "bool_time":
        frames[0]["captured_sim_s"] = False
    with pytest.raises(ValueError):
        probe(frames)


def test_wrong_calibration_stale_window_and_invalid_now_reject():
    for now, calibration in [(.2, "other"), (.5, CALIBRATION_ID), (True, CALIBRATION_ID)]:
        with pytest.raises(ValueError):
            probe_rgb_window(window(), now_s=now, calibration_id=calibration)


def test_input_unchanged_output_json_and_pixel_binding():
    frames = window()
    before = copy.deepcopy(frames)
    result = probe(frames)
    json.dumps(result, allow_nan=False)
    for actual, original in zip(frames, before):
        assert np.array_equal(actual["rgb"], original["rgb"])
    import hashlib
    for frame, reference in zip(frames, result["evidence_packet"]["frames"]):
        assert reference["rgb_sha256"] == hashlib.sha256(frame["rgb"].tobytes()).hexdigest()
    import roadvision.corridor_rgb_probe as module
    source = inspect.getsource(module)
    assert "import pybullet" not in source and "import torch" not in source


def test_deterministic_diagnostics_and_no_color_rule():
    frames = window()
    assert probe(frames) == probe(frames)
    # A channel permutation preserves luminance only for this grayscale fixture.
    for frame in frames:
        frame["rgb"] = frame["rgb"][:, :, ::-1].copy()
    assert probe(frames)["diagnostic_status"] == "planar_flow_candidate"
