"""Read-only three-frame RGB diagnostics, not a physical geometry producer.

Planar image flow and homography decompositions are competing explanations,
not proof of ego translation, ground/wall identity, metric scale or arrival.
No existing motion entrypoint calls this module. RANSAC is serialized with a
fixed OpenCV seed; concurrent calls sharing OpenCV's RNG are not supported.
"""
import hashlib
import math

import cv2
import numpy as np

from roadvision.corridor_evidence import review_corridor_evidence


CALIBRATION_ID = "corridor_panel_128_f64_c64_level_v1"
K = np.array([[64., 0., 64.], [0., 64., 64.], [0., 0., 1.]])
POLICY = {
    "corners": 80, "quality": .02, "corner_distance_px": 7, "corner_block_px": 5,
    "patch_px": 7, "ambiguous_patch_correlation": .98,
    "lk_window_px": 15, "lk_levels": 2, "lk_iterations": 30, "lk_epsilon": .01,
    "fb_max_px": .5, "path_max_px": .75, "min_tracks": 12,
    "grid_side": 4, "min_grid_cells": 6, "min_spread_ratio": .1,
    "ransac_px": 1., "ransac_iterations": 2000, "ransac_confidence": .995,
    "ransac_seed": 0, "min_inlier_ratio": .8, "composition_max_px": .75,
    "min_displacement_px": .75, "min_rotation_residual_px": 1., "normal_tolerance_deg": 15.,
}


def _project(matrix, points):
    if (type(points) is np.ndarray and points.dtype == np.float64 and
            points.ndim == 2 and points.shape[1] == 2 and points.flags.c_contiguous):
        # Exactly the original C float64 (N,3) operand, without an intermediate
        # ones array or column_stack's concatenate dispatch. No batching or
        # change to the matrix product, division, or validity checks below.
        homogeneous = np.empty((len(points), 3), dtype=np.float64)
        homogeneous[:, :2] = points
        homogeneous[:, 2] = 1.
        homogeneous = homogeneous @ matrix.T
    else:
        homogeneous = np.column_stack((points, np.ones(len(points)))) @ matrix.T
    if np.any(np.abs(homogeneous[:, 2]) < 1e-9):
        return None
    projected = homogeneous[:, :2] / homogeneous[:, 2:]
    return projected if np.isfinite(projected).all() else None


def plane_hypotheses(homography):
    """Retain *all* finite decompositions, including sign/plane ambiguities.

    Camera axes: x right, y down, z forward. t/d is dimensionless, not
    metric translation. Families assume the declared level installation;
    neither their normal nor cheirality/physical identity is verified here.
    """
    h = np.asarray(homography, dtype=float)
    if h.shape != (3, 3) or not np.isfinite(h).all():
        raise ValueError("Expected finite 3x3 homography")
    try:
        count, rotations, translations, normals = cv2.decomposeHomographyMat(h, K)
    except cv2.error:
        return []
    result = []
    tolerance = math.radians(POLICY["normal_tolerance_deg"])
    for index in range(count):
        r, t, n = rotations[index], translations[index].ravel(), normals[index].ravel()
        if not all(np.isfinite(v).all() for v in (r, t, n)):
            continue
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            family = "rotation_only"
        else:
            down = abs(float(n[1] / norm))
            family = ("ground_compatible" if down >= math.cos(tolerance) else
                      "vertical_compatible" if down <= math.sin(tolerance) else "oblique")
        result.append({"normal_camera": n.tolist(), "rotation": r.tolist(),
                       "translation_over_plane_distance": t.tolist(), "normal_family": family})
    return result


def _unique_corners(gray):
    points = cv2.goodFeaturesToTrack(gray, maxCorners=POLICY["corners"], qualityLevel=POLICY["quality"],
                                   minDistance=POLICY["corner_distance_px"], blockSize=POLICY["corner_block_px"])
    raw = np.empty((0, 2), np.float32) if points is None else points.reshape(-1, 2)
    eligible, patches = [], []
    margin = POLICY["patch_px"] // 2 + 1
    for point in raw:
        if np.any(point < margin) or np.any(point >= 128 - margin):
            continue
        patch = cv2.getRectSubPix(gray, (POLICY["patch_px"],) * 2, tuple(map(float, point))).astype(float).ravel()
        patch -= patch.mean()
        norm = np.linalg.norm(patch)
        if norm <= 1:
            continue
        eligible.append(point)
        patches.append(patch / norm)
    if not eligible:
        return raw, np.empty((0, 2), np.float32), 0
    correlation = np.abs(np.asarray(patches) @ np.asarray(patches).T)
    np.fill_diagonal(correlation, 0)
    ambiguous = (correlation >= POLICY["ambiguous_patch_correlation"]).any(axis=1)
    return raw, np.asarray(eligible, np.float32)[~ambiguous], int(ambiguous.sum())


def _lk(first, second, points):
    if len(points) == 0:
        return points.copy(), np.zeros(0, bool)
    target, status, _ = cv2.calcOpticalFlowPyrLK(
        first, second, points.reshape(-1, 1, 2), None,
        winSize=(POLICY["lk_window_px"],) * 2, maxLevel=POLICY["lk_levels"],
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                  POLICY["lk_iterations"], POLICY["lk_epsilon"]))
    if target is None or status is None:
        return np.zeros_like(points), np.zeros(len(points), bool)
    target = target.reshape(-1, 2)
    valid = status.ravel().astype(bool) & np.isfinite(target).all(axis=1)
    valid &= (target >= 4).all(axis=1) & (target < 124).all(axis=1)
    # Never send NaN/Inf from a failed LK point into the next optical-flow call.
    target[~valid] = 0
    return target, valid


def _grid_count(points):
    cells = np.clip((points / (128 / POLICY["grid_side"])).astype(int), 0, POLICY["grid_side"] - 1)
    return int(len(np.unique(cells, axis=0)))


def _spread_ratio(points):
    singular = np.linalg.svd(points - points.mean(axis=0), compute_uv=False)
    return float(singular[-1] / singular[0]) if singular[0] > 1e-9 else 0.


def _homography(source, target):
    cv2.setRNGSeed(POLICY["ransac_seed"])
    h, mask = cv2.findHomography(source, target, cv2.RANSAC, POLICY["ransac_px"],
                               maxIters=POLICY["ransac_iterations"], confidence=POLICY["ransac_confidence"])
    if h is None or mask is None or not np.isfinite(h).all() or np.linalg.cond(h) > 1e8:
        return None, np.zeros(len(source), bool)
    return h, mask.ravel().astype(bool)


def _rotation_residual(source, target):
    inv = np.linalg.inv(K)
    first = np.column_stack((source, np.ones(len(source)))) @ inv.T
    last = np.column_stack((target, np.ones(len(target)))) @ inv.T
    first /= np.linalg.norm(first, axis=1)[:, None]
    last /= np.linalg.norm(last, axis=1)[:, None]
    u, _, vt = np.linalg.svd(first.T @ last)
    correction = np.diag([1., 1., np.linalg.det(vt.T @ u.T)])
    rotation = vt.T @ correction @ u.T
    predicted = _project(K @ rotation @ inv, source)
    return (None if predicted is None else float(np.median(np.linalg.norm(predicted - target, axis=1)))), rotation


def _models(first, middle, last):
    reasons, diagnostics = [], {}
    h, inliers = _homography(first, last)
    diagnostics["homography_inlier_count"] = int(inliers.sum())
    diagnostics["homography_inlier_ratio"] = float(inliers.mean())
    diagnostics["homography_inlier_mask"] = inliers.tolist()
    if h is None or inliers.sum() < POLICY["min_tracks"] or inliers.mean() < POLICY["min_inlier_ratio"]:
        return diagnostics, ["planar_model_unsupported"]
    # A well-spread raw set can still contain a degenerate inlier cluster.
    if (min(_grid_count(first[inliers]), _grid_count(last[inliers])) < POLICY["min_grid_cells"] or
            min(_spread_ratio(first[inliers]), _spread_ratio(last[inliers])) < POLICY["min_spread_ratio"]):
        return diagnostics, ["planar_inliers_poorly_distributed"]
    predicted = _project(h, first[inliers])
    if predicted is None:
        return diagnostics, ["homography_projection_invalid"]
    diagnostics["homography"] = h.tolist()
    diagnostics["homography_residual_median_px"] = float(np.median(np.linalg.norm(predicted - last[inliers], axis=1)))
    diagnostics["plane_hypotheses"] = plane_hypotheses(h)
    first_h, first_mask = _homography(first, middle)
    last_h, last_mask = _homography(middle, last)
    if (first_h is None or last_h is None or min(first_mask.mean(), last_mask.mean()) < POLICY["min_inlier_ratio"]):
        reasons.append("adjacent_planar_models_unsupported")
        diagnostics["composition_median_px"] = None
    else:
        composed = _project(last_h @ first_h, first[inliers])
        error = None if composed is None else float(np.median(np.linalg.norm(composed - predicted, axis=1)))
        diagnostics["composition_median_px"] = error
        if error is None or error > POLICY["composition_max_px"]:
            reasons.append("temporal_models_inconsistent")
    displacement = float(np.median(np.linalg.norm(last[inliers] - first[inliers], axis=1)))
    residual, rotation = _rotation_residual(first[inliers], last[inliers])
    diagnostics.update(displacement_median_px=displacement, rotation_residual_median_px=residual,
                       best_fit_rotation=rotation.tolist())
    if displacement < POLICY["min_displacement_px"]:
        reasons.append("insufficient_image_displacement")
    if residual is None or residual <= POLICY["min_rotation_residual_px"]:
        reasons.append("rotation_or_low_parallax")
    return diagnostics, reasons


def _analyze_view(images):
    gray = [cv2.cvtColor(np.ascontiguousarray(im), cv2.COLOR_RGB2GRAY) for im in images]
    raw, first, ambiguous = _unique_corners(gray[0])
    output = {"status": "unknown", "reasons": [], "detected_count": len(raw),
              "ambiguous_corner_count": ambiguous, "usable_corner_count": len(first),
              "tracked_count": 0, "initial_corners": raw.tolist(), "tracks": [], "models": {}}
    if len(first) < POLICY["min_tracks"]:
        output["reasons"] = ["insufficient_unique_texture"]
        return output
    middle, ok01 = _lk(gray[0], gray[1], first)
    back0, ok10 = _lk(gray[1], gray[0], middle)
    last, ok12 = _lk(gray[1], gray[2], middle)
    back1, ok21 = _lk(gray[2], gray[1], last)
    direct, ok02 = _lk(gray[0], gray[2], first)
    back_direct, ok20 = _lk(gray[2], gray[0], direct)
    fb = np.maximum.reduce([np.linalg.norm(back0 - first, axis=1),
                            np.linalg.norm(back1 - middle, axis=1),
                            np.linalg.norm(back_direct - first, axis=1)])
    path = np.linalg.norm(direct - last, axis=1)
    valid = ok01 & ok10 & ok12 & ok21 & ok02 & ok20
    valid &= (fb <= POLICY["fb_max_px"]) & (path <= POLICY["path_max_px"])
    first, middle, last, fb, path = [v[valid] for v in (first, middle, last, fb, path)]
    output["tracked_count"] = len(first)
    output["tracks"] = [{"first": a.tolist(), "middle": b.tolist(), "last": c.tolist(),
                         "fb_max_px": float(f), "path_error_px": float(p)}
                        for a, b, c, f, p in zip(first, middle, last, fb, path)]
    if len(first) < POLICY["min_tracks"]:
        output["reasons"] = ["insufficient_consistent_tracks"]
        return output
    output["grid_cells"] = {"first": _grid_count(first), "last": _grid_count(last)}
    output["spread_ratio"] = {"first": _spread_ratio(first), "last": _spread_ratio(last)}
    if (min(output["grid_cells"].values()) < POLICY["min_grid_cells"] or
            min(output["spread_ratio"].values()) < POLICY["min_spread_ratio"]):
        output["reasons"] = ["poor_spatial_support"]
        return output
    output["models"], output["reasons"] = _models(first, middle, last)
    if not output["reasons"]:
        output["status"] = "planar_flow_candidate"
    return output


def probe_rgb_window(frames, *, now_s, calibration_id):
    """Analyze exactly three RGB captures with no scene/pose/command inputs.

    All physical evidence remains unknown: this version does not establish
    unique plane identity, shared rigid motion, metric scale or calibration
    validity from pixels. Diagnostics are deliberately a separate namespace.
    """
    if calibration_id != CALIBRATION_ID:
        raise ValueError("Unsupported calibration / level installation assumption")
    if not isinstance(frames, list) or len(frames) != 3:
        raise ValueError("Expected exactly three captures")
    references = []
    for frame in frames:
        if not isinstance(frame, dict) or set(frame) != {"frame_id", "captured_sim_s", "rgb"}:
            raise ValueError("Expected only frame_id/captured_sim_s/rgb")
        rgb = frame["rgb"]
        if not isinstance(rgb, np.ndarray) or rgb.shape != (256, 256, 3) or rgb.dtype != np.uint8:
            raise ValueError("Expected uint8 256x256 RGB panel")
        references.append({"frame_id": frame["frame_id"], "captured_sim_s": frame["captured_sim_s"],
                           "rgb_sha256": hashlib.sha256(rgb.tobytes(order="C")).hexdigest()})
    packet = {"version": "corridor_evidence_v1", "evidence_id": "rgb-probe-" + references[-1]["rgb_sha256"],
              "calibration_id": calibration_id, "frames": references,
              "appearance_boundary": {"state": "unknown", "method": "none"},
              "corridor_boundary": {"state": "unknown", "method": "none", "views": [],
                                    "offset_m": None, "heading_rad": None},
              "front_surface": {"state": "unknown", "method": "none", "views": [], "range_m": None}}
    review = review_corridor_evidence(packet, now_s=now_s, calibration_id=CALIBRATION_ID)
    views = {}
    for name, (y, x) in {"front": (0, 0), "left": (0, 128), "right": (128, 0)}.items():
        views[name] = _analyze_view([f["rgb"][y:y+128, x:x+128] for f in frames])
    return {"version": "corridor_rgb_probe_v1", "diagnostic_status":
            "planar_flow_candidate" if any(v["status"] == "planar_flow_candidate" for v in views.values()) else "unknown",
            "views": views, "evidence_packet": packet, "review": review,
            "limitations": ["diagnostic_flow_is_not_ego_translation", "plane_decomposition_not_physical_identity",
                            "level_calibration_assumed_not_verified", "no_metric_or_cross_view_rigid_solution"]}
