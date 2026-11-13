"""Restricted RGB-only corridor controller, not a trained structure classifier.

Assumes flat blue-grey floor, grey walls, level calibrated 90-degree cameras.
No physical poses, velocities, depth buffers or contact sensors are inputs.
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VisionConfig:
    camera_height_m: float = 0.45
    max_speed_m_s: float = 0.35
    max_omega_rad_s: float = 0.8
    visual_stop_range_m: float = 1.3
    max_frame_age_s: float = 0.25
    max_duration_s: float = 45.0
    brake_timeout_s: float = 3.0
    stop_confirmations: int = 3

    def __post_init__(self):
        for name, value in vars(self).items():
            if name == "stop_confirmations":
                if type(value) is not int or value < 2:
                    raise ValueError("At least two stop confirmations are required")
            elif type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"Invalid {name}")
        if self.max_speed_m_s > 0.5 or self.max_omega_rad_s > 1.0:
            raise ValueError("Policy exceeds the corridor motor command limits")


def analyze_panel(rgb, camera_height_m=0.45):
    """Fit projected floor boundaries; all numbers are estimated from RGB pixels.

    Calibration: f=64px, principal point=(64,64), view=128px, FOV=90deg.
    Ground mask uses a declared restricted colour rule, never segmentation IDs.
    """
    invalid = lambda reason: {"status": "invalid", "reason": reason}
    if not isinstance(rgb, np.ndarray) or rgb.shape != (256, 256, 3) or rgb.dtype != np.uint8:
        return invalid("invalid_rgb_panel")
    if any(rgb[y:y+128, x:x+128].max() == 0 for y, x in ((0, 0), (0, 128), (128, 0), (128, 128))):
        return invalid("missing_view")
    front = rgb[:128, :128].astype(np.int16)
    floor = (front[..., 2] - front[..., 0] > 8) & (front[..., 2] - front[..., 1] > 3)
    boundaries = []
    for column in (48, 56, 64, 72, 80):
        rows = np.flatnonzero(floor[65:, column])
        if not len(rows):
            return invalid("ground_not_visible")
        boundaries.append(int(rows[0]) + 65)
    boundary = float(np.median(boundaries))
    distance = camera_height_m * 64 / (boundary - 64 + 0.5)
    spread = max(boundaries) - min(boundaries)
    if distance < 1.9 and spread > 4:
        return invalid("nonuniform_front_obstruction")
    rows, centers = [], []
    for row in range(68, 105):
        indices = np.flatnonzero(floor[row])
        if len(indices) < 8:
            continue
        groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
        group = max(groups, key=len)
        left, right = int(group[0]), int(group[-1])
        if left <= 1 or right >= 126 or right - left < 8:
            continue
        # A center obstruction must not make a side patch become the whole floor.
        if not left <= 64 <= right:
            continue
        rows.append(row - 64)
        centers.append((left + right) / 2)
    if len(rows) < 6:
        return invalid("insufficient_floor_geometry")
    # End-plane clipping can corrupt the first visible row. Use a robust
    # center-line consensus rather than fitting that row as a side wall.
    # A global top-row cutoff is unsafe here: for off-center/yawed starts,
    # some central sample rays hit a side wall before they reach the end.
    rows, centers = np.asarray(rows), np.asarray(centers)
    first, second = np.triu_indices(len(rows), 1)
    robust_slope = float(np.median((centers[second] - centers[first]) / (rows[second] - rows[first])))
    robust_intercept = float(np.median(centers - robust_slope * rows))
    inliers = np.abs(centers - (robust_slope * rows + robust_intercept)) <= 1.5
    if inliers.sum() < 6 or inliers.mean() < 0.65:
        return invalid("unreliable_floor_geometry")
    discarded = int((~inliers).sum())
    rows, centers = rows[inliers], centers[inliers]
    slope, intercept = np.polyfit(rows, centers, 1)
    residual = float(np.sqrt(np.mean((np.polyval((slope, intercept), rows) - centers) ** 2)))
    heading, offset = math.atan((intercept - 64) / 64), float(slope * camera_height_m)
    if residual > 2 or abs(heading) > 0.5 or abs(offset) > 0.6:
        return invalid("unreliable_floor_geometry")
    return {"status": "valid", "reason": "calibrated_floor_boundaries", "heading_error_rad": heading,
            "lateral_error_m": offset, "front_range_m": distance, "front_spread_px": spread,
            "fit_rms_px": residual, "fit_rows": len(rows), "discarded_rows": discarded}


class CorridorController:
    """One serialized motion primitive; malformed input never authorizes a move."""
    def __init__(self, config=None):
        self.config = config or VisionConfig()
        self.state, self.reason = "observing", "waiting_rgb"
        self.started_s, self.last_now_s, self.brake_started_s = None, None, None
        self.last_capture_s, self.last_rgb = -1.0, None
        self.seen, self.stationary_frames, self.motion_commands = set(), 0, 0

    def _abort(self, reason):
        self.state, self.reason = "aborted", reason
        return self._output()

    def _output(self, v=0.0, omega=0.0, perception=None, image_change=None):
        return {"state": self.state, "reason": self.reason, "v_m_s": v, "omega_rad_s": omega,
                "stationary_frames": self.stationary_frames, "perception": perception,
                "image_change": image_change}

    def _time(self, now_s):
        if type(now_s) not in (int, float) or not math.isfinite(now_s) or now_s < 0:
            return "invalid_sim_time"
        if self.last_now_s is not None and now_s < self.last_now_s:
            return "sim_time_regression"
        self.last_now_s = now_s
        if self.started_s is None:
            self.started_s = now_s
        if now_s - self.started_s >= self.config.max_duration_s:
            return "motion_timeout"
        if self.brake_started_s is not None and now_s - self.brake_started_s >= self.config.brake_timeout_s:
            return "brake_confirmation_timeout"
        return None

    def missing(self, now_s):
        if self.state in {"visual_stop", "aborted"}:
            return self._output()
        return self._abort(self._time(now_s) or "missing_frame")

    def update(self, frame, now_s):
        if not isinstance(frame, dict) or set(frame) != {"frame_id", "captured_sim_s", "rgb"}:
            self._abort("invalid_frame_envelope")
            raise ValueError("Expected only frame_id/captured_sim_s/rgb; no truth fields")
        if self.state in {"visual_stop", "aborted"}:
            return self._output()
        error = self._time(now_s)
        if error:
            return self._abort(error)
        identity, captured = frame["frame_id"], frame["captured_sim_s"]
        if not isinstance(identity, str) or not identity or len(identity) > 256:
            return self._abort("invalid_frame_identity")
        if identity in self.seen:
            return self._abort("replayed_frame")
        self.seen.add(identity)
        if type(captured) not in (int, float) or not math.isfinite(captured) or captured < 0:
            return self._abort("invalid_capture_time")
        if captured > now_s:
            return self._abort("future_frame")
        if now_s - captured > self.config.max_frame_age_s:
            return self._abort("stale_frame")
        if captured <= self.last_capture_s:
            return self._abort("out_of_order_frame")
        self.last_capture_s = captured
        perception = analyze_panel(frame["rgb"], self.config.camera_height_m)
        if perception["status"] != "valid":
            return self._abort(perception["reason"])
        change = (float(np.abs(frame["rgb"].astype(np.float32) - self.last_rgb).mean())
                  if self.last_rgb is not None else None)
        self.last_rgb = frame["rgb"].astype(np.float32).copy()
        distance = perception["front_range_m"]
        heading, offset = perception["heading_error_rad"], perception["lateral_error_m"]
        if distance <= self.config.visual_stop_range_m or self.state == "braking":
            if abs(heading) > 0.15 or abs(offset) > 0.2:
                return self._abort("end_alignment_uncertain")
            if self.state != "braking":
                self.state, self.reason = "braking", "front_plane_near"
                self.brake_started_s = now_s
                self.stationary_frames = 0
            else:
                stable = change is not None and change < 0.8 and distance <= self.config.visual_stop_range_m + 0.1
                self.stationary_frames = self.stationary_frames + 1 if stable else 0
                if self.stationary_frames >= self.config.stop_confirmations:
                    self.state, self.reason = "visual_stop", "fresh_images_support_stationary_front_stop"
            return self._output(perception=perception, image_change=change)
        self.state, self.reason = "moving", "image_based_corridor_following"
        omega = max(-self.config.max_omega_rad_s, min(self.config.max_omega_rad_s, -2.0 * heading - 1.3 * offset))
        range_scale = max(0.15, min(1.0, (distance - self.config.visual_stop_range_m) / 1.5))
        alignment_scale = max(0.25, 1 - abs(heading) / 0.5)
        self.motion_commands += 1
        return self._output(self.config.max_speed_m_s * range_scale * alignment_scale, omega, perception, change)
