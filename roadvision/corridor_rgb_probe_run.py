"""Offline RGB-only manifest runner. No simulator or model imports.

Original run.json is deliberately not an accepted input format. Extraction
of the whitelist is an evaluation-side operation before this runner starts.
"""
import argparse
from collections import Counter, deque
import hashlib
import json
import math
from pathlib import Path
import re
import time

import cv2
import numpy as np
from PIL import Image

from roadvision.corridor_rgb_probe import CALIBRATION_ID, probe_rgb_window


def load_manifest(path):
    data = json.loads(Path(path).read_text())
    if not isinstance(data, dict) or set(data) != {"version", "calibration_id", "frames"}:
        raise ValueError("Expected only version/calibration_id/frames")
    if data["version"] != "rgb_probe_sequence_v1" or data["calibration_id"] != CALIBRATION_ID:
        raise ValueError("Unsupported sequence version or calibration")
    frames = data["frames"]
    if not isinstance(frames, list) or not 1 <= len(frames) <= 10000:
        raise ValueError("Expected 1..10000 frame references")
    seen, previous = set(), -1.
    for frame in frames:
        if not isinstance(frame, dict) or set(frame) != {"frame_id", "captured_sim_s", "rgb_path", "rgb_sha256"}:
            raise ValueError("Invalid frame reference fields")
        identity, capture = frame["frame_id"], frame["captured_sim_s"]
        if not isinstance(identity, str) or not identity.strip() or len(identity) > 256 or identity in seen:
            raise ValueError("Invalid / duplicate frame identity")
        if type(capture) not in (int, float) or not math.isfinite(capture) or capture < 0 or capture <= previous:
            raise ValueError("Invalid / unordered capture time")
        if (not isinstance(frame["rgb_sha256"], str) or
                not re.fullmatch(r"[0-9a-f]{64}", frame["rgb_sha256"])):
            raise ValueError("Invalid RGB pixel hash")
        if not isinstance(frame["rgb_path"], str) or not frame["rgb_path"]:
            raise ValueError("Invalid RGB path")
        seen.add(identity)
        previous = capture
    return data


def _read_rgb(reference, root):
    relative = Path(reference["rgb_path"])
    path = (root / relative).resolve()
    if relative.is_absolute() or not path.is_relative_to(root) or path.suffix != ".png":
        raise ValueError("RGB path outside declared root or not PNG")
    with Image.open(path) as image:
        rgb = np.asarray(image).copy()
    if rgb.shape != (256, 256, 3) or rgb.dtype != np.uint8:
        raise ValueError("Expected uint8 256x256 RGB PNG, no silent conversion")
    if hashlib.sha256(rgb.tobytes(order="C")).hexdigest() != reference["rgb_sha256"]:
        raise ValueError("RGB pixels do not match frozen reference")
    return {"frame_id": reference["frame_id"], "captured_sim_s": reference["captured_sim_s"], "rgb": rgb}


def run_sequence(manifest_path, rgb_root, output_dir):
    output_dir, root = Path(output_dir), Path(rgb_root).resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    data = load_manifest(manifest_path)
    start = time.perf_counter()
    buffer, statuses = deque(maxlen=3), Counter()
    view_statuses = {view: Counter() for view in ("front", "left", "right")}
    view_reasons = {view: Counter() for view in view_statuses}
    windows, authorized = 0, 0
    with (output_dir / "windows.jsonl").open("x") as log:
        for reference in data["frames"]:
            buffer.append(_read_rgb(reference, root))
            if len(buffer) < 3:
                continue
            started = time.perf_counter()
            try:
                result = probe_rgb_window(list(buffer), now_s=buffer[-1]["captured_sim_s"],
                                          calibration_id=data["calibration_id"])
            except ValueError as error:
                # A gap wider than the existing envelope is a recorded input
                # rejection, not permission to extend the temporal baseline.
                result = {"diagnostic_status": "input_rejected", "reason": str(error),
                          "motion_authorized": False, "arrival_status": "unverified"}
            statuses[result["diagnostic_status"]] += 1
            if "review" in result:
                authorized += int(result["review"]["motion_authorized"])
                for name, value in result["views"].items():
                    view_statuses[name][value["status"]] += 1
                    view_reasons[name].update(value["reasons"])
            record = {"index": windows, "frame_ids": [f["frame_id"] for f in buffer],
                      "elapsed_wall_s": time.perf_counter() - started, "result": result}
            log.write(json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n")
            windows += 1
    summary = {"version": "corridor_rgb_probe_run_v1", "frame_count": len(data["frames"]),
               "window_count": windows, "insufficient_window_frames": len(data["frames"]) < 3,
               "diagnostic_status_counts": dict(statuses),
               "view_status_counts": {k: dict(v) for k, v in view_statuses.items()},
               "view_reason_counts": {k: dict(v) for k, v in view_reasons.items()},
               "motion_authorized_count": authorized, "arrival_status": "unverified",
               "metric_geometry_status": "unknown", "elapsed_wall_s": time.perf_counter() - start,
               "clock_domain": "capture simulation seconds; runtime is wall time, not hard realtime"}
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--rgb-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    cv2.setNumThreads(1)
    try:
        result = run_sequence(args.manifest, args.rgb_root, args.output)
    except (ValueError, OSError) as error:
        parser.exit(2, f"RGB probe input/run error: {error}\n")
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
