"""Offline loader must bind real pixels and never forward audit fields."""
import hashlib
import json

import numpy as np
from PIL import Image
import pytest

from roadvision.corridor_rgb_probe import CALIBRATION_ID
from roadvision.corridor_rgb_probe_run import run_sequence


def inputs(tmp_path, count=3):
    rgb_root = tmp_path / "rgb"
    rgb_root.mkdir()
    frames = []
    for i in range(count):
        rgb = np.full((256, 256, 3), 80 + i, np.uint8)
        Image.fromarray(rgb).save(rgb_root / f"{i}.png")
        frames.append({"frame_id": f"f{i}", "captured_sim_s": i / 10,
                       "rgb_path": f"{i}.png", "rgb_sha256": hashlib.sha256(rgb.tobytes()).hexdigest()})
    manifest = {"version": "rgb_probe_sequence_v1", "calibration_id": CALIBRATION_ID, "frames": frames}
    path = tmp_path / "input.json"
    path.write_text(json.dumps(manifest))
    return path, rgb_root, manifest


def test_all_consecutive_windows_bind_pixels_and_keep_unknown(tmp_path):
    path, root, _ = inputs(tmp_path, count=5)
    out = tmp_path / "out"
    summary = run_sequence(path, root, out)
    assert summary["frame_count"] == 5
    assert summary["window_count"] == 3
    assert summary["diagnostic_status_counts"] == {"unknown": 3}
    assert summary["motion_authorized_count"] == 0
    records = [json.loads(line) for line in (out / "windows.jsonl").read_text().splitlines()]
    assert [r["index"] for r in records] == [0, 1, 2]
    assert all(r["result"]["review"]["arrival_status"] == "unverified" for r in records)
    assert json.loads((out / "summary.json").read_text()) == summary


def test_short_sequence_is_recorded_not_fabricated(tmp_path):
    path, root, _ = inputs(tmp_path, count=1)
    summary = run_sequence(path, root, tmp_path / "out")
    assert summary["window_count"] == 0 and summary["insufficient_window_frames"] is True


@pytest.mark.parametrize("kind", ["extra_audit", "frame_truth", "bad_hash", "escape", "absolute",
                                  "replay", "time_order", "bad_version", "bad_calibration", "missing_png"])
def test_loader_rejects_tainted_or_invalid_sources(tmp_path, kind):
    path, root, data = inputs(tmp_path)
    if kind == "extra_audit":
        data["audit"] = {"position": [1, 2, 3]}
    elif kind == "frame_truth":
        data["frames"][0]["pose"] = [0, 0, 0]
    elif kind == "bad_hash":
        data["frames"][0]["rgb_sha256"] = "0" * 64
    elif kind == "escape":
        data["frames"][0]["rgb_path"] = "../outside.png"
    elif kind == "absolute":
        data["frames"][0]["rgb_path"] = str(root / "0.png")
    elif kind == "replay":
        data["frames"][1]["frame_id"] = "f0"
    elif kind == "time_order":
        data["frames"][1]["captured_sim_s"] = 0
    elif kind == "bad_version":
        data["version"] = "old"
    elif kind == "bad_calibration":
        data["calibration_id"] = "other"
    else:
        (root / "0.png").unlink()
    path.write_text(json.dumps(data))
    with pytest.raises((ValueError, FileNotFoundError)):
        run_sequence(path, root, tmp_path / "out")


def test_time_gap_rejects_window_without_extending_time_budget(tmp_path):
    path, root, data = inputs(tmp_path)
    data["frames"][2]["captured_sim_s"] = .4
    path.write_text(json.dumps(data))
    summary = run_sequence(path, root, tmp_path / "out")
    assert summary["diagnostic_status_counts"] == {"input_rejected": 1}
    assert summary["motion_authorized_count"] == 0


def test_existing_output_is_never_replaced(tmp_path):
    path, root, _ = inputs(tmp_path)
    out = tmp_path / "out"
    run_sequence(path, root, out)
    with pytest.raises(FileExistsError):
        run_sequence(path, root, out)


def test_only_whitelisted_frame_fields_reach_probe(tmp_path, monkeypatch):
    path, root, _ = inputs(tmp_path)
    import roadvision.corridor_rgb_probe_run as module
    original = module.probe_rgb_window
    calls = []
    def checking(frames, **kwargs):
        assert all(set(f) == {"frame_id", "captured_sim_s", "rgb"} for f in frames)
        assert set(kwargs) == {"now_s", "calibration_id"}
        calls.append(frames)
        return original(frames, **kwargs)
    monkeypatch.setattr(module, "probe_rgb_window", checking)
    run_sequence(path, root, tmp_path / "out")
    assert len(calls) == 1
