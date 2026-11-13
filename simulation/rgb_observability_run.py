"""Frozen controlled RGB study: generate -> isolated prediction -> audit.

The predictor subprocess runs the existing RGB-only entrypoint. Audit files
are read only after every predictor process has finished. This is logical
input separation, not an OS security sandbox or a claim of blind evaluation.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import traceback

import numpy as np

from roadvision.corridor_rgb_probe import CALIBRATION_ID
from simulation.rgb_observability import generate_sequence, load_suite, plane_transfer, view_basis

ROOT = Path(__file__).resolve().parents[1]


def verify_freeze(path, *, root=ROOT):
    root = Path(root).resolve()
    lock = json.loads(Path(path).read_text())
    for name, expected in lock["files"].items():
        target = (root / name).resolve()
        if Path(name).is_absolute() or not target.is_relative_to(root):
            raise ValueError("Frozen file outside workspace")
        if hashlib.sha256(target.read_bytes()).hexdigest() != expected:
            raise ValueError(f"Frozen file changed: {name}")
    return lock


def generate_suite(suite_path, output):
    suite = load_suite(suite_path)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    sequences, errors = [], []
    for case in suite["cases"]:
        try:
            directory = output / "cases" / case["id"]
            generate_sequence(case, directory)
            sequences.append({"sequence_id": case["id"], "public_root": str((directory / "public").resolve())})
        except Exception:
            errors.append({"sequence_id": case["id"], "traceback": traceback.format_exc()})
    (output / "public-index.json").write_text(json.dumps({"version": "rgb_observability_public_index_v1",
                                                         "sequences": sequences}, indent=2) + "\n")
    generation = {"declared_cases": len(suite["cases"]), "generated_cases": len(sequences), "errors": errors}
    (output / "generation.json").write_text(json.dumps(generation, indent=2) + "\n")
    return generation


def predict_index(index_path, output):
    data = json.loads(Path(index_path).read_text())
    if (not isinstance(data, dict) or set(data) != {"version", "sequences"} or
            data["version"] != "rgb_observability_public_index_v1" or not isinstance(data["sequences"], list)):
        raise ValueError("Invalid public sequence index")
    seen = set()
    for item in data["sequences"]:
        if not isinstance(item, dict) or set(item) != {"sequence_id", "public_root"}:
            raise ValueError("Only public sequence paths / external report IDs allowed")
        identity = item["sequence_id"]
        if not isinstance(identity, str) or not re.fullmatch(r"[a-z0-9_]{1,64}", identity) or identity in seen:
            raise ValueError("Invalid / duplicate report ID")
        if not isinstance(item["public_root"], str) or not Path(item["public_root"]).is_absolute():
            raise ValueError("Expected explicit public RGB root")
        seen.add(identity)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    results = []
    environment = {**os.environ, "PYTHONDONTWRITEBYTECODE": "1", "OMP_NUM_THREADS": "2",
                   "OPENBLAS_NUM_THREADS": "2", "MKL_NUM_THREADS": "2"}
    for item in data["sequences"]:
        public, directory = Path(item["public_root"]), output / item["sequence_id"]
        directory.mkdir()
        command = [sys.executable, "-m", "roadvision.corridor_rgb_probe_run",
                   "--manifest", str(public / "sequence.json"), "--rgb-root", str(public),
                   "--output", str(directory / "diagnostics")]
        # Neither case.json nor truth.json nor scene labels enter the child.
        completed = subprocess.run(command, cwd=ROOT, env=environment, text=True, capture_output=True)
        (directory / "stdout.log").write_text(completed.stdout)
        (directory / "stderr.log").write_text(completed.stderr)
        results.append({"sequence_id": item["sequence_id"], "exit_code": completed.returncode})
    phase = {"scope": "public manifest/RGB only; all predictor children completed", "sequences": results}
    (output / "phase-complete.json").write_text(json.dumps(phase, indent=2) + "\n")
    return phase


def _project(h, points):
    projected = np.column_stack((points, np.ones(len(points)))) @ h.T
    return projected[:, :2] / projected[:, 2:]


def _single_plane_audit(truth, record, view, diagnostic):
    models = diagnostic["models"]
    if "homography" not in models:
        return None
    first, last = truth["captures"][record["index"]], truth["captures"][record["index"] + 2]
    surface = truth["surfaces"][0]
    h = plane_transfer(surface, first, last, view)
    points = np.array([track["first"] for track in diagnostic["tracks"]], dtype=float)
    inliers = np.array(models["homography_inlier_mask"], dtype=bool)
    # Evaluate only rays actually meeting the finite reference plane, not sky.
    f = 64 / math.tan(math.radians(first["rendered_fov_deg"]) / 2)
    k = np.array([[f, 0, 63.5], [0, f, 63.5], [0, 0, 1.]])
    _, _, basis = view_basis(first, view)
    rays = np.column_stack((points, np.ones(len(points)))) @ np.linalg.inv(k).T @ basis.T
    normal, origin = np.array(surface["normal_world"]), np.array(first["position_m"])
    vertices = np.array(surface["vertices_world_m"]) + np.array(first["surface_shift_m"])
    numerator, denominator = float(normal @ (vertices[0] - origin)), rays @ normal
    good = np.abs(denominator) > 1e-9
    distance = np.divide(numerator, denominator, out=np.zeros_like(denominator), where=good)
    hit = origin + rays * distance[:, None]
    for edge in (vertices[1] - vertices[0], vertices[3] - vertices[0]):
        coordinate = (hit - vertices[0]) @ edge / (edge @ edge)
        good &= (coordinate >= 0) & (coordinate <= 1)
    good &= (distance > .05) & inliers
    if not good.any():
        return {"reference_plane_points": 0, "transfer_error_median_px": None}
    error = np.linalg.norm(_project(np.array(models["homography"]), points[good]) - _project(h, points[good]), axis=1)
    return {"reference_plane_points": int(good.sum()), "transfer_error_median_px": float(np.median(error)),
            "transfer_error_max_px": float(error.max()), "scope": "audit-only image transfer, not metric pose error"}


def audit_experiment(generation_root, prediction_root, output_path):
    generation_root, prediction_root = Path(generation_root), Path(prediction_root)
    # This must be opened before any audit truth, establishing phase ordering.
    phase = json.loads((prediction_root / "phase-complete.json").read_text())
    index = json.loads((generation_root / "public-index.json").read_text())["sequences"]
    statuses = {r["sequence_id"]: r["exit_code"] for r in phase["sequences"]}
    if set(statuses) != {s["sequence_id"] for s in index}:
        raise ValueError("Prediction phase incomplete")
    rows, errors, totals = [], [], Counter()
    for sequence in index:
        identity = sequence["sequence_id"]
        public = Path(sequence["public_root"])
        case = json.loads((public.parent / "audit/case.json").read_text())
        truth = json.loads((public.parent / "audit/truth.json").read_text())
        explicit_mismatch = case["declared_calibration_id"] != CALIBRATION_ID
        row = {"sequence_id": identity, "case": case, "predictor_exit_code": statuses[identity],
               "diagnostic_status_counts": {}, "views": {}, "single_plane_transfer": [],
               "expected_explicit_calibration_rejection": explicit_mismatch}
        if statuses[identity] != 0:
            stderr = (prediction_root / identity / "stderr.log").read_text()
            row["input_rejected"] = explicit_mismatch and statuses[identity] == 2 and "Unsupported sequence version or calibration" in stderr
            if not row["input_rejected"]:
                errors.append({"sequence_id": identity, "reason": "unexpected predictor exit", "stderr": stderr})
            rows.append(row)
            continue
        if explicit_mismatch:
            errors.append({"sequence_id": identity, "reason": "explicit wrong ID was not rejected"})
        logs = prediction_root / identity / "diagnostics/windows.jsonl"
        records = [json.loads(line) for line in logs.read_text().splitlines()]
        row["diagnostic_status_counts"] = dict(Counter(r["result"]["diagnostic_status"] for r in records))
        totals.update(row["diagnostic_status_counts"])
        for view in ("front", "left", "right"):
            row["views"][view] = {"status_counts": dict(Counter(r["result"]["views"][view]["status"] for r in records)),
                "reason_counts": dict(Counter(reason for r in records for reason in r["result"]["views"][view]["reasons"]))}
        for record in records:
            result = record["result"]
            packet, review = result["evidence_packet"], result["review"]
            if (review["motion_authorized"] is not False or review["arrival_status"] != "unverified" or
                    packet["corridor_boundary"]["state"] != "unknown" or packet["front_surface"]["state"] != "unknown" or
                    packet["corridor_boundary"]["offset_m"] is not None or packet["front_surface"]["range_m"] is not None):
                errors.append({"sequence_id": identity, "reason": "unsound physical/arrival claim"})
            if case["scene"] in {"wall", "ground"}:
                for view, diagnostic in result["views"].items():
                    metric = _single_plane_audit(truth, record, view, diagnostic)
                    if metric is not None:
                        row["single_plane_transfer"].append({"window_index": record["index"], "view": view,
                                                            "diagnostic_status": diagnostic["status"], **metric})
        rows.append(row)
    degeneracies = [r for r in rows if r["case"]["motion"] in {"static", "yaw", "tiny_forward"} or
                    r["case"]["texture"] in {"uniform", "repeated"}]
    result = {"scope": "targeted frozen-probe observability experiment, not navigation or generalization success",
              "case_count": len(rows), "processed_window_count": sum(totals.values()), "diagnostic_status_counts": dict(totals),
              "explicit_calibration_rejections": sum(r.get("input_rejected", False) for r in rows),
              "degeneracy_cases": len(degeneracies),
              "degeneracy_candidate_windows": sum(r["diagnostic_status_counts"].get("planar_flow_candidate", 0) for r in degeneracies),
              "errors": errors, "physical_geometry_status": "unknown", "arrival_status": "unverified", "cases": rows}
    path = Path(output_path)
    with path.open("x") as file:
        file.write(json.dumps(result, indent=2, allow_nan=False) + "\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", required=True, type=Path)
    parser.add_argument("--freeze", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    verify_freeze(args.freeze)
    args.output.mkdir(parents=True, exist_ok=False)
    generated = generate_suite(args.suite, args.output / "generated")
    predict_index(args.output / "generated/public-index.json", args.output / "predictions")
    result = audit_experiment(args.output / "generated", args.output / "predictions", args.output / "audit-result.json")
    verify_freeze(args.freeze)
    print(json.dumps({k: v for k, v in result.items() if k != "cases"}, indent=2))
    if generated["errors"]:
        print(json.dumps(generated, indent=2))
    return 1 if result["errors"] or generated["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
