"""Reproduce predefined temporal fault cases; exclusively create a new run folder."""
import argparse
import hashlib
import json
import platform
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

# Fixed before running the fault experiments; no selection based on results.
RECOVERABLE = ("uncertain_once", "mismatch_once", "stale_once", "future_once", "replay_once",
               "out_of_order_once", "drop_once", "execution_failed_once", "late_success", "duplicate_feedback")
STOPPING = ("execution_unknown", "execution_sent", "drop_all", "mismatch_persistent",
            "execution_failed_always", "conflicting_feedback")


def audit(run):
    """Independent event-log checks, not fed back to the running controller."""
    completed, pending, frames, issues = set(), None, set(), []
    frame_by_id = {s["frame"]["frame_id"]: s["frame"] for s in run["steps"] if s["frame"] is not None}
    for event in run["controller"]["history"]:
        if event["event"] == "command_issued":
            if pending is not None:
                issues.append("command issued while previous command unresolved")
            pending = event["command_id"]
            ids = event["frame_ids"]
            if len(ids) != run["controller"]["config"]["confirmation_frames"] or frames.intersection(ids):
                issues.append("invalid or reused confirmation evidence")
            frames.update(ids)
            observations = [frame_by_id[identity]["observation"] for identity in ids]
            if any(o["confidence"] < 0.6 or any(v is None for v in o["relative_open"].values()) for o in observations):
                issues.append("uncertain frame authorized motion")
            if any(o["relative_open"] != observations[0]["relative_open"] for o in observations):
                issues.append("inconsistent frames authorized motion")
        elif event["event"] == "completion_consumed" and event["consumed"]:
            command = event["feedback_event"]["command_id"]
            if command in completed or command != pending:
                issues.append("completion consumed twice or for wrong command")
            completed.add(command)
            pending = None
    if run["audit"]["false_arrival"]:
        issues.append("false arrival")
    if run["audit"]["mislocalized_commands"]:
        issues.append("motion authorized from incorrect pose estimate")
    return issues


def hashes():
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (ROOT / "simulation/navigation_session.py", ROOT / "simulation/temporal_navigation.py",
                      ROOT / "roadvision/temporal_navigation.py", Path(__file__).resolve())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    import torch
    from roadvision.learn import Predictor
    from roadvision.temporal_navigation import run_navigation
    torch.set_num_threads(2)
    version = hashes()
    model_dir = ROOT / "models/road_structure_frozen_v1_20261014"
    predictor = Predictor(model_dir, "cpu")
    cases = [(name, {"scenario": name}, True) for name in RECOVERABLE]
    cases += [(name, {"scenario": name}, False) for name in STOPPING]
    cases += [("visual_" + fault, {"visual_fault": fault}, True)
              for fault in ("missing_view", "stale_frame", "future_frame")]
    rows = []
    started = time.monotonic()
    for name, options, expected_reached in cases:
        run = run_navigation(predictor, "test_locked", "R000", 0, "R007", **options)
        with (args.output / f"{name}.json").open("x") as file:
            json.dump(run, file, indent=2, allow_nan=False)
        metrics = run["controller"]["metrics"]
        row = {"case": name, "expected_reached": expected_reached, "state": run["state"],
               "reason": run["reason"], "goal_verified": run["audit"]["goal_verified"],
               "false_arrival": run["audit"]["false_arrival"],
               "mislocalized_commands": run["audit"]["mislocalized_commands"],
               "commands_sent": metrics["commands_sent"], "actual_moves": run["audit"]["actual_moves"],
               "stop_requests": run["audit"]["stop_requests"], "waiting_time_s": run["waiting_time_s"],
               "recoveries_started": metrics["recoveries_started"],
               "recoveries_succeeded": metrics["recoveries_succeeded"],
               "recoveries_failed": metrics["recoveries_failed"],
               "recovery_durations_s": metrics["recovery_durations_s"], "audit_issues": audit(run)}
        row["expectation_passed"] = (row["goal_verified"] == expected_reached and not row["audit_issues"]
                                     and (expected_reached or row["state"] == "failed"))
        rows.append(row)
        print(json.dumps(row), flush=True)
    summary = {"run_id": args.output.name, "cases": rows, "all_expectations_passed": all(
        r["expectation_passed"] for r in rows), "code_unchanged_during_run": hashes() == version,
        "source_hashes": version, "model_sha256": predictor.metadata["model_sha256"],
        "manifest_sha256": predictor.metadata["manifest_sha256"], "python": platform.python_version(),
        "torch": torch.__version__, "torch_threads": torch.get_num_threads(), "device": "cpu",
        "elapsed_s": time.monotonic() - started,
        "scope": "Predefined single-factor engineering injections on exposed test_locked; not independent generalization. Output faults are injected AFTER inference, not model improvements."}
    with (args.output / "summary.json").open("x") as file:
        json.dump(summary, file, indent=2)
    return 0 if summary["all_expectations_passed"] and summary["code_unchanged_during_run"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
