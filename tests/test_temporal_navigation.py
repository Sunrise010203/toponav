import copy
from collections import deque
import json
from pathlib import Path
import subprocess
import sys

import pytest

from simulation.navigation_session import SessionConfig
from simulation.temporal_navigation import run_navigation

ROOT = Path(__file__).resolve().parents[1]


class Clock:
    def __init__(self):
        self.mono, self.wall = 0.0, 1_000_000_000_000

    def sleep(self, seconds):
        self.mono += seconds
        self.wall += int(seconds * 1e9)


def inputs():
    return [json.loads((ROOT / "configs" / name).read_text()) for name in ("graph.json", "scene.json")]


def run(**kwargs):
    c = Clock()
    return run_navigation(*inputs(), clock=lambda: c.mono, wall_ns=lambda: c.wall,
                          sleep=c.sleep, poll_interval_s=kwargs.pop("poll_interval_s", 0.1),
                          session_id="deterministic-test-session", **kwargs)


def test_ideal_session_reaches_with_three_frames_per_station_and_fresh_final_confirmation():
    r = run()
    assert r["audit"]["goal_verified"]
    assert r["audit"]["actual_moves"] == 4
    assert r["controller"]["metrics"]["frames_received"] == 15
    assert r["audit"]["mislocalized_commands"] == 0


@pytest.mark.parametrize("scenario", ["uncertain_once", "mismatch_once", "stale_once", "future_once",
                                       "replay_once", "out_of_order_once", "drop_once",
                                       "execution_failed_once", "late_success", "duplicate_feedback"])
def test_recoverable_faults_reach_without_false_arrival_or_wrong_commands(scenario):
    r = run(scenario=scenario)
    assert r["audit"]["goal_verified"]
    assert r["audit"]["mislocalized_commands"] == 0
    assert not r["audit"]["false_arrival"]
    assert r["controller"]["metrics"]["transitions_applied"] == 4


@pytest.mark.parametrize("scenario,reason", [("execution_unknown", "stage_timeout"),
                                             ("execution_sent", "stage_timeout"),
                                             ("drop_all", "stage_timeout"),
                                             ("mismatch_persistent", "conflict_limit"),
                                             ("execution_failed_always", "execution_failure_limit"),
                                             ("conflicting_feedback", "feedback_protocol_error")])
def test_unrecoverable_faults_stop_with_bounded_failure(scenario, reason):
    r = run(scenario=scenario)
    assert r["state"] == "failed" and r["reason"] == reason
    assert not r["audit"]["goal_verified"]
    assert r["controller"]["metrics"]["commands_sent"] <= 2


def test_detour_unreachable_and_no_geometry():
    assert run(disabled_channels=("C02",))["audit"]["goal_verified"]
    r = run(disabled_channels=("C06",))
    assert r["reason"] == "unreachable" and r["audit"]["actual_moves"] == 0
    graph, scene = inputs()
    scene.pop("geometry")
    r = run_navigation(graph, scene, poll_interval_s=0.001)
    assert r["audit"]["goal_verified"] and r["audit"]["final_truth"]["position_m"] is None


def test_controller_trace_independent_of_actual_initial_truth_when_input_is_fixed():
    class FixedSensor:
        def capture(self, index):
            return {"frame_id": str(index), "captured_at_ns": 1_000_000_000_001 + index,
                    "observation": {"relative_open": {"front": True, "left": False,
                                                       "right": False, "back": False}, "confidence": 0.2}}, None

        def close(self):
            pass
    a = run(sensor_factory=lambda env: FixedSensor())
    b = run(sensor_factory=lambda env: FixedSensor(), actual_start="N06", actual_heading=180)
    assert a["controller"] == b["controller"]
    assert a["audit"]["final_truth"] != b["audit"]["final_truth"]


def test_driver_cycle_limit_stops_and_inputs_are_preserved():
    g, s = inputs()
    before = copy.deepcopy((g, s))
    r = run_navigation(g, s, max_cycles=1)
    assert r["reason"] == "driver_cycle_limit" and (g, s) == before


def test_image_session_never_calls_ideal_sensor_and_recovers_missing_view(monkeypatch):
    pytest.importorskip("pybullet")
    from simulation.structural_navigation import StructuralEnvironment
    from roadvision.temporal_navigation import run_navigation as image_run
    from roadvision.learn import Predictor
    def forbidden(*args, **kwargs):
        raise AssertionError("Ideal sensor is forbidden in image session")
    monkeypatch.setattr(StructuralEnvironment, "observe", forbidden)
    predictor = Predictor(ROOT / "models/road_structure_frozen_v1_20261014")
    r = image_run(predictor, "test_locked", "R000", 0, "R007", visual_fault="missing_view",
                  poll_interval_s=0.001)
    assert r["audit"]["goal_verified"]
    assert r["controller"]["metrics"]["recoveries_succeeded"] >= 1
    assert r["steps"][0]["prediction"]["rejection_reason"] == "missing_view"


def test_cli_exit_codes_and_nonoverwriting_outputs(tmp_path):
    destination = tmp_path / "new.json"
    command = [sys.executable, "-m", "simulation.temporal_navigation", "--output", str(destination),
               "--poll-interval", "0.001"]
    first = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    assert first.returncode == 0, first.stderr
    before = destination.read_bytes()
    assert subprocess.run(command, cwd=ROOT, capture_output=True).returncode == 2
    assert destination.read_bytes() == before
    failed = subprocess.run([sys.executable, "-m", "simulation.temporal_navigation", "--disable", "C06",
                             "--poll-interval", "0.001"], cwd=ROOT, capture_output=True)
    assert failed.returncode == 1


def test_policy_and_scenario_validation():
    with pytest.raises(ValueError):
        run(scenario="not-a-scenario")
    with pytest.raises(ValueError):
        run(poll_interval_s=-1)


@pytest.mark.parametrize("mask", range(64))
def test_all_closures_and_start_goal_pairs_against_independent_bfs(mask):
    g, scene = inputs()
    disabled = [e["channel_id"] for i, e in enumerate(g["edges"]) if mask & (1 << i)]
    neighbors = {n["node_id"]: [] for n in g["nodes"]}
    for edge in g["edges"]:
        if edge["enabled"] and edge["channel_id"] not in disabled:
            neighbors[edge["source"]].append(edge["target"])
            neighbors[edge["target"]].append(edge["source"])
    for start in neighbors:
        distances, queue = {start: 0}, deque([start])
        while queue:
            node = queue.popleft()
            for other in neighbors[node]:
                if other not in distances:
                    distances[other] = distances[node] + 1
                    queue.append(other)
        for goal in neighbors:
            r = run(start=start, goal=goal, disabled_channels=disabled)
            assert r["audit"]["goal_verified"] == (goal in distances)
            assert not r["audit"]["false_arrival"] and r["audit"]["mislocalized_commands"] == 0
            if goal in distances:
                assert r["audit"]["actual_moves"] == distances[goal]
                assert r["controller"]["metrics"]["frames_received"] == 3 * (distances[goal] + 1)
            else:
                assert r["reason"] == "unreachable"


@pytest.mark.parametrize("heading", [0, 90, 180, 270])
def test_all_headings_and_tasks_keep_turn_windows_separate(heading):
    nodes = [n["node_id"] for n in inputs()[0]["nodes"]]
    for start in nodes:
        for goal in nodes:
            r = run(start=start, goal=goal, heading=heading)
            assert r["audit"]["goal_verified"] and r["audit"]["mislocalized_commands"] == 0
            issued = [h for h in r["controller"]["history"] if h["event"] == "command_issued"]
            used = [identity for h in issued for identity in h["frame_ids"]]
            assert len(set(used)) == len(used)


def test_false_arrival_is_exposed_not_corrected_by_truth_audit():
    r = run(start="N01", goal="N01", heading=0, actual_start="N06", actual_heading=180)
    assert r["state"] == "reached"
    assert r["audit"]["false_arrival"] and not r["audit"]["goal_verified"]
    assert r["audit"]["actual_moves"] == 0


def test_camera_failure_during_capture_requests_stop_and_closes_sensor(monkeypatch):
    from simulation.structural_navigation import StructuralEnvironment
    closed, stops = [], []
    original = StructuralEnvironment.execute
    def track(self, action, *args):
        stops.append(action)
        return original(self, action, *args)
    monkeypatch.setattr(StructuralEnvironment, "execute", track)
    class BrokenSensor:
        def capture(self, index):
            raise RuntimeError("camera disconnected")
        def close(self):
            closed.append(True)
    with pytest.raises(RuntimeError, match="camera disconnected"):
        run(sensor_factory=lambda env: BrokenSensor())
    assert closed and stops[-1] == "stop"


def test_independent_log_audit_detects_duplicate_completion_and_bad_confirmation():
    from tools.verify_temporal_acceptance import audit
    result = run()
    assert audit(result) == []
    consumed = next(h for h in result["controller"]["history"] if h["event"] == "completion_consumed")
    result["controller"]["history"].append(copy.deepcopy(consumed))
    assert audit(result)
