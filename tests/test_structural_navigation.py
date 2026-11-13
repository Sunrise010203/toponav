import copy
import inspect
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest

from simulation.dijkstra import plan
from simulation.structural_localization import decide, update_candidates
from simulation.structural_navigation import StructuralEnvironment, run_navigation


ROOT = Path(__file__).resolve().parents[1]
DIRECTIONS = ("front", "left", "right", "back")


def inputs(name="graph.json"):
    return [json.loads((ROOT / "configs" / "legacy" / path).read_text(encoding="utf-8"))
            for path in (name, "scene.json")]


def pose(node, heading=0):
    return {"node_id": node, "heading_deg": heading}


def observation(*opened, confidence=1.0):
    return {"relative_open": {d: d in opened for d in DIRECTIONS}, "confidence": confidence}


def pairs(candidates):
    return {(p["node_id"], p["heading_deg"]) for p in candidates}


def test_observation_is_relative_structure_not_identity():
    graph, scene = inputs()
    a = StructuralEnvironment(graph, scene, start="N02", heading=0)
    b = StructuralEnvironment(graph, scene, start="N04", heading=180)
    assert a.observe() == b.observe() == observation("front", "left", "back")
    assert set(a.observe()) == {"relative_open", "confidence"}
    assert set(a.observe()["relative_open"]) == set(DIRECTIONS)
    assert "N02" not in json.dumps(a.observe())
    assert "N04" not in json.dumps(b.observe())


def test_same_structure_does_not_identify_same_place():
    graph, _ = inputs()
    prior = [pose("N02"), pose("N04", 180)]
    result = update_candidates(graph, prior, observation("front", "left", "back"))
    assert result["status"] == "ambiguous"
    assert pairs(result["candidates"]) == {("N02", 0), ("N04", 180)}
    assert result["estimate"] is None
    assert decide(graph, result, "N06")["action"] == "stop"


def test_success_predicts_from_history_before_matching_structure():
    graph, _ = inputs()
    result = update_candidates(graph, [pose("N01")], observation("front", "left", "back"),
                               previous_action="front", feedback={"status": "succeeded"})
    assert result["status"] == "localized"
    assert result["candidates"] == [pose("N02")]
    assert pose("N04", 180) not in result["candidates"]


def test_later_structure_can_resolve_multiple_same_structure_hypotheses():
    graph, _ = inputs()
    prior = [pose("N02"), pose("N04", 180)]
    # front from N02 -> N03/east (left+back); front from N04/west -> N05/west (left+back).
    middle = update_candidates(graph, prior, observation("left", "back"), "front",
                               {"status": "succeeded"})
    assert middle["status"] == "ambiguous"
    assert pairs(middle["candidates"]) == {("N03", 0), ("N05", 180)}
    # left -> N04/north (left+right+back), or N02/south (left+right+back): still not unique.
    later = update_candidates(graph, middle["candidates"], observation("left", "right", "back"),
                              "left", {"status": "succeeded"})
    assert later["status"] == "ambiguous"
    assert len(later["candidates"]) == 2
    # A different pair with different outgoing topology can be resolved by a later observation.
    resolved = update_candidates(graph, [pose("N01"), pose("N05")],
                                 observation("front", "left", "back"), "front",
                                 {"status": "succeeded"})
    assert resolved["candidates"] == [pose("N02")]


def test_failed_execution_preserves_prior_and_stops():
    graph, _ = inputs()
    prior = [pose("N02")]
    result = update_candidates(graph, prior, observation("front", "left", "back"),
                               "front", {"status": "failed"})
    assert result["predicted_candidates"] == prior
    assert result["candidates"] == prior
    assert result["estimate"] == pose("N02")
    assert result["status"] == "execution_failed"
    assert decide(graph, result, "N06")["action"] == "stop"


@pytest.mark.parametrize("feedback", [None, {"status": "sent"}, {"status": "unknown"}])
def test_unconfirmed_execution_does_not_assume_success(feedback):
    graph, _ = inputs()
    unclear = {"relative_open": {d: None for d in DIRECTIONS}, "confidence": 0.2}
    result = update_candidates(graph, [pose("N02")], unclear, "front", feedback)
    assert pairs(result["predicted_candidates"]) == {("N02", 0), ("N03", 0)}
    assert pairs(result["candidates"]) == {("N02", 0), ("N03", 0)}
    assert result["status"] == "execution_unconfirmed"
    assert result["estimate"] is None
    assert decide(graph, result, "N06")["action"] == "stop"


def test_unknown_feedback_still_stops_when_observation_resolves_position():
    graph, _ = inputs()
    result = update_candidates(graph, [pose("N02")], observation("left", "back"),
                               "front", {"status": "unknown"})
    assert pairs(result["predicted_candidates"]) == {("N02", 0), ("N03", 0)}
    assert result["estimate"] == pose("N03")
    assert result["status"] == "execution_unconfirmed"


def test_impossible_success_is_transition_mismatch_not_silent_stay():
    graph, _ = inputs()
    result = update_candidates(graph, [pose("N01")], observation("front"),
                               "left", {"status": "succeeded"})
    assert result["status"] == "transition_mismatch"
    assert result["predicted_candidates"] == []
    assert result["candidates"] == []


def test_mismatch_does_not_jump_to_matching_node_elsewhere():
    graph, _ = inputs()
    # N02's structure cannot be used to reset an N01 prior to N02 or N04.
    result = update_candidates(graph, [pose("N01")], observation("front", "left", "back"))
    assert result["status"] == "observation_mismatch"
    assert result["candidates"] == []
    assert result["estimate"] is None
    assert decide(graph, result, "N06")["action"] == "stop"


def test_partial_observation_filters_only_known_directions_but_stops():
    graph, _ = inputs()
    partial = observation("front")
    partial["relative_open"]["back"] = None
    result = update_candidates(graph, [pose("N01"), pose("N06", 180)], partial)
    assert pairs(result["candidates"]) == {("N01", 0), ("N06", 180)}
    assert result["status"] == "observation_uncertain"
    assert result["estimate"] is None


def test_low_confidence_does_not_eliminate_prior_using_unreliable_bits():
    graph, _ = inputs()
    result = update_candidates(graph, [pose("N01")], observation(confidence=0.2))
    assert result["candidates"] == [pose("N01")]
    assert result["status"] == "observation_uncertain"
    assert decide(graph, result, "N06")["action"] == "stop"


@pytest.mark.parametrize("kind", ["extra_truth", "bad_confidence", "missing_direction", "nonboolean"])
def test_invalid_or_truth_bearing_observations_are_rejected(kind):
    obs = observation("front")
    if kind == "extra_truth":
        obs["node_id"] = "N01"
    elif kind == "bad_confidence":
        obs["confidence"] = math.nan
    elif kind == "missing_direction":
        del obs["relative_open"]["back"]
    else:
        obs["relative_open"]["front"] = 1
    with pytest.raises(ValueError):
        update_candidates(inputs()[0], [pose("N01")], obs)


@pytest.mark.parametrize("feedback", [{"status": "accepted"}, {"status": "succeeded", "node_id": "N02"},
                                      {"status": "succeeded", "position_m": [2, 0]}])
def test_feedback_cannot_smuggle_truth_or_treat_acceptance_as_completion(feedback):
    with pytest.raises(ValueError):
        update_candidates(inputs()[0], [pose("N01")], observation("front"), "front", feedback)


def test_ideal_run_has_separate_truth_estimate_observation_and_feedback():
    graph, scene = inputs()
    original = copy.deepcopy((graph, scene))
    result = run_navigation(graph, scene)
    assert result["status"] == "reached"
    assert result["audit"]["goal_verified"] is True
    assert result["audit"]["visited_nodes"] == ["N01", "N02", "N03", "N04", "N06"]
    assert result["audit"]["actual_moves"] == 4
    assert result["final_truth"]["position_m"] == [6, 2]
    assert result["final_truth"]["distance_m"] == 8
    assert len(result["steps"]) == 5
    assert [s["observation"] for s in result["steps"]] == [
        observation("front"), observation("front", "left", "back"), observation("left", "back"),
        observation("left", "right", "back"), observation("back")]
    assert [s["decision"]["action"] for s in result["steps"]] == [
        "straight", "straight", "left", "right", "stop"]
    for index, step in enumerate(result["steps"]):
        assert set(step["observation"]) == {"relative_open", "confidence"}
        assert set(step["execution_feedback"]) == {"status"}
        assert step["localization"]["estimate"]["node_id"] == step["truth"]["before"]["node_id"]
        if step["truth"]["execution"] == "moved":
            assert step["truth"]["before"]["position_m"] != step["truth"]["after"]["position_m"]
            following = result["steps"][index + 1]
            assert following["truth"]["before"] == step["truth"]["after"]
            assert following["feedback_used"] == step["execution_feedback"]
            assert following["prior_candidates"] == step["localization"]["candidates"]
    assert result["steps"][-1]["localization"]["estimate"] == pose("N06")
    assert result["steps"][-1]["decision"]["action"] == "stop"
    assert (graph, scene) == original


@pytest.mark.parametrize("scenario,status,actual_node,estimated_node,moves", [
    ("execution_failed", "execution_failed", "N02", "N02", 1),
    ("execution_unknown", "execution_unconfirmed", "N03", "N03", 2),
    ("execution_sent", "execution_unconfirmed", "N02", "N02", 1),
    ("ambiguous", "observation_uncertain", "N02", None, 1),
    ("mismatch", "observation_mismatch", "N02", None, 1),
])
def test_fault_scenarios_have_explicit_safe_stop(scenario, status, actual_node, estimated_node, moves):
    result = run_navigation(*inputs(), scenario=scenario, fault_step=1)
    assert result["status"] == status
    assert result["final_truth"]["node_id"] == actual_node
    assert result["audit"]["actual_moves"] == moves
    assert not result["audit"]["goal_verified"]
    terminal = result["steps"][-1]
    estimate = terminal["localization"]["estimate"]
    assert (estimate["node_id"] if estimate else None) == estimated_node
    assert terminal["decision"]["action"] == "stop"
    assert terminal["truth"]["before"] == terminal["truth"]["after"]


@pytest.mark.parametrize("outcome", ["failed", "sent"])
def test_failed_or_unexecuted_relative_turn_does_not_change_truth_heading(outcome):
    env = StructuralEnvironment(*inputs(), start="N02", heading=0)
    before = env.state()
    assert env.execute("left", outcome=outcome) == {"status": outcome}
    assert env.state() == before


def test_executor_uses_true_heading_without_exposing_it_in_feedback():
    env = StructuralEnvironment(*inputs(), start="N02", heading=90)
    # Relative right is east at the actual heading 90, not north at some estimated heading.
    feedback = env.execute("right")
    assert feedback == {"status": "succeeded"}
    assert env.state()["node_id"] == "N03"
    assert env.state()["heading_deg"] == 0


def test_task_prior_is_not_initialized_or_corrected_from_truth():
    graph, scene = inputs()
    a = run_navigation(graph, scene, start="N01", heading=0, goal="N01")
    # N06/west and N01/east have the same observed dead-end structure.
    b = run_navigation(graph, scene, start="N01", heading=0, goal="N01",
                       actual_start="N06", actual_heading=180)
    assert a["status"] == b["status"] == "reached"  # controller verdict, independent of truth
    assert a["steps"][0]["observation"] == b["steps"][0]["observation"]
    assert a["steps"][0]["localization"] == b["steps"][0]["localization"]
    assert a["steps"][0]["decision"] == b["steps"][0]["decision"]
    assert a["audit"]["goal_verified"] is True
    assert b["audit"]["goal_verified"] is False
    assert b["final_estimate"] == pose("N01")  # must not reset to true N06


def test_localizer_and_decision_interfaces_have_no_truth_parameters():
    assert set(inspect.signature(update_candidates).parameters) == {
        "graph", "previous_candidates", "observation", "previous_action", "feedback", "disabled_channels"}
    assert set(inspect.signature(decide).parameters) == {"graph", "localization", "goal", "disabled_channels"}


def test_audit_truth_cannot_seed_advance_repair_or_stop_controller(monkeypatch):
    original = run_navigation(*inputs())

    def audit_only_state(self):
        return {"node_id": "AUDIT_ONLY", "position_m": [999, 999],
                "heading_deg": 270, "distance_m": 12345}

    # Sensor and physics still own the real internal world. Spoof only the truth
    # exposed to logging/audit; controller trajectories must be unaffected.
    monkeypatch.setattr(StructuralEnvironment, "state", audit_only_state)
    spoofed = run_navigation(*inputs())
    assert spoofed["status"] == "reached"
    assert spoofed["final_estimate"] == original["final_estimate"]
    for a, b in zip(original["steps"], spoofed["steps"]):
        for key in ("prior_candidates", "observation", "localization", "decision", "feedback_used"):
            assert a[key] == b[key]
    assert len(spoofed["steps"]) == len(original["steps"])
    assert spoofed["audit"]["goal_verified"] is False


def test_driver_passes_only_public_observation_feedback_and_previous_estimate(monkeypatch):
    import simulation.structural_navigation as driver
    original_update = driver.update_candidates
    calls = []

    def checked(graph, previous_candidates, observation, previous_action=None, feedback=None,
                disabled_channels=()):
        assert set(observation) == {"relative_open", "confidence"}
        assert feedback is None or set(feedback) == {"status"}
        assert all(set(p) == {"node_id", "heading_deg"} for p in previous_candidates)
        result = original_update(graph, previous_candidates, observation, previous_action,
                                 feedback, disabled_channels)
        if calls:
            assert previous_candidates == calls[-1]["candidates"]
        else:
            assert previous_candidates == [pose("N01")]
        calls.append(result)
        return result

    monkeypatch.setattr(driver, "update_candidates", checked)
    assert driver.run_navigation(*inputs())["audit"]["goal_verified"]
    assert len(calls) == 5


@pytest.mark.parametrize("heading", [0, 90, 180, 270])
def test_all_pairs_for_each_start_heading(heading):
    graph, scene = inputs()
    ids = [n["node_id"] for n in graph["nodes"]]
    for start, goal in itertools.product(ids, repeat=2):
        result = run_navigation(graph, scene, start=start, heading=heading, goal=goal)
        assert result["status"] == "reached"
        assert result["audit"]["goal_verified"]
        assert result["final_truth"]["distance_m"] == plan(graph, start, goal)["length_m"]


@pytest.mark.parametrize("mask", range(64))
def test_all_closure_subsets_and_node_pairs(mask):
    graph, scene = inputs()
    disabled = [e["channel_id"] for i, e in enumerate(graph["edges"]) if mask & (1 << i)]
    ids = [n["node_id"] for n in graph["nodes"]]
    for start, goal in itertools.product(ids, repeat=2):
        expected = plan(graph, start, goal, disabled)
        result = run_navigation(graph, scene, start=start, goal=goal, disabled_channels=disabled)
        assert result["status"] == ("reached" if expected["status"] == "reachable" else "unreachable")
        if result["status"] == "reached":
            assert result["audit"]["goal_verified"]
            assert result["final_truth"]["distance_m"] == expected["length_m"]


def test_detour_limits_and_start_at_goal():
    graph, scene = inputs()
    assert run_navigation(graph, scene, disabled_channels=["C02"])["audit"]["visited_nodes"] == [
        "N01", "N02", "N05", "N04", "N06"]
    assert run_navigation(*inputs("graph_blocked_C02.json"))["audit"]["goal_verified"]
    limited = run_navigation(graph, scene, max_steps=2)
    assert limited["status"] == "step_limit"
    assert limited["final_estimate"] == pose("N03")
    assert run_navigation(graph, scene, max_steps=4)["status"] == "reached"
    assert run_navigation(graph, scene, max_steps=0)["status"] == "step_limit"
    assert run_navigation(graph, scene, start="N06", max_steps=0)["audit"]["goal_verified"]


@pytest.mark.parametrize("args,code,status", [
    ([], 0, "reached"), (["--disable", "C02"], 0, "reached"),
    (["--disable", "C06"], 1, "unreachable"),
    (["--scenario", "execution_failed"], 1, "execution_failed"),
    (["--scenario", "execution_unknown"], 1, "execution_unconfirmed"),
    (["--scenario", "execution_sent"], 1, "execution_unconfirmed"),
    (["--scenario", "ambiguous"], 1, "observation_uncertain"),
    (["--scenario", "mismatch"], 1, "observation_mismatch"),
    (["--heading", "45"], 2, "invalid"), (["--start", "N99"], 2, "invalid"),
])
def test_cli_output_and_codes(tmp_path, args, code, status):
    output = tmp_path / "structural.json"
    process = subprocess.run([sys.executable, "-m", "simulation.structural_navigation", *args,
                              "--output", str(output)], cwd=ROOT,
                             capture_output=True, text=True, timeout=10)
    assert process.returncode == code, process.stderr
    result = json.loads(process.stdout)
    assert result["status"] == status
    if status != "invalid":
        assert json.loads(output.read_text()) == result
