import copy
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest

from simulation.dijkstra import plan
from simulation.navigation import Environment2D, SimulatedRecognizer, decide, localize, run_navigation


ROOT = Path(__file__).resolve().parents[1]


def inputs(map_name="graph.json"):
    graph = json.loads((ROOT / "configs" / "legacy" / map_name).read_text(encoding="utf-8"))
    scene = json.loads((ROOT / "configs" / "legacy" / "scene.json").read_text(encoding="utf-8"))
    return graph, scene


def assert_fresh_observations(result, scene):
    positions = {node["node_id"]: node["position_m"] for node in scene["nodes"]}
    for index, record in enumerate(result["steps"]):
        before, after = record["before"], record["after"]
        assert before["position_m"] == positions[before["node_id"]]
        assert after["position_m"] == positions[after["node_id"]]
        assert record["probabilities"][before["node_id"]] == 1
        assert sum(record["probabilities"].values()) == 1
        assert record["localization"]["node_id"] == before["node_id"]
        if record["execution"] == "moved":
            assert before["node_id"] != after["node_id"]
            assert before["position_m"] != after["position_m"]
            assert result["steps"][index + 1]["before"] == after
            assert result["steps"][index + 1]["probabilities"][after["node_id"]] == 1
        else:
            assert before == after


def test_action_changes_actual_position_and_next_observation():
    graph, scene = inputs()
    env = Environment2D(graph, scene)
    recognizer = SimulatedRecognizer(graph)
    observation = recognizer.observe(env)
    assert observation["N01"] == 1
    assert env.state()["position_m"] == [0, 0]
    assert env.step("east") == "moved"
    assert env.state() == {"node_id": "N02", "position_m": [2, 0],
                           "heading_deg": 0, "distance_m": 2}
    next_observation = recognizer.observe(env)
    assert next_observation["N02"] == 1
    assert next_observation["N01"] == 0
    assert observation["N01"] == 1  # observations are snapshots, not live references


def test_ideal_closed_loop_reaches_goal_and_observes_it():
    graph, scene = inputs()
    original = copy.deepcopy((graph, scene))
    result = run_navigation(graph, scene)
    assert result["status"] == "reached"
    assert result["visited_nodes"] == ["N01", "N02", "N03", "N04", "N06"]
    assert result["moves"] == 4
    assert result["final_state"]["node_id"] == "N06"
    assert result["final_state"]["position_m"] == [6, 2]
    assert result["final_state"]["distance_m"] == 8
    assert result["steps"][-1]["decision"]["action"] == "stop"
    assert result["steps"][-1]["probabilities"]["N06"] == 1
    assert_fresh_observations(result, scene)
    assert (graph, scene) == original


@pytest.mark.parametrize("map_name,disabled", [
    ("graph.json", ["C02"]), ("graph_blocked_C02.json", []),
])
def test_closed_loop_detour(map_name, disabled):
    graph, scene = inputs(map_name)
    result = run_navigation(graph, scene, disabled_channels=disabled)
    assert result["status"] == "reached"
    assert result["visited_nodes"] == ["N01", "N02", "N05", "N04", "N06"]
    assert result["final_state"]["distance_m"] == 8
    assert_fresh_observations(result, scene)


def test_unreachable_stops_without_moving():
    graph, scene = inputs()
    result = run_navigation(graph, scene, disabled_channels=["C06"])
    assert result["status"] == "unreachable"
    assert result["moves"] == 0
    assert result["final_state"]["node_id"] == "N01"
    assert result["steps"][-1]["decision"]["action"] == "stop"
    assert_fresh_observations(result, scene)


def test_actions_use_true_position_not_planner_destination():
    graph, scene = inputs()
    env = Environment2D(graph, scene)
    mistaken = {"status": "localized", "node_id": "N02", "confidence": 1}
    command = decide(graph, mistaken, "N06", disabled_channels=["C02"])
    assert command["action"] == "north"  # from N02; no north exit exists at actual N01
    before = env.state()
    assert env.step(command["action"]) == "blocked"
    assert env.state() == before  # cannot teleport to the planner's next node N05


@pytest.mark.parametrize("disabled", [True, False])
def test_blocked_channel_is_not_traversed(disabled):
    graph, scene = inputs()
    if not disabled:
        graph["edges"][0]["enabled"] = False
    env = Environment2D(graph, scene, disabled_channels=["C01"] if disabled else [])
    before = env.state()
    assert env.step("east") == "blocked"
    assert env.state() == before


def test_physical_channel_is_required_even_if_map_lists_it():
    graph, scene = inputs()
    scene["channels"] = scene["channels"][1:]
    env = Environment2D(graph, scene)
    assert env.step("east") == "blocked"
    result = run_navigation(graph, scene)
    assert result["status"] == "blocked"
    assert result["moves"] == 0


def test_stop_and_invalid_action_do_not_move():
    graph, scene = inputs()
    env = Environment2D(graph, scene)
    before = env.state()
    assert env.step("stop") == "stopped"
    with pytest.raises(ValueError):
        env.step("teleport_N06")
    assert env.state() == before


def test_reverse_directions_and_one_way_channels():
    graph, scene = inputs()
    result = run_navigation(graph, scene, start="N06", goal="N01")
    assert result["status"] == "reached"
    assert [s["decision"]["action"] for s in result["steps"]] == [
        "west", "south", "west", "west", "stop"]
    assert result["final_state"]["heading_deg"] == 180
    graph["edges"][0]["bidirectional"] = False
    env = Environment2D(graph, scene, start="N02")
    assert env.step("west") == "blocked"
    assert run_navigation(graph, scene, start="N02", goal="N01")["status"] == "unreachable"


def test_localization_consumes_only_probability_distribution():
    assert localize({"N01": 0.1, "N02": 0.9}) == {
        "status": "localized", "node_id": "N02", "confidence": 0.9}
    assert localize({"N01": 0.5, "N02": 0.5}) == {
        "status": "uncertain", "node_id": None, "confidence": 0.5}
    command = decide(inputs()[0], localize({"N01": 0.5, "N02": 0.5}), "N06")
    assert command["status"] == "localization_uncertain"
    assert command["action"] == "stop"


@pytest.mark.parametrize("probabilities", [
    {}, {"N01": -0.1, "N02": 1.1}, {"N01": math.nan},
    {"N01": math.inf}, {"N01": 0}, {"N01": 0.2}, {"N01": "1"},
])
def test_invalid_probabilities(probabilities):
    with pytest.raises(ValueError):
        localize(probabilities)


def test_simulated_probabilities_and_low_confidence_stop():
    graph, scene = inputs()
    env = Environment2D(graph, scene, start="N04")
    distribution = SimulatedRecognizer(graph, confidence=0.8).observe(env)
    assert distribution["N04"] == 0.8
    assert distribution["N01"] == pytest.approx(0.04)
    assert sum(distribution.values()) == pytest.approx(1)
    assert localize(distribution)["node_id"] == "N04"
    result = run_navigation(graph, scene, confidence=1 / 6)
    assert result["status"] == "localization_uncertain"
    assert result["moves"] == 0
    assert run_navigation(graph, scene, confidence=0.8)["status"] == "reached"


@pytest.mark.parametrize("confidence", [-0.1, 1.1, math.nan, math.inf])
def test_invalid_recognition_confidence(confidence):
    with pytest.raises(ValueError):
        SimulatedRecognizer(inputs()[0], confidence)


def test_limits_and_start_at_goal():
    graph, scene = inputs()
    result = run_navigation(graph, scene, max_steps=2)
    assert result["status"] == "step_limit"
    assert result["moves"] == 2
    assert result["final_state"]["node_id"] == "N03"
    assert_fresh_observations(result, scene)
    assert run_navigation(graph, scene, max_steps=0)["moves"] == 0
    assert run_navigation(graph, scene, max_steps=4)["status"] == "reached"
    result = run_navigation(graph, scene, start="N06", max_steps=0)
    assert result["status"] == "reached"
    assert result["moves"] == 0
    assert result["final_state"]["position_m"] == [6, 2]


@pytest.mark.parametrize("kwargs", [{"start": "N99"}, {"goal": "N99"},
                                   {"disabled_channels": ["C99"]}, {"max_steps": -1}])
def test_invalid_run_arguments(kwargs):
    with pytest.raises(ValueError):
        run_navigation(*inputs(), **kwargs)


@pytest.mark.parametrize("mask", range(64))
def test_all_closures_and_start_goal_pairs(mask):
    graph, scene = inputs()
    disabled = [edge["channel_id"] for i, edge in enumerate(graph["edges"]) if mask & (1 << i)]
    ids = [node["node_id"] for node in graph["nodes"]]
    for start, goal in itertools.product(ids, repeat=2):
        expected = plan(graph, start, goal, disabled)
        result = run_navigation(graph, scene, start, goal, disabled_channels=disabled)
        assert result["status"] == ("reached" if expected["status"] == "reachable" else "unreachable")
        assert_fresh_observations(result, scene)
        if result["status"] == "reached":
            assert result["final_state"]["node_id"] == goal
            assert result["final_state"]["distance_m"] == expected["length_m"]
            assert result["steps"][-1]["probabilities"][goal] == 1


@pytest.mark.parametrize("args,code,status", [
    ([], 0, "reached"), (["--disable", "C02"], 0, "reached"),
    (["--disable", "C06"], 1, "unreachable"),
    (["--max-steps", "1"], 1, "step_limit"),
    (["--confidence", "0.2"], 1, "localization_uncertain"),
    (["--start", "N99"], 2, "invalid"), (["--confidence", "nan"], 2, "invalid"),
])
def test_cli_json_and_exit_codes(tmp_path, args, code, status):
    output = tmp_path / "run.json"
    process = subprocess.run([sys.executable, "-m", "simulation.navigation", *args,
                              "--output", str(output)], cwd=ROOT,
                             capture_output=True, text=True, timeout=10)
    assert process.returncode == code, process.stderr
    result = json.loads(process.stdout)
    assert result["status"] == status
    if status != "invalid":
        assert json.loads(output.read_text()) == result
