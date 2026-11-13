import copy
import itertools
import json
from pathlib import Path
import subprocess
import sys

import pytest

from simulation.dijkstra import plan
from simulation import path_planning
from simulation.structural_localization import update_candidates, decide
from simulation.structural_navigation import run_navigation, StructuralEnvironment
from simulation.topology import validate_pair

ROOT = Path(__file__).resolve().parents[1]


def inputs(name="graph.json"):
    return [json.loads((ROOT / "configs" / "v2" / p).read_text())
            for p in (name, "scene.json")]


def estimate(node="N02", heading=0):
    return {"status": "localized", "node_id": node,
            "heading_status": "certain", "heading_deg": heading}


def control_trace(result):
    return [{key: step[key] for key in ("prior_candidates", "observation", "localization",
                                       "decision", "feedback_used", "execution_feedback")}
            for step in result["steps"]]


def test_default_configs_are_v2_and_legacy_is_preserved():
    for name in ("graph.json", "scene.json", "graph.schema.json", "scene.schema.json",
                 "graph_blocked_C02.json"):
        assert (ROOT / "configs" / name).read_bytes() == (ROOT / "configs" / "v2" / name).read_bytes()
    assert json.loads((ROOT / "configs" / "legacy" / "graph.json").read_text())["schema_version"] == "1.0"


def test_v2_planner_reports_hops_not_fabricated_metres():
    graph, scene = inputs()
    original = copy.deepcopy((graph, scene))
    route = plan(graph)
    assert route["nodes"] == ["N01", "N02", "N03", "N04", "N06"]
    assert route["cost_unit"] == "hops"
    assert route["cost"] == route["hop_count"] == 4
    assert route["length_m"] is None
    result = run_navigation(graph, scene)
    assert result["audit"]["goal_verified"]
    assert result["final_truth"]["distance_m"] == 8
    assert [s["decision"]["action"] for s in result["steps"]] == [
        "straight", "straight", "left", "right", "stop"]
    assert (graph, scene) == original


def test_navigation_really_calls_the_public_planning_block(monkeypatch):
    counts = {"plan": 0, "act": 0}
    original_plan, original_act = path_planning.plan_from_estimate, path_planning.next_action

    def checked_plan(*args, **kwargs):
        counts["plan"] += 1
        return original_plan(*args, **kwargs)

    def checked_act(*args, **kwargs):
        counts["act"] += 1
        return original_act(*args, **kwargs)

    monkeypatch.setattr(path_planning, "plan_from_estimate", checked_plan)
    monkeypatch.setattr(path_planning, "next_action", checked_act)
    assert run_navigation(*inputs())["audit"]["goal_verified"]
    assert counts == {"plan": 5, "act": 5}


def test_display_layout_cannot_change_observation_or_control():
    graph, scene = inputs()
    a = run_navigation(graph, scene)
    shifted = copy.deepcopy(scene)
    shifted["geometry"]["node_positions_m"] = {
        n["node_id"]: [index * 7 + 1, (-1) ** index * (index + 3)]
        for index, n in enumerate(graph["nodes"])}
    b = run_navigation(graph, shifted)
    assert a["audit"] == b["audit"]
    assert control_trace(a) == control_trace(b)
    assert b["final_truth"]["position_m"] != a["final_truth"]["position_m"]
    route = path_planning.plan_from_estimate(graph, estimate(), "N05")
    assert path_planning.next_action(graph, estimate(), route)["action"] == "left"


def test_geometry_is_optional_for_the_entire_discrete_loop():
    graph, scene = inputs()
    scene.pop("geometry")
    result = run_navigation(graph, scene)
    assert result["audit"]["goal_verified"]
    assert result["final_truth"]["position_m"] is None
    assert result["final_truth"]["distance_m"] is None
    assert result["audit"]["actual_moves"] == 4
    assert all(step["decision"]["route"]["length_m"] is None for step in result["steps"])


def test_no_task_requires_explicit_task_and_orientation():
    graph, scene = inputs()
    graph.pop("task")
    with pytest.raises(ValueError):
        run_navigation(graph, scene)
    with pytest.raises(ValueError):
        run_navigation(graph, scene, start="N01", goal="N06")
    assert plan(graph, "N01", "N06")["hop_count"] == 4
    assert run_navigation(graph, scene, start="N01", goal="N06", heading=0)["audit"]["goal_verified"]


def test_task_heading_is_used_instead_of_assuming_east():
    graph, scene = inputs()
    graph["task"]["initial_heading_deg"] = 90
    result = run_navigation(graph, scene)
    assert result["heading_prior"] == 90
    assert result["steps"][0]["decision"]["action"] == "right"
    assert result["audit"]["goal_verified"]
    graph["task"].pop("initial_heading_deg")
    with pytest.raises(ValueError):
        run_navigation(graph, scene)


def test_curved_channel_uses_both_local_endpoint_directions():
    graph, scene = inputs()
    exit_back = next(n for n in graph["nodes"] if n["node_id"] == "N02")["exits"][0]
    assert exit_back["channel_id"] == "C01"
    exit_back["direction"] = "south"
    scene["channels"][0]["target_direction"] = "south"
    result = run_navigation(graph, scene)
    assert result["audit"]["goal_verified"]
    second = result["steps"][1]
    assert second["truth"]["before"]["heading_deg"] == 90
    assert second["localization"]["estimate"]["heading_deg"] == 90
    assert second["decision"]["action"] == "right"
    current = {"status": "localized", "node_id": "N02",
               "incoming_channel": "C01", "incoming_status": "certain"}
    route = path_planning.plan_from_estimate(graph, current, "N03")
    assert path_planning.next_action(graph, current, route)["action"] == "right"


@pytest.mark.parametrize("name", [
    "four_nodes", "seven_nodes", "renamed_six_nodes", "no_task_no_geometry_no_photos",
    "layout_changed", "closed_goal_channel", "same_map_different_task"])
def test_all_v2_example_pairs_run_without_map_specific_constants(name):
    graph = json.loads((ROOT / "configs/v2/examples" / (name + ".graph.json")).read_text())
    scene = json.loads((ROOT / "configs/v2/examples" / (name + ".scene.json")).read_text())
    validate_pair(graph, scene)
    task = graph.get("task", {})
    start = task.get("start_node", graph["nodes"][0]["node_id"])
    goal = task.get("goal_node", graph["nodes"][-1]["node_id"])
    heading = task.get("initial_heading_deg", 0)
    result = run_navigation(graph, scene, start=start, goal=goal, heading=heading)
    expected = plan(graph, start, goal)
    assert result["status"] == ("reached" if expected["status"] == "reachable" else "unreachable")
    if result["status"] == "reached":
        assert result["audit"]["goal_verified"]


@pytest.mark.parametrize("case", json.loads(
    (ROOT / "configs/v2/invalid_cases/manifest.json").read_text()), ids=lambda c: c["name"])
def test_all_v2_invalid_pairs_are_rejected_by_runtime(case):
    base = ROOT / "configs/v2/invalid_cases"
    graph = json.loads((base / (case["name"] + ".json")).read_text())
    scene = json.loads((base / case["scene_file"]).read_text())
    with pytest.raises(ValueError):
        validate_pair(graph, scene)
    with pytest.raises(ValueError):
        run_navigation(graph, scene)


@pytest.mark.parametrize("scenario,status", [
    ("execution_failed", "execution_failed"), ("execution_unknown", "execution_unconfirmed"),
    ("execution_sent", "execution_unconfirmed"), ("ambiguous", "observation_uncertain"),
    ("mismatch", "observation_mismatch")])
def test_v2_faults_stop_without_truth_repair(scenario, status):
    result = run_navigation(*inputs(), scenario=scenario)
    assert result["status"] == status
    assert result["steps"][-1]["decision"]["action"] == "stop"
    assert not result["audit"]["goal_verified"]


@pytest.mark.parametrize("mask", range(64))
def test_v2_all_closures_and_pairs_against_independent_floyd_warshall(mask):
    graph, scene = inputs()
    ids = [n["node_id"] for n in graph["nodes"]]
    disabled = [e["channel_id"] for i, e in enumerate(graph["edges"]) if mask & (1 << i)]
    distance = {(a, b): 0 if a == b else float("inf") for a in ids for b in ids}
    for edge in graph["edges"]:
        if edge["enabled"] and edge["channel_id"] not in disabled:
            distance[edge["source"], edge["target"]] = 1
            distance[edge["target"], edge["source"]] = 1
    for mid in ids:
        for a, b in itertools.product(ids, repeat=2):
            distance[a, b] = min(distance[a, b], distance[a, mid] + distance[mid, b])
    for start, goal in itertools.product(ids, repeat=2):
        route = plan(graph, start, goal, disabled)
        run = run_navigation(graph, scene, start=start, goal=goal, disabled_channels=disabled)
        if distance[start, goal] == float("inf"):
            assert route["status"] == run["status"] == "unreachable"
            assert route["cost"] is None and route["hop_count"] is None
        else:
            assert route["hop_count"] == distance[start, goal]
            assert run["audit"]["goal_verified"]
            assert run["audit"]["actual_moves"] == distance[start, goal]
            assert run["final_truth"]["distance_m"] == 2 * distance[start, goal]


@pytest.mark.parametrize("alias,canonical", [("front", "straight"), ("back", "uturn"),
                                           ("u_turn", "uturn")])
def test_legacy_commands_are_normalized_only_at_input_boundary(alias, canonical):
    graph, scene = inputs()
    env = StructuralEnvironment(graph, scene, start="N02", heading=0)
    other = StructuralEnvironment(graph, scene, start="N02", heading=0)
    assert env.execute(alias) == other.execute(canonical)
    assert env.state() == other.state()
    obs = {"relative_open": {"front": None, "left": None, "right": None, "back": None},
           "confidence": 0.1}
    a = update_candidates(graph, [{"node_id": "N02", "heading_deg": 0}], obs, alias, {"status": "unknown"})
    b = update_candidates(graph, [{"node_id": "N02", "heading_deg": 0}], obs, canonical, {"status": "unknown"})
    assert a == b


@pytest.mark.parametrize("module,args,code,status", [
    ("simulation.dijkstra", [], 0, "reachable"),
    ("simulation.structural_navigation", [], 0, "reached"),
    ("simulation.structural_navigation", ["--disable", "C02"], 0, "reached"),
    ("simulation.structural_navigation", ["--disable", "C06"], 1, "unreachable"),
    ("simulation.navigation", [], 0, "reached")])
def test_default_cli_contract(module, args, code, status):
    p = subprocess.run([sys.executable, "-m", module, *args], cwd=ROOT,
                       capture_output=True, text=True, timeout=15)
    assert p.returncode == code, p.stderr
    result = json.loads(p.stdout)
    assert result["status"] == status
    if module == "simulation.dijkstra":
        assert result["cost_unit"] == "hops" and result["length_m"] is None


def test_plotting_without_geometry_reports_input_error(tmp_path):
    graph, scene = inputs()
    scene.pop("geometry")
    scene_path = tmp_path / "scene.json"
    scene_path.write_text(json.dumps(scene))
    p = subprocess.run([sys.executable, "-m", "simulation.structural_navigation",
                        "--map", str(ROOT / "configs/v2/graph.json"), "--scene", str(scene_path), "--plot", str(tmp_path / "plot.png")],
                       cwd=ROOT, capture_output=True, text=True, timeout=15)
    assert p.returncode == 2
    assert json.loads(p.stdout)["status"] == "invalid"
    assert "geometry" in json.loads(p.stdout)["error"].lower()

@pytest.mark.parametrize("heading", [0, 90, 180, 270])
def test_v2_all_pairs_under_each_confirmed_heading(heading):
    graph, scene = inputs()
    ids = [n["node_id"] for n in graph["nodes"]]
    for start, goal in itertools.product(ids, repeat=2):
        result = run_navigation(graph, scene, start=start, goal=goal, heading=heading)
        assert result["audit"]["goal_verified"]


def test_v2_audit_truth_never_repairs_or_drives_the_controller(monkeypatch):
    original = run_navigation(*inputs())
    monkeypatch.setattr(StructuralEnvironment, "state", lambda self: {
        "node_id": "AUDIT_ONLY", "position_m": [999, 999], "heading_deg": 270, "distance_m": 999})
    spoofed = run_navigation(*inputs())
    assert control_trace(original) == control_trace(spoofed)
    assert original["final_estimate"] == spoofed["final_estimate"]
    assert not spoofed["audit"]["goal_verified"]


def test_v2_planning_uses_hops_even_if_evaluation_lengths_favor_another_route():
    graph, scene = inputs()
    scene["geometry"]["channel_dimensions_m"]["C02"]["length"] = 100
    a = plan(graph)
    b = run_navigation(graph, scene)
    assert a["nodes"] == ["N01", "N02", "N03", "N04", "N06"]
    assert a["hop_count"] == 4 and a["length_m"] is None
    assert b["final_truth"]["distance_m"] == 106


@pytest.mark.parametrize("contents", ["[]", "null", "{invalid"])
def test_malformed_cli_input_is_invalid_not_a_traceback(tmp_path, contents):
    path = tmp_path / "bad.json"
    path.write_text(contents)
    p = subprocess.run([sys.executable, "-m", "simulation.structural_navigation",
                        "--map", str(path)], cwd=ROOT, capture_output=True, text=True, timeout=15)
    assert p.returncode == 2
    assert json.loads(p.stdout)["status"] == "invalid"
    assert "Traceback" not in p.stderr

@pytest.mark.parametrize("incoming,target,expected", [
    ("C01", "N01", "uturn"), ("C01", "N03", "straight"), ("C01", "N05", "left"),
    ("C02", "N01", "straight"), ("C02", "N03", "uturn"), ("C02", "N05", "right"),
    ("C05", "N01", "right"), ("C05", "N03", "left"), ("C05", "N05", "uturn")])
def test_v2_n02_all_incoming_outgoing_correspondences(incoming, target, expected):
    graph, _ = inputs()
    current = {"status": "localized", "node_id": "N02",
               "incoming_channel": incoming, "incoming_status": "certain"}
    route = path_planning.plan_from_estimate(graph, current, target)
    assert path_planning.next_action(graph, current, route)["action"] == expected


def test_direct_planner_script_uses_v2_defaults_from_another_directory(tmp_path):
    p = subprocess.run([sys.executable, str(ROOT / "simulation/dijkstra.py")],
                       cwd=tmp_path, capture_output=True, text=True, timeout=15)
    assert p.returncode == 0, p.stderr
    assert json.loads(p.stdout)["cost_unit"] == "hops"


def test_plot_with_coordinates_but_no_lengths_does_not_fabricate_metres(tmp_path):
    from simulation.structural_navigation import plot_run
    graph, scene = inputs()
    scene["geometry"].pop("channel_dimensions_m")
    result = run_navigation(graph, scene)
    assert result["audit"]["goal_verified"] and result["final_truth"]["distance_m"] is None
    path = tmp_path / "unknown-distance.png"
    plot_run(graph, scene, result, path)
    assert path.is_file()
