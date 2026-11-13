import copy
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest

from simulation.dijkstra import plan


ROOT = Path(__file__).resolve().parents[1]


def read_map(name="graph.json"):
    return json.loads((ROOT / "configs" / "legacy" / name).read_text(encoding="utf-8"))


def small_graph(edges):
    ids = sorted({node for source, target, _, _ in edges for node in (source, target)})
    return {
        "start_node": ids[0],
        "goal_node": ids[-1],
        "nodes": [{"node_id": node} for node in ids],
        "edges": [
            {"channel_id": f"C{i}", "source": source, "target": target,
             "length_m": length, "bidirectional": reverse, "enabled": True}
            for i, (source, target, length, reverse) in enumerate(edges)
        ],
    }


def assert_route(result, nodes, channels, length):
    assert result["status"] == "reachable"
    assert result["nodes"] == nodes
    assert result["channels"] == channels
    assert result["length_m"] == pytest.approx(length)


def assert_unreachable(result):
    assert result["status"] == "unreachable"
    assert result["nodes"] == []
    assert result["channels"] == []
    assert result["length_m"] is None
    json.dumps(result, allow_nan=False)


def test_normal_route():
    assert_route(plan(read_map()), ["N01", "N02", "N03", "N04", "N06"],
                 ["C01", "C02", "C03", "C06"], 8)


def test_disable_channel_and_replan_without_mutating_map():
    graph = read_map()
    original = copy.deepcopy(graph)
    assert_route(plan(graph, disabled_channels=["C02"]),
                 ["N01", "N02", "N05", "N04", "N06"],
                 ["C01", "C05", "C04", "C06"], 8)
    assert_route(plan(graph, start="N02", disabled_channels=["C02"]),
                 ["N02", "N05", "N04", "N06"], ["C05", "C04", "C06"], 6)
    assert graph == original
    assert plan(graph)["channels"] == ["C01", "C02", "C03", "C06"]


def test_blocked_map_respects_enabled_even_when_neighbors_still_list_channel():
    graph = read_map("graph_blocked_C02.json")
    assert "N03" in graph["nodes"][1]["neighbors"]
    assert_route(plan(graph), ["N01", "N02", "N05", "N04", "N06"],
                 ["C01", "C05", "C04", "C06"], 8)
    assert_unreachable(plan(graph, disabled_channels=["C05"]))


@pytest.mark.parametrize("disabled", [["C06"], ["C02", "C05"], ["C01"]])
def test_unreachable(disabled):
    assert_unreachable(plan(read_map(), disabled_channels=disabled))


def test_replanning_reads_current_enabled_state():
    graph = read_map()
    assert "C02" in plan(graph)["channels"]
    graph["edges"][1]["enabled"] = False
    assert "C02" not in plan(graph)["channels"]
    graph["edges"][-1]["enabled"] = False
    assert_unreachable(plan(graph))


def test_reverse_bidirectional_route():
    assert_route(plan(read_map(), "N06", "N01"),
                 ["N06", "N04", "N03", "N02", "N01"],
                 ["C06", "C03", "C02", "C01"], 8)


def test_same_start_goal_with_all_channels_disabled():
    graph = read_map()
    assert_route(plan(graph, "N02", "N02", [e["channel_id"] for e in graph["edges"]]),
                 ["N02"], [], 0)


def test_weighted_shortest_path_not_fewest_hops():
    graph = small_graph([("A", "D", 10, True), ("A", "B", 1.25, True),
                         ("B", "C", 0.5, True), ("C", "D", 1, True)])
    assert_route(plan(graph), ["A", "B", "C", "D"], ["C1", "C2", "C3"], 2.75)


def test_stale_heap_entries_and_zero_weight_cycles():
    graph = small_graph([("A", "B", 8, True), ("A", "C", 1, True),
                         ("C", "B", 0, True), ("B", "D", 10, True)])
    assert_route(plan(graph), ["A", "C", "B", "D"], ["C1", "C2", "C3"], 11)


def test_one_way_channel_is_not_reversible():
    graph = small_graph([("A", "B", 1, False)])
    assert_route(plan(graph), ["A", "B"], ["C0"], 1)
    assert_unreachable(plan(graph, "B", "A"))


@pytest.mark.parametrize("kwargs", [{"start": "N99"}, {"goal": "N99"},
                                   {"disabled_channels": ["C99"]}])
def test_unknown_ids_are_errors_not_unreachable(kwargs):
    with pytest.raises(ValueError):
        plan(read_map(), **kwargs)


@pytest.mark.parametrize("weight", [-1, math.inf, -math.inf, math.nan, True, "2", None])
def test_invalid_weights_are_rejected_even_on_disabled_edges(weight):
    graph = read_map()
    graph["edges"][0]["length_m"] = weight
    graph["edges"][0]["enabled"] = False
    with pytest.raises(ValueError):
        plan(graph)


@pytest.mark.parametrize("kind", ["duplicate_node", "duplicate_channel", "unknown_endpoint",
                                  "missing_weight", "enabled_string", "direction_string",
                                  "missing_nodes", "bad_edges", "bad_graph"])
def test_malformed_graph_is_rejected(kind):
    graph = read_map()
    if kind == "duplicate_node":
        graph["nodes"].append(copy.deepcopy(graph["nodes"][0]))
    elif kind == "duplicate_channel":
        graph["edges"].append(copy.deepcopy(graph["edges"][0]))
    elif kind == "unknown_endpoint":
        graph["edges"][0]["target"] = "N99"
    elif kind == "missing_weight":
        del graph["edges"][0]["length_m"]
    elif kind == "enabled_string":
        graph["edges"][0]["enabled"] = "false"
    elif kind == "direction_string":
        graph["edges"][0]["bidirectional"] = "true"
    elif kind == "missing_nodes":
        del graph["nodes"]
    elif kind == "bad_edges":
        graph["edges"] = None
    else:
        graph = []
    with pytest.raises(ValueError):
        plan(graph)


def oracle_distances(graph, disabled):
    ids = [node["node_id"] for node in graph["nodes"]]
    distances = {(a, b): 0 if a == b else math.inf for a in ids for b in ids}
    for edge in graph["edges"]:
        if not edge["enabled"] or edge["channel_id"] in disabled:
            continue
        a, b, weight = edge["source"], edge["target"], edge["length_m"]
        distances[a, b] = min(distances[a, b], weight)
        if edge["bidirectional"]:
            distances[b, a] = min(distances[b, a], weight)
    for mid in ids:
        for a in ids:
            for b in ids:
                distances[a, b] = min(distances[a, b], distances[a, mid] + distances[mid, b])
    return distances


@pytest.mark.parametrize("mask", range(64))
def test_all_closure_subsets_and_node_pairs_against_floyd_warshall(mask):
    graph = read_map()
    snapshot = copy.deepcopy(graph)
    disabled = {edge["channel_id"] for i, edge in enumerate(graph["edges"]) if mask & (1 << i)}
    expected = oracle_distances(graph, disabled)
    edges = {edge["channel_id"]: edge for edge in graph["edges"]}
    ids = [node["node_id"] for node in graph["nodes"]]
    for start, goal in itertools.product(ids, repeat=2):
        result = plan(graph, start, goal, disabled)
        if math.isinf(expected[start, goal]):
            assert_unreachable(result)
            continue
        assert result["status"] == "reachable"
        assert result["nodes"][0] == start and result["nodes"][-1] == goal
        assert len(result["channels"]) == len(result["nodes"]) - 1
        total = 0
        for a, b, channel in zip(result["nodes"], result["nodes"][1:], result["channels"]):
            edge = edges[channel]
            assert channel not in disabled and edge["enabled"]
            assert (a, b) == (edge["source"], edge["target"]) or (
                edge["bidirectional"] and (a, b) == (edge["target"], edge["source"]))
            total += edge["length_m"]
        assert result["length_m"] == pytest.approx(total)
        assert total == pytest.approx(expected[start, goal])
    assert graph == snapshot


@pytest.mark.parametrize("args,code,status", [
    ([], 0, "reachable"), (["--disable", "C02"], 0, "reachable"),
    (["--disable", "C06"], 1, "unreachable"),
    (["--disable", "C02", "C05"], 1, "unreachable"),
    (["--start", "N99"], 2, "invalid"),
    (["--disable", "C99"], 2, "invalid"),
])
def test_cli_exit_codes_and_json(args, code, status):
    process = subprocess.run([sys.executable, "-m", "simulation.dijkstra", "--map", str(ROOT / "configs/legacy/graph.json"), *args],
                             cwd=ROOT, capture_output=True, text=True, timeout=10)
    assert process.returncode == code, process.stderr
    result = json.loads(process.stdout)
    assert result["status"] == status
    if status == "invalid":
        assert result["error"]


@pytest.mark.parametrize("contents", ["{not json", "[]"])
def test_cli_invalid_map(tmp_path, contents):
    path = tmp_path / "bad.json"
    path.write_text(contents)
    process = subprocess.run([sys.executable, "-m", "simulation.dijkstra", "--map", str(path)],
                             cwd=ROOT, capture_output=True, text=True, timeout=10)
    assert process.returncode == 2
    assert json.loads(process.stdout)["status"] == "invalid"


def test_direct_script_defaults_do_not_depend_on_current_directory(tmp_path):
    process = subprocess.run([sys.executable, str(ROOT / "simulation" / "dijkstra.py"), "--map", str(ROOT / "configs/legacy/graph.json")],
                             cwd=tmp_path, capture_output=True, text=True, timeout=10)
    assert process.returncode == 0, process.stderr
    assert json.loads(process.stdout)["length_m"] == 8
