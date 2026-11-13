"""Shared v2 topology contract. Geometry is never a navigation input."""

import math
from pathlib import Path
import json

from jsonschema import Draft7Validator

HEADINGS = {"east": 0, "north": 90, "west": 180, "south": 270}
MOVES = {"straight": 0, "left": 90, "right": -90, "uturn": 180}
ALIASES = {"front": "straight", "back": "uturn", "u_turn": "uturn"}
SCHEMA_ROOT = Path(__file__).resolve().parents[1] / "configs" / "v2"
GRAPH_SCHEMA = Draft7Validator(json.loads((SCHEMA_ROOT / "graph.schema.json").read_text()))
SCENE_SCHEMA = Draft7Validator(json.loads((SCHEMA_ROOT / "scene.schema.json").read_text()))


def is_v2(graph):
    return isinstance(graph, dict) and graph.get("schema_version") == "2.0"


def normalize_action(action):
    action = ALIASES.get(action, action)
    if action not in (*MOVES, "stop"):
        raise ValueError(f"Unknown relative action: {action}")
    return action


def cardinal_heading(heading):
    if (type(heading) not in (int, float) or not math.isfinite(heading)
            or heading not in (0, 90, 180, 270)):
        raise ValueError("A confirmed heading of 0, 90, 180 or 270 degrees is required")
    return int(heading)


def task_values(graph, start=None, goal=None):
    task = graph.get("task", {}) if is_v2(graph) else graph
    return (task.get("start_node") if start is None else start,
            task.get("goal_node") if goal is None else goal)


def exit_direction(graph, node_id, neighbor_id):
    node = next((n for n in graph["nodes"] if n["node_id"] == node_id), None)
    if node is None:
        raise ValueError(f"Unknown map node: {node_id}")
    if is_v2(graph):
        exits = [x for x in node["exits"] if x["neighbor_id"] == neighbor_id]
        if len(exits) != 1:
            raise ValueError(f"Missing or ambiguous exit: {node_id} -> {neighbor_id}")
        return exits[0]["direction"]
    try:
        return node["actions"][node["neighbors"].index(neighbor_id)]
    except (KeyError, ValueError, IndexError) as error:
        raise ValueError(f"Missing legacy exit: {node_id} -> {neighbor_id}") from error


def arrival_heading(graph, source, target):
    if is_v2(graph):
        return (HEADINGS[exit_direction(graph, target, source)] + 180) % 360
    return HEADINGS[exit_direction(graph, source, target)]


def _schema_check(validator, data):
    error = next(validator.iter_errors(data), None)
    if error:
        location = "/".join(map(str, error.absolute_path))
        raise ValueError(f"Schema {location}: {error.message}")


def validate_graph(graph):
    """Check v2 schema and topology references, independently of scene geometry."""
    _schema_check(GRAPH_SCHEMA, graph)
    nodes = {n["node_id"]: n for n in graph["nodes"]}
    edges = {e["channel_id"]: e for e in graph["edges"]}
    if len(nodes) != len(graph["nodes"]) or len(edges) != len(graph["edges"]):
        raise ValueError("Duplicate node or channel ID")
    undirected = set()
    adjacent = {node: set() for node in nodes}
    for edge in graph["edges"]:
        a, b = edge["source"], edge["target"]
        if a not in nodes or b not in nodes or a == b:
            raise ValueError("Invalid edge endpoints")
        pair = frozenset((a, b))
        if pair in undirected:
            raise ValueError("Duplicate undirected edge")
        undirected.add(pair)
        adjacent[a].add(b)
        adjacent[b].add(a)
    for node_id, node in nodes.items():
        exits = node["exits"]
        if set(node["neighbors"]) != adjacent[node_id] or len(node["neighbors"]) != len(adjacent[node_id]):
            raise ValueError("Neighbors and physical edges disagree")
        if (sorted(x["neighbor_id"] for x in exits) != sorted(node["neighbors"])
                or len({x["channel_id"] for x in exits}) != len(exits)
                or len({x["direction"] for x in exits}) != len(exits)):
            raise ValueError("Missing, duplicate or ambiguous exit bindings")
        for x in exits:
            edge = edges.get(x["channel_id"])
            if edge is None or {node_id, x["neighbor_id"]} != {edge["source"], edge["target"]}:
                raise ValueError("Exit binding does not match map edge")
    task = graph.get("task")
    if task and (task["start_node"] not in nodes or task["goal_node"] not in nodes
                 or task["start_node"] == task["goal_node"]):
        raise ValueError("Invalid task endpoints")
    seen, pending = set(), [next(iter(nodes))]
    while pending:
        node = pending.pop()
        if node not in seen:
            seen.add(node)
            pending.extend(adjacent[node] - seen)
    if seen != set(nodes):
        raise ValueError("Physical topology must be connected; disabled edges remain physical edges")


def validate_pair(graph, scene):
    """Validate independent physical channels before a v2 simulation executes."""
    if not isinstance(graph, dict) or not isinstance(scene, dict):
        raise ValueError("Map and scene must be JSON objects")
    if not is_v2(graph):
        if scene.get("schema_version") == "2.0":
            raise ValueError("Graph and scene versions differ")
        return
    validate_graph(graph)
    _schema_check(SCENE_SCHEMA, scene)
    if graph["scene_id"] != scene["scene_id"] or graph["exit_direction_frame"] != scene["exit_direction_frame"]:
        raise ValueError("Graph and scene identity/direction frames differ")
    nodes = {n["node_id"] for n in graph["nodes"]}
    scene_nodes = [n["node_id"] for n in scene["nodes"]]
    if len(set(scene_nodes)) != len(scene_nodes) or set(scene_nodes) != nodes:
        raise ValueError("Graph and scene node inventories differ")
    channels = {c["channel_id"]: c for c in scene["channels"]}
    edges = {e["channel_id"]: e for e in graph["edges"]}
    if len(channels) != len(scene["channels"]) or set(channels) != set(edges):
        raise ValueError("Graph and scene channel inventories differ")
    physical_pairs = set()
    for channel_id, c in channels.items():
        edge = edges[channel_id]
        endpoints = frozenset((c["source"], c["target"]))
        if (len(endpoints) != 2 or not endpoints <= nodes or endpoints in physical_pairs
                or endpoints != frozenset((edge["source"], edge["target"]))
                or c["bidirectional"] != edge["bidirectional"]):
            raise ValueError("Map edge differs from physical channel")
        physical_pairs.add(endpoints)
        for endpoint, neighbor, direction in (
                (c["source"], c["target"], c["source_direction"]),
                (c["target"], c["source"], c["target_direction"])):
            if exit_direction(graph, endpoint, neighbor) != direction:
                raise ValueError("Map exit direction differs from independent physical channel")
    geo = scene.get("geometry", {})
    for field, allowed in (("node_positions_m", nodes), ("node_annotations", nodes),
                           ("channel_dimensions_m", set(channels))):
        if not set(geo.get(field, {})) <= allowed:
            raise ValueError(f"Unknown geometry metadata reference in {field}")
    origin = geo.get("coordinate_system", {}).get("origin_node")
    if origin is not None and origin not in nodes:
        raise ValueError("Unknown geometry origin")
    bounds = geo.get("bounds_m")
    if bounds and not (bounds["x_min"] < bounds["x_max"] and bounds["y_min"] < bounds["y_max"]):
        raise ValueError("Invalid geometry bounds")
    evaluation = geo.get("evaluation", {})
    reference = evaluation.get("reference_task")
    if reference and not {reference["start_node"], reference["goal_node"]} <= nodes:
        raise ValueError("Unknown evaluation task endpoint")
    for route in evaluation.get("reference_routes", {}).values():
        if (any(node not in nodes for node in route)
                or (reference and (route[0] != reference["start_node"] or route[-1] != reference["goal_node"]))
                or any(frozenset(pair) not in physical_pairs for pair in zip(route, route[1:]))):
            raise ValueError("Invalid evaluation reference route")


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description="Validate v2 schema and independent graph/scene consistency")
    root = Path(__file__).resolve().parents[1]
    parser.add_argument("--map", type=Path, default=root / "configs/graph.json")
    parser.add_argument("--scene", type=Path, default=root / "configs/scene.json")
    args = parser.parse_args(argv)
    try:
        graph = json.loads(args.map.read_text(encoding="utf-8"))
        scene = json.loads(args.scene.read_text(encoding="utf-8"))
        if not is_v2(graph):
            raise ValueError("This validation CLI requires v2; legacy schemas remain in configs/legacy")
        validate_pair(graph, scene)
        from simulation.dijkstra import plan
        task = graph.get("task")
        reachable = plan(graph)["status"] == "reachable" if task else None
    except (OSError, ValueError) as error:
        print(json.dumps({"status": "invalid", "error": str(error)}, ensure_ascii=False))
        return 2
    print(json.dumps({"status": "valid", "schema_version": "2.0", "scene_id": graph["scene_id"],
                      "nodes": len(graph["nodes"]), "channels": len(graph["edges"]),
                      "task_reachable": reachable}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
