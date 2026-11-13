"""Shortest paths over enabled topology channels, without mutating the map."""

import argparse
import heapq
import json
import math
from pathlib import Path

if __package__ in (None, ""):
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from simulation.topology import is_v2, task_values, validate_graph


def plan(graph, start=None, goal=None, disabled_channels=()):
    """Recompute a route from start using current enabled flags and closures.

    V2 minimizes hops and reports cost_unit/hop_count, never fictitious metres.
    V1 retains weighted length_m. Invalid inputs raise ValueError, not unreachable.
    """
    if not isinstance(graph, dict) or not isinstance(graph.get("nodes"), list) or not isinstance(graph.get("edges"), list):
        raise ValueError("Map must contain nodes and edges arrays")
    modern = is_v2(graph)
    if modern:
        validate_graph(graph)
    elif graph.get("schema_version") not in (None, "1.0"):
        raise ValueError("Unsupported map schema version")
    adjacency = {}
    for node in graph["nodes"]:
        if not isinstance(node, dict) or not isinstance(node.get("node_id"), str) or not node["node_id"]:
            raise ValueError("Each node must have a nonempty node_id")
        node_id = node["node_id"]
        if node_id in adjacency:
            raise ValueError(f"Duplicate node: {node_id}")
        adjacency[node_id] = []

    start, goal = task_values(graph, start, goal)
    for name, node_id in (("start", start), ("goal", goal)):
        if not isinstance(node_id, str) or node_id not in adjacency:
            raise ValueError(f"Unknown {name} node: {node_id}")
    disabled = set(disabled_channels)
    channel_ids = set()
    for edge in graph["edges"]:
        if not isinstance(edge, dict):
            raise ValueError("Each edge must be an object")
        channel = edge.get("channel_id")
        if not isinstance(channel, str) or not channel:
            raise ValueError("Each edge must have a nonempty channel_id")
        if channel in channel_ids:
            raise ValueError(f"Duplicate channel: {channel}")
        channel_ids.add(channel)
        source, target = edge.get("source"), edge.get("target")
        if any(not isinstance(node, str) or node not in adjacency for node in (source, target)):
            raise ValueError(f"Unknown endpoint on channel {channel}")
        weight = 1 if modern else edge.get("length_m")
        if type(weight) not in (int, float) or not math.isfinite(weight) or weight < 0:
            raise ValueError(f"Channel {channel} needs a finite nonnegative length_m")
        if type(edge.get("enabled")) is not bool or type(edge.get("bidirectional")) is not bool:
            raise ValueError(f"Channel {channel} needs Boolean enabled/bidirectional flags")
        if edge["enabled"] and channel not in disabled:
            adjacency[source].append((target, weight, channel))
            if edge["bidirectional"]:
                adjacency[target].append((source, weight, channel))
    unknown = disabled - channel_ids
    if unknown:
        raise ValueError(f"Unknown disabled channels: {sorted(unknown)}")

    distances = {start: 0.0}
    previous = {}
    queue = [(0.0, start)]
    while queue:
        distance, node = heapq.heappop(queue)
        if distance != distances[node]:
            continue
        if node == goal:
            nodes, channels = [goal], []
            while nodes[-1] != start:
                parent, channel = previous[nodes[-1]]
                nodes.append(parent)
                channels.append(channel)
            result = {"status": "reachable", "start": start, "goal": goal,
                      "nodes": nodes[::-1], "channels": channels[::-1],
                      "length_m": None if modern else distance}
            if modern:
                result.update(cost=int(distance), cost_unit="hops", hop_count=len(channels))
            return result
        for neighbor, weight, channel in adjacency[node]:
            candidate = distance + weight
            if not math.isfinite(candidate):
                raise ValueError("Route length exceeds the finite numeric range")
            if candidate < distances.get(neighbor, math.inf):
                distances[neighbor] = candidate
                previous[neighbor] = (node, channel)
                heapq.heappush(queue, (candidate, neighbor))
    result = {"status": "unreachable", "start": start, "goal": goal,
              "nodes": [], "channels": [], "length_m": None}
    if modern:
        result.update(cost=None, cost_unit="hops", hop_count=None)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description="Dijkstra planning over enabled channels")
    parser.add_argument("--map", type=Path,
                        default=Path(__file__).resolve().parents[1] / "configs" / "graph.json")
    parser.add_argument("--start", help="Current node; defaults to map start_node")
    parser.add_argument("--goal", help="Target node; defaults to map goal_node")
    parser.add_argument("--disable", nargs="+", action="extend", default=[], metavar="CHANNEL",
                        help="Temporarily disabled channels (does not rewrite the map)")
    args = parser.parse_args(argv)
    try:
        graph = json.loads(args.map.read_text(encoding="utf-8"))
        result = plan(graph, args.start, args.goal, args.disable)
    except (OSError, ValueError) as error:
        print(json.dumps({"status": "invalid", "error": str(error)}, ensure_ascii=False))
        return 2
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return 0 if result["status"] == "reachable" else 1


if __name__ == "__main__":
    raise SystemExit(main())
