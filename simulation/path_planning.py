"""Planning block: estimated node -> route -> safe relative first-exit action.

No localization guesses, environment truth, robot motion or map mutations.
"""

import math

from simulation.dijkstra import plan as dijkstra_plan
from simulation.topology import HEADINGS, exit_direction, is_v2


def _accepted_estimate(estimate):
    if not isinstance(estimate, dict) or estimate.get("status") != "localized":
        return None
    if "estimate" in estimate or "candidates" in estimate:
        pose, candidates = estimate.get("estimate"), estimate.get("candidates")
        if (estimate.get("observation_reliable") is not True or not isinstance(pose, dict)
                or set(pose) != {"node_id", "heading_deg"} or not isinstance(candidates, list)
                or len(candidates) != 1 or candidates[0] != pose):
            return None
        # A unique, reliable, actionable upstream hypothesis confirms orientation.
        estimate = {**pose, "status": "localized", "heading_status": "certain"}
    node = estimate.get("node_id")
    return estimate if isinstance(node, str) and node else None


def plan_from_estimate(graph, estimate, goal, disabled_channels=()):
    """Use an accepted node estimate; never fall back to the map start.

    A localized estimate uses the original Dijkstra and its validation. An
    uncertain estimate produces no route. Orientation is not needed to plan.
    """
    accepted = _accepted_estimate(estimate)
    if accepted is None:
        result = {"status": "localization_uncertain", "start": None, "goal": goal,
                  "nodes": [], "channels": [], "length_m": None}
        if is_v2(graph):
            result.update(cost=None, cost_unit="hops", hop_count=None)
        return result
    return dijkstra_plan(graph, accepted["node_id"], goal, disabled_channels)


def _stop(status, observe=False):
    return {"status": status, "action": "stop", "request_observation": observe}


def _bearing(positions, source, target):
    a, b = positions.get(source), positions.get(target)
    for position in (a, b):
        if not isinstance(position, (list, tuple)) or len(position) != 2:
            return None
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in position):
            return None
    dx, dy = b[0] - a[0], b[1] - a[1]
    if (dx == 0 and dy == 0) or not math.isfinite(dx) or not math.isfinite(dy):
        return None
    return math.degrees(math.atan2(dy, dx)) % 360


def next_action(graph, estimate, route, disabled_channels=()):
    """Convert only the next exit to straight/left/right/uturn, or stop.

    Orientation evidence must explicitly have heading_status or incoming_status
    equal to 'certain'. A certain incoming channel means a confirmed arrival
    from its other endpoint, with no subsequent unaccounted rotation. Unknown,
    uncertain or conflicting evidence never becomes an arbitrary motion command.
    Pass the route returned by plan_from_estimate and current channel closures.
    """
    estimate = _accepted_estimate(estimate)
    if estimate is None:
        return _stop("localization_uncertain", True)
    node = estimate["node_id"]
    if not isinstance(route, dict) or route.get("start") != node:
        return _stop("replan_required")
    if route.get("status") == "unreachable":
        return _stop("unreachable")
    nodes, channels = route.get("nodes"), route.get("channels")
    if (route.get("status") != "reachable" or not isinstance(nodes, list)
            or not nodes or nodes[0] != node or not isinstance(channels, list)
            or len(channels) != len(nodes) - 1):
        return _stop("replan_required")
    if len(nodes) == 1:
        return _stop("reached" if node == route.get("goal") else "replan_required")

    edges = {edge["channel_id"]: edge for edge in graph["edges"]}
    edge = edges.get(channels[0])
    if not edge or not edge["enabled"] or channels[0] in disabled_channels:
        return _stop("replan_required")
    forward = (node, nodes[1]) == (edge["source"], edge["target"])
    reverse = edge["bidirectional"] and (node, nodes[1]) == (edge["target"], edge["source"])
    if not forward and not reverse:
        return _stop("replan_required")
    positions = ({} if is_v2(graph) else
                 {item["node_id"]: item.get("position_m") for item in graph["nodes"]})
    exit_heading = (HEADINGS[exit_direction(graph, node, nodes[1])] if is_v2(graph)
                    else _bearing(positions, node, nodes[1]))
    if exit_heading is None:
        return _stop("geometry_unknown", True)

    if estimate.get("heading_status") == "uncertain" or estimate.get("incoming_status") == "uncertain":
        return _stop("orientation_uncertain", True)
    heading = None
    if estimate.get("heading_status") == "certain":
        heading = estimate.get("heading_deg")
        if type(heading) not in (int, float) or not math.isfinite(heading):
            return _stop("orientation_uncertain", True)
        heading %= 360
    if estimate.get("incoming_status") == "certain":
        channel = estimate.get("incoming_channel")
        arrival = edges.get(channel) if isinstance(channel, str) else None
        if not arrival:
            return _stop("orientation_uncertain", True)
        if node == arrival["target"]:
            previous = arrival["source"]
        elif node == arrival["source"] and arrival["bidirectional"]:
            previous = arrival["target"]
        else:
            return _stop("orientation_uncertain", True)
        incoming_heading = ((HEADINGS[exit_direction(graph, node, previous)] + 180) % 360
                            if is_v2(graph) else _bearing(positions, previous, node))
        if incoming_heading is None:
            return _stop("orientation_uncertain", True)
        if heading is not None:
            difference = (heading - incoming_heading + 180) % 360 - 180
            if not math.isclose(difference, 0, abs_tol=1e-6):
                return _stop("orientation_conflict", True)
        heading = incoming_heading
    if heading is None:
        return _stop("orientation_uncertain", True)

    turn = (exit_heading - heading) % 360
    for angle, action in ((0, "straight"), (90, "left"), (180, "uturn"), (270, "right")):
        difference = (turn - angle + 180) % 360 - 180
        if math.isclose(difference, 0, abs_tol=1e-6):
            return {"status": "move", "action": action, "request_observation": False}
    return _stop("turn_unresolved", True)
