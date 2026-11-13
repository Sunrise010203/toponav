"""Candidate-set localization from topology, relative structure and execution feedback.

This module has no simulator/scene dependency and never receives actual poses.
"""

import math

from simulation import path_planning
from simulation.topology import MOVES, arrival_heading, cardinal_heading, exit_direction, normalize_action


TURNS = {"front": 0, "left": 90, "right": -90, "back": 180}
ABSOLUTE = {0: "east", 90: "north", 180: "west", 270: "south"}
MIN_CONFIDENCE = 0.6


def absolute_action(heading, action):
    heading = cardinal_heading(heading)
    action = normalize_action(action)
    if action == "stop":
        raise ValueError("A stop command has no movement direction")
    return ABSOLUTE[(heading + MOVES[action]) % 360]


def _relative_exits(graph, candidate, *, traversable, disabled_channels=()):
    node = next((n for n in graph["nodes"] if n["node_id"] == candidate["node_id"]), None)
    if node is None:
        raise ValueError(f"Unknown candidate node: {candidate['node_id']}")
    absolute = {}
    for edge in graph["edges"]:
        if traversable and (not edge["enabled"] or edge["channel_id"] in disabled_channels):
            continue
        if node["node_id"] == edge["source"]:
            neighbor = edge["target"]
        elif node["node_id"] == edge["target"] and (not traversable or edge["bidirectional"]):
            neighbor = edge["source"]
        else:
            continue
        action = exit_direction(graph, node["node_id"], neighbor)
        if action in absolute:
            raise ValueError("Multiple map exits in one direction are not supported")
        absolute[action] = neighbor
    return {relative: absolute[absolute_action(candidate["heading_deg"], relative)]
            for relative in TURNS if absolute_action(candidate["heading_deg"], relative) in absolute}


def map_exits(graph, candidate, disabled_channels=()):
    """Physical relative exit signature at a hypothetical pose, not permission.

    Channel closures and one-way restrictions do not remove physical mouths.
    The optional disabled_channels argument is retained for caller compatibility
    but intentionally has no effect. Use traversable_exits for motion prediction
    and the public planner for command authorization. A true visual mismatch
    still rejects the candidate; this does not synthesize sensor evidence.
    """
    return _relative_exits(graph, candidate, traversable=False)


def traversable_exits(graph, candidate, disabled_channels=()):
    """Relative transitions allowed by enabled, closure and direction policy."""
    return _relative_exits(graph, candidate, traversable=True, disabled_channels=disabled_channels)


def _canonical(candidates):
    return [{"node_id": node, "heading_deg": heading}
            for node, heading in sorted({(p["node_id"], p["heading_deg"]) for p in candidates})]


def _validate_observation(observation):
    if not isinstance(observation, dict) or set(observation) != {"relative_open", "confidence"}:
        raise ValueError("Observation must contain only relative_open and confidence")
    opened, confidence = observation["relative_open"], observation["confidence"]
    if not isinstance(opened, dict) or set(opened) != set(TURNS):
        raise ValueError("Observation must describe front/left/right/back")
    if any(value is not None and type(value) is not bool for value in opened.values()):
        raise ValueError("Each direction must be Boolean or null (unknown)")
    if type(confidence) not in (int, float) or not math.isfinite(confidence) or not 0 <= confidence <= 1:
        raise ValueError("Observation confidence must be finite and between 0 and 1")


def update_candidates(graph, previous_candidates, observation, previous_action=None,
                      feedback=None, disabled_channels=()):
    """Predict from confirmed execution, then intersect with structural evidence.

    succeeded means one entire atomic turn+edge traversal completed; failed
    means neither translation nor rotation occurred. unknown/sent/no report
    must retain both stay and possible-success hypotheses and force a stop.
    """
    _validate_observation(observation)
    for candidate in previous_candidates:
        if not isinstance(candidate, dict) or set(candidate) != {"node_id", "heading_deg"}:
            raise ValueError("Candidate must contain only node_id and heading_deg")
        absolute_action(candidate["heading_deg"], "front")
        map_exits(graph, candidate)
    if previous_action is not None:
        previous_action = normalize_action(previous_action)
    if feedback is not None:
        if not isinstance(feedback, dict) or set(feedback) != {"status"}:
            raise ValueError("Execution feedback must contain only status, never actual pose")
        if feedback["status"] not in {"succeeded", "failed", "unknown", "sent", "stopped"}:
            raise ValueError("Unknown execution feedback status")
        if previous_action is None:
            raise ValueError("Execution feedback requires a previous command")
        if (feedback["status"] == "stopped") != (previous_action == "stop"):
            raise ValueError("stopped feedback is only valid for a stop command")

    prior = _canonical(previous_candidates)
    execution = feedback["status"] if feedback else None
    unconfirmed = previous_action in MOVES and execution in (None, "unknown", "sent")
    predicted = []
    for candidate in prior:
        if previous_action is None or previous_action == "stop" or execution == "failed" or unconfirmed:
            predicted.append(candidate)
        if previous_action in MOVES and (execution == "succeeded" or unconfirmed):
            exits = traversable_exits(graph, candidate, disabled_channels)
            sensor_direction = {"straight": "front", "uturn": "back"}.get(previous_action, previous_action)
            if sensor_direction in exits:
                neighbor = exits[sensor_direction]
                predicted.append({"node_id": neighbor,
                                  "heading_deg": arrival_heading(graph, candidate["node_id"], neighbor)})
    predicted = _canonical(predicted)
    trusted = observation["confidence"] >= MIN_CONFIDENCE
    complete = trusted and all(value is not None for value in observation["relative_open"].values())
    matched = []
    for candidate in predicted:
        exits = map_exits(graph, candidate)
        if not trusted or all(value is None or value == (relative in exits)
                              for relative, value in observation["relative_open"].items()):
            matched.append(candidate)

    if not predicted:
        status, reason = "transition_mismatch", "No candidate supports the confirmed transition; no reset"
    elif not matched:
        status, reason = "observation_mismatch", "New structure contradicts every predicted candidate; no reset"
    elif execution == "failed":
        status, reason = "execution_failed", "No motion confirmed; keep prior-compatible candidates and stop"
    elif unconfirmed:
        status, reason = "execution_unconfirmed", "Command is not completion; retain compatible stay/move hypotheses"
    elif not complete:
        status, reason = "observation_uncertain", "Incomplete or low-confidence structure; retain candidates and stop"
    elif len(matched) != 1:
        status, reason = "ambiguous", "Several places/orientations fit; structure is not place identity"
    else:
        status, reason = "localized", "One history- and observation-consistent map hypothesis"
    return {"status": status, "reason": reason, "predicted_candidates": predicted,
            "candidates": matched, "estimate": dict(matched[0]) if complete and len(matched) == 1 else None,
            "observation_reliable": complete}


def decide(graph, localization, goal, disabled_channels=()):
    """Choose a relative command using only map hypotheses and task goal."""
    if localization["status"] != "localized":
        return {"status": localization["status"], "action": "stop", "route": None, "request_observation": True}
    if len(localization["candidates"]) != 1 or localization["estimate"] is None:
        return {"status": "ambiguous", "action": "stop", "route": None, "request_observation": True}
    route = path_planning.plan_from_estimate(graph, localization, goal, disabled_channels)
    decision = path_planning.next_action(graph, localization, route, disabled_channels)
    return {**decision, "route": route}
