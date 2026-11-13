import copy
import json
import math
from pathlib import Path

import pytest

from simulation.dijkstra import plan
from simulation.path_planning import next_action, plan_from_estimate


ROOT = Path(__file__).resolve().parents[1]


def graph_input(name="graph.json"):
    return json.loads((ROOT / "configs" / "legacy" / name).read_text(encoding="utf-8"))


def estimate(node="N02", **orientation):
    return {"status": "localized", "node_id": node, **orientation}


def assert_stopped(result, status, observation=False):
    assert result["status"] == status
    assert result["action"] == "stop"
    assert result["request_observation"] is observation


@pytest.mark.parametrize("map_name,disabled", [
    ("graph.json", []), ("graph.json", ["C02"]),
    ("graph_blocked_C02.json", []), ("graph.json", ["C06"]),
    ("graph.json", ["C02", "C05"]),
])
def test_reuses_original_dijkstra_and_does_not_mutate(map_name, disabled):
    graph = graph_input(map_name)
    before = copy.deepcopy(graph)
    for node in ("N01", "N02", "N06"):
        result = plan_from_estimate(graph, estimate(node), "N06", disabled)
        assert result == plan(graph, node, "N06", disabled)
        next_action(graph, estimate(node, heading_deg=0, heading_status="certain"),
                    result, disabled)
    assert graph == before


@pytest.mark.parametrize("heading,expected", [(0, "left"), (90, "straight"),
                                            (180, "right"), (270, "uturn"),
                                            (-90, "uturn"), (450, "straight")])
def test_same_north_exit_under_different_headings(heading, expected):
    graph = graph_input()
    current = estimate(heading_deg=heading, heading_status="certain")
    route = plan_from_estimate(graph, current, "N05")
    assert route["channels"] == ["C05"]
    assert next_action(graph, current, route) == {
        "status": "move", "action": expected, "request_observation": False}


@pytest.mark.parametrize("channel,expected", [("C01", "left"), ("C02", "right"),
                                            ("C05", "uturn")])
def test_same_exit_under_different_entering_channels(channel, expected):
    graph = graph_input()
    current = estimate(incoming_channel=channel, incoming_status="certain")
    route = plan_from_estimate(graph, current, "N05")
    assert next_action(graph, current, route)["action"] == expected


@pytest.mark.parametrize("node,goal,exit_heading", [
    ("N02", "N03", 0), ("N02", "N05", 90),
    ("N02", "N01", 180), ("N04", "N03", 270),
])
@pytest.mark.parametrize("turn,expected", [(0, "straight"), (90, "left"),
                                         (180, "uturn"), (270, "right")])
def test_four_exit_directions_and_all_relative_turns(node, goal, exit_heading, turn, expected):
    graph = graph_input()
    current = estimate(node, heading_status="certain", heading_deg=(exit_heading - turn) % 360)
    assert next_action(graph, current, plan_from_estimate(graph, current, goal))["action"] == expected


@pytest.mark.parametrize("current", [None, {}, {"status": "uncertain", "node_id": "N01"},
                                     {"status": "localized", "node_id": None},
                                     {"status": "localized"},
                                     {"status": "lost", "node_id": "N01"}])
def test_uncertain_position_never_falls_back_to_map_start_or_picks_a_hypothesis(current):
    graph = graph_input()
    route = plan_from_estimate(graph, current, "N06")
    assert route["status"] == "localization_uncertain"
    assert route["start"] is None and route["nodes"] == [] and route["channels"] == []
    assert_stopped(next_action(graph, current, route), "localization_uncertain", True)


@pytest.mark.parametrize("orientation", [
    {}, {"heading_deg": 0}, {"incoming_channel": "C01"},
    {"heading_deg": 0, "heading_status": "uncertain"},
    {"incoming_channel": "C01", "incoming_status": "uncertain"},
    {"heading_deg": 0, "heading_status": "certain", "incoming_status": "uncertain"},
    {"incoming_channel": "C01", "incoming_status": "certain", "heading_status": "uncertain"},
])
def test_missing_or_uncertain_orientation_stops_instead_of_assuming(orientation):
    graph = graph_input()
    current = estimate(**orientation)
    route = plan_from_estimate(graph, current, "N06")
    assert route["status"] == "reachable"  # route does not require an orientation
    assert_stopped(next_action(graph, current, route), "orientation_uncertain", True)


@pytest.mark.parametrize("heading", [None, math.nan, math.inf, "east", True])
def test_invalid_certain_heading_does_not_fall_back_to_entering_channel(heading):
    graph = graph_input()
    current = estimate(heading_deg=heading, heading_status="certain",
                       incoming_channel="C01", incoming_status="certain")
    assert_stopped(next_action(graph, current, plan_from_estimate(graph, current, "N06")),
                   "orientation_uncertain", True)


def test_conflicting_heading_and_entering_channel_request_observation():
    graph = graph_input()
    current = estimate(incoming_channel="C01", incoming_status="certain",
                       heading_deg=90, heading_status="certain")
    assert_stopped(next_action(graph, current, plan_from_estimate(graph, current, "N05")),
                   "orientation_conflict", True)
    current["heading_deg"] = 360
    assert next_action(graph, current, plan_from_estimate(graph, current, "N05"))["action"] == "left"


@pytest.mark.parametrize("channel", ["C99", "C06", None, ["C01", "C02"]])
def test_unknown_unconnected_or_ambiguous_entering_channel_is_not_guessed(channel):
    graph = graph_input()
    current = estimate(incoming_channel=channel, incoming_status="certain")
    assert_stopped(next_action(graph, current, plan_from_estimate(graph, current, "N05")),
                   "orientation_uncertain", True)


def test_one_way_channel_cannot_be_used_as_reverse_arrival_evidence():
    graph = graph_input()
    graph["edges"][1]["bidirectional"] = False  # N02 -> N03 only
    current = estimate(incoming_channel="C02", incoming_status="certain")
    assert_stopped(next_action(graph, current, plan_from_estimate(graph, current, "N05")),
                   "orientation_uncertain", True)


def test_closed_entering_channel_can_describe_a_confirmed_previous_arrival():
    graph = graph_input()
    current = estimate(incoming_channel="C01", incoming_status="certain")
    route = plan_from_estimate(graph, current, "N05", ["C01"])
    assert next_action(graph, current, route, ["C01"])["action"] == "left"


@pytest.mark.parametrize("heading", [30, 45, 80, 135, 225, 315])
def test_non_four_way_turn_does_not_get_rounded_into_an_action(heading):
    graph = graph_input()
    current = estimate(heading_deg=heading, heading_status="certain")
    assert_stopped(next_action(graph, current, plan_from_estimate(graph, current, "N05")),
                   "turn_unresolved", True)


def test_start_equals_goal_stops_without_requiring_orientation():
    graph = graph_input()
    current = estimate("N06")
    route = plan_from_estimate(graph, current, "N06", ["C06"])
    assert route["nodes"] == ["N06"] and route["channels"] == [] and route["length_m"] == 0
    assert_stopped(next_action(graph, current, route, ["C06"]), "reached")


def test_unreachable_stops_without_requiring_orientation():
    graph = graph_input()
    current = estimate("N01")
    route = plan_from_estimate(graph, current, "N06", ["C06"])
    assert route["status"] == "unreachable"
    assert_stopped(next_action(graph, current, route, ["C06"]), "unreachable")


def test_absolute_map_actions_are_metadata_not_control_commands():
    graph = graph_input()
    before = copy.deepcopy(graph)
    current = estimate(heading_deg=0, heading_status="certain")
    route = plan_from_estimate(graph, current, "N05")
    assert graph["nodes"][1]["actions"][2] == "north"
    assert next_action(graph, current, route)["action"] == "left"
    assert graph == before
    graph["nodes"][1]["actions"][2] = "west"
    assert next_action(graph, current, route)["action"] == "left"  # uses channel geometry


def test_stale_route_from_different_estimated_node_cannot_be_executed():
    graph = graph_input()
    route = plan_from_estimate(graph, estimate("N01"), "N06")
    current = estimate("N02", heading_deg=0, heading_status="certain")
    assert_stopped(next_action(graph, current, route), "replan_required")


@pytest.mark.parametrize("disabled,change_enabled", [(["C02"], False), ([], True)])
def test_stale_route_after_closure_stops_until_replanned(disabled, change_enabled):
    graph = graph_input()
    current = estimate(heading_deg=0, heading_status="certain")
    original = plan_from_estimate(graph, current, "N06")
    if change_enabled:
        graph["edges"][1]["enabled"] = False
    assert_stopped(next_action(graph, current, original, disabled), "replan_required")
    rerouted = plan_from_estimate(graph, current, "N06", disabled)
    assert rerouted["nodes"] == ["N02", "N05", "N04", "N06"]
    assert next_action(graph, current, rerouted, disabled)["action"] == "left"


@pytest.mark.parametrize("position", [None, [2], [2, math.nan], [2, "0"], [2, 0]])
def test_missing_invalid_or_zero_length_channel_geometry_requires_observation(position):
    graph = graph_input()
    graph["nodes"][4]["position_m"] = position  # N05; [2,0] coincides with N02
    current = estimate(heading_deg=0, heading_status="certain")
    route = plan_from_estimate(graph, current, "N05")
    assert_stopped(next_action(graph, current, route), "geometry_unknown", True)


@pytest.mark.parametrize("kwargs", [{"goal": "N99"}, {"disabled_channels": ["C99"]}])
def test_existing_dijkstra_validation_is_preserved(kwargs):
    arguments = {"goal": "N06", **kwargs}
    with pytest.raises(ValueError):
        plan_from_estimate(graph_input(), estimate(), **arguments)


def structural_result(status="localized", **overrides):
    pose = {"node_id": "N02", "heading_deg": 0}
    return {"status": status, "candidates": [dict(pose)], "estimate": dict(pose),
            "observation_reliable": True, **overrides}


def test_accepts_unique_reliable_actionable_structural_localization_without_mutating_it():
    graph = graph_input()
    localization = structural_result()
    before = copy.deepcopy((graph, localization))
    route = plan_from_estimate(graph, localization, "N05")
    assert route["nodes"] == ["N02", "N05"]
    assert next_action(graph, localization, route)["action"] == "left"
    assert (graph, localization) == before


@pytest.mark.parametrize("status", ["ambiguous", "observation_uncertain", "execution_unconfirmed",
                                    "execution_failed", "transition_mismatch", "observation_mismatch"])
def test_single_nested_estimate_does_not_override_non_actionable_localization_status(status):
    graph = graph_input()
    localization = structural_result(status)
    route = plan_from_estimate(graph, localization, "N05")
    assert route["status"] == "localization_uncertain"
    assert_stopped(next_action(graph, localization, route), "localization_uncertain", True)


@pytest.mark.parametrize("overrides", [
    {"candidates": []}, {"estimate": None}, {"observation_reliable": False},
    {"candidates": [{"node_id": "N02", "heading_deg": 0}, {"node_id": "N02", "heading_deg": 90}]},
    {"candidates": [{"node_id": "N03", "heading_deg": 0}]},
])
def test_structural_result_must_confirm_one_consistent_pose(overrides):
    graph = graph_input()
    localization = structural_result(**overrides)
    route = plan_from_estimate(graph, localization, "N05")
    assert route["status"] == "localization_uncertain"
    assert_stopped(next_action(graph, localization, route), "localization_uncertain", True)


def test_real_first_block_output_can_feed_new_planning_interface():
    from simulation.structural_localization import update_candidates

    graph = graph_input()
    localization = update_candidates(graph, [{"node_id": "N02", "heading_deg": 0}],
                                     {"relative_open": {"front": True, "left": True,
                                                        "right": False, "back": True},
                                      "confidence": 1.0})
    assert localization["status"] == "localized"
    route = plan_from_estimate(graph, localization, "N05")
    assert route["channels"] == ["C05"]
    assert next_action(graph, localization, route)["action"] == "left"
