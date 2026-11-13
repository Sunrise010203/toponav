"""Physical openings are evidence, never permission to traverse a channel.

Ideal observations and fake model predictions exercise contracts, not pixel
recognition, continuous driving or formal Gazebo/ROS acceptance.
"""
import copy
import json
from pathlib import Path

import pytest

from roadvision.model_route import ModelRouteController
from simulation import structural_localization as localization
from simulation.path_planning import next_action
from simulation.structural_navigation import StructuralEnvironment, run_navigation
from simulation.temporal_navigation import run_navigation as run_temporal
from simulation.topology import validate_pair


ROOT = Path(__file__).resolve().parents[1]
DIRECTIONS = ("front", "left", "right", "back")
UNKNOWN = {"relative_open": dict.fromkeys(DIRECTIONS), "confidence": 0.0}


def inputs(version="v2"):
    directory = ROOT / "configs"
    if version == "legacy":
        directory /= "legacy"
    return [json.loads((directory / name).read_text()) for name in ("graph.json", "scene.json")]


def pose(node="N02", heading=0):
    return {"node_id": node, "heading_deg": heading}


def observation(*opened):
    return {"relative_open": {d: d in opened for d in DIRECTIONS}, "confidence": 1.0}


def close(graph, channels, mode):
    if mode == "enabled_flag":
        for edge in graph["edges"]:
            if edge["channel_id"] in channels:
                edge["enabled"] = False
        return ()
    return tuple(channels)


@pytest.mark.parametrize("version", ["legacy", "v2"])
@pytest.mark.parametrize("mode", ["enabled_flag", "temporary"])
@pytest.mark.parametrize("heading", [0, 90, 180, 270])
def test_disabled_channel_does_not_erase_physical_signature(version, mode, heading):
    graph, _ = inputs(version)
    candidate = pose(heading=heading)
    expected = localization.map_exits(graph, candidate)
    disabled = close(graph, ("C02",), mode)
    before = copy.deepcopy(graph)
    assert localization.map_exits(graph, candidate, disabled) == expected
    assert graph == before


@pytest.mark.parametrize("version", ["legacy", "v2"])
@pytest.mark.parametrize("mask", range(64))
def test_ideal_sensor_structure_is_independent_of_all_closure_subsets(version, mask):
    graph, scene = inputs(version)
    reference = copy.deepcopy(graph)
    disabled = [edge["channel_id"] for i, edge in enumerate(graph["edges"]) if mask & (1 << i)]
    # Mix persistent and task-local closures, with no effect on physical mouths.
    close(graph, disabled[::2], "enabled_flag")
    for node in graph["nodes"]:
        for heading in (0, 90, 180, 270):
            actual = StructuralEnvironment(graph, scene, node["node_id"], heading, disabled[1::2])
            expected = StructuralEnvironment(reference, scene, node["node_id"], heading).observe()
            assert actual.observe() == expected


@pytest.mark.parametrize("version", ["legacy", "v2"])
@pytest.mark.parametrize("mode", ["enabled_flag", "temporary"])
def test_structure_localizes_but_planner_detours_around_visible_closed_exit(version, mode):
    graph, _ = inputs(version)
    disabled = close(graph, ("C02",), mode)
    before = copy.deepcopy(graph)
    loc = localization.update_candidates(graph, [pose()], observation("front", "left", "back"),
                                         disabled_channels=disabled)
    assert loc["status"] == "localized" and loc["estimate"] == pose()
    decision = localization.decide(graph, loc, "N06", disabled)
    assert decision["action"] == "left"
    assert decision["route"]["nodes"] == ["N02", "N05", "N04", "N06"]
    assert "C02" not in decision["route"]["channels"]
    assert localization.traversable_exits(graph, pose(), disabled) == {"left": "N05", "back": "N01"}
    assert graph == before


@pytest.mark.parametrize("version", ["legacy", "v2"])
@pytest.mark.parametrize("mode", ["enabled_flag", "temporary"])
@pytest.mark.parametrize("status", [None, "sent", "unknown", "failed", "succeeded"])
def test_visible_closed_exit_cannot_generate_a_motion_transition(version, mode, status):
    graph, _ = inputs(version)
    disabled = close(graph, ("C02",), mode)
    result = localization.update_candidates(graph, [pose()], UNKNOWN, "straight",
                                            None if status is None else {"status": status}, disabled)
    expected = [] if status == "succeeded" else [pose()]
    assert result["predicted_candidates"] == expected
    assert result["status"] == ("transition_mismatch" if status == "succeeded" else
                                "execution_failed" if status == "failed" else "execution_unconfirmed")
    assert localization.decide(graph, result, "N06", disabled)["action"] == "stop"


@pytest.mark.parametrize("mode", ["enabled_flag", "temporary"])
def test_unreachable_goal_stays_stopped_despite_a_visible_exit(mode):
    graph, scene = inputs()
    disabled = close(graph, ("C06",), mode)
    observed = StructuralEnvironment(graph, scene, "N04", 0, disabled).observe()
    assert observed == observation("front", "right", "back")
    loc = localization.update_candidates(graph, [pose("N04")], observed, disabled_channels=disabled)
    assert loc["status"] == "localized"
    decision = localization.decide(graph, loc, "N06", disabled)
    assert decision["status"] == "unreachable" and decision["action"] == "stop"


@pytest.mark.parametrize("mode", ["enabled_flag", "temporary"])
def test_closure_does_not_excuse_genuinely_contradictory_observation(mode):
    graph, _ = inputs()
    disabled = close(graph, ("C02",), mode)
    result = localization.update_candidates(graph, [pose()], observation("left", "back"),
                                            disabled_channels=disabled)
    assert result["status"] == "observation_mismatch"
    assert result["candidates"] == []  # Do not invent a replacement position.
    assert localization.decide(graph, result, "N06", disabled)["action"] == "stop"


def test_route_planned_before_closure_is_rejected_not_authorized_by_visible_structure():
    graph, _ = inputs()
    loc = localization.update_candidates(graph, [pose()], observation("front", "left", "back"))
    route = localization.decide(graph, loc, "N06")["route"]
    assert route["channels"][0] == "C02"
    assert next_action(graph, loc, route, ("C02",))["status"] == "replan_required"
    assert next_action(graph, loc, route, ("C02",))["action"] == "stop"


@pytest.mark.parametrize("version", ["legacy", "v2"])
@pytest.mark.parametrize("mode", ["enabled_flag", "temporary"])
def test_executor_still_rejects_a_direct_command_through_visible_closed_exit(version, mode):
    graph, scene = inputs(version)
    disabled = close(graph, ("C02",), mode)
    env = StructuralEnvironment(graph, scene, "N02", 0, disabled)
    assert env.observe() == observation("front", "left", "back")
    before = env.state()
    assert env.execute("straight") == {"status": "failed"}
    assert env.state() == before


def test_legacy_one_way_channel_is_visible_from_both_ends_but_not_reversible():
    graph, scene = inputs("legacy")
    baseline = StructuralEnvironment(graph, scene, "N03", 180).observe()
    for collection in (graph["edges"], scene["channels"]):
        next(edge for edge in collection if edge["channel_id"] == "C02")["bidirectional"] = False
    env = StructuralEnvironment(graph, scene, "N03", 180)
    assert env.observe() == baseline
    assert localization.map_exits(graph, pose("N03", 180))["front"] == "N02"
    assert "front" not in localization.traversable_exits(graph, pose("N03", 180))
    before = env.state()
    assert env.execute("straight") == {"status": "failed"}
    assert env.state() == before
    loc = localization.update_candidates(graph, [pose("N03", 180)], baseline)
    assert loc["status"] == "localized"
    assert localization.decide(graph, loc, "N02")["route"]["nodes"] == ["N03", "N04", "N05", "N02"]


@pytest.mark.parametrize("version", ["legacy", "v2"])
@pytest.mark.parametrize("mode", ["enabled_flag", "temporary"])
def test_ideal_discrete_detour_keeps_physical_evidence_and_never_executes_closed_edge(version, mode):
    graph, scene = inputs(version)
    disabled = close(graph, ("C02",), mode)
    validate_pair(graph, scene)
    before = copy.deepcopy((graph, scene))
    result = run_navigation(graph, scene, disabled_channels=disabled)
    assert result["audit"]["goal_verified"]
    assert result["audit"]["visited_nodes"] == ["N01", "N02", "N05", "N04", "N06"]
    station = result["steps"][1]
    assert station["observation"] == observation("front", "left", "back")
    assert station["decision"]["action"] == "left"
    assert (graph, scene) == before


class Clock:
    now = 0.0

    def sleep(self, seconds):
        self.now += seconds

    def wall_ns(self):
        return 1_000_000_000_000 + int(self.now * 1e9)


@pytest.mark.parametrize("mode", ["enabled_flag", "temporary"])
def test_temporal_session_requires_fresh_physical_evidence_on_detour(mode):
    graph, scene = inputs()
    disabled = close(graph, ("C02",), mode)
    clock = Clock()
    result = run_temporal(graph, scene, disabled_channels=disabled, clock=lambda: clock.now,
                          wall_ns=clock.wall_ns, sleep=clock.sleep, poll_interval_s=.1)
    assert result["state"] == "reached" and result["audit"]["goal_verified"]
    assert result["audit"]["actual_moves"] == 4
    assert result["audit"]["mislocalized_commands"] == 0
    assert result["controller"]["metrics"]["frames_received"] == 15
    assert result["controller"]["metrics"]["transitions_applied"] == 4


class FakePerception:
    metadata = {"model_sha256": "fake-contract-test-not-model-validation"}


def test_model_route_matches_visible_signature_but_authorizes_only_detour():
    graph, _ = inputs()
    close(graph, ("C02",), "enabled_flag")
    controller = ModelRouteController(graph, FakePerception())
    controller.estimate = pose()
    controller.prediction = {"accepted": True, "class_id": "exits_1101", "score": 1.,
                             "observation": observation("front", "left", "back")}
    assert controller._matches(pose())
    # Exercise the actual node-confirmation/planning method, not RGB servoing.
    controller._authorize({"frame_id": "contract-only", "captured_sim_s": 1.}, 1.)
    assert controller.localization["status"] == "localized"
    assert controller.visited == ["N02"]
    assert controller.edge_action == "left" and controller.target == "N05"
    assert "C02" not in controller.planner["route"]["channels"]
    assert controller.motion_commands == 0
