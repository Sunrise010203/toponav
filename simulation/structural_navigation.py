"""Structure-only navigation simulator; actual state stays outside the controller."""

import argparse
import json
from pathlib import Path

from simulation.dijkstra import plan
from simulation.navigation import Environment2D, direction, plot_trajectory
from simulation.structural_localization import TURNS, absolute_action, decide, update_candidates
from simulation.topology import HEADINGS, cardinal_heading, is_v2, task_values, validate_pair


SCENARIOS = ("ideal", "execution_failed", "execution_unknown", "execution_sent", "ambiguous", "mismatch")


class _TopologicalWorld:
    """Discrete physics from independently specified channel endpoint directions."""

    def __init__(self, graph, scene, start, heading, disabled_channels):
        self.node_id, self.heading_deg = start, heading
        self.disabled = set(disabled_channels)
        self.channels = scene["channels"]
        self.edges = {e["channel_id"]: e for e in graph["edges"]}
        geometry = scene.get("geometry", {})
        self.positions = geometry.get("node_positions_m", {})
        self.dimensions = geometry.get("channel_dimensions_m", {})
        self.distance_m = 0

    def state(self):
        position = self.positions.get(self.node_id)
        return {"node_id": self.node_id, "position_m": list(position) if position is not None else None,
                "heading_deg": self.heading_deg, "distance_m": self.distance_m}

    def step(self, action):
        if action == "stop":
            return "stopped"
        for channel in self.channels:
            if self.node_id == channel["source"]:
                neighbor, outgoing, incoming = (channel["target"], channel["source_direction"],
                                                 channel["target_direction"])
            elif self.node_id == channel["target"] and channel["bidirectional"]:
                neighbor, outgoing, incoming = (channel["source"], channel["target_direction"],
                                                 channel["source_direction"])
            else:
                continue
            edge = self.edges[channel["channel_id"]]
            if outgoing != action or not edge["enabled"] or channel["channel_id"] in self.disabled:
                continue
            self.node_id = neighbor
            self.heading_deg = (HEADINGS[incoming] + 180) % 360
            length = self.dimensions.get(channel["channel_id"], {}).get("length")
            self.distance_m = (self.distance_m + length
                               if self.distance_m is not None and length is not None else None)
            return "moved"
        return "blocked"


class StructuralEnvironment:
    """Truth-owning physics/sensor simulator, never an input to localization."""

    def __init__(self, graph, scene, start=None, heading=0, disabled_channels=()):
        heading = cardinal_heading(heading)
        validate_pair(graph, scene)
        self.modern = is_v2(graph)
        start, _ = task_values(graph, start)
        plan(graph, start, start, disabled_channels)
        self._world = (_TopologicalWorld(graph, scene, start, heading, disabled_channels)
                       if self.modern else Environment2D(graph, scene, start, disabled_channels))
        self._world.heading_deg = heading
        self.last_execution = "stopped"

    def state(self):
        return self._world.state()

    def observe(self, mode="ideal"):
        """Ideal physical mouth structure, independent of traversal policy.

        A disabled edge may be obstructed farther along the channel; its mouth
        does not disappear. This topology-only sensor does not model RGB or
        occlusion. Execution still enforces closures and one-way restrictions.
        """
        world = self._world
        opened = set()
        for channel in world.channels:
            if world.node_id == channel["source"]:
                neighbor = channel["target"]
            elif world.node_id == channel["target"]:
                neighbor = channel["source"]
            else:
                continue
            if self.modern:
                endpoint = "source" if world.node_id == channel["source"] else "target"
                opened.add(channel[endpoint + "_direction"])
            else:
                opened.add(direction(world.positions[world.node_id], world.positions[neighbor]))
        observed = {relative: absolute_action(world.heading_deg, relative) in opened for relative in TURNS}
        if mode == "ambiguous":
            return {"relative_open": {relative: None for relative in TURNS}, "confidence": 0.2}
        if mode == "mismatch":
            observed["front"] = not observed["front"]
        elif mode != "ideal":
            raise ValueError(f"Unknown observation mode: {mode}")
        return {"relative_open": observed, "confidence": 1.0}

    def execute(self, action, outcome="normal"):
        if action == "stop":
            self.last_execution = "stopped"
            return {"status": "stopped"}
        absolute = absolute_action(self._world.heading_deg, action)
        if outcome not in {"normal", "failed", "unknown", "sent"}:
            raise ValueError(f"Unknown simulated execution outcome: {outcome}")
        if outcome in {"failed", "sent"}:
            self.last_execution = "not_executed"
            return {"status": outcome}
        self.last_execution = self._world.step(absolute)
        status = "succeeded" if self.last_execution == "moved" else "failed"
        return {"status": "unknown" if outcome == "unknown" else status}


def run_navigation(graph, scene, start=None, heading=None, goal=None, disabled_channels=(),
                   scenario="ideal", fault_step=1, max_steps=30, actual_start=None, actual_heading=None):
    """Task prior and simulator initial truth are independently configurable.

    actual_* are simulation-only fault fixtures; they are never read to seed,
    repair or advance the controller's candidates. There is no truth feedback.
    """
    if scenario not in SCENARIOS:
        raise ValueError(f"Unknown scenario: {scenario}")
    if type(max_steps) is not int or max_steps < 0 or type(fault_step) is not int or fault_step < 0:
        raise ValueError("max_steps and fault_step must be nonnegative integers")
    validate_pair(graph, scene)
    start, goal = task_values(graph, start, goal)
    if heading is None:
        if is_v2(graph):
            heading = graph.get("task", {}).get("initial_heading_deg")
        else:
            heading = scene["start"]["heading_deg"] if start == scene["start"]["node_id"] else 0
    heading = cardinal_heading(heading)
    disabled_channels = tuple(disabled_channels)
    plan(graph, start, goal, disabled_channels)
    absolute_action(heading, "front")
    candidates = [{"node_id": start, "heading_deg": heading}]  # task prior, not env.state()
    env = StructuralEnvironment(graph, scene, start if actual_start is None else actual_start,
                                heading if actual_heading is None else actual_heading, disabled_channels)
    steps, commands_sent = [], 0
    previous_action, feedback = None, None
    while True:
        index = len(steps)
        mode = scenario if scenario in {"ambiguous", "mismatch"} and index == fault_step else "ideal"
        observation = env.observe(mode)
        localization = update_candidates(graph, candidates, observation, previous_action, feedback,
                                          disabled_channels)
        decision = decide(graph, localization, goal, disabled_channels)
        if decision["status"] == "move" and commands_sent >= max_steps:
            decision = {**decision, "status": "step_limit", "action": "stop"}
        outcome = scenario.removeprefix("execution_") if (
            scenario.startswith("execution_") and commands_sent == fault_step) else "normal"
        # Truth is collected only for the log; neither controller call receives it.
        before = env.state()
        execution_feedback = env.execute(decision["action"], outcome)
        steps.append({"index": index, "truth": {"before": before, "after": env.state(),
                                                "execution": env.last_execution},
                      "prior_candidates": candidates, "previous_action": previous_action,
                      "feedback_used": feedback, "observation": observation, "localization": localization,
                      "decision": decision, "execution_feedback": execution_feedback})
        if decision["action"] == "stop":
            break
        commands_sent += 1
        candidates = localization["candidates"]
        previous_action, feedback = decision["action"], execution_feedback

    # One-way terminal audit: these values never affect estimation or decisions.
    actual_moves = [s for s in steps if s["truth"]["execution"] == "moved"]
    final_truth = env.state()
    final_estimate = localization["estimate"]
    return {"version": "unified_topology_v2", "status": decision["status"], "start_prior": start,
            "heading_prior": heading, "goal": goal, "scenario": scenario, "fault_step": fault_step,
            "disabled_channels": list(disabled_channels), "commands_sent": commands_sent,
            "final_truth": final_truth, "final_estimate": final_estimate, "steps": steps,
            "audit": {"goal_verified": decision["status"] == "reached" and final_truth["node_id"] == goal,
                      "estimate_matches_truth": final_estimate == {"node_id": final_truth["node_id"],
                                                                  "heading_deg": final_truth["heading_deg"]},
                      "actual_moves": len(actual_moves),
                      "visited_nodes": [steps[0]["truth"]["before"]["node_id"]] + [
                          s["truth"]["after"]["node_id"] for s in actual_moves]}}


def plot_run(graph, scene, result, path):
    """Reuse baseline plotting for audit-only actual trajectory, not controller input."""
    if is_v2(graph):
        geometry = scene.get("geometry", {})
        positions = geometry.get("node_positions_m", {})
        if not all(n["node_id"] in positions for n in graph["nodes"]):
            raise ValueError("Plotting requires geometry.node_positions_m for every node")
        xs, ys = zip(*positions.values())
        scene = {"nodes": [{"node_id": node, "position_m": position} for node, position in positions.items()],
                 "channels": scene["channels"],
                 "bounds_m": geometry.get("bounds_m", {"x_min": min(xs)-1, "x_max": max(xs)+1,
                                                       "y_min": min(ys)-1, "y_max": max(ys)+1})}
    plot_trajectory(graph, scene, {
        "status": result["status"], "goal": result["goal"], "disabled_channels": result["disabled_channels"],
        "final_state": result["final_truth"], "moves": result["audit"]["actual_moves"],
        "steps": [{"before": s["truth"]["before"], "after": s["truth"]["after"],
                   "execution": s["truth"]["execution"]} for s in result["steps"]]}, path)


def main(argv=None):
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=root / "configs" / "graph.json")
    parser.add_argument("--scene", type=Path, default=root / "configs" / "scene.json")
    parser.add_argument("--start", help="Task-given start prior, not a sensor measurement")
    parser.add_argument("--heading", type=int, help="Confirmed heading; defaults to task.initial_heading_deg")
    parser.add_argument("--goal")
    parser.add_argument("--disable", nargs="+", action="extend", default=[], metavar="CHANNEL")
    parser.add_argument("--scenario", choices=SCENARIOS, default="ideal")
    parser.add_argument("--fault-step", type=int, default=1, help="Zero-based observation/command index")
    parser.add_argument("--max-steps", type=int, default=30, help="Maximum movement commands, not true moves")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--plot", type=Path)
    args = parser.parse_args(argv)
    try:
        graph = json.loads(args.map.read_text(encoding="utf-8"))
        scene = json.loads(args.scene.read_text(encoding="utf-8"))
        result = run_navigation(graph, scene, args.start, args.heading, args.goal, args.disable,
                                args.scenario, args.fault_step, args.max_steps)
        serialized = json.dumps(result, ensure_ascii=False, allow_nan=False)
        if args.plot:
            plot_run(graph, scene, result, args.plot)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(serialized + "\n", encoding="utf-8")
    except (OSError, ValueError, KeyError) as error:
        print(json.dumps({"status": "invalid", "error": str(error)}, ensure_ascii=False))
        return 2
    print(serialized)
    return 0 if result["audit"]["goal_verified"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
