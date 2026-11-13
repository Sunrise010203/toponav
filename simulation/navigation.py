"""Discrete 2D navigation: simulated recognition -> localization -> plan -> act."""

import argparse
import json
import math
from pathlib import Path

from simulation.dijkstra import plan


HEADINGS = {"east": 0, "north": 90, "west": 180, "south": 270}


def direction(source, target):
    dx, dy = target[0] - source[0], target[1] - source[1]
    if dy == 0 and dx != 0:
        return "east" if dx > 0 else "west"
    if dx == 0 and dy != 0:
        return "north" if dy > 0 else "south"
    raise ValueError("A channel must connect distinct axis-aligned positions")


class Environment2D:
    """Ground truth is owned here; commands are directions, not destinations."""

    def __init__(self, graph, scene, start=None, disabled_channels=()):
        self.node_id = graph["start_node"] if start is None else start
        self.disabled = set(disabled_channels)
        plan(graph, self.node_id, self.node_id, self.disabled)
        self.positions = {node["node_id"]: node["position_m"] for node in scene["nodes"]}
        if self.node_id not in self.positions:
            raise ValueError(f"Start node absent from physical scene: {self.node_id}")
        self.channels = scene["channels"]
        self.edges = {edge["channel_id"]: edge for edge in graph["edges"]}
        self.heading_deg = scene["start"]["heading_deg"] if self.node_id == scene["start"]["node_id"] else 0
        self.distance_m = 0

    def state(self):
        return {"node_id": self.node_id, "position_m": list(self.positions[self.node_id]),
                "heading_deg": self.heading_deg, "distance_m": self.distance_m}

    def step(self, action):
        if action == "stop":
            return "stopped"
        if action not in HEADINGS:
            raise ValueError(f"Unknown direction action: {action}")
        for channel in self.channels:
            source, target = channel["source"], channel["target"]
            if self.node_id == source:
                neighbor = target
            elif self.node_id == target and channel["bidirectional"]:
                neighbor = source
            else:
                continue
            if direction(self.positions[self.node_id], self.positions[neighbor]) != action:
                continue
            edge = self.edges.get(channel["channel_id"])
            if not edge or not edge["enabled"] or channel["channel_id"] in self.disabled:
                continue
            allowed = (self.node_id, neighbor) == (edge["source"], edge["target"]) or (
                edge["bidirectional"] and (self.node_id, neighbor) == (edge["target"], edge["source"]))
            if not allowed:
                continue
            self.node_id = neighbor
            self.heading_deg = HEADINGS[action]
            self.distance_m += channel["length_m"]
            return "moved"
        return "blocked"


class SimulatedRecognizer:
    """A camera substitute emitting a fresh node-probability snapshot."""

    def __init__(self, graph, confidence=1.0):
        if not math.isfinite(confidence) or not 0 <= confidence <= 1:
            raise ValueError("Recognition confidence must be finite and between 0 and 1")
        self.node_ids = [node["node_id"] for node in graph["nodes"]]
        self.confidence = confidence

    def observe(self, environment):
        if environment.node_id not in self.node_ids:
            raise ValueError("Physical node is absent from the recognition labels")
        if len(self.node_ids) == 1:
            return {self.node_ids[0]: 1.0}
        other = (1 - self.confidence) / (len(self.node_ids) - 1)
        return {node: self.confidence if node == environment.node_id else other
                for node in self.node_ids}


def localize(probabilities, threshold=0.6):
    """Estimate a node from probabilities alone; no ground-truth input."""
    if not math.isfinite(threshold) or not 0 <= threshold <= 1:
        raise ValueError("Localization threshold must be between 0 and 1")
    if not probabilities or any(type(p) not in (int, float) or not math.isfinite(p) or p < 0
                                for p in probabilities.values()):
        raise ValueError("Probabilities must be a nonempty finite nonnegative distribution")
    if not math.isclose(sum(probabilities.values()), 1.0, abs_tol=1e-9):
        raise ValueError("Node probabilities must sum to 1")
    node = max(probabilities, key=probabilities.get)
    confidence = probabilities[node]
    certain = confidence >= threshold and list(probabilities.values()).count(confidence) == 1
    return {"status": "localized" if certain else "uncertain",
            "node_id": node if certain else None, "confidence": confidence}


def decide(graph, localization, goal, disabled_channels=()):
    """Replan from the estimate and select just the first exit direction."""
    if localization["status"] != "localized":
        return {"status": "localization_uncertain", "action": "stop", "route": None}
    route = plan(graph, localization["node_id"], goal, disabled_channels)
    if route["status"] == "unreachable":
        return {"status": "unreachable", "action": "stop", "route": route}
    if len(route["nodes"]) == 1:
        return {"status": "reached", "action": "stop", "route": route}
    node = next(node for node in graph["nodes"] if node["node_id"] == localization["node_id"])
    neighbor = route["nodes"][1]
    action = node["actions"][node["neighbors"].index(neighbor)]
    return {"status": "move", "action": action, "route": route}


def run_navigation(graph, scene, start=None, goal=None, disabled_channels=(),
                   confidence=1.0, max_steps=30):
    """Each iteration reobserves actual environment state, including arrival."""
    if type(max_steps) is not int or max_steps < 0:
        raise ValueError("max_steps must be a nonnegative integer")
    disabled_channels = tuple(disabled_channels)
    goal = graph["goal_node"] if goal is None else goal
    env = Environment2D(graph, scene, start, disabled_channels)
    plan(graph, env.node_id, goal, disabled_channels)
    recognizer = SimulatedRecognizer(graph, confidence)
    records, visited, moves = [], [env.node_id], 0
    while True:
        before = env.state()
        probabilities = recognizer.observe(env)
        localization = localize(probabilities)
        decision = decide(graph, localization, goal, disabled_channels)
        if decision["status"] == "move" and moves >= max_steps:
            decision = {**decision, "status": "step_limit", "action": "stop"}
        execution = env.step(decision["action"])
        records.append({"index": len(records), "before": before,
                        "probabilities": probabilities, "localization": localization,
                        "decision": decision, "execution": execution, "after": env.state()})
        if execution == "moved":
            moves += 1
            visited.append(env.node_id)
            continue
        status = "blocked" if execution == "blocked" else decision["status"]
        # Truth is used only to audit success, never to choose the route/action.
        if status == "reached" and env.node_id != goal:
            status = "false_goal"
        return {"status": status, "start": visited[0], "goal": goal,
                "disabled_channels": list(disabled_channels), "recognition_confidence": confidence,
                "moves": moves, "visited_nodes": visited, "final_state": env.state(), "steps": records}


def plot_trajectory(graph, scene, result, path):
    """Draw physical channels and the executed trajectory (not the plan)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    positions = {node["node_id"]: node["position_m"] for node in scene["nodes"]}
    enabled = {edge["channel_id"] for edge in graph["edges"] if edge["enabled"]}
    enabled -= set(result["disabled_channels"])
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for channel in scene["channels"]:
        a, b = positions[channel["source"]], positions[channel["target"]]
        opened = channel["channel_id"] in enabled
        ax.plot([a[0], b[0]], [a[1], b[1]], color="0.85", linewidth=14, zorder=0)
        ax.plot([a[0], b[0]], [a[1], b[1]], color="0.45" if opened else "#c62828",
                linestyle="-" if opened else "--", linewidth=2)
        ax.text((a[0] + b[0]) / 2, (a[1] + b[1]) / 2 + 0.12,
                channel["channel_id"], fontsize=9)
    trajectory = [result["steps"][0]["before"]["position_m"]]
    trajectory += [record["after"]["position_m"] for record in result["steps"]
                   if record["execution"] == "moved"]
    ax.plot([p[0] for p in trajectory], [p[1] for p in trajectory], "o-",
            color="#1565c0", linewidth=3, label="Executed trajectory", zorder=3)
    for node, position in positions.items():
        ax.scatter(*position, color="black", s=25, zorder=4)
        ax.annotate(node, position, xytext=(0, -17), textcoords="offset points", ha="center")
    ax.scatter(*positions[result["goal"]], s=200, marker="*", color="#2e7d32",
               label="Goal", zorder=5)
    ax.scatter(*result["final_state"]["position_m"], s=130, facecolors="none",
               edgecolors="#ff8f00", linewidths=2, label="Final actual position", zorder=6)
    bounds = scene["bounds_m"]
    distance = result["final_state"]["distance_m"]
    distance_text = f"{distance:g} m" if distance is not None else "unknown distance"
    ax.set(xlim=(bounds["x_min"], bounds["x_max"]),
           ylim=(bounds["y_min"], bounds["y_max"]), xlabel="x / m (east)", ylabel="y / m (north)",
           title=f"{result['status']} | {result['moves']} moves | "
                 f"{distance_text} | discrete node simulation")
    ax.set_aspect("equal")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(alpha=0.2)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main(argv=None):
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=root / "configs" / "legacy" / "graph.json")
    parser.add_argument("--scene", type=Path, default=root / "configs" / "legacy" / "scene.json")
    parser.add_argument("--start")
    parser.add_argument("--goal")
    parser.add_argument("--disable", nargs="+", action="extend", default=[], metavar="CHANNEL")
    parser.add_argument("--confidence", type=float, default=1.0)
    parser.add_argument("--max-steps", type=int, default=30)
    parser.add_argument("--output", type=Path, help="Save full JSON trace")
    parser.add_argument("--plot", type=Path, help="Save actual 2D trajectory PNG")
    args = parser.parse_args(argv)
    try:
        graph = json.loads(args.map.read_text(encoding="utf-8"))
        scene = json.loads(args.scene.read_text(encoding="utf-8"))
        result = run_navigation(graph, scene, args.start, args.goal, args.disable,
                                args.confidence, args.max_steps)
        serialized = json.dumps(result, ensure_ascii=False, allow_nan=False)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(serialized + "\n", encoding="utf-8")
        if args.plot:
            plot_trajectory(graph, scene, result, args.plot)
    except (OSError, ValueError, KeyError) as error:
        print(json.dumps({"status": "invalid", "error": str(error)}, ensure_ascii=False))
        return 2
    print(serialized)
    return 0 if result["status"] == "reached" else 1


if __name__ == "__main__":
    raise SystemExit(main())
