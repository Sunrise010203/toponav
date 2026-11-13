"""Image-driven third-block integration; does not call the ideal structure sensor."""
import argparse
import json
import time
from collections import Counter
from pathlib import Path

import numpy as np
import torch

from roadvision.learn import Predictor
from roadvision.scene import LAYOUTS, World, topology_pair


class CameraSensor:
    """Owns simulator truth for rendering only; returns RGB and frame times."""
    def __init__(self, environment, world, cells, seed):
        self.environment, self.world, self.cells = environment, world, cells
        self.rng = np.random.default_rng(seed)

    def capture(self, frame_id, fault=None):
        state = self.environment.state()
        image, _ = self.world.sample(0, self.rng, pose=(self.cells[state["node_id"]], state["heading_deg"]))
        captured = time.time_ns()
        if fault == "missing_view":
            image[:128, :128] = 0
        elif fault == "stale_frame":
            captured -= 2_000_000_000
        return image, captured, frame_id


def run_navigation(predictor, layout_id, start, heading, goal, seed=20261014, fault=None, max_steps=32):
    from simulation.structural_navigation import StructuralEnvironment
    from simulation.structural_localization import update_candidates, decide
    graph, scene, cells = topology_pair(layout_id)
    environment = StructuralEnvironment(graph, scene, start, heading)
    candidates = [{"node_id": start, "heading_deg": heading}]  # Task-given prior, never read from truth.
    previous_action, feedback, steps = None, None, []
    with World(layout_id) as world:
        sensor = CameraSensor(environment, world, cells, seed)
        while True:
            index = len(steps)
            image, captured, frame_id = sensor.capture(f"{layout_id}_{seed}_{index}", fault if index == 0 else None)
            packet = predictor.predict(image, captured_at_ns=captured, frame_id=frame_id)
            # Strict payload only; no pixels, probabilities, true pose or simulator access in controller calls.
            localization = update_candidates(graph, candidates, packet["observation"], previous_action, feedback)
            decision = decide(graph, localization, goal)
            if decision["status"] == "move" and index >= max_steps:
                decision = {**decision, "status": "step_limit", "action": "stop"}
            before = environment.state()  # Audit only; cannot alter the decision.
            execution = environment.execute(decision["action"])
            steps.append({"prediction": packet, "localization": localization, "decision": decision,
                          "feedback_used": feedback, "execution_feedback": execution,
                          "audit_truth": {"before": before, "after": environment.state()}})
            if decision["action"] == "stop":
                break
            candidates = localization["candidates"]
            previous_action, feedback = decision["action"], execution
    actual = environment.state()
    return {"layout_id": layout_id, "start_prior": start, "heading_prior": heading, "goal": goal,
            "seed": seed, "fault": fault, "status": decision["status"], "steps": steps,
            "audit": {"goal_verified": decision["status"] == "reached" and actual["node_id"] == goal,
                      "final_truth": actual,
                      "estimate_matches_truth": localization["estimate"] == {
                          "node_id": actual["node_id"], "heading_deg": actual["heading_deg"]}}}


def suite(predictor, layout_id):
    # Declared before test results: each node, four headings, goal seven IDs ahead (wrapping).
    graph, _, _ = topology_pair(layout_id)
    nodes = [n["node_id"] for n in graph["nodes"]]
    tasks = [(start, heading, nodes[(index+7) % len(nodes)])
             for index, start in enumerate(nodes) for heading in (0, 90, 180, 270)]
    runs = []
    started = time.perf_counter()
    for index, (start, heading, goal) in enumerate(tasks):
        result = run_navigation(predictor, layout_id, start, heading, goal, seed=20261014+index)
        runs.append(result)
        print(f"Image-driven task {index+1}/{len(tasks)}: {result['status']}", flush=True)
    safety = [run_navigation(predictor, layout_id, nodes[0], 0, nodes[7 % len(nodes)], fault=fault)
              for fault in ("missing_view", "stale_frame")]
    steps = [step for run in runs for step in run["steps"]]
    return {"model_sha256": predictor.metadata["model_sha256"], "layout_id": layout_id,
            "node_count": len(nodes), "classification_dimension": 17,
            "task_count": len(runs), "verified_successes": sum(r["audit"]["goal_verified"] for r in runs),
            "success_rate": sum(r["audit"]["goal_verified"] for r in runs)/len(runs),
            "status_counts": dict(Counter(r["status"] for r in runs)),
            "observation_acceptance_rate": sum(s["prediction"]["accepted"] for s in steps)/len(steps),
            "elapsed_s": time.perf_counter()-started, "runs": runs, "safety_runs": safety,
            "scope": "Known task start/heading prior, atomic topological execution, simplified simulator only; no continuous control or geometry estimation"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--layout", choices=tuple(LAYOUTS), default="test_locked")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("New report path required")
    torch.set_num_threads(2)
    report = suite(Predictor(args.run, "cpu"), args.layout)
    with args.output.open("x") as file:
        json.dump(report, file, indent=2)
    print(json.dumps({key: value for key, value in report.items() if key not in ("runs", "safety_runs")}, indent=2))


if __name__ == "__main__":
    main()
