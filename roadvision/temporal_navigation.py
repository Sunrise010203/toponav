"""Real RGB Predictor adapter for passive temporal navigation; no ideal fallback."""
import argparse
import hashlib
import json
import time
from pathlib import Path

from roadvision.learn import Predictor
from roadvision.navigation import CameraSensor
from roadvision.scene import LAYOUTS, World, topology_pair
from simulation.temporal_navigation import SCENARIOS, run_navigation as drive


class ImageSensor:
    def __init__(self, environment, predictor, layout_id, cells, seed, visual_fault):
        self.predictor, self.layout_id, self.seed = predictor, layout_id, seed
        self.visual_fault = visual_fault
        self.world = World(layout_id)
        self.camera = CameraSensor(environment, self.world, cells, seed)

    def capture(self, index):
        fault = self.visual_fault if index == 0 else None
        image, captured, identity = self.camera.capture(f"{self.layout_id}-{self.seed}-{index}", fault)
        if fault == "future_frame":
            captured += 2_000_000_000
        packet = self.predictor.predict(image, captured_at_ns=captured, frame_id=identity)
        return {key: packet[key] for key in ("frame_id", "captured_at_ns", "observation")}, packet

    def close(self):
        self.world.__exit__(None, None, None)


def run_navigation(predictor, layout_id, start, heading, goal, seed=20261014,
                   visual_fault=None, **kwargs):
    if visual_fault not in {None, "missing_view", "stale_frame", "future_frame"}:
        raise ValueError("Unknown visual fault")
    graph, scene, cells = topology_pair(layout_id)
    result = drive(graph, scene, start, heading, goal,
                   sensor_factory=lambda env: ImageSensor(env, predictor, layout_id, cells, seed, visual_fault),
                   **kwargs)
    result.update({"layout_id": layout_id, "seed": seed, "visual_fault": visual_fault,
                   "model_sha256": predictor.metadata["model_sha256"]})
    return result


def suite(predictor, layout_id, **kwargs):
    """Same predefined 15 nodes x 4 headings as the exposed old regression."""
    graph, _, _ = topology_pair(layout_id)
    nodes = [n["node_id"] for n in graph["nodes"]]
    tasks = [(start, heading, nodes[(index + 7) % len(nodes)])
             for index, start in enumerate(nodes) for heading in (0, 90, 180, 270)]
    started = time.monotonic()
    runs = []
    for index, (start, heading, goal) in enumerate(tasks):
        run = run_navigation(predictor, layout_id, start, heading, goal, seed=20261014 + index, **kwargs)
        runs.append(run)
        print(f"Temporal image task {index + 1}/{len(tasks)}: {run['state']} / {run['reason']}", flush=True)
    return {"version": "temporal_image_suite_v1", "layout_id": layout_id,
            "model_sha256": predictor.metadata["model_sha256"], "task_count": len(runs),
            "verified_successes": sum(r["audit"]["goal_verified"] for r in runs),
            "false_arrivals": sum(r["audit"]["false_arrival"] for r in runs),
            "mislocalized_commands": sum(r["audit"]["mislocalized_commands"] for r in runs),
            "elapsed_s": time.monotonic() - started, "runs": runs,
            "scope": "Exposed test_locked regression, not a new independent generalization evaluation"}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--layout", choices=tuple(LAYOUTS), default="test_locked")
    parser.add_argument("--start", default="R000")
    parser.add_argument("--heading", type=int, default=0)
    parser.add_argument("--goal", default="R007")
    parser.add_argument("--scenario", choices=SCENARIOS, default="ideal")
    parser.add_argument("--visual-fault", choices=("missing_view", "stale_frame", "future_frame"))
    parser.add_argument("--suite", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError("New report path required")
        import torch
        torch.set_num_threads(2)
        predictor = Predictor(args.run, "cpu")
        if args.suite:
            if args.visual_fault or args.scenario != "ideal":
                raise ValueError("Suite fixes ideal conditions; use single-task faults separately")
            result = suite(predictor, args.layout)
            success = result["verified_successes"] == result["task_count"]
        else:
            result = run_navigation(predictor, args.layout, args.start, args.heading, args.goal,
                                    visual_fault=args.visual_fault, scenario=args.scenario)
            success = result["audit"]["goal_verified"]
        result["source_hashes"] = {
            str(path.relative_to(Path(__file__).resolve().parents[1])): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (Path(__file__).resolve(), Path(__file__).resolve().parents[1] / "simulation/navigation_session.py",
                         Path(__file__).resolve().parents[1] / "simulation/temporal_navigation.py")}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x") as file:
            json.dump(result, file, ensure_ascii=False, indent=2, allow_nan=False)
        print(json.dumps({key: value for key, value in result.items()
                          if key not in {"runs", "steps", "controller"}}, ensure_ascii=False))
    except (OSError, ValueError, KeyError) as error:
        print(json.dumps({"state": "invalid", "error": str(error)}, ensure_ascii=False))
        return 2
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
