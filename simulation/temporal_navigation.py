"""Temporal session driver and separate ideal-sensor regression CLI.

The driver owns simulator truth only for rendering, execution and one-way audit.
The coordinator receives strict event envelopes, never the environment object.
"""
import argparse
import copy
import json
import math
import time
from pathlib import Path

from simulation.navigation_session import NavigationSession, SessionConfig, TERMINAL
from simulation.structural_navigation import StructuralEnvironment
from simulation.topology import cardinal_heading, task_values, validate_pair


SCENARIOS = ("ideal", "uncertain_once", "mismatch_once", "mismatch_persistent", "stale_once",
             "future_once", "replay_once", "out_of_order_once", "drop_once", "drop_all",
             "execution_failed_once", "execution_failed_always", "execution_unknown",
             "execution_sent", "late_success", "duplicate_feedback", "conflicting_feedback")


class IdealSensor:
    def __init__(self, environment, wall_ns):
        self.environment, self.wall_ns = environment, wall_ns

    def capture(self, index):
        observation = self.environment.observe()
        return {"frame_id": f"ideal-{index}", "captured_at_ns": self.wall_ns(),
                "observation": observation}, None

    def close(self):
        pass


def run_navigation(graph, scene, start=None, heading=None, goal=None, disabled_channels=(),
                   config=None, scenario="ideal", fault_index=0, sensor_factory=None,
                   max_cycles=1000, poll_interval_s=0.01, clock=time.monotonic,
                   wall_ns=time.time_ns, sleep=time.sleep, actual_start=None, actual_heading=None,
                   session_id=None):
    if scenario not in SCENARIOS:
        raise ValueError(f"Unknown scenario: {scenario}")
    if type(fault_index) is not int or fault_index < 0 or type(max_cycles) is not int or max_cycles < 1:
        raise ValueError("Invalid fault index or cycle limit")
    if (type(poll_interval_s) not in (int, float) or not math.isfinite(poll_interval_s)
            or poll_interval_s <= 0):
        raise ValueError("Polling interval must be finite and positive")
    validate_pair(graph, scene)
    start, goal = task_values(graph, start, goal)
    heading = cardinal_heading(graph.get("task", {}).get("initial_heading_deg") if heading is None else heading)
    session = NavigationSession(graph, start, heading, goal, disabled_channels=disabled_channels,
                                config=config, clock=clock, wall_ns=wall_ns, session_id=session_id)
    environment = StructuralEnvironment(graph, scene, start if actual_start is None else actual_start,
                                        heading if actual_heading is None else actual_heading, disabled_channels)
    sensor = None
    steps, queued, cached = [], [], None
    event_number, commands, waiting_time_s = 0, 0, 0.0
    previous_cycle, previous_waiting = clock(), False

    def deliver(command_id, payload, repeat=False):
        nonlocal event_number
        event_number += 1
        event = {"event_id": f"exec-{event_number}", "command_id": command_id, "feedback": payload}
        session.feedback(event)
        if repeat:
            session.feedback(event)
            session.feedback({**event, "event_id": event["event_id"] + "-replay"})
        return event

    try:
        sensor = sensor_factory(environment) if sensor_factory else IdealSensor(environment, wall_ns)
        for index in range(max_cycles):
            if session.state in TERMINAL:
                break
            cycle_now = clock()
            if previous_waiting:
                waiting_time_s += max(0.0, cycle_now - previous_cycle)
            previous_cycle = cycle_now
            for item in list(queued):
                due, command_id, payload = item
                if index >= due:
                    deliver(command_id, payload)
                    queued.remove(item)
            before = environment.state()  # Audit only.
            injection = None
            if scenario == "drop_all" or (scenario == "drop_once" and index == fault_index):
                frame, prediction, injection = None, None, "dropped_frame"
            else:
                frame, prediction = sensor.capture(index)
            if frame is not None:
                frame = copy.deepcopy(frame)
                if scenario in {"replay_once", "out_of_order_once"} and index == max(1, fault_index):
                    if cached is not None:
                        frame = copy.deepcopy(cached)
                        if scenario == "out_of_order_once":
                            frame["frame_id"] += "-out-of-order"
                            frame["captured_at_ns"] -= 1
                        injection = scenario
                cached = copy.deepcopy(frame)
                if index == fault_index or scenario == "mismatch_persistent":
                    if scenario == "uncertain_once":
                        frame["observation"]["confidence"] = 0.2
                        injection = scenario
                    elif scenario in {"mismatch_once", "mismatch_persistent"}:
                        value = frame["observation"]["relative_open"]["front"]
                        frame["observation"]["relative_open"]["front"] = not value
                        injection = scenario
                    elif scenario == "stale_once":
                        frame["captured_at_ns"] -= 2_000_000_000
                        injection = scenario
                    elif scenario == "future_once":
                        frame["captured_at_ns"] += 2_000_000_000
                        injection = scenario
                output = session.observe(frame)
            else:
                output = session.tick()
            execution_event = None
            if output["command_id"] is not None:
                fault_here = commands == fault_index
                outcome = "normal"
                if scenario == "execution_failed_always" or (fault_here and scenario == "execution_failed_once"):
                    outcome = "failed"
                elif fault_here and scenario in {"execution_unknown", "execution_sent"}:
                    outcome = scenario.removeprefix("execution_")
                public_feedback = environment.execute(output["action"], outcome)
                execution = environment.last_execution
                if fault_here and scenario == "late_success":
                    queued.append((index + 3, output["command_id"], public_feedback))
                    public_feedback = {"status": "unknown"}
                execution_event = deliver(output["command_id"], public_feedback,
                                           repeat=fault_here and scenario == "duplicate_feedback")
                if fault_here and scenario == "conflicting_feedback":
                    deliver(output["command_id"], {"status": "failed"})
                commands += 1
            else:
                environment.execute("stop")  # Safety request, not completion of a pending move.
                execution = "stopped"
            steps.append({"index": index, "frame": frame, "prediction": prediction,
                          "injection": injection, "controller_output": output,
                          "execution_event": execution_event,
                          "truth": {"before": before, "after": environment.state(), "execution": execution}})
            previous_waiting = output["action"] == "stop"
            if session.state not in TERMINAL:
                sleep(poll_interval_s)
        if session.state not in TERMINAL:
            session.abort("driver_cycle_limit")
    finally:
        environment.execute("stop")
        if sensor is not None:
            sensor.close()

    truth = environment.state()
    movements = [s for s in steps if s["truth"]["execution"] == "moved"]
    mislocalized = 0
    for step in steps:
        if step["controller_output"]["command_id"]:
            estimate = step["controller_output"]["localization"]["estimate"]
            pose = step["truth"]["before"]
            mislocalized += estimate != {"node_id": pose["node_id"], "heading_deg": pose["heading_deg"]}
    return {"version": "temporal_navigation_v1", "state": session.state, "reason": session.reason,
            "start_prior": start, "heading_prior": heading, "goal": goal, "scenario": scenario,
            "fault_index": fault_index, "disabled_channels": list(disabled_channels),
            "poll_interval_s": poll_interval_s, "waiting_time_s": waiting_time_s,
            "controller": session.report(), "steps": steps,
            "audit": {"goal_verified": session.state == "reached" and truth["node_id"] == goal,
                      "false_arrival": session.state == "reached" and truth["node_id"] != goal,
                      "mislocalized_commands": mislocalized, "actual_moves": len(movements),
                      "stop_requests": sum(s["controller_output"]["action"] == "stop" for s in steps),
                      "final_truth": truth,
                      "visited_nodes": ([steps[0]["truth"]["before"]["node_id"]] if steps else []) +
                                       [s["truth"]["after"]["node_id"] for s in movements]},
            "scope": "Passive temporal recovery; known task prior; four directions; atomic moves only"}


def main(argv=None):
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=root / "configs/graph.json")
    parser.add_argument("--scene", type=Path, default=root / "configs/scene.json")
    parser.add_argument("--start")
    parser.add_argument("--goal")
    parser.add_argument("--heading", type=int)
    parser.add_argument("--disable", nargs="+", default=[])
    parser.add_argument("--scenario", choices=SCENARIOS, default="ideal")
    parser.add_argument("--poll-interval", type=float, default=0.01)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output and args.output.exists():
            raise ValueError("New output path required; never overwrite an old report")
        graph, scene = [json.loads(p.read_text()) for p in (args.map, args.scene)]
        result = run_navigation(graph, scene, args.start, args.heading, args.goal, args.disable,
                                scenario=args.scenario, poll_interval_s=args.poll_interval)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x") as file:
                json.dump(result, file, ensure_ascii=False, indent=2, allow_nan=False)
        print(json.dumps({key: result[key] for key in ("state", "reason", "audit")}, ensure_ascii=False))
    except (ValueError, OSError, KeyError) as error:
        print(json.dumps({"state": "invalid", "error": str(error)}, ensure_ascii=False))
        return 2
    return 0 if result["audit"]["goal_verified"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
