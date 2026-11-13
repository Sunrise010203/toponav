"""Truth-free, single-flight temporal coordinator for atomic topological moves.

Frames and feedback have outer event metadata; legacy strict payloads are unchanged.
No image, environment, scene, physical pose or arrival audit enters this module.
"""
import copy
import math
import time
import uuid
from dataclasses import asdict, dataclass

from simulation.dijkstra import plan
from simulation.structural_localization import MIN_CONFIDENCE, TURNS, decide, update_candidates
from simulation.topology import cardinal_heading, task_values


UNKNOWN = {"relative_open": dict.fromkeys(TURNS), "confidence": 0.0}
TERMINAL = {"reached", "failed"}


def _finite_number(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


@dataclass(frozen=True)
class SessionConfig:
    confirmation_frames: int = 3
    max_frame_age_s: float = 1.0
    stage_timeout_s: float = 5.0
    session_timeout_s: float = 120.0
    max_conflicts: int = 2
    max_execution_failures: int = 1
    max_commands: int = 32
    max_events: int = 1000

    def __post_init__(self):
        for name in ("confirmation_frames", "max_conflicts", "max_execution_failures",
                     "max_commands", "max_events"):
            value = getattr(self, name)
            minimum = 2 if name == "confirmation_frames" else 1 if name == "max_events" else 0
            if type(value) is not int or value < minimum:
                raise ValueError(f"Invalid {name}")
        for name in ("max_frame_age_s", "stage_timeout_s", "session_timeout_s"):
            value = getattr(self, name)
            if not _finite_number(value) or value <= 0:
                raise ValueError(f"Invalid {name}")


class NavigationSession:
    def __init__(self, graph, start=None, heading=None, goal=None, candidates=None,
                 disabled_channels=(), config=None, clock=time.monotonic, wall_ns=time.time_ns,
                 session_id=None):
        self.session_id = uuid.uuid4().hex if session_id is None else session_id
        self._identity(self.session_id)
        if len(self.session_id) > 200:
            raise ValueError("Session identity is too long")
        self.config = config or SessionConfig()
        if not isinstance(self.config, SessionConfig):
            raise ValueError("Expected SessionConfig")
        self.graph = copy.deepcopy(graph)
        self.start, self.goal = task_values(self.graph, start, goal)
        heading = self.graph.get("task", {}).get("initial_heading_deg") if heading is None else heading
        self.heading = cardinal_heading(heading)
        self.disabled_channels = tuple(disabled_channels)
        plan(self.graph, self.start, self.goal, self.disabled_channels)
        prior = ([{"node_id": self.start, "heading_deg": self.heading}]
                 if candidates is None else copy.deepcopy(candidates))
        if not prior:
            raise ValueError("At least one task-given candidate is required")
        self.candidates = update_candidates(self.graph, prior, UNKNOWN,
                                            disabled_channels=self.disabled_channels)["candidates"]
        self.clock, self.wall_ns = clock, wall_ns
        self.started = self.stage_started = self.last_mono = self._finite_clock(clock())
        # The initial task prior has not moved: capture at session creation is valid.
        # Later movement/completion barriers require strictly newer captures.
        self.last_wall_ns = self._timestamp(wall_ns())
        self.epoch_wall_ns = self.last_wall_ns - 1
        self.state, self.reason = "observing", "awaiting_fresh_evidence"
        self.pending, self.localization = None, None
        self.window, self.seen_frames, self.feedback_events, self.commands = [], set(), {}, {}
        self.last_capture_ns = -1
        self.recovery_started = None
        self.history = []
        self.metrics = {key: 0 for key in (
            "events", "frames_received", "valid_frames", "duplicate_frames", "rejected_frames",
            "commands_sent", "transitions_applied", "duplicate_feedback", "execution_failures",
            "conflicts", "stop_requests", "recoveries_started", "recoveries_succeeded",
            "recoveries_failed")}
        self.metrics["recovery_durations_s"] = []

    @staticmethod
    def _finite_clock(value):
        if not _finite_number(value):
            raise ValueError("Clock must return a finite number")
        return value

    @staticmethod
    def _timestamp(value):
        if type(value) is not int or value < 0 or not _finite_number(value):
            raise ValueError("Timestamp must be nonnegative Unix nanoseconds")
        return value

    @staticmethod
    def _identity(value):
        if not isinstance(value, str) or not value or len(value) > 256:
            raise ValueError("Event identities must be nonempty strings of at most 256 characters")

    def _fail(self, reason):
        self.state, self.reason = "failed", reason
        self.window.clear()
        if self.recovery_started is not None:
            self.metrics["recoveries_failed"] += 1
            self.recovery_started = None

    def _guard(self):
        if self.state in TERMINAL:
            return False
        try:
            now = self._finite_clock(self.clock())
        except Exception:
            self._fail("monotonic_clock_error")
            raise
        if now < self.last_mono:
            self._fail("monotonic_clock_error")
            return False
        elapsed, stage_elapsed = now - self.started, now - self.stage_started
        if not _finite_number(elapsed) or not _finite_number(stage_elapsed):
            self._fail("monotonic_clock_error")
            return False
        self.last_mono = now
        if elapsed >= self.config.session_timeout_s:
            self._fail("session_timeout")
        elif stage_elapsed >= self.config.stage_timeout_s:
            self._fail("stage_timeout")
        elif self.metrics["events"] >= self.config.max_events:
            self._fail("event_limit")
        return self.state not in TERMINAL

    def _wall_clock(self):
        try:
            now = self._timestamp(self.wall_ns())
            if now < self.last_wall_ns:
                raise ValueError("Wall clock regressed; fresh capture barrier is not trustworthy")
        except Exception:
            self._fail("wall_clock_error")
            raise
        self.last_wall_ns = now
        return now

    def _event(self, kind, handler, payload):
        # Terminal and timeout protection precedes any access to the payload.
        if not self._guard():
            return self._record(kind + "_ignored")
        try:
            return handler(payload)
        except (ValueError, TypeError, OverflowError) as error:
            if self.state not in TERMINAL:
                self._fail("invalid_" + kind)
            if not isinstance(error, ValueError):
                raise ValueError("Invalid " + kind + " event") from error
            raise
        except Exception:
            if self.state not in TERMINAL:
                self._fail(kind + "_error")
            raise

    def _recover(self, reason, clear=True):
        if clear:
            self.window.clear()
        if self.recovery_started is None:
            self.recovery_started = self.last_mono
            self.metrics["recoveries_started"] += 1
        self.state = "waiting_execution" if self.pending else "recovering"
        self.reason = reason

    def _recovered(self):
        if self.recovery_started is not None:
            self.metrics["recoveries_succeeded"] += 1
            self.metrics["recovery_durations_s"].append(self.last_mono - self.recovery_started)
            self.recovery_started = None

    def _record(self, kind, action="stop", command_id=None, **details):
        if action == "stop" and self.state not in TERMINAL:
            self.metrics["stop_requests"] += 1
        snapshot = {"state": self.state, "reason": self.reason, "action": action,
                    "command_id": command_id, "candidates": copy.deepcopy(self.candidates),
                    "localization": copy.deepcopy(self.localization), "window_size": len(self.window),
                    "pending_command_id": self.pending["command_id"] if self.pending else None}
        self.history.append({"event": kind, "monotonic_s": self.last_mono,
                             **copy.deepcopy(details), **copy.deepcopy(snapshot)})
        return snapshot

    def tick(self):
        """Call even with no camera frames or feedback; never repeats a move."""
        if self._guard():
            self.metrics["events"] += 1
            self._recover("no_new_observation", clear=False)
        return self._record("tick")

    def abort(self, reason="cancelled"):
        """Explicit driver cancellation; cannot turn a completed task into motion."""
        if self.state not in TERMINAL:
            self._fail(reason)
        return self._record("abort")

    def observe(self, frame):
        """Process one frame, latching failure before propagating input errors."""
        return self._event("frame", self._observe, frame)

    def _observe(self, frame):
        if not isinstance(frame, dict) or set(frame) != {"frame_id", "captured_at_ns", "observation"}:
            raise ValueError("Frame envelope requires only frame_id/captured_at_ns/observation")
        identity, captured, observation = frame["frame_id"], frame["captured_at_ns"], frame["observation"]
        self._identity(identity)
        self._timestamp(captured)
        # Validate strict observation even when it cannot count as evidence.
        probe = update_candidates(self.graph, self.candidates, observation,
                                  disabled_channels=self.disabled_channels)
        self.metrics["events"] += 1
        self.metrics["frames_received"] += 1
        now_wall = self._wall_clock()
        rejection = None
        if identity in self.seen_frames:
            self.metrics["duplicate_frames"] += 1
            rejection = "duplicate_frame"
        self.seen_frames.add(identity)
        if rejection is None:
            if captured > now_wall:
                rejection = "future_frame"
            elif (now_wall - captured) / 1e9 > self.config.max_frame_age_s:
                rejection = "stale_frame"
            elif captured <= self.epoch_wall_ns:
                rejection = "previous_epoch_frame"
            elif captured <= self.last_capture_ns:
                rejection = "out_of_order_frame"
        if rejection:
            self.metrics["rejected_frames"] += 1
            self._recover(rejection)
            return self._record("frame_rejected", frame_id=identity, captured_at_ns=captured,
                                observation=observation, accepted=False)
        self.last_capture_ns = captured
        self.metrics["valid_frames"] += 1
        if self.pending:
            self.reason = "execution_unconfirmed"
            return self._record("frame_waiting_execution", frame_id=identity,
                                captured_at_ns=captured, observation=observation, accepted=False)
        if (observation["confidence"] < MIN_CONFIDENCE or
                any(v is None for v in observation["relative_open"].values())):
            self._recover("observation_uncertain")
            return self._record("frame_uncertain", frame_id=identity, captured_at_ns=captured,
                                observation=observation, accepted=False)
        signature = tuple(observation["relative_open"][d] for d in TURNS)
        if self.window and self.window[-1]["signature"] != signature:
            self._recover("inconsistent_structure")
        self.window.append({"signature": signature, "confidence": observation["confidence"],
                            "frame_id": identity})
        if probe["status"] in {"observation_mismatch", "ambiguous"}:
            self._recover(probe["status"], clear=False)
        else:
            self.state = "recovering" if self.recovery_started is not None else "confirming"
            self.reason = "confirming_structure"
        if len(self.window) < self.config.confirmation_frames:
            return self._record("frame_confirming", frame_id=identity, captured_at_ns=captured,
                                observation=observation, accepted=True)

        # Unanimous window; min confidence preserves the legacy threshold semantics.
        confirmed = {"relative_open": dict(observation["relative_open"]),
                     "confidence": min(f["confidence"] for f in self.window)}
        self.localization = update_candidates(self.graph, self.candidates, confirmed,
                                              disabled_channels=self.disabled_channels)
        confirmed_ids = [f["frame_id"] for f in self.window]
        self.window.clear()
        if self.localization["status"] != "localized":
            reason = self.localization["status"]
            if reason == "observation_mismatch":
                self.metrics["conflicts"] += 1
            self._recover(reason)
            if self.metrics["conflicts"] > self.config.max_conflicts:
                self._fail("conflict_limit")
            return self._record("confirmation_rejected", frame_ids=confirmed_ids,
                                observation=confirmed, accepted=False)

        self.candidates = copy.deepcopy(self.localization["candidates"])
        decision = decide(self.graph, self.localization, self.goal, self.disabled_channels)
        if decision["status"] == "reached":
            self._recovered()
            self.state, self.reason = "reached", "goal_confirmed"
            return self._record("goal_confirmed", frame_ids=confirmed_ids, decision=decision)
        if decision["action"] == "stop":
            self._fail(decision["status"])
            return self._record("planning_stopped", frame_ids=confirmed_ids, decision=decision)
        if self.metrics["commands_sent"] >= self.config.max_commands:
            self._fail("command_limit")
            return self._record("command_limit", frame_ids=confirmed_ids)
        self._recovered()
        self.metrics["commands_sent"] += 1
        command_id = f"{self.session_id}:cmd-{self.metrics['commands_sent']:04d}"
        command = {"command_id": command_id, "action": decision["action"],
                   "prior": copy.deepcopy(self.candidates), "completion": None}
        self.commands[command_id] = command
        self.pending = command
        # An unconfirmed command means stay OR complete, predicted once from its prior.
        self.candidates = update_candidates(self.graph, command["prior"], UNKNOWN, command["action"],
                                            disabled_channels=self.disabled_channels)["predicted_candidates"]
        self.state, self.reason = "waiting_execution", "command_issued"
        self.epoch_wall_ns = now_wall
        self.stage_started = self.last_mono
        return self._record("command_issued", action=decision["action"], command_id=command_id,
                            frame_ids=confirmed_ids, decision=decision)

    def feedback(self, event):
        """Consume completion only after its transition and clock are validated."""
        return self._event("feedback", self._feedback, event)

    def _feedback(self, event):
        if not isinstance(event, dict) or set(event) != {"event_id", "command_id", "feedback"}:
            raise ValueError("Feedback envelope requires only event_id/command_id/feedback")
        identity, command_id, payload = event["event_id"], event["command_id"], event["feedback"]
        self._identity(identity)
        self._identity(command_id)
        if (not isinstance(payload, dict) or set(payload) != {"status"} or
                type(payload["status"]) is not str or
                payload["status"] not in {"succeeded", "failed", "unknown", "sent"}):
            raise ValueError("Motion feedback requires only status succeeded/failed/unknown/sent")
        self.metrics["events"] += 1
        if identity in self.feedback_events:
            if self.feedback_events[identity] != event:
                self._fail("feedback_protocol_error")
            else:
                self.metrics["duplicate_feedback"] += 1
            return self._record("feedback_replay", feedback_event=event, consumed=False)
        if command_id not in self.commands:
            self._fail("feedback_protocol_error")
            return self._record("unknown_command", feedback_event=event, consumed=False)
        command, status = self.commands[command_id], payload["status"]
        if command["completion"] is not None:
            self.feedback_events[identity] = copy.deepcopy(event)
            if status in {"succeeded", "failed"} and status != command["completion"]:
                self._fail("feedback_protocol_error")
            else:
                self.metrics["duplicate_feedback"] += 1
            return self._record("completed_command_replay", feedback_event=event, consumed=False)
        if self.pending is not command:
            self._fail("feedback_protocol_error")
            return self._record("feedback_mismatch", feedback_event=event, consumed=False)
        if status in {"sent", "unknown"}:
            self.feedback_events[identity] = copy.deepcopy(event)
            self._recover("execution_unconfirmed")
            return self._record("execution_unconfirmed", feedback_event=event, consumed=False)

        result = update_candidates(self.graph, command["prior"], UNKNOWN, command["action"], payload,
                                   self.disabled_channels)
        completion_wall_ns = self._wall_clock()
        # Commit only after both computations succeed. A broken clock must not
        # consume a completion, clear pending, or lower the capture barrier.
        self.feedback_events[identity] = copy.deepcopy(event)
        command["completion"] = status
        self.pending = None
        self.candidates = result["predicted_candidates"]
        self.localization = None  # Completion is not a fresh structural observation.
        self.window.clear()
        self.epoch_wall_ns = completion_wall_ns
        self.stage_started = self.last_mono
        if not self.candidates:
            self._fail("transition_mismatch")
        elif status == "succeeded":
            self.metrics["transitions_applied"] += 1
            self.state, self.reason = "observing", "completion_consumed"
        else:
            self.metrics["execution_failures"] += 1
            self._recover("execution_failed")
            if self.metrics["execution_failures"] > self.config.max_execution_failures:
                self._fail("execution_failure_limit")
        return self._record("completion_consumed", feedback_event=event, consumed=True)

    def report(self):
        """Controller-only report. Physical audit belongs to the caller, never here."""
        return copy.deepcopy({"session_id": self.session_id, "state": self.state,
                              "reason": self.reason, "candidates": self.candidates,
                              "config": asdict(self.config), "metrics": self.metrics,
                              "history": self.history})
