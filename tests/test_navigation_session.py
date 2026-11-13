"""Deterministic clocks and public map hypotheses, never simulator truth."""
import copy
import json
from pathlib import Path

import pytest

from simulation.navigation_session import NavigationSession, SessionConfig
from simulation.structural_localization import map_exits

ROOT = Path(__file__).resolve().parents[1]


class Clock:
    def __init__(self):
        self.mono, self.wall = 0.0, 1_000_000_000_000

    def advance(self, seconds=0.1):
        self.mono += seconds
        self.wall += int(seconds * 1e9)


def graph():
    return json.loads((ROOT / "configs/graph.json").read_text())


def obs(node="N01", heading=0, confidence=1.0):
    exits = map_exits(graph(), {"node_id": node, "heading_deg": heading})
    return {"relative_open": {d: d in exits for d in ("front", "left", "right", "back")},
            "confidence": confidence}


def setup(**kwargs):
    clock = Clock()
    session = NavigationSession(graph(), clock=lambda: clock.mono, wall_ns=lambda: clock.wall, **kwargs)
    return session, clock


def frame(session, clock, observation=None, identity=None, captured=None):
    clock.advance()
    return session.observe({"frame_id": identity or f"f-{clock.mono}",
                            "captured_at_ns": clock.wall if captured is None else captured,
                            "observation": observation if observation is not None else obs()})


def confirm(session, clock, observation=None):
    return [frame(session, clock, observation) for _ in range(3)][-1]


def feedback(session, command, status="succeeded", event="e1"):
    return session.feedback({"event_id": event, "command_id": command,
                             "feedback": {"status": status}})


def test_three_new_frames_are_required_and_command_is_emitted_only_once():
    s, c = setup()
    assert frame(s, c)["action"] == "stop"
    assert frame(s, c)["state"] == "confirming"
    issued = frame(s, c)
    assert issued["action"] == "straight" and issued["command_id"]
    assert issued["state"] == "waiting_execution"
    for _ in range(8):
        assert frame(s, c)["action"] == "stop"
    assert s.metrics["commands_sent"] == 1


def test_low_confidence_stops_then_fresh_consistent_evidence_recovers():
    s, c = setup()
    assert frame(s, c, obs(confidence=0.2))["state"] == "recovering"
    assert confirm(s, c)["action"] == "straight"
    assert s.metrics["recoveries_succeeded"] == 1
    assert s.metrics["recovery_durations_s"][0] > 0


def test_same_frame_replay_does_not_accumulate_votes():
    s, c = setup()
    frame(s, c, identity="same")
    for _ in range(12):
        r = frame(s, c, identity="same")
        assert r["action"] == "stop"
    assert s.metrics["commands_sent"] == 0
    assert s.metrics["duplicate_frames"] == 12
    # Replays break the consecutive evidence window rather than confirm it.
    assert r["window_size"] == 0


@pytest.mark.parametrize("kind", ["stale", "future", "out_of_order", "same_time"])
def test_bad_frame_times_cannot_confirm(kind):
    s, c = setup()
    frame(s, c)
    last = c.wall
    captured = {"stale": last - 2_000_000_000, "future": last + 3_000_000_000,
                "out_of_order": last - 1, "same_time": last}[kind]
    r = frame(s, c, captured=captured)
    assert r["action"] == "stop" and r["window_size"] == 0
    assert r["reason"] in {"stale_frame", "future_frame", "out_of_order_frame"}


def test_missing_frames_timeout_on_monotonic_clock_even_if_wall_clock_rolls_back():
    s, c = setup()
    c.mono = 6
    c.wall -= 99_000_000_000
    r = s.tick()
    assert r["state"] == "failed" and r["reason"] == "stage_timeout"
    assert s.metrics["commands_sent"] == 0


def test_succeeded_feedback_advances_once_even_with_new_event_ids():
    s, c = setup()
    cmd = confirm(s, c)["command_id"]
    feedback(s, cmd)
    assert s.candidates == [{"node_id": "N02", "heading_deg": 0}]
    feedback(s, cmd)  # Exact event replay.
    feedback(s, cmd, event="another-success")
    assert s.candidates == [{"node_id": "N02", "heading_deg": 0}]
    frame(s, c, obs("N02"))
    frame(s, c, obs("N02"))
    assert s.metrics["transitions_applied"] == 1
    assert s.metrics["duplicate_feedback"] == 2


@pytest.mark.parametrize("status", ["sent", "unknown"])
def test_unconfirmed_execution_cannot_be_unlocked_by_many_correct_images(status):
    s, c = setup()
    cmd = confirm(s, c)["command_id"]
    feedback(s, cmd, status)
    assert {p["node_id"] for p in s.candidates} == {"N01", "N02"}
    for _ in range(7):
        r = frame(s, c, obs("N02"))
        assert r["action"] == "stop" and r["state"] == "waiting_execution"
    assert s.metrics["transitions_applied"] == 0
    feedback(s, cmd, "succeeded", event="late-success")
    assert s.candidates == [{"node_id": "N02", "heading_deg": 0}]
    assert s.metrics["transitions_applied"] == 1
    assert confirm(s, c, obs("N02"))["action"] == "straight"


def test_no_feedback_preserves_stay_move_and_times_out():
    s, c = setup()
    confirm(s, c)
    assert {p["node_id"] for p in s.candidates} == {"N01", "N02"}
    c.advance(6)
    assert s.tick()["reason"] == "stage_timeout"


def test_failed_execution_does_not_move_and_retry_is_bounded():
    s, c = setup()
    cmd = confirm(s, c)["command_id"]
    feedback(s, cmd, "failed")
    assert s.candidates == [{"node_id": "N01", "heading_deg": 0}]
    assert s.state == "recovering"
    cmd2 = confirm(s, c)["command_id"]
    assert cmd2 != cmd
    r = feedback(s, cmd2, "failed", "e2")
    assert r["state"] == "failed" and r["reason"] == "execution_failure_limit"
    assert s.metrics["transitions_applied"] == 0


@pytest.mark.parametrize("kind", ["unknown_command", "changed_event", "conflicting_completion"])
def test_feedback_protocol_errors_stop(kind):
    s, c = setup()
    cmd = confirm(s, c)["command_id"]
    if kind == "unknown_command":
        r = feedback(s, "not-issued")
    elif kind == "changed_event":
        feedback(s, cmd, "sent")
        r = feedback(s, cmd, "succeeded")
    else:
        feedback(s, cmd)
        r = feedback(s, cmd, "failed", "different-event")
    assert r["state"] == "failed" and r["reason"] == "feedback_protocol_error"
    assert r["action"] == "stop"


def test_old_completion_cannot_advance_the_next_pending_command():
    s, c = setup()
    first = confirm(s, c)["command_id"]
    feedback(s, first)
    second = confirm(s, c, obs("N02"))["command_id"]
    assert second != first
    before = copy.deepcopy(s.candidates)
    feedback(s, first, event="late-old-success")
    assert s.candidates == before and s.pending["command_id"] == second


def test_pre_completion_image_cannot_be_used_after_turn_or_traversal():
    s, c = setup()
    cmd = confirm(s, c)["command_id"]
    captured = c.wall
    c.advance(0.2)
    feedback(s, cmd)
    r = frame(s, c, obs("N02"), captured=captured)
    assert r["action"] == "stop" and r["reason"] == "previous_epoch_frame"
    assert r["window_size"] == 0


def test_short_wrong_frame_does_not_erase_history_and_can_recover():
    s, c = setup()
    frame(s, c, obs("N02"))  # Incompatible, but not enough evidence to commit it.
    assert s.candidates == [{"node_id": "N01", "heading_deg": 0}]
    assert confirm(s, c)["action"] == "straight"
    assert s.metrics["recoveries_succeeded"] == 1


def test_confirmed_mismatch_reobserves_without_all_map_reset():
    s, c = setup()
    r = confirm(s, c, obs("N02"))
    assert r["state"] == "recovering" and r["reason"] == "observation_mismatch"
    assert s.candidates == [{"node_id": "N01", "heading_deg": 0}]
    assert confirm(s, c)["action"] == "straight"


def test_persistent_mismatch_exits_after_fixed_number_of_windows():
    s, c = setup()
    confirm(s, c, obs("N02"))
    confirm(s, c, obs("N02"))
    r = confirm(s, c, obs("N02"))
    assert r["state"] == "failed" and r["reason"] == "conflict_limit"


def test_symmetric_candidates_remain_stopped_until_timeout():
    s, c = setup(candidates=[{"node_id": "N02", "heading_deg": 0},
                             {"node_id": "N04", "heading_deg": 180}])
    assert confirm(s, c, obs("N02"))["reason"] == "ambiguous"
    assert s.metrics["commands_sent"] == 0 and len(s.candidates) == 2
    c.advance(6)
    assert s.tick()["state"] == "failed"


def test_partial_frame_breaks_confirmation_without_treating_unknown_as_closed():
    s, c = setup()
    frame(s, c)
    partial = obs()
    partial["relative_open"]["back"] = None
    assert frame(s, c, partial)["window_size"] == 0
    assert s.candidates == [{"node_id": "N01", "heading_deg": 0}]
    assert frame(s, c)["action"] == "stop"


def test_goal_also_requires_fresh_confirmation_and_terminal_state_never_moves():
    s, c = setup(start="N01", goal="N01", heading=0)
    assert frame(s, c)["action"] == "stop"
    assert confirm(s, c)["state"] == "reached"
    assert s.metrics["commands_sent"] == 0
    c.advance(200)
    assert s.tick()["state"] == "reached"
    assert frame(s, c)["action"] == "stop"


def test_public_planner_is_reused_and_inputs_not_modified(monkeypatch):
    from simulation import path_planning
    calls = []
    original = path_planning.plan_from_estimate
    monkeypatch.setattr(path_planning, "plan_from_estimate", lambda *a, **k: (calls.append(1), original(*a, **k))[1])
    g = graph()
    before = copy.deepcopy(g)
    c = Clock()
    s = NavigationSession(g, clock=lambda: c.mono, wall_ns=lambda: c.wall)
    confirm(s, c)
    assert calls and g == before


@pytest.mark.parametrize("field,value", [("confirmation_frames", 1), ("stage_timeout_s", 0),
                                         ("session_timeout_s", float("nan")), ("max_commands", -1),
                                         ("max_events", True)])
def test_invalid_policy_is_rejected(field, value):
    with pytest.raises(ValueError):
        setup(config=SessionConfig(**{field: value}))


def test_command_event_and_task_limits_are_enforced():
    s, c = setup(config=SessionConfig(max_commands=0))
    assert confirm(s, c)["reason"] == "command_limit"
    s, c = setup(config=SessionConfig(max_events=2))
    frame(s, c)
    frame(s, c)
    assert frame(s, c)["reason"] == "event_limit"
    s, c = setup(config=SessionConfig(session_timeout_s=1, stage_timeout_s=5))
    c.advance(1.1)
    assert s.tick()["reason"] == "session_timeout"


def test_truth_fields_are_rejected_and_snapshot_cannot_mutate_session():
    s, c = setup()
    with pytest.raises(ValueError):
        s.observe({"frame_id": "x", "captured_at_ns": c.wall, "observation": obs(), "true_node": "N01"})
    assert s.state == "failed"
    # Each active session rejects truth fields. A failed session deliberately
    # ignores later payloads without parsing them, so use a new active session.
    s, c = setup()
    bad = obs()
    bad["node_id"] = "N01"
    with pytest.raises(ValueError):
        frame(s, c, bad)
    assert s.state == "failed"
    result = s.tick()
    result["candidates"].clear()
    assert s.candidates


def test_monotonic_clock_regression_causes_safe_failure():
    s, c = setup()
    frame(s, c)
    c.mono = -1
    assert s.tick()["reason"] == "monotonic_clock_error"


def test_late_feedback_from_previous_session_is_not_a_current_command():
    first, c1 = setup()
    old = confirm(first, c1)["command_id"]
    second, c2 = setup()
    current = confirm(second, c2)["command_id"]
    assert old != current
    r = feedback(second, old)
    assert r["state"] == "failed" and r["reason"] == "feedback_protocol_error"


def test_timestamp_at_completion_barrier_is_not_provably_a_post_completion_frame():
    s, c = setup()
    cmd = confirm(s, c)["command_id"]
    feedback(s, cmd)
    barrier = c.wall
    r = frame(s, c, obs("N02"), captured=barrier)
    assert r["reason"] == "previous_epoch_frame" and r["window_size"] == 0


def test_missing_observation_tick_records_recovery_without_creating_a_vote():
    s, c = setup()
    frame(s, c)
    c.advance()
    r = s.tick()
    assert r["state"] == "recovering" and r["window_size"] == 1
    assert s.metrics["recoveries_started"] == 1 and r["action"] == "stop"
    assert frame(s, c)["action"] == "stop"
    assert frame(s, c)["action"] == "straight"


def test_repeated_nonterminal_feedback_never_extends_execution_deadline_or_propagates_again():
    s, c = setup(config=SessionConfig(stage_timeout_s=1))
    cmd = confirm(s, c)["command_id"]
    for i in range(3):
        c.advance(0.2)
        feedback(s, cmd, "sent" if i % 2 else "unknown", f"event-{i}")
        assert {p["node_id"] for p in s.candidates} == {"N01", "N02"}
    c.advance(0.5)
    r = feedback(s, cmd, "succeeded", "too-late")
    assert r["reason"] == "stage_timeout" and s.metrics["transitions_applied"] == 0


@pytest.mark.parametrize("extra", ["true_node", "true_heading", "position_m"])
def test_truth_bearing_feedback_is_rejected(extra):
    s, c = setup()
    cmd = confirm(s, c)["command_id"]
    with pytest.raises(ValueError):
        s.feedback({"event_id": "e", "command_id": cmd,
                    "feedback": {"status": "succeeded", extra: 0}})
    assert s.metrics["transitions_applied"] == 0
