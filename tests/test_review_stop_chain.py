"""Gate/bridge fault-chain regressions; actual DDS/physics checked separately."""
import json

import pytest

from ros2_ws.src.toponav_bridge.toponav_bridge.protocol import (
    MotorGate, command_from_decision, decode, number, validate_command, validate_event,
)
from test_review_safety_regressions import bridge


SESSION = 'stop-chain'


def command(seq=1, **changes):
    decision = dict(control_frame_id='f1', captured_sim_s=1., decision_sim_s=1.,
                    v_m_s=.1, omega_rad_s=.1, state='moving')
    packet = command_from_decision(SESSION, seq, decision)
    packet.update(changes)
    return packet


def moving(pending=True):
    gate = MotorGate(SESSION)
    gate.arm('f1', 1., 10.)
    assert gate.accept(command(), 1., 10.1) == (.1, .1)
    if pending:
        gate.arm('f2', 1.1, 10.2)
    return gate


def assert_latched(gate, result):
    assert result == (0., 0.) and gate.wheels == (0., 0.) and gate.latched
    reason = gate.reason
    gate.arm('new', 1.2, 10.4)
    assert gate.accept(command(seq=2, frame_id='new', captured_sim_s=1.2, decision_sim_s=1.2),
                       1.2, 10.5) == (0., 0.)
    assert gate.reason == reason


@pytest.mark.parametrize('value', [10**400, -10**400])
def test_numeric_predicate_is_total_for_overflowing_wire_integers(value):
    assert number(value) is False


@pytest.mark.parametrize('field', ['captured_sim_s', 'decision_sim_s', 'v_m_s', 'omega_rad_s'])
@pytest.mark.parametrize('value', [10**400, -10**400])
def test_oversized_numeric_wire_command_latches_already_moving_gate(field, value):
    gate = moving()
    packet = command(seq=2, frame_id='f2', captured_sim_s=1.1, decision_sim_s=1.1)
    packet[field] = value
    packet = decode(json.dumps(packet, allow_nan=False))  # Valid JSON integer, not NaN syntax.
    result = gate.accept(packet, 1.1, 10.3)
    assert_latched(gate, result)
    assert gate.reason == 'invalid_command_envelope'


@pytest.mark.parametrize('field', ['captured_sim_s', 'decision_sim_s', 'v_m_s', 'omega_rad_s'])
def test_direct_numeric_command_validator_raises_value_error_not_overflow(field):
    with pytest.raises(ValueError):
        validate_command(command(**{field: 10**400}))


@pytest.mark.parametrize('value', [10**400, -10**400])
def test_oversized_event_time_is_a_value_error(value):
    event = dict(event_id=SESSION+':cmd-0001:visual-exit', command_id=SESSION+':cmd-0001',
                 status='visual_segment_complete', frame_id='f1', captured_sim_s=value)
    with pytest.raises(ValueError):
        validate_event(event, SESSION)


@pytest.mark.parametrize('operation', ['poll', 'arm', 'accept'])
@pytest.mark.parametrize('value', [10**400, -10**400])
def test_oversized_steady_clock_cannot_leave_held_velocity(operation, value):
    gate = moving(pending=False)
    if operation == 'poll':
        result = gate.poll(value)
    elif operation == 'arm':
        gate.arm('f2', 1.1, value)
        result = gate.wheels
    else:
        result = gate.accept(command(seq=2), 1.1, value)
    assert_latched(gate, result)


@pytest.mark.parametrize('pending', [False, True])
@pytest.mark.parametrize('value', [10**400, -10**400])
def test_oversized_simulation_clock_trips_without_throwing(pending, value):
    gate = moving(pending)
    assert_latched(gate, gate.accept(command(seq=2), value, 10.3))


@pytest.mark.parametrize('value', [10**400, -10**400])
def test_oversized_frame_capture_cannot_renew_held_velocity(value):
    gate = moving(pending=False)
    gate.arm('f2', value, 10.2)
    assert_latched(gate, gate.wheels)
    assert gate.reason == 'invalid_frame_lease'


@pytest.mark.parametrize('value', [10**400, -10**400])
def test_oversized_lease_configuration_is_value_error(value):
    with pytest.raises(ValueError):
        MotorGate(SESSION, lease_s=value)


@pytest.mark.parametrize('initial', ['moving_without_pending', 'disarmed'])
def test_explicit_abort_needs_no_new_camera_lease(initial):
    gate = moving(pending=False) if initial == 'moving_without_pending' else MotorGate(SESSION)
    packet = command(seq=2 if initial.startswith('moving') else 1, state='aborted', v_m_s=0., omega_rad_s=0.)
    assert_latched(gate, gate.accept(packet, 1.1, 10.2))
    assert gate.reason == 'remote_abort' and gate.seq == packet['seq']
    assert gate.hold_deadline == (10.9 if initial.startswith('moving') else None)


@pytest.mark.parametrize('change,reason', [
    ({'session_id': 'other'}, 'wrong_session'),
    ({'seq': 3}, 'replayed_or_out_of_order_command'),
    ({'decision_sim_s': 2.}, 'invalid_command_clock'),
    ({'v_m_s': .1}, 'invalid_command_envelope'),
])
def test_abort_exception_never_bypasses_session_sequence_clock_or_zero_checks(change, reason):
    gate = moving(pending=False)
    packet = command(seq=2, state='aborted', v_m_s=0., omega_rad_s=0.)
    packet.update(change)
    assert_latched(gate, gate.accept(packet, 1.1, 10.2))
    assert gate.reason == reason


@pytest.mark.parametrize('state', ['moving', 'observing', 'visual_stop', 'waiting_feedback'])
def test_only_abort_is_exempt_from_camera_lease(state):
    gate = moving(pending=False)
    packet = command(seq=2, state=state, v_m_s=.1 if state == 'moving' else 0., omega_rad_s=0.)
    assert_latched(gate, gate.accept(packet, 1.1, 10.2))
    assert gate.reason == 'command_without_frame_lease'


def test_expired_velocity_lease_wins_over_late_remote_abort():
    gate = moving(pending=False)
    packet = command(seq=2, state='aborted', v_m_s=0., omega_rad_s=0.)
    assert_latched(gate, gate.accept(packet, 1.1, 11.))
    assert gate.reason == 'command_lease_expired' and gate.seq == 1


@pytest.mark.parametrize('value', [0, 0., .15, -.3, 10**100])
def test_regular_finite_numeric_domain_is_not_narrowed(value):
    assert number(value) is True


@pytest.mark.parametrize('cause', ['camera', 'feedback'])
def test_real_navigation_watchdog_delivers_zero_to_active_gate(bridge, monkeypatch, cause):
    nav, ns = bridge
    from test_continuous_straight import route_hold
    route_hold(nav.controller)
    rgb = ns['np'].frombuffer(bytes(nav.queued.data), dtype=ns['np'].uint8).reshape(256, 256, 3)
    nav.controller.update(dict(frame_id=nav.queued.header.frame_id, captured_sim_s=1., rgb=rgb), 1.)
    assert nav.controller.state not in ('aborted', 'visual_stop')
    gate = MotorGate(nav.session)
    gate.arm('previous', .9, 10.)
    previous = dict(control_frame_id='previous', captured_sim_s=.9, decision_sim_s=.9,
                    v_m_s=.15, omega_rad_s=0., state='moving')
    assert gate.accept(command_from_decision(nav.session, 1, previous), .9, 10.1) == (.15, 0.)
    nav.queued = None
    nav.last_delivery = 10.
    if cause == 'feedback':
        nav.waiting, nav.wait_started = {'unconfirmed': True}, 10.
    monkeypatch.setattr(ns['time'], 'monotonic', lambda: 10.7)
    nav.ready = type('Publisher', (), {'publish': lambda self, msg: None})()
    nav.commands.publish.side_effect = lambda msg: gate.accept(decode(msg.data), 1., 10.7)
    nav.tick()
    assert gate.latched and gate.reason == 'remote_abort' and gate.wheels == (0., 0.)
    assert nav.controller.state == 'aborted' and nav.commands.publish.call_count == 1
    assert nav.bridge_reason == ('feedback_wall_timeout' if cause == 'feedback' else 'camera_or_clock_wall_timeout')


def queued_terminal_replay(tmp_path):
    """One already queued frame + three declared post-stop probes, no physics."""
    from test_ros2_bridge import replay_fixture
    import copy
    events = replay_fixture(tmp_path)
    extra = copy.deepcopy(events[1])
    extra['input'].update(frame_id='queued-before-stop', captured_sim_s=1.05)
    extra['now_sim_s'] = 1.05
    events.insert(1, extra)
    for i, e in enumerate(events):
        e['command']['seq'] = i + 1
    nav = tmp_path/'navigation'
    (nav/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
    result = json.loads((nav/'result.json').read_text())
    result.update(commands=5, images=5)
    (nav/'result.json').write_text(json.dumps(result))
    inventory = [dict(e['input'], terminal=i>1, suppressed=False, tick=i,
                      lease_deadline_steady_s=None) for i,e in enumerate(events)]
    (tmp_path/'public/frames.json').write_text(json.dumps(inventory))
    return events, inventory


def test_replay_requires_three_declared_stop_probes_not_three_arbitrary_terminal_images(tmp_path):
    from toponav_bridge.runner import replay_navigation
    queued_terminal_replay(tmp_path)
    result, rows = replay_navigation(tmp_path)
    assert result['terminal_frames'] == 4 and len(rows) == 5
    assert result['terminal_probe_frames'] == 3 and result['queued_terminal_frames'] == 1


@pytest.mark.parametrize('tamper', ['missing_probe', 'extra_probe', 'unknown_frame', 'pixel_binding',
                                  'capture_binding', 'duplicate_id', 'suppressed', 'false_terminal',
                                  'duplicate_delivery'])
def test_declared_probe_mapping_never_masks_missing_unbound_or_duplicate_terminal_evidence(tmp_path, tamper):
    from toponav_bridge.runner import replay_navigation
    events, inventory = queued_terminal_replay(tmp_path)
    if tamper == 'missing_probe':
        inventory.pop()
    elif tamper == 'extra_probe':
        inventory[1]['terminal'] = True
    elif tamper == 'unknown_frame':
        inventory[1]['frame_id'] = 'not-delivered'
    elif tamper == 'pixel_binding':
        inventory[2]['rgb_sha256'] = '0'*64
    elif tamper == 'capture_binding':
        inventory[2]['captured_sim_s'] += .01
    elif tamper == 'duplicate_id':
        inventory.append(dict(inventory[1]))
    elif tamper == 'suppressed':
        inventory[2]['suppressed'] = True
    elif tamper == 'false_terminal':
        events[2]['terminal'] = False
    else:
        events[3]['input'] = dict(events[2]['input'])
    (tmp_path/'public/frames.json').write_text(json.dumps(inventory))
    (tmp_path/'navigation/events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
    with pytest.raises(ValueError):
        replay_navigation(tmp_path)


@pytest.mark.parametrize('pending,accepted,expected', [(10.95, 10.1, 10.9), (10.9, 10.2, 10.9)])
def test_watchdog_audit_uses_earliest_pending_or_held_deadline(pending, accepted, expected):
    from toponav_bridge.runner import watchdog_deadline
    frames = [dict(terminal=False, lease_deadline_steady_s=pending)]
    records = [dict(already_latched=False, reason='accepted', steady_s=accepted)]
    result = watchdog_deadline(frames, records, .8)
    assert result['effective_deadline_s'] == pytest.approx(expected)
    assert result['pending_frame_deadline_s'] == pending
    assert result['held_velocity_deadline_s'] == pytest.approx(accepted+.8)


def test_rejected_commands_cannot_extend_audited_watchdog_deadline():
    from toponav_bridge.runner import watchdog_deadline
    frames = [dict(terminal=False, lease_deadline_steady_s=10.95)]
    records = [dict(already_latched=False, reason='accepted', steady_s=10.1),
               dict(already_latched=True, reason='command_lease_expired', steady_s=11.)]
    assert watchdog_deadline(frames, records, .8)['effective_deadline_s'] == pytest.approx(10.9)


def test_watchdog_audit_does_not_silently_increase_frozen_lease():
    from toponav_bridge.runner import watchdog_deadline
    with pytest.raises(ValueError):
        watchdog_deadline([dict(terminal=False, lease_deadline_steady_s=11.)],
                          [dict(already_latched=False, reason='accepted', steady_s=10.)], 1.)
