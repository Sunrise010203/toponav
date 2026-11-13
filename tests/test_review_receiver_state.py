"""Receiver audit consistency, not status-source authentication or braking."""
import json
import threading
from unittest.mock import Mock

import pytest

from simulation.gazebo_receiver import ReceiverEvidence


def accepted(**changes):
    item = dict(version=1, session_id='test', wall_s=10., sim_s=.1, latched=False,
                reason='accepted', command_seq=1, deadline_steady_s=10.8, wheels=[.1, 0.])
    item.update(changes)
    return item


def disarmed(**changes):
    item = accepted(reason='disarmed', command_seq=0, deadline_steady_s=0., wheels=[0., 0.])
    item.update(changes)
    return item


def stopped(**changes):
    item = accepted(reason='remote_stop', latched=True, wheels=[0., 0.])
    item.update(changes)
    return item


def push(evidence, item):
    evidence.push(json.dumps(item))


INVALID = [
    disarmed(command_seq=1, deadline_steady_s=10.8),
    disarmed(wheels=[.1, 0.]),
    disarmed(wheels=[0., .1]),
    disarmed(deadline_steady_s=10.8),
    accepted(command_seq=0),
    accepted(command_seq=0, deadline_steady_s=0.),
    accepted(deadline_steady_s=0.),
    accepted(deadline_steady_s=10.),
    accepted(deadline_steady_s=9.9),
    accepted(deadline_steady_s=10.800000001),
    accepted(command_seq=2**64),
    stopped(deadline_steady_s=0.),
    stopped(command_seq=0),
    stopped(reason='invalid_receiver_envelope', command_seq=0),
    stopped(reason='steady_clock_regression', deadline_steady_s=0.),
    stopped(reason='command_lease_expired'),
    stopped(reason='command_lease_expired', command_seq=0, deadline_steady_s=0.),
]


@pytest.mark.parametrize('item', INVALID)
def test_impossible_authority_states_are_sticky_audit_faults(item):
    evidence = ReceiverEvidence('test')
    push(evidence, item)
    assert evidence.error and 'authority state' in evidence.error
    assert not evidence.events
    with pytest.raises(RuntimeError, match='Receiver interlock'):
        evidence.require_ready()
    # Later valid stop evidence cannot erase an earlier malformed authority.
    push(evidence, stopped(wall_s=11., sim_s=.2))
    summary = evidence.summary(.1, .2)
    assert summary['fault_latched'] and summary['final_zero'] and summary['final_step_fresh']
    assert 'stationary' not in summary


VALID = [
    disarmed(),
    accepted(),
    accepted(wheels=[0., 0.]),
    accepted(command_seq=2**64-1),
    accepted(deadline_steady_s=10.000000001),
    stopped(),
    stopped(wall_s=11.),
    stopped(command_seq=0, deadline_steady_s=0.),
    stopped(reason='invalid_receiver_envelope', command_seq=0, deadline_steady_s=0.),
    stopped(reason='invalid_receiver_envelope'),
    stopped(reason='command_lease_expired', wall_s=10.8),
    stopped(reason='command_lease_expired', wall_s=11.),
    stopped(reason='steady_clock_regression', wall_s=0.),
]


@pytest.mark.parametrize('item', VALID)
def test_valid_startup_commands_and_fault_states_are_retained(item):
    evidence = ReceiverEvidence('test')
    push(evidence, item)
    assert evidence.error is None and evidence.events == [item]
    if item['latched']:
        with pytest.raises(RuntimeError):
            evidence.require_ready()
    else:
        evidence.require_ready()


@pytest.mark.parametrize('changes', [dict(deadline_steady_s=10.9), dict(deadline_steady_s=10.7),
                                   dict(wheels=[.15, 0.]), dict(wheels=[.1, .1])])
def test_same_sequence_cannot_change_deadline_or_authorized_velocity(changes):
    evidence = ReceiverEvidence('test')
    push(evidence, accepted())
    push(evidence, accepted(wall_s=10.1, sim_s=.2, **changes))
    assert evidence.error and len(evidence.events) == 1
    push(evidence, stopped(wall_s=10.2, sim_s=.3))
    assert evidence.summary(.2, .3)['fault_latched']


@pytest.mark.parametrize('changes', [dict(command_seq=2), dict(deadline_steady_s=10.9),
                                   dict(deadline_steady_s=10.7)])
def test_latched_receiver_cannot_change_sequence_or_deadline(changes):
    evidence = ReceiverEvidence('test')
    push(evidence, stopped())
    push(evidence, stopped(wall_s=11., sim_s=.2, **changes))
    assert evidence.error and len(evidence.events) == 1
    assert evidence.summary(0., .2)['fault_latched']


def test_repeated_same_command_and_later_stop_are_valid_not_renewals():
    evidence = ReceiverEvidence('test')
    for item in [disarmed(wall_s=9., sim_s=0.), accepted(),
                 accepted(wall_s=10.1, sim_s=.2), stopped(wall_s=10.2, sim_s=.3),
                 stopped(wall_s=11., sim_s=.4)]:
        push(evidence, item)
    assert evidence.error is None and len(evidence.events) == 5
    summary = evidence.summary(.3, .4)
    assert not summary['fault_latched'] and summary['final_zero'] and summary['final_step_fresh']
    assert summary['final']['deadline_steady_s'] == 10.8


@pytest.mark.parametrize('deadline', [10.7, 10.9])
def test_new_sequences_can_change_targets_and_skip_unreceived_statuses(deadline):
    evidence = ReceiverEvidence('test')
    push(evidence, accepted())
    push(evidence, accepted(wall_s=10.1, sim_s=.2, command_seq=4,
                           deadline_steady_s=deadline, wheels=[.15, -.3]))
    assert evidence.error is None and len(evidence.events) == 2
    evidence.require_ready()


@pytest.mark.parametrize('reason', ['remote_stop', 'invalid_receiver_envelope', 'steady_clock_regression'])
def test_zero_sequence_may_latch_before_first_command(reason):
    evidence = ReceiverEvidence('test')
    push(evidence, disarmed())
    push(evidence, stopped(wall_s=10.1, sim_s=.1, command_seq=0,
                          deadline_steady_s=0., reason=reason))
    assert evidence.error is None
    summary = evidence.summary(0., .1)
    assert summary['final_zero'] and summary['fault_latched'] == (reason != 'remote_stop')


@pytest.mark.parametrize('with_prior_status', [False, True])
def test_deeply_nested_json_cannot_escape_callback_or_leave_audit_healthy(with_prior_status):
    evidence = ReceiverEvidence('test')
    if with_prior_status:
        push(evidence, accepted())
    raw = '{"nested":' + '['*2000 + '0' + ']'*2000 + '}'
    evidence.push(raw)
    assert evidence.error and evidence.error.startswith('RecursionError:')
    error = evidence.error
    push(evidence, stopped(wall_s=11., sim_s=.2))
    assert evidence.error == error and evidence.summary(.1, .2)['fault_latched']
    with pytest.raises(RuntimeError):
        evidence.require_ready()


def test_bad_status_blocks_real_adapter_submission_but_not_zero_stop():
    from simulation.gazebo_cross_validation import GazeboIO
    io = GazeboIO.__new__(GazeboIO)
    io.condition = threading.Condition()
    io.actuator = Mock()
    io.receiver_evidence = ReceiverEvidence('test')
    push(io.receiver_evidence, disarmed(command_seq=1, deadline_steady_s=10.8))
    io.command_context = {'bound': 'pending'}
    with pytest.raises(RuntimeError, match='authority state'):
        io.command(.1, 0.)
    io.actuator.request.assert_not_called()
    assert io.command_context is None
    io.command(0., 0.)
    io.actuator.request.assert_called_once_with('stop')


def test_transport_delay_is_not_confused_with_status_emission_deadline():
    # No read of receipt-time wall clock is needed for consistency checks.
    evidence = ReceiverEvidence('test')
    push(evidence, accepted(wall_s=1., deadline_steady_s=1.8))
    assert evidence.error is None
    # This validation alone does NOT add a heartbeat/receipt-age guarantee.
    assert evidence.events[-1]['deadline_steady_s'] == 1.8
