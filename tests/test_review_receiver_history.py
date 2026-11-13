"""Finite strict status prefix, sticky exhaustion; zero submission is independent."""
import copy
import json
import threading
from types import SimpleNamespace as NS

import pytest

from simulation import gazebo_receiver as receiver
from simulation import gazebo_cross_validation as runner
from tests.test_review_receiver_frame import strict, echo, disarmed
from tests.test_review_receiver_binding import sent, push
from tests.test_review_receiver_freshness import adapter
from tests.test_review_gazebo_watchdog import packet, SESSION, client
from tests.test_review_gazebo_failure_capture import harness

LIMIT = 65536


@pytest.fixture(scope='module')
def prefix():
    e = strict(); raw = json.dumps(disarmed(wall_s=10.))
    for _ in range(LIMIT): e.push(raw)
    assert not e.error and len(e.events) == LIMIT
    return tuple(e.events)


def full(prefix, e=None, last=None):
    e = e if e is not None else strict()
    e.events = list(prefix if last is None else prefix[:-1])
    if last is not None: push(e, last)
    assert len(e.events) == LIMIT and e.error is None
    return e


def test_actual_limit_keeps_all_valid_events_without_preemptive_fault(prefix):
    e = full(prefix); e.require_fresh(10.)
    summary = e.summary(0., .1)
    assert receiver.MAX_RECEIVER_EVENTS == LIMIT
    assert not summary['fault_latched'] and len(e.events) == LIMIT
    assert summary['status_history']['retained'] == LIMIT
    assert not summary['status_history']['capacity_exceeded']
    assert summary['status_history']['not_retained'] == 0


@pytest.mark.parametrize('payload', [json.dumps(disarmed(wall_s=10.2)), '{}', 'not JSON',
                                   '', None, 'x'*2_000_001])
def test_first_unretained_input_faults_without_parsing_or_mutating_prefix(prefix, payload, monkeypatch):
    e = full(prefix); original = list(e.events)
    monkeypatch.setattr(receiver, 'decode', lambda _: pytest.fail('Exhausted history must not decode'))
    e.push(payload)
    assert e.error and 'history capacity exceeded' in e.error and e.events == original
    summary = e.summary(0., .1)
    assert summary['fault_latched'] and summary['status_history']['capacity_exceeded']
    assert summary['status_history']['not_retained'] == 1
    assert not summary['status_history']['not_retained_count_is_lower_bound']
    with pytest.raises(RuntimeError): e.require_fresh(10.2)


def test_last_free_slot_is_usable_but_next_status_cannot_renew_evidence(prefix):
    e = strict(); e.events = list(prefix[:-1])
    push(e, disarmed(wall_s=10.1, sim_s=.2)); e.require_fresh(10.2)
    assert len(e.events) == LIMIT and e.events[-1]['wall_s'] == 10.1
    push(e, disarmed(wall_s=10.3, sim_s=.3))
    assert len(e.events) == LIMIT and e.events[-1]['wall_s'] == 10.1
    assert e._last_ready_wall_s == 10.2
    with pytest.raises(RuntimeError): e.require_fresh(10.4)


@pytest.mark.parametrize('previous', ['ValueError: receiver status differs from actual submission',
                                    'RuntimeError: clock regressed'])
def test_original_error_is_never_replaced_by_capacity_fault(prefix, previous):
    e = full(prefix); e.error = previous
    for _ in range(3): push(e, disarmed(wall_s=10.2))
    assert e.error == previous and e.summary(0., .1)['status_history']['not_retained'] == 3


@pytest.mark.parametrize('later', [echo(wall_s=10.2, sim_s=.2),
    echo(wall_s=10.2, sim_s=.2, latched=True, reason='remote_stop', wheels=[0., 0.])])
def test_correct_later_accepted_or_zero_state_cannot_clear_capacity_or_certify_frame(prefix, later):
    e, _ = sent(strict()); full(prefix, e, echo())
    final = copy.deepcopy(e.events[-1]); push(e, later); error = e.error
    push(e, later)
    result = e.summary(0., .1)
    assert e.error == error and e.events[-1] == final
    assert result['fault_latched'] and not result['submission_binding']['frame_echo_verified']
    assert result['status_history']['not_retained'] == 2


def test_counter_saturates_at_fixed_uint64_and_is_declared_only_lower_bound(prefix):
    e = full(prefix); maximum = receiver.MAX_HISTORY_NOT_RETAINED
    assert maximum == 2**64-1
    e._history_not_retained = maximum-1
    push(e, disarmed()); assert e._history_not_retained == maximum
    for _ in range(10): push(e, disarmed())
    result = e.summary(0., .1)
    assert result['status_history']['not_retained'] == maximum
    assert result['status_history']['not_retained_count_is_lower_bound']
    assert len(e.events) == LIMIT


@pytest.mark.parametrize('kind', ['idle', 'stop', 'shutdown', 'terminal'])
def test_history_exhaustion_never_blocks_recording_a_successfully_sent_zero_request(prefix, kind):
    e = full(prefix); push(e, disarmed()); error = e.error
    if kind == 'terminal':
        p = packet(); p['command'].update(state='aborted', v_m_s=0., omega_rad_s=0.)
    else: p = dict(kind=kind, session_id=SESSION, request_id=1)
    e.record_submission(p, 10.01)
    assert e.error == error and len(e.submissions) == 1 and not e._submitted_commands


def test_no_submission_or_candidate_is_evicted_or_rewritten_by_history_capacity(prefix):
    e, _ = sent(strict()); full(prefix, e, echo())
    before = copy.deepcopy((e.submissions, e._submitted_commands))
    for _ in range(5): push(e, echo(wall_s=10.2))
    assert (e.submissions, e._submitted_commands) == before


@pytest.mark.parametrize('kind', ['idle', 'stop'])
def test_real_ipc_zero_request_and_ack_stay_healthy_after_history_exhaustion(prefix, client, kind):
    obj, _, _ = client; e = full(prefix); push(e, disarmed()); error = e.error
    obj.submission_observer = e.record_submission
    first = obj.request(kind); second = obj.request('stop')
    assert first['accepted'] and second['accepted'] and not obj.failed
    assert obj.channel.fileno() >= 0 and obj.status['wheels'] == [0., 0.]
    assert e.error == error and len(e.submissions) == 2 and not e._submitted_commands
    assert [r['packet']['kind'] for r in e.submissions] == [kind, 'stop']


@pytest.mark.parametrize('frame_echo', [False, True])
def test_actual_adapter_checks_capacity_before_nonzero_but_zero_stop_is_available(prefix, monkeypatch, frame_echo):
    io = adapter(monkeypatch, None, now=10.2)
    io.receiver_evidence = full(prefix)
    if frame_echo:
        sent(io.receiver_evidence)
    push(io.receiver_evidence, disarmed(wall_s=10.2))
    with pytest.raises(RuntimeError, match='history capacity'): io.command(.1, 0.)
    io.actuator.request.assert_not_called()
    io.command(0., 0.); io.actuator.request.assert_called_once_with('stop')


def test_callback_capacity_check_is_serialized_by_original_io_condition(prefix):
    io = runner.GazeboIO.__new__(runner.GazeboIO)
    io.condition = threading.Condition(); io.receiver_evidence = strict()
    io.receiver_evidence.events = list(prefix[:-1])
    threads = [threading.Thread(target=io.receiver_status,
        args=(NS(data=json.dumps(disarmed(wall_s=10.1))),)) for _ in range(8)]
    for t in threads: t.start()
    for t in threads: t.join(2.); assert not t.is_alive()
    assert len(io.receiver_evidence.events) == LIMIT
    assert io.receiver_evidence._history_not_retained == 7
    assert io.receiver_evidence.error


@pytest.mark.parametrize('bound', [False, True])
def test_explicit_legacy_offline_mode_preserves_prior_history_behavior(prefix, bound):
    e = receiver.ReceiverEvidence(SESSION, bind_submissions=bound)
    e.events = list(prefix)
    push(e, disarmed(wall_s=10.1))
    assert len(e.events) == LIMIT+1 and not e.error and 'status_history' not in e.summary(0., .1)


def test_invalid_input_before_capacity_does_not_consume_slot_or_erase_original_error(prefix):
    e = strict(); e.events = list(prefix[:-1]); e.push('{}'); error = e.error
    assert error and len(e.events) == LIMIT-1
    push(e, disarmed(wall_s=10.1)); assert len(e.events) == LIMIT and e.error == error
    push(e, disarmed(wall_s=10.2)); assert e.error == error and e._history_not_retained == 1


def test_runner_exhaustion_preserves_failed_tick_cancel_and_original_cleanup(tmp_path, harness, prefix):
    control, _, calls = harness; original = runner.GazeboIO
    class IO(original):
        def __init__(self, **kw):
            super().__init__(**kw); self.receiver_evidence = full(prefix)
            push(self.receiver_evidence, disarmed())
        def command(self, v, omega):
            if v or omega: self.receiver_evidence.require_fresh(10.2)
            return super().command(v, omega)
    runner.GazeboIO = IO
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=3)
    assert not result['completed'] and result['exception'] and control.cancelled
    assert result['controller_result']['state'] == 'aborted'
    assert result['failure_record']['pending_tick']['stage'] == 'command_submission'
    assert result['failure_record']['zero_command_submitted'] and result['cleanup_stop']['steps_completed'] == 10
    assert result['receiver_interlock']['fault_latched'] and result['receiver_interlock']['status_history']['capacity_exceeded']
    assert not any(c[0] == 'command' and (c[1] or c[2]) for c in calls)
    assert len(json.loads((tmp_path/'capture/audit/receiver.json').read_text())) == LIMIT


def test_budget_exceeds_declared_normal_tick_status_budget_but_is_not_rate_or_process_bound():
    assert LIMIT > 2200*13+5*12+10*12+12+20
    summary = strict().summary(None, None)
    assert 'NOT submission/pose/transport/whole-process memory bounds' in summary['status_history']['scope']
    assert 'NOT parsed/validated/freshness evidence' in summary['status_history']['scope']
