"""Finite strict post-send ledger; omitted calls are not validated send evidence."""
import copy
import json
import threading

import pytest

from simulation import gazebo_receiver as receiver
from simulation import gazebo_cross_validation as runner
from tests.test_review_receiver_frame import strict, echo, disarmed
from tests.test_review_receiver_binding import sent, push
from tests.test_review_receiver_freshness import adapter
from tests.test_review_gazebo_failure_capture import harness
from tests.test_review_gazebo_watchdog import SESSION, packet, client

LIMIT = 16384


def control(kind='idle', request_id=1):
    return dict(kind=kind, session_id=SESSION, request_id=request_id)


@pytest.fixture(scope='module')
def prefix():
    e = strict()
    for request_id in range(1, LIMIT+1): e.record_submission(control(request_id=request_id), 10.)
    assert not e.error and len(e.submissions) == LIMIT
    return tuple(e.submissions)


def full(prefix, e=None):
    e = e if e is not None else strict()
    if not e.submissions: e.submissions = list(prefix)
    else:
        for i in range(len(e.submissions), LIMIT): e.record_submission(control(request_id=i+1), 10.1)
    assert len(e.submissions) == LIMIT and not e.error
    return e


def overflow(prefix, e=None):
    e = full(prefix, e); e.record_submission(control(request_id=LIMIT+1), 10.2); return e


def test_actual_record_limit_retains_every_slot_without_preemptive_fault(prefix):
    e = full(prefix); push(e, disarmed(wall_s=10.))
    e.require_fresh(10.)
    assert receiver.MAX_RECEIVER_SUBMISSIONS == LIMIT > receiver.MAX_RECEIVER_CANDIDATES
    assert [r['packet']['request_id'] for r in e.submissions] == list(range(1, LIMIT+1))
    s = e.summary(None, None)
    assert not s['fault_latched'] and not s['submission_history']['capacity_exceeded']
    assert s['submission_history']['not_retained'] == 0


@pytest.mark.parametrize('payload', [control(request_id=LIMIT+1), {}, None, 'x'*2_000_001,
                                   {'deep': [[[[None]]]]}])
def test_first_omitted_call_faults_without_copying_parsing_or_advancing_retained_state(prefix, payload, monkeypatch):
    e = full(prefix); before = list(e.submissions)
    monkeypatch.setattr(receiver.copy, 'deepcopy', lambda _: pytest.fail('Exhausted ledger must not copy input'))
    e.record_submission(payload, None)
    assert e.submissions == before and not e._submitted_commands and e._submitted_seq == 0
    assert not e._terminal_sent and e._submission_not_retained == 1
    assert 'submission history capacity exceeded' in e.error
    with pytest.raises(RuntimeError): e.require_fresh(10.2)


def test_last_slot_runs_original_validation_and_first_extra_call_does_not_change_it(prefix):
    e = strict(); e.submissions = list(prefix[:-1])
    e.record_submission(control(request_id=LIMIT), 10.1)
    assert not e.error and e.submissions[-1]['sent_steady_s'] == 10.1
    e.record_submission(control(request_id=LIMIT+1), 10.2)
    assert len(e.submissions) == LIMIT and e.submissions[-1]['sent_steady_s'] == 10.1


@pytest.mark.parametrize('bad', ['request_id', 'session', 'clock', 'fields'])
def test_invalid_record_before_last_slot_still_raises_and_does_not_consume_capacity(prefix, bad):
    e = strict(); e.submissions = list(prefix[:-1]); p = control(request_id=LIMIT); stamp = 10.1
    if bad == 'request_id': p['request_id'] += 1
    elif bad == 'session': p['session_id'] = 'other'
    elif bad == 'clock': stamp = 9.
    else: p['extra'] = 1
    with pytest.raises(RuntimeError, match='recording failed'): e.record_submission(p, stamp)
    error = e.error
    assert len(e.submissions) == LIMIT-1 and not e._submission_not_retained
    e.record_submission(control(request_id=LIMIT), 10.1)
    e.record_submission(control(request_id=LIMIT+1), 10.2)
    assert e.error == error and e._submission_not_retained == 1


@pytest.mark.parametrize('original', ['ValueError: bad frame echo', 'RuntimeError: Receiver command candidate capacity exceeded'])
def test_earlier_error_is_not_replaced_by_ledger_capacity(prefix, original):
    e = full(prefix); e.error = original
    for _ in range(3): e.record_submission(None, None)
    assert e.error == original and e._submission_not_retained == 3


def test_counter_saturates_at_uint64_and_summary_declares_lower_bound(prefix):
    e = overflow(prefix); maximum = receiver.MAX_SUBMISSION_NOT_RETAINED
    assert maximum == 2**64-1
    e._submission_not_retained = maximum-1
    for _ in range(5): e.record_submission(None, None)
    s = e.summary(None, None)['submission_history']
    assert s['not_retained'] == maximum and s['not_retained_count_is_lower_bound']
    assert len(e.submissions) == LIMIT


@pytest.mark.parametrize('state', ['moving', 'observing', 'visual_stop', 'aborted'])
def test_omitted_command_never_adds_candidate_or_terminal_or_sequence_state(prefix, state):
    e = full(prefix); p = packet(request_id=LIMIT+1)
    p['command'].update(state=state, v_m_s=0. if state != 'moving' else .1)
    e.record_submission(p, 10.01)
    assert e.error and not e._submitted_commands and e._submitted_seq == 0 and not e._terminal_sent
    assert len(e.submissions) == LIMIT and e._submission_not_retained == 1


@pytest.mark.parametrize('latched', [False, True])
def test_kept_feedback_cannot_certify_binding_after_an_omitted_send(prefix, latched):
    e, _ = sent(strict()); full(prefix, e); before = copy.deepcopy(e._submitted_commands)
    e.record_submission(control('stop', LIMIT+1), 10.2)
    push(e, echo(wall_s=10.3, latched=latched, reason='remote_stop' if latched else 'accepted',
                 wheels=[0., 0.] if latched else [.1, 0.]))
    s = e.summary(0., .1)
    assert s['fault_latched'] and not s['submission_binding']['final_authority_bound']
    assert not s['submission_binding']['frame_echo_verified'] and e._submitted_commands == before


def test_omitted_command_feedback_is_not_reconstructed_or_blessed(prefix):
    e = full(prefix); e.record_submission(packet(request_id=LIMIT+1), 10.01); error = e.error
    push(e, echo())
    assert not e.events and e.error == error and not e._submitted_commands


def test_correct_later_disarmed_and_stop_do_not_clear_original_capacity_fault(prefix):
    e = overflow(prefix); error = e.error
    push(e, disarmed(wall_s=10.2))
    push(e, dict(disarmed(wall_s=10.3), latched=True, reason='remote_stop'))
    assert len(e.events) == 2 and e.error == error and e.summary(0., .1)['fault_latched']


def test_callback_limit_is_serialized_by_original_io_condition(prefix):
    io = runner.GazeboIO.__new__(runner.GazeboIO); io.condition = threading.Condition()
    io.receiver_evidence = strict(); io.receiver_evidence.submissions = list(prefix[:-1])
    threads = [threading.Thread(target=io._record_actuator_submission,
        args=(control(request_id=LIMIT), 10.1)) for _ in range(8)]
    for t in threads: t.start()
    for t in threads: t.join(2.); assert not t.is_alive()
    assert len(io.receiver_evidence.submissions) == LIMIT and io.receiver_evidence._submission_not_retained == 7


def test_full_ledger_adapter_blocks_nonzero_and_still_sends_stop(prefix, monkeypatch):
    io = adapter(monkeypatch, None, now=10.2); io.receiver_evidence = overflow(prefix)
    with pytest.raises(RuntimeError, match='submission history capacity'): io.command(.1, 0.)
    io.actuator.request.assert_not_called()
    io.command(0., 0.); io.actuator.request.assert_called_once_with('stop')


def test_real_ipc_original_full_budget_idle_stop_shutdown_ack_and_worker_remain_healthy(client):
    obj, _, _ = client; e = strict(); obj.submission_observer = e.record_submission
    for _ in range(LIMIT+1): assert obj.request('idle')['accepted']
    assert len(e.submissions) == LIMIT and e._submission_not_retained == 1 and e.error
    assert not obj.failed and obj.request('stop')['accepted'] and obj.request('shutdown')['accepted']
    assert not obj.failed and obj.channel.fileno() >= 0 and e._submission_not_retained == 3
    assert not e._submitted_commands and e._submitted_seq == 0
    obj.process.wait(timeout=5)


def test_actual_ipc_ack_validation_is_not_bypassed_when_observer_ledger_is_full(prefix, client, monkeypatch):
    obj, _, _ = client; e = overflow(prefix)
    obj.submission_observer = e.record_submission; original = obj._receive
    def corrupt(timeout):
        reply = original(timeout); reply['status']['command_seq'] += 1
        reply['status']['deadline_steady_s'] = reply['published_steady_s']+.8
        return reply  # Structurally valid, but STOP must retain prior seq/deadline.
    monkeypatch.setattr(obj, '_receive', corrupt)
    with pytest.raises(RuntimeError, match='stop acknowledgement changed authority'): obj.request('stop')
    assert obj.failed and obj.channel.fileno() < 0 and e._submission_not_retained == 2


@pytest.mark.parametrize('bound', [False, True])
def test_legacy_offline_compatibility_remains_explicitly_unbounded(prefix, bound):
    e = receiver.ReceiverEvidence(SESSION, bind_submissions=bound)
    e.submissions = list(prefix)
    if bound:
        e.record_submission(control(request_id=LIMIT+1), 10.1)
        assert len(e.submissions) == LIMIT+1 and not e.error
    else:
        with pytest.raises(RuntimeError): e.record_submission(control(request_id=LIMIT+1), 10.1)
    assert not e._submission_not_retained and 'submission_history' not in e.summary(None, None)


def test_runner_keeps_failed_tick_cancel_and_original_cleanup_with_incomplete_ledger(tmp_path, harness, prefix):
    control_object, _, calls = harness; original = runner.GazeboIO
    class IO(original):
        def __init__(self, **kw):
            super().__init__(**kw); self.receiver_evidence = overflow(prefix)
        def command(self, v, omega):
            if v or omega: self.receiver_evidence.require_fresh(10.2)
            return super().command(v, omega)
    runner.GazeboIO = IO
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=3)
    assert not result['completed'] and control_object.cancelled and result['controller_result']['state'] == 'aborted'
    assert result['failure_record']['pending_tick']['stage'] == 'command_submission'
    assert result['failure_record']['zero_command_submitted'] and result['cleanup_stop']['steps_completed'] == 10
    assert result['receiver_interlock']['submission_history']['capacity_exceeded']
    records = json.loads((tmp_path/'capture/audit/receiver-submissions.json').read_text())
    assert len(records) == LIMIT
    assert not any(c[0] == 'command' and (c[1] or c[2]) for c in calls)


def test_summary_never_claims_prefix_length_is_complete_send_count_or_whole_memory_bound(prefix):
    e = overflow(prefix); s = e.summary(None, None)
    assert s['submission_binding']['sent_requests'] == LIMIT
    history = s['submission_history']; assert history['sent_requests_field_is_retained_count']
    assert 'UNVALIDATED observer calls' in history['scope'] and 'whole-process memory bounds' in history['scope']
    assert history['retained'] == LIMIT and history['not_retained'] == 1
