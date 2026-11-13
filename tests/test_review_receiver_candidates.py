"""Bound only strict sent-candidate storage, not accepted authority or all memory."""
import copy
import json
import time
from pathlib import Path

import pytest

from simulation import gazebo_receiver as receiver
from simulation import gazebo_cross_validation as runner
from tests.test_review_receiver_frame import strict, echo, disarmed
from tests.test_review_receiver_binding import push
from tests.test_review_receiver_freshness import adapter
from tests.test_review_gazebo_failure_capture import harness
from tests.test_review_gazebo_watchdog import SESSION, packet, client

LIMIT = 8192


def candidate(seq, request_id=None, **changes):
    p = packet(request_id=seq if request_id is None else request_id, seq=seq,
               stamp=seq*.001, issued=10.+seq*.00001)
    p['command'].update(changes)
    return p


def record(e, p):
    e.record_submission(p, p['issued_steady_s']+.000001)


@pytest.fixture(scope='module')
def prefix():
    e = strict()
    for seq in range(1, LIMIT+1): record(e, candidate(seq))
    assert not e.error and len(e._submitted_commands) == LIMIT
    return e


def full(prefix): return copy.deepcopy(prefix)


def overflow(prefix, **changes):
    e = full(prefix); record(e, candidate(LIMIT+1, **changes)); return e


def status_for(e, seq, **changes):
    c = e._submitted_commands[seq]
    item = echo(command_seq=seq, wall_s=10.2, sim_s=seq*.001,
        deadline_steady_s=c['deadline_steady_s'], frame_id_hex=c['frame_id_hex'],
        captured_sim_s=c['captured_sim_s'], issued_steady_s=c['issued_steady_s'],
        wheels=list(c['wheels']))
    item.update(changes)
    return item


def test_real_record_path_keeps_last_slot_and_does_not_preemptively_fault(prefix):
    e = full(prefix)
    assert receiver.MAX_RECEIVER_CANDIDATES == LIMIT > receiver.MAX_RECEIVER_FRAMES == 4096
    assert list(e._submitted_commands) == list(range(1, LIMIT+1))
    assert len(e.submissions) == LIMIT and e._submitted_seq == LIMIT and not e.error
    push(e, status_for(e, LIMIT)); e.require_fresh(10.2)
    s = e.summary(0., LIMIT*.001)
    assert not s['fault_latched'] and s['submission_binding']['frame_echo_verified']
    assert s['candidate_history']['retained'] == LIMIT and not s['candidate_history']['capacity_exceeded']
    # Synthetic bound-status test, not proof native accepts more than 4096.


@pytest.mark.parametrize('state,v,w', [('moving', .1, 0.), ('observing', 0., 0.),
                                     ('moving', 0., .3)])
def test_first_excess_send_is_logged_but_cannot_grow_or_replace_candidate_prefix(prefix, state, v, w):
    e = overflow(prefix, state=state, v_m_s=v, omega_rad_s=w)
    assert e.error and 'candidate capacity exceeded' in e.error
    assert e._submitted_commands == prefix._submitted_commands and e._submitted_seq == LIMIT+1
    assert len(e.submissions) == LIMIT+1 and e.submissions[:-1] == prefix.submissions
    assert e.submissions[-1]['packet']['command']['seq'] == LIMIT+1
    assert e.summary(None, None)['candidate_history']['capacity_exceeded']
    with pytest.raises(RuntimeError, match='candidate capacity'): e.require_fresh(10.2)


def test_repeated_zero_sends_advance_original_order_without_candidate_growth(prefix):
    e = full(prefix)
    for seq in range(LIMIT+1, LIMIT+9): record(e, candidate(seq, state='observing', v_m_s=0.))
    assert e._submitted_seq == LIMIT+8 and len(e.submissions) == LIMIT+8
    assert e._submitted_commands == prefix._submitted_commands and e._candidate_capacity_exceeded


@pytest.mark.parametrize('original', ['ValueError: bad receiver frame', 'RuntimeError: history capacity exceeded'])
def test_original_error_survives_candidate_exhaustion(prefix, original):
    e = full(prefix); e.error = original; record(e, candidate(LIMIT+1))
    assert e.error == original and e._candidate_capacity_exceeded


@pytest.mark.parametrize('state', ['visual_stop', 'aborted'])
@pytest.mark.parametrize('exhausted', [False, True])
def test_terminal_commands_need_no_candidate_and_do_not_clear_fault(prefix, state, exhausted):
    e = overflow(prefix) if exhausted else full(prefix); old_error = e.error
    seq = e._submitted_seq+1
    record(e, candidate(seq, state=state, v_m_s=0., omega_rad_s=0.))
    assert e.error == old_error and e._terminal_sent and e._submitted_seq == seq
    assert e._submitted_commands == prefix._submitted_commands
    assert len(e.submissions) == seq and e._candidate_capacity_exceeded == exhausted


@pytest.mark.parametrize('kind', ['idle', 'stop', 'shutdown'])
@pytest.mark.parametrize('exhausted', [False, True])
def test_control_requests_record_normally_without_using_a_candidate(prefix, kind, exhausted):
    e = overflow(prefix) if exhausted else full(prefix); old_error = e.error
    e.record_submission(dict(kind=kind, session_id=SESSION, request_id=len(e.submissions)+1), 10.2)
    assert e.error == old_error and e._submitted_commands == prefix._submitted_commands
    assert e.submissions[-1]['packet']['kind'] == kind


@pytest.mark.parametrize('change', ['request_id', 'seq', 'clock', 'fields', 'terminal_velocity'])
@pytest.mark.parametrize('exhausted', [False, True])
def test_capacity_never_bypasses_original_submission_validation(prefix, change, exhausted):
    e = overflow(prefix) if exhausted else full(prefix); seq = e._submitted_seq+1
    p = candidate(seq); stamp = p['issued_steady_s']+.000001
    if change == 'request_id': p['request_id'] += 1
    elif change == 'seq': p['command']['seq'] += 1
    elif change == 'clock': stamp = 9.
    elif change == 'fields': p['extra'] = True
    else: p['command']['state'] = 'aborted'  # Terminal nonzero is invalid.
    before = copy.deepcopy((e.submissions, e._submitted_commands, e._submitted_seq)); old_error = e.error
    with pytest.raises(RuntimeError, match='recording failed'): e.record_submission(p, stamp)
    assert (e.submissions, e._submitted_commands, e._submitted_seq) == before
    if exhausted: assert e.error == old_error
    else: assert e.error is not None


@pytest.mark.parametrize('latched', [False, True])
def test_old_retained_feedback_cannot_certify_latest_omitted_send(prefix, latched):
    e = overflow(prefix)
    item = status_for(e, LIMIT)
    if latched: item.update(latched=True, reason='remote_stop', wheels=[0., 0.])
    push(e, item)
    s = e.summary(0., LIMIT*.001)
    assert s['fault_latched'] and not s['submission_binding']['final_authority_bound']
    assert not s['submission_binding']['frame_echo_verified'] and e.error


def test_omitted_candidate_is_not_reconstructed_from_full_send_ledger(prefix):
    e = overflow(prefix); c = e.submissions[-1]['packet']; command = c['command']
    item = echo(command_seq=LIMIT+1, sim_s=command['captured_sim_s'], wall_s=10.2,
        deadline_steady_s=c['expires_steady_s'], issued_steady_s=c['issued_steady_s'],
        captured_sim_s=command['captured_sim_s'], frame_id_hex=command['frame_id'].encode().hex())
    error = e.error; push(e, item)
    assert e.error == error and not e.events and len(e._submitted_commands) == LIMIT


def test_later_valid_status_and_stop_cannot_wash_candidate_fault(prefix):
    e = overflow(prefix); push(e, status_for(e, LIMIT)); error = e.error
    push(e, dict(status_for(e, LIMIT), wall_s=10.3, latched=True, reason='remote_stop', wheels=[0., 0.]))
    assert len(e.events) == 2 and e.error == error
    assert e.summary(0., LIMIT*.001)['fault_latched']


def test_status_history_exhaustion_remains_separate_and_does_not_evict_candidates(prefix):
    e = overflow(prefix); error = e.error
    e.events = [disarmed()]*receiver.MAX_RECEIVER_EVENTS
    push(e, disarmed())
    s = e.summary(0., .1)
    assert s['candidate_history']['capacity_exceeded'] and s['status_history']['capacity_exceeded']
    assert e._submitted_commands == prefix._submitted_commands and e.error == error


def test_legacy_bound_offline_mode_keeps_previous_candidate_behavior(prefix):
    e = full(prefix); e.require_frame_echo = False
    record(e, candidate(LIMIT+1))
    assert not e.error and len(e._submitted_commands) == LIMIT+1
    assert not e._candidate_capacity_exceeded and 'candidate_history' not in e.summary(None, None)


def test_actual_adapter_blocks_nonzero_after_capacity_but_stop_is_independent(prefix, monkeypatch):
    io = adapter(monkeypatch, None, now=10.2); io.receiver_evidence = overflow(prefix)
    with pytest.raises(RuntimeError, match='candidate capacity'): io.command(.1, 0.)
    io.actuator.request.assert_not_called()
    io.command(0., 0.); io.actuator.request.assert_called_once_with('stop')


def test_runner_preserves_failed_tick_cancel_cleanup_and_both_audits(tmp_path, harness, prefix):
    control, _, calls = harness; original = runner.GazeboIO
    class IO(original):
        def __init__(self, **kw):
            super().__init__(**kw); self.receiver_evidence = overflow(prefix)
        def command(self, v, omega):
            if v or omega: self.receiver_evidence.require_fresh(10.2)
            return super().command(v, omega)
    runner.GazeboIO = IO
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=3)
    assert not result['completed'] and control.cancelled and result['controller_result']['state'] == 'aborted'
    assert result['failure_record']['pending_tick']['stage'] == 'command_submission'
    assert result['failure_record']['zero_command_submitted'] and result['cleanup_stop']['steps_completed'] == 10
    assert result['receiver_interlock']['candidate_history']['capacity_exceeded']
    records = json.loads((tmp_path/'capture/audit/receiver-submissions.json').read_text())
    assert len(records) == LIMIT+1
    assert not any(c[0] == 'command' and (c[1] or c[2]) for c in calls)


def test_real_ipc_full_original_budget_zero_commands_and_acks_remain_healthy(client):
    obj, _, _ = client; e = strict(); obj.submission_observer = e.record_submission
    for seq in range(1, LIMIT+2):
        p = candidate(seq, state='observing', v_m_s=0.)
        p['issued_steady_s'] = time.monotonic(); p['expires_steady_s'] = p['issued_steady_s']+.8
        reply = obj.request('command', **{k:v for k,v in p.items() if k not in ('kind','session_id','request_id')})
        assert reply['accepted'] and reply['status']['wheels'] == [0., 0.]
    assert e.error and e._candidate_capacity_exceeded and not obj.failed
    assert len(e._submitted_commands) == LIMIT and len(e.submissions) == LIMIT+1
    assert obj.request('stop')['accepted'] and obj.request('shutdown')['accepted']
    assert not obj.failed and obj.channel.fileno() >= 0 and len(e.submissions) == LIMIT+3
    obj.process.wait(timeout=5)
    # Real worker/socket, synthetic frame metadata, no native/RGB/physics claim.


def test_no_tuning_or_whole_process_bound_claim():
    s = strict().summary(None, None)['candidate_history']
    assert 'NOT accepted commands' in s['scope'] and 'whole-process memory bounds' in s['scope']
    assert LIMIT > 2200 and LIMIT > receiver.MAX_RECEIVER_FRAMES
    source = (Path(__file__).resolve().parents[1]/'simulation/gazebo_cross_validation.py').read_text()
    assert 'MAX_RECEIVER_CANDIDATES' not in source
