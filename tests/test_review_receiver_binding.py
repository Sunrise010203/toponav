"""Bind observable native content to SENT packets, not to self-consistent claims."""
import copy
import json
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from simulation import gazebo_cross_validation as runner
from simulation import gazebo_actuator as actuator
from simulation.gazebo_receiver import ReceiverEvidence
from tests.test_review_gazebo_watchdog import client, packet, SESSION, send_motion
from tests.test_review_gazebo_failure_capture import harness


def status(**changes):
    result = dict(version=1, session_id=SESSION, wall_s=10.1, sim_s=.1,
        latched=False, reason='accepted', command_seq=1, deadline_steady_s=10.8, wheels=[.1, 0.])
    result.update(changes)
    return result


def push(e, item): e.push(json.dumps(item))


def bound(): return ReceiverEvidence(SESSION, bind_submissions=True)


def sent(e=None, **changes):
    e = e if e is not None else bound()
    p = packet(**changes)
    e.record_submission(p, p['issued_steady_s']+.01)
    return e, p


@pytest.mark.parametrize('changes', [dict(wheels=[.14, .05]), dict(wheels=[.1, .01]),
    dict(wheels=[0., 0.]), dict(command_seq=2), dict(command_seq=501),
    dict(deadline_steady_s=10.7), dict(deadline_steady_s=10.9), dict(wall_s=10.)])
def test_coherent_but_unbound_status_is_rejected(changes):
    item = status(**changes)
    legacy = ReceiverEvidence(SESSION); push(legacy, item)
    assert legacy.error is None  # This is not a malformed-status test.
    e, _ = sent(); push(e, item)
    assert e.error and 'submission' in e.error and not e.events
    with pytest.raises(RuntimeError): e.require_fresh(10.2)


@pytest.mark.parametrize('latched', [False, True])
def test_status_without_sent_command_cannot_invent_a_sequence(latched):
    e = bound()
    push(e, status(latched=latched, reason='remote_stop' if latched else 'accepted',
                   wheels=[0., 0.] if latched else [.1, 0.]))
    assert e.error and not e.events


def test_unknown_early_status_is_not_blessed_by_a_later_matching_submission():
    e = bound(); push(e, status()); error = e.error
    sent(e); push(e, status(wall_s=10.2, sim_s=.2))
    assert e.error == error
    with pytest.raises(RuntimeError): e.require_ready()


@pytest.mark.parametrize('seq', [1, 2, 3])
def test_known_sequences_can_skip_missing_feedback_without_extending_deadline(seq):
    e = bound()
    for i in range(1, 4): sent(e, request_id=i, seq=i, stamp=i*.1, issued=10.+i*.01)
    p = e.submissions[seq-1]['packet']
    push(e, status(command_seq=seq, deadline_steady_s=p['issued_steady_s']+.8))
    assert e.error is None
    result = e.summary(0., .1)
    assert result['submission_binding']['final_authority_bound'] == (seq == 3)
    assert result['fault_latched'] == (seq != 3)


def test_lagged_known_content_may_be_fresh_but_cannot_certify_latest_final_zero():
    e, _ = sent(); sent(e, request_id=2, seq=2, stamp=.2, issued=10.05)
    push(e, status()); e.require_fresh(10.2)
    push(e, status(wall_s=10.3, sim_s=.3, latched=True, reason='remote_stop', wheels=[0., 0.]))
    result = e.summary(.2, .3)
    assert e.error is None and result['final_zero'] and result['final_step_fresh']
    assert result['fault_latched'] and not result['submission_binding']['final_authority_bound']


@pytest.mark.parametrize('velocity', [0., .1])
def test_fresh_disarmed_cannot_replace_feedback_after_a_command_was_sent(velocity):
    e = bound()
    push(e, status(wall_s=10., command_seq=0, reason='disarmed', deadline_steady_s=0., wheels=[0., 0.]))
    e.require_fresh(10.)  # Only before the first submitted command.
    p = packet(); p['command'].update(v_m_s=velocity, state='moving' if velocity else 'observing')
    e.record_submission(p, 10.01)
    push(e, status(wall_s=10.2, sim_s=.2, command_seq=0, reason='disarmed', deadline_steady_s=0., wheels=[0., 0.]))
    with pytest.raises(RuntimeError, match='submitted command feedback'): e.require_fresh(10.3)
    error = e.error
    push(e, status(wall_s=10.4, sim_s=.3, wheels=[velocity, 0.]))
    assert e.error == error


@pytest.mark.parametrize('reason', ['remote_stop', 'command_lease_expired', 'invalid_receiver_envelope', 'steady_clock_regression'])
def test_bound_zero_preserves_original_deadline_and_does_not_erase_faults(reason):
    e, _ = sent()
    wall = 10.8 if reason == 'command_lease_expired' else 10.2
    push(e, status(wall_s=wall, latched=True, reason=reason, wheels=[0., 0.]))
    result = e.summary(0., .1)
    assert e.error is None and result['submission_binding']['final_authority_bound'] and result['final_zero']
    assert result['fault_latched'] == (reason != 'remote_stop')
    assert 'stationary' not in result


@pytest.mark.parametrize('reason', ['disarmed', 'remote_stop', 'invalid_receiver_envelope', 'steady_clock_regression'])
def test_zero_sequence_has_no_invented_command_or_frame_proof(reason):
    e = bound()
    push(e, status(command_seq=0, deadline_steady_s=0., wheels=[0., 0.],
                   reason=reason, latched=reason != 'disarmed'))
    result = e.summary(0., .1)
    assert not e.error and result['submission_binding']['latest_native_candidate_seq'] == 0
    assert result['submission_binding']['final_authority_bound'] and not result['submission_binding']['frame_echo_verified']


@pytest.mark.parametrize('state', ['visual_stop', 'aborted'])
def test_terminal_command_is_stop_not_a_new_native_sequence_or_deadline(state):
    e, _ = sent(); p = packet(request_id=2, seq=2, stamp=.2, issued=10.1)
    p['command'].update(state=state, v_m_s=0., omega_rad_s=0.)
    e.record_submission(p, 10.11)
    push(e, status(wall_s=10.2, latched=True, reason='remote_stop', wheels=[0., 0.]))
    result = e.summary(0., .1)
    assert not e.error and not result['fault_latched'] and result['submission_binding']['final_authority_bound']
    assert e.submissions[-1]['packet']['command']['seq'] == 2
    assert result['submission_binding']['latest_native_candidate_seq'] == 1
    other = bound(); sent(other); other.record_submission(p, 10.11)
    push(other, status(command_seq=2, deadline_steady_s=10.9, latched=True, reason='remote_stop', wheels=[0., 0.]))
    assert other.error and 'submission' in other.error


def test_first_terminal_command_does_not_invent_native_authority():
    e = bound(); p = packet(); p['command'].update(state='aborted', v_m_s=0.)
    e.record_submission(p, 10.01)
    push(e, status(command_seq=0, deadline_steady_s=0., latched=True, reason='remote_stop', wheels=[0., 0.]))
    assert not e.error and e.summary(0., .1)['submission_binding']['latest_native_candidate_seq'] == 0


def test_exact_original_clamp_not_unclamped_expiry_or_receipt_renewal():
    e, _ = sent(expires_steady_s=10.8+5e-10)
    push(e, status()); assert not e.error
    other, _ = sent(expires_steady_s=10.8+5e-10)
    push(other, status(deadline_steady_s=10.8+5e-10))
    assert other.error


def test_submitted_packet_and_expected_content_are_copied_not_caller_aliases():
    e, p = sent(); original = copy.deepcopy(p)
    p['command'].update(v_m_s=.14, frame_id='tampered'); p['expires_steady_s'] = 10.9
    push(e, status())
    assert not e.error and e.submissions[0]['packet'] == original
    assert e.submissions[0]['packet']['command']['frame_id'] == 'f1'
    assert not e.summary(0., .1)['submission_binding']['frame_echo_verified']


@pytest.mark.parametrize('kind', ['idle', 'stop', 'shutdown'])
def test_noncommand_requests_do_not_create_or_renew_native_command(kind):
    e = bound()
    e.record_submission(dict(kind=kind, session_id=SESSION, request_id=1), 10.)
    assert not e._submitted_commands and len(e.submissions) == 1


@pytest.mark.parametrize('change', ['wrong_session', 'bool_id', 'skip_id', 'extra', 'unknown_kind',
    'wrong_command_session', 'skip_command_seq', 'bad_clock', 'future_issue', 'expired', 'extension', 'bad_sim', 'bad_frame'])
def test_bad_submission_recording_latches_and_never_grants_a_candidate(change):
    e = bound(); p = packet(); now = 10.01
    if change == 'wrong_session': p['session_id'] = 'other'
    elif change == 'bool_id': p['request_id'] = True
    elif change == 'skip_id': p['request_id'] = 2
    elif change == 'extra': p['extra'] = 1
    elif change == 'unknown_kind': p['kind'] = 'heartbeat'
    elif change == 'wrong_command_session': p['command']['session_id'] = 'other'
    elif change == 'skip_command_seq': p['command']['seq'] = 2
    elif change == 'bad_clock': now = float('nan')
    elif change == 'future_issue': now = 9.99
    elif change == 'expired': now = 10.8
    elif change == 'extension': p['expires_steady_s'] = 11.
    elif change == 'bad_sim': p['sim_s'] = .01
    elif change == 'bad_frame': p['command']['frame_id'] = ''
    with pytest.raises(RuntimeError, match='submission'): e.record_submission(p, now)
    assert e.error and not e._submitted_commands and not e.submissions


def test_submissions_after_terminal_or_clock_regression_fail_closed():
    e, _ = sent(); p = packet(2, 2, .2, 10.1); p['command'].update(state='visual_stop', v_m_s=0.)
    e.record_submission(p, 10.11)
    with pytest.raises(RuntimeError): sent(e, request_id=3, seq=3, stamp=.3, issued=10.2)
    assert list(e._submitted_commands) == [1]
    other, _ = sent()
    with pytest.raises(RuntimeError): other.record_submission(dict(kind='stop', session_id=SESSION, request_id=2), 10.)


def test_binding_error_cannot_block_zero_request_recording_or_be_erased():
    e, _ = sent(); push(e, status(wheels=[.14, .05])); error = e.error
    e.record_submission(dict(kind='stop', session_id=SESSION, request_id=2), 10.2)
    push(e, status(wall_s=10.3, latched=True, reason='remote_stop', wheels=[0., 0.]))
    assert e.error == error and e.summary(0., .1)['fault_latched']


def test_observer_sees_actual_sent_copy_before_ack_wait(client):
    obj, _, _ = client; e = bound(); order = []
    receive = obj._receive
    def observer(p, sent_at):
        order.append('sent'); e.record_submission(p, sent_at)
        p['command']['v_m_s'] = .14  # Cannot mutate the client's wire packet.
    def ack(timeout):
        order.append('ack'); assert len(e.submissions) == 1
        return receive(timeout)
    obj.submission_observer = observer; obj._receive = ack
    send_motion(client)
    assert order == ['sent', 'ack'] and obj.status['wheels'] == [.1, 0.]
    assert e.submissions[0]['packet']['command']['v_m_s'] == .1
    obj.submission_observer = None  # Fixture shutdown is unrelated to this observer's mutation.


def test_bad_ack_retains_unconfirmed_sent_candidate_but_does_not_authorize_recovery(client):
    obj, _, _ = client; e = bound(); obj.submission_observer = e.record_submission
    receive = obj._receive
    def bad(timeout):
        reply = receive(timeout); reply['status']['command_seq'] += 1; return reply
    obj._receive = bad
    with pytest.raises(RuntimeError): send_motion(client)
    assert obj.failed and obj.channel.fileno() == -1 and obj.status['command_seq'] == 0
    p = e.submissions[0]['packet']
    push(e, status(wall_s=time.monotonic(), latched=True, reason='remote_stop', wheels=[0., 0.],
                   deadline_steady_s=min(p['expires_steady_s'], p['issued_steady_s']+.8)))
    assert not e.error and e.summary(0., .1)['submission_binding']['final_authority_bound']
    with pytest.raises(RuntimeError, match='prior failure'): obj.request('command')
    assert len(e.submissions) == 1


def test_send_failure_never_notifies_observer_or_retries(client):
    obj, _, _ = client; e = bound(); obj.submission_observer = e.record_submission
    obj.channel.close()
    with pytest.raises(OSError): send_motion(client)
    assert obj.failed and not e.submissions
    with pytest.raises(RuntimeError): obj.request('stop')


def test_observer_failure_closes_channel_without_waiting_for_ack_or_resetting(client):
    obj, _, _ = client; calls = []
    def observer(*a): calls.append('observe'); raise RuntimeError('ledger unavailable')
    obj.submission_observer = observer; obj._receive = Mock(side_effect=AssertionError('must not await ACK'))
    with pytest.raises(RuntimeError, match='ledger unavailable'): send_motion(client)
    assert calls == ['observe'] and obj.failed and obj.channel.fileno() == -1
    obj._receive.assert_not_called()


def test_send_and_record_are_serialized_against_pre_ack_native_callback(monkeypatch):
    io = runner.GazeboIO.__new__(runner.GazeboIO); io.condition = threading.Condition()
    io.receiver_evidence = bound()
    push(io.receiver_evidence, status(wall_s=10., command_seq=0, reason='disarmed', deadline_steady_s=0., wheels=[0., 0.]))
    p = packet()
    io.command_context = {k: v for k, v in p.items() if k not in ('kind', 'session_id', 'request_id')}
    monkeypatch.setattr(runner, 'time', NS(monotonic=lambda: 10.02))
    started = threading.Event(); completed = threading.Event(); threads = []
    def request(kind, **values):
        assert kind == 'command' and values['command'] == p['command']
        def callback():
            started.set(); io.receiver_status(NS(data=json.dumps(status())))
            completed.set()
        t = threading.Thread(target=callback); threads.append(t); t.start(); assert started.wait(1.)
        assert not completed.wait(.02)  # Callback may not reject a send not yet recorded.
        io._record_actuator_submission(packet(), 10.01)
    io.actuator = NS(request=request)
    io.command(.1, 0.)
    for t in threads: t.join(1.)
    assert completed.is_set() and not io.receiver_evidence.error


@pytest.mark.parametrize('kind', ['velocity', 'deadline', 'sequence'])
def test_runner_rejects_coherent_wrong_feedback_and_keeps_failure_evidence(tmp_path, harness, monkeypatch, kind):
    control, _, _ = harness; fake_io = runner.GazeboIO; objects = []
    class IO(fake_io):
        command = real_command
        def __init__(self, **kw):
            super().__init__(**kw); self.receiver_evidence = bound(); self.session = kw['session_id']
            self.receiver_evidence.session = self.session
            push(self.receiver_evidence, status(session_id=self.session, wall_s=10., command_seq=0,
                reason='disarmed', deadline_steady_s=0., wheels=[0., 0.]))
            self.actuator = Mock(); self.actuator.status = dict(command_seq=0)
            self.actuator.request.side_effect = self.request; self.request_id = 0; objects.append(self)
        def request(self, kind, **values):
            self.request_id += 1
            p = dict(kind=kind, session_id=self.session, request_id=self.request_id, **values)
            self.receiver_evidence.record_submission(p, 10.)
        def next_frame(self, previous):
            result = super().next_frame(previous)
            if control.calls:
                p = self.receiver_evidence.submissions[0]['packet']; c = p['command']
                item = status(session_id=self.session, wall_s=10., command_seq=c['seq'],
                    wheels=[c['v_m_s'], c['omega_rad_s']], deadline_steady_s=p['expires_steady_s'])
                if kind == 'velocity': item['wheels'] = [.14, .05]
                elif kind == 'deadline': item['deadline_steady_s'] -= .01
                else: item['command_seq'] += 500
                push(self.receiver_evidence, item)
            return result
    monkeypatch.setattr(runner, 'GazeboIO', IO)
    monkeypatch.setattr(actuator, 'summarize_audit', lambda *a, **kw: {})
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=3)
    assert not result['completed'] and control.state == 'aborted' and 'submission' in result['exception']
    failure = result['failure_record']
    assert failure['pending_tick']['tick'] == 1 and failure['pending_tick']['stage'] == 'command_submission'
    assert failure['committed_decision_count'] == 1 and failure['zero_command_submitted']
    assert [c.args[0] for c in objects[0].actuator.request.call_args_list] == ['command', 'stop', 'stop']
    assert result['cleanup_stop']['steps_completed'] == 10 and result['receiver_interlock']['fault_latched']
    assert len(json.loads((tmp_path/'capture/audit/receiver-submissions.json').read_text())) == 3


real_command = runner.GazeboIO.command
