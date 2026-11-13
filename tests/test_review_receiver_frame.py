"""Accepted native metadata echo: no image/identity/physical feedback claim."""
import json
from pathlib import Path
import subprocess

import pytest

from simulation.gazebo_receiver import ReceiverEvidence, SOURCES
from tests.test_review_receiver_binding import sent, status, push
from tests.test_review_receiver_freshness import adapter
from tests.test_review_gazebo_watchdog import SESSION, packet

ROOT = Path(__file__).resolve().parents[1]
FIELDS = ('frame_id_hex', 'captured_sim_s', 'issued_steady_s')


def strict():
    return ReceiverEvidence(SESSION, bind_submissions=True, require_frame_echo=True)


def echo(**changes):
    item = status(version=2, frame_id_hex='6631', captured_sim_s=.1, issued_steady_s=10.)
    item.update(changes)
    return item


def disarmed(**changes):
    return echo(command_seq=0, deadline_steady_s=0., reason='disarmed', wheels=[0., 0.],
                frame_id_hex=None, captured_sim_s=None, issued_steady_s=None, **changes)


def test_echo_is_mandatory_only_in_actual_production_constructor():
    # No runtime/CLI switch or fallback to v1 in GazeboIO.
    import ast
    tree = ast.parse((ROOT/'simulation/gazebo_cross_validation.py').read_text())
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Name) and n.func.id == 'ReceiverEvidence']
    assert len(calls) == 1
    assert {k.arg: ast.literal_eval(k.value) for k in calls[0].keywords} == {
        'bind_submissions': True, 'require_frame_echo': True}
    with pytest.raises(ValueError, match='binding'):
        ReceiverEvidence(SESSION, require_frame_echo=True)


@pytest.mark.parametrize('initial', [True, False])
def test_production_refuses_v1_even_if_old_binding_would_accept_it(initial):
    item = status(command_seq=0, deadline_steady_s=0., reason='disarmed', wheels=[0., 0.]) if initial else status()
    old = ReceiverEvidence(SESSION, bind_submissions=True); e = strict()
    if not initial: sent(old); sent(e)
    push(old, item); push(e, item)
    assert old.error is None and e.error and 'downgrade' in e.error and not e.events
    error = e.error
    push(e, disarmed() if initial else echo())
    assert e.error == error
    with pytest.raises(RuntimeError): e.require_fresh(10.2)


@pytest.mark.parametrize('field,value', [
    ('frame_id_hex', '6632'), ('frame_id_hex', '663100'),
    ('captured_sim_s', .10000000000000002), ('captured_sim_s', 0.),
    ('issued_steady_s', 10.01), ('issued_steady_s', 9.999999999999998)])
@pytest.mark.parametrize('latched', [False, True])
def test_internally_coherent_wrong_frame_capture_or_issue_is_rejected(field, value, latched):
    # For earlier issued time, retain a coherent <= .8 lease, but it is not the SENT lease.
    item = echo(**{field: value}, latched=latched, reason='remote_stop' if latched else 'accepted',
                wheels=[0., 0.] if latched else [.1, 0.])
    if field == 'issued_steady_s' and value < 10.:
        item['deadline_steady_s'] = value+.8
    legacy = ReceiverEvidence(SESSION); push(legacy, item)
    assert not legacy.error
    e, _ = sent(strict()); push(e, item)
    assert e.error and 'submission' in e.error and not e.events
    error = e.error; push(e, echo(wall_s=10.2, sim_s=.2))
    assert e.error == error
    with pytest.raises(RuntimeError): e.require_fresh(10.3)


@pytest.mark.parametrize('field,value', [
    ('frame_id_hex', None), ('frame_id_hex', ''), ('frame_id_hex', 'a'),
    ('frame_id_hex', 'GG'), ('frame_id_hex', 'AF'), ('frame_id_hex', '00 '*3),
    ('frame_id_hex', '66'*257), ('frame_id_hex', 6631), ('frame_id_hex', ['66']),
    ('captured_sim_s', None), ('captured_sim_s', True), ('captured_sim_s', -1.),
    ('captured_sim_s', '0.1'), ('captured_sim_s', float('inf')),
    ('issued_steady_s', None), ('issued_steady_s', True), ('issued_steady_s', -1.),
    ('issued_steady_s', 10.2), ('issued_steady_s', 9.), ('issued_steady_s', float('nan'))])
def test_malformed_v2_metadata_never_enters_history(field, value):
    e, _ = sent(strict()); push(e, echo(**{field: value}))
    assert e.error and not e.events


@pytest.mark.parametrize('field', FIELDS)
@pytest.mark.parametrize('mode', ['missing', 'extra_in_v1', 'non_null_seq0'])
def test_exact_schema_and_uncommanded_nulls(field, mode):
    e = strict(); item = echo()
    if mode == 'missing': del item[field]
    elif mode == 'extra_in_v1': item = status(); item[field] = '66' if field == FIELDS[0] else 0.
    else: item = disarmed(); item[field] = '66' if field == FIELDS[0] else 0.
    push(e, item)
    assert e.error and not e.events


@pytest.mark.parametrize('reason', ['disarmed', 'remote_stop', 'invalid_receiver_envelope', 'steady_clock_regression'])
def test_uncommanded_nulls_do_not_claim_a_verified_frame(reason):
    e = strict(); item = disarmed(); item.update(reason=reason, latched=reason != 'disarmed')
    push(e, item); summary = e.summary(0., .1)
    assert not e.error and summary['submission_binding']['frame_echo_required']
    assert not summary['submission_binding']['frame_echo_verified']


@pytest.mark.parametrize('reason', ['accepted', 'remote_stop', 'command_lease_expired',
                                    'invalid_receiver_envelope', 'steady_clock_regression'])
def test_echo_retained_when_latched_never_proves_physical_motion_or_clears_fault(reason):
    e, p = sent(strict())
    push(e, echo(reason=reason, latched=reason != 'accepted',
                 wheels=[.1, 0.] if reason == 'accepted' else [0., 0.],
                 wall_s=10.8 if reason == 'command_lease_expired' else 10.1))
    summary = e.summary(0., .1)
    assert not e.error and summary['submission_binding']['frame_echo_verified']
    assert summary['fault_latched'] == (reason not in ('accepted', 'remote_stop'))
    assert 'stationary' not in summary
    assert 'NOT image content or source authentication' in summary['submission_binding']['scope']


@pytest.mark.parametrize('state', ['visual_stop', 'aborted'])
def test_terminal_command_metadata_cannot_replace_prior_accepted_frame(state):
    e, _ = sent(strict()); p = packet(2, 2, .2, 10.1)
    p['command'].update(state=state, v_m_s=0., omega_rad_s=0.)
    e.record_submission(p, 10.11)
    push(e, echo(wall_s=10.2, reason='remote_stop', latched=True, wheels=[0., 0.]))
    assert not e.error and e.summary(0., .1)['submission_binding']['frame_echo_verified']
    other, _ = sent(strict()); other.record_submission(p, 10.11)
    push(other, echo(wall_s=10.2, reason='remote_stop', latched=True, wheels=[0., 0.],
                     frame_id_hex=p['command']['frame_id'].encode().hex(), captured_sim_s=.2,
                     issued_steady_s=10.1))
    assert other.error


@pytest.mark.parametrize('field,value', [('frame_id_hex', '6632'), ('captured_sim_s', .2), ('issued_steady_s', 10.01)])
def test_offline_v2_same_sequence_cannot_change_echo(field, value):
    e = ReceiverEvidence(SESSION); push(e, echo())
    push(e, echo(wall_s=10.2, sim_s=.2, **{field: value}))
    assert e.error and 'regressed' in e.error and len(e.events) == 1


def test_offline_upgrade_is_allowed_but_downgrade_is_not():
    e = ReceiverEvidence(SESSION); push(e, status()); push(e, echo(wall_s=10.2, sim_s=.2))
    assert not e.error
    push(e, status(wall_s=10.3, sim_s=.3))
    assert e.error and len(e.events) == 2


@pytest.mark.parametrize('captured', [.1, 0.])
def test_offline_native_capture_must_increase_with_sequence(captured):
    e = ReceiverEvidence(SESSION); push(e, echo())
    push(e, echo(command_seq=2, frame_id_hex='6632', captured_sim_s=captured, wall_s=10.2, sim_s=.2))
    assert e.error


def test_known_lagged_echo_can_be_fresh_but_cannot_certify_latest_final():
    e, _ = sent(strict()); sent(e, request_id=2, seq=2, stamp=.2, issued=10.05)
    push(e, echo()); e.require_fresh(10.2)
    result = e.summary(0., .1)
    assert not result['submission_binding']['frame_echo_verified'] and result['fault_latched']
    push(e, echo(command_seq=2, frame_id_hex='6632', captured_sim_s=.2, issued_steady_s=10.05,
                 deadline_steady_s=10.05+.8, wall_s=10.3, sim_s=.3))
    assert not e.error and e.summary(.2, .3)['submission_binding']['frame_echo_verified']


@pytest.mark.parametrize('frame', ['a', 'x'*256, '帧-α-🙂', '"\\\n\r\t\0'])
def test_frame_encoding_preserves_exact_utf8_bytes_and_caller_alias_isolation(frame):
    e = strict(); p = packet(); p['command']['frame_id'] = frame
    e.record_submission(p, 10.01); p['command']['frame_id'] = 'changed'
    push(e, echo(frame_id_hex=frame.encode('utf-8').hex()))
    assert not e.error and e.summary(0., .1)['submission_binding']['frame_echo_verified']


def test_missing_echo_blocks_real_adapter_nonzero_but_zero_stays_available(monkeypatch):
    io = adapter(monkeypatch, None, now=10.2)
    io.receiver_evidence, _ = sent(strict()); push(io.receiver_evidence, status())
    with pytest.raises(RuntimeError): io.command(.1, 0.)
    io.actuator.request.assert_not_called()
    io.command(0., 0.)
    io.actuator.request.assert_called_once_with('stop')
    assert io.receiver_evidence.error


@pytest.fixture(scope='module')
def native(tmp_path_factory):
    folder = tmp_path_factory.mktemp('receiver-frame-native')
    source = folder/'probe.cc'; binary = folder/'probe'
    source.write_text(r'''
#include "simulation/native/receiver_status.hh"
#include <iostream>
#include <locale>
struct Comma : std::numpunct<char> { char do_decimal_point() const override { return ','; } };
int main(int argc, char **argv) {
  std::locale::global(std::locale(std::locale::classic(), new Comma));
  const std::string mode=argv[1], hex=argv[2];
  std::string frame;
  for (std::size_t i=0; i<hex.size(); i+=2)
    frame += static_cast<char>(std::stoul(hex.substr(i,2), nullptr, 16));
  toponav::ReceiverLease guard("test");
  auto emit=[&](double now) { std::cout << toponav::ReceiverStatus(guard,"test",now,.2) << '\n'; };
  emit(10.);
  toponav::Envelope p;
  p.session="test"; p.kind="command"; p.seq=1; p.frame=frame;
  p.captured=.10000000000000002; p.issued=10.; p.expires=10.8;
  p.v=.1; p.w=.05;
  guard.Accept(p,10.01); emit(10.1);
  if (mode=="stop") { p.kind="stop"; p.seq=500; p.frame="not-accepted";
    p.captured=500; p.issued=500; p.expires=501; p.v=p.w=0; guard.Accept(p,10.2); }
  if (mode=="expiry") guard.Poll(10.8);
  if (mode=="clock") guard.Poll(9.);
  if (mode=="reject") { p.seq=5; p.frame="not-accepted"; p.captured=.2; guard.Accept(p,10.2); }
  if (mode=="duplicate") { p.seq=2; p.captured=.2; guard.Accept(p,10.2); }
  if (mode=="same_capture") { p.seq=2; p.frame="next"; guard.Accept(p,10.2); }
  if (mode=="next") { p.seq=2; p.frame="next"; p.captured=.2; p.issued=10.1;
    p.expires=10.9; guard.Accept(p,10.2); }
  emit(mode=="expiry"?10.8:10.2);
  return argc==3?0:1;
}
''')
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', str(ROOT),
                    str(source), '-o', str(binary)], check=True, capture_output=True)
    def run(mode='normal', frame=b'f1'):
        result = subprocess.run([str(binary), mode, frame.hex()], check=True, capture_output=True, text=True)
        return [json.loads(line) for line in result.stdout.splitlines()]
    return run


@pytest.mark.parametrize('frame', [b'a', b'x'*256, bytes(range(256)), '帧-🙂'.encode(), b'"\\\n\r\t\0'])
def test_actual_cpp_status_roundtrips_all_accepted_bytes_and_exact_numbers(native, frame):
    initial, accepted, final = native(frame=frame)
    assert initial['version'] == 2 and all(initial[n] is None for n in FIELDS)
    assert accepted['frame_id_hex'] == frame.hex() and accepted['command_seq'] == 1
    assert accepted['captured_sim_s'] == .10000000000000002
    assert accepted['issued_steady_s'] == 10. and accepted['deadline_steady_s'] == 10.8
    assert final['frame_id_hex'] == frame.hex()
    e = ReceiverEvidence('test')
    for item in (initial, accepted, final): push(e, item)
    assert not e.error


@pytest.mark.parametrize('mode,reason', [('stop', 'remote_stop'), ('expiry', 'command_lease_expired'),
    ('clock', 'steady_clock_regression'), ('reject', 'invalid_receiver_envelope'),
    ('duplicate', 'invalid_receiver_envelope'), ('same_capture', 'invalid_receiver_envelope')])
def test_actual_cpp_stop_expiry_fault_and_rejection_preserve_last_accepted_metadata(native, mode, reason):
    _, accepted, final = native(mode)
    assert final['latched'] and final['reason'] == reason and final['wheels'] == [0., 0.]
    assert all(final[n] == accepted[n] for n in (*FIELDS, 'command_seq', 'deadline_steady_s'))
    e = ReceiverEvidence('test')
    push(e, accepted); push(e, final)
    assert not e.error


@pytest.mark.parametrize('frame', [b'', b'x'*257])
def test_actual_cpp_rejected_initial_frame_cannot_leak_into_echo(native, frame):
    _, rejected, final = native(frame=frame)
    assert rejected['latched'] and rejected['command_seq'] == final['command_seq'] == 0
    assert all(rejected[n] is None and final[n] is None for n in FIELDS)


def test_actual_cpp_next_accepted_command_updates_all_echo_fields_atomically(native):
    _, accepted, final = native('next')
    assert final['command_seq'] == 2 and final['frame_id_hex'] == b'next'.hex()
    assert final['captured_sim_s'] == .2 and final['issued_steady_s'] == 10.1
    assert not final['latched'] and final['deadline_steady_s'] == 10.9
    e = ReceiverEvidence('test'); push(e, accepted); push(e, final)
    assert not e.error


def test_actual_cpp_echo_binds_to_exact_sent_packet_without_float_tolerance(native):
    _, accepted, final = native('stop', '帧-🙂'.encode())
    e = ReceiverEvidence('test', bind_submissions=True, require_frame_echo=True)
    p = packet(stamp=.10000000000000002)
    p['session_id'] = p['command']['session_id'] = 'test'
    p['command'].update(frame_id='帧-🙂', omega_rad_s=.05)
    e.record_submission(p, 10.005)
    push(e, accepted); push(e, final)
    assert not e.error and e.summary(0., .2)['submission_binding']['frame_echo_verified']


def test_matching_known_metadata_is_not_a_publisher_authentication_test():
    # No source credential exists in this protocol: an identical fabricated
    # payload is indistinguishable. Do not sell equality as trusted identity.
    e, _ = sent(strict()); fabricated = echo()
    push(e, fabricated)
    assert not e.error and e.summary(0., .1)['submission_binding']['frame_echo_verified']
    assert 'NOT image content or source authentication' in e.summary(0., .1)['submission_binding']['scope']


def test_serializer_header_is_in_both_build_provenance_and_run_freeze():
    assert 'simulation/native/receiver_status.hh' in SOURCES
    assert "'simulation/native/receiver_status.hh'" in (ROOT/'tools/build_gazebo_receiver.sh').read_text()
    assert '*(ROOT/n for n in SOURCES)' in (ROOT/'simulation/gazebo_cross_validation.py').read_text()
