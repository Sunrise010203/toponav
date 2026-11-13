"""Local ACK binding and sticky failure; not trusted physical feedback."""
import copy
from types import SimpleNamespace

import pytest

from simulation import gazebo_actuator as actuator
from tests.test_review_gazebo_watchdog import client, packet, send_motion, wait_rows


def disarmed():
    return dict(latched=False, reason='disarmed', wheels=[0., 0.],
                command_seq=0, deadline_steady_s=None)


def exchange():
    obj = actuator.ActuatorClient.__new__(actuator.ActuatorClient)
    obj.session = 'watchdog-test'; obj.status = disarmed(); obj.last_publication_s = None
    request = packet()
    reply = dict(kind='ack', request_id=1, accepted=True, published_steady_s=10.1,
        status=dict(latched=False, reason='accepted', wheels=[.1, 0.], command_seq=1, deadline_steady_s=10.8))
    return obj, request, reply


def test_matching_ack_does_not_claim_physical_stop():
    obj, request, reply = exchange()
    obj._validate_ack(reply, request, 10., 10.2)
    assert 'stationary' not in reply and 'physical_stop' not in reply
    assert obj.status == disarmed()  # Validation itself commits nothing.


@pytest.mark.parametrize('field,value', [
    ('request_id', True), ('request_id', 1.), ('request_id', 2), ('kind', 'ready'),
    ('accepted', 1), ('accepted', 'true'), ('published_steady_s', True),
    ('published_steady_s', float('nan')), ('published_steady_s', float('inf')),
    ('published_steady_s', 9.99), ('published_steady_s', 10.21),
    ('status', None), ('status', []), ('status', {}),
])
def test_malformed_ack_is_not_an_acceptance(field, value):
    obj, request, reply = exchange(); reply[field] = value
    with pytest.raises((ValueError, RuntimeError)):
        obj._validate_ack(reply, request, 10., 10.2)
    assert obj.status == disarmed()


@pytest.mark.parametrize('field', ['kind', 'request_id', 'accepted', 'published_steady_s', 'status'])
def test_missing_ack_field_fails(field):
    obj, request, reply = exchange(); del reply[field]
    with pytest.raises(RuntimeError): obj._validate_ack(reply, request, 10., 10.2)


def test_extra_ack_field_fails():
    obj, request, reply = exchange(); reply['extra'] = 1
    with pytest.raises(RuntimeError): obj._validate_ack(reply, request, 10., 10.2)


@pytest.mark.parametrize('field,value', [
    ('command_seq', True), ('command_seq', 0), ('command_seq', 2),
    ('deadline_steady_s', None), ('deadline_steady_s', 10.7), ('deadline_steady_s', 11.),
    ('wheels', [.09, 0.]), ('wheels', [.1, .1]), ('wheels', [.151, 0.]),
    ('wheels', [True, 0.]), ('reason', 'disarmed'), ('latched', True),
])
def test_ack_must_confirm_exact_submitted_command(field, value):
    obj, request, reply = exchange(); reply['status'][field] = value
    with pytest.raises((ValueError, RuntimeError)):
        obj._validate_ack(reply, request, 10., 10.2)


def test_valid_but_wrong_zero_latch_cannot_confirm_nonzero_command():
    obj, request, reply = exchange()
    reply['status'].update(latched=True, reason='remote_stop', wheels=[0., 0.])
    with pytest.raises(RuntimeError, match='differs from submitted command'):
        obj._validate_ack(reply, request, 10., 10.2)


@pytest.mark.parametrize('received', [10.8, 10.9])
def test_lease_is_not_renewed_from_ack_receipt(received):
    obj, request, reply = exchange()
    with pytest.raises(RuntimeError, match='arrived after command deadline'):
        obj._validate_ack(reply, request, 10., received)


@pytest.mark.parametrize('published,received', [(10., 10.), (10., 10.2), (10.2, 10.2)])
def test_inclusive_exchange_timestamp_boundaries(published, received):
    obj, request, reply = exchange(); reply['published_steady_s'] = published
    obj._validate_ack(reply, request, 10., received)


def test_publication_timestamp_cannot_regress():
    obj, request, reply = exchange(); obj.last_publication_s = 10.15
    with pytest.raises(RuntimeError): obj._validate_ack(reply, request, 10., 10.2)


@pytest.mark.parametrize('state', ['observing', 'moving', 'waiting_feedback', 'visual_stop', 'aborted'])
def test_zero_commands_confirm_exact_terminal_semantics(state):
    obj, request, reply = exchange()
    request['command'].update(state=state, v_m_s=0.)
    reason = {'visual_stop': 'remote_visual_stop', 'aborted': 'remote_abort'}.get(state, 'accepted')
    reply['status'].update(reason=reason, latched=reason != 'accepted', wheels=[0., 0.])
    obj._validate_ack(reply, request, 10., 10.2)
    if reason != 'accepted': obj._validate_ack(reply, request, 10., 10.9)


@pytest.mark.parametrize('change', ['extra', 'session', 'seq', 'issued', 'expires', 'sim', 'capture', 'already_latched'])
def test_accepted_ack_cannot_bless_invalid_request_authority(change):
    obj, request, reply = exchange()
    if change == 'extra': request['extra'] = 1
    elif change == 'session': request['command']['session_id'] = 'other'
    elif change == 'seq': request['command']['seq'] = 2
    elif change == 'issued': request['issued_steady_s'] = 10.2
    elif change == 'expires': request['expires_steady_s'] = 11.
    elif change == 'sim': request['sim_s'] = .01
    elif change == 'capture': request['command']['captured_sim_s'] = -.1
    elif change == 'already_latched': obj.status.update(latched=True, reason='remote_stop')
    with pytest.raises((ValueError, RuntimeError)):
        obj._validate_ack(reply, request, 10., 10.2)


def test_fractional_rounding_uses_original_clamped_deadline():
    obj, request, reply = exchange()
    request['expires_steady_s'] = 10.8+5e-10
    obj._validate_ack(reply, request, 10., 10.2)
    reply['status']['deadline_steady_s'] = request['expires_steady_s']
    with pytest.raises(RuntimeError): obj._validate_ack(reply, request, 10., 10.2)


@pytest.mark.parametrize('kind', ['stop', 'shutdown'])
@pytest.mark.parametrize('reason', ['remote_stop', 'command_lease_expired', 'audit_failure'])
def test_zero_stop_ack_may_report_a_real_fault_without_granting_motion(kind, reason):
    obj, _, reply = exchange()
    obj.status = copy.deepcopy(reply['status'])
    reply['status'].update(latched=True, reason=reason, wheels=[0., 0.])
    request = dict(kind=kind, request_id=1, session_id=obj.session)
    obj._validate_ack(reply, request, 10., 11.)


@pytest.mark.parametrize('change', ['seq', 'deadline', 'unlatched', 'reason_after_latch', 'extra'])
def test_stop_ack_must_preserve_existing_authority(change):
    obj, _, reply = exchange(); obj.status = copy.deepcopy(reply['status'])
    reply['status'].update(latched=True, reason='remote_stop', wheels=[0., 0.])
    request = dict(kind='stop', request_id=1, session_id=obj.session)
    if change == 'seq': reply['status']['command_seq'] = 2
    elif change == 'deadline': reply['status']['deadline_steady_s'] = 10.9
    elif change == 'unlatched': reply['status'] = copy.deepcopy(obj.status)
    elif change == 'reason_after_latch': obj.status.update(latched=True, reason='remote_abort', wheels=[0., 0.])
    elif change == 'extra': request['extra'] = 1
    with pytest.raises(RuntimeError): obj._validate_ack(reply, request, 10., 10.2)


def test_idle_cannot_replace_or_create_authority():
    obj, _, reply = exchange(); active = copy.deepcopy(reply['status'])
    request = dict(kind='idle', request_id=1, session_id=obj.session)
    with pytest.raises(RuntimeError): obj._validate_ack(reply, request, 10., 10.2)
    reply['status'] = disarmed(); obj._validate_ack(reply, request, 10., 10.2)
    obj.status = active
    with pytest.raises(RuntimeError): obj._validate_ack(reply, request, 10., 10.2)


@pytest.mark.parametrize('change', ['extra', 'pid_bool', 'wrong_pid', 'audit_bool', 'audit_zero',
                                   'same_pid', 'no_status', 'active_status', 'latched_status'])
def test_invalid_ready_handshake_reaps_only_just_created_worker(tmp_path, monkeypatch, change):
    operations = []
    process = SimpleNamespace(pid=1234, kill=lambda: operations.append('kill'),
                              wait=lambda timeout: operations.append(('wait', timeout)))
    ready = dict(kind='ready', pid=1234, audit_pid=5678, status=disarmed())
    if change == 'extra': ready['extra'] = 1
    elif change == 'pid_bool': ready['pid'] = True
    elif change == 'wrong_pid': ready['pid'] = 4321
    elif change == 'audit_bool': ready['audit_pid'] = True
    elif change == 'audit_zero': ready['audit_pid'] = 0
    elif change == 'same_pid': ready['audit_pid'] = 1234
    elif change == 'no_status': del ready['status']
    elif change == 'active_status': ready['status'] = exchange()[2]['status']
    elif change == 'latched_status': ready['status'].update(latched=True, reason='publisher_failure')
    monkeypatch.setattr(actuator.subprocess, 'Popen', lambda *a, **k: process)
    monkeypatch.setattr(actuator.ActuatorClient, '_receive', lambda *a: ready)
    with pytest.raises((ValueError, RuntimeError)):
        actuator.ActuatorClient('ack-test', tmp_path/'audit.jsonl')
    assert operations == ['kill', ('wait', 5)]


@pytest.mark.parametrize('corruption', ['sequence', 'target', 'deadline', 'false_stop', 'future_stamp', 'missing_stamp'])
def test_real_worker_bad_ack_closes_channel_latches_zero_and_cannot_recover(client, corruption, monkeypatch):
    obj, publications, audit = client
    original = obj._receive; before = copy.deepcopy(obj.status); captured = {}
    def receive(timeout):
        raw = original(timeout); captured['raw'] = copy.deepcopy(raw); reply = copy.deepcopy(raw)
        if corruption == 'sequence': reply['status']['command_seq'] += 1
        elif corruption == 'target': reply['status']['wheels'][0] = .09
        elif corruption == 'deadline': reply['status']['deadline_steady_s'] -= .01
        elif corruption == 'false_stop': reply['status'].update(latched=True, reason='remote_stop', wheels=[0., 0.])
        elif corruption == 'future_stamp': reply['published_steady_s'] += 10.
        elif corruption == 'missing_stamp': del reply['published_steady_s']
        return reply
    monkeypatch.setattr(obj, '_receive', receive)
    with pytest.raises((ValueError, RuntimeError)): send_motion(client)
    assert captured['raw']['accepted'] and captured['raw']['status']['command_seq'] == 1
    assert obj.failed and obj.channel.fileno() == -1 and obj.status == before
    event = wait_rows(audit, lambda rows: next((e for e in rows if e['reason'] == 'peer_closed'), None))
    assert event['wheels'] == [0., 0.]
    assert event['wall_s'] < captured['raw']['status']['deadline_steady_s']
    rows = wait_rows(publications, lambda rows: rows if any(r['v'] for r in rows) and rows[-1]['v'] == rows[-1]['w'] == 0 else None)
    assert any(r['v'] == .1 for r in rows)
    with pytest.raises(RuntimeError, match='prior failure'): obj.request('stop')
    assert obj.request_id == 1  # No recovery request/heartbeat was sent.
