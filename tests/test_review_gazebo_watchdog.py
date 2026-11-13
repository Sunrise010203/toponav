"""Independent lease policy + real process/socket tests; not braking certification."""
import importlib
import json
from pathlib import Path
import socket
import subprocess
import sys
import time

import pytest

from ros2_ws.src.toponav_bridge.toponav_bridge.protocol import command_from_decision

SESSION = 'watchdog-test'


def module():
    return importlib.import_module('simulation.gazebo_actuator')


def packet(request_id=1, seq=1, stamp=.1, issued=10., **changes):
    c = command_from_decision(SESSION, seq, dict(state='moving', v_m_s=.1, omega_rad_s=0.,
        control_frame_id=f'f{seq}', captured_sim_s=stamp, decision_sim_s=stamp))
    result = dict(kind='command', session_id=SESSION, request_id=request_id, command=c,
                  sim_s=stamp, issued_steady_s=issued, expires_steady_s=issued+.8)
    result.update(changes)
    return result


def moving():
    guard = module().LeaseGuard(SESSION)
    assert guard.handle(packet(), 10.)
    assert guard.gate.wheels == (.1, 0.)
    return guard


def test_expiry_is_absolute_and_latches_at_original_boundary():
    guard = moving()
    assert guard.poll(10.799) == (.1, 0.)
    assert guard.poll(10.8) == (0., 0.)
    assert guard.gate.reason == 'command_lease_expired'
    assert not guard.handle(packet(2, 2, .2, 10.8), 10.8)
    assert guard.poll(11.) == (0., 0.)


def test_fresh_bound_frame_can_renew_before_expiry_not_after():
    guard = moving()
    assert guard.handle(packet(2, 2, .2, 10.5), 10.55)
    assert guard.gate.hold_deadline == 11.3  # Not receipt time + .8 = 11.35.
    assert guard.poll(11.3) == (0., 0.)


@pytest.mark.parametrize('field,value', [
    ('issued_steady_s', 11.), ('issued_steady_s', True), ('issued_steady_s', float('nan')),
    ('expires_steady_s', 11.4), ('expires_steady_s', 10.), ('expires_steady_s', float('inf')),
    ('sim_s', False), ('sim_s', .01), ('request_id', 2), ('request_id', True),
    ('session_id', 'other'), ('kind', 'heartbeat'), ('extra', 1),
])
def test_invalid_envelope_fails_closed_without_renewal(field, value):
    guard = module().LeaseGuard(SESSION)
    assert not guard.handle(packet(**{field: value}), 10.)
    assert guard.gate.latched and guard.gate.wheels == (0., 0.)


@pytest.mark.parametrize('corruption', ['frame_id', 'capture', 'sequence', 'velocity', 'state', 'session'])
def test_bound_command_cannot_refresh_with_bad_or_reused_evidence(corruption):
    guard = moving()
    p = packet(2, 2, .2, 10.2)
    if corruption == 'frame_id': p['command']['frame_id'] = 'f1'
    elif corruption == 'capture': p['command']['captured_sim_s'] = .1
    elif corruption == 'sequence': p['command']['seq'] = 1
    elif corruption == 'velocity': p['command']['v_m_s'] = .2
    elif corruption == 'state': p['command']['state'] = 'observing'
    else: p['command']['session_id'] = 'other'
    assert not guard.handle(p, 10.2)
    assert guard.gate.latched and guard.gate.wheels == (0., 0.)


def test_transport_delay_consumes_budget_instead_of_creating_new_lease():
    guard = module().LeaseGuard(SESSION)
    assert not guard.handle(packet(), 10.8)
    assert guard.gate.wheels == (0., 0.)


def test_idle_does_not_create_or_renew_authority():
    guard = module().LeaseGuard(SESSION)
    assert guard.handle(dict(kind='idle', request_id=1, session_id=SESSION), 9.)
    assert guard.gate.hold_deadline is None
    assert guard.handle(packet(request_id=2), 10.)
    assert not guard.handle(dict(kind='idle', request_id=3, session_id=SESSION), 10.1)
    assert guard.gate.latched and guard.gate.wheels == (0., 0.)


def test_emergency_zero_needs_no_new_frame_and_cannot_rearm():
    guard = moving()
    assert guard.handle(dict(kind='stop', request_id=2, session_id=SESSION), 10.1)
    assert guard.gate.wheels == (0., 0.) and guard.gate.latched
    assert not guard.handle(packet(3, 2, .2, 10.2), 10.2)


def test_clock_regression_fails_closed():
    guard = moving()
    assert guard.poll(9.9) == (0., 0.)
    assert guard.gate.reason == 'steady_clock_regression'


@pytest.fixture
def client(tmp_path, monkeypatch):
    act = module()
    original = subprocess.Popen
    publications = tmp_path/'publications.jsonl'
    script = '''import json,os,socket,sys,time
from pathlib import Path
from simulation.gazebo_actuator import serve
fd=int(sys.argv[sys.argv.index('--fd')+1])
session=sys.argv[sys.argv.index('--session')+1]
audit=Path(sys.argv[sys.argv.index('--audit')+1])
with open(os.environ['TEST_WATCHDOG_PUBLICATIONS'],'x',buffering=1) as pub, audit.open('x') as log:
 def publish(v,w):
  pub.write(json.dumps(dict(wall_s=time.monotonic(),v=v,w=w))+'\\n')
 serve(socket.socket(fileno=fd),session,publish,log)
'''
    def spawn(args, **kwargs):
        return original([sys.executable, '-u', '-c', script, *args[4:]], **kwargs)
    monkeypatch.setenv('TEST_WATCHDOG_PUBLICATIONS', str(publications))
    monkeypatch.setattr(act.subprocess, 'Popen', spawn)
    obj = act.ActuatorClient(SESSION, tmp_path/'audit.jsonl')
    try:
        yield obj, publications, tmp_path/'audit.jsonl'
    finally:
        try: obj.close()
        except (OSError, RuntimeError): pass
        assert obj.process.poll() is not None


def send_motion(client):
    obj = client[0]
    p = packet(issued=time.monotonic())
    obj.request('command', **{k: v for k, v in p.items() if k not in ('kind','session_id','request_id')})
    return p['expires_steady_s']


def wait_rows(path, condition, timeout=2.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        rows = [json.loads(l) for l in path.read_text().splitlines()] if path.exists() else []
        result = condition(rows)
        if result: return result
        time.sleep(.01)
    raise AssertionError('Independent worker did not produce required evidence')


def test_worker_expires_without_any_parent_poll_or_heartbeat(client):
    obj, publications, audit = client
    deadline = send_motion(client)
    time.sleep(1.)  # Sender performs no gate polling, status request or renewal.
    event = wait_rows(audit, lambda rows: next((r for r in rows if r['reason']=='command_lease_expired'), None))
    assert deadline <= event['wall_s'] < deadline+.15  # Test scheduling allowance, NOT lease extension.
    assert event['wheels'] == [0., 0.]
    assert obj.process.pid != __import__('os').getpid()
    rows = [json.loads(l) for l in publications.read_text().splitlines()]
    moving_at = next(i for i, r in enumerate(rows) if r['v'])
    assert any(r['v'] == r['w'] == 0 for r in rows[moving_at+1:])
    p = packet(2, 2, .2, time.monotonic())
    with pytest.raises(RuntimeError, match='rejected'):
        obj.request('command', **{k: v for k, v in p.items() if k not in ('kind','session_id','request_id')})


def test_socket_eof_stops_worker_before_old_motion_lease_expires(client):
    obj, _, audit = client
    deadline = send_motion(client)
    obj.channel.shutdown(socket.SHUT_WR)
    event = wait_rows(audit, lambda rows: next((r for r in rows if r['reason']=='peer_closed'), None))
    assert event['wall_s'] < deadline and event['wheels'] == [0., 0.]


def test_client_ack_is_publication_not_physical_stop(client):
    obj, _, _ = client
    send_motion(client)
    answer = obj.request('stop')
    assert answer['accepted'] and answer['status']['latched']
    assert answer['status']['wheels'] == [0., 0.]
    assert 'stationary' not in answer and 'physical_stop' not in answer
