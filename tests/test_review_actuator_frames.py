"""Finite Python worker replay history; synthetic packets/transport, not braking proof."""
import copy
import json
from pathlib import Path
import socket
import subprocess
import sys
import time
from unittest.mock import Mock

import pytest

from simulation import gazebo_actuator as act
from tests.test_review_gazebo_watchdog import SESSION, packet

LIMIT = 16384
REASON = 'actuator_capacity_exceeded'


def command(seq, state='moving', width=None):
    p = packet(request_id=seq, seq=seq, stamp=seq*.001, issued=10.+seq*.0001)
    p['command']['state'] = state
    if state != 'moving': p['command']['v_m_s'] = 0.
    if width is not None: p['command']['frame_id'] = str(seq).ljust(width, 'x')
    return p


def filled(count=LIMIT, state='moving', width=None):
    g = act.LeaseGuard(SESSION)
    for seq in range(1, count+1):
        p = command(seq, state, width)
        assert g.handle(p, p['issued_steady_s'])
    return g


def metadata(g):
    return dict(seq=g.gate.seq, deadline=g.gate.hold_deadline, pending=g.gate.expected,
                pending_deadline=g.gate.deadline, captured=g.captured, frame=g.frame_id,
                issued=g.issued, frames=set(g.frames))


def assert_stopped(g, reason, before):
    assert g.gate.latched and g.gate.reason == reason and g.gate.wheels == (0., 0.)
    assert metadata(g) == before
    assert g.publication()['kind'] == 'stop'
    assert g.publication()['command_seq'] == str(before['seq'])
    assert g.publication()['frame_id'] == before['frame']
    assert g.publication()['expires_steady_s'] == repr(before['deadline'] if before['deadline'] is not None else -1.)


@pytest.mark.parametrize('count', [0, 1, 2200, LIMIT-1, LIMIT])
@pytest.mark.parametrize('state', ['moving', 'observing'])
def test_exact_budget_is_usable_without_proactive_latch(count, state):
    assert act.MAX_ACTUATOR_FRAMES == LIMIT
    g = filled(count, state)
    assert len(g.frames) == count and not g.gate.latched and g.gate.seq == count
    assert g.gate.wheels == ((.1, 0.) if count and state == 'moving' else (0., 0.))


@pytest.mark.parametrize('state', ['moving', 'observing', 'waiting_feedback', 'visual_stop', 'aborted'])
@pytest.mark.parametrize('width', [None, 256])
def test_full_history_rejects_all_new_command_evidence_but_latches_zero(state, width):
    g = filled(width=width); before = metadata(g)
    # Any arm/accept would already change pending or accepted authority.
    g.gate.arm = Mock(side_effect=AssertionError('must not arm at capacity'))
    g.gate.accept = Mock(side_effect=AssertionError('must not accept at capacity'))
    p = command(LIMIT+1, state, width)
    assert not g.handle(p, p['issued_steady_s'])
    assert_stopped(g, REASON, before)
    g.gate.arm.assert_not_called(); g.gate.accept.assert_not_called()
    for seq, kind in [(LIMIT+2, 'command'), (LIMIT+3, 'stop'), (LIMIT+4, 'shutdown')]:
        p = command(seq, width=width) if kind == 'command' else dict(kind=kind, session_id=SESSION, request_id=seq)
        assert g.handle(p, 10.+LIMIT*.0001+.1) == (kind != 'command')
        assert_stopped(g, REASON, before)


@pytest.mark.parametrize('state', ['moving', 'observing', 'waiting_feedback', 'visual_stop', 'aborted'])
def test_last_slot_preserves_original_terminal_and_zero_command_semantics(state):
    g = filled(LIMIT-1); p = command(LIMIT, state)
    assert g.handle(p, p['issued_steady_s'])
    assert len(g.frames) == LIMIT and g.gate.seq == LIMIT and g.frame_id == f'f{LIMIT}'
    expected = {'visual_stop': 'remote_visual_stop', 'aborted': 'remote_abort'}.get(state, 'accepted')
    assert g.gate.reason == expected and g.gate.latched == (expected != 'accepted')
    assert g.gate.wheels == ((.1, 0.) if state == 'moving' else (0., 0.))
    assert g.gate.hold_deadline == p['expires_steady_s']


@pytest.mark.parametrize('kind', ['stop', 'shutdown'])
@pytest.mark.parametrize('count', [0, 1, 2200, LIMIT])
def test_frame_free_stop_and_shutdown_do_not_consume_capacity_or_replace_metadata(kind, count):
    g = filled(count); before = metadata(g)
    assert g.handle(dict(kind=kind, session_id=SESSION, request_id=count+1), 10.+count*.0001+.01)
    assert_stopped(g, 'remote_stop', before)


@pytest.mark.parametrize('action,reason', [
    ('first_replay', 'invalid_actuator_packet'), ('middle_replay', 'invalid_actuator_packet'),
    ('same_capture', 'invalid_actuator_packet'), ('bad_session', 'invalid_actuator_packet'),
    ('bad_request', 'invalid_actuator_packet'), ('extra_field', 'invalid_actuator_packet'),
    ('bad_speed', 'invalid_actuator_packet'), ('bad_frame', 'invalid_actuator_packet'),
    ('expired', 'invalid_actuator_packet'), ('future', 'invalid_actuator_packet'),
    ('extended', 'invalid_actuator_packet'), ('idle', 'invalid_actuator_packet'),
    ('clock_regression', 'steady_clock_regression'), ('lease_expired', 'command_lease_expired'),
    ('at_expiry', 'command_lease_expired'), ('prior_error', 'audit_failure'),
])
def test_existing_pre_capacity_validation_and_first_fault_win(action, reason):
    g = filled(); before = metadata(g); p = command(LIMIT+1); now = p['issued_steady_s']
    if action == 'first_replay': p['command']['frame_id'] = 'f1'
    if action == 'middle_replay': p['command']['frame_id'] = 'f2048'
    if action == 'same_capture': p['command']['captured_sim_s'] = g.captured
    if action == 'bad_session': p['session_id'] = 'other'
    if action == 'bad_request': p['request_id'] += 1
    if action == 'extra_field': p['extra'] = True
    if action == 'bad_speed': p['command']['v_m_s'] = .151
    if action == 'bad_frame': p['command']['frame_id'] = 'x'*257
    if action == 'expired': p['expires_steady_s'] = now
    if action == 'future': p['issued_steady_s'] = now+.01
    if action == 'extended': p['expires_steady_s'] += .001
    if action == 'idle': p = dict(kind='idle', session_id=SESSION, request_id=LIMIT+1)
    if action == 'clock_regression': now = 10.
    if action == 'lease_expired': now = g.gate.hold_deadline+.001
    if action == 'at_expiry': now = g.gate.hold_deadline
    if action == 'prior_error': g.gate.trip('audit_failure')
    assert not g.handle(p, now)
    assert_stopped(g, reason, before)


@pytest.mark.parametrize('action', ['wrong_command_session', 'bad_seq', 'bad_sim_clock', 'stale'])
def test_no_lower_level_gate_failure_can_create_authority_at_capacity(action):
    g = filled(); before = metadata(g); p = command(LIMIT+1)
    if action == 'wrong_command_session': p['command']['session_id'] = 'other'
    if action == 'bad_seq': p['command']['seq'] += 1
    if action == 'bad_sim_clock': p['sim_s'] = 0.
    if action == 'stale': p['sim_s'] += 1.
    assert not g.handle(p, p['issued_steady_s'])
    # Capacity deliberately runs before MotorGate.arm/accept; no claim that
    # this error proves the rest of an envelope was valid.
    assert_stopped(g, REASON, before)


WORKER = r'''
import json,socket,sys
from pathlib import Path
from simulation import gazebo_actuator as act
fd,log,pub,proof=sys.argv[1:]
Original=act.LeaseGuard
snapshots=[]
class Guard(Original):
 def handle(self,p,now):
  ok=super().handle(p,now)
  if ok and p['kind']=='command' and not self.gate.latched:
   # DECLARED synthetic replay history only; real command/meta/ACK unchanged.
   self.frames.update('diagnostic-history-'+str(i) for i in range(act.MAX_ACTUATOR_FRAMES-len(self.frames)))
  snapshots.append(dict(request_id=p['request_id'],count=len(self.frames),status=self.status(),metadata=self.publication()))
  return ok
act.LeaseGuard=Guard
context={}
def capture(meta):context.clear();context.update(meta)
with Path(log).open('x') as audit,Path(pub).open('x',buffering=1) as output:
 def publish(v,w):output.write(json.dumps(dict(v=v,w=w,metadata=context.copy()))+'\n')
 failure=act.serve(socket.socket(fileno=int(fd)),'watchdog-test',publish,audit,publication_context=capture)
Path(proof).write_text(json.dumps(dict(snapshots=snapshots,failure=failure)))
'''


@pytest.fixture
def ipc(tmp_path):
    parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    log, pub, proof = [tmp_path/n for n in ('audit.jsonl', 'publications.jsonl', 'proof.json')]
    process = subprocess.Popen([sys.executable, '-u', '-c', WORKER, str(child.fileno()), str(log), str(pub), str(proof)], pass_fds=(child.fileno(),))
    child.close(); parent.settimeout(8.)
    client = act.ActuatorClient.__new__(act.ActuatorClient)
    client.session=SESSION; client.request_id=0; client.failed=False
    client.channel=parent; client.process=process; client.last_publication_s=None
    ready = act.decode(parent.recv(act.MAX_PACKET).decode()); client.status=ready['status']
    assert ready['kind'] == 'ready' and ready['pid'] == process.pid
    def exchange(p):
        sent=time.monotonic(); parent.send(act.encode(p).encode())
        reply=act.decode(parent.recv(act.MAX_PACKET).decode()); received=time.monotonic()
        client._validate_ack(reply,p,sent,received)
        client.status=reply['status']; client.last_publication_s=reply['published_steady_s']
        return reply
    try: yield client,exchange,log,pub,proof
    finally:
        parent.close()
        try: process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait(); raise
        assert process.returncode == 0


@pytest.mark.parametrize('state', ['moving', 'observing', 'visual_stop', 'aborted'])
def test_real_worker_rejection_ack_zero_metadata_stop_shutdown_and_strict_audit(ipc, state):
    client, exchange, log, pub, proof = ipc
    first=packet(issued=time.monotonic()); accepted=exchange(first)
    before=copy.deepcopy(accepted['status'])
    assert accepted['accepted'] and before['wheels'] == [.1, 0.]
    p=packet(2, 2, .2, time.monotonic()); p['command']['state']=state
    if state != 'moving': p['command']['v_m_s']=0.
    rejected=exchange(p)
    assert not rejected['accepted'] and rejected['status']==dict(before,latched=True,reason=REASON,wheels=[0.,0.])
    for request_id,kind in [(3,'stop'), (4,'shutdown')]:
        reply=exchange(dict(kind=kind,session_id=SESSION,request_id=request_id))
        assert reply['accepted'] and reply['status']==rejected['status']
    client.process.wait(timeout=5.)
    evidence=json.loads(proof.read_text())
    assert evidence['failure'] is None
    assert [s['count'] for s in evidence['snapshots']] == [LIMIT]*4
    metas=[s['metadata'] for s in evidence['snapshots']]
    assert all(dict(m,kind='stop')==metas[-1] for m in metas)
    rows=[json.loads(line) for line in pub.read_text().splitlines()]
    pivot=next(i for i,r in enumerate(rows) if r['v'])
    assert all(r['v']==r['w']==0 and r['metadata']['command_seq']=='1' for r in rows[pivot+1:])
    result=act.summarize_audit(log,client.process.pid,expected_status=reply['status'])
    assert result['audit_complete'] and result['terminal_status_bound'] and result['fault_latched']
    assert result['final']['reason']==REASON and result['failure_diagnostic']['status']=='absent'
    assert 'stationary' not in result and 'physical_stop' not in result


def test_actual_client_rejection_is_sticky_closes_ipc_and_worker_reaps(ipc):
    client, _, log, pub, proof = ipc
    p=packet(issued=time.monotonic())
    client.request('command',**{k:v for k,v in p.items() if k not in ('kind','session_id','request_id')})
    p=packet(2,2,.2,time.monotonic())
    with pytest.raises(RuntimeError,match=REASON):
        client.request('command',**{k:v for k,v in p.items() if k not in ('kind','session_id','request_id')})
    assert client.failed and client.channel.fileno()==-1
    with pytest.raises(RuntimeError,match='prior failure'): client.request('stop')
    client.close()
    result=act.summarize_audit(log,client.process.pid,expected_status=client.status)
    assert result['audit_complete'] and result['terminal_status_bound'] and result['fault_latched']
    evidence=json.loads(proof.read_text()); assert evidence['failure'] is None
    assert [s['count'] for s in evidence['snapshots']]==[LIMIT,LIMIT]


def test_worker_policy_is_fixed_not_world_or_cli_configuration():
    from simulation.gazebo_receiver import MAX_RECEIVER_CANDIDATES, MAX_RECEIVER_FRAMES
    root=Path(__file__).resolve().parents[1]
    assert 'args.max_ticks = 2200' in (root/'simulation/gazebo_cross_validation.py').read_text()
    assert act.MAX_ACTUATOR_FRAMES == LIMIT
    assert LIMIT > MAX_RECEIVER_CANDIDATES+1 > MAX_RECEIVER_FRAMES == 4096
    assert not hasattr(act.MotorGate(SESSION), 'frames')
