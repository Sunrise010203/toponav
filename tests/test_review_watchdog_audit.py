"""Real blocked OS pipe and writer-process faults, not physical braking tests."""
import fcntl
import json
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import time

import pytest

from tests.test_review_gazebo_watchdog import packet, SESSION
from simulation.gazebo_actuator import AuditPipe, AUDIT_TIMEOUT_S, MAX_AUDIT_PENDING, summarize_audit

SCRIPT = '''import os,socket,sys,time,json
from simulation.gazebo_actuator import serve
with os.fdopen(int(sys.argv[2]),'w') as audit, open(sys.argv[3],'x',buffering=1) as out:
 def publish(v,w):
  out.write(json.dumps(dict(wall_s=time.monotonic(),v=v,w=w))+'\\n')
 serve(socket.socket(fileno=int(sys.argv[1])),sys.argv[4],publish,audit)
'''


def receive(channel, timeout=2.):
    assert select.select([channel], [], [], timeout)[0], 'Missing watchdog reply'
    return json.loads(channel.recv(16384))


def rows(path):
    return [json.loads(l) for l in path.read_text().splitlines()] if path.exists() else []


@pytest.fixture
def worker(tmp_path):
    parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    read_fd, write_fd = os.pipe()
    output = tmp_path/'publications.jsonl'
    process = subprocess.Popen([sys.executable, '-u', '-c', SCRIPT, str(child.fileno()),
        str(write_fd), str(output), SESSION], pass_fds=(child.fileno(), write_fd))
    child.close()
    info = receive(parent)
    assert info['kind'] == 'ready'
    state = dict(channel=parent, read_fd=read_fd, write_fd=write_fd,
                 output=output, process=process, ready=info)
    try:
        yield state
    finally:
        parent.close()
        # Stop only owned test processes; no native world is running here.
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
            if info.get('audit_pid'):
                try: os.kill(info['audit_pid'], signal.SIGKILL)
                except ProcessLookupError: pass
        os.close(state['read_fd']); os.close(write_fd)


def fill_audit_pipe(w):
    # Drain the ready record, then fill a REAL blocking pipe exactly. Do not
    # set O_NONBLOCK: that flag would also alter the writer's inherited FD.
    assert select.select([w['read_fd']], [], [], 2.)[0]
    os.read(w['read_fd'], 65536)
    capacity = fcntl.fcntl(w['write_fd'], fcntl.F_GETPIPE_SZ)
    assert os.write(w['write_fd'], b'x'*capacity) == capacity
    return capacity


def send_motion(w):
    p = packet(issued=time.monotonic())
    w['channel'].send(json.dumps(p).encode())
    return p


def first_stop_after_motion(w, timeout=1.4):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        samples = rows(w['output'])
        moving = next((i for i,r in enumerate(samples) if r['v']), None)
        if moving is not None:
            zero = next((r for r in samples[moving+1:] if r['v'] == r['w'] == 0.), None)
            if zero: return zero
        time.sleep(.01)
    pytest.fail('Blocked audit prevented independent zero publication')


def test_full_blocking_audit_pipe_cannot_block_lease(worker):
    fill_audit_pipe(worker)
    p = send_motion(worker)
    ack = receive(worker['channel'])
    assert ack['accepted']
    zero = first_stop_after_motion(worker)
    assert zero['wall_s'] < p['expires_steady_s']+.15
    # No new command/parent polling is needed to stop. A later fresh command
    # must not re-arm even if storage starts working again.
    os.read(worker['read_fd'], 65536)
    time.sleep(.05)
    fresh = packet(2, 2, .2, time.monotonic())
    worker['channel'].send(json.dumps(fresh).encode())
    refusal = receive(worker['channel'])
    assert not refusal['accepted']
    assert refusal['status']['reason'] == 'audit_failure'
    assert refusal['status']['wheels'] == [0., 0.]


@pytest.mark.parametrize('fault', [signal.SIGSTOP, signal.SIGKILL])
def test_stopped_or_dead_audit_writer_latches_zero(worker, fault):
    send_motion(worker)
    assert receive(worker['channel'])['accepted']
    os.kill(worker['ready']['audit_pid'], fault)
    # Submit a fresh bound command so a stopped writer has pending work.
    time.sleep(.03)
    p = packet(2, 2, .2, time.monotonic())
    worker['channel'].send(json.dumps(p).encode())
    receive(worker['channel'])
    zero = first_stop_after_motion(worker)
    assert zero['wall_s'] < p['expires_steady_s']+.15
    time.sleep(.5)
    worker['channel'].send(json.dumps(dict(kind='stop',session_id=SESSION,request_id=3)).encode())
    final = receive(worker['channel'])
    assert final['status']['reason'] == 'audit_failure'
    assert final['status']['wheels'] == [0., 0.]
    assert not final['accepted']


def test_broken_audit_pipe_fails_closed(worker):
    os.close(worker['read_fd'])
    # Retain a harmless FD for fixture cleanup, without retaining a reader.
    worker['read_fd'] = os.open(os.devnull, os.O_RDONLY)
    p = send_motion(worker)
    receive(worker['channel'])
    assert first_stop_after_motion(worker)['wall_s'] < p['expires_steady_s']+.15
    time.sleep(.1)
    worker['channel'].send(json.dumps(dict(kind='stop',session_id=SESSION,request_id=2)).encode())
    assert receive(worker['channel'])['status']['reason'] == 'audit_failure'


@pytest.fixture
def mailbox():
    from collections import deque
    from types import SimpleNamespace
    obj = AuditPipe.__new__(AuditPipe)
    obj.channel, peer = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    obj.channel.setblocking(False)
    obj.pending = deque()
    obj.sequence = 0
    obj.failed = False
    obj.process = SimpleNamespace(poll=lambda: None)
    try:
        yield obj, peer
    finally:
        obj.channel.close(); peer.close()


def test_mailbox_is_nonblocking_and_ack_only_retires_bound_record(mailbox):
    obj, peer = mailbox
    assert obj.submit({'event': 'test'})
    sent = json.loads(peer.recv(16384))
    assert sent == dict(id=1, record={'event': 'test'})
    assert len(obj.pending) == 1
    peer.send(b'1')
    assert obj.poll() and not obj.pending
    assert obj.channel.getblocking() is False


@pytest.mark.parametrize('ack', [b'2', b'invalid', b''])
def test_invalid_or_closed_ack_latches_audit_failure(mailbox, ack):
    obj, peer = mailbox
    assert obj.submit({'event':'test'})
    if ack: peer.send(ack)
    else: peer.shutdown(socket.SHUT_WR)
    assert not obj.poll() and obj.failed
    assert not obj.submit({'event': 'late'})


def test_full_mailbox_fails_instead_of_unbounded_backlog(mailbox):
    obj, _ = mailbox
    for i in range(MAX_AUDIT_PENDING):
        assert obj.submit({'index': i})
    assert not obj.submit({'index': MAX_AUDIT_PENDING})
    assert len(obj.pending) == MAX_AUDIT_PENDING


def test_unacknowledged_flush_has_its_own_bounded_budget(mailbox):
    obj, _ = mailbox
    assert obj.submit({'event':'test'})
    seq, stamp = obj.pending.popleft()
    obj.pending.append((seq, stamp-AUDIT_TIMEOUT_S))
    assert not obj.poll()


def audit_record(event, **changes):
    return dict(event=event, reason='remote_stop', latched=True, wheels=[0., 0.],
                wall_s=10., publish_success=True, **changes)


def test_complete_zero_audit_is_not_physical_stop_confirmation(tmp_path):
    path = tmp_path/'audit.jsonl'
    path.write_text('\n'.join(json.dumps(audit_record(e)) for e in ('ready','worker_exit'))+'\n')
    result = summarize_audit(path, 123)
    assert result['audit_complete'] and not result['fault_latched']
    assert 'stationary' not in result


@pytest.mark.parametrize('corruption', ['missing', 'empty', 'prefix', 'torn', 'invalid_json',
    'missing_ready', 'not_zero', 'not_latched', 'publish_failure', 'fault', 'bad_schema', 'bad_boolean'])
def test_incomplete_or_failed_log_cannot_be_promoted_to_success(tmp_path, corruption):
    path = tmp_path/'audit.jsonl'
    entries = [audit_record('ready'), audit_record('worker_exit')]
    if corruption == 'prefix': entries.pop()
    elif corruption == 'missing_ready': entries.pop(0)
    elif corruption == 'not_zero': entries[-1]['wheels'] = [.1, 0.]
    elif corruption == 'not_latched': entries[-1]['latched'] = False
    elif corruption == 'publish_failure': entries[-1]['publish_success'] = False
    elif corruption == 'fault': entries[-1]['reason'] = 'audit_failure'
    elif corruption == 'bad_schema': del entries[-1]['wheels']
    elif corruption == 'bad_boolean': entries[-1]['publish_success'] = 1
    data = '\n'.join(json.dumps(e) for e in entries)+'\n'
    if corruption == 'empty': data = ''
    elif corruption == 'torn': data = data[:-10]
    elif corruption == 'invalid_json': data += '{not-json}\n'
    if corruption != 'missing': path.write_text(data)
    result = summarize_audit(path, 123)
    assert result['fault_latched']
    assert not result['audit_complete'] or corruption in ('publish_failure','fault')
