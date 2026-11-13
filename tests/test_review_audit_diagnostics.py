"""First-cause diagnostics do not weaken audit rejection or lease deadlines."""
import errno
import json
import socket
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from simulation import gazebo_actuator as actuator
from tests.test_review_watchdog_audit import mailbox


def failure(obj, reason, pending, error_number=None):
    assert obj.failed
    assert obj.failure == dict(version=1, reason=reason, pending=pending, errno=error_number)
    assert len(json.dumps(obj.failure)) < 160
    before = dict(obj.failure)
    assert not obj.poll() and not obj.submit({'late': 'x'*100_000})
    assert obj.failure == before


def test_healthy_mailbox_remains_nonblocking_with_no_failure(mailbox):
    obj, peer = mailbox
    assert obj.submit({'event': 'ready'})
    peer.recv(16384); peer.send(b'1')
    assert obj.poll() and not obj.pending and not getattr(obj, 'failure', None)
    assert obj.channel.getblocking() is False
    assert actuator.MAX_AUDIT_PENDING == 64 and actuator.AUDIT_TIMEOUT_S == .4 and actuator.LEASE_S == .8


def test_full_original_mailbox_reports_capacity_without_growing_or_retrying(mailbox):
    obj, _ = mailbox
    for i in range(64): assert obj.submit({'index': i})
    before = list(obj.pending)
    assert not obj.submit({'index': 64})
    assert obj.sequence == 64 and list(obj.pending) == before
    failure(obj, 'mailbox_full', 64)


@pytest.mark.parametrize('raw,reason', [(b'2', 'invalid_ack'), (b'bad', 'invalid_ack'), (b'', 'ack_channel_closed')])
def test_ack_faults_are_distinguished_without_retiring_pending_record(mailbox, raw, reason):
    obj, peer = mailbox
    assert obj.submit({'event': 'idle'}); peer.recv(16384)
    if raw: peer.send(raw)
    else: peer.shutdown(socket.SHUT_WR)
    before = list(obj.pending)
    assert not obj.poll() and list(obj.pending) == before
    failure(obj, reason, 1)


def test_unsolicited_ack_is_invalid_without_granting_a_record(mailbox):
    obj, peer = mailbox; peer.send(b'1')
    assert not obj.poll() and obj.sequence == 0
    failure(obj, 'invalid_ack', 0)


@pytest.mark.parametrize('at,healthy', [(.399999, True), (.4, False), (.400001, False)])
def test_original_inclusive_audit_timeout_boundary_is_unchanged(mailbox, monkeypatch, at, healthy):
    obj, _ = mailbox; obj.pending.append((1, 0.))
    monkeypatch.setattr(actuator.time, 'monotonic', lambda: at)
    assert obj.poll() is healthy
    if not healthy: failure(obj, 'flush_ack_timeout', 1)


def test_process_exit_precedes_timeout_and_closed_socket_like_original(mailbox, monkeypatch):
    obj, peer = mailbox; obj.pending.append((1, 0.)); peer.shutdown(socket.SHUT_WR)
    obj.process = NS(poll=lambda: 7)
    monkeypatch.setattr(actuator.time, 'monotonic', lambda: .5)
    assert not obj.poll()
    failure(obj, 'writer_exited', 1)


def test_real_nonblocking_socket_backpressure_has_a_distinct_reason(mailbox):
    obj, _ = mailbox
    obj.channel.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
    filled = 0
    while True:
        try: obj.channel.send(b'x'*1024); filled += 1
        except BlockingIOError: break
    assert 0 < filled < 64
    assert not obj.submit({'event': 'idle'}) and obj.sequence == 0 and not obj.pending
    failure(obj, 'record_send_would_block', 0, errno.EAGAIN)


@pytest.mark.parametrize('operation,reason,code', [
    ('recv', 'ack_receive_error', errno.ECONNRESET), ('send', 'record_send_error', errno.EPIPE)])
def test_other_socket_errors_keep_bounded_errno_without_exception_text(mailbox, monkeypatch, operation, reason, code):
    obj, _ = mailbox
    channel = Mock()
    channel.recv.side_effect = BlockingIOError()
    getattr(channel, operation).side_effect = OSError(code, 'x'*1_000_000)
    monkeypatch.setattr(obj, 'channel', channel)
    assert not obj.submit({'event': 'idle'}) and obj.sequence == 0
    failure(obj, reason, 0, code)


def test_partial_record_send_is_not_committed_or_retried(mailbox, monkeypatch):
    obj, _ = mailbox; channel = Mock()
    channel.recv.side_effect = BlockingIOError(); channel.send.return_value = 1
    monkeypatch.setattr(obj, 'channel', channel)
    assert not obj.submit({'event': 'idle'}) and obj.sequence == 0 and not obj.pending
    failure(obj, 'partial_record_send', 0)
    assert channel.send.call_count == 1


def test_oversized_record_is_not_stored_in_diagnostic_or_sent(mailbox):
    obj, peer = mailbox
    assert not obj.submit({'raw': 'x'*actuator.MAX_PACKET})
    failure(obj, 'record_too_large', 0)
    peer.setblocking(False)
    with pytest.raises(BlockingIOError): peer.recv(16384)


@pytest.mark.parametrize('error_number', [None, -1, True, 2**40, 'bad'])
def test_invalid_or_unbounded_errno_is_not_copied(mailbox, error_number):
    obj, _ = mailbox; assert obj._fail('record_send_error', error_number) is False
    failure(obj, 'record_send_error', 0)


def test_first_cause_is_not_replaced_by_later_shutdown_or_writer_exit(mailbox):
    obj, _ = mailbox; obj._fail('mailbox_full')
    obj._fail('writer_exited', errno.EIO)
    failure(obj, 'mailbox_full', 0)


def test_failure_capture_does_not_perform_socket_or_storage_io(mailbox, monkeypatch):
    obj, _ = mailbox; channel = Mock(); monkeypatch.setattr(obj, 'channel', channel)
    monkeypatch.setattr(Path, 'open', Mock(side_effect=AssertionError('Diagnostic disk I/O in active loop')))
    obj._fail('flush_ack_timeout')
    channel.send.assert_not_called(); channel.recv.assert_not_called()
    assert obj.failed and obj.failure['reason'] == 'flush_ack_timeout'


@pytest.mark.parametrize('diagnostic', [None, dict(version=1, reason='mailbox_full', pending=64, errno=None)])
def test_native_entry_writes_sidecar_only_after_serve_returns_and_never_rewrites(tmp_path, monkeypatch, diagnostic):
    order = []; audit = tmp_path/'actuator.jsonl'
    class Node:
        def advertise(self, *_): return NS(has_connections=lambda: False)
    monkeypatch.setitem(sys.modules, 'gz.transport13', NS(Node=Node))
    monkeypatch.setitem(sys.modules, 'gz.msgs10.twist_pb2', NS(Twist=object))
    monkeypatch.setattr(actuator.socket, 'socket', Mock(return_value=NS()))
    def serve(*args):
        order.append('serve_return_after_original_finally')
        assert not audit.with_name(audit.name+'.failure.json').exists()
        return diagnostic
    monkeypatch.setattr(actuator, 'serve', serve)
    original = Path.open
    def open_file(path, *args, **kwargs):
        mode = args[0] if args else kwargs.get('mode', 'r')
        if path.name.endswith('.failure.json') and mode != 'r':
            assert order == ['serve_return_after_original_finally']
            assert mode == 'x'
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', open_file)
    assert actuator.main(['--fd', '0', '--session', 'test', '--audit', str(audit)]) == 0
    sidecar = audit.with_name(audit.name+'.failure.json')
    assert sidecar.exists() is (diagnostic is not None)
    if diagnostic: assert json.loads(sidecar.read_text()) == diagnostic
    with pytest.raises(FileExistsError): actuator.main(['--fd', '0', '--session', 'test', '--audit', str(audit)])
