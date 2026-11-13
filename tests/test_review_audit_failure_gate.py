"""Diagnostic evidence may only veto completion, never replace audit/IPC/pose proof."""
import errno
import json
import os
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from simulation import gazebo_actuator as actuator
from simulation import gazebo_cross_validation as runner
from tests.test_review_actuator_history import history, terminal
from tests.test_review_gazebo_failure_capture import harness
from tests.test_review_gazebo_post_stop import terminal_io

REASONS = ('writer_exited', 'flush_ack_timeout', 'ack_channel_closed', 'invalid_ack',
           'ack_receive_error', 'mailbox_full', 'record_too_large', 'partial_record_send',
           'record_send_would_block', 'record_send_error')


def cause(**changes):
    return dict(dict(version=1, reason='flush_ack_timeout', pending=1, errno=None), **changes)


def fixture_log(tmp_path):
    path = tmp_path/'actuator.jsonl'; rows = history()
    path.write_text(''.join(json.dumps(x)+'\n' for x in rows))
    return path, path.with_name(path.name+'.failure.json'), terminal(rows)


def summary(path, status):
    return actuator.summarize_audit(path, 123, expected_status=status)


def test_absent_sidecar_does_not_change_original_complete_bound_audit(tmp_path):
    path, _, status = fixture_log(tmp_path); result = summary(path, status)
    assert result['audit_complete'] and result['terminal_status_bound'] and not result['fault_latched']
    assert result['failure_diagnostic'] == dict(status='absent', failure=None, blocks_completion=False)
    assert 'stationary' not in result


@pytest.mark.parametrize('reason', REASONS)
def test_each_real_failure_reason_vetoes_even_complete_bound_safe_history(tmp_path, reason):
    path, sidecar, status = fixture_log(tmp_path); value = cause(reason=reason)
    sidecar.write_text(json.dumps(value))
    result = summary(path, status)
    assert not result['audit_complete'] and result['fault_latched'] and 'sidecar' in result['read_error']
    assert result['terminal_status_bound']  # Structural binding fact is not completion.
    assert result['failure_diagnostic'] == dict(status='present', failure=value, blocks_completion=True)


@pytest.mark.parametrize('raw', [b'', b'{}', b'[]', b'null', b'{bad', b'\xff',
    b'{"version":1,"version":1,"reason":"mailbox_full","pending":64,"errno":null}',
    *[json.dumps(cause(**change)).encode() for change in [
      dict(version=True), dict(version=1.), dict(version=2), dict(reason='healthy'), dict(reason=[]),
      dict(pending=True), dict(pending=-1), dict(pending=65), dict(pending=1.),
      dict(errno=True), dict(errno=-1), dict(errno=2**31), dict(errno='bad'), dict(extra=1)]],
    b'{"version":1,"reason":"mailbox_full","pending":64}'])
def test_malformed_sidecar_can_never_restore_completion(tmp_path, raw):
    path, sidecar, status = fixture_log(tmp_path); sidecar.write_bytes(raw)
    result = summary(path, status)
    assert not result['audit_complete'] and result['fault_latched']
    assert result['failure_diagnostic'] == dict(status='invalid', failure=None, blocks_completion=True)


@pytest.mark.parametrize('pending,code', [(0, 0), (64, 2**31-1)])
def test_fixed_schema_numeric_boundaries_are_accepted_only_as_failure(tmp_path, pending, code):
    path, sidecar, status = fixture_log(tmp_path); sidecar.write_text(json.dumps(cause(pending=pending, errno=code)))
    result = summary(path, status)
    assert result['failure_diagnostic']['status'] == 'present' and result['fault_latched']


def test_oversize_diagnostic_has_a_fixed_read_budget_and_is_not_copied(tmp_path, monkeypatch):
    path, sidecar, status = fixture_log(tmp_path); sidecar.write_bytes(b'x'*2_000_000)
    original = os.read; requests = []
    def read(fd, size): requests.append(size); return original(fd, size)
    monkeypatch.setattr(actuator.os, 'read', read)
    result = summary(path, status)
    assert actuator.MAX_AUDIT_FAILURE_BYTES == 1024 and requests == [1025]
    assert result['failure_diagnostic']['status'] == 'invalid' and result['fault_latched']
    assert len(json.dumps(result['failure_diagnostic'])) < 100


@pytest.mark.parametrize('target', ['existing', 'dangling'])
def test_symlinks_are_not_followed_and_always_veto(tmp_path, target, monkeypatch):
    path, sidecar, status = fixture_log(tmp_path); destination = tmp_path/'target.json'
    if target == 'existing': destination.write_text(json.dumps(cause()))
    sidecar.symlink_to(destination)
    read = Mock(side_effect=AssertionError('Read through symlink'))
    monkeypatch.setattr(actuator.os, 'read', read)
    result = summary(path, status)
    assert result['failure_diagnostic']['status'] == 'unreadable' and result['fault_latched']
    read.assert_not_called()


@pytest.mark.parametrize('kind', ['fifo', 'directory'])
def test_special_paths_are_opened_nonblocking_not_read_and_fd_is_closed(tmp_path, kind, monkeypatch):
    path, sidecar, status = fixture_log(tmp_path)
    if kind == 'fifo': os.mkfifo(sidecar)
    else: sidecar.mkdir()
    opened = []; original = os.open
    def open_file(name, flags):
        assert flags & os.O_NONBLOCK and flags & os.O_NOFOLLOW
        fd = original(name, flags); opened.append(fd); return fd
    monkeypatch.setattr(actuator.os, 'open', open_file)
    read = Mock(side_effect=AssertionError('Read from non-regular diagnostic'))
    monkeypatch.setattr(actuator.os, 'read', read)
    result = summary(path, status)
    assert result['failure_diagnostic']['status'] == 'invalid' and result['fault_latched']
    assert len(opened) == 1; read.assert_not_called()
    with pytest.raises(OSError): os.fstat(opened[0])


@pytest.mark.parametrize('operation', ['open', 'read', 'fstat'])
def test_io_failures_are_unreadable_not_absent_or_success(tmp_path, monkeypatch, operation):
    path, sidecar, status = fixture_log(tmp_path); sidecar.write_text(json.dumps(cause()))
    monkeypatch.setattr(actuator.os, operation, Mock(side_effect=OSError(errno.EACCES, 'x'*100_000)))
    result = summary(path, status)
    assert result['failure_diagnostic']['status'] == 'unreadable' and result['fault_latched']
    assert len(json.dumps(result['failure_diagnostic'])) < 100


def test_original_primary_audit_error_is_not_replaced_by_secondary_diagnostic(tmp_path):
    path, sidecar, status = fixture_log(tmp_path); path.write_text('{bad\n'); sidecar.write_text(json.dumps(cause()))
    result = summary(path, status)
    assert result['read_error'].startswith('JSONDecodeError:') and 'sidecar' not in result['read_error']
    assert result['failure_diagnostic']['status'] == 'present' and result['fault_latched']


def test_bad_original_ipc_binding_cannot_be_fixed_by_sidecar_absence(tmp_path):
    path, _, status = fixture_log(tmp_path); status['command_seq'] += 1
    result = summary(path, status)
    assert result['fault_latched'] and not result['terminal_status_bound']
    assert result['failure_diagnostic']['status'] == 'absent'


def test_unbound_legacy_log_is_also_vetoed_by_present_failure(tmp_path):
    path, sidecar, _ = fixture_log(tmp_path); sidecar.write_text(json.dumps(cause()))
    result = actuator.summarize_audit(path, 123)
    assert not result['terminal_status_binding_requested'] and not result['audit_complete'] and result['fault_latched']


@pytest.mark.parametrize('with_failure', [False, True])
def test_real_runner_completion_uses_summary_veto_even_with_fresh_stationary_terminal_pair(tmp_path, monkeypatch, harness, with_failure):
    terminal_io(monkeypatch, harness); original = runner.GazeboIO; rows = history()
    class IO(original):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            path = kwargs['actuator_audit']; path.write_text(''.join(json.dumps(x)+'\n' for x in rows))
            if with_failure: path.with_name(path.name+'.failure.json').write_text(json.dumps(cause()))
            self.actuator = NS(process=NS(pid=1234), status=terminal(rows), close=lambda: None)
    monkeypatch.setattr(runner, 'GazeboIO', IO)
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=1)
    assert result['audit']['normal_geometric_stop_pass'] and result['audit']['stationary']
    assert result['audit']['post_stop_pose_pair_fresh'] and result['controller_result']['state'] == 'visual_stop'
    assert result['exception'] is None and result['completed'] is (not with_failure)
    assert result['actuator_watchdog']['fault_latched'] is with_failure
