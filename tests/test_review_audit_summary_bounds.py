"""Post-exit audit input bounds may veto, never replace IPC/physical proof."""
import json
import os
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from simulation import gazebo_actuator as act
from simulation import gazebo_cross_validation as runner
from tests.test_review_actuator_history import history, terminal
from tests.test_review_gazebo_failure_capture import harness
from tests.test_review_gazebo_post_stop import terminal_io


def encoded(rows=None, separator='\n'):
    return (separator.join(json.dumps(e) for e in (history() if rows is None else rows))+'\n').encode()


def log(tmp_path, raw=None):
    path = tmp_path/'actuator.jsonl'; path.write_bytes(encoded() if raw is None else raw)
    return path


def summary(path, status=None):
    return act.summarize_audit(path, 123, expected_status=terminal(history()) if status is None else status)


def zero_rows(count):
    rows = [history()[0], *[{**history()[0], 'event': 'idle'} for _ in range(count-2)],
            {**history()[-1], 'command_seq': 0, 'deadline_steady_s': None}]
    return rows


@pytest.mark.parametrize('separator', ['\n', '\r\n', '\r', '\v', '\f', '\x85', '\u2028', '\u2029'])
def test_bounded_reader_preserves_existing_splitlines_semantics(tmp_path, separator):
    raw = encoded(separator=separator); result = summary(log(tmp_path, raw))
    assert result['audit_complete'] and result['terminal_status_bound'] and not result['fault_latched']
    assert result['audit_history']['retained'] == result['events'] == 6
    assert result['audit_history']['bytes_read'] == len(raw)


@pytest.mark.parametrize('delta', [-1, 0, 1])
def test_real_record_byte_boundary_includes_newline(tmp_path, delta):
    raw = encoded(); first = raw.index(b'\n')+1
    raw = b' '*(act.MAX_AUDIT_RECORD_BYTES+delta-first)+raw
    result = summary(log(tmp_path, raw))
    assert result['audit_complete'] is (delta <= 0)
    assert result['audit_history']['record_limit_exceeded'] is (delta > 0)
    assert result['fault_latched'] is (delta > 0)
    assert result['audit_history']['bytes_read'] <= act.MAX_AUDIT_BYTES+1


@pytest.mark.parametrize('delta', [-1, 0, 1])
def test_real_total_byte_budget_boundary(tmp_path, delta):
    rows = zero_rows(1100); lines = [encoded([e]) for e in rows]
    target = act.MAX_AUDIT_BYTES+delta; remaining = target-sum(map(len, lines))
    for index, line in enumerate(lines):
        pad = min(remaining, act.MAX_AUDIT_RECORD_BYTES-len(line))
        lines[index] = b' '*pad+line; remaining -= pad
    assert remaining == 0
    raw = b''.join(lines); assert len(raw) == target
    result = summary(log(tmp_path, raw), terminal(rows))
    assert result['audit_complete'] is (delta <= 0)
    assert result['audit_history']['byte_limit_exceeded'] is (delta > 0)
    assert result['audit_history']['bytes_read'] == target
    assert result['fault_latched'] is (delta > 0)


@pytest.mark.parametrize('delta', [-1, 0, 1])
def test_real_event_budget_boundary_retains_prefix_never_tail(tmp_path, delta):
    rows = zero_rows(act.MAX_AUDIT_EVENTS+delta); result = summary(log(tmp_path, encoded(rows)), terminal(rows))
    assert result['audit_complete'] is (delta <= 0)
    assert result['audit_history']['event_limit_exceeded'] is (delta > 0)
    assert result['events'] == min(len(rows), act.MAX_AUDIT_EVENTS)
    if delta > 0:
        assert not result['terminal_status_bound'] and result['final']['event'] == 'idle'
        assert result['fault_latched']


def test_omitted_event_is_not_json_parsed_even_in_one_legacy_lf_block(tmp_path, monkeypatch):
    monkeypatch.setattr(act, 'MAX_AUDIT_EVENTS', 2)
    raw = encoded([history()[0], history()[1]], separator='\r')[:-1]+b'\r{invalid omitted}\n'
    original = act.decode; calls = []
    def decode(value): calls.append(value); return original(value)
    monkeypatch.setattr(act, 'decode', decode)
    result = summary(log(tmp_path, raw))
    assert result['events'] == 2 and result['audit_history']['event_limit_exceeded']
    assert len(calls) == 2 and all('invalid omitted' not in x for x in calls)


@pytest.mark.parametrize('kind', ['long', 'unterminated', 'invalid-utf8'])
def test_oversized_input_is_rejected_before_utf8_or_json_and_fd_closed(tmp_path, monkeypatch, kind):
    raw = (b'\xff' if kind == 'invalid-utf8' else b' ')*2_000_000
    if kind == 'long': raw += b'{}\n'
    path = log(tmp_path, raw); original_open = os.open; original_read = os.read; reads = []; fds = []
    def open_file(name, flags):
        fd = original_open(name, flags)
        if name == path: fds.append(fd)
        return fd
    def read(fd, size):
        result = original_read(fd, size); reads.append((size, len(result))); return result
    monkeypatch.setattr(act.os, 'open', open_file); monkeypatch.setattr(act.os, 'read', read)
    decode = Mock(side_effect=AssertionError('Oversized input parsed')); monkeypatch.setattr(act, 'decode', decode)
    result = summary(path)
    assert result['audit_history']['record_limit_exceeded'] and result['events'] == 0
    assert sum(size for _, size in reads) == act.MAX_AUDIT_RECORD_BYTES+1
    assert max(size for size, _ in reads) <= 8192
    decode.assert_not_called()
    with pytest.raises(OSError): os.fstat(fds[0])


@pytest.mark.parametrize('delta', [-1, 0, 1])
def test_small_total_budget_counts_actual_reads_not_file_stat(tmp_path, monkeypatch, delta):
    raw = encoded(); monkeypatch.setattr(act, 'MAX_AUDIT_BYTES', len(raw)+delta)
    result = summary(log(tmp_path, raw))
    assert result['audit_complete'] is (delta >= 0)
    assert result['audit_history']['byte_limit_exceeded'] is (delta < 0)
    assert result['audit_history']['bytes_read'] <= act.MAX_AUDIT_BYTES+1


@pytest.mark.parametrize('mode', ['missing', 'empty', 'truncated', 'bad-json', 'utf8', 'unicode-in-string'])
def test_existing_bad_inputs_still_fail_closed(tmp_path, mode):
    raw = encoded()
    if mode == 'empty': raw = b''
    elif mode == 'truncated': raw = raw[:-1]
    elif mode == 'bad-json': raw = b'{bad}\n'+raw
    elif mode == 'utf8': raw = b'\xff\n'+raw
    elif mode == 'unicode-in-string': raw = raw.replace(b'disarmed', 'disarmed\u2028evil'.encode())
    path = log(tmp_path, raw)
    if mode == 'missing': path.unlink()
    result = summary(path)
    assert not result['audit_complete'] and not result['terminal_status_bound'] and result['fault_latched'] and result['read_error']


@pytest.mark.parametrize('kind', ['fifo', 'directory', 'symlink', 'dangling'])
def test_special_files_do_not_block_or_get_read(tmp_path, monkeypatch, kind):
    path = tmp_path/'actuator.jsonl'
    if kind == 'fifo': os.mkfifo(path)
    elif kind == 'directory': path.mkdir()
    else:
        target = tmp_path/'target'
        if kind == 'symlink': target.write_bytes(encoded())
        path.symlink_to(target)
    original = os.open
    def open_file(name, flags):
        if name == path: assert flags & os.O_NONBLOCK and flags & os.O_NOFOLLOW
        return original(name, flags)
    monkeypatch.setattr(act.os, 'open', open_file)
    read = Mock(side_effect=AssertionError('Read special file')); monkeypatch.setattr(act.os, 'read', read)
    result = summary(path)
    assert result['fault_latched'] and result['audit_history']['bytes_read'] == 0
    read.assert_not_called()


@pytest.mark.parametrize('operation', ['read', 'fstat'])
def test_io_errors_close_audit_descriptor(tmp_path, monkeypatch, operation):
    path = log(tmp_path); original = os.open; opened = []
    def open_file(name, flags):
        fd = original(name, flags)
        if name == path: opened.append(fd)
        return fd
    monkeypatch.setattr(act.os, 'open', open_file)
    with monkeypatch.context() as context:
        context.setattr(act.os, operation, Mock(side_effect=OSError('test')))
        result = summary(path)
    assert result['fault_latched'] and result['read_error'].startswith('OSError:')
    with pytest.raises(OSError): os.fstat(opened[0])


def test_json_error_explicitly_closes_generator_descriptor(tmp_path, monkeypatch):
    path = log(tmp_path, b'{bad}\n'); original = os.open; opened = []
    def open_file(name, flags):
        fd = original(name, flags)
        if name == path: opened.append(fd)
        return fd
    monkeypatch.setattr(act.os, 'open', open_file)
    result = summary(path); assert result['fault_latched']
    with pytest.raises(OSError): os.fstat(opened[0])


def test_capacity_error_retains_sidecar_diagnostic_without_replacing_primary_error(tmp_path):
    path = log(tmp_path, b' '*(act.MAX_AUDIT_RECORD_BYTES+1)+encoded())
    cause = dict(version=1, reason='flush_ack_timeout', pending=1, errno=None)
    path.with_name(path.name+'.failure.json').write_text(json.dumps(cause))
    result = summary(path)
    assert 'record limit' in result['read_error'] and result['failure_diagnostic']['failure'] == cause
    assert result['fault_latched'] and not result['terminal_status_bound']


@pytest.mark.parametrize('kind', ['invalid', 'unreadable'])
def test_invalid_or_unreadable_sidecar_vetoes_without_opening_main_audit(tmp_path, monkeypatch, kind):
    path = log(tmp_path, b'{bad original audit}\n')
    sidecar = path.with_name(path.name+'.failure.json')
    if kind == 'invalid': sidecar.write_bytes(b'{bad sidecar}')
    else: sidecar.symlink_to(tmp_path/'missing')
    reader = Mock(side_effect=AssertionError('Main audit read after diagnostic veto'))
    monkeypatch.setattr(act, '_audit_lines', reader)
    result = summary(path)
    reader.assert_not_called()
    assert result['failure_diagnostic']['status'] == kind and result['fault_latched']
    assert not result['audit_history']['read_attempted'] and result['audit_history']['bytes_read'] == result['events'] == 0
    assert not result['terminal_status_bound'] and not result['audit_complete'] and 'sidecar' in result['read_error']


@pytest.mark.parametrize('over_limit', [False, True])
def test_actual_runner_denies_completion_despite_fresh_stationary_geometric_stop(tmp_path, monkeypatch, harness, over_limit):
    terminal_io(monkeypatch, harness); original = runner.GazeboIO
    class IO(original):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            kwargs['actuator_audit'].write_bytes((b' '*(act.MAX_AUDIT_RECORD_BYTES+1) if over_limit else b'')+encoded())
            self.actuator = NS(process=NS(pid=1234), status=terminal(history()), close=lambda: None)
    monkeypatch.setattr(runner, 'GazeboIO', IO)
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=1)
    assert result['exception'] is None and result['audit']['stationary'] and result['audit']['post_stop_pose_pair_fresh']
    assert result['audit']['normal_geometric_stop_pass'] and result['controller_result']['state'] == 'visual_stop'
    assert result['completed'] is (not over_limit) and result['actuator_watchdog']['fault_latched'] is over_limit
