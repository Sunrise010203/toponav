"""Authority-log consistency/IPC binding; never physical-stop certification."""
import ast
import copy
import json
from pathlib import Path
import time

import pytest

from simulation.gazebo_actuator import summarize_audit
from tests.test_review_gazebo_watchdog import client, packet, send_motion


STATUS = {'latched', 'reason', 'wheels', 'deadline_steady_s', 'command_seq'}


def history():
    def row(event, wall, seq=0, deadline=None, latched=False, reason='disarmed', wheels=None):
        return dict(event=event, wall_s=wall, command_seq=seq, deadline_steady_s=deadline,
                    latched=latched, reason=reason, wheels=[0., 0.] if wheels is None else wheels,
                    publish_success=True)
    return [row('ready', 10.), row('command', 10.1, 1, 10.8, reason='accepted', wheels=[.1, .2]),
            row('command', 10.2, 2, 10.9, reason='accepted', wheels=[.12, -.2]),
            row('stop', 10.3, 2, 10.9, True, 'remote_stop'),
            row('shutdown', 10.4, 2, 10.9, True, 'remote_stop'),
            row('worker_exit', 10.5, 2, 10.9, True, 'remote_stop')]


def terminal(rows):
    return {key: rows[-1][key] for key in STATUS}


def summarize(tmp_path, rows, expected=None, *, unbound=False):
    path = tmp_path/'actuator.jsonl'
    path.write_text(''.join(json.dumps(e)+'\n' for e in rows))
    return summarize_audit(path, 123, **({} if unbound else {'expected_status': terminal(rows) if expected is None else expected}))


def test_full_history_is_complete_bound_but_not_physical_confirmation(tmp_path):
    result = summarize(tmp_path, history())
    assert result['audit_complete'] and not result['fault_latched']
    assert result['terminal_status_bound'] and result['terminal_status_binding_requested']
    assert result['validation_scope'] == 'full_authority_history'
    assert 'stationary' not in result


@pytest.mark.parametrize('index', [1, 2])
def test_missing_middle_or_last_command_is_not_complete(tmp_path, index):
    rows = history(); expected = terminal(rows); rows.pop(index)
    result = summarize(tmp_path, rows, expected)
    assert not result['audit_complete'] and result['fault_latched']
    assert 'audit' in result['read_error']


@pytest.mark.parametrize('field,value', [
    ('command_seq', True), ('command_seq', -1), ('command_seq', 1.0),
    ('deadline_steady_s', True), ('deadline_steady_s', None), ('deadline_steady_s', 0.),
    ('deadline_steady_s', float('inf')), ('deadline_steady_s', 11.),
    ('wheels', [.151, 0.]), ('wheels', [-.01, 0.]), ('wheels', [0., .301]),
    ('wheels', [0., -.301]), ('wheels', [True, 0.]), ('wheels', [0., float('nan')]),
    ('wall_s', -1.), ('wall_s', True), ('latched', 0), ('reason', 'disarmed'),
    ('event', None), ('event', 'ready'), ('event', 'worker_exit'), ('publish_success', 1),
])
def test_impossible_record_fails_closed(tmp_path, field, value):
    rows = history(); rows[1][field] = value
    result = summarize(tmp_path, rows)
    assert not result['audit_complete'] and result['fault_latched']


@pytest.mark.parametrize('field', sorted(STATUS | {'event', 'wall_s', 'publish_success'}))
def test_missing_full_schema_never_falls_back_in_production(tmp_path, field):
    rows = history(); del rows[1][field]
    result = summarize(tmp_path, rows)
    assert not result['audit_complete'] and result['fault_latched']


@pytest.mark.parametrize('change', ['deadline', 'target', 'sequence_gap', 'replay', 'clock',
                                  'unlatch', 'latch_reason', 'latch_seq', 'latch_deadline',
                                  'stop_not_latched', 'extra', 'initial_authority'])
def test_invalid_history_fails_closed(tmp_path, change):
    rows = history()
    if change in ('deadline', 'target', 'replay'):
        row = copy.deepcopy(rows[1]); row['wall_s'] = 10.15
        if change != 'replay': row['event'] = 'idle'
        if change == 'deadline': row['deadline_steady_s'] = 10.85
        if change == 'target': row['wheels'] = [.11, .2]
        rows.insert(2, row)
    elif change == 'sequence_gap': rows[2]['command_seq'] = 3
    elif change == 'clock': rows[2]['wall_s'] = 10.05
    elif change == 'unlatch': rows.insert(4, {**rows[2], 'wall_s': 10.35})
    elif change == 'latch_reason': rows[4]['reason'] = 'remote_abort'
    elif change == 'latch_seq': rows[4]['command_seq'] = 3
    elif change == 'latch_deadline': rows[4]['deadline_steady_s'] = 10.95
    elif change == 'stop_not_latched': rows[3] = {**rows[2], 'wall_s': 10.3, 'event': 'stop'}
    elif change == 'extra': rows[1]['extra'] = 1
    elif change == 'initial_authority': rows[0] = {**rows[1], 'wall_s': 10., 'event': 'ready'}
    result = summarize(tmp_path, rows)
    assert not result['audit_complete'] and result['fault_latched']


@pytest.mark.parametrize('field,value', [
    ('command_seq', 1), ('deadline_steady_s', 11.), ('latched', False),
    ('reason', 'remote_abort'), ('wheels', [.1, 0.]), ('command_seq', True),
])
def test_independent_terminal_ack_must_match(tmp_path, field, value):
    rows = history(); status = terminal(rows); status[field] = value
    result = summarize(tmp_path, rows, status)
    assert not result['audit_complete'] and not result['terminal_status_bound']
    assert result['fault_latched']


@pytest.mark.parametrize('status', [None, {}, [], 'status'])
def test_missing_ack_is_not_a_legacy_fallback(tmp_path, status):
    path = tmp_path/'actuator.jsonl'; path.write_text(''.join(json.dumps(e)+'\n' for e in history()))
    result = summarize_audit(path, 123, expected_status=status)
    assert result['terminal_status_binding_requested'] and not result['terminal_status_bound']
    assert result['fault_latched'] and not result['audit_complete']


def test_self_consistent_zero_replacement_detected_only_by_external_binding(tmp_path):
    rows = history(); expected = terminal(rows)
    replaced = [rows[0], {**rows[-1], 'command_seq': 0, 'deadline_steady_s': None}]
    unbound = summarize(tmp_path, replaced, unbound=True)
    assert unbound['audit_complete'] and not unbound['terminal_status_bound']
    bound = summarize(tmp_path, replaced, expected)
    assert not bound['audit_complete'] and bound['fault_latched']
    assert 'IPC acknowledgement' in bound['read_error']


def test_legacy_minimal_fixture_retains_compatibility_and_labels_limit(tmp_path):
    rows = [{key: val for key, val in e.items() if key not in ('command_seq', 'deadline_steady_s')}
            for e in history()[3::2]]
    rows[0]['event'] = 'ready'
    result = summarize(tmp_path, rows, unbound=True)
    assert result['audit_complete'] and not result['fault_latched']
    assert result['validation_scope'] == 'legacy_minimal_records'
    assert not result['terminal_status_binding_requested'] and not result['terminal_status_bound']
    assert not summarize(tmp_path, rows, terminal(history()))['audit_complete']


@pytest.mark.parametrize('reason', ['remote_stop', 'remote_visual_stop', 'remote_abort',
                                  'command_lease_expired', 'publisher_failure', 'audit_failure'])
def test_consistent_latches_preserve_complete_vs_fault_distinction(tmp_path, reason):
    rows = history()
    for row in rows[3:]: row['reason'] = reason
    if reason == 'command_lease_expired':
        for i, row in enumerate(rows[3:]): row['wall_s'] = 11.+i*.1
    if reason == 'publisher_failure': rows[3]['publish_success'] = False
    result = summarize(tmp_path, rows)
    assert result['audit_complete'] and result['terminal_status_bound']
    assert result['fault_latched'] == (reason not in ('remote_stop', 'remote_visual_stop', 'remote_abort'))


def test_publish_completion_after_deadline_does_not_invalidate_fault_history(tmp_path):
    rows = history()
    rows[2]['wall_s'] = 11.1
    for i, row in enumerate(rows[3:]):
        row.update(wall_s=11.2+i*.1, reason='command_lease_expired')
    result = summarize(tmp_path, rows)
    assert result['audit_complete'] and result['fault_latched']


def test_zero_only_start_stop_needs_no_motion_authority(tmp_path):
    rows = [history()[0], *[{**e, 'command_seq': 0, 'deadline_steady_s': None} for e in history()[3:]]]
    result = summarize(tmp_path, rows)
    assert result['audit_complete'] and not result['fault_latched']


def test_deep_json_is_a_read_error_not_an_escaped_exception(tmp_path):
    path = tmp_path/'actuator.jsonl'
    path.write_text(''.join(json.dumps(e)+'\n' for e in history())+'{"nested":'+'['*2000+'0'+']'*2000+'}\n')
    result = summarize_audit(path, 123, expected_status=terminal(history()))
    assert result['read_error'].startswith('RecursionError:')
    assert result['fault_latched'] and not result['audit_complete']


def test_real_worker_log_and_shutdown_ack_bind_then_deleted_command_fails(client):
    obj, _, path = client
    send_motion(client)
    p = packet(2, 2, .2, time.monotonic())
    obj.request('command', **{k: v for k, v in p.items() if k not in ('kind', 'session_id', 'request_id')})
    obj.request('stop'); obj.close()
    result = summarize_audit(path, obj.process.pid, expected_status=obj.status)
    assert result['audit_complete'] and result['terminal_status_bound'] and not result['fault_latched']
    rows = [json.loads(s) for s in path.read_text().splitlines()]
    rows.pop(next(i for i, e in enumerate(rows) if e['event'] == 'command'))
    path.write_text(''.join(json.dumps(e)+'\n' for e in rows))
    result = summarize_audit(path, obj.process.pid, expected_status=obj.status)
    assert not result['audit_complete'] and result['fault_latched']


def test_production_summary_always_requests_independent_status_binding():
    root = Path(__file__).resolve().parents[1]
    tree = ast.parse((root/'simulation/gazebo_cross_validation.py').read_text())
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Name) and n.func.id == 'summarize_audit']
    assert len(calls) == 1
    assert any(k.arg == 'expected_status' and ast.unparse(k.value) == 'io.actuator.status'
               for k in calls[0].keywords)
