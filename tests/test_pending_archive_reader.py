"""Filesystem facts must not become atomic snapshot/authenticated provenance."""
import ast
import copy
import errno
import json
import os
from pathlib import Path
import pytest
from tools import pending_archive_reader as r
from tools import pending_archive_reader_cases as d

ROOT = Path(__file__).resolve().parents[1]
CASES = d.corpus(ROOT)
BY_NAME = {c['case']: c for c in CASES}


@pytest.mark.parametrize('row', CASES, ids=[c['case'] for c in CASES])
def test_closed_archive_and_temporary_filesystem_boundary_cases(row):
    before = len(list(Path('/proc/self/fd').iterdir()))
    a = d.evaluate_once(ROOT, row); b = d.evaluate_once(ROOT, row)
    assert d.check(row, a, b)
    assert len(list(Path('/proc/self/fd').iterdir())) == before


@pytest.mark.parametrize('value', [None, True, b'/tmp', Path('/tmp'), '', 'relative', '/tmp/', '/tmp//a',
                                  '/tmp/../a', '/tmp/./a', '/tmp/\0a', '/tmp/\ud800', '/' + 'a/' * 65 + 'b', '/' + 'x' * 4096])
def test_root_shape_precedes_any_filesystem_activity(value, monkeypatch):
    monkeypatch.setattr(r, '_open', lambda *a: pytest.fail('opened invalid root'))
    assert r.read_archive_pair(value, {})['error'] == 'reader_root_shape'


@pytest.mark.parametrize('value', [None, True, [], {}, dict(profile='bad')])
def test_claim_shape_precedes_any_filesystem_activity(value, monkeypatch):
    monkeypatch.setattr(r, '_open', lambda *a: pytest.fail('opened invalid claim'))
    assert r.read_archive_pair('/tmp', value)['error'] == 'reader_claim_shape'


def test_all_three_path_roles_reject_escape_alias_and_unbounded_strings(tmp_path, monkeypatch):
    _, _, _, _, claim = d._fixture(tmp_path, 'regular')
    monkeypatch.setattr(r, '_open', lambda *a: pytest.fail('opened invalid claim'))
    for key in ('manifest_ref', 'raw_ref', 'observation_ref'):
        for value in ('/etc/passwd', '../outside', 'a//b', 'a/./b', 'a/../b', 'a\\b', '档案', 'x' * 100000):
            assert r.read_archive_pair('/tmp', dict(claim, **{key: value}))['error'] == 'reader_claim_shape'
    assert r.read_archive_pair('/tmp', dict(claim, raw_ref=claim['observation_ref']))['error'] == 'reader_claim_shape'


def test_missing_platform_features_refuse_without_unsafe_fallback(tmp_path, monkeypatch):
    root, _, _, _, claim = d._fixture(tmp_path, 'regular')
    monkeypatch.delattr(r.os, 'O_PATH')
    monkeypatch.setattr(r, '_open', lambda *a: pytest.fail('unsafe fallback'))
    assert r.read_archive_pair(str(root), claim)['error'] == 'reader_platform_unsupported'


def test_read_error_closes_every_descriptor_and_reports_role(tmp_path, monkeypatch):
    root, _, _, _, claim = d._fixture(tmp_path, 'regular'); before = len(list(Path('/proc/self/fd').iterdir()))
    def fail(*args): raise OSError(errno.EIO, 'synthetic read failure')
    monkeypatch.setattr(r.os, 'read', fail)
    result = r.read_archive_pair(str(root), claim)
    assert (result['stage'], result['error'], result['filesystem_error_role'], result['os_errno']) == ('reader_refused', 'archive_read', 'manifest', errno.EIO)
    assert len(list(Path('/proc/self/fd').iterdir())) == before


def test_close_error_is_not_silently_upgraded_to_success_or_retried(tmp_path, monkeypatch):
    root, _, _, _, claim = d._fixture(tmp_path, 'regular'); close = os.close; calls = []
    before = len(list(Path('/proc/self/fd').iterdir()))
    def close_then_fail(fd):
        calls.append(fd); close(fd)
        if len(calls) == 1: raise OSError(errno.EIO, 'synthetic cleanup error after release')
    monkeypatch.setattr(r.os, 'close', close_then_fail)
    result = r.read_archive_pair(str(root), claim)
    assert result['error'] == 'archive_close' and result['stage'] == 'reader_refused'
    assert not result['local_regular_file_reads_complete'] and len(calls) == len(set(calls))
    assert len(list(Path('/proc/self/fd').iterdir())) == before


def test_nonregular_leaf_is_never_opened_for_reading(tmp_path, monkeypatch):
    root, _, refs, _, claim = d._fixture(tmp_path, 'fifo')
    original = r._open; pins = []
    def inspect(name, flags, state, role, parent=None):
        if role == 'raw' and not flags & os.O_DIRECTORY:
            assert flags & os.O_PATH  # no second ordinary open on FIFO
            pins.append(name)
        return original(name, flags, state, role, parent)
    monkeypatch.setattr(r, '_open', inspect)
    result = r.read_archive_pair(str(root), claim)
    assert result['error'] == 'archive_file_type' and pins == [Path(refs['raw']).name]


def test_final_failure_does_not_promote_nested_positive_contract_or_stale_checks():
    result = d.evaluate_once(ROOT, BY_NAME['fixture--raw_replace'])['result']
    assert result['reference_result']['file_references_agree'] is True
    assert result['stage'] == 'reader_refused' and not result['local_regular_file_reads_complete']
    assert all(f['final_recheck_agreed'] is None for f in result['files'])


def test_transient_ancestor_restore_is_not_falsely_called_atomic_snapshot():
    record = d.evaluate_once(ROOT, BY_NAME['fixture--transient_ancestor_restore'])
    assert record['result']['stage'] == 'reader_reference_checked' and len(record['events']) == 1
    assert record['result']['atomic_multi_file_snapshot_observed'] is False
    assert record['result']['root_origin_authenticated'] is False


def test_actual_2W_delivery_catalog_over_entry_cap_is_not_silently_relaxed():
    manifest = json.loads((ROOT / d.OVERSIZED_CATALOG).read_bytes())
    assert len(manifest['files']) == 12266 > r.binding.ENTRY_LIMIT
    record = d.evaluate_once(ROOT, BY_NAME['archive-catalog--sealed_2W_over_entry_cap'])
    assert record['result']['local_regular_file_reads_complete'] is True
    assert record['result']['error'] == 'manifest_shape'
    assert record['result']['reference_result']['file_references_agree'] is False


def test_returned_facts_do_not_alias_caller_or_later_reads(tmp_path):
    root, _, _, _, claim = d._fixture(tmp_path, 'regular'); before = copy.deepcopy(claim)
    first = r.read_archive_pair(str(root), claim); expected = copy.deepcopy(first)
    first['claim_snapshot']['raw_ref'] = 'changed'; first['files'].clear()
    assert claim == before and r.read_archive_pair(str(root), claim) == expected


def test_read_sizes_and_flags_are_bounded_and_descriptors_noninheritable(tmp_path, monkeypatch):
    root, _, _, _, claim = d._fixture(tmp_path, 'artifact_caps'); original = r._open; read = os.read
    opened = []; reads = []
    def observed_open(name, flags, state, role, parent=None):
        fd = original(name, flags, state, role, parent); opened.append((name, flags, role))
        assert not os.get_inheritable(fd)
        assert not flags & (os.O_CREAT | os.O_TRUNC | os.O_WRONLY | os.O_RDWR)
        # Only numeric pinned descriptors in fixed proc_fd intentionally follow.
        if not (parent == state.get('proc_fd') and str(name).isdigit()): assert flags & os.O_NOFOLLOW
        return fd
    def observed_read(fd, count):
        assert 1 <= count <= 65536; reads.append(count); return read(fd, count)
    monkeypatch.setattr(r, '_open', observed_open); monkeypatch.setattr(r.os, 'read', observed_read)
    assert r.read_archive_pair(str(root), claim)['local_regular_file_reads_complete']
    assert opened and reads


def test_preserved_16_66_four_unknown_late_and_original_resign():
    old = json.loads((ROOT / d.PREVIOUS / 'SUMMARY.json').read_bytes())
    assert old['same_label_links'] == 16 and old['old_execution_sites_observed'] == 0
    assert old['source_error_differences_preserved'] == 66 and old['original_four_mismatches_retained']
    assert old['runtime_requirements_not_implemented'] == 14 and old['runtime_scenarios_planned_not_executed'] == 25
    late = ROOT / 'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample'
    result = json.loads((late / 'result.json').read_bytes())
    assert d.sha((late / 'input.json').read_bytes()) == result['input_sha256']
    assert all(result[k][0]['accepted'] and result[k][0]['deadline_steady_s'] > 10.8 for k in ('python', 'receiver'))
    for n in d.PRESERVED:
        if '/preserve--' in n:
            ledger = json.loads((ROOT / n).read_bytes())['result']['review']['submissions']
            unknown = [s for s in ledger if s['outcome'] == 'unknown_outcome']; assert len(unknown) == 1
            if 'unknown_result_native' not in n: assert unknown[0]['reply_indices'] == [8]


def test_bad_audit_pins_precede_reads_or_output(tmp_path, monkeypatch):
    pin = tmp_path / 'pins.json'; pin.write_bytes(b'{}'); output = tmp_path / 'audit'
    monkeypatch.setattr(r, 'read_archive_pair', lambda *a: pytest.fail('reader before pins'))
    with pytest.raises(ValueError, match='input_pin_mismatch'): d.run(ROOT, output, pin, d.sha(pin.read_bytes()))
    assert not output.exists()


def test_reader_has_no_write_network_clock_compiler_or_protocol_execution():
    tree = ast.parse((ROOT / d.SOURCE_FILES[0]).read_bytes())
    assert {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names} == {'hashlib', 'os', 're', 'stat'}
    assert {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)} == {'tools'}
    forbidden = {'write', 'write_bytes', 'mkdir', 'unlink', 'rename', 'replace', 'socket', 'sleep', 'time', 'monotonic',
                 'review_envelope', 'inspect_evidence', 'decode_message', 'eval', 'exec', '__import__'}
    assert not any(isinstance(n, ast.Call) and (n.func.id if isinstance(n.func, ast.Name) else n.func.attr if isinstance(n.func, ast.Attribute) else '') in forbidden for n in ast.walk(tree))
    spec = json.loads((ROOT / 'docs/PENDING_ARCHIVE_READER_V1.json').read_bytes())
    assert spec['case_count'] == len(CASES) and spec['profile'] == r.PROFILE
    assert spec['old_flags_remain_false'] == list(r.binding.FALSE_FIELDS)
