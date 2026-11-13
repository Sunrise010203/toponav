"""Reference agreement must not become authenticated input/execution binding."""
import ast
import copy
import json
from pathlib import Path
import pytest
from tools import pending_archive_binding as b
from tools import pending_archive_binding_cases as d

ROOT = Path(__file__).resolve().parents[1]
CASES = d.corpus(ROOT)
BY_NAME = {r['case']: r for r in CASES}
ARCHIVE = {(r['source_case'], r['origin']): r for r in CASES if r['family'] == 'sealed_archive'}
LINKS = json.loads((ROOT / d.PREVIOUS / 'SAME_LABEL_EVIDENCE.json').read_bytes())


@pytest.mark.parametrize('row', CASES, ids=[r['case'] for r in CASES])
def test_closed_reference_cases_are_repeatable_and_non_authoritative(row):
    output, repeat = d.evaluate(row)
    assert d.check(row, output, repeat)
    assert output['observation_encoding'] == 'opaque_archive_bytes_not_parsed'
    if output['file_references_agree']:
        assert output['manifest_pin_basis'] == 'caller_supplied_not_trust_anchor'
        assert output['claim_snapshot'] == row['claim'] and output['claim_snapshot'] is not row['claim']


@pytest.mark.parametrize('link', LINKS, ids=[r['parent'] for r in LINKS])
def test_same_labels_remain_different_raw_refs_with_unknown_execution(link):
    for origin in ('python', 'cpp'):
        parent = ARCHIVE[link['parent'], origin]; child = ARCHIVE[link['child'], origin]
        a, _ = d.evaluate(parent); c, _ = d.evaluate(child)
        assert a['file_references_agree'] and c['file_references_agree']
        assert a['claim_snapshot']['raw_ref'] != c['claim_snapshot']['raw_ref']
        assert a['artifact_sha256']['raw'] != c['artifact_sha256']['raw']
        # Python JSON is formatted, C++ stdout compact; compare snapshots,
        # not canonicalized bytes as a substitute for artifact identity.
        assert json.loads(parent['observation_raw']) == json.loads(child['observation_raw'])
        assert a['observed_validation_site'] is c['observed_validation_site'] is None
        assert not a['observation_input_binding_authenticated'] and not c['pair_execution_link_observed']


@pytest.mark.parametrize('name', ['claim--unrelated_sealed_pair', 'claim--joint_replacement',
                                 'claim--empty_artifacts', 'claim--manifest_ref_is_declarative'])
def test_ref_agreement_does_not_authenticate_pairing_provenance_or_semantics(name):
    row = BY_NAME[name]; result, _ = d.evaluate(row)
    assert result['file_references_agree'] and result['pairing_declared']
    assert not any(result[k] for k in b.FALSE_FIELDS)


@pytest.mark.parametrize('value', [None, True, [], {}, '', bytearray(b'a'), memoryview(b'a')])
@pytest.mark.parametrize('position,role', [(0, 'manifest'), (2, 'raw'), (3, 'observation')])
def test_non_exact_byte_types_are_refused_before_hashing(value, position, role, monkeypatch):
    row = BY_NAME['claim--baseline']
    args = [row['manifest_raw'], row['claim'], row['raw'], row['observation_raw']]; args[position] = value
    monkeypatch.setattr(b, 'sha', lambda *a: pytest.fail('hashed invalid input'))
    result = b.review_archive_binding(*args)
    assert result['error'] == role + '_bytes' and result['artifact_sha256'] is None


@pytest.mark.parametrize('role', ['manifest', 'raw', 'observation'])
def test_bounds_and_claim_shape_are_checked_before_hash_or_json(role, monkeypatch):
    row = BY_NAME['bound--' + role]
    monkeypatch.setattr(b, 'sha', lambda *a: pytest.fail('hashed oversized input'))
    monkeypatch.setattr(b, '_manifest', lambda *a: pytest.fail('parsed oversized input'))
    assert d.evaluate(row)[0]['error'] == role + '_byte_bound'
    assert d.evaluate(BY_NAME['shape--profile'])[0]['error'] == 'claim_shape'


def test_failed_pin_precedes_manifest_decode(monkeypatch):
    monkeypatch.setattr(b, '_manifest', lambda *a: pytest.fail('decoded before pin matched'))
    out, _ = d.evaluate(BY_NAME['mismatch--manifest_pin'])
    assert out['manifest_digest_matches'] is False
    assert out['raw_reference_digest_matches'] is out['observation_reference_digest_matches'] is None


def test_exact_manifest_limit_scope_path_and_entry_limits_are_inclusive():
    row = BY_NAME['claim--baseline']; doc = json.loads(row['manifest_raw'])
    doc['scope'] = 'x' * 4096
    doc['files']['a' * 512] = '0' * 64
    for i in range(b.ENTRY_LIMIT - len(doc['files'])): doc['files'][f'z/{i}'] = '0' * 64
    assert len(doc['files']) == b.ENTRY_LIMIT
    encoded = d.encode(doc); manifest = encoded + b' ' * (b.MANIFEST_LIMIT - len(encoded))
    claim = dict(row['claim'], manifest_sha256=d.sha(manifest))
    result = b.review_archive_binding(manifest, claim, row['raw'], row['observation_raw'])
    assert result['file_references_agree'] and len(manifest) == b.MANIFEST_LIMIT


def test_manifest_raw_encoding_and_observation_format_are_not_canonicalized():
    a, _ = d.evaluate(BY_NAME['claim--baseline'])
    wrong, _ = d.evaluate(BY_NAME['manifest--whitespace_without_repin'])
    repin, _ = d.evaluate(BY_NAME['manifest--whitespace_with_repin'])
    assert a['artifact_sha256']['manifest'] != repin['artifact_sha256']['manifest']
    assert wrong['error'] == 'manifest_digest_mismatch' and repin['file_references_agree']
    row = copy.deepcopy(BY_NAME['claim--baseline']); row['observation_raw'] += b' '
    out, _ = d.evaluate(row)
    assert out['error'] == 'observation_digest_mismatch'


def test_returned_claim_and_input_claim_do_not_alias_or_gain_state():
    row = copy.deepcopy(BY_NAME['claim--baseline']); first, _ = d.evaluate(row)
    expected = copy.deepcopy(first); first['claim_snapshot']['raw_ref'] = 'mutated'
    assert d.evaluate(row)[0] == expected
    row['claim']['raw_ref'] = 'changed'
    assert expected['claim_snapshot']['raw_ref'] != row['claim']['raw_ref']


def test_532_raws_66_labels_four_wrapper_differences_and_old_limitations_preserved():
    assert len(ARCHIVE) == 1064 and len(LINKS) == 16
    oldplan = json.loads((ROOT / d.ARCHIVE / 'PLAN.json').read_bytes())['cases']
    different = 0
    for name, spec in oldplan.items():
        py = ARCHIVE[name, 'python']; cpp = ARCHIVE[name, 'cpp']
        assert d.sha(py['raw']) == d.sha(cpp['raw']) == spec['input_sha256']
        different += json.loads(py['observation_raw'])['error'] != json.loads(cpp['observation_raw'])['error']
    assert different == 66
    old = json.loads((ROOT / d.PREVIOUS / 'SUMMARY.json').read_bytes())
    assert old['original_four_mismatches_retained'] and old['actual_old_execution_sites_observed'] == 0
    assert old['late_joint_resign_original_both_accepted'] and old['runtime_requirements_not_implemented'] == 14
    assert old['runtime_scenarios_planned_not_executed'] == 25
    assert json.loads(ARCHIVE['baseline--original--declared_refusal', 'python']['observation_raw'])['result']['review']['synthetic_reference_matches'] is False
    late = ROOT / 'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample'
    result = json.loads((late / 'result.json').read_bytes())
    assert d.sha((late / 'input.json').read_bytes()) == result['input_sha256']
    assert all(result[k][0]['accepted'] and result[k][0]['deadline_steady_s'] > 10.8 for k in ('python', 'receiver'))
    for n in d.PRESERVED:
        if '/preserve--' in n:
            ledger = json.loads((ROOT / n).read_bytes())['result']['review']['submissions']
            unknown = [r for r in ledger if r['outcome'] == 'unknown_outcome']
            assert len(unknown) == 1
            if 'unknown_result_native' not in n: assert unknown[0]['reply_indices'] == [8]


def test_bad_audit_pins_precede_contract_and_output(tmp_path, monkeypatch):
    pin = tmp_path / 'pins.json'; pin.write_bytes(b'{}'); output = tmp_path / 'audit'
    monkeypatch.setattr(b, 'review_archive_binding', lambda *a: pytest.fail('contract before pins'))
    with pytest.raises(ValueError, match='input_pin_mismatch'):
        d.run(ROOT, output, pin, d.sha(pin.read_bytes()))
    assert not output.exists()


def test_pure_module_has_no_file_clock_process_or_old_validator_calls():
    tree = ast.parse((ROOT / d.SOURCE_FILES[0]).read_bytes())
    assert {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names} == {'hashlib', 'json', 're'}
    assert not any(isinstance(n, ast.ImportFrom) for n in ast.walk(tree))
    forbidden = {'open', 'read_bytes', 'write_bytes', 'review_envelope', 'inspect_evidence',
                 'review_reference', 'decode_message', 'eval', 'exec', '__import__'}
    for n in ast.walk(tree):
        if isinstance(n, ast.Call):
            name = n.func.id if isinstance(n.func, ast.Name) else n.func.attr if isinstance(n.func, ast.Attribute) else ''
            assert name not in forbidden
    driver = ast.parse((ROOT / d.SOURCE_FILES[1]).read_bytes())
    assert not any(isinstance(n, ast.Import) and a.name in ('time', 'subprocess', 'socket', 'ctypes') for n in ast.walk(driver) for a in getattr(n, 'names', []))


def test_spec_matches_caps_and_profile():
    spec = json.loads((ROOT / 'docs/PENDING_ARCHIVE_BINDING_V1.json').read_bytes())
    assert spec['profile'] == b.PROFILE and spec['case_count'] == len(CASES)
    assert spec['false_fields'] == list(b.FALSE_FIELDS)
    assert spec['implementation'] == 'pure_python_bytes_no_filesystem_loader'
