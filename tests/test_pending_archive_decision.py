"""Caller JSON is not a credential, consumed token, live read or authority."""
import ast
import copy
import json
from pathlib import Path
import pytest
from tools import pending_archive_decision as d
from tools import pending_archive_decision_cases as c

ROOT=Path(__file__).resolve().parents[1]
CASES=c.corpus(ROOT)
BY_NAME={r['case']:r for r in CASES}
BASE=BY_NAME['closed--fixture--regular']


@pytest.mark.parametrize('row',CASES,ids=[r['case'] for r in CASES])
def test_closed_records_and_mutation_boundaries(row):
    before=copy.deepcopy(row['expected_claim']);a=c.evaluate(row);b=c.evaluate(row,True)
    assert c.check(row,a,b) and row['expected_claim']==before


@pytest.mark.parametrize('value',[None,True,{},[],bytearray(b'{}'),memoryview(b'{}'),'{}',1])
def test_nonbytes_precedes_hash_and_decode(value,monkeypatch):
    monkeypatch.setattr(d.hashlib,'sha256',lambda *a:pytest.fail('hash before byte check'))
    assert d.review_reader_decision(value,{})['error']=='decision_record_bytes'


def test_byte_subclass_is_not_accepted():
    class B(bytes):pass
    assert d.review_reader_decision(B(BASE['record_raw']),BASE['expected_claim'])['error']=='decision_record_bytes'


def test_byte_limit_precedes_hash(monkeypatch):
    monkeypatch.setattr(d.hashlib,'sha256',lambda *a:pytest.fail('hash before cap'))
    assert d.review_reader_decision(b'x'*(d.RECORD_LIMIT+1),{})['error']=='decision_record_byte_bound'


@pytest.mark.parametrize('stage',['projection_reader_refused','projection_reader_reference_refused',None,'unknown_outcome'])
def test_top_refusal_precedes_any_nested_positive_or_invalid_expected_claim(stage):
    rec=json.loads(BASE['record_raw']);rec['result']['stage']=stage
    rec['result']['error']=None;rec['result']['local_regular_file_reads_complete']=True
    out=d.review_reader_decision(c.encode(rec),object())
    assert out['error']=='reader_top_level_not_checked' and not out['offline_reference_review_ready']
    assert out['reader_stage']==stage and out['reader_error'] is None and out['observed_validation_site'] is None


@pytest.mark.parametrize('value',[None,{},[],True,dict(BASE['expected_claim'],motion_authority=True)])
def test_bad_expected_claim_is_not_coerced(value):
    assert d.review_reader_decision(BASE['record_raw'],value)['error']=='decision_expected_claim_shape'


@pytest.mark.parametrize('name',['inplace_after','replace_after','unlink_after'])
@pytest.mark.parametrize('role',d.ROLES)
def test_positive_nested_cannot_override_genuine_reader_refusal(name,role):
    row=BY_NAME['closed--fixture--'+name+'--'+role];rec=json.loads(row['record_raw']);out=c.evaluate(row)
    assert rec['result']['projection_result']['stage']=='projection_reference_checked'
    assert out['error']=='reader_top_level_not_checked' and out['reader_error']==rec['result']['error']


@pytest.mark.parametrize('name',['regular','empty','hardlink_pair','transient_ancestor_restore','joint_repin','unrelated_sealed_pair'])
def test_retained_reader_limitations_do_not_grant_new_authority(name):
    out=c.evaluate(BY_NAME['closed--fixture--'+name]);assert out['stage']=='offline_reference_consistent'
    assert not out['serialized_record_origin_authenticated'] and not out['atomic_multi_file_snapshot_observed']
    assert not out['observation_semantics_validated'] and not out['motion_authority']


@pytest.mark.parametrize('name',['cross-call-consistent-splice','fabricated-metadata','coherent-forgery-and-repin','same-record-replay'])
def test_coherent_forgery_splice_replay_can_pass_but_never_authenticate(name):
    row=BY_NAME['limitation--'+name];out=c.evaluate(row)
    assert out['offline_reference_review_ready'] is True
    assert not out['result_tampering_excluded'] and not out['cross_call_splice_excluded']
    assert not out['freshness_proved'] and not out['one_time_result_consumption_enforced']


def test_repeat_is_stateless_not_one_time_consumption():
    first=c.evaluate(BASE)
    assert all(c.evaluate(BASE)==first for _ in range(8))
    assert first['stateful_consumption'] is False and first['runtime_result_consumed'] is False


def test_output_and_claim_do_not_alias_later_calls():
    expected=copy.deepcopy(BASE['expected_claim']);out=d.review_reader_decision(BASE['record_raw'],expected)
    saved=copy.deepcopy(out);out['expected_claim_snapshot']['raw_ref']='changed';out['sampled_artifact_sha256'].clear()
    assert expected==BASE['expected_claim'] and d.review_reader_decision(BASE['record_raw'],expected)==saved


def test_old_16_66_four_failures_and_runtime_gaps_remain():
    old=json.loads((ROOT/c.PREVIOUS/'SUMMARY.json').read_bytes())
    assert old['source_error_differences_preserved']==66 and old['same_label_links']==16 and old['old_execution_sites_observed']==0
    assert old['original_four_mismatches_retained'] and old['late_joint_resign_original_preserved'] and old['unknown_late_preserved']
    assert old['old_12266_refusal_preserved'] and old['old_2X_initial_1064_failures_preserved']
    assert old['runtime_requirements_not_implemented']==14 and old['runtime_scenarios_planned_not_executed']==25


def test_bad_pins_precede_decision_and_output(tmp_path,monkeypatch):
    p=tmp_path/'pins';p.write_bytes(b'{}');out=tmp_path/'audit'
    monkeypatch.setattr(d,'review_reader_decision',lambda *a:pytest.fail('decision before pins'))
    with pytest.raises(ValueError,match='input_pin_mismatch'):c.run(ROOT,out,p,c.sha(p.read_bytes()))
    assert not out.exists()


def test_no_io_clocks_old_readers_contracts_or_protocol_calls():
    tree=ast.parse((ROOT/c.SOURCE_FILES[0]).read_bytes())
    assert {a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names}=={'hashlib','json','re'}
    assert not any(isinstance(n,ast.ImportFrom) for n in ast.walk(tree))
    forbidden={'open','read','read_bytes','write','mkdir','unlink','rename','replace','socket','sleep','time','monotonic',
        'read_projection_archive','review_projection','review_archive_binding','review_envelope','inspect_evidence','eval','exec','__import__'}
    assert not any(isinstance(n,ast.Call) and (n.func.id if isinstance(n.func,ast.Name) else n.func.attr if isinstance(n.func,ast.Attribute) else '') in forbidden for n in ast.walk(tree))
    spec=json.loads((ROOT/'docs/PENDING_ARCHIVE_DECISION_V1.json').read_bytes())
    assert spec['case_count']==len(CASES) and spec['old_flags_remain_false']==list(d.FALSE_FIELDS)
