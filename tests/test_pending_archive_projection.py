"""Large parent agreement is not old schema relaxation or source authentication."""
import ast
import copy
import json
from pathlib import Path
import pytest
from tools import pending_archive_projection as p
from tools import pending_archive_projection_cases as d

ROOT=Path(__file__).resolve().parents[1];CASES=d.corpus(ROOT);BY_NAME={r['case']:r for r in CASES}
ARCHIVE={(r['source_case'],r['origin']):r for r in CASES if r['family']=='sealed_large_parent'}
LINKS=json.loads((ROOT/'reports/straight-followup-20261103-31/SAME_LABEL_EVIDENCE.json').read_bytes())


@pytest.mark.parametrize('row',CASES,ids=[r['case'] for r in CASES])
def test_closed_parent_projection_cases_are_repeatable_and_non_authoritative(row):
    a,b=d.evaluate(row);assert d.check(row,a,b)
    if a['reference_result'] is not None:
        assert all(a['reference_result'][k] is False for k in p.binding.FALSE_FIELDS)
    if a['stage']=='projection_reference_checked':
        assert a['parent_digest_matches'] is a['projection_digest_matches'] is True
        assert a['raw_parent_projection_digest_matches'] is a['observation_parent_projection_digest_matches'] is True


@pytest.mark.parametrize('link',LINKS,ids=[r['parent'] for r in LINKS])
def test_16_same_old_labels_keep_different_raw_refs_but_no_old_execution_site(link):
    for origin in ('python','cpp'):
        parent=ARCHIVE[link['parent'],origin];child=ARCHIVE[link['child'],origin]
        a,_=d.evaluate(parent);b,_=d.evaluate(child)
        assert a['stage']==b['stage']=='projection_reference_checked'
        assert a['artifact_sha256']['raw']!=b['artifact_sha256']['raw']
        assert a['claim_snapshot']['raw_ref']!=b['claim_snapshot']['raw_ref']
        assert json.loads(parent['observation_raw'])==json.loads(child['observation_raw'])
        assert a['observed_validation_site'] is b['observed_validation_site'] is None
        assert not a['projection_parent_link_authenticated'] and not b['observation_input_binding_authenticated']


@pytest.mark.parametrize('value',[None,True,'',[],{},bytearray(b'x'),memoryview(b'x')])
@pytest.mark.parametrize('position,role',[(0,'parent'),(1,'projection'),(3,'raw'),(4,'observation')])
def test_non_exact_bytes_refuse_before_hash_or_parse(value,position,role,monkeypatch):
    row=BY_NAME['claim--baseline'];args=[row['parent_raw'],row['projection_raw'],row['claim'],row['raw'],row['observation_raw']];args[position]=value
    monkeypatch.setattr(p,'sha',lambda *a:pytest.fail('hashed invalid bytes'))
    monkeypatch.setattr(p,'_catalog',lambda *a:pytest.fail('parsed invalid bytes'))
    out=p.review_projection(*args);assert out['error']==role+'_bytes' and out['artifact_sha256'] is None


@pytest.mark.parametrize('role',['parent','projection','raw','observation'])
def test_byte_bounds_claim_and_pin_precede_work(role,monkeypatch):
    monkeypatch.setattr(p,'sha',lambda *a:pytest.fail('hashed oversized input'))
    assert d.evaluate(BY_NAME['bound--'+role])[0]['error']==role+'_byte_bound'
    assert d.evaluate(BY_NAME['shape--profile'])[0]['error']=='projection_claim_shape'


def test_parent_pin_precedes_parent_parse_and_later_comparisons(monkeypatch):
    monkeypatch.setattr(p,'_catalog',lambda *a:pytest.fail('parsed mismatching parent'))
    out,_=d.evaluate(BY_NAME['mismatch--parent_pin'])
    assert out['parent_digest_matches'] is False and out['projection_digest_matches'] is None
    assert out['raw_parent_projection_digest_matches'] is out['observation_parent_projection_digest_matches'] is None


def test_raw_mismatch_cannot_inherit_parent_selection_success():
    out,_=d.evaluate(BY_NAME['mismatch--raw'])
    assert out['projection_members_agree_with_parent'] is True
    assert out['stage']=='projection_refused' and out['error']=='projection_reference_refused'
    assert out['reference_result']['error']=='raw_digest_mismatch'
    assert out['reference_result']['file_references_agree'] is False


def test_real_12266_parent_still_fails_unmodified_2W_while_tiny_selection_agrees():
    row=CASES[0];assert len(json.loads(row['parent_raw'])['files'])==12266
    oldclaim=dict(profile=p.binding.PROFILE,manifest_ref=d.PARENT,manifest_sha256=d.sha(row['parent_raw']),
        raw_ref=row['claim']['raw_ref'],observation_ref=row['claim']['observation_ref'],pairing_basis='caller_declared_only')
    refused=p.binding.review_archive_binding(row['parent_raw'],oldclaim,row['raw'],row['observation_raw'])
    assert refused['error']=='manifest_shape' and p.binding.ENTRY_LIMIT==12000
    out,_=d.evaluate(row);assert out['reference_result']['file_references_agree']
    assert len(json.loads(row['projection_raw'])['files'])==2
    assert out['reference_result']['artifact_sha256']['manifest']==d.sha(row['projection_raw'])
    original=json.loads((ROOT/d.PREVIOUS/'archive-reader-audit/archive-catalog--sealed_2W_over_entry_cap/read.json').read_bytes())
    assert original['result']['local_regular_file_reads_complete'] and original['result']['error']=='manifest_shape'


@pytest.mark.parametrize('name',['claim--unrelated_sealed_pair','claim--joint_replacement','claim--empty_artifacts','claim--parent_name_is_declarative'])
def test_declared_parent_selection_is_not_authentic_pairing_or_filesystem_proof(name):
    out,_=d.evaluate(BY_NAME[name]);assert out['stage']=='projection_reference_checked'
    assert out['pin_basis']=='caller_supplied_not_trust_anchor'
    assert all(out[k] is False for k in (*p.binding.FALSE_FIELDS,*p.EXTRA_FALSE))


def test_output_claim_does_not_alias_caller_or_repeated_calls():
    row=copy.deepcopy(BY_NAME['claim--baseline']);before=copy.deepcopy(row['claim']);first,_=d.evaluate(row);expected=copy.deepcopy(first)
    first['claim_snapshot']['raw_ref']='changed';first['reference_result']['claim_snapshot']['raw_ref']='changed'
    assert row['claim']==before and d.evaluate(row)[0]==expected


def test_original_532_raws_66_labels_four_wrappers_unknown_late_and_resign_preserved():
    oldplan=json.loads((ROOT/d.ARCHIVE/'PLAN.json').read_bytes())['cases'];differences=0
    for name,spec in oldplan.items():
        py,cpp=ARCHIVE[name,'python'],ARCHIVE[name,'cpp'];assert d.sha(py['raw'])==d.sha(cpp['raw'])==spec['input_sha256']
        differences+=json.loads(py['observation_raw'])['error']!=json.loads(cpp['observation_raw'])['error']
    assert len(ARCHIVE)==1064 and differences==66 and len(LINKS)==16
    old=json.loads((ROOT/d.PREVIOUS/'SUMMARY.json').read_bytes());assert old['original_four_mismatches_retained']
    assert old['development_initial_failures']==1064 and old['development_initial_failures_preserved']
    assert old['runtime_requirements_not_implemented']==14 and old['runtime_scenarios_planned_not_executed']==25
    fs=json.loads((ROOT/d.PREVIOUS/'FILESYSTEM_CASES.json').read_bytes())
    assert fs['transient_ancestor_restore']['stage']=='reader_reference_checked' and fs['hardlink_pair']['local_read_complete']
    assert not fs['transient_ancestor_restore']['atomic_multi_file_snapshot_observed']
    assert json.loads(ARCHIVE['baseline--original--declared_refusal','python']['observation_raw'])['result']['review']['synthetic_reference_matches'] is False
    for n in d.PRESERVED:
        if '/preserve--' in n:
            ledger=json.loads((ROOT/n).read_bytes())['result']['review']['submissions'];unknown=[s for s in ledger if s['outcome']=='unknown_outcome'];assert len(unknown)==1
            if 'unknown_result_native' not in n:assert unknown[0]['reply_indices']==[8]
    late=ROOT/'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample';result=json.loads((late/'result.json').read_bytes())
    assert d.sha((late/'input.json').read_bytes())==result['input_sha256']
    assert all(result[k][0]['accepted'] and result[k][0]['deadline_steady_s']>10.8 for k in ('python','receiver'))


def test_bad_formal_pins_precede_contract_or_output(tmp_path,monkeypatch):
    pin=tmp_path/'pins.json';pin.write_bytes(b'{}');output=tmp_path/'audit'
    monkeypatch.setattr(p,'review_projection',lambda *a:pytest.fail('contract before pins'))
    with pytest.raises(ValueError,match='input_pin_mismatch'):d.run(ROOT,output,pin,d.sha(pin.read_bytes()))
    assert not output.exists()


def test_pure_module_has_no_reader_validator_clock_process_or_write_calls():
    tree=ast.parse((ROOT/d.SOURCE_FILES[0]).read_bytes())
    assert {a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names}=={'hashlib','json','re'}
    assert {n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)}=={'tools'}
    forbidden={'open','read_archive_pair','read_bytes','write_bytes','review_envelope','inspect_evidence','decode_message','time','monotonic','sleep','eval','exec','__import__'}
    assert not any(isinstance(n,ast.Call) and (n.func.id if isinstance(n.func,ast.Name) else n.func.attr if isinstance(n.func,ast.Attribute) else '') in forbidden for n in ast.walk(tree))
    spec=json.loads((ROOT/'docs/PENDING_ARCHIVE_PROJECTION_V1.json').read_bytes())
    assert spec['profile']==p.PROFILE and spec['case_count']==len(CASES)
    assert spec['old_flags_remain_false']==list(p.binding.FALSE_FIELDS) and spec['old_2X_reader_calls']==0
