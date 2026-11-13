"""Readonly byte prediction must never impersonate observed validator execution."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import pytest
from tools import pending_location_evidence as e
from tools import pending_location_evidence_cases as d

ROOT=Path(__file__).resolve().parents[1];CASES=d.corpus(ROOT);BY_SOURCE={f['source_case']:f for f in CASES if f['family']=='sealed_2U'}
LINKS=[x for x in json.loads((ROOT/d.ARCHIVE/'COUNTERFACTUAL_LINKS.json').read_bytes()) if not x['output_stage_or_error_changed']]


@pytest.mark.parametrize('f',CASES,ids=[f['case'] for f in CASES])
def test_closed_snapshots_and_bytes_produce_repeatable_non_authoritative_evidence(f):
    outputs,repeated=d.evaluate(f);assert d.check(f,outputs,repeated)
    for doc in outputs.values():
        assert doc['profile']==e.PROFILE and doc['prediction_scope']=='envelope_structure_only_controls_and_inner_packets_not_inspected'
        assert 'consistent' not in doc  # evidence availability is not old validation


@pytest.mark.parametrize('link',LINKS,ids=[r['parent'] for r in LINKS])
def test_16_equal_old_labels_distinguish_byte_predictions_but_never_observed_sites(link):
    parent=BY_SOURCE[link['parent']];child=BY_SOURCE[link['child']]
    for origin in ('python','cpp'):
        a=e.inspect_evidence(parent['raw'],parent['observations'][origin]);b=e.inspect_evidence(child['raw'],child['observations'][origin])
        assert a['observation_sha256']==b['observation_sha256'] and a['input_sha256']!=b['input_sha256']
        assert a['predicted_first_structural_failure']['reason']==b['predicted_first_structural_failure']['reason']
        assert a['predicted_first_structural_failure']['field_index']!=b['predicted_first_structural_failure']['field_index']
        assert a['byte_facts']!=b['byte_facts'] and a['reported_reason_agrees_with_prediction'] is b['reported_reason_agrees_with_prediction'] is True
        for doc in (a,b):
            assert doc['observed_validation_site'] is None
            assert all(doc[k] is False for k in e.EXTRA_FALSE)


@pytest.mark.parametrize('raw',[None,'TPT1',bytearray(b'TPT1'),memoryview(b'TPT1'),True,[],{}])
def test_non_exact_bytes_refused_before_any_scan_or_snapshot_hash(raw,monkeypatch):
    monkeypatch.setattr(e,'_scan',lambda *a:pytest.fail('scanned invalid type'))
    snapshot=BY_SOURCE['baseline--original--reserve']['observations']['python']
    doc=e.inspect_evidence(raw,snapshot)
    assert doc['error']=='evidence_bytes' and doc['input_sha256'] is None and doc['byte_facts'] is None


def test_invalid_snapshot_refused_before_scan_and_preserved_as_external_audit_input(monkeypatch):
    monkeypatch.setattr(e,'_scan',lambda *a:pytest.fail('scanned invalid snapshot'))
    f=next(f for f in CASES if f['case']=='invalid-snapshot--fake_site')
    assert e.inspect_evidence(f['raw'],f['observations']['python'])['error']=='evidence_observation_shape'
    assert 'observed_validation_site' in f['observations']['python']


def test_mutating_returned_snapshot_or_byte_facts_does_not_mutate_caller_or_repeat():
    f=BY_SOURCE['baseline--original--reserve'];before=copy.deepcopy(f['observations']['python'])
    a=e.inspect_evidence(f['raw'],before);expected=copy.deepcopy(a)
    a['observation_snapshot']['result']['review']['classification']='changed'
    a['byte_facts']['fields'].clear()
    assert before==f['observations']['python'] and e.inspect_evidence(f['raw'],before)==expected


def test_ignored_payload_change_changes_byte_evidence_not_snapshot_or_authority():
    before=BY_SOURCE['baseline--original--reserve'];after=next(f for f in CASES if f['case']=='claim--ignored_reply_byte_mutation')
    a=e.inspect_evidence(before['raw'],before['observations']['python']);b=e.inspect_evidence(after['raw'],after['observations']['python'])
    assert a['input_sha256']!=b['input_sha256'] and a['observation_sha256']==b['observation_sha256']
    assert a['byte_facts']['fields'][4]['observed_payload_sha256']!=b['byte_facts']['fields'][4]['observed_payload_sha256']
    assert a['predicted_first_structural_failure'] is b['predicted_first_structural_failure'] is None
    assert a['observation_input_binding_authenticated'] is b['observation_input_binding_authenticated'] is False


@pytest.mark.parametrize('name',['claim--positive_on_required_null_bytes','claim--wrong_reported_error_on_required_null'])
def test_snapshot_contradictions_do_not_rewrite_snapshot_or_accuse_old_validator(name):
    f=next(f for f in CASES if f['case']==name);doc=e.inspect_evidence(f['raw'],f['observations']['python'])
    assert doc['evidence_stage']=='evidence_returned' and doc['reported_reason_agrees_with_prediction'] is False
    assert doc['observation_snapshot']==f['observations']['python']
    assert not doc['old_validator_executed_by_diagnostic'] and doc['observed_validation_site'] is None


def test_opaque_invalid_controls_do_not_become_implicitly_schema_accepted():
    f=next(f for f in CASES if f['case']=='scan--opaque_controls_and_packets');doc=e.inspect_evidence(f['raw'],f['observations']['python'])
    assert doc['evidence_stage']=='evidence_returned' and doc['predicted_first_structural_failure'] is None
    assert doc['reported_reason_agrees_with_prediction'] is None
    assert doc['byte_facts']['fields'][0]['observed_payload_sha256']==hashlib.sha256(b'\xff').hexdigest()
    assert doc['observation_input_binding_authenticated'] is False


def test_huge_declared_length_only_hashes_existing_byte_and_stops_safely():
    f=next(f for f in CASES if f['case']=='scan--huge_declared_length');doc=e.inspect_evidence(f['raw'],f['observations']['python'])
    field=doc['byte_facts']['fields'][0]
    assert field['declared_length']==0xfffffffe and field['observed_payload_bytes']==1
    assert field['observed_payload_sha256']==hashlib.sha256(b'\xff').hexdigest() and field['payload_complete'] is False
    assert doc['byte_facts']['scan_disposition']=='stopped_at_incomplete_payload'


def test_facts_after_predicted_failure_are_not_old_execution_trace():
    f=next(f for f in CASES if f['case']=='scan--continue_facts_after_first_prediction');doc=e.inspect_evidence(f['raw'],f['observations']['python'])
    assert doc['predicted_first_structural_failure']['field_index']==0
    assert len(doc['byte_facts']['fields'])==5 and doc['byte_facts']['fields'][1]['required_null_violation'] is True
    assert doc['byte_facts']['source']=='supplied_bytes_not_runtime_trace' and doc['observed_validation_site'] is None


def test_532_archive_raw_inputs_and_all_snapshots_remain_exact_including_66_errors():
    plan=json.loads((ROOT/d.ARCHIVE/'PLAN.json').read_bytes())['cases'];different=0
    for name,f in BY_SOURCE.items():
        assert hashlib.sha256(f['raw']).hexdigest()==plan[name]['input_sha256']
        if f['observations']['python']['error']!=f['observations']['cpp']['error']:
            different+=1
            for origin in ('python','cpp'):
                doc=e.inspect_evidence(f['raw'],f['observations'][origin])
                assert doc['observation_snapshot']['error']==f['observations'][origin]['error']
    assert len(BY_SOURCE)==532 and different==66
    summary=json.loads((ROOT/'reports/straight-followup-20261103-28/SUMMARY.json').read_bytes())
    assert summary['known_wrapper_outcome_differences']==4 and summary['all_results_exactly_equivalent'] is False


def test_joint_replacement_repeat_refusal_unknown_and_original_late_resign_are_not_upgraded():
    joint=BY_SOURCE['baseline--original--joint_context_replacement_counterexample']
    assert b'11500000000' in joint['raw']
    doc=e.inspect_evidence(joint['raw'],joint['observations']['python'])
    assert doc['observation_snapshot']['result']['review']['synthetic_reference_matches'] is True
    assert all(doc[k] is False for k in (*e.FALSE_FIELDS,*e.EXTRA_FALSE))
    refusal=BY_SOURCE['baseline--original--declared_refusal']['observations']['python']
    assert refusal['result']['review']['synthetic_reference_matches'] is False
    base=ROOT/'reports/straight-followup-20261103-28/cpp-differential-audit'
    for name in ('unknown_result_native','late_reply_retains_unknown','edge--late_unupgraded_future_observation','edge--late_unupgraded_wrong_original_record'):
        ledger=json.loads((base/('preserve--'+name)/'python.json').read_bytes())['result']['review']['submissions']
        unknown=[x for x in ledger if x['outcome']=='unknown_outcome'];assert len(unknown)==1
        if name!='unknown_result_native':assert unknown[0]['reply_indices']==[8]
    late=ROOT/'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample'
    result=json.loads((late/'result.json').read_bytes())
    assert hashlib.sha256((late/'input.json').read_bytes()).hexdigest()==result['input_sha256']
    assert all(result[b][0]['accepted'] and result[b][0]['deadline_steady_s']>10.8 for b in ('python','receiver'))


def test_diagnostic_and_driver_have_no_protocol_reviewer_compiler_clock_or_process_calls():
    tree=ast.parse((ROOT/'tools/pending_location_evidence.py').read_bytes())
    assert {a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names}=={'hashlib','json','struct'}
    assert {n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)}=={'tools.pending_common_boundary_review'}
    driver=ast.parse((ROOT/'tools/pending_location_evidence_cases.py').read_bytes())
    imported={a.name for n in ast.walk(driver) if isinstance(n,ast.Import) for a in n.names}
    assert not imported.intersection({'subprocess','ctypes','time','socket'})
    forbidden={'Decode','ReviewEnvelope','review_envelope','decode_message','review_reference','review_reply','compile_probe','cpp_run','eval','exec','__import__'}
    for t in (tree,driver):
        assert not any(isinstance(n,ast.Call) and (isinstance(n.func,ast.Name) and n.func.id in forbidden or isinstance(n.func,ast.Attribute) and n.func.attr in forbidden) for n in ast.walk(t))


def test_evidence_corpus_and_spec_are_deterministic_and_not_a_new_cpp_implementation():
    assert CASES==d.corpus(ROOT) and len(CASES)==567 and len(LINKS)==16
    spec=json.loads((ROOT/'docs/PENDING_LOCATION_EVIDENCE_V1.json').read_bytes())
    assert spec['case_count']==567 and spec['implementation']=='python_only_no_cpp_diagnostic'
    assert spec['formal_compilations']==spec['portable_cpp_executions']==spec['old_protocol_validator_calls']==0


def test_bad_pins_precede_output_or_diagnostic_execution(tmp_path,monkeypatch):
    pin=tmp_path/'pins.json';pin.write_text('{}');output=tmp_path/'output'
    monkeypatch.setattr(e,'inspect_evidence',lambda *a:pytest.fail('diagnostic ran before pins'))
    with pytest.raises(ValueError,match='archive_plan_pin'):d.run(ROOT,output,pin,hashlib.sha256(pin.read_bytes()).hexdigest())
    assert not output.exists()


def test_oversized_snapshot_is_rejected_before_old_shape_checker_or_encoding(monkeypatch):
    monkeypatch.setattr(e,'valid',lambda *a:pytest.fail('unbounded snapshot reached shape checker'))
    f=next(f for f in CASES if f['case']=='invalid-snapshot--error_overbound')
    assert e.inspect_evidence(f['raw'],f['observations']['python'])['error']=='evidence_observation_shape'
