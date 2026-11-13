"""Registered boundary observations are not validator fixes or motion authority."""
import ast
import copy
import hashlib
import itertools
import json
from pathlib import Path
import pytest
from tools import pending_common_boundary_cases as d
from tools import pending_common_boundary_review as r

ROOT=Path(__file__).resolve().parents[1];CASES=d.corpus(ROOT)
MULTI=[f for f in CASES if f['family']=='priority' and len(f['faults'])>1]


@pytest.fixture(scope='module')
def observations(tmp_path_factory):
    binary=d.prior.compile_probe(ROOT,tmp_path_factory.mktemp('pending-common-boundary'));out={}
    for f in CASES:
        py=d.c.review_envelope(f['data']);py2=d.c.review_envelope(f['data'])
        cpp=d.prior.cpp_run(binary,f['data']);cpp2=d.prior.cpp_run(binary,f['data'])
        assert all(not x['stderr'] and x['returncode']==(0 if x['document']['consistent'] else 2) for x in (cpp,cpp2))
        out[f['case']]=(py,cpp['document'],py2,cpp2['document'])
    return out


@pytest.mark.parametrize('f',CASES,ids=[f['case'] for f in CASES])
def test_fixed_boundary_stages_raw_errors_complete_results_and_repeated_calls(observations,f):
    verdict=r.assess(f,*observations[f['case']])
    assert verdict['expectation_matches'],(f['case'],observations[f['case']],verdict)
    assert verdict['authority_safe'] and verdict['repeated_calls_equal']
    assert verdict['actual_exposure_wall_s'] is None and all(verdict[k] is False for k in r.FALSE_FIELDS)


@pytest.mark.parametrize('f',MULTI,ids=[f['case'] for f in MULTI])
def test_removing_first_fault_matches_successor_contract_without_field_localization_claim(observations,f):
    mask=[x for x in f['faults'] if x!=f['first_fault']]
    name='priority--'+str(f['variant'])+'--'+'+'.join(mask)
    successor=next(x for x in CASES if x['case']==name)
    assert successor['first_fault']==min(mask,key=d.FAULTS.index)
    assert d.FAULTS.index(successor['first_fault'])>d.FAULTS.index(f['first_fault'])
    assert r.assess(f,*observations[f['case']])['expectation_matches']
    assert r.assess(successor,*observations[name])['expectation_matches']
    assert all(doc['consistent'] is False for doc in observations[name])


@pytest.mark.parametrize('mutation',['stage','domain_acceptance','bool_consistent','drop_result','change_kind','change_repeat',
    'extra','motion','exposure','inner_motion','inner_exposure','wrong_error','new_error_pair'])
def test_comparator_cannot_hide_stage_result_type_authority_or_repeat_changes(mutation):
    f=next(x for x in CASES if x['case']=='baseline--original--repeat_snapshot_counterexample')
    py=d.c.review_envelope(f['data']);cpp=copy.deepcopy(py);repeat=copy.deepcopy(py);meta=copy.deepcopy(f)
    if mutation=='stage':cpp['stage']='domain_refused'
    if mutation=='domain_acceptance':cpp['domain_accepted']=False
    if mutation=='bool_consistent':cpp['consistent']=1
    if mutation=='drop_result':cpp['result']=None
    if mutation=='change_kind':cpp['result']['review']['message_kind']='stop'
    if mutation=='change_repeat':cpp['result']['repeated_review']['classification']='frame_free_stop_schema_only'
    if mutation=='extra':cpp['new_field']=False
    if mutation=='motion':cpp['motion_authority']=True
    if mutation=='exposure':cpp['actual_exposure_wall_s']=0
    if mutation=='inner_motion':cpp['result']['review']['motion_authority']=True
    if mutation=='inner_exposure':cpp['result']['review']['actual_exposure_wall_s']=0
    if mutation=='wrong_error':cpp['error']='unregistered'
    if mutation=='new_error_pair':meta['error_pair']=['new','new']
    assert not r.assess(meta,py,cpp,py,repeat)['expectation_matches']


@pytest.mark.parametrize('value',[None,True,[],{},dict(stage='domain_refused')])
def test_invalid_observation_or_expectation_is_not_registered(value):
    f=CASES[0];py=d.c.review_envelope(f['data'])
    assert not r.assess(f,py,value,py,py)['expectation_matches']
    assert not r.assess(value,py,py,py,py)['expectation_matches']


def test_registry_reproducibility_pair_coverage_and_no_mutually_exclusive_writes():
    assert d.corpus(ROOT)==CASES and len(CASES)==532 and len({f['case'] for f in CASES})==532
    masks=d.priority_masks();assert len(masks)==87
    assert len([m for m in masks if len(m)==1])==11
    assert {m for m in masks if len(m)==2}=={m for m in itertools.combinations(d.FAULTS,2) if d.compatible(m)}
    assert len([m for m in masks if len(m)==3])==24 and all(d.compatible(m) for m in masks)
    spec=json.loads((ROOT/'docs/PENDING_COMMON_BOUNDARY_AUDIT.json').read_bytes())
    assert spec['seed']==d.SEED==2026110330 and spec['case_count']==532 and spec['fault_order']==list(d.FAULTS)


def test_baseline_inputs_preserve_every_original_byte_and_66_inner_differences():
    oldplan=json.loads((ROOT/'reports/straight-followup-20261103-29/PLAN.json').read_bytes())['cases']
    base=[f for f in CASES if f['family']=='sealed_2T'];assert len(base)==243
    for f in base:
        old=oldplan[f['case'].removeprefix('baseline--')]
        assert hashlib.sha256(f['data']).hexdigest()==old['input_sha256']
        assert f['expected_stage']==old['metadata']['expected_stage'] and f['error_pair']==old['metadata']['error_pair']
    assert sum(f['error_pair'] is not None and f['error_pair'][0]!=f['error_pair'][1] for f in base)==66
    summary=json.loads((ROOT/'reports/straight-followup-20261103-28/SUMMARY.json').read_bytes())
    assert summary['known_wrapper_outcome_differences']==4 and not summary['all_results_exactly_equivalent']


def test_python_domain_refusal_does_not_invoke_inner_message_reviewer(monkeypatch):
    def forbidden(*a,**k):pytest.fail('domain refusal invoked old reviewer')
    for name in ('decode_message','review_reference','review_reply'):monkeypatch.setattr(d.c.p,name,forbidden)
    for f in CASES:
        if f['family']!='sealed_2T' and f['expected_stage']=='domain_refused':assert d.c.review_envelope(f['data'])['stage']=='domain_refused'


def test_total_cap_is_not_a_claim_of_canonical_cap_sized_controls():
    for delta in (-1,0,1):
        f=next(f for f in CASES if f['case']=='total-length--'+str(delta))
        assert len(f['data'])==d.c.LIMIT+delta
        assert d.c.review_envelope(f['data'])['error']==('domain_byte_bound' if delta==1 else 'domain_controls')


def test_archived_unknown_late_and_original_joint_resign_are_not_rerun_or_upgraded():
    prior=ROOT/'reports/straight-followup-20261103-28/cpp-differential-audit'
    for name in ('unknown_result_native','late_reply_retains_unknown','edge--late_unupgraded_future_observation','edge--late_unupgraded_wrong_original_record'):
        doc=json.loads((prior/('preserve--'+name)/'python.json').read_bytes())['result']['review']
        unknown=[s for s in doc['submissions'] if s['outcome']=='unknown_outcome'];assert len(unknown)==1
        if name!='unknown_result_native':assert unknown[0]['reply_indices']==[8]
    base=ROOT/'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample'
    doc=json.loads((base/'result.json').read_bytes())
    assert doc['input_sha256']==hashlib.sha256((base/'input.json').read_bytes()).hexdigest()
    assert all(doc[b][0]['accepted'] and doc[b][0]['deadline_steady_s']>10.8 for b in ('python','receiver'))


def test_pure_observation_checker_imports_only_json_and_never_executes_io():
    tree=ast.parse((ROOT/'tools/pending_common_boundary_review.py').read_bytes())
    assert [a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names]==['json']
    assert not any(isinstance(n,ast.ImportFrom) for n in ast.walk(tree))
    assert not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id in ('open','eval','exec','__import__') for n in ast.walk(tree))


def test_invalid_pins_fail_before_output_or_compile(tmp_path,monkeypatch):
    pin=tmp_path/'pin.json';pin.write_text('{}');out=tmp_path/'out'
    monkeypatch.setattr(d.prior,'compile_probe',lambda *a:pytest.fail('compiled before pins'))
    with pytest.raises(d.c.DomainError,match='input_pin_schema'):d.run(ROOT,out,pin,hashlib.sha256(pin.read_bytes()).hexdigest())
    assert not out.exists()
