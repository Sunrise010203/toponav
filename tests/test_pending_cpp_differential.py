"""Known differential observations are NOT repaired equivalence or permission."""
import ast
import copy
import hashlib
import json
from pathlib import Path

import pytest
from tools import pending_cpp_differential_cases as c
from tools import pending_cpp_difference_review as d
from tools import pending_protocol_draft as p

ROOT=Path(__file__).resolve().parents[1];REG=c.registry((ROOT/'docs/PENDING_CPP_DIFFERENCES.json').read_bytes());CASES=c.corpus(REG)


@pytest.fixture(scope='module')
def binaries(tmp_path_factory):return c.compile_probes(ROOT,tmp_path_factory.mktemp('pending-cpp-differential'))


@pytest.mark.parametrize('f',CASES,ids=[f['case'] for f in CASES])
def test_fixed_observations_match_explicit_registry_not_blanket_equivalence(binaries,f):
    py=c.python_fact(f);run=c.cpp_run(binaries[f['mode']],f)
    assert run['failure'] is None and not run['stderr'],(f['case'],run)
    cpp=c.cpp_fact(f['mode'],run['document']);review=c.evaluate_observation(f,py,cpp,run['failure'])
    assert review['registered_expectation_matches'],(f['case'],py,cpp,review)
    assert review['relation']==f['rule']['relation']
    assert review['exact_equal'] is (review['relation']=='exact')
    assert review['ledger_equal'] is not False
    assert review['actual_exposure_wall_s'] is None and all(review[k] is False for k in d.FALSE_FIELDS)


@pytest.fixture
def registered_trace():
    f=next(f for f in CASES if f['case']=='trace--record_overflow--0');py=c.python_fact(f);cpp=copy.deepcopy(py)
    for r in cpp['result'].values():r['violations'][0][1]=f['rule']['error_pair'][1]
    assert d.assess('trace',py,cpp,f['rule'])['registered_expectation_matches']
    return f,py,cpp


@pytest.mark.parametrize('mutation',['unknown_error','wrong_index','drop_send','change_failure','upgrade_unknown',
    'wrong_disposition','bool_time','motion','exposure','repeat_only','missing_field','changed_count'])
def test_registered_error_pair_cannot_hide_other_differences(registered_trace,mutation):
    f,py,cpp=registered_trace;r=cpp['result']['review']
    if mutation=='unknown_error':r['violations'][0][1]='new-unregistered-label'
    if mutation=='wrong_index':r['violations'][0][0]+=1
    if mutation=='drop_send':r['submissions'].pop()
    if mutation=='change_failure':r['failures'].append([1,'rewritten'])
    if mutation=='upgrade_unknown':r['submissions'][0]['outcome']='declared_confirmed'
    if mutation=='wrong_disposition':r['disposition']='completed_claim'
    if mutation=='bool_time':r['submissions'][0]['send_ns']=True
    if mutation=='motion':r['motion_authority']=True
    if mutation=='exposure':r['actual_exposure_wall_s']=0.0
    if mutation=='repeat_only':cpp['result']['repeated_review']['violations'].pop()
    if mutation=='missing_field':r.pop('violations')
    if mutation=='changed_count':r['event_count']+=1
    review=d.assess('trace',py,cpp,f['rule']);assert not review['registered_expectation_matches']


@pytest.mark.parametrize('value',[True,1.0,'1'])
def test_canonical_types_not_python_loose_equality(value):
    f=next(f for f in CASES if f['case']=='schema--unchanged--0');py=c.python_fact(f);cpp=copy.deepcopy(py)
    cpp['result']['request_id']=value
    assert not d.assess('schema',py,cpp,f['rule'])['registered_expectation_matches']


@pytest.mark.parametrize('value',[None,{},[],True,{'returned_result':False}])
def test_invalid_fact_shape_never_registered(value):
    f=CASES[0];py=c.python_fact(f)
    assert not d.assess(f['mode'],py,value,f['rule'])['registered_expectation_matches']


def test_unknown_mode_not_accepted_even_for_matching_refusals():
    f=dict(returned_result=False,consistent=False,result=None,error='bad')
    assert not d.assess('unknown',f,f,dict(relation='exact'))['registered_expectation_matches']


def test_registry_seed_and_authority_are_not_optional():
    for field,value in [('seed',0),('variants_per_family',4),('movement_authorized',True)]:
        reg=copy.deepcopy(REG);reg[field]=value
        with pytest.raises(p.DraftError):c.registry(json.dumps(reg).encode())


def test_corpus_deterministic_bounded_and_case_ids_unique():
    assert CASES==c.corpus(REG) and len(CASES)==247 and len({f['case'] for f in CASES})==247
    assert sum(f['rule']['relation']=='reference_envelope_refusal' for f in CASES)==4
    assert max(len(f['data']) for f in CASES)<=262145


def test_raw_invalid_utf8_not_replaced_by_unicode_string():
    f=next(f for f in CASES if f['case']=='reference--raw_invalid_utf8--0')
    assert b'\xff' in f['data'] and f['fixture']['message'].startswith(b'\xff')


def test_integer_limit_not_silently_changed(monkeypatch):
    monkeypatch.setattr(c.sys,'get_int_max_str_digits',lambda:0)
    with pytest.raises(p.DraftError,match='python_integer_digit_limit'):c.corpus(REG)


def test_pins_precede_output_compilers_or_cases(tmp_path,monkeypatch):
    pin=tmp_path/'pins.json';pin.write_text('{}');out=tmp_path/'out'
    monkeypatch.setattr(c,'compile_probes',lambda *a:pytest.fail('pins first'));monkeypatch.setattr(c,'corpus',lambda *a:pytest.fail('pins first'))
    for digest in ('0'*64,c.sha(pin)):
        with pytest.raises(p.DraftError):c.run(ROOT,out,pin,digest)
    assert not out.exists()


def test_missing_compiler_is_failure_not_skip_or_fallback(tmp_path,monkeypatch):
    monkeypatch.setattr(c.shutil,'which',lambda n:None)
    with pytest.raises(p.DraftError,match='compiler_missing'):c.compile_probes(ROOT,tmp_path)


def test_execution_failure_never_registered_as_difference():
    f=CASES[0];review=c.evaluate_observation(f,c.python_fact(f),None,'timeout')
    assert review['relation']=='execution_failure' and not review['registered_expectation_matches']


def test_comparator_is_pure_json_only():
    tree=ast.parse((ROOT/'tools/pending_cpp_difference_review.py').read_text())
    assert {a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names}=={'json'}
    assert not any(isinstance(n,ast.ImportFrom) for n in ast.walk(tree))
    calls={ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n,ast.Call)}
    assert not calls.intersection({'open','time.monotonic','subprocess.run','socket.socket'})


def test_old_both_backend_counterexample_not_reexecuted_or_fixed():
    folder=ROOT/'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample';v=json.loads((folder/'result.json').read_bytes())
    assert hashlib.sha256((folder/'input.json').read_bytes()).hexdigest()==v['input_sha256']
    assert all(v[b][0]['accepted'] and v[b][0]['deadline_steady_s']>10.8 for b in ('python','receiver'))
