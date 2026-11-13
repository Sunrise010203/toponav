"""Portable C++ caller-authored transcript claims, NOT native/runtime proof."""
import hashlib
import json
from pathlib import Path
import subprocess

import pytest
from tools import pending_trace_cpp_cases as c
from tools import pending_trace_cases as old
from tools import pending_trace_review as t
from tools import pending_protocol_draft as p

ROOT=Path(__file__).resolve().parents[1];CASES=c.corpus()


@pytest.fixture(scope='module')
def binary(tmp_path_factory):return c.compile_probe(ROOT,tmp_path_factory.mktemp('pending-cpp-trace'))


@pytest.mark.parametrize('name,raw,expected',CASES,ids=[n for n,_,_ in CASES])
def test_fixed_schema_semantic_results_and_full_typed_ledgers_match(binary,name,raw,expected):
    py=c.python_review(raw);cpp=c.cpp_review(binary,raw)
    assert cpp['failure'] is None,(name,cpp)
    assert not cpp['stderr']
    assert cpp['document']['schema_valid'] is py['schema_valid']
    assert cpp['document']['consistent'] is py['consistent'] is expected
    assert c.canonical(cpp['document']['result'])==c.canonical(py['result'])
    assert all(cpp['document'][k] is False for k in t.FALSE_FIELDS)


@pytest.mark.parametrize('name',['wrong_send_actor','wrong_send_target','duplicate_send','retry_after_unknown','wrong_session'])
def test_failed_send_attempts_remain_in_ledger(binary,name):
    raw=old.fixture(name);doc=c.cpp_review(binary,raw)['document']
    assert not doc['consistent'] and doc['schema_valid']
    assert len(doc['result']['review']['submissions'])==sum(e['action']=='send' for e in json.loads(raw)['events'])


@pytest.mark.parametrize('name',['unknown_reserve_native','unknown_result_native','late_reply_retains_unknown'])
def test_unknown_never_upgraded_by_late_claim(binary,name):
    doc=c.cpp_review(binary,old.fixture(name))['document'];review=doc['result']['review']
    assert doc['consistent'] and review['disposition']=='failed_claim'
    unknown=next(s for s in review['submissions'] if s['outcome']=='unknown_outcome')
    if name=='late_reply_retains_unknown':assert len(unknown['reply_indices'])==1
    assert review['failures'] and not review['runtime_result_consumed']


@pytest.mark.parametrize('name',['hash_failure','save_failure','compute_failure','cancel_before_compute','declared_refusal'])
def test_original_error_stop_cancel_diagnostic_is_only_a_transcript(binary,name):
    doc=c.cpp_review(binary,old.fixture(name))['document'];review=doc['result']['review']
    assert review['disposition']=='failed_claim' and review['failures']
    assert not review['physical_stop_proved'] and not review['independent_enforcement_observed']


@pytest.mark.parametrize('name',['normal','stop_no_frame','stop_after_old_expiry','finish_after_replaced_old_hold','joint_context_replacement'])
def test_repeat_same_process_same_output_is_not_consumption_or_authentication(binary,name):
    raw=old.fixture(name);doc=c.cpp_review(binary,raw)['document'];a=doc['result']['review'];b=doc['result']['repeated_review']
    assert doc['consistent'] and a==b
    assert all(a[k] is False for k in t.FALSE_FIELDS) and a['actual_exposure_wall_s'] is None


def test_failed_prefix_cannot_be_repaired_by_success_suffix(binary):
    doc=c.cpp_review(binary,old.fixture('success_suffix_after_failure'))['document']
    assert not doc['consistent'] and doc['result']['review']['failures']
    assert doc['result']['review']['disposition']=='rejected_transcript'


def test_schema_refusal_is_distinct_from_semantic_rejection(binary):
    a=c.cpp_review(binary,b'\xff');b=c.cpp_review(binary,old.fixture('wrong_send_actor'))
    assert a['returncode']==2 and not a['document']['schema_valid'] and a['document']['result'] is None
    assert b['returncode']==3 and b['document']['schema_valid'] and b['document']['result']['review']['violations']


def test_no_session_transport_operation_arguments(binary):
    r=subprocess.run([str(binary),'--live'],input=b'',capture_output=True,timeout=10,check=False)
    assert r.returncode==64 and not r.stdout


def test_pure_headers_no_io_clock_runtime_or_production_dependency():
    for name in ('pending_trace_cpp_json.hh','pending_trace_cpp_review.hh'):
        s=(ROOT/'tools'/name).read_text()
        assert not any(w in s for w in ('std::cin','std::cout','std::chrono','sleep(','ReceiverLease','MotorGate','gazebo::','gz::','system(','socket('))
        includes=[line for line in s.splitlines() if line.startswith('#include')]
        assert includes==['#include "'+('pending_protocol_cpp_reference.hh' if name.endswith('json.hh') else 'pending_trace_cpp_json.hh')+'"']


def test_corpus_unique_and_old_two_backend_counterexample_unchanged():
    assert len(CASES)==len({n for n,_,_ in CASES})
    folder=ROOT/'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample';r=json.loads((folder/'result.json').read_bytes())
    assert hashlib.sha256((folder/'input.json').read_bytes()).hexdigest()==r['input_sha256']
    assert all(r[b][0]['accepted'] and r[b][0]['deadline_steady_s']>10.8 for b in ('python','receiver'))


def test_pins_fail_before_output_compile_or_corpus(tmp_path,monkeypatch):
    pins=tmp_path/'pins.json';pins.write_text('{}');out=tmp_path/'output'
    monkeypatch.setattr(c,'compile_probe',lambda *a:pytest.fail('pins first'));monkeypatch.setattr(c,'corpus',lambda *a:pytest.fail('pins first'))
    for digest in ('0'*64,c.sha(pins)):
        with pytest.raises(p.DraftError):c.run(ROOT,out,pins,digest)
    assert not out.exists()


def test_compiler_missing_is_failure_not_skip_or_fallback(tmp_path,monkeypatch):
    monkeypatch.setattr(c.shutil,'which',lambda n:None)
    with pytest.raises(p.DraftError,match='compiler_missing'):c.compile_probe(ROOT,tmp_path)


@pytest.mark.parametrize('stdout,code',[(b'{}',0),(b'null',0),(b'[]',2),(b'not-json',3)])
def test_malformed_child_output_retained(monkeypatch,stdout,code):
    monkeypatch.setattr(c.subprocess,'run',lambda *a,**k:subprocess.CompletedProcess(a[0],code,stdout,b'original'))
    r=c.cpp_review('/not/executed',b'raw');assert r['failure'] and r['stderr']==b'original'


def test_timeout_retains_partial_without_retry(monkeypatch):
    calls=[]
    def timeout(*a,**k):
        calls.append(1);raise subprocess.TimeoutExpired(a[0],10,output=b'partial',stderr=b'original')
    monkeypatch.setattr(c.subprocess,'run',timeout);r=c.cpp_review('/not/executed',b'raw')
    assert r['failure']=='trace_timeout' and r['returncode'] is None and len(calls)==1
    assert r['stdout']==b'partial' and r['stderr']==b'original'


def test_compile_timeout_preserves_partial_and_returns_no_fallback(tmp_path,monkeypatch):
    def timeout(*a,**k):raise subprocess.TimeoutExpired(a[0],60,output=b'partial-compiler',stderr=b'original-compiler')
    monkeypatch.setattr(c.subprocess,'run',timeout)
    with pytest.raises(p.DraftError,match='compile_failed'):c.saved_run(['/not/executed'],tmp_path,'compile',60)
    assert (tmp_path/'compile.stdout.bin').read_bytes()==b'partial-compiler'
    assert (tmp_path/'compile.stderr.bin').read_bytes()==b'original-compiler'
    assert json.loads((tmp_path/'compile.exit.json').read_bytes())==dict(returncode=None,error='timeout')
