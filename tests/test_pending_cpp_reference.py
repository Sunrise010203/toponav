"""Fixed portable C++ caller-snapshot claims, NOT actual backend execution."""
import copy
import hashlib
import json
from pathlib import Path
import struct
import subprocess

import pytest
from tools import pending_cpp_reference_cases as c
from tools import pending_protocol_cases as old
from tools import pending_protocol_draft as p

ROOT=Path(__file__).resolve().parents[1]
CASES=c.corpus()


@pytest.fixture(scope='module')
def binary(tmp_path_factory):
    return c.compile_probe(ROOT,tmp_path_factory.mktemp('pending-cpp-reference'))


@pytest.mark.parametrize('f,valid',CASES,ids=[f['case'] for f,_ in CASES])
def test_contextual_outcomes_and_typed_success_fields_match(binary,f,valid):
    py=c.python_review(f);cpp=c.cpp_review(binary,c.envelope(f))
    assert cpp['failure'] is None,(f['case'],cpp)
    assert not cpp['stderr']
    assert cpp['document']['consistent'] is py['consistent'] is valid
    assert c.canonical(cpp['document']['result'])==c.canonical(py['result'])
    assert all(cpp['document'][k] is False for k in p.FALSE_FIELDS)


@pytest.mark.parametrize('source',['joint_context_replacement_counterexample','repeat_snapshot_counterexample',
    'expired_schema_is_not_admission','declared_refusal','stop_no_frame','stop_ack_after_deadline'])
def test_limits_are_explicit_not_runtime_evidence(binary,source):
    f=old.fixture(source);a=c.cpp_review(binary,c.envelope(f));b=c.cpp_review(binary,c.envelope(f))
    assert a==b and a['document']['consistent']
    review=a['document']['result']['review']
    assert review['actual_exposure_wall_s'] is None and all(review[k] is False for k in p.FALSE_FIELDS)
    if source=='declared_refusal':assert review['synthetic_reference_matches'] is False
    if source=='repeat_snapshot_counterexample':assert review==a['document']['result']['repeated_review']


@pytest.mark.parametrize('raw',[b'',b'TPQ1',b'BAD!',b'TPQ1'+struct.pack('>I',4097),
    b'TPQ1'+struct.pack('>I',0xffffffff),b'x'*(c.ENVELOPE_LIMIT+1)])
def test_malformed_envelope_never_admits(binary,raw):
    result=c.cpp_review(binary,raw)
    assert result['failure'] is None and not result['document']['consistent']


def test_envelope_truncation_trailing_null_and_length_fields(binary):
    f=old.fixture('stop_no_frame');raw=c.envelope(f)
    for bad in (raw[:-1],raw+b'x',raw[:4]+struct.pack('>I',0)+raw[8:],
        raw[:4]+struct.pack('>I',8194)+raw[8:]):
        doc=c.cpp_review(binary,bad)['document'];assert not doc['consistent']
    for role in ('reserve','seal'):
        f[role]=b'\xff'
    assert c.cpp_review(binary,c.envelope(f))['document']['consistent']


def test_envelope_preserves_raw_invalid_utf8_and_empty_vs_absent(binary):
    f=old.fixture('result_reference');f['seal']=b'\xff'
    assert b'\xff' in c.envelope(f)
    assert not c.cpp_review(binary,c.envelope(f))['document']['consistent']
    for value in (b'',None):
        f['seal']=value;assert not c.cpp_review(binary,c.envelope(f))['document']['consistent']


def test_exact_message_byte_bound(binary):
    f=old.fixture('reserve');f['message']+=b' '*(8192-len(f['message']))
    assert c.cpp_review(binary,c.envelope(f))['document']['consistent']
    f['message']+=b' ';assert not c.cpp_review(binary,c.envelope(f))['document']['consistent']


def test_same_instance_two_calls_and_joint_replacement_are_not_consumption(binary):
    f=old.fixture('repeat_snapshot_counterexample');doc=c.cpp_review(binary,c.envelope(f))['document']
    assert doc['result']['review']==doc['result']['repeated_review']
    changed=old.fixture('joint_context_replacement_counterexample');doc=c.cpp_review(binary,c.envelope(changed))['document']
    assert doc['consistent'] and not doc['result']['review']['provenance_authenticated']
    changed['reserve']=f['reserve'];assert not c.cpp_review(binary,c.envelope(changed))['document']['consistent']


def test_no_session_transport_cli(binary):
    r=subprocess.run([str(binary),'--native'],capture_output=True,input=b'',timeout=10,check=False)
    assert r.returncode==64 and not r.stdout


def test_core_is_pure_and_old_header_only_dependency():
    source=(ROOT/'tools/pending_protocol_cpp_reference.hh').read_text()
    assert [s for s in source.splitlines() if s.startswith('#include')]==[
        '#include "pending_protocol_cpp_draft.hh"','#include <optional>']
    assert not any(s in source for s in ('std::cin','std::cout','std::chrono','sleep(',
        'static Json','ReceiverLease','MotorGate','gazebo::','system(','socket('))


def test_names_unique_and_counterexample_preserved():
    assert len(CASES)==len({f['case'] for f,_ in CASES})
    folder=ROOT/'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample'
    result=json.loads((folder/'result.json').read_bytes())
    assert hashlib.sha256((folder/'input.json').read_bytes()).hexdigest()==result['input_sha256']
    assert all(result[b][0]['accepted'] and result[b][0]['deadline_steady_s']>10.8 for b in ('python','receiver'))


def test_bad_pins_precede_output_compile_or_corpus(tmp_path,monkeypatch):
    pins=tmp_path/'pins.json';pins.write_text('{}');out=tmp_path/'output'
    monkeypatch.setattr(c,'compile_probe',lambda *a:pytest.fail('pins first'))
    monkeypatch.setattr(c,'corpus',lambda *a:pytest.fail('pins first'))
    for digest in ('0'*64,c.sha(pins)):
        with pytest.raises(p.DraftError):c.run(ROOT,out,pins,digest)
    assert not out.exists()


def test_missing_compiler_fails_without_skip_or_fallback(tmp_path,monkeypatch):
    monkeypatch.setattr(c.shutil,'which',lambda n:None)
    with pytest.raises(p.DraftError,match='compiler_missing'):c.compile_probe(ROOT,tmp_path)


@pytest.mark.parametrize('stdout,code',[(b'{}',0),(b'null',0),(b'[]',2),(b'not-json',0)])
def test_bad_child_output_retained(monkeypatch,stdout,code):
    monkeypatch.setattr(c.subprocess,'run',lambda *a,**k:subprocess.CompletedProcess(a[0],code,stdout,b'original'))
    r=c.cpp_review('/not/executed',b'raw');assert r['failure'] and r['stderr']==b'original'


def test_child_timeout_has_no_retry(monkeypatch):
    calls=[]
    def timeout(*a,**k):
        calls.append(1);raise subprocess.TimeoutExpired(a[0],10,output=b'partial',stderr=b'original')
    monkeypatch.setattr(c.subprocess,'run',timeout);r=c.cpp_review('/not/executed',b'raw')
    assert r['failure']=='review_timeout' and r['returncode'] is None
    assert r['stdout']==b'partial' and r['stderr']==b'original' and len(calls)==1
