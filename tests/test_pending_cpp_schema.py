"""Portable standalone C++ schema decoding, not production/native admission."""
import hashlib
import json
from pathlib import Path
import subprocess

import pytest

from tools import pending_cpp_cases as c
from tools import pending_protocol_cases as old
from tools import pending_protocol_draft as p

ROOT=Path(__file__).resolve().parents[1]
PACKETS=c.corpus()


@pytest.fixture(scope='module')
def binary(tmp_path_factory):
    return c.compile_probe(ROOT,tmp_path_factory.mktemp('pending-cpp-schema'))


@pytest.mark.parametrize('name,raw',PACKETS,ids=[n for n,_ in PACKETS])
def test_fixed_packet_matches_python_schema_and_normalized_fields(binary,name,raw):
    expected=c.python_decode(raw);actual=c.cpp_decode(binary,raw)
    assert actual['failure'] is None,(name,actual)
    assert actual['document']['schema_consistent']==expected['schema_consistent']
    assert actual['document']['decoded']==expected['decoded']
    assert not actual['stderr']


@pytest.mark.parametrize('kind',['reserve','seal','result_ref','ack','status','stop'])
def test_all_missing_or_extra_keys_refused_without_compatibility_fallback(binary,kind):
    source={'reserve':'reserve','seal':'seal','result_ref':'result_reference','ack':'result_ack_native',
        'status':'disarmed_status','stop':'stop_no_frame'}[kind]
    f=old.fixture(source);raw=f['reply'] if kind=='ack' else f['message'];doc=json.loads(raw)
    for key in doc:
        bad=dict(doc);bad.pop(key);assert not c.cpp_decode(binary,old.wire(bad))['document']['schema_consistent']
    for key in ('version','issued_steady_s','expires_steady_s','v_m_s','omega_rad_s','motion_authority'):
        bad=dict(doc);bad[key]=0;assert not c.cpp_decode(binary,old.wire(bad))['document']['schema_consistent']


def test_schema_does_not_reject_context_mismatch_as_if_it_were_reference_review(binary):
    f=old.fixture('wrong_frame');assert c.cpp_decode(binary,f['message'])['document']['schema_consistent']
    with pytest.raises(p.DraftError):p.review_reference(f['message'],f['reserve'],f['seal'])


def test_expired_schema_and_repeat_are_not_admission_or_consumption(binary):
    raw=c.edge('expired_record_schema');a=c.cpp_decode(binary,raw);b=c.cpp_decode(binary,raw)
    assert a==b and a['document']['schema_consistent']
    assert a['document']['actual_exposure_wall_s'] is None
    assert all(a['document'][key] is False for key in p.FALSE_FIELDS)


def test_joint_context_replacement_still_has_no_authenticated_original(binary):
    changed=old.fixture('joint_context_replacement_counterexample')['message']
    ordinary=old.fixture('result_reference')['message']
    a=c.cpp_decode(binary,changed)['document'];b=c.cpp_decode(binary,ordinary)['document']
    assert a['schema_consistent'] and b['schema_consistent']
    assert a['decoded']['record']['deadline_ns']>b['decoded']['record']['deadline_ns']
    assert not a['provenance_authenticated'] and not a['production_guard_fixed']


def test_stop_is_schema_only_without_frame_or_capabilities(binary):
    raw=old.fixture('stop_no_frame')['message'];doc=c.cpp_decode(binary,raw)['document']
    assert doc['schema_consistent'] and doc['decoded']['record'] is None
    assert not doc['physical_stop_proved'] and not doc['runtime_pending_installed']


def test_cli_has_no_session_transport_or_operation_arguments(binary):
    result=subprocess.run([str(binary),'--live'],input=b'',capture_output=True,check=False,timeout=10)
    assert result.returncode==64 and not result.stdout


def test_header_has_no_io_clock_runtime_or_production_includes():
    source=(ROOT/'tools/pending_protocol_cpp_draft.hh').read_text()
    includes=[line for line in source.splitlines() if line.startswith('#include')]
    assert all('<' in line and '>' in line for line in includes)
    assert not any(word in source for word in ('std::cin','std::cout','std::chrono','sleep(',
        'ReceiverLease','MotorGate','gazebo::','gz::','system(','socket('))


def test_corpus_names_are_unique_and_old_context_cases_not_misclassified():
    assert len(PACKETS)==len({name for name,_ in PACKETS})
    assert c.python_decode(dict(PACKETS)['wrong_frame--message'])['schema_consistent']
    with pytest.raises(p.DraftError):c.edge('unknown')


def test_pin_failure_precedes_compilation_output_or_corpus(tmp_path,monkeypatch):
    pins=tmp_path/'pins.json';pins.write_text('{}');output=tmp_path/'out'
    monkeypatch.setattr(c,'compile_probe',lambda *a:pytest.fail('pins first'))
    monkeypatch.setattr(c,'corpus',lambda *a:pytest.fail('pins first'))
    for digest in ('0'*64,hashlib.sha256(pins.read_bytes()).hexdigest()):
        with pytest.raises(p.DraftError):c.run(ROOT,output,pins,digest)
    assert not output.exists()


def test_missing_compiler_is_failure_not_skip_or_python_fallback(tmp_path,monkeypatch):
    monkeypatch.setattr(c.shutil,'which',lambda name:None)
    with pytest.raises(p.DraftError,match='compiler_missing'):c.compile_probe(ROOT,tmp_path)


@pytest.mark.parametrize('stdout,returncode',[(b'{}',0),(b'null',0),(b'not-json',0),(b'[]',2)])
def test_invalid_child_output_never_counts_as_schema_agreement(monkeypatch,stdout,returncode):
    class Run:pass
    result=Run();result.stdout=stdout;result.stderr=b'';result.returncode=returncode
    monkeypatch.setattr(c.subprocess,'run',lambda *a,**k:result)
    assert c.cpp_decode('/not/executed',b'{}')['failure']


def test_child_timeout_is_retained_not_retried(monkeypatch):
    calls=[]
    def timeout(*a,**k):
        calls.append(1);raise subprocess.TimeoutExpired(a[0],10,output=b'partial',stderr=b'original')
    monkeypatch.setattr(c.subprocess,'run',timeout)
    result=c.cpp_decode('/not/executed',b'{}')
    assert result['failure']=='parser_timeout' and result['stdout']==b'partial' and result['stderr']==b'original'
    assert result['returncode'] is None and len(calls)==1
