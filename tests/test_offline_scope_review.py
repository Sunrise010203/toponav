"""Closed saved evidence only; no IO fixture or controller construction."""
from dataclasses import asdict, FrozenInstanceError
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from tools import offline_scope_review as module
from tools.offline_scope_review import FILES, BundleError, ScopeReview, parse, read_files, read_pins, review_scope
from tools.offline_scope_review_cases import CASES, OLD, PUBLIC_CASES, exercise, load, pins, public_inputs, scope_paths, sha

ROOT=Path(__file__).resolve().parents[1]
ORIGINALS=scope_paths(ROOT)


@pytest.fixture
def copied(tmp_path):
    source=ROOT/OLD/'lifecycle-audit/four/scope';out=tmp_path/'scope';out.mkdir()
    for name in FILES:shutil.copyfile(source/name,out/name)
    return out


def assert_no_authority(result):
    assert result.actual_exposure_wall_s is None
    assert not any(getattr(result,k) for k in module.AUTH)


@pytest.mark.parametrize('path',ORIGINALS,ids=lambda p:str(p.parent.relative_to(ROOT/OLD/'lifecycle-audit')))
def test_all_23_original_closed_scopes_keep_failures_and_unknown_exposure(path):
    out=path.parent;status=load(path);result=review_scope(out,trusted_files=pins(ROOT,out))
    assert_no_authority(result)
    assert result.recorded_status==status['status'] and result.trusted_bytes_match is True
    assert result.complete_evidence==result.evidence_consistent==status['artifact_files_complete']
    assert result.classification!='invalid_bundle',result
    if status['status'].endswith('failed') and status['artifact_files_complete']:assert result.classification=='recorded_failure'
    if not status['artifact_files_complete']:assert result.classification=='recorded_incomplete'
    assert result.adapted_receipts==status['receipt_adapted'] and result.rejected_receipts==status['receipt_rejected']


@pytest.mark.parametrize('name',CASES)
def test_saved_file_corruption_and_nonauthentication_counterexamples(name,tmp_path):
    record=exercise(ROOT,name,tmp_path/name);result=record['result']
    assert result['actual_exposure_wall_s'] is None and not any(result[k] for k in module.AUTH)
    assert record['input_files_unchanged']


@pytest.mark.parametrize('name,_relative',PUBLIC_CASES)
def test_independent_public_pngs_match_all_12_original_replay_inputs(name,_relative):
    path=ROOT/OLD/'lifecycle-audit'/('public-'+name)
    result=review_scope(path,trusted_files=pins(ROOT,path),external_inputs=public_inputs(ROOT,name))
    assert result.evidence_consistent and result.external_inputs_match and result.adapted_receipts==4
    assert_no_authority(result)


@pytest.mark.parametrize('payload,code',[
    (b'{"a":1,"a":2}','duplicate_json_key'),(b'{"a":NaN}','nonfinite_json_number'),
    (b'{"a":1e999}','nonfinite_json_number'),(b'\xff','invalid_json'),(b'{','invalid_json'),
    (b'['*30+b'0'+b']'*30,'json_structure_limit'),(b'"'+b'x'*2049+b'"','json_string_limit'),
    (b'['+b'0,'*1024+b'0]','json_array_limit'),(b'123456789012345678901234','json_integer_limit')])
def test_bounded_strict_json(payload,code):
    with pytest.raises(BundleError,match=code):parse(payload)


def test_symlink_file_refused_without_opening_target(copied,tmp_path):
    outside=tmp_path/'outside';outside.write_text('secret')
    (copied/'begin.json').unlink();(copied/'begin.json').symlink_to(outside)
    result=review_scope(copied)
    assert result.classification=='invalid_bundle' and result.diagnostics==('unreadable_or_unsafe_bundle',)
    assert outside.read_text()=='secret'


def test_symlink_directory_and_ancestor_refused(copied,tmp_path):
    link=tmp_path/'alias';link.symlink_to(copied,target_is_directory=True)
    assert review_scope(link).classification=='invalid_bundle'
    ancestor=tmp_path/'parent';ancestor.symlink_to(tmp_path,target_is_directory=True)
    assert review_scope(ancestor/'scope').classification=='invalid_bundle'


def test_parent_component_and_manifest_escape_never_followed(copied,tmp_path,monkeypatch):
    result=review_scope(copied/'..'/'scope')
    assert result.diagnostics==('parent_path_refused',)
    status=load(copied/'status.json');status['artifact_files']['../outside']='0'*64
    (copied/'status.json').write_text(json.dumps(status))
    opened=[];original=module.os.open
    def observe(path,flags,*args,**kwargs):
        assert not flags & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC)
        opened.append(str(path));return original(path,flags,*args,**kwargs)
    monkeypatch.setattr(module.os,'open',observe)
    result=review_scope(copied)
    assert result.diagnostics==('artifact_file_set',) and '../outside' not in opened


def test_fifo_refused_without_blocking(copied):
    (copied/'begin.json').unlink();os.mkfifo(copied/'begin.json')
    result=review_scope(copied)
    assert result.diagnostics==('nonregular_file:begin.json',)


def test_detect_changed_file_between_two_reads(copied,monkeypatch):
    original=module._read;counts={}
    def changed(fd,name):
        data,sig=original(fd,name);counts[name]=counts.get(name,0)+1
        if name=='begin.json' and counts[name]==2:data+=b' '
        return data,sig
    monkeypatch.setattr(module,'_read',changed)
    assert review_scope(copied).diagnostics==('bundle_changed:begin.json',)


def test_total_size_bound_is_separate_from_per_file(copied,monkeypatch):
    monkeypatch.setattr(module,'MAX_TOTAL_BYTES',1)
    assert review_scope(copied).diagnostics==('bundle_size_limit',)


@pytest.mark.parametrize('pins_value',[{},[],{'../outside':'0'*64},{n:'X'*64 for n in FILES}])
def test_external_pin_set_must_be_complete_fixed_names(copied,pins_value):
    result=review_scope(copied,trusted_files=pins_value)
    assert result.diagnostics==('trusted_file_set',) and not result.evidence_consistent


def test_full_external_pins_include_status_not_only_self_recorded_six(copied):
    trusted={n:sha(copied/n) for n in FILES}
    status=load(copied/'status.json');status['body_error']={'type':'RuntimeError','message':'new failure'};status['status']='body_failed'
    (copied/'status.json').write_text(json.dumps(status))
    unpinned=review_scope(copied);pinned=review_scope(copied,trusted_files=trusted)
    assert unpinned.evidence_consistent and unpinned.recorded_status=='body_failed'
    assert pinned.trusted_bytes_match is False and pinned.diagnostics==('trusted_hash_mismatch',)
    assert_no_authority(unpinned)


def test_extra_files_are_ignored_and_never_repaired(copied):
    extra=copied/'other.json';extra.write_bytes(b'bad extra')
    before={p.name:sha(p) for p in copied.iterdir()};result=review_scope(copied)
    assert result.evidence_consistent and before=={p.name:sha(p) for p in copied.iterdir()}
    assert 'other.json' not in dict(result.file_sha256)


def test_result_frozen_and_no_write_descriptors_leaked(copied):
    result=review_scope(copied)
    with pytest.raises(FrozenInstanceError):result.motion_authority=True
    for fd in Path('/proc/self/fd').iterdir():
        try:target=fd.resolve()
        except OSError:continue
        assert not target.is_relative_to(copied)


def test_reader_imports_no_runtime_fixture_controller_or_numpy_in_clean_process():
    command=[sys.executable,'-c',"import sys; import tools.offline_scope_review; assert not any(n.startswith(('simulation','roadvision','numpy','tools.capture_timing','tools.offline_scope_session')) for n in sys.modules)"]
    assert subprocess.run(command,cwd=ROOT,capture_output=True,check=False).returncode==0


def test_clock_or_thread_not_read_or_created_during_review(copied,monkeypatch):
    import time
    import threading
    def forbidden(*a,**k):raise AssertionError('runtime resource use')
    monkeypatch.setattr(time,'monotonic',forbidden);monkeypatch.setattr(time,'time',forbidden)
    monkeypatch.setattr(threading,'Thread',forbidden)
    assert review_scope(copied).evidence_consistent


def test_cli_stdout_only_no_input_writes_and_exit_codes(copied,tmp_path):
    pinfile=tmp_path/'pins.json';pinfile.write_text(json.dumps({n:sha(copied/n) for n in FILES}))
    before={p.name:sha(p) for p in copied.iterdir()}
    cmd=[sys.executable,'-m','tools.offline_scope_review','--scope',str(copied),'--pins',str(pinfile)]
    good=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,check=False)
    assert good.returncode==0 and json.loads(good.stdout)['evidence_consistent'] and not good.stderr
    (copied/'begin.json').unlink()
    bad=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,check=False)
    assert bad.returncode==2 and not json.loads(bad.stdout)['evidence_consistent']
    assert all(sha(copied/n)==h for n,h in before.items() if n!='begin.json')


def test_opt_in_not_imported_by_production():
    for directory in ('simulation','roadvision'):
        for path in (ROOT/directory).rglob('*.py'):
            assert 'offline_scope_review' not in path.read_text()


def test_compute_request_must_precede_original_selection_not_only_update(copied):
    events=[json.loads(line) for line in (copied/'events.jsonl').read_text().splitlines()]
    snapshot=load(copied/'snapshot.json');selected=snapshot['histories']['selections'][0]
    for event in events:
        if event['kind']=='compute_requested':event['observed_s']=(selected['next_frame_started_s']+selected['selected_observed_s'])/2
    (copied/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
    status=load(copied/'status.json');status['artifact_files']['events.jsonl']=sha(copied/'events.jsonl')
    (copied/'status.json').write_text(json.dumps(status))
    result=review_scope(copied)
    assert result.diagnostics==('compute_event_time',)


@pytest.mark.parametrize('value',['{}',None,bytearray(b'{}'),b' '*(module.MAX_FILE_BYTES+1)])
def test_parse_entry_itself_has_byte_limit(value):
    with pytest.raises(BundleError,match='json_byte_limit'):parse(value)


def test_external_input_duplicate_and_bad_digest_rejected(copied):
    obj=load(copied/'inputs.json')['inputs'][0]
    external=[dict(expected=obj['expected'],composite_sha256=None)]
    assert review_scope(copied,external_inputs=external).diagnostics==('external_input_digest',)
    external[0]['composite_sha256']=obj['composite_sha256']
    assert review_scope(copied,external_inputs=external+external).diagnostics==('external_input_set',)


def test_recorded_failure_not_relabelled_success_even_when_files_complete():
    path=ROOT/OLD/'lifecycle-audit/body_exception_while_callback/scope'
    result=review_scope(path,trusted_files=pins(ROOT,path))
    assert result.complete_evidence and result.evidence_consistent and result.classification=='recorded_failure'
    assert result.recorded_status=='body_failed' and result.adapted_receipts==0
