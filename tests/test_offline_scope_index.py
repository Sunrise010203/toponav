"""Offline explicit windows only; no runtime IO, callbacks or controllers."""
from dataclasses import asdict, FrozenInstanceError
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from tools import offline_scope_index as module
from tools.offline_scope_index import review_index, validate_index
from tools.offline_scope_index_cases import CASES, OLD, exercise, load, make_index, originals, sha, write
from tools.offline_scope_review import AUTH, BundleError, FILES

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture
def fixture_index(tmp_path):
    scopes=tmp_path/'scopes';scopes.mkdir();paths=[]
    for i in (0,1):
        source=ROOT/OLD/f'lifecycle-audit/reattach/scope-{i}';out=scopes/f'scope-{i}';out.mkdir()
        for name in FILES:shutil.copyfile(source/name,out/name)
        paths.append(out)
    doc=make_index(scopes,paths);manifest=tmp_path/'index.json';write(manifest,doc)
    return scopes,manifest,doc


def run_fixture(fixture_index):
    scopes,manifest,_=fixture_index
    return review_index(scopes,manifest,manifest_sha256=sha(manifest))


def save(fixture_index,doc):
    fixture_index[1].write_text(json.dumps(doc,indent=2)+'\n')


def no_authority(result):
    assert result.actual_exposure_wall_s is None and all(getattr(result,k) is False for k in AUTH)
    for entry in result.entries:
        assert entry.review.actual_exposure_wall_s is None and all(getattr(entry.review,k) is False for k in AUTH)


@pytest.mark.parametrize('name',CASES)
def test_saved_file_index_corruption_continuity_and_limits(name,tmp_path):
    record=exercise(ROOT,name,tmp_path/name)
    assert record['input_files_unchanged'] and record['no_runtime_callbacks']
    result=record['result'];assert result['actual_exposure_wall_s'] is None and not any(result[k] for k in AUTH)


def test_original_two_scope_reattach_is_one_consistent_declared_window(tmp_path):
    paths=[ROOT/OLD/f'lifecycle-audit/reattach/scope-{i}' for i in (0,1)]
    manifest=tmp_path/'index.json';write(manifest,make_index(ROOT,paths,original_root=ROOT))
    result=review_index(ROOT,manifest,manifest_sha256=sha(manifest))
    assert result.evidence_consistent and result.verified_edges==1 and len(result.entries)==2
    assert result.declared_sessions==1 and result.declared_scopes==2 and result.coverage=='declared_window_only'
    assert [e.incoming_edge_verified for e in result.entries]==[False,True]
    no_authority(result)


def test_all_23_original_entries_retained_and_three_incomplete_fail_entire_index(tmp_path):
    manifest=tmp_path/'index.json';write(manifest,make_index(ROOT,originals(ROOT),original_root=ROOT))
    result=review_index(ROOT,manifest,manifest_sha256=sha(manifest))
    assert not result.evidence_consistent and len(result.entries)==23 and result.declared_sessions==22
    assert sum(e.review.classification=='recorded_incomplete' for e in result.entries)==3
    assert sum(e.review.classification=='recorded_failure' for e in result.entries)==6
    assert sum(e.review.classification=='recorded_rejection' for e in result.entries)==2
    no_authority(result)


@pytest.mark.parametrize('pin',[None,True,1,'','X'*64])
def test_independent_index_pin_is_mandatory_before_any_reads(fixture_index,monkeypatch,pin):
    def forbidden(*a,**k):raise AssertionError('must not read unpinned index or scope')
    monkeypatch.setattr(module,'read_index',forbidden)
    result=review_index(fixture_index[0],fixture_index[1],manifest_sha256=pin)
    assert result.issues[0].code=='independent_index_digest_required' and not result.entries


def test_whole_manifest_schema_checked_before_opening_any_scope(fixture_index,monkeypatch):
    doc=fixture_index[2];doc['sessions'][0]['scopes'][-1]['path']='../outside';save(fixture_index,doc)
    def forbidden(*a,**k):raise AssertionError('unsafe index must not read even first scope')
    monkeypatch.setattr(module,'review_scope',forbidden)
    result=run_fixture(fixture_index)
    assert result.issues[0].code=='index_relative_path' and not result.entries


@pytest.mark.parametrize('path',['/absolute','../outside','x/../y','./x','x/.','x//y','x/','x\\y','x\x00y'])
def test_unsafe_index_paths_rejected_before_scope_read(fixture_index,path):
    doc=fixture_index[2];doc['sessions'][0]['scopes'][0]['path']=path;save(fixture_index,doc)
    result=run_fixture(fixture_index)
    assert result.issues[0].code=='index_relative_path'


def test_symlink_root_and_ancestor_refused_and_both_entries_kept(fixture_index,tmp_path):
    root,manifest,_=fixture_index;alias=tmp_path/'alias';alias.symlink_to(root,target_is_directory=True)
    result=review_index(alias,manifest,manifest_sha256=sha(manifest))
    assert not result.evidence_consistent and len(result.entries)==2
    assert all(e.review.classification=='invalid_bundle' for e in result.entries)


def test_symlink_index_and_fifo_refused_without_blocking(fixture_index,tmp_path):
    root,manifest,_=fixture_index;link=tmp_path/'alias.json';link.symlink_to(manifest)
    result=review_index(root,link,manifest_sha256=sha(manifest))
    assert result.issues[0].code=='unreadable_or_unsafe_index'
    fifo=tmp_path/'pipe.json';os.mkfifo(fifo)
    result=review_index(root,fifo,manifest_sha256='0'*64)
    assert result.issues[0].code.startswith('nonregular_file:')


def test_index_change_after_children_keeps_results_but_fails_entire_window(fixture_index,monkeypatch):
    original=module.read_index;calls=[]
    def change(path):
        raw=original(path);calls.append(1)
        return raw if len(calls)==1 else raw+b' '
    monkeypatch.setattr(module,'read_index',change)
    result=run_fixture(fixture_index)
    assert len(result.entries)==2 and result.issues[-1].code=='index_changed_during_review' and not result.evidence_consistent


def test_metadata_must_equal_same_independent_child_pins(fixture_index,monkeypatch):
    original=module.read_files;calls=[]
    def change(path):
        raw=original(path);calls.append(str(path));raw['scope.json']+=b' ';return raw
    monkeypatch.setattr(module,'read_files',change)
    result=run_fixture(fixture_index)
    assert len(result.entries)==2 and any(i.code=='scope_metadata_unavailable_or_changed' for i in result.issues)


def test_child_failure_never_short_circuits_or_passes_successful_subset(fixture_index):
    root,_,_=fixture_index;(root/'scope-0/scope.json').unlink()
    result=run_fixture(fixture_index)
    assert len(result.entries)==2 and result.entries[1].review.evidence_consistent
    assert not result.evidence_consistent and result.entries[0].review.classification=='invalid_bundle'
    assert not result.entries[1].incoming_edge_verified


def test_index_size_bound(fixture_index):
    fixture_index[1].write_bytes(b' '*(module.MAX_INDEX_BYTES+1))
    assert run_fixture(fixture_index).issues[0].code=='index_byte_limit'


@pytest.mark.parametrize('field',['initial_counts','terminal_counts'])
def test_bool_is_not_a_cumulative_counter(fixture_index,field):
    doc=fixture_index[2];doc['sessions'][0][field][0][1]=True;save(fixture_index,doc)
    assert run_fixture(fixture_index).issues[0].code=='index_counts_schema'


def test_session_and_total_scope_bounds_before_reading(fixture_index,monkeypatch):
    doc=copy.deepcopy(fixture_index[2]);doc['sessions']*=33
    with pytest.raises(BundleError,match='index_session_limit'):validate_index(doc)
    doc=copy.deepcopy(fixture_index[2]);doc['sessions'][0]['scopes']*=17
    with pytest.raises(BundleError,match='index_scope_limit'):validate_index(doc)
    monkeypatch.setattr(module,'MAX_SCOPES',1)
    with pytest.raises(BundleError,match='index_total_scope_limit'):validate_index(fixture_index[2])


def test_initial_predecessor_not_silently_used_to_extend_window(fixture_index):
    doc=fixture_index[2];entry=doc['sessions'][0]['scopes'][0];entry.update(predecessor_scope_id='outside-window',predecessor_snapshot_sha256='0'*64)
    save(fixture_index,doc);result=run_fixture(fixture_index)
    assert any(i.code=='window_start_has_predecessor' for i in result.issues) and not result.evidence_consistent


def test_frozen_result_and_no_open_input_fds_after_review(fixture_index):
    result=run_fixture(fixture_index)
    with pytest.raises(FrozenInstanceError):result.motion_authority=True
    for fd in Path('/proc/self/fd').iterdir():
        try:target=fd.resolve()
        except OSError:continue
        assert not target.is_relative_to(fixture_index[1].parent)


def test_no_clock_threads_or_runtime_imports(fixture_index,monkeypatch):
    import time
    import threading
    def forbidden(*a,**k):raise AssertionError('runtime clock/thread use')
    monkeypatch.setattr(time,'time',forbidden);monkeypatch.setattr(time,'monotonic',forbidden);monkeypatch.setattr(threading,'Thread',forbidden)
    assert run_fixture(fixture_index).evidence_consistent
    command=[sys.executable,'-c',"import sys; import tools.offline_scope_index; assert not any(n.startswith(('simulation','roadvision','numpy','tools.capture_timing','tools.offline_scope_session')) for n in sys.modules)"]
    assert subprocess.run(command,cwd=ROOT,capture_output=True,check=False).returncode==0


def test_cli_stdout_and_exit_codes_without_input_changes(fixture_index):
    root,manifest,_=fixture_index
    before={str(p):sha(p) for p in manifest.parent.rglob('*') if p.is_file()}
    command=[sys.executable,'-m','tools.offline_scope_index','--root',str(root),'--manifest',str(manifest),'--manifest-sha256',sha(manifest)]
    good=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,check=False)
    assert good.returncode==0 and json.loads(good.stdout)['evidence_consistent'] and not good.stderr
    command[-1]='0'*64;bad=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,check=False)
    assert bad.returncode==2 and not json.loads(bad.stdout)['evidence_consistent'] and not bad.stderr
    assert before=={str(p):sha(p) for p in manifest.parent.rglob('*') if p.is_file()}


def test_opt_in_not_imported_by_production():
    for directory in ('simulation','roadvision'):
        for path in (ROOT/directory).rglob('*.py'):assert 'offline_scope_index' not in path.read_text()
