"""Opt-in offline requirements only, no capture/controller/live gate."""
from dataclasses import FrozenInstanceError
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from tools import offline_scope_coverage as module
from tools.offline_scope_coverage import review_coverage
from tools.offline_scope_coverage_cases import CASES, CONSISTENT, COUNTEREXAMPLES, exercise, make_requirements
from tools.offline_scope_index_cases import OLD, load, make_index, sha, write
from tools.offline_scope_pixels import review_pixels
from tools.offline_scope_pixels_cases import datasets, make_payload
from tools.offline_scope_review import AUTH, FILES
from tools.offline_scope_review_cases import PUBLIC_CASES

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def independent():return datasets(ROOT)


@pytest.fixture
def coverage_files(tmp_path,independent):
    root=tmp_path/'scopes';root.mkdir();paths=[];by_scope={}
    for pos,(case,_) in enumerate(PUBLIC_CASES[:2]):
        source=ROOT/OLD/'lifecycle-audit'/('public-'+case);dest=root/str(pos);dest.mkdir()
        for n in FILES:shutil.copyfile(source/n,dest/n)
        paths.append(dest);by_scope[load(dest/'scope.json')['scope_id']]=independent[case]
    index=make_index(root,paths);manifest=tmp_path/'index.json';write(manifest,index)
    payload=tmp_path/'pixels.json';write(payload,make_payload(index,sha(manifest),by_scope))
    requirements=tmp_path/'requirements.json';write(requirements,make_requirements(index,sha(manifest),sha(payload),{s:4 for s in by_scope}))
    return root,manifest,payload,requirements


def run(files,**kwargs):
    root,index,pixels,requirements=files
    options=dict(manifest_sha256=sha(index),payload=pixels,payload_sha256=sha(pixels),
        requirements=requirements,requirements_sha256=sha(requirements));options.update(kwargs)
    return review_coverage(root,index,**options)


def safe(result):
    for parent in (result,result.pixels,result.pixels.base):
        assert parent.actual_exposure_wall_s is None and not any(getattr(parent,k) for k in AUTH)
    children=[e.review for e in result.pixels.base.entries]+[e.review for e in result.pixels.entries if e.review is not None]
    for child in children:assert child.actual_exposure_wall_s is None and not any(getattr(child,k) for k in AUTH)


@pytest.mark.parametrize('name',CASES)
def test_explicit_saved_requirements_gaps_and_counterexamples(name,tmp_path,independent):
    row=exercise(ROOT,name,tmp_path/name,independent)
    assert row['input_files_unchanged'] and row['result']['requirements_satisfied']==(name in CONSISTENT)
    assert row['nonauthentication_or_coverage_counterexample']==(name in COUNTEREXAMPLES)
    assert not any(row['result'][k] for k in AUTH)


def test_requirement_success_preserves_exact_2i_default_result(coverage_files):
    result=run(coverage_files);root,index,pixels,_=coverage_files
    assert result.requirements_satisfied and result.required_scopes==result.locally_met_scopes==2
    assert result.pixels==review_pixels(root,index,manifest_sha256=sha(index),payload=pixels,payload_sha256=sha(pixels))
    assert all(e.independent_input_count==e.expected_input_count==4 and e.locally_met is True for e in result.entries)
    safe(result)


def test_matching_pixels_do_not_satisfy_wrong_count(coverage_files):
    req=coverage_files[3];doc=load(req);doc['scopes'][0]['expected_input_count']=3;req.write_text(json.dumps(doc))
    result=run(coverage_files)
    assert result.pixels.evidence_consistent and result.pixels.all_declared_inputs_match is True
    assert not result.requirements_satisfied and result.locally_met_scopes==1 and len(result.entries)==2
    assert result.entries[0].status=='required_input_count_mismatch' and result.entries[0].locally_met is False


def test_absent_optional_pixels_never_become_a_full_window_claim(coverage_files):
    payload=coverage_files[2];pixels=load(payload);pixels['scopes'][1]['external_inputs']=None;payload.write_text(json.dumps(pixels))
    req=coverage_files[3];doc=load(req);doc['payload_sha256']=sha(payload);doc['scopes'][1].update(required=False,expected_input_count=None);req.write_text(json.dumps(doc))
    result=run(coverage_files);assert result.requirements_satisfied
    assert result.pixels.pixel_coverage=='partial' and result.pixels.all_declared_inputs_match is None
    assert result.required_scopes==result.locally_met_scopes==1
    assert result.entries[1].status=='not_required' and result.entries[1].locally_met is None


def test_missing_required_material_is_not_a_false_pixel_mismatch(coverage_files):
    payload=coverage_files[2];pixels=load(payload);pixels['scopes'][0]['external_inputs']=None;payload.write_text(json.dumps(pixels))
    req=coverage_files[3];doc=load(req);doc['payload_sha256']=sha(payload);req.write_text(json.dumps(doc))
    result=run(coverage_files);assert result.pixels.evidence_consistent and not result.requirements_satisfied
    assert result.entries[0].status=='required_material_missing' and result.entries[0].locally_met is False
    assert result.pixels.entries[0].external_inputs_match is None and result.pixels.entries[0].review is None
    assert result.entries[1].locally_met is True


@pytest.mark.parametrize('pin',[None,True,1,'','X'*64])
def test_policy_pin_required_before_any_policy_read(coverage_files,monkeypatch,pin):
    def forbidden(*a,**k):raise AssertionError('untrusted policy must not be read')
    monkeypatch.setattr(module,'read_index',forbidden)
    result=run(coverage_files,requirements_sha256=pin)
    assert result.issues==('independent_coverage_digest_required',) and result.pixels.evidence_consistent
    assert len(result.entries)==2 and all(e.required is None and e.locally_met is None for e in result.entries)


@pytest.mark.parametrize('kind',['symlink','ancestor','fifo'])
def test_unsafe_policy_paths_refuse_without_blocking(coverage_files,tmp_path,kind):
    req=coverage_files[3]
    if kind=='symlink':
        path=tmp_path/'alias.json';path.symlink_to(req)
    elif kind=='ancestor':
        alias=tmp_path/'alias';alias.symlink_to(tmp_path,target_is_directory=True);path=alias/req.name
    else:path=tmp_path/'pipe';os.mkfifo(path)
    result=run(coverage_files,requirements=path)
    assert not result.requirements_satisfied and result.pixels.matched_scopes==2 and len(result.entries)==2


@pytest.mark.parametrize('raw',[b'{',b'\xff',b'{"x":NaN}',b'{"x":12345678901234567890123456789}',b'[]'])
def test_invalid_policy_json_cannot_erase_underlying_evidence(coverage_files,raw):
    coverage_files[3].write_bytes(raw);result=run(coverage_files)
    assert not result.requirements_satisfied and result.pixels.evidence_consistent and len(result.entries)==2


@pytest.mark.parametrize('field,stage',[(f,s) for f in ('index','pixels','requirements') for s in ('before','after')])
def test_outer_files_changed_around_requirement_evaluation_refuse_and_retain(coverage_files,monkeypatch,field,stage):
    target=coverage_files[{'index':1,'pixels':2,'requirements':3}[field]];original=module.read_index;calls=[]
    def change(path):
        raw=original(path)
        if Path(path)==target:
            calls.append(1)
            if len(calls)==(1 if stage=='before' else 2):return raw+b' '
        return raw
    monkeypatch.setattr(module,'read_index',change);result=run(coverage_files)
    assert not result.requirements_satisfied and len(result.entries)==2 and result.pixels.evidence_consistent
    assert result.locally_met_scopes==(0 if stage=='before' else 2)
    assert result.issues


def test_entire_policy_schema_before_evaluating_even_first_scope(coverage_files):
    req=coverage_files[3];doc=load(req);doc['scopes'][-1]['image_path']='../../do-not-open';req.write_text(json.dumps(doc))
    result=run(coverage_files)
    assert result.issues==('coverage_scope_schema',) and result.locally_met_scopes==0
    assert all(e.status=='not_evaluated_invalid_requirements' for e in result.entries)


@pytest.mark.parametrize('pin',[None,'0'*64])
def test_unpinned_index_never_promoted(coverage_files,pin):
    result=run(coverage_files,manifest_sha256=pin)
    assert not result.requirements_satisfied and not result.entries and not result.pixels.base.entries
    assert 'coverage_base_index_not_pinned' in result.issues


def test_expected_count_bound_is_enforced(coverage_files,monkeypatch):
    monkeypatch.setattr(module,'MAX_EXPECTED_INPUTS',3);result=run(coverage_files)
    assert result.issues==('coverage_required_count',) and result.locally_met_scopes==0


def test_original_23_have_local_public_success_but_no_parent_promotion(tmp_path,independent):
    from tools.offline_scope_index_cases import originals
    index=make_index(ROOT,originals(ROOT),original_root=ROOT);manifest=tmp_path/'index.json';write(manifest,index)
    by_scope={load(ROOT/OLD/'lifecycle-audit'/('public-'+n)/'scope.json')['scope_id']:independent[n] for n,_ in PUBLIC_CASES}
    payload=tmp_path/'pixels.json';write(payload,make_payload(index,sha(manifest),by_scope))
    req=tmp_path/'requirements.json';write(req,make_requirements(index,sha(manifest),sha(payload),{s:4 for s in by_scope}))
    result=review_coverage(ROOT,manifest,manifest_sha256=sha(manifest),payload=payload,payload_sha256=sha(payload),
        requirements=req,requirements_sha256=sha(req))
    assert result.required_scopes==result.locally_met_scopes==3 and not result.requirements_satisfied
    assert len(result.entries)==len(result.pixels.base.entries)==23
    assert sum(e.review.classification=='recorded_incomplete' for e in result.pixels.base.entries)==3
    assert sum(e.review.classification=='recorded_failure' for e in result.pixels.base.entries)==6
    assert result.issues==('pixel_index_not_consistent',);safe(result)


def test_no_runtime_import_clock_thread_or_decoder(coverage_files,monkeypatch):
    import time
    import threading
    def forbidden(*a,**k):raise AssertionError('runtime clock or thread')
    monkeypatch.setattr(time,'time',forbidden);monkeypatch.setattr(time,'monotonic',forbidden);monkeypatch.setattr(threading,'Thread',forbidden)
    assert run(coverage_files).requirements_satisfied
    code="import sys; import tools.offline_scope_coverage; assert not any(n.startswith(('PIL','numpy','simulation','roadvision','tools.offline_scope_session','tools.capture_timing')) for n in sys.modules)"
    p=subprocess.run([sys.executable,'-c',code],cwd=ROOT,capture_output=True,check=False);assert p.returncode==0,p.stderr


def test_frozen_result_and_all_input_fds_closed_without_writes(coverage_files):
    folder=coverage_files[1].parent;before={str(p):sha(p) for p in folder.rglob('*') if p.is_file()}
    result=run(coverage_files)
    with pytest.raises(FrozenInstanceError):result.requirements_satisfied=True
    assert before=={str(p):sha(p) for p in folder.rglob('*') if p.is_file()}
    for fd in Path('/proc/self/fd').iterdir():
        try:target=fd.resolve()
        except OSError:continue
        assert not target.is_relative_to(folder)


def test_cli_exit_codes(coverage_files):
    root,index,pixels,req=coverage_files
    cmd=[sys.executable,'-m','tools.offline_scope_coverage','--root',str(root),'--manifest',str(index),'--manifest-sha256',sha(index),
        '--payload',str(pixels),'--payload-sha256',sha(pixels),'--requirements',str(req),'--requirements-sha256',sha(req)]
    good=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,check=False)
    assert good.returncode==0 and json.loads(good.stdout)['requirements_satisfied'] and not good.stderr
    cmd[-1]='0'*64;bad=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,check=False)
    assert bad.returncode==2 and len(json.loads(bad.stdout)['pixels']['base']['entries'])==2 and not bad.stderr


def test_opt_in_not_imported_by_production():
    for folder in ('simulation','roadvision'):
        for p in (ROOT/folder).rglob('*.py'):assert 'offline_scope_coverage' not in p.read_text()
