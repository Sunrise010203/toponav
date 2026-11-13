"""Independent saved RGB data only. No new callbacks or runtime sources."""
from dataclasses import asdict, FrozenInstanceError
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from tools import offline_scope_pixels as module
from tools.offline_scope_index import review_index
from tools.offline_scope_index_cases import OLD, load, make_index, originals, sha, write
from tools.offline_scope_pixels import review_pixels
from tools.offline_scope_pixels_cases import CASES, COUNTEREXAMPLES, datasets, exercise, make_payload
from tools.offline_scope_review import AUTH, FILES
from tools.offline_scope_review_cases import PUBLIC_CASES

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def independent():return datasets(ROOT)


@pytest.fixture
def fixture_pixels(tmp_path,independent):
    root=tmp_path/'scopes';root.mkdir();paths=[];by_scope={}
    for pos,(case,_) in enumerate(PUBLIC_CASES[:2]):
        dest=root/str(pos);dest.mkdir();source=ROOT/OLD/'lifecycle-audit'/('public-'+case)
        for n in FILES:shutil.copyfile(source/n,dest/n)
        paths.append(dest);by_scope[load(dest/'scope.json')['scope_id']]=independent[case]
    index=make_index(root,paths);manifest=tmp_path/'index.json';write(manifest,index)
    payload=tmp_path/'pixels.json';write(payload,make_payload(index,sha(manifest),by_scope))
    return root,manifest,payload


def run(fixture_pixels,**kwargs):
    root,manifest,payload=fixture_pixels
    opts=dict(manifest_sha256=sha(manifest),payload=payload,payload_sha256=sha(payload));opts.update(kwargs)
    return review_pixels(root,manifest,**opts)


def assert_safe(result):
    for item in (result,result.base):
        assert item.actual_exposure_wall_s is None and all(getattr(item,k) is False for k in AUTH)
    for item in result.entries:
        if item.review is not None:
            assert item.review.actual_exposure_wall_s is None and all(getattr(item.review,k) is False for k in AUTH)


@pytest.mark.parametrize('name',CASES)
def test_saved_data_cases_keep_all_entries_and_no_authority(name,tmp_path,independent):
    row=exercise(ROOT,name,tmp_path/name,independent)
    assert row['input_files_unchanged'] and len(row['result']['entries'])==len(row['result']['base']['entries'])==2
    assert row['nonauthentication_counterexample']==(name in COUNTEREXAMPLES)
    assert not any(row['result'][k] for k in AUTH)


def test_full_public_comparison_and_absence_have_distinct_meaning(fixture_pixels):
    good=run(fixture_pixels);assert good.evidence_consistent and good.all_declared_inputs_match is True
    assert good.pixel_coverage=='all' and good.compared_scopes==good.matched_scopes==2
    absent=run(fixture_pixels,payload=None,payload_sha256=None)
    assert absent.evidence_consistent and absent.pixel_coverage=='none' and absent.all_declared_inputs_match is None
    assert all(e.external_inputs_match is None and e.review is None for e in absent.entries)
    assert absent.base==good.base==review_index(fixture_pixels[0],fixture_pixels[1],manifest_sha256=sha(fixture_pixels[1]))
    assert_safe(good);assert_safe(absent)


def test_original_23_scopes_partial_pixels_cannot_promote_old_failures(tmp_path,independent):
    index=make_index(ROOT,originals(ROOT),original_root=ROOT);manifest=tmp_path/'index.json';write(manifest,index)
    by_scope={load(ROOT/OLD/'lifecycle-audit'/('public-'+n)/'scope.json')['scope_id']:independent[n] for n,_ in PUBLIC_CASES}
    payload=tmp_path/'pixels.json';write(payload,make_payload(index,sha(manifest),by_scope))
    result=review_pixels(ROOT,manifest,manifest_sha256=sha(manifest),payload=payload,payload_sha256=sha(payload))
    assert len(result.entries)==len(result.base.entries)==23 and not result.evidence_consistent
    assert result.compared_scopes==result.matched_scopes==3 and result.pixel_coverage=='partial'
    assert sum(e.status=='not_compared_no_material' for e in result.entries)==20
    assert sum(e.review.classification=='recorded_incomplete' for e in result.base.entries)==3
    assert sum(e.review.classification=='recorded_failure' for e in result.base.entries)==6
    assert result.all_declared_inputs_match is None;assert_safe(result)


@pytest.mark.parametrize('pin',[None,True,1,'','X'*64])
def test_payload_pin_required_before_reading_payload_but_base_retained(fixture_pixels,monkeypatch,pin):
    def forbidden(*a,**k):raise AssertionError('untrusted payload read')
    monkeypatch.setattr(module,'read_index',forbidden)
    result=run(fixture_pixels,payload_sha256=pin)
    assert result.issues==('independent_pixel_digest_required',) and len(result.base.entries)==2
    assert not result.evidence_consistent


def test_payload_digest_without_file_is_refused(fixture_pixels):
    result=run(fixture_pixels,payload=None)
    assert result.issues==('pixel_digest_without_payload',) and not result.evidence_consistent


def test_no_extra_child_review_until_entire_payload_valid(fixture_pixels,monkeypatch):
    payload=fixture_pixels[2];doc=load(payload);doc['scopes'][-1]['external_inputs'][0]['expected']['rgb_path']='../forbidden'
    payload.write_text(json.dumps(doc))
    def forbidden(*a,**k):raise AssertionError('partial schema must not start extra review')
    monkeypatch.setattr(module,'review_scope',forbidden)
    result=run(fixture_pixels)
    assert result.issues==('pixel_expected_schema',) and len(result.entries)==2 and result.compared_scopes==0


@pytest.mark.parametrize('kind',['symlink','ancestor','fifo'])
def test_unsafe_payload_files_refused_without_following_or_blocking(fixture_pixels,tmp_path,kind):
    payload=fixture_pixels[2]
    if kind=='symlink':
        path=tmp_path/'alias.json';path.symlink_to(payload)
    elif kind=='ancestor':
        alias=tmp_path/'alias';alias.symlink_to(tmp_path,target_is_directory=True);path=alias/payload.name
    else:path=tmp_path/'pipe';os.mkfifo(path)
    result=run(fixture_pixels,payload=path)
    assert not result.evidence_consistent and len(result.base.entries)==2 and not result.compared_scopes


@pytest.mark.parametrize('raw',[b'{',b'\xff',b'{"x":NaN}',b'{"x":12345678901234567890123456789}',b'[]'])
def test_invalid_json_payload_retains_base(fixture_pixels,raw):
    fixture_pixels[2].write_bytes(raw);result=run(fixture_pixels)
    assert not result.evidence_consistent and len(result.base.entries)==2 and result.compared_scopes==0


def test_payload_change_after_comparisons_keeps_matches_but_refuses(fixture_pixels,monkeypatch):
    original=module.read_index;calls=[]
    def change(path):
        raw=original(path)
        if Path(path)==fixture_pixels[2]:
            calls.append(1)
            if len(calls)==2:return raw+b' '
        return raw
    monkeypatch.setattr(module,'read_index',change)
    result=run(fixture_pixels)
    assert result.issues==('pixel_payload_changed_during_review',) and result.matched_scopes==2
    assert not result.evidence_consistent and result.all_declared_inputs_match is None


@pytest.mark.parametrize('stage',['before','after'])
def test_index_changed_around_extra_reviews_refused(fixture_pixels,monkeypatch,stage):
    original=module.read_index;calls=[]
    def change(path):
        raw=original(path)
        if Path(path)==fixture_pixels[1]:
            calls.append(1)
            if len(calls)==(1 if stage=='before' else 2):return raw+b' '
        return raw
    monkeypatch.setattr(module,'read_index',change)
    result=run(fixture_pixels)
    assert not result.evidence_consistent and len(result.entries)==2
    assert ('index_changed_before_pixels' if stage=='before' else 'index_changed_during_pixels') in result.issues


def test_external_reader_failure_is_null_not_invented_false(fixture_pixels):
    payload=fixture_pixels[2];doc=load(payload);doc['scopes'][0]['external_inputs'][0]['composite_sha256']='0'*64
    payload.write_text(json.dumps(doc));result=run(fixture_pixels)
    assert result.base.evidence_consistent and not result.evidence_consistent
    first=result.entries[0];assert first.status=='comparison_refused' and first.external_inputs_match is None
    assert first.review.classification=='invalid_bundle' and 'external_pixel_or_identity_mismatch' in first.review.diagnostics
    assert result.entries[1].external_inputs_match is True and len(result.entries)==2


def test_input_frame_order_not_view_order_may_differ(fixture_pixels):
    payload=fixture_pixels[2];doc=load(payload);doc['scopes'][0]['external_inputs'].reverse();payload.write_text(json.dumps(doc))
    assert run(fixture_pixels).all_declared_inputs_match is True


def test_per_scope_input_bound_checked_before_extra_reads(fixture_pixels,monkeypatch):
    monkeypatch.setattr(module,'MAX_INPUTS_PER_SCOPE',3)
    result=run(fixture_pixels);assert result.issues==('pixel_input_limit',) and result.compared_scopes==0


def test_no_clock_thread_decoder_or_runtime_imports(fixture_pixels,monkeypatch):
    import time
    import threading
    def forbidden(*a,**k):raise AssertionError('clock/thread used')
    monkeypatch.setattr(time,'time',forbidden);monkeypatch.setattr(time,'monotonic',forbidden);monkeypatch.setattr(threading,'Thread',forbidden)
    assert run(fixture_pixels).evidence_consistent
    code="import sys; import tools.offline_scope_pixels; assert not any(n.startswith(('PIL','numpy','simulation','roadvision','tools.offline_scope_session','tools.capture_timing')) for n in sys.modules)"
    p=subprocess.run([sys.executable,'-c',code],cwd=ROOT,capture_output=True,check=False)
    assert p.returncode==0,p.stderr


def test_frozen_result_no_open_input_fds_and_no_write_side_effect(fixture_pixels):
    root=fixture_pixels[1].parent;before={str(p):sha(p) for p in root.rglob('*') if p.is_file()}
    result=run(fixture_pixels)
    with pytest.raises(FrozenInstanceError):result.motion_authority=True
    assert before=={str(p):sha(p) for p in root.rglob('*') if p.is_file()}
    for fd in Path('/proc/self/fd').iterdir():
        try:target=fd.resolve()
        except OSError:continue
        assert not target.is_relative_to(root)


def test_cli_exit_codes_and_unchanged_files(fixture_pixels):
    root,manifest,payload=fixture_pixels
    cmd=[sys.executable,'-m','tools.offline_scope_pixels','--root',str(root),'--manifest',str(manifest),
        '--manifest-sha256',sha(manifest),'--payload',str(payload),'--payload-sha256',sha(payload)]
    good=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,check=False)
    assert good.returncode==0 and json.loads(good.stdout)['all_declared_inputs_match'] is True and not good.stderr
    cmd[-1]='0'*64;bad=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,check=False)
    assert bad.returncode==2 and len(json.loads(bad.stdout)['base']['entries'])==2 and not bad.stderr


def test_opt_in_reader_not_referenced_by_production():
    for folder in ('roadvision','simulation'):
        for p in (ROOT/folder).rglob('*.py'):assert 'offline_scope_pixels' not in p.read_text()


def test_child_changes_after_base_preserve_base_and_all_other_entries(fixture_pixels,monkeypatch):
    original=module.review_scope;calls=[]
    def changed(path,**kwargs):
        calls.append(str(path))
        if len(calls)==1:
            p=Path(path)/'begin.json';p.write_bytes(p.read_bytes()+b' ')
        return original(path,**kwargs)
    monkeypatch.setattr(module,'review_scope',changed)
    result=run(fixture_pixels)
    assert result.base.evidence_consistent and result.base.entries[0].review.classification=='receipt_only'
    assert not result.evidence_consistent and len(calls)==len(result.entries)==2
    assert result.entries[0].status=='comparison_refused' and result.entries[1].external_inputs_match is True
    assert any(i.startswith('scope_changed_between_pixel_reviews:') for i in result.issues)


@pytest.mark.parametrize('pin',[None,'0'*64])
def test_unpinned_base_never_promoted_by_valid_payload(fixture_pixels,pin):
    result=run(fixture_pixels,manifest_sha256=pin)
    assert not result.evidence_consistent and not result.base.entries and not result.entries
    assert result.issues==('base_index_not_pinned',) and result.all_declared_inputs_match is None
