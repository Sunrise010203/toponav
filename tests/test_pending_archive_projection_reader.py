"""Four sequential samples must never be promoted to snapshot or authority."""
import ast
import copy
import errno
import json
import os
from pathlib import Path
import pytest
from tools import pending_archive_projection_reader as r
from tools import pending_archive_projection_reader_cases as d

ROOT=Path(__file__).resolve().parents[1]
CASES=d.corpus(ROOT)
BY_NAME={c['case']:c for c in CASES}
fdcount=lambda:len(list(Path('/proc/self/fd').iterdir()))


@pytest.mark.parametrize('row',CASES,ids=[c['case'] for c in CASES])
def test_four_file_boundary_corpus(row):
    before=fdcount();a=d.evaluate_once(ROOT,row);b=d.evaluate_once(ROOT,row)
    assert d.check(row,a,b) and fdcount()==before


@pytest.mark.parametrize('value',[None,True,b'/tmp',Path('/tmp'),'','relative','/tmp/','/tmp//a','/tmp/../a',
    '/tmp/./a','/tmp/\0a','/tmp/\ud800','/'+'a/'*65+'b','/'+'x'*4096])
def test_bad_root_before_open(value,monkeypatch):
    monkeypatch.setattr(r,'_open',lambda *a:pytest.fail('invalid root opened'))
    assert r.read_projection_archive(value,{})['error']=='projection_reader_root_shape'


@pytest.mark.parametrize('value',[None,True,[],{},dict(profile='bad')])
def test_bad_claim_before_open(value,monkeypatch):
    monkeypatch.setattr(r,'_open',lambda *a:pytest.fail('invalid claim opened'))
    assert r.read_projection_archive('/tmp',value)['error']=='projection_reader_claim_shape'


@pytest.mark.parametrize('role',d.ROLES)
def test_paths_are_bounded_distinct_and_relative(tmp_path,monkeypatch,role):
    _,_,_,_,claim=d.fixture(tmp_path,'regular')
    monkeypatch.setattr(r,'_open',lambda *a:pytest.fail('invalid path opened'))
    for value in ('/etc/passwd','../outside','a//b','a/./b','a/../b','a\\b','档案','a\0b','x'*513,'a/'*32+'x'):
        assert r.read_projection_archive('/tmp',dict(claim,**{role+'_ref':value}))['error']=='projection_reader_claim_shape'
    other=next(k for k in d.ROLES if k!=role)
    assert r.read_projection_archive('/tmp',dict(claim,**{role+'_ref':claim[other+'_ref']}))['error']=='projection_reader_claim_shape'


@pytest.mark.parametrize('feature',['O_PATH','O_NOFOLLOW','O_DIRECTORY','O_CLOEXEC','O_NONBLOCK'])
def test_no_unsafe_platform_fallback(tmp_path,monkeypatch,feature):
    _,requested,_,_,claim=d.fixture(tmp_path,'regular');monkeypatch.delattr(r.os,feature)
    monkeypatch.setattr(r,'_open',lambda *a:pytest.fail('unsafe fallback'))
    assert r.read_projection_archive(requested,claim)['error']=='projection_reader_platform_unsupported'


@pytest.mark.parametrize('role',d.ROLES)
def test_read_error_cleanup(tmp_path,monkeypatch,role):
    root,requested,refs,_,claim=d.fixture(tmp_path,'regular');before=fdcount();read=os.read
    inode=(root/refs[role]).stat().st_ino
    def fail(fd,count):
        if os.fstat(fd).st_ino==inode:raise OSError(errno.EIO,'synthetic read failure')
        return read(fd,count)
    monkeypatch.setattr(r.os,'read',fail);out=r.read_projection_archive(requested,claim)
    assert (out['stage'],out['error'],out['filesystem_error_role'],out['os_errno'])==('projection_reader_refused','archive_read',role,errno.EIO)
    assert fdcount()==before


def test_close_error_refuses_and_never_retries(tmp_path,monkeypatch):
    _,requested,_,_,claim=d.fixture(tmp_path,'regular');close=os.close;calls=[];before=fdcount()
    def close_then_fail(fd):
        calls.append(fd);close(fd)
        if len(calls)==1:raise OSError(errno.EIO,'synthetic after release')
    monkeypatch.setattr(r.os,'close',close_then_fail);out=r.read_projection_archive(requested,claim)
    assert out['error']=='archive_close' and out['stage']=='projection_reader_refused'
    assert not out['local_regular_file_reads_complete'] and len(calls)==len(set(calls)) and fdcount()==before


@pytest.mark.parametrize('role',d.ROLES)
def test_final_refusal_overrides_nested_positive(role):
    out=d.evaluate_once(ROOT,BY_NAME['fixture--replace_after--'+role])['result']
    assert out['projection_result']['stage']=='projection_reference_checked'
    assert out['stage']=='projection_reader_refused' and not out['local_regular_file_reads_complete']
    assert all(f['final_recheck_agreed'] is None for f in out['files'])


@pytest.mark.parametrize('role',d.ROLES)
def test_first_recheck_refuses_before_contract(role):
    out=d.evaluate_once(ROOT,BY_NAME['fixture--replace_before_review--'+role])['result']
    assert out['projection_result'] is None and out['error']=='archive_path_changed'


def test_fd_flags_bounded_reads_and_no_nonregular_reopen(tmp_path,monkeypatch):
    _,requested,_,_,claim=d.fixture(tmp_path,'artifact_caps');opened=[];sizes=[]
    op=r.io._open;read=os.read
    def observed(name,flags,state,role,parent=None):
        fd=op(name,flags,state,role,parent);opened.append((name,flags))
        assert not os.get_inheritable(fd) and not flags&(os.O_CREAT|os.O_TRUNC|os.O_WRONLY|os.O_RDWR)
        if not (parent==state.get('proc_fd') and str(name).isdigit()):assert flags&os.O_NOFOLLOW
        return fd
    def bounded(fd,count):
        assert 1<=count<=65536;sizes.append(count);return read(fd,count)
    monkeypatch.setattr(r.io,'_open',observed);monkeypatch.setattr(r,'_open',observed);monkeypatch.setattr(r.os,'read',bounded)
    assert r.read_projection_archive(requested,claim)['local_regular_file_reads_complete'] and opened and sizes


@pytest.mark.parametrize('role',d.ROLES)
def test_fifo_never_read_open(tmp_path,monkeypatch,role):
    _,requested,_,_,claim=d.fixture(tmp_path,'fifo',role);op=r.io._open;pins=[]
    def inspected(name,flags,state,observed_role,parent=None):
        if observed_role==role and not flags&os.O_DIRECTORY:
            assert flags&os.O_PATH;pins.append(name)
        return op(name,flags,state,observed_role,parent)
    monkeypatch.setattr(r.io,'_open',inspected)
    assert r.read_projection_archive(requested,claim)['error']=='archive_file_type' and len(pins)==1


def test_unexpected_programming_exception_still_closes_descriptors(tmp_path,monkeypatch):
    _,requested,_,_,claim=d.fixture(tmp_path,'regular');before=fdcount()
    def fail(*args):raise RuntimeError('synthetic pure contract bug')
    monkeypatch.setattr(r.projection,'review_projection',fail)
    with pytest.raises(RuntimeError,match='synthetic'):r.read_projection_archive(requested,claim)
    assert fdcount()==before


def test_claim_and_facts_do_not_alias(tmp_path):
    _,requested,_,_,claim=d.fixture(tmp_path,'regular');before=copy.deepcopy(claim)
    out=r.read_projection_archive(requested,claim);expected=copy.deepcopy(out)
    out['claim_snapshot']['raw_ref']='changed';out['files'].clear()
    assert claim==before and r.read_projection_archive(requested,claim)==expected


def test_old_refusal_failures_and_runtime_gaps_retained():
    old=json.loads((ROOT/d.PREVIOUS/'SUMMARY.json').read_bytes())
    assert old['old_12266_catalog_refusal_preserved'] and old['old_2X_initial_1064_failures_preserved']
    assert old['source_error_differences_preserved']==66 and old['same_label_links']==16 and old['original_four_mismatches_retained']
    assert old['runtime_requirements_not_implemented']==14 and old['runtime_scenarios_planned_not_executed']==25
    assert r.io._read_regular is r._read_regular and r.io._recheck is r._recheck


def test_pin_mismatch_precedes_reader_and_output(tmp_path,monkeypatch):
    p=tmp_path/'pins';p.write_bytes(b'{}');out=tmp_path/'audit'
    monkeypatch.setattr(r,'read_projection_archive',lambda *a:pytest.fail('reader before pins'))
    with pytest.raises(ValueError,match='input_pin_mismatch'):d.run(ROOT,out,p,d.sha(p.read_bytes()))
    assert not out.exists()


def test_no_production_write_protocol_or_clock_calls():
    tree=ast.parse((ROOT/d.SOURCE_FILES[0]).read_bytes())
    assert {a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names}=={'os'}
    forbidden={'write','write_bytes','mkdir','unlink','rename','replace','socket','sleep','time','monotonic',
        'review_envelope','inspect_evidence','decode_message','read_archive_pair','eval','exec','__import__'}
    assert not any(isinstance(n,ast.Call) and (n.func.id if isinstance(n.func,ast.Name) else n.func.attr if isinstance(n.func,ast.Attribute) else '') in forbidden for n in ast.walk(tree))
    spec=json.loads((ROOT/'docs/PENDING_ARCHIVE_PROJECTION_READER_V1.json').read_bytes())
    assert spec['case_count']==len(CASES) and spec['old_flags_remain_false']==list(r.binding.FALSE_FIELDS)
