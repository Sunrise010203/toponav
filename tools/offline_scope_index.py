"""READ-ONLY explicit offline scope index, never a runtime owner or motion gate.

Checks one caller-frozen declared window, not a complete real session archive.
No discovery, IO/controller/probe construction, current clock, worker or token.
Local index/file SHA256 equality is NOT authentication or cross-run anti-replay.
"""
import argparse
from dataclasses import asdict, dataclass, field
import hashlib
from pathlib import Path

from tools.offline_scope_review import (
    AUTH, FILES, KINDS, BundleError, ScopeReview, _directory, _read, check, parse,
    read_files, review_scope, same, shape, uint,
)
from tools.time_provider_contract import digest, identity

MAX_SESSIONS=32
MAX_SCOPES=64
MAX_INDEX_BYTES=256*1024
FORMAT='offline_scope_index_v1'
CONTEXT=('session_id','config_id','provider_id','host_clock_id','host_boot_id','source_kind','cameras')


@dataclass(frozen=True)
class IndexIssue:
    code: str
    session_id: str | None = None
    scope_id: str | None = None


@dataclass(frozen=True)
class IndexedScope:
    session_id: str
    scope_id: str
    path: str
    review: ScopeReview
    cross_scope_diagnostics: tuple[str,...]
    incoming_edge_verified: bool


@dataclass(frozen=True)
class IndexReview:
    classification: str
    evidence_consistent: bool
    manifest_bytes_match: bool | None
    manifest_sha256: str | None
    declared_sessions: int
    declared_scopes: int
    verified_edges: int
    issues: tuple[IndexIssue,...]
    entries: tuple[IndexedScope,...]
    coverage: str = field(default='declared_window_only',init=False)
    actual_exposure_wall_s: None = field(default=None,init=False)
    provenance_authenticated: bool = field(default=False,init=False)
    shadow_admission_authorized: bool = field(default=False,init=False)
    native_live_observation_authorized: bool = field(default=False,init=False)
    motion_authority: bool = field(default=False,init=False)


def read_index(path):
    """Read the explicitly selected index twice, including NOFOLLOW ancestors."""
    path=Path(path)
    try:
        with _directory(path.parent) as fd:
            raw,signature=_read(fd,path.name)
            check(len(raw)<=MAX_INDEX_BYTES,'index_byte_limit')
            second,again=_read(fd,path.name)
            check(raw==second and signature==again,'index_changed')
            return raw
    except OSError as exc:raise BundleError('unreadable_or_unsafe_index') from exc


def _counts(value):
    check(type(value) is list and len(value)==4 and all(type(pair) is list and len(pair)==2
          and type(pair[0]) is str and uint(pair[1]) for pair in value),'index_counts_schema')
    check([pair[0] for pair in value]==list(KINDS),'index_counts_schema')


def _relative(path):
    check(type(path) is str and 0<len(path)<=512 and '\x00' not in path and '\\' not in path
          and all(part not in ('','.','..') for part in path.split('/')),'index_relative_path')


def validate_index(doc):
    """Validate the entire explicit path/pin schema before opening any scope."""
    shape(doc,'format coverage sessions','index_schema')
    check(doc['format']==FORMAT and doc['coverage']=='declared_window_only','index_format_or_coverage')
    sessions=doc['sessions'];check(type(sessions) is list and 1<=len(sessions)<=MAX_SESSIONS,'index_session_limit')
    session_ids=set();scope_ids=set();paths=set();total=0
    for session in sessions:
        shape(session,' '.join(CONTEXT)+' initial_counts terminal_counts scopes','index_session_schema')
        check(all(identity(session[key]) for key in CONTEXT if key not in ('cameras','source_kind')),'index_context_identity')
        sid=session['session_id'];check(sid not in session_ids,'duplicate_session_id');session_ids.add(sid)
        check(session['source_kind'] in ('isolated_synthetic_delivery','replay_delivery'),'index_offline_source_only')
        check(session['host_clock_id']=='CLOCK_MONOTONIC','index_host_clock')
        cameras=session['cameras'];check(type(cameras) is list and 4<=len(cameras)<=6,'index_camera_schema')
        for camera in cameras:
            shape(camera,'view sensor_id device_clock_id device_boot_id mapping_sha256','index_camera_schema')
            check(identity(camera['view']) and identity(camera['sensor_id']) and all(camera[key] is None
                  for key in ('device_clock_id','device_boot_id','mapping_sha256')),'index_receipt_only_cameras')
        names=[camera['view'] for camera in cameras]
        check(names[:4]==['front','left','right','back'] and len(set(names))==len(names) and set(names[4:])<={'up','down'},'index_camera_set')
        _counts(session['initial_counts']);_counts(session['terminal_counts'])
        scopes=session['scopes'];check(type(scopes) is list and 1<=len(scopes)<=32,'index_scope_limit')
        total+=len(scopes);check(total<=MAX_SCOPES,'index_total_scope_limit')
        for scope in scopes:
            shape(scope,'scope_id path files predecessor_scope_id predecessor_snapshot_sha256','index_scope_schema')
            check(identity(scope['scope_id']),'index_scope_identity');_relative(scope['path'])
            check(scope['scope_id'] not in scope_ids,'duplicate_scope_id');scope_ids.add(scope['scope_id'])
            check(scope['path'] not in paths,'duplicate_scope_path');paths.add(scope['path'])
            check(type(scope['files']) is dict and set(scope['files'])==set(FILES)
                  and all(digest(value) for value in scope['files'].values()),'index_seven_file_pins')
            check(scope['predecessor_scope_id'] is None or identity(scope['predecessor_scope_id']),'index_predecessor_identity')
            check(scope['predecessor_snapshot_sha256'] is None or digest(scope['predecessor_snapshot_sha256']),'index_predecessor_digest')
    return total


def _metadata(root,entry,result):
    """Bind cross-scope metadata to the very same seven independently pinned bytes."""
    raw=read_files(root/entry['path']);hashes={name:hashlib.sha256(data).hexdigest() for name,data in raw.items()}
    check(hashes==entry['files']==dict(result.file_sha256),'scope_changed_between_reviews')
    return {name:parse(raw[name]) for name in ('begin.json','scope.json','snapshot.json','status.json')}


def _context(meta):
    begin=meta['begin.json'];spec=begin['spec']
    return {key:begin['source_kind'] if key=='source_kind' else spec[key] for key in CONTEXT}


def _edge(previous,current):
    """No clock-domain subtraction: compare raw boundaries in declared same domain."""
    codes=[];left=previous['scope.json'];right=current['scope.json']
    if left['closed_s']>right['started_s']:codes.append('scope_overlap_or_reversed')
    if not same(previous['snapshot.json']['counts'],dict(right['counts_before'])):codes.append('counter_continuity_gap')
    if current['snapshot.json']['observer_errors']<previous['snapshot.json']['observer_errors']:codes.append('observer_error_counter_decreased')
    if previous['status.json']['fixture_error'] is not None:codes.append('continued_after_fixture_error')
    # A ring may discard older records; only records still retained in BOTH are comparable.
    for kind in KINDS:
        before={row['record_id']:row for row in previous['snapshot.json']['histories'][kind]}
        after={row['record_id']:row for row in current['snapshot.json']['histories'][kind]}
        if any(not same(before[rid],after[rid]) for rid in before.keys()&after.keys()):
            codes.append('retained_history_changed:'+kind)
    return codes


def review_index(root,manifest,*,manifest_sha256):
    """Mandatory externally held index digest, plus seven pins per declared scope.

    All declared entries are retained on child/cross-scope failures; the entire
    index remains NOT consistent, never a successful subset. No persistent state
    is kept across calls. A valid result may contain original recorded failures.
    """
    raw=None;actual=None;matched=None;sessions=0;total=0;entries=[];issues=[];edges=0
    try:
        check(digest(manifest_sha256),'independent_index_digest_required')
        raw=read_index(manifest);actual=hashlib.sha256(raw).hexdigest();matched=actual==manifest_sha256
        check(matched,'index_digest_mismatch')
        doc=parse(raw);total=validate_index(doc);sessions=len(doc['sessions']);root=Path(root).absolute()
        for session in doc['sessions']:
            sid=session['session_id'];previous=None;previous_declared=None
            for index,declared in enumerate(session['scopes']):
                result=review_scope(root/declared['path'],trusted_files=declared['files']);codes=[];meta=None;edge_ok=False
                if not result.evidence_consistent:
                    codes.append('scope_evidence_not_complete')
                if (result.evidence_consistent or result.classification=='recorded_incomplete') and result.trusted_bytes_match is True:
                    try:meta=_metadata(root,declared,result)
                    except (BundleError,KeyError,TypeError,ValueError,OverflowError,RecursionError):codes.append('scope_metadata_unavailable_or_changed')
                if meta is not None:
                    if meta['scope.json']['scope_id']!=declared['scope_id']:codes.append('declared_scope_id_mismatch')
                    if not same(_context(meta),{key:session[key] for key in CONTEXT}):codes.append('session_context_mismatch')
                if index==0:
                    if declared['predecessor_scope_id'] is not None or declared['predecessor_snapshot_sha256'] is not None:codes.append('window_start_has_predecessor')
                    if meta is None:codes.append('window_start_unverifiable')
                    elif not same(meta['scope.json']['counts_before'],session['initial_counts']):codes.append('initial_count_mismatch')
                else:
                    if declared['predecessor_scope_id']!=previous_declared['scope_id']:codes.append('predecessor_scope_link')
                    if previous is None or meta is None:codes.append('continuity_edge_unverifiable')
                    else:
                        if declared['predecessor_snapshot_sha256']!=previous['scope.json']['snapshot_sha256']:codes.append('predecessor_snapshot_link')
                        codes.extend(_edge(previous,meta))
                    edge_ok=not codes and previous is not None and meta is not None
                    if edge_ok:edges+=1
                if index==len(session['scopes'])-1:
                    if meta is None:codes.append('window_end_unverifiable')
                    elif not same(meta['snapshot.json']['counts'],dict(session['terminal_counts'])):codes.append('terminal_count_mismatch')
                # Only a locally complete, correctly bound predecessor may anchor a later edge.
                previous=meta if result.evidence_consistent and not codes else None;previous_declared=declared
                entries.append(IndexedScope(sid,declared['scope_id'],declared['path'],result,tuple(codes),edge_ok))
                issues.extend(IndexIssue(code,sid,declared['scope_id']) for code in codes)
        try:check(read_index(manifest)==raw,'index_changed_during_review')
        except (BundleError,OSError,ValueError):issues.append(IndexIssue('index_changed_during_review'))
    except (BundleError,KeyError,TypeError,ValueError,OverflowError,RecursionError) as exc:
        code=str(exc) if isinstance(exc,BundleError) else 'invalid_index_schema'
        issues.append(IndexIssue(code))
    consistent=not issues and len(entries)==total and total>0
    return IndexReview('consistent_declared_window' if consistent else 'incomplete_or_invalid_index',
        consistent,matched,actual,sessions,total,edges,tuple(issues),tuple(entries))


def main():
    import json
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True);parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--manifest-sha256',required=True);args=parser.parse_args()
    result=review_index(args.root,args.manifest,manifest_sha256=args.manifest_sha256)
    print(json.dumps(asdict(result),allow_nan=False));return 0 if result.evidence_consistent else 2


if __name__=='__main__':raise SystemExit(main())
