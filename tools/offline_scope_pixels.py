"""Opt-in pure-data RGB comparison for an explicit offline v1 scope index.

No image paths, image decoder, runtime owner, current clock or motion gate.
Pins compare caller-held bytes, NOT authenticated pixels or anti-replay state.
"""
import argparse
from dataclasses import asdict, dataclass, field
import hashlib
from pathlib import Path

from tools.offline_scope_index import IndexReview, read_index, review_index, validate_index
from tools.offline_scope_review import BundleError, ScopeReview, check, parse, review_scope, shape, uint
from tools.time_provider_contract import digest, identity

FORMAT='offline_scope_pixels_v1'
MAX_INPUTS_PER_SCOPE=256


@dataclass(frozen=True)
class PixelScope:
    session_id: str
    scope_id: str
    status: str
    external_inputs_match: bool | None
    review: ScopeReview | None


@dataclass(frozen=True)
class PixelIndexReview:
    classification: str
    evidence_consistent: bool
    base: IndexReview
    payload_bytes_match: bool | None
    payload_sha256: str | None
    issues: tuple[str,...]
    entries: tuple[PixelScope,...]
    compared_scopes: int
    matched_scopes: int
    pixel_coverage: str
    all_declared_inputs_match: bool | None
    coverage: str = field(default='declared_window_only',init=False)
    actual_exposure_wall_s: None = field(default=None,init=False)
    provenance_authenticated: bool = field(default=False,init=False)
    shadow_admission_authorized: bool = field(default=False,init=False)
    native_live_observation_authorized: bool = field(default=False,init=False)
    motion_authority: bool = field(default=False,init=False)


def validate_pixels(doc,index,index_sha256):
    """Whole schema before any extra child review; null explicitly means no data.

    Every declared scope appears once in index order, including unpaired scopes.
    A supplied list must be nonempty and complete, never a sampled successful set.
    The original 2G reader checks the full saved input identity/digest set.
    """
    shape(doc,'format index_sha256 scopes','pixel_schema')
    check(doc['format']==FORMAT,'pixel_format')
    check(digest(doc['index_sha256']) and doc['index_sha256']==index_sha256,'pixel_index_digest')
    declared=[(s,e) for s in index['sessions'] for e in s['scopes']]
    check(type(doc['scopes']) is list and len(doc['scopes'])==len(declared),'pixel_scope_set')
    for item,(session,scope) in zip(doc['scopes'],declared):
        shape(item,'session_id scope_id external_inputs','pixel_scope_schema')
        check(item['session_id']==session['session_id'] and item['scope_id']==scope['scope_id'],'pixel_scope_binding')
        inputs=item['external_inputs']
        if inputs is None:continue
        check(type(inputs) is list and 1<=len(inputs)<=MAX_INPUTS_PER_SCOPE,'pixel_input_limit')
        frames=set();names=[c['view'] for c in session['cameras']]
        for obj in inputs:
            shape(obj,'expected composite_sha256','pixel_input_schema')
            shape(obj['expected'],'frame_id captured_sim_ns view_digests','pixel_expected_schema')
            exp=obj['expected'];check(identity(exp['frame_id']) and uint(exp['captured_sim_ns']),'pixel_input_identity')
            check(exp['frame_id'] not in frames,'pixel_duplicate_input');frames.add(exp['frame_id'])
            check(digest(obj['composite_sha256']),'pixel_composite_digest')
            views=exp['view_digests']
            check(type(views) is list and len(views)==len(names) and all(type(v) is list and len(v)==2
                  and type(v[0]) is str and digest(v[1]) for v in views),'pixel_view_schema')
            check([v[0] for v in views]==names,'pixel_view_set_or_order')
    return declared


def review_pixels(root,manifest,*,manifest_sha256,payload=None,payload_sha256=None):
    """Always retain the entire original v1 result, even if payload is refused.

    Optional payload absence is unknown coverage, not pixel success. Malformed
    supplied material refuses the wrapper, but cannot erase base failures.
    Requires quiescent bounded files; multiple reads are not an atomic snapshot.
    """
    base=review_index(root,manifest,manifest_sha256=manifest_sha256)
    entries=[PixelScope(e.session_id,e.scope_id,'not_compared_not_supplied',None,None) for e in base.entries]
    issues=[];actual=None;matched=None;compared=0;hits=0
    try:
        if payload is None:
            check(payload_sha256 is None,'pixel_digest_without_payload')
        else:
            entries=[PixelScope(e.session_id,e.scope_id,'not_compared_invalid_payload',None,None) for e in base.entries]
            check(digest(payload_sha256),'independent_pixel_digest_required')
            raw=read_index(payload);actual=hashlib.sha256(raw).hexdigest();matched=actual==payload_sha256
            check(matched,'pixel_digest_mismatch')
            check(base.manifest_bytes_match is True,'base_index_not_pinned')
            index_raw=read_index(manifest)
            check(hashlib.sha256(index_raw).hexdigest()==manifest_sha256==base.manifest_sha256,'index_changed_before_pixels')
            index=parse(index_raw);validate_index(index)
            doc=parse(raw);declared=validate_pixels(doc,index,manifest_sha256)
            check(len(base.entries)==len(declared),'base_entries_unavailable')
            for pos,(item,(session,scope)) in enumerate(zip(doc['scopes'],declared)):
                sid=session['session_id'];scope_id=scope['scope_id']
                if item['external_inputs'] is None:
                    entries[pos]=PixelScope(sid,scope_id,'not_compared_no_material',None,None)
                    continue
                result=review_scope(Path(root)/scope['path'],trusted_files=scope['files'],external_inputs=item['external_inputs'])
                compared+=1
                # Original result stays in base, including its failure classification.
                ok=result.evidence_consistent and result.external_inputs_match is True
                if ok:hits+=1
                else:issues.append('scope_pixel_comparison_refused:'+scope_id)
                if result.file_sha256!=base.entries[pos].review.file_sha256:
                    issues.append('scope_changed_between_pixel_reviews:'+scope_id)
                entries[pos]=PixelScope(sid,scope_id,'matched_declared_inputs' if ok else 'comparison_refused',result.external_inputs_match,result)
            check(read_index(payload)==raw,'pixel_payload_changed_during_review')
            check(read_index(manifest)==index_raw,'index_changed_during_pixels')
    except (BundleError,KeyError,TypeError,ValueError,OverflowError,RecursionError) as exc:
        issues.append(str(exc) if isinstance(exc,BundleError) else 'invalid_pixel_schema')
    consistent=base.evidence_consistent and not issues
    all_match=True if consistent and entries and hits==len(entries) and not issues else None
    coverage='all' if all_match else 'partial' if hits else 'none'
    return PixelIndexReview('consistent_declared_window' if consistent else 'incomplete_or_invalid_pixel_index',
        consistent,base,matched,actual,tuple(issues),tuple(entries),compared,hits,coverage,all_match)


def main():
    import json
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True);parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--manifest-sha256',required=True);parser.add_argument('--payload',type=Path)
    parser.add_argument('--payload-sha256');args=parser.parse_args()
    result=review_pixels(args.root,args.manifest,manifest_sha256=args.manifest_sha256,
        payload=args.payload,payload_sha256=args.payload_sha256)
    print(json.dumps(asdict(result),allow_nan=False));return 0 if result.evidence_consistent else 2


if __name__=='__main__':raise SystemExit(main())
