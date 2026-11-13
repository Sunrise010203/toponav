"""Opt-in explicit offline RGB coverage requirements, never a live safety gate.

Retains all v1/2I results. No image decoder, current clock, runtime owner or IO
construction. Caller-held pins are byte comparisons, NOT authenticated policy.
"""
import argparse
from dataclasses import asdict, dataclass, field
import hashlib

from tools.offline_scope_index import read_index, validate_index
from tools.offline_scope_pixels import PixelIndexReview, review_pixels, validate_pixels
from tools.offline_scope_review import BundleError, check, parse, shape, uint
from tools.time_provider_contract import digest

FORMAT='offline_scope_coverage_v1'
MAX_EXPECTED_INPUTS=256


@dataclass(frozen=True)
class ScopeRequirement:
    session_id: str
    scope_id: str
    required: bool | None
    expected_input_count: int | None
    independent_input_count: int | None
    status: str
    locally_met: bool | None


@dataclass(frozen=True)
class CoverageReview:
    classification: str
    evidence_consistent: bool
    requirements_satisfied: bool
    pixels: PixelIndexReview
    requirements_bytes_match: bool | None
    requirements_sha256: str | None
    required_scopes: int
    locally_met_scopes: int
    issues: tuple[str,...]
    entries: tuple[ScopeRequirement,...]
    coverage: str = field(default='declared_window_only',init=False)
    actual_exposure_wall_s: None = field(default=None,init=False)
    provenance_authenticated: bool = field(default=False,init=False)
    shadow_admission_authorized: bool = field(default=False,init=False)
    native_live_observation_authorized: bool = field(default=False,init=False)
    motion_authority: bool = field(default=False,init=False)


def validate_requirements(doc,index,index_sha256,payload_sha256):
    """Explicit complete scope table, with at least one nonempty requirement.

    Nonrequired rows must say false/null; no discovery or successful-subset
    inference. Required rows state an independent exact full input count.
    """
    shape(doc,'format index_sha256 payload_sha256 scopes','coverage_schema')
    check(doc['format']==FORMAT,'coverage_format')
    check(digest(doc['index_sha256']) and doc['index_sha256']==index_sha256,'coverage_index_binding')
    check((doc['payload_sha256'] is None or digest(doc['payload_sha256']))
          and doc['payload_sha256']==payload_sha256,'coverage_payload_binding')
    declared=[(s,e) for s in index['sessions'] for e in s['scopes']]
    check(type(doc['scopes']) is list and len(doc['scopes'])==len(declared),'coverage_scope_set')
    required=0
    for item,(session,scope) in zip(doc['scopes'],declared):
        shape(item,'session_id scope_id required expected_input_count','coverage_scope_schema')
        check(item['session_id']==session['session_id'] and item['scope_id']==scope['scope_id'],'coverage_scope_binding')
        check(type(item['required']) is bool,'coverage_required_bool')
        count=item['expected_input_count']
        if item['required']:
            required+=1;check(uint(count,MAX_EXPECTED_INPUTS) and count>0,'coverage_required_count')
        else:check(count is None,'coverage_optional_count_must_be_null')
    check(required>0,'coverage_nonempty_requirement_required')
    return declared,required


def review_coverage(root,manifest,*,manifest_sha256,requirements,requirements_sha256,
                    payload=None,payload_sha256=None):
    """Apply explicit frozen requirements without changing v1/2I defaults.

    Local matches stay visible on ANY parent failure; they cannot promote a
    successful subset. Original recorded failures/incomplete evidence are kept
    verbatim in pixels.base. No source authentication or fresh exposure follows.
    """
    pixels=review_pixels(root,manifest,manifest_sha256=manifest_sha256,payload=payload,payload_sha256=payload_sha256)
    entries=[ScopeRequirement(e.session_id,e.scope_id,None,None,None,'not_evaluated_invalid_requirements',None)
             for e in pixels.entries]
    issues=[] if pixels.evidence_consistent else ['pixel_index_not_consistent']
    actual=None;matched=None;required=0;met=0
    try:
        check(digest(requirements_sha256),'independent_coverage_digest_required')
        raw=read_index(requirements);actual=hashlib.sha256(raw).hexdigest();matched=actual==requirements_sha256
        check(matched,'coverage_digest_mismatch')
        check(pixels.base.manifest_bytes_match is True,'coverage_base_index_not_pinned')
        index_raw=read_index(manifest)
        check(hashlib.sha256(index_raw).hexdigest()==manifest_sha256==pixels.base.manifest_sha256,'index_changed_before_coverage')
        index=parse(index_raw);validate_index(index)
        doc=parse(raw);declared,required=validate_requirements(doc,index,manifest_sha256,payload_sha256)
        check(len(pixels.entries)==len(declared),'coverage_base_entries_unavailable')
        payload_raw=None;material=[None]*len(declared)
        if payload is not None:
            check(pixels.payload_bytes_match is True,'coverage_pixel_payload_not_pinned')
            payload_raw=read_index(payload)
            check(hashlib.sha256(payload_raw).hexdigest()==payload_sha256==pixels.payload_sha256,'pixels_changed_before_coverage')
            pixel_doc=parse(payload_raw);validate_pixels(pixel_doc,index,manifest_sha256)
            material=[row['external_inputs'] for row in pixel_doc['scopes']]
        for pos,(item,data,previous) in enumerate(zip(doc['scopes'],material,pixels.entries)):
            sid=item['session_id'];scope=item['scope_id'];count=None if data is None else len(data)
            check(previous.session_id==sid and previous.scope_id==scope,'coverage_result_identity_changed')
            local=None;status='not_required'
            if item['required']:
                local=False
                if data is None:status='required_material_missing'
                elif previous.status!='matched_declared_inputs' or previous.external_inputs_match is not True:
                    status='required_comparison_refused'
                elif count!=item['expected_input_count']:status='required_input_count_mismatch'
                else:local=True;met+=1;status='matched_required_inputs'
                if not local:issues.append(status+':'+scope)
            entries[pos]=ScopeRequirement(sid,scope,item['required'],item['expected_input_count'],count,status,local)
        check(read_index(requirements)==raw,'requirements_changed_during_coverage')
        if payload is not None:check(read_index(payload)==payload_raw,'pixels_changed_during_coverage')
        check(read_index(manifest)==index_raw,'index_changed_during_coverage')
    except (BundleError,KeyError,TypeError,ValueError,OverflowError,RecursionError) as exc:
        issues.append(str(exc) if isinstance(exc,BundleError) else 'invalid_coverage_schema')
    satisfied=not issues and pixels.evidence_consistent and required>0 and met==required
    return CoverageReview('declared_requirements_satisfied' if satisfied else 'requirements_not_satisfied',
        satisfied,satisfied,pixels,matched,actual,required,met,tuple(issues),tuple(entries))


def main():
    import json
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',required=True);parser.add_argument('--manifest',required=True)
    parser.add_argument('--manifest-sha256',required=True);parser.add_argument('--payload');parser.add_argument('--payload-sha256')
    parser.add_argument('--requirements',required=True);parser.add_argument('--requirements-sha256',required=True)
    args=parser.parse_args()
    result=review_coverage(args.root,args.manifest,manifest_sha256=args.manifest_sha256,
        requirements=args.requirements,requirements_sha256=args.requirements_sha256,payload=args.payload,payload_sha256=args.payload_sha256)
    print(json.dumps(asdict(result),allow_nan=False));return 0 if result.requirements_satisfied else 2


if __name__=='__main__':raise SystemExit(main())
