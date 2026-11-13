"""Explicit saved-data coverage fixtures. No live IO or controller execution."""
import argparse
import copy
from dataclasses import asdict
from pathlib import Path
import shutil

from tools.offline_scope_coverage import FORMAT, review_coverage
from tools.offline_scope_index_cases import OLD, load, make_index, originals, rewrite, sha, write
from tools.offline_scope_pixels_cases import datasets, make_payload
from tools.offline_scope_review import FILES
from tools.offline_scope_review_cases import PUBLIC_CASES

CASES=('full_required','optional_gap_allowed','required_gap','payload_absent','all_null','no_required',
    'wrong_requirements_digest','missing_requirements_digest','wrong_index_binding','wrong_payload_binding','wrong_format',
    'missing_scope','duplicate_scope','reordered_scopes','wrong_scope','wrong_session','extra_scope_path','extra_policy_path',
    'duplicate_json_key','oversized_requirements','required_not_bool','bool_count','zero_count','negative_count','large_count',
    'fractional_count','missing_count','optional_nonnull_count','wrong_expected_count','wrong_composite','wrong_view',
    'missing_input','duplicate_input','invalid_optional_pixels','child_missing_file','child_hash_changed','base_context_failure',
    'recorded_failure_with_pixels','incomplete_with_pixels','joint_session_relabel','joint_failure_erasure','repeated_review',
    'joint_policy_narrowing','policy_narrowing_old_pin','joint_window_truncation')
CONSISTENT={'full_required','optional_gap_allowed','recorded_failure_with_pixels','joint_session_relabel',
    'joint_failure_erasure','repeated_review','joint_policy_narrowing','joint_window_truncation'}
COUNTEREXAMPLES={'joint_session_relabel','joint_failure_erasure','repeated_review','joint_policy_narrowing','joint_window_truncation'}


def make_requirements(index,index_sha256,payload_sha256,required_counts):
    """Explicit driver policy only, never infer required scopes from successes."""
    return dict(format=FORMAT,index_sha256=index_sha256,payload_sha256=payload_sha256,
        scopes=[dict(session_id=s['session_id'],scope_id=e['scope_id'],required=e['scope_id'] in required_counts,
            expected_input_count=required_counts.get(e['scope_id'])) for s in index['sessions'] for e in s['scopes']])


def exercise(root,name,output,independent=None):
    assert name in CASES;root=Path(root);output=Path(output);output.mkdir(exist_ok=False)
    scope_root=output/'scopes';scope_root.mkdir();paths=[];by_scope={}
    independent=datasets(root) if independent is None else independent
    for pos,(case,_) in enumerate(PUBLIC_CASES[:2]):
        source=root/OLD/'lifecycle-audit'/('public-'+case);dest=scope_root/str(pos);dest.mkdir()
        for n in FILES:shutil.copyfile(source/n,dest/n)
        paths.append(dest);by_scope[load(dest/'scope.json')['scope_id']]=independent[case]
    if name in ('recorded_failure_with_pixels','incomplete_with_pixels','joint_failure_erasure'):
        rewrite(paths[0],lambda d:d['status.json'].update(body_error=dict(type='RuntimeError',message='synthetic saved failure'),status='body_failed'))
        if name=='incomplete_with_pixels':
            rewrite(paths[0],lambda d:d['status.json'].update(artifact_files_complete=False,
                cleanup_errors=[dict(type='OSError',message='synthetic saved log close failure')]))
        if name=='joint_failure_erasure':
            write(output/'synthetic_failure_status_before.json',load(paths[0]/'status.json'))
            rewrite(paths[0],lambda d:d['status.json'].update(body_error=None,status='receipt_only_recorded'))
    if name=='joint_session_relabel':
        def relabel(d):
            sid='jointly-relabelled-coverage-session'
            d['begin.json']['spec']['session_id']=d['scope.json']['session_id']=d['status.json']['session_id']=sid
            for r in d['receipts.json']:
                if r['adapted']:r['value']['bundle']['session_id']=sid
        rewrite(paths[0],relabel)
    if name=='joint_window_truncation':paths=paths[:1]
    index=make_index(scope_root,paths)
    if name=='base_context_failure':index['sessions'][0]['config_id']='foreign-config'
    manifest=output/'index.json';write(manifest,index);pixel_doc=make_payload(index,sha(manifest),by_scope)
    rows=pixel_doc['scopes']
    if name in ('optional_gap_allowed','required_gap','joint_policy_narrowing','policy_narrowing_old_pin'):
        rows[1]['external_inputs']=None
    if name=='all_null':
        for row in rows:row['external_inputs']=None
    if name=='wrong_composite':rows[0]['external_inputs'][0]['composite_sha256']='0'*64
    if name=='wrong_view':rows[0]['external_inputs'][0]['expected']['view_digests'][0][1]='0'*64
    if name=='missing_input':rows[0]['external_inputs'].pop()
    if name=='duplicate_input':rows[0]['external_inputs'].append(copy.deepcopy(rows[0]['external_inputs'][0]))
    if name=='invalid_optional_pixels':rows[1]['external_inputs'][0]['composite_sha256']='0'*64
    pixels=output/'pixels.json';write(pixels,pixel_doc)
    absent=name=='payload_absent';counts={e['scope_id']:4 for s in index['sessions'] for e in s['scopes']}
    req=make_requirements(index,sha(manifest),None if absent else sha(pixels),counts);items=req['scopes'];raw=None;held_pin=None
    if name in ('joint_policy_narrowing','policy_narrowing_old_pin'):
        old=output/'synthetic_requirements_before.json';write(old,req);held_pin=sha(old)
    if name in ('optional_gap_allowed','invalid_optional_pixels','joint_policy_narrowing','policy_narrowing_old_pin'):
        items[1].update(required=False,expected_input_count=None)
    if name=='no_required':
        for item in items:item.update(required=False,expected_input_count=None)
    elif name=='wrong_index_binding':req['index_sha256']='0'*64
    elif name=='wrong_payload_binding':req['payload_sha256']='0'*64
    elif name=='wrong_format':req['format']='live_authority'
    elif name=='missing_scope':items.pop()
    elif name=='duplicate_scope':items[1]=copy.deepcopy(items[0])
    elif name=='reordered_scopes':items.reverse()
    elif name=='wrong_scope':items[0]['scope_id']='other-scope'
    elif name=='wrong_session':items[0]['session_id']='other-session'
    elif name=='extra_scope_path':items[0]['path']='../../do-not-open'
    elif name=='extra_policy_path':req['path']='/do-not-open'
    elif name=='duplicate_json_key':raw=b'{"format":"first","format":"second"}'
    elif name=='oversized_requirements':raw=b' '*(256*1024+1)
    elif name=='required_not_bool':items[0]['required']=1
    elif name=='bool_count':items[0]['expected_input_count']=True
    elif name=='zero_count':items[0]['expected_input_count']=0
    elif name=='negative_count':items[0]['expected_input_count']=-1
    elif name=='large_count':items[0]['expected_input_count']=257
    elif name=='fractional_count':items[0]['expected_input_count']=4.0
    elif name=='missing_count':items[0].pop('expected_input_count')
    elif name=='optional_nonnull_count':items[0]['required']=False
    elif name=='wrong_expected_count':items[0]['expected_input_count']=3
    elif name=='child_missing_file':(paths[0]/'begin.json').unlink()
    elif name=='child_hash_changed':(paths[0]/'begin.json').write_bytes((paths[0]/'begin.json').read_bytes()+b' ')
    requirements=output/'requirements.json'
    if raw is None:write(requirements,req)
    else:requirements.write_bytes(raw)
    pin=sha(requirements)
    if name=='wrong_requirements_digest':pin='0'*64
    if name=='missing_requirements_digest':pin=None
    if name=='policy_narrowing_old_pin':pin=held_pin
    kwargs=dict(manifest_sha256=sha(manifest),payload=None if absent else pixels,payload_sha256=None if absent else sha(pixels),
        requirements=requirements,requirements_sha256=pin)
    before={str(p.relative_to(output)):sha(p) for p in output.rglob('*') if p.is_file()}
    result=review_coverage(scope_root,manifest,**kwargs)
    assert result.requirements_satisfied==result.evidence_consistent==(name in CONSISTENT),(name,result)
    assert len(result.entries)==len(result.pixels.entries)==len(result.pixels.base.entries)==len(paths)
    if name=='repeated_review':assert result==review_coverage(scope_root,manifest,**kwargs)
    if name=='recorded_failure_with_pixels':assert result.pixels.base.entries[0].review.classification=='recorded_failure'
    if name=='incomplete_with_pixels':assert result.pixels.base.entries[0].review.classification=='recorded_incomplete'
    after={str(p.relative_to(output)):sha(p) for p in output.rglob('*') if p.is_file()};assert before==after
    record=dict(case=name,result=asdict(result),input_files_unchanged=True,input_sha256=before,
        synthetic_saved_data_only=True,nonauthentication_or_coverage_counterexample=name in COUNTEREXAMPLES)
    write(output/'result.json',record);return record


def run(root,output):
    root=Path(root);output=Path(output);output.mkdir(exist_ok=False);independent=datasets(root)
    public_paths=[root/OLD/'lifecycle-audit'/('public-'+name) for name,_ in PUBLIC_CASES]
    by_scope={load(p/'scope.json')['scope_id']:independent[name] for p,(name,_) in zip(public_paths,PUBLIC_CASES)}
    original=[]
    for name,paths,require_all in (('original_public_required',public_paths,True),
            ('original_23_public_required',originals(root),False),('original_23_all_required',originals(root),True)):
        index=make_index(root,paths,original_root=root);manifest=output/(name+'-index.json');write(manifest,index)
        payload=output/(name+'-pixels.json');write(payload,make_payload(index,sha(manifest),by_scope))
        counts={e['scope_id']:4 for s in index['sessions'] for e in s['scopes'] if require_all or e['scope_id'] in by_scope}
        req=output/(name+'-requirements.json');write(req,make_requirements(index,sha(manifest),sha(payload),counts))
        result=review_coverage(root,manifest,manifest_sha256=sha(manifest),payload=payload,payload_sha256=sha(payload),
            requirements=req,requirements_sha256=sha(req))
        assert result.requirements_satisfied==(name=='original_public_required')
        assert result.locally_met_scopes==3 and len(result.entries)==len(paths)
        assert result.required_scopes==(23 if name=='original_23_all_required' else 3)
        original.append(dict(case=name,result=asdict(result)))
    cases=[exercise(root,name,output/name,independent) for name in CASES]
    summary=dict(original=original,cases=cases,case_count=len(CASES),refused=len(CASES)-len(CONSISTENT),
        consistent_fixture_cases=len(CONSISTENT),nonauthentication_or_coverage_counterexamples=len(COUNTEREXAMPLES),
        independently_decoded_public_pngs=12,production_changes=False,native_transport=False,movement=False)
    write(output/'summary.json',summary);return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    import json
    print(json.dumps(run(Path(__file__).resolve().parents[1],args.output)))
