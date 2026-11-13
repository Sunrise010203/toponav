"""Saved-data RGB index fixtures. PNG decoding stays here, never in the reader."""
import argparse
import copy
from dataclasses import asdict
import json
from pathlib import Path
import shutil

from tools.offline_scope_index_cases import OLD, load, make_index, originals, rewrite, sha, write
from tools.offline_scope_pixels import FORMAT, review_pixels
from tools.offline_scope_review import FILES
from tools.offline_scope_review_cases import PUBLIC_CASES, public_inputs

CASES=('matched_all','payload_absent','all_null','partial_material','wrong_payload_digest','missing_payload_digest',
    'wrong_index_binding','wrong_format','missing_scope','duplicate_scope','reordered_scopes','wrong_scope','wrong_session',
    'extra_scope_path','extra_payload_path','duplicate_json_key','oversized_payload','wrong_frame','wrong_sim_stamp',
    'wrong_view_digest','crosswired_digests','wrong_composite','missing_input','duplicate_input','extra_input','missing_view',
    'duplicate_view','reordered_views','empty_inputs','bool_sim_stamp','extra_image_path','child_missing_file','child_hash_changed',
    'recorded_failure_with_pixels','incomplete_with_pixels','base_context_failure','joint_session_relabel',
    'joint_failure_erasure','repeated_review')
CONSISTENT={'matched_all','payload_absent','all_null','partial_material','recorded_failure_with_pixels',
    'joint_session_relabel','joint_failure_erasure','repeated_review'}
COUNTEREXAMPLES={'joint_session_relabel','joint_failure_erasure','repeated_review'}


def make_payload(index,index_sha256,by_scope):
    """Explicit full scope table: absent independently read pixels are null."""
    return dict(format=FORMAT,index_sha256=index_sha256,scopes=[dict(session_id=s['session_id'],scope_id=e['scope_id'],
        external_inputs=copy.deepcopy(by_scope.get(e['scope_id']))) for s in index['sessions'] for e in s['scopes']])


def datasets(root):
    # Read each public PNG prefix independently; saved inputs.json is NEVER the digest source.
    return {name:public_inputs(root,name) for name,_ in PUBLIC_CASES}


def exercise(root,name,output,independent=None):
    assert name in CASES;root=Path(root);output=Path(output);output.mkdir(exist_ok=False)
    scope_root=output/'scopes';scope_root.mkdir();paths=[];by_scope={}
    independent=datasets(root) if independent is None else independent
    for pos,(case,_) in enumerate(PUBLIC_CASES[:2]):
        source=root/OLD/'lifecycle-audit'/('public-'+case);dest=scope_root/str(pos);dest.mkdir()
        for n in FILES:shutil.copyfile(source/n,dest/n)
        paths.append(dest);by_scope[load(dest/'scope.json')['scope_id']]=independent[case]
    if name in ('recorded_failure_with_pixels','incomplete_with_pixels','joint_failure_erasure'):
        def failure(d):d['status.json'].update(body_error=dict(type='RuntimeError',message='synthetic saved-data failure'),status='body_failed')
        rewrite(paths[0],failure)
        if name=='incomplete_with_pixels':
            rewrite(paths[0],lambda d:d['status.json'].update(artifact_files_complete=False,
                cleanup_errors=[dict(type='OSError',message='synthetic log close failure')],status='body_failed'))
        if name=='joint_failure_erasure':
            # Keep the pre-erasure copy as disclosed counterexample material, not original evidence.
            write(output/'synthetic_failure_status_before.json',load(paths[0]/'status.json'))
            rewrite(paths[0],lambda d:d['status.json'].update(body_error=None,status='receipt_only_recorded'))
    if name=='joint_session_relabel':
        def relabel(d):
            sid='jointly-relabelled-local-session'
            d['begin.json']['spec']['session_id']=d['scope.json']['session_id']=d['status.json']['session_id']=sid
            for r in d['receipts.json']:
                if r['adapted']:r['value']['bundle']['session_id']=sid
        rewrite(paths[0],relabel)
    index=make_index(scope_root,paths)
    if name=='base_context_failure':index['sessions'][0]['config_id']='foreign-config'
    manifest=output/'index.json';write(manifest,index)
    payload=make_payload(index,sha(manifest),by_scope);items=payload['scopes'];inp=items[0]['external_inputs'];raw=None
    if name=='wrong_index_binding':payload['index_sha256']='0'*64
    elif name=='wrong_format':payload['format']='authenticated_pixels'
    elif name=='missing_scope':items.pop()
    elif name=='duplicate_scope':items[1]=copy.deepcopy(items[0])
    elif name=='reordered_scopes':items.reverse()
    elif name=='wrong_scope':items[0]['scope_id']='other-scope'
    elif name=='wrong_session':items[0]['session_id']='other-session'
    elif name=='extra_scope_path':items[0]['image_path']='/must/not/be/opened'
    elif name=='extra_payload_path':payload['image_path']='/must/not/be/opened'
    elif name=='duplicate_json_key':raw=b'{"format":"first","format":"second"}'
    elif name=='oversized_payload':raw=b' '*(256*1024+1)
    elif name=='wrong_frame':inp[0]['expected']['frame_id']='other-frame'
    elif name=='wrong_sim_stamp':inp[0]['expected']['captured_sim_ns']+=1
    elif name=='wrong_view_digest':inp[0]['expected']['view_digests'][0][1]='0'*64
    elif name=='crosswired_digests':
        a=inp[0]['expected']['view_digests'];a[0][1],a[1][1]=a[1][1],a[0][1]
    elif name=='wrong_composite':inp[0]['composite_sha256']='0'*64
    elif name=='missing_input':inp.pop()
    elif name=='duplicate_input':inp.append(copy.deepcopy(inp[0]))
    elif name=='extra_input':
        new=copy.deepcopy(inp[0]);new['expected']['frame_id']='extra-frame';inp.append(new)
    elif name=='missing_view':inp[0]['expected']['view_digests'].pop()
    elif name=='duplicate_view':inp[0]['expected']['view_digests'][1]=copy.deepcopy(inp[0]['expected']['view_digests'][0])
    elif name=='reordered_views':inp[0]['expected']['view_digests'].reverse()
    elif name=='empty_inputs':items[0]['external_inputs']=[]
    elif name=='bool_sim_stamp':inp[0]['expected']['captured_sim_ns']=True
    elif name=='extra_image_path':inp[0]['expected']['rgb_path']='../../outside.png'
    elif name=='child_missing_file':(paths[0]/'begin.json').unlink()
    elif name=='child_hash_changed':(paths[0]/'begin.json').write_bytes((paths[0]/'begin.json').read_bytes()+b' ')
    elif name=='all_null':
        for i in items:i['external_inputs']=None
    elif name=='partial_material':items[1]['external_inputs']=None
    target=output/'pixels.json'
    if raw is None:write(target,payload)
    else:target.write_bytes(raw)
    pin=sha(target)
    if name=='wrong_payload_digest':pin='0'*64
    if name=='missing_payload_digest':pin=None
    kwargs=dict(manifest_sha256=sha(manifest),payload=target,payload_sha256=pin)
    if name=='payload_absent':kwargs.update(payload=None,payload_sha256=None)
    before={str(p.relative_to(output)):sha(p) for p in output.rglob('*') if p.is_file()}
    result=review_pixels(scope_root,manifest,**kwargs)
    assert result.evidence_consistent==(name in CONSISTENT),(name,result)
    assert len(result.entries)==len(result.base.entries)==2
    if name=='repeated_review':assert result==review_pixels(scope_root,manifest,**kwargs)
    if name=='recorded_failure_with_pixels':assert result.base.entries[0].review.classification=='recorded_failure'
    if name=='incomplete_with_pixels':assert result.base.entries[0].review.classification=='recorded_incomplete'
    if name in ('child_missing_file','child_hash_changed','incomplete_with_pixels','base_context_failure'):
        assert not result.base.evidence_consistent and result.entries[1].external_inputs_match is True
    after={str(p.relative_to(output)):sha(p) for p in output.rglob('*') if p.is_file()};assert before==after
    record=dict(case=name,result=asdict(result),input_sha256=before,input_files_unchanged=True,
        synthetic_saved_data_only=True,nonauthentication_counterexample=name in COUNTEREXAMPLES)
    write(output/'result.json',record);return record


def run(root,output):
    root=Path(root);output=Path(output);output.mkdir(exist_ok=False)
    independent=datasets(root);original=[]
    public_paths=[root/OLD/'lifecycle-audit'/('public-'+name) for name,_ in PUBLIC_CASES]
    by_scope={load(p/'scope.json')['scope_id']:independent[name] for p,(name,_) in zip(public_paths,PUBLIC_CASES)}
    for name,paths in (('original_public_three',public_paths),('original_all_23',originals(root))):
        index=make_index(root,paths,original_root=root);manifest=output/(name+'-index.json');write(manifest,index)
        payload=output/(name+'-pixels.json');write(payload,make_payload(index,sha(manifest),by_scope))
        result=review_pixels(root,manifest,manifest_sha256=sha(manifest),payload=payload,payload_sha256=sha(payload))
        assert result.evidence_consistent==(name=='original_public_three')
        assert result.compared_scopes==result.matched_scopes==3 and len(result.entries)==len(paths)
        assert result.pixel_coverage==('all' if len(paths)==3 else 'partial')
        original.append(dict(case=name,result=asdict(result)))
    cases=[exercise(root,n,output/n,independent) for n in CASES]
    summary=dict(original=original,cases=cases,case_count=len(CASES),refused=len(CASES)-len(CONSISTENT),
        consistent_fixture_cases=len(CONSISTENT),nonauthentication_counterexamples=len(COUNTEREXAMPLES),
        independently_decoded_public_pngs=12,public_pixels_source='public PNG bytes, never saved inputs declarations',
        original_all_uncompared_scopes=20,production_changes=False,native_transport=False,movement=False)
    write(output/'summary.json',summary);return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    print(json.dumps(run(Path(__file__).resolve().parents[1],args.output)))
