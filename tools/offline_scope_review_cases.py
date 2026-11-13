"""Offline saved-file review/corruption fixtures only. No live IO or callbacks."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil

from PIL import Image

from tools.offline_scope_review import FILES, review_scope

OLD='reports/straight-followup-20261102-15'
PUBLIC_CASES=(('baseline_known_center','reports/straight-followup-20261101-01/round2/after-known-center'),
    ('new_center_gradient_transfer','reports/straight-followup-20261101-03/comparison/new_center-gradient_transfer'),
    ('new_offset_plus_wide_reference','reports/straight-followup-20261101-03/comparison/new_offset_plus-wide_reference'))
CASES=('missing_file','truncated_json','duplicate_json_key','nonfinite_json','large_integer','invalid_utf8','oversized_file',
    'manifest_escape','manifest_self_reference','raw_hash_mismatch','snapshot_pin','baseline_changed','scope_close_early',
    'duplicate_input','changed_input_digest','duplicate_event','missing_delivery','missing_compute_return','event_reordered',
    'foreign_receipt_reference','changed_receipt','missing_receipt','duplicate_receipt','false_authority','invented_exposure',
    'workers_alive','status_accounting','failure_laundered_status_only','bad_camera_set','changed_boot',
    'unsupported_source','callback_time_outside','duplicate_record_id','wrong_rejection_reason','external_rgb_mismatch',
    'joint_session_relabel_unpinned','joint_session_relabel_pinned','joint_failure_erasure_unpinned','joint_failure_erasure_pinned',
    'repeated_bundle_unpinned')
CONSISTENCY_COUNTEREXAMPLES={'joint_session_relabel_unpinned','joint_failure_erasure_unpinned','repeated_bundle_unpinned'}
load=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def write(path,value):
    with Path(path).open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')


def scope_paths(root):
    return sorted((Path(root)/OLD/'lifecycle-audit').rglob('status.json'))


def pins(root,scope):
    manifest=load(Path(root)/OLD/'DELIVERY_MANIFEST.json')['files']
    return {name:manifest[str((scope/name).relative_to(root))] for name in FILES}


def public_inputs(root,name):
    relative=dict(PUBLIC_CASES)[name];base=Path(root)/relative;result=[]
    for row in [json.loads(line) for line in (base/'decisions.jsonl').read_text().splitlines()][:4]:
        ref=row['input'];path=(base/'public'/ref['rgb_path']).resolve();assert path.is_relative_to((base/'public').resolve())
        with Image.open(path) as im:
            assert im.mode=='RGB' and im.size==(256,256)
            composite=hashlib.sha256(im.tobytes()).hexdigest();assert composite==ref['rgb_sha256']
            tiles=[im.crop(box) for box in ((0,0,128,128),(128,0,256,128),(0,128,128,256),(128,128,256,256))]
            views=[[v,hashlib.sha256(tile.tobytes()).hexdigest()] for v,tile in zip(('front','left','right','back'),tiles)]
        result.append(dict(expected=dict(frame_id=ref['frame_id'],captured_sim_ns=round(ref['captured_sim_s']*1e9),view_digests=views),composite_sha256=composite))
    assert len(result)==4;return result


def exercise(root,name,output):
    assert name in CASES;root=Path(root);output=Path(output);output.mkdir(exist_ok=False)
    source=('body_exception_while_callback' if name.startswith('joint_failure_erasure') or name=='failure_laundered_status_only' else
            'history_drop' if name=='wrong_rejection_reason' else 'four')
    source=root/OLD/'lifecycle-audit'/source/'scope';out=output/'scope';out.mkdir()
    original_pins=pins(root,source)
    for filename in FILES:shutil.copyfile(source/filename,out/filename)
    docs={n:load(out/n) for n in FILES if n!='events.jsonl'}
    events=[json.loads(line) for line in (out/'events.jsonl').read_text().splitlines()]
    begin,scope,snapshot,inputs,receipts,status=(docs[n] for n in ('begin.json','scope.json','snapshot.json','inputs.json','receipts.json','status.json'))
    raw=None;external=None
    if name=='missing_file':(out/'begin.json').unlink()
    elif name=='truncated_json':raw=('begin.json',b'{')
    elif name=='duplicate_json_key':raw=('begin.json',b'{"scope_id":"first","scope_id":"second"}')
    elif name=='nonfinite_json':raw=('begin.json',b'{"started_s":NaN}')
    elif name=='large_integer':raw=('begin.json',b'{"started_s":123456789012345678901234567890}')
    elif name=='invalid_utf8':raw=('begin.json',b'\xff')
    elif name=='oversized_file':raw=('begin.json',b' '*(2*1024*1024+1))
    elif name=='manifest_escape':status['artifact_files']['../outside.json']=status['artifact_files'].pop('begin.json')
    elif name=='manifest_self_reference':status['artifact_files']['status.json']='0'*64
    elif name=='raw_hash_mismatch':raw=('snapshot.json',(out/'snapshot.json').read_bytes()+b' ')
    elif name=='snapshot_pin':scope['snapshot_sha256']='0'*64
    elif name=='baseline_changed':scope['counts_before'][0][1]=begin['counts_before'][0][1]=9
    elif name=='scope_close_early':scope['closed_s']=scope['started_s']
    elif name=='duplicate_input':inputs['inputs'].append(inputs['inputs'][0])
    elif name=='changed_input_digest':inputs['inputs'][0]['expected']['view_digests'][0][1]='0'*64
    elif name=='duplicate_event':events.insert(3,events[2])
    elif name=='missing_delivery':events=[e for e in events if e['kind']!='delivery_requested']
    elif name=='missing_compute_return':events=[e for e in events if e['kind']!='compute_returned']
    elif name=='event_reordered':events[2],events[3]=events[3],events[2]
    elif name=='foreign_receipt_reference':receipts[0]['reference']['scope_id']='foreign'
    elif name=='changed_receipt':receipts[0]['value']['bundle']['views'][0]['callback_host_ns']+=1
    elif name=='missing_receipt':receipts.clear()
    elif name=='duplicate_receipt':receipts.append(receipts[0])
    elif name=='false_authority':status['motion_authority']=True
    elif name=='invented_exposure':receipts[0]['value']['bundle']['views'][0]['exposure_start_tick']=0
    elif name=='workers_alive':status['workers_alive']=['still-running']
    elif name=='status_accounting':status['receipt_adapted']=True
    elif name=='failure_laundered_status_only':status['status']='receipt_only_recorded'
    elif name=='bad_camera_set':begin['spec']['cameras'][0]['view']='left'
    elif name=='changed_boot':scope['host_boot_id']='other-boot'
    elif name=='unsupported_source':begin['source_kind']=scope['source_kind']='live_callback_delivery'
    elif name=='callback_time_outside':snapshot['histories']['callbacks'][0]['callback_entry_s']=scope['started_s']-1
    elif name=='duplicate_record_id':snapshot['histories']['callbacks'][1]['record_id']=1
    elif name=='wrong_rejection_reason':receipts[0]['error']='invented failure'
    elif name=='external_rgb_mismatch':
        external=[dict(expected=inputs['inputs'][0]['expected'],composite_sha256='0'*64)]
    elif name.startswith('joint_session_relabel'):
        begin['spec']['session_id']=scope['session_id']=status['session_id']='jointly-forged-local-session'
        receipts[0]['value']['bundle']['session_id']='jointly-forged-local-session'
    elif name.startswith('joint_failure_erasure'):
        status['body_error']=None;status['status']='observation_only'
    elif name=='repeated_bundle_unpinned':pass
    else:raise AssertionError(name)
    # Recompute local pins for semantic mutations, not to claim provenance.
    if name in ('callback_time_outside','duplicate_record_id'):
        scope['snapshot_sha256']=hashlib.sha256(json.dumps(snapshot,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
        for r in receipts:
            if r['adapted']:r['value']['snapshot_sha256']=scope['snapshot_sha256']
    for filename,value in docs.items():
        if (out/filename).exists():(out/filename).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    (out/'events.jsonl').write_text(''.join(json.dumps(e,allow_nan=False)+'\n' for e in events))
    if raw is not None:(out/raw[0]).write_bytes(raw[1])
    if name not in ('manifest_escape','manifest_self_reference','raw_hash_mismatch'):
        status['artifact_files']={n:sha(out/n) for n in FILES if n!='status.json' and (out/n).is_file()}
        (out/'status.json').write_text(json.dumps(status,indent=2,allow_nan=False)+'\n')
    pinned=name in ('joint_session_relabel_pinned','joint_failure_erasure_pinned')
    before={p.name:sha(p) for p in out.iterdir()}
    result=review_scope(out,trusted_files=original_pins if pinned else None,external_inputs=external)
    after={p.name:sha(p) for p in out.iterdir()};assert before==after
    if name in CONSISTENCY_COUNTEREXAMPLES:
        assert result.evidence_consistent,(name,result)
        assert not result.provenance_authenticated and result.trusted_bytes_match is None
        if name=='repeated_bundle_unpinned':assert result==review_scope(out)
    else:assert not result.evidence_consistent and result.classification=='invalid_bundle',(name,result)
    record=dict(case=name,result=asdict(result),input_files_unchanged=True,
        consistency_without_authentication_counterexample=name in CONSISTENCY_COUNTEREXAMPLES)
    write(output/'result.json',record);return record


def run(root,output):
    root=Path(root);output=Path(output);output.mkdir(exist_ok=False);original=[]
    for path in scope_paths(root):
        scope=path.parent;name=str(scope.relative_to(root/OLD/'lifecycle-audit'))
        public=name.removeprefix('public-') if name.startswith('public-') else None
        result=review_scope(scope,trusted_files=pins(root,scope),external_inputs=public_inputs(root,public) if public else None)
        status=load(path);incomplete=not status['artifact_files_complete']
        assert result.recorded_status==status['status'] and result.trusted_bytes_match is True
        assert result.complete_evidence==result.evidence_consistent==(not incomplete),(name,result)
        assert result.classification==('recorded_incomplete' if incomplete else result.classification)
        original.append(dict(scope=name,result=asdict(result)))
    assert len(original)==23
    mutations=[exercise(root,n,output/n) for n in CASES]
    result=dict(original_scopes=original,mutation_cases=mutations,original_count=23,
        original_complete=20,original_recorded_incomplete=3,corruption_refused=37,
        nonauthentication_counterexamples=3,public_inputs=12,all_authority_false=True,
        actual_exposure_wall_s=None,native_transport=False,movement=False,production_changes=False,
        note='Only closed saved files and pure receipt recomputation. Consistency counterexamples are not security passes; original failures retained.')
    write(output/'summary.json',result);return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    print(json.dumps(run(Path(__file__).resolve().parents[1],args.output)))
