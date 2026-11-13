"""Saved-file scope-index fixtures. Never reconstruct runtime callbacks/IO."""
import argparse
import copy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil

from tools.offline_scope_index import CONTEXT, FORMAT, review_index
from tools.offline_scope_review import FILES, KINDS

OLD='reports/straight-followup-20261102-15'
CASES=('wrong_index_digest','missing_index_digest','duplicate_json_key','missing_file_pin','absolute_path','parent_path',
    'empty_path_component','duplicate_scope_id','duplicate_session_id','duplicate_scope_path','wrong_coverage','live_source',
    'wrong_session_context','wrong_config_context','wrong_boot_context','wrong_camera_context','wrong_initial_counts',
    'wrong_terminal_counts','wrong_predecessor_id','wrong_predecessor_pin','reversed_scopes','omitted_first','omitted_last',
    'child_hash_changed','child_missing_file','child_invalid_schema','counter_gap','retained_history_changed',
    'continued_after_io_error','incomplete_predecessor','empty_bridge_valid','empty_bridge_omitted','observer_counter_decrease',
    'recorded_body_failure_continues','repeated_index','declared_suffix_window','joint_relabel_nonauthentication')
CONSISTENT={'empty_bridge_valid','recorded_body_failure_continues','repeated_index','declared_suffix_window','joint_relabel_nonauthentication'}
COUNTEREXAMPLES={'repeated_index','declared_suffix_window','joint_relabel_nonauthentication'}
load=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def write(path,value):
    with Path(path).open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')


def originals(root):
    rows=load(Path(root)/OLD/'LIFECYCLE_ANALYSIS.json')['scopes_detail']
    assert len(rows)==23
    return [Path(root)/OLD/'lifecycle-audit'/r['scope'] for r in rows]


def original_pins(root,path):
    manifest=load(Path(root)/OLD/'DELIVERY_MANIFEST.json')['files']
    return {name:manifest[str((path/name).relative_to(root))] for name in FILES}


def make_index(root,paths,*,original_root=None):
    """Driver only: explicit paths, no live discovery. Local fixture pins if copied."""
    root=Path(root);groups={}
    for path in paths:
        begin=load(path/'begin.json');scope=load(path/'scope.json');snapshot=load(path/'snapshot.json')
        context={k:begin['source_kind'] if k=='source_kind' else begin['spec'][k] for k in CONTEXT};sid=context['session_id']
        if sid not in groups:groups[sid]={**context,'initial_counts':scope['counts_before'],'terminal_counts':None,'scopes':[]}
        group=groups[sid];prior=group['scopes'][-1] if group['scopes'] else None
        group['scopes'].append(dict(scope_id=scope['scope_id'],path=str(path.relative_to(root)),
            files=original_pins(original_root,path) if original_root else {n:sha(path/n) for n in FILES},
            predecessor_scope_id=prior['scope_id'] if prior else None,
            predecessor_snapshot_sha256=load(root/prior['path']/'scope.json')['snapshot_sha256'] if prior else None))
        group['terminal_counts']=[[k,snapshot['counts'][k]] for k in KINDS]
    return dict(format=FORMAT,coverage='declared_window_only',sessions=list(groups.values()))


def rewrite(path,change):
    docs={name:load(path/name) for name in FILES if name!='events.jsonl'};change(docs)
    snapshot=docs['snapshot.json'];pin=hashlib.sha256(json.dumps(snapshot,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
    docs['scope.json']['snapshot_sha256']=pin
    for r in docs['receipts.json']:
        if r['adapted']:r['value']['snapshot_sha256']=pin
    for name,value in docs.items():(path/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    docs['status.json']['artifact_files']={n:sha(path/n) for n in FILES if n!='status.json'}
    (path/'status.json').write_text(json.dumps(docs['status.json'],indent=2,allow_nan=False)+'\n')


def empty_bridge(left,right,path):
    """Artificial saved-data empty scope inserted between real scopes, NOT an observation."""
    path.mkdir()
    for name in FILES:shutil.copyfile(left/name,path/name)
    begin=load(left/'begin.json');scope=load(left/'scope.json');snapshot=load(left/'snapshot.json');status=load(left/'status.json')
    moment=(scope['closed_s']+load(right/'scope.json')['started_s'])/2;scope_id='synthetic-empty-bridge'
    baseline=[[k,snapshot['counts'][k]] for k in KINDS]
    begin.update(scope_id=scope_id,started_s=moment,counts_before=baseline)
    scope.update(scope_id=scope_id,started_s=moment,closed_s=moment,counts_before=baseline)
    status.update(scope_id=scope_id,status='empty',completed_computations=0,failed_attempts=0,receipt_adapted=0,receipt_rejected=0)
    for name,value in [('begin.json',begin),('scope.json',scope),('status.json',status),('inputs.json',dict(inputs=[],attempts=[])),('receipts.json',[])]:
        (path/name).write_text(json.dumps(value,indent=2)+'\n')
    (path/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in (
        dict(kind='before_attach',observed_s=moment,scope_id=scope_id),dict(kind='attached',observed_s=moment),
        dict(kind='drained_and_detached',observed_s=moment,workers_alive=[],wrappers_restored=True))))
    rewrite(path,lambda _:None)


def exercise(root,name,output):
    assert name in CASES;root=Path(root);output=Path(output);output.mkdir(exist_ok=False);scope_root=output/'scopes';scope_root.mkdir()
    paths=[]
    for index in (0,1):
        source=root/OLD/f'lifecycle-audit/reattach/scope-{index}';dest=scope_root/f'scope-{index}';dest.mkdir()
        for filename in FILES:shutil.copyfile(source/filename,dest/filename)
        paths.append(dest)
    if name in ('empty_bridge_valid','empty_bridge_omitted','observer_counter_decrease'):
        middle=scope_root/'bridge';empty_bridge(*paths,middle)
        if name=='observer_counter_decrease':rewrite(middle,lambda d:d['snapshot.json'].update(observer_errors=1))
        paths.insert(1,middle)
    if name=='counter_gap':
        def gap(d):
            d['begin.json']['counts_before'][-1][1]=0;d['scope.json']['counts_before'][-1][1]=0
        rewrite(paths[1],gap)
    if name=='retained_history_changed':
        def retained(d):d['snapshot.json']['histories']['callbacks'][0]['callback_entry_s']-=.000001
        rewrite(paths[1],retained)
    if name=='continued_after_io_error':
        rewrite(paths[0],lambda d:d['status.json'].update(fixture_error='injected persisted IO error',status='operation_failed'))
    if name=='recorded_body_failure_continues':
        rewrite(paths[0],lambda d:d['status.json'].update(body_error=dict(type='RuntimeError',message='synthetic saved-data body failure'),status='body_failed'))
    if name=='incomplete_predecessor':
        rewrite(paths[0],lambda d:d['status.json'].update(cleanup_errors=[dict(type='OSError',message='synthetic saved-data close failure')],artifact_files_complete=False,status='cleanup_failed'))
    if name=='joint_relabel_nonauthentication':
        def relabel(d):
            label='jointly-forged-local-session';d['begin.json']['spec']['session_id']=label;d['scope.json']['session_id']=label;d['status.json']['session_id']=label
            for r in d['receipts.json']:
                if r['adapted']:r['value']['bundle']['session_id']=label
        for path in paths:rewrite(path,relabel)
    if name=='child_invalid_schema':
        # Pin the corruption too: a child schema failure must not truncate later entries.
        (paths[0]/'begin.json').write_text('[]\n')
        invalid_status=load(paths[0]/'status.json');invalid_status['artifact_files']['begin.json']=sha(paths[0]/'begin.json')
        (paths[0]/'status.json').write_text(json.dumps(invalid_status,indent=2)+'\n')
        doc=make_index(scope_root,paths[1:]);normal=make_index(root,[root/OLD/'lifecycle-audit/reattach/scope-0'],original_root=root)
        first=normal['sessions'][0]['scopes'][0];first['path']='scope-0';first['files']={n:sha(paths[0]/n) for n in FILES}
        doc['sessions'][0]['initial_counts']=normal['sessions'][0]['initial_counts'];doc['sessions'][0]['scopes'].insert(0,first)
        second=doc['sessions'][0]['scopes'][1];second['predecessor_scope_id']=first['scope_id'];second['predecessor_snapshot_sha256']=load(paths[0]/'scope.json')['snapshot_sha256']
    else:doc=make_index(scope_root,paths)
    session=doc['sessions'][0];entries=session['scopes'];raw=None
    if name=='duplicate_json_key':raw=b'{"format":"one","format":"two"}'
    elif name=='missing_file_pin':entries[0]['files'].pop('status.json')
    elif name=='absolute_path':entries[0]['path']='/outside'
    elif name=='parent_path':entries[0]['path']='../outside'
    elif name=='empty_path_component':entries[0]['path']='scope-0//child'
    elif name=='duplicate_scope_id':entries[1]['scope_id']=entries[0]['scope_id']
    elif name=='duplicate_session_id':doc['sessions'].append(copy.deepcopy(session))
    elif name=='duplicate_scope_path':entries[1]['path']=entries[0]['path']
    elif name=='wrong_coverage':doc['coverage']='entire_real_session'
    elif name=='live_source':session['source_kind']='live_callback_delivery'
    elif name=='wrong_session_context':session['session_id']='other-session'
    elif name=='wrong_config_context':session['config_id']='other-config'
    elif name=='wrong_boot_context':session['host_boot_id']='other-boot'
    elif name=='wrong_camera_context':session['cameras'][0]['sensor_id']='other-camera'
    elif name=='wrong_initial_counts':session['initial_counts'][0][1]+=1
    elif name=='wrong_terminal_counts':session['terminal_counts'][0][1]+=1
    elif name=='wrong_predecessor_id':entries[1]['predecessor_scope_id']='omitted-scope'
    elif name=='wrong_predecessor_pin':entries[1]['predecessor_snapshot_sha256']='0'*64
    elif name=='reversed_scopes':
        entries.reverse();entries[0].update(predecessor_scope_id=None,predecessor_snapshot_sha256=None)
        entries[1].update(predecessor_scope_id=entries[0]['scope_id'],predecessor_snapshot_sha256=load(scope_root/entries[0]['path']/'scope.json')['snapshot_sha256'])
        session['initial_counts']=load(scope_root/entries[0]['path']/'scope.json')['counts_before']
        session['terminal_counts']=[[k,load(scope_root/entries[1]['path']/'snapshot.json')['counts'][k]] for k in KINDS]
    elif name=='omitted_first':entries.pop(0);entries[0].update(predecessor_scope_id=None,predecessor_snapshot_sha256=None)
    elif name=='omitted_last':entries.pop()
    elif name=='empty_bridge_omitted':entries.pop(1)
    elif name=='child_hash_changed':(paths[0]/'begin.json').write_bytes((paths[0]/'begin.json').read_bytes()+b' ')
    elif name=='child_missing_file':(paths[0]/'begin.json').unlink()
    elif name=='declared_suffix_window':
        entries.pop(0);entries[0].update(predecessor_scope_id=None,predecessor_snapshot_sha256=None)
        session['initial_counts']=load(paths[-1]/'scope.json')['counts_before']
    manifest=output/'index.json'
    if raw is not None:manifest.write_bytes(raw)
    else:write(manifest,doc)
    pin=sha(manifest)
    if name=='wrong_index_digest':pin='0'*64
    if name=='missing_index_digest':pin=None
    before={str(p.relative_to(output)):sha(p) for p in output.rglob('*') if p.is_file()}
    result=review_index(scope_root,manifest,manifest_sha256=pin)
    after={str(p.relative_to(output)):sha(p) for p in output.rglob('*') if p.is_file()};assert before==after
    assert result.evidence_consistent==(name in CONSISTENT),(name,result)
    if name=='repeated_index':assert result==review_index(scope_root,manifest,manifest_sha256=pin)
    if name in ('child_missing_file','child_invalid_schema','child_hash_changed','incomplete_predecessor'):assert len(result.entries)==2
    if name=='recorded_body_failure_continues':assert result.entries[0].review.recorded_status=='body_failed'
    if name=='counter_gap':assert all(e.review.evidence_consistent for e in result.entries) and any(i.code=='counter_continuity_gap' for i in result.issues)
    if name=='retained_history_changed':assert all(e.review.evidence_consistent for e in result.entries) and any(i.code.startswith('retained_history_changed:') for i in result.issues)
    record=dict(case=name,result=asdict(result),input_files_unchanged=True,input_sha256=before,
        synthetic_saved_data_only=True,no_runtime_callbacks=True,nonauthentication_or_coverage_counterexample=name in COUNTEREXAMPLES)
    write(output/'result.json',record);return record


def run(root,output):
    root=Path(root);output=Path(output);output.mkdir(exist_ok=False);old_paths=originals(root)
    pair=[root/OLD/f'lifecycle-audit/reattach/scope-{i}' for i in (0,1)]
    original=[]
    for name,paths in (('original_reattach_pair',pair),('original_all_declared_scopes',old_paths)):
        doc=make_index(root,paths,original_root=root);manifest=output/(name+'.json');write(manifest,doc)
        result=review_index(root,manifest,manifest_sha256=sha(manifest))
        assert result.evidence_consistent==(name=='original_reattach_pair')
        assert len(result.entries)==len(paths) and result.verified_edges==1
        if len(paths)==23:assert sum(e.review.classification=='recorded_incomplete' for e in result.entries)==3
        original.append(dict(case=name,result=asdict(result)))
    cases=[exercise(root,n,output/n) for n in CASES]
    result=dict(original=original,cases=cases,case_count=len(CASES),corruption_or_continuity_refused=len(CASES)-len(CONSISTENT),
        consistent_fixture_cases=len(CONSISTENT),nonauthentication_or_coverage_counterexamples=len(COUNTEREXAMPLES),
        all_authority_false=True,actual_exposure_wall_s=None,production_changes=False,native_transport=False,movement=False,
        note='Original pair and explicit 23-scope list only; failures retained, no successful subset. Additional copies/empty bridge are artificial saved-data fixtures, not observed sessions. No cross-run replay or archive completeness proof.')
    write(output/'summary.json',result);return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    print(json.dumps(run(Path(__file__).resolve().parents[1],args.output)))
