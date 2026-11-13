"""Opt-in OFFLINE synthetic wire/schema/reference corpus. No transport or owner.

Only the pure pending_protocol_draft functions execute. Source/prior evidence
are read as bytes, never imported. Every successful result has no authority.
"""
import argparse
import copy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

from tools.offline_scope_index import read_index
from tools.offline_scope_review import parse
from tools import pending_protocol_draft as p

VALID=('reserve','seal','result_reference','reserve_ack_python','reserve_ack_native','seal_status_native',
    'result_ack_native','stop_no_frame','stop_ack_after_deadline','declared_refusal','six_views',
    'one_ns_before','disarmed_status','joint_context_replacement_counterexample','repeat_snapshot_counterexample',
    'expired_schema_is_not_admission')
INVALID=('old_protocol','old_wire_kind','capability_downgrade','duplicate_capability','extra_field','missing_field',
    'duplicate_root','duplicate_record','duplicate_snapshot','escaped_duplicate','bool_request','zero_request',
    'bool_generation','zero_generation','negative_ns','float_ns','overflow_ns','wrong_deadline','wrong_clock',
    'wrong_boot','wrong_generation','wrong_frame','joint_resign_against_original','wrong_view_config',
    'missing_digest_view','duplicate_view','uppercase_digest','wrong_digest_order','result_issued','result_expires',
    'wrong_seal_reference','result_before_seal','ack_wrong_phase','ack_wrong_request','ack_wrong_backend',
    'numeric_accepted','missing_pending_status','old_accepted_only_status','ack_wrong_kind','future_ack',
    'ack_exact_boundary','old_hold_exact_boundary','stop_with_frame','byte_overflow','depth_overflow','invalid_utf8',
    'nonobject','null_message','nonfinite_json','long_identity','bool_review_time','missing_seal_context','malformed_json',
    'wrong_ack_digest','wrong_ack_seal','wrong_ack_record','ack_before_availability')
INPUT_FILES=('docs/PRECOMPUTE_INTEGRATION_REVIEW.json','docs/PRECOMPUTE_INTEGRATION_REVIEW.md',
    'tools/pending_protocol_draft.py','tools/pending_protocol_cases.py','docs/PENDING_PROTOCOL_DRAFT.md',
    'simulation/native/receiver_envelope.hh','ros2_ws/src/toponav_bridge/toponav_bridge/protocol.py',
    'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample/input.json',
    'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample/result.json')
sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()


def wire(doc):return json.dumps(doc,ensure_ascii=True,separators=(',',':'),allow_nan=False).encode()


def message(kind,request_id,**payload):
    d=dict(protocol=p.PROTOCOL,scope=p.SCOPE,kind=kind,session_id='fictional-session',request_id=request_id)
    if kind!='stop':d['capabilities']=list(p.CAPABILITIES)
    d.update(payload);return d


def fixture(name):
    p.check(name in VALID+INVALID,'unknown_case')
    record=dict(clock_domain=p.CLOCK,host_boot_id='fictional-boot',generation=1,frame_id='fictional-frame',
        captured_sim_ns=100_000_000,required_views=['front','left','right','back'],available_ns=10_000_000_000,
        deadline_ns=10_800_000_000)
    digests=[[v,str(i+1)*64] for i,v in enumerate(record['required_views'])]
    reserve=message('reserve',1,record=copy.deepcopy(record))
    seal=message('seal',2,record=copy.deepcopy(record),view_digests=copy.deepcopy(digests))
    request=message('result_ref',3,record=copy.deepcopy(record),view_digests=copy.deepcopy(digests),seal_request_id=2)
    snapshot=dict(backend='native',phase='consumed',record=copy.deepcopy(record),view_digests=copy.deepcopy(digests),
        seal_request_id=2,observed_ns=10_300_000_000)
    reply=message('ack',3,for_kind='result_ref',accepted=True,status=snapshot)
    operation='reference';kwargs=dict(required_views=tuple(record['required_views']))
    if name=='reserve':request=copy.deepcopy(reserve)
    if name=='seal':request=copy.deepcopy(seal)
    if name in ('reserve_ack_python','reserve_ack_native'):
        operation='reply';request=copy.deepcopy(reserve);reply.update(request_id=1,for_kind='reserve')
        snapshot.update(backend='python' if name.endswith('python') else 'native',phase='reserved',view_digests=[],
            seal_request_id=None,observed_ns=10_100_000_000)
    if name=='seal_status_native':
        operation='reply';request=copy.deepcopy(seal)
        snapshot.update(phase='sealed',observed_ns=10_200_000_000)
        reply=message('status',2,status=snapshot)
    if name in ('result_ack_native','one_ns_before'):operation='reply'
    if name in ('stop_no_frame','stop_ack_after_deadline'):
        request=message('stop',1,reason='cancel');reserve=seal=None
        if name=='stop_ack_after_deadline':
            operation='reply';snapshot.update(phase='tripped',record=None,view_digests=[],seal_request_id=None,observed_ns=100_000_000_000)
            reply=message('ack',1,for_kind='stop',accepted=True,status=snapshot)
    if name=='declared_refusal':
        operation='reply';reply['accepted']=False;snapshot.update(phase='tripped',observed_ns=100_000_000_000)
    if name=='six_views':
        for d in (reserve,seal,request):d['record']['required_views']+=['up','down']
        for d in (seal,request):d['view_digests']+=[['up','5'*64],['down','6'*64]]
        kwargs['required_views']=('front','left','right','back','up','down')
    if name=='disarmed_status':
        operation='schema';snapshot.update(phase='disarmed',record=None,view_digests=[],seal_request_id=None)
        request=message('status',1,status=snapshot)
    if name=='joint_context_replacement_counterexample':
        for d in (reserve,seal,request):d['record'].update(available_ns=10_700_000_000,deadline_ns=11_500_000_000)
    if name=='repeat_snapshot_counterexample':operation='repeat'
    if name=='expired_schema_is_not_admission':
        operation='schema';request=copy.deepcopy(reserve);request['record'].update(available_ns=0,deadline_ns=800_000_000)
    if name in INVALID:
        if name.startswith('ack_') or name in ('numeric_accepted','missing_pending_status','old_accepted_only_status',
            'future_ack','old_hold_exact_boundary','bool_review_time','wrong_ack_digest','wrong_ack_seal',
            'wrong_ack_record'):
            operation='reply'
        if name=='old_protocol':request['protocol']=1
        if name=='old_wire_kind':request['kind']='command'
        if name=='capability_downgrade':request['capabilities'].pop()
        if name=='duplicate_capability':request['capabilities'].append('reserve')
        if name=='extra_field':request['motion_authority']=True
        if name=='missing_field':request.pop('scope')
        if name=='bool_request':request['request_id']=True
        if name=='zero_request':request['request_id']=0
        if name=='bool_generation':request['record']['generation']=True
        if name=='zero_generation':request['record']['generation']=0
        if name=='negative_ns':request['record']['available_ns']=-1
        if name=='float_ns':request['record']['available_ns']=10_000_000_000.
        if name=='overflow_ns':request['record']['deadline_ns']=p.MAX_NS+1
        if name=='wrong_deadline':request['record']['deadline_ns']+=1
        if name=='wrong_clock':request['record']['clock_domain']='CLOCK_MONOTONIC'
        if name=='wrong_boot':request['record']['host_boot_id']='other-boot'
        if name=='wrong_generation':request['record']['generation']=2
        if name=='wrong_frame':request['record']['frame_id']='other-frame'
        if name=='joint_resign_against_original':request['record'].update(available_ns=10_700_000_000,deadline_ns=11_500_000_000)
        if name=='wrong_view_config':kwargs['required_views']=('front','left','right','back','up')
        if name=='missing_digest_view':request['view_digests'].pop()
        if name=='duplicate_view':request['record']['required_views'][3]='front'
        if name=='uppercase_digest':request['view_digests'][0][1]='A'*64
        if name=='wrong_digest_order':request['view_digests'][0],request['view_digests'][1]=request['view_digests'][1],request['view_digests'][0]
        if name=='result_issued':request['issued_steady_s']=10
        if name=='result_expires':request['expires_steady_s']=11
        if name=='wrong_seal_reference':request['seal_request_id']=1
        if name=='result_before_seal':request['request_id']=2
        if name=='ack_wrong_phase':snapshot['phase']='reserved';snapshot['view_digests']=[];snapshot['seal_request_id']=None
        if name=='ack_wrong_request':reply['request_id']=4
        if name=='ack_wrong_backend':snapshot['backend']='python'
        if name=='numeric_accepted':reply['accepted']=1
        if name=='missing_pending_status':snapshot.pop('record')
        if name=='old_accepted_only_status':reply['status']=dict(version=2,command_seq=1,deadline_steady_s=10)
        if name=='ack_wrong_kind':reply['for_kind']='seal';snapshot['phase']='sealed'
        if name=='future_ack':snapshot['observed_ns']=10_700_000_001
        if name=='stop_with_frame':request=message('stop',1,reason='failure',record=record)
        if name=='long_identity':request['session_id']='s'*129
        if name=='missing_seal_context':seal=None
        if name=='wrong_ack_digest':snapshot['view_digests'][0][1]='f'*64
        if name=='wrong_ack_seal':snapshot['seal_request_id']=1
        if name=='wrong_ack_record':snapshot['record']['generation']=2
        if name=='ack_before_availability':snapshot['observed_ns']=9_999_999_999
    if operation=='reply':
        kwargs.update(backend='native',review_ns=10_799_999_999 if name=='one_ns_before' else
            100_000_000_000 if name in ('declared_refusal','stop_ack_after_deadline') else
            10_800_000_000 if name=='ack_exact_boundary' else 10_700_000_000,old_hold_deadline_ns=None)
        if name=='reserve_ack_python':kwargs['backend']='python'
        if name=='old_hold_exact_boundary':kwargs['old_hold_deadline_ns']=kwargs['review_ns']
        if name=='bool_review_time':kwargs['review_ns']=True
    raw=wire(request);reply_raw=wire(reply)
    if name=='duplicate_root':raw=raw.replace(b'"request_id":3',b'"request_id":3,"request_id":3')
    if name=='escaped_duplicate':raw=raw.replace(b'"request_id":3',b'"request_id":3,"request_\\u0069d":3')
    if name=='duplicate_record':raw=raw.replace(b'"generation":1',b'"generation":1,"generation":1')
    if name=='duplicate_snapshot':
        operation='reply';reply_raw=reply_raw.replace(b'"phase":"consumed"',b'"phase":"consumed","phase":"consumed"')
        kwargs.update(backend='native',review_ns=10_700_000_000)
    if name=='float_ns':pass  # Rejected by JSON integer-only parsing, not silently truncated.
    if name=='byte_overflow':raw=b' '*(p.MAX_BYTES+1)
    if name=='depth_overflow':raw=b'['*(p.MAX_DEPTH+1)+b'0'+b']'*(p.MAX_DEPTH+1)
    if name=='invalid_utf8':raw=b'\xff'
    if name=='nonobject':raw=b'[]'
    if name=='null_message':raw=b'null'
    if name=='nonfinite_json':raw=raw.replace(b'"available_ns":10000000000',b'"available_ns":NaN')
    if name=='malformed_json':raw=b'{'
    return dict(case=name,operation=operation,message=raw,reserve=None if reserve is None else wire(reserve),
        seal=None if seal is None else wire(seal),reply=reply_raw,kwargs=kwargs)


def evaluate(f):
    if f['operation']=='schema':
        message=p.decode_message(f['message']);result=p.DraftReview('decoded_schema_not_admission',message.kind,False)
    elif f['operation']=='reply':result=p.review_reply(f['message'],f['reply'],f['reserve'],f['seal'],**f['kwargs'])
    else:result=p.review_reference(f['message'],f['reserve'],f['seal'],**f['kwargs'])
    repeat=None
    if f['operation']=='repeat':repeat=asdict(p.review_reference(f['message'],f['reserve'],f['seal'],**f['kwargs']))
    return dict(review=asdict(result),repeated_review=repeat)


def write(path,value):
    with Path(path).open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')


def run(root,output,pins_path,pins_digest):
    root=Path(root);output=Path(output);pin_raw=read_index(pins_path)
    p.check(hashlib.sha256(pin_raw).hexdigest()==pins_digest,'input_pin_file_hash')
    pins=parse(pin_raw);p.check(type(pins) is dict and set(pins)==set(INPUT_FILES),'input_pin_schema')
    raw={n:read_index(root/n) for n in INPUT_FILES}
    p.check(all(hashlib.sha256(raw[n]).hexdigest()==pins[n] for n in INPUT_FILES),'input_pin_mismatch')
    design=parse(raw[INPUT_FILES[0]])
    p.check(design['status']=='review_only_not_implemented' and len(design['test_plan'])==25
        and all(t['status']=='planned_not_executed' for t in design['test_plan']),'prior_review_not_implementation')
    original=parse(raw[INPUT_FILES[-2]]);old=parse(raw[INPUT_FILES[-1]])
    p.check(hashlib.sha256(raw[INPUT_FILES[-2]]).hexdigest()==old['input_sha256'],'prior_counterexample_input')
    p.check(original['comparison']['windows'][2]['reported_result_within_window'] is False
        and all(old[b][0]['accepted'] and old[b][0]['deadline_steady_s']>10.8 for b in ('python','receiver')),'prior_unfixed_counterexample')
    output.mkdir(exist_ok=False);rows=[]
    for name in VALID+INVALID:
        folder=output/name;folder.mkdir();f=fixture(name);inputs={}
        for role in ('message','reserve','seal','reply'):
            if f[role] is not None:
                path=folder/(role+'.bin')
                with path.open('xb') as stream:stream.write(f[role])
                inputs[role+'.bin']=sha(path)
        context=dict(case=name,operation=f['operation'],kwargs=f['kwargs']);write(folder/'context.json',context)
        inputs['context.json']=sha(folder/'context.json')
        try:result=evaluate(f);error=None;valid=True
        except p.DraftError as exc:result=None;error=str(exc);valid=False
        row=dict(case=name,expected_schema_consistent=name in VALID,schema_consistent=valid,
            outcome_matches=valid==(name in VALID),result=result,error=error,input_sha256=inputs,
            synthetic_only=True,actual_runtime_backend_executed=False,production_changes=False,native_transport=False,movement=False)
        p.check(all(sha(folder/n)==h for n,h in inputs.items()),'saved_case_input_changed')
        write(folder/'result.json',row);rows.append(row)
    p.check(read_index(pins_path)==pin_raw and all(read_index(root/n)==raw[n] for n in INPUT_FILES),'input_changed')
    summary=dict(cases=rows,valid_cases=len(VALID),invalid_refused=sum(not r['schema_consistent'] for r in rows),
        outcomes_match=all(r['outcome_matches'] for r in rows),input_sha256=pins,formal_retries=0,
        scope='bounded offline schema and caller-snapshot consistency only; not a runtime owner/registry or authenticated source',
        prior_joint_resign_counterexample_preserved=True,prior_2m_runtime_scenarios_still_not_executed=25,
        new_runtime_backend_experiments=0,production_changes=False,native_transport=False,movement=False)
    write(output/'summary.json',summary);return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--input-pins',type=Path,required=True);parser.add_argument('--input-pins-sha256',required=True);args=parser.parse_args()
    result=run(Path(__file__).resolve().parents[1],args.output,args.input_pins,args.input_pins_sha256)
    print(json.dumps(dict(valid_cases=result['valid_cases'],invalid_refused=result['invalid_refused'],outcomes_match=result['outcomes_match'])))
    raise SystemExit(0 if result['outcomes_match'] else 1)
