"""Opt-in synthetic transcript corpus. No runtime backend, owner or transport."""
import argparse
import copy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

from tools.offline_scope_index import read_index
from tools.offline_scope_review import parse
from tools import pending_protocol_draft as p
from tools import pending_trace_review as t

VALID=('normal','six_views','old_hold_future','one_ns_before','stop_no_frame','stop_after_old_expiry',
    'hash_failure','save_failure','compute_failure','cancel_before_compute','unknown_reserve_native',
    'unknown_result_native','declared_refusal','late_reply_retains_unknown','joint_context_replacement',
    'repeat_document','finish_after_replaced_old_hold')
INVALID=('hash_before_both_reserve_claims','save_before_both_seal_claims','compute_before_both_seal_claims',
    'wrong_send_actor','wrong_send_target','wrong_reply_direction','wrong_reply_backend','duplicate_result',
    'duplicate_send','duplicate_reply','retry_after_unknown','upgrade_unknown_ack','unknown_unsent',
    'unknown_without_stop','cancel_before_stop','diagnostic_before_cancel','diagnostic_rewrites_error',
    'work_after_failure','work_after_stop','completed_missing_native_result','success_suffix_after_failure',
    'changed_send_record','result_before_compute_done','duplicate_hash','duplicate_seal_ready',
    'save_done_without_begin','compute_done_without_begin','duplicate_origin','missing_origin','second_frame',
    'request_regression','time_regression','late_reply_without_uncertainty','reply_observed_before_send',
    'future_reply_observation','reply_exact_deadline','old_hold_exact_boundary','hash_exact_deadline',
    'result_issued_injection','finish_exact_deadline','missing_finish','event_after_finish','malformed_wire',
    'unknown_action','authority_injection','duplicate_root','float_time','bool_time','event_extra',
    'old_hold_bool','depth_overflow','byte_overflow','missing_required_view','wrong_session')
INPUT_FILES=('docs/PRECOMPUTE_INTEGRATION_REVIEW.json','tools/pending_protocol_draft.py','docs/PENDING_PROTOCOL_DRAFT.md',
    'tools/pending_trace_review.py','tools/pending_trace_cases.py','docs/PENDING_TRACE_REVIEW.md',
    'reports/straight-followup-20261102-23/SUMMARY.json',
    'reports/straight-followup-20261102-23/pending-protocol-audit/joint_context_replacement_counterexample/result.json',
    'reports/straight-followup-20261102-23/pending-protocol-audit/repeat_snapshot_counterexample/result.json',
    'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample/input.json',
    'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample/result.json')
sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()


def wire(doc):return json.dumps(doc,separators=(',',':'),ensure_ascii=True,allow_nan=False)


def message(kind,request_id,**payload):
    doc=dict(protocol=p.PROTOCOL,scope=p.SCOPE,kind=kind,session_id='fictional-session',request_id=request_id)
    if kind!='stop':doc['capabilities']=list(p.CAPABILITIES)
    return dict(doc,**payload)


def normal():
    record=dict(clock_domain=p.CLOCK,host_boot_id='fictional-boot',generation=1,frame_id='fictional-frame',
        captured_sim_ns=100_000_000,required_views=['front','left','right','back'],
        available_ns=10_000_000_000,deadline_ns=10_800_000_000)
    digests=[[v,str(i+1)*64] for i,v in enumerate(record['required_views'])]
    reserve=message('reserve',1,record=record)
    seal=message('seal',2,record=record,view_digests=digests)
    result=message('result_ref',3,record=record,view_digests=digests,seal_request_id=2)
    events=[]
    def event(action,actor='owner',target='local',**payload):
        events.append(dict(action=action,actor=actor,target=target,at_ns=10_000_000_000+len(events)*1_000_000,**payload))
    def phase(request):
        for b in t.BACKENDS:event('send',target=b,wire=wire(request))
        kind=request['kind']
        for b in t.BACKENDS:
            status=dict(backend=b,phase=dict(reserve='reserved',seal='sealed',result_ref='consumed')[kind],
                record=record,view_digests=[] if kind=='reserve' else digests,
                seal_request_id=None if kind=='reserve' else 2,observed_ns=10_000_000_000+len(events)*1_000_000)
            event('reply',b,'owner',wire=wire(message('ack',request['request_id'],for_kind=kind,accepted=True,status=status)))
    event('freeze','capture','owner',wire=wire(reserve));phase(reserve)
    event('hash_begin');event('hash_done',wire=wire(seal));phase(seal)
    event('save_begin');event('save_done');event('compute_begin','owner','controller')
    event('compute_done','controller','owner');event('result','controller','owner',wire=wire(result));phase(result)
    event('finish',outcome='completed_claim')
    return dict(format=t.FORMAT,scope=t.SCOPE,session_id='fictional-session',required_views=record['required_views'],
        old_hold_deadline_ns=None,events=events)


def locate(doc,action,kind=None,backend=None):
    return next(i for i,e in enumerate(doc['events']) if e['action']==action
        and (kind is None or json.loads(e['wire'])['kind']==kind)
        and (backend is None or e['actor']==backend or e['target']==backend))


def cleanup(doc,error=None):
    events=doc['events'];now=events[-1]['at_ns'] if events else 100_000_000_000
    def add(action,**payload):
        nonlocal now
        now+=1_000_000;events.append(dict(action=action,actor='owner',target='local',at_ns=now,**payload))
    if error is not None:add('failure',error=error)
    for b in t.BACKENDS:
        add('send',wire=wire(message('stop',9,reason='failure' if error else 'terminal')));events[-1]['target']=b
    add('cancel')
    if error is not None:add('diagnostic',error=error)
    add('finish',outcome='failed_claim' if error else 'stopped_claim')
    return doc


def uncertain(phase='reserve'):
    doc=normal();idx=next(i for i,e in enumerate(doc['events']) if e['action']=='reply' and e['actor']=='native'
        and json.loads(e['wire'])['for_kind']==phase)
    late=copy.deepcopy(doc['events'][idx]);doc['events']=doc['events'][:idx]
    doc['events'].append(dict(action='ack_unknown',actor='owner',target='local',at_ns=late['at_ns'],
        request_id=1 if phase=='reserve' else 3,backend='native',error='ack_timeout_claim'))
    cleanup(doc,'ack_timeout_claim')
    return doc,late


def fixture(name):
    p.check(name in VALID+INVALID,'unknown_case');doc=normal()
    events=doc['events']
    def pos(action,kind=None,backend=None):return locate(doc,action,kind,backend)
    def change_wire(index,fn):
        d=json.loads(doc['events'][index]['wire']);fn(d);doc['events'][index]['wire']=wire(d)
    if name in ('stop_no_frame','stop_after_old_expiry'):
        doc['events']=[]
        if name.endswith('expiry'):doc['old_hold_deadline_ns']=1
        cleanup(doc)
    if name in ('hash_failure','save_failure','compute_failure','cancel_before_compute'):
        stage=dict(hash_failure='hash_begin',save_failure='save_begin',compute_failure='compute_begin',
            cancel_before_compute='save_done')[name]
        doc['events']=events[:pos(stage)+1];cleanup(doc,'KeyboardInterrupt' if name=='cancel_before_compute' else name)
    if name in ('unknown_reserve_native','unknown_result_native','late_reply_retains_unknown','retry_after_unknown',
        'upgrade_unknown_ack','unknown_unsent','unknown_without_stop'):
        doc,late=uncertain('result_ref' if name=='unknown_result_native' else 'reserve')
        if name=='late_reply_retains_unknown':
            index=locate(doc,'cancel');late.update(action='late_reply',at_ns=doc['events'][index]['at_ns'])
            doc['events'].insert(index,late)
        if name=='retry_after_unknown':
            index=locate(doc,'ack_unknown')+1;retry=copy.deepcopy(doc['events'][1]);retry['at_ns']=doc['events'][index]['at_ns']
            doc['events'].insert(index,retry)
        if name=='upgrade_unknown_ack':
            index=locate(doc,'ack_unknown')+1;late['at_ns']=doc['events'][index]['at_ns'];doc['events'].insert(index,late)
        if name=='unknown_unsent':doc['events'][locate(doc,'ack_unknown')]['request_id']=99
        if name=='unknown_without_stop':doc['events']=[e for e in doc['events'] if e['action']!='send' or json.loads(e['wire'])['kind']!='stop']
    if name=='declared_refusal':
        idx=pos('reply','ack','native');doc['events']=events[:idx+1]
        change_wire(idx,lambda d:(d.update(accepted=False),d['status'].update(phase='tripped')))
        cleanup(doc,'declared_refusal')
    if name in ('cancel_before_stop','diagnostic_before_cancel','diagnostic_rewrites_error','work_after_failure','success_suffix_after_failure'):
        doc=normal();doc['events']=doc['events'][:locate(doc,'hash_begin')+1];cleanup(doc,'original_hash_failure')
        if name=='cancel_before_stop':
            e=doc['events'].pop(locate(doc,'cancel'));doc['events'].insert(locate(doc,'failure')+1,e)
        if name=='diagnostic_before_cancel':
            e=doc['events'].pop(locate(doc,'diagnostic'));doc['events'].insert(locate(doc,'cancel'),e)
        if name=='diagnostic_rewrites_error':doc['events'][locate(doc,'diagnostic')]['error']='replacement_error'
        if name=='work_after_failure':
            index=locate(doc,'failure')+1;doc['events'].insert(index,copy.deepcopy(normal()['events'][locate(normal(),'hash_done')]))
        if name=='success_suffix_after_failure':doc['events'][-1]['outcome']='completed_claim'
    if name in ('six_views','joint_context_replacement'):
        if name=='six_views':doc['required_views']+=['up','down']
        for e in events:
            if name=='joint_context_replacement':e['at_ns']+=100_000_000
            if 'wire' not in e:continue
            d=json.loads(e['wire']);container=d['status'] if d['kind']=='ack' else d;r=container.get('record')
            if name=='six_views':
                if r is not None:r['required_views']+=['up','down']
                if container.get('view_digests'):container['view_digests']+=[['up','5'*64],['down','6'*64]]
            else:
                if r is not None:r['available_ns']+=100_000_000;r['deadline_ns']+=100_000_000
                if 'observed_ns' in container:container['observed_ns']+=100_000_000
            e['wire']=wire(d)
    if name=='old_hold_future':doc['old_hold_deadline_ns']=10_500_000_000
    if name=='finish_after_replaced_old_hold':
        doc['old_hold_deadline_ns']=10_020_000_000;events[-1]['at_ns']=10_500_000_000
    if name=='one_ns_before':events[-1]['at_ns']=10_799_999_999
    if name in ('hash_before_both_reserve_claims','save_before_both_seal_claims','compute_before_both_seal_claims'):
        kind='reserve' if name.startswith('hash') else 'seal'
        idx=next(i for i,e in enumerate(events) if e['action']=='reply' and e['actor']=='native' and json.loads(e['wire'])['for_kind']==kind)
        events.pop(idx)
    if name=='wrong_send_actor':events[pos('send')]['actor']='controller'
    if name=='wrong_send_target':events[pos('send')]['target']='local'
    if name=='wrong_reply_direction':events[pos('reply')]['target']='controller'
    if name=='wrong_reply_backend':change_wire(pos('reply'),lambda d:d['status'].update(backend='native'))
    if name in ('duplicate_result','duplicate_send','duplicate_reply','duplicate_hash','duplicate_seal_ready','duplicate_origin','second_frame'):
        action=dict(duplicate_result='result',duplicate_send='send',duplicate_reply='reply',duplicate_hash='hash_begin',
            duplicate_seal_ready='hash_done',duplicate_origin='freeze',second_frame='freeze')[name]
        idx=pos(action);extra=copy.deepcopy(events[idx]);events.insert(idx+1,extra)
        if name=='second_frame':
            d=json.loads(extra['wire']);d['record']['frame_id']='different-frame';extra['wire']=wire(d)
    if name=='work_after_stop':
        doc['events']=[];cleanup(doc);e=copy.deepcopy(events[0]);e['at_ns']=doc['events'][1]['at_ns'];doc['events'].insert(2,e)
    if name=='completed_missing_native_result':events.pop(next(i for i,e in enumerate(events) if e['action']=='reply' and e['actor']=='native' and json.loads(e['wire'])['for_kind']=='result_ref'))
    if name=='changed_send_record':change_wire(pos('send'),lambda d:d['record'].update(frame_id='replacement'))
    if name=='result_before_compute_done':events.pop(pos('compute_done'))
    if name=='save_done_without_begin':events.pop(pos('save_begin'))
    if name=='compute_done_without_begin':events.pop(pos('compute_begin'))
    if name=='missing_origin':events.pop(0)
    if name=='request_regression':change_wire(pos('hash_done'),lambda d:d.update(request_id=1))
    if name=='time_regression':events[1]['at_ns']=events[0]['at_ns']-1
    if name=='late_reply_without_uncertainty':events[pos('reply')]['action']='late_reply'
    if name=='reply_observed_before_send':change_wire(pos('reply'),lambda d:d['status'].update(observed_ns=10_000_000_000))
    if name=='future_reply_observation':change_wire(pos('reply'),lambda d:d['status'].update(observed_ns=10_700_000_000))
    if name=='reply_exact_deadline':events[pos('reply')]['at_ns']=10_800_000_000
    if name=='old_hold_exact_boundary':doc['old_hold_deadline_ns']=events[pos('hash_begin')]['at_ns']
    if name=='hash_exact_deadline':events[pos('hash_begin')]['at_ns']=10_800_000_000
    if name=='result_issued_injection':change_wire(pos('result'),lambda d:d.update(issued_steady_s=10))
    if name=='finish_exact_deadline':events[-1]['at_ns']=10_800_000_000
    if name=='missing_finish':events.pop()
    if name=='event_after_finish':events.append(copy.deepcopy(events[-1]))
    if name=='malformed_wire':events[pos('send')]['wire']='{'
    if name=='unknown_action':events[0]['action']='issue_motion_permit'
    if name=='authority_injection':doc['motion_authority']=True
    if name=='float_time':events[0]['at_ns']=10_000_000_000.
    if name=='bool_time':events[0]['at_ns']=True
    if name=='event_extra':events[0]['fallback']=True
    if name=='old_hold_bool':doc['old_hold_deadline_ns']=True
    if name=='missing_required_view':doc['required_views'].pop()
    if name=='wrong_session':doc['session_id']='other-session'
    raw=wire(doc).encode()
    if name=='duplicate_root':raw=raw.replace(b'"scope":"offline_transcript_only"',b'"scope":"offline_transcript_only","scope":"offline_transcript_only"')
    if name=='depth_overflow':raw=b'['*(t.MAX_DEPTH+1)+b'0'+b']'*(t.MAX_DEPTH+1)
    if name=='byte_overflow':raw=b' '*(t.MAX_BYTES+1)
    return raw


def evaluate(raw,repeat=False):
    review=asdict(t.review_trace(raw))
    return dict(review=review,repeated_review=asdict(t.review_trace(raw)) if repeat else None)


def write(path,value):
    with Path(path).open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')


def run(root,output,pins_path,pins_digest):
    root=Path(root);output=Path(output);pin_raw=read_index(pins_path)
    p.check(hashlib.sha256(pin_raw).hexdigest()==pins_digest,'input_pin_file_hash')
    pins=parse(pin_raw);p.check(type(pins) is dict and set(pins)==set(INPUT_FILES),'input_pin_schema')
    raw={n:read_index(root/n) for n in INPUT_FILES}
    p.check(all(hashlib.sha256(raw[n]).hexdigest()==pins[n] for n in INPUT_FILES),'input_pin_mismatch')
    design=parse(raw[INPUT_FILES[0]])
    p.check(len(design['test_plan'])==25 and all(x['status']=='planned_not_executed' for x in design['test_plan']),
        'prior_runtime_scenarios_unexecuted')
    prior=parse(raw[INPUT_FILES[6]])
    p.check(prior['task']=='2N' and prior['production_guard_fixed'] is False and prior['one_time_consumption_implemented'] is False,'prior_limits')
    joint=parse(raw[INPUT_FILES[7]]);repeat=parse(raw[INPUT_FILES[8]])
    p.check(joint['schema_consistent'] and joint['result']['review']['synthetic_reference_matches']
        and repeat['result']['review']==repeat['result']['repeated_review'],'prior_snapshot_counterexamples')
    old=parse(raw[INPUT_FILES[-1]])
    p.check(hashlib.sha256(raw[INPUT_FILES[-2]]).hexdigest()==old['input_sha256'] and all(
        old[b][0]['accepted'] and old[b][0]['deadline_steady_s']>10.8 for b in ('python','receiver')),'prior_backend_counterexample')
    output.mkdir(exist_ok=False);rows=[]
    for name in VALID+INVALID:
        folder=output/name;folder.mkdir();data=fixture(name)
        with (folder/'trace.bin').open('xb') as stream:stream.write(data)
        try:
            result=evaluate(data,name=='repeat_document');consistent=result['review']['consistent'];error=None
        except p.DraftError as exc:result=None;error=str(exc);consistent=False
        row=dict(case=name,expected_consistent=name in VALID,consistent=consistent,
            outcome_matches=consistent==(name in VALID),result=result,error=error,input_sha256=sha(folder/'trace.bin'),
            synthetic_only=True,actual_runtime_backend_executed=False,production_changes=False,native_transport=False,movement=False)
        p.check(sha(folder/'trace.bin')==hashlib.sha256(data).hexdigest(),'saved_input_changed')
        write(folder/'result.json',row);rows.append(row)
    p.check(read_index(pins_path)==pin_raw and all(read_index(root/n)==raw[n] for n in INPUT_FILES),'input_changed')
    summary=dict(cases=rows,valid_cases=len(VALID),invalid_refused=sum(not r['consistent'] for r in rows),
        outcomes_match=all(r['outcome_matches'] for r in rows),input_sha256=pins,formal_retries=0,
        new_runtime_backend_experiments=0,prior_runtime_scenarios_still_not_executed=25,
        prior_2n_counterexamples_preserved=True,prior_2l_joint_resign_unfixed=True,
        production_changes=False,native_transport=False,movement=False,
        scope='caller-authored synthetic transcript role/order/claimed ACK and failure-ledger lint only, NOT execution or owner')
    write(output/'summary.json',summary);return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--input-pins',type=Path,required=True);parser.add_argument('--input-pins-sha256',required=True);args=parser.parse_args()
    result=run(Path(__file__).resolve().parents[1],args.output,args.input_pins,args.input_pins_sha256)
    print(json.dumps(dict(valid_cases=result['valid_cases'],invalid_refused=result['invalid_refused'],outcomes_match=result['outcomes_match'])))
    raise SystemExit(0 if result['outcomes_match'] else 1)
