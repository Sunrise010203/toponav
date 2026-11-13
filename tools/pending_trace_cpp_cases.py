"""Opt-in portable C++/Python caller-authored transcript lint, NOT endpoints."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from tools.offline_scope_index import read_index
from tools.offline_scope_review import parse
from tools import pending_protocol_draft as p
from tools import pending_trace_review as t
from tools import pending_trace_cases as old
from tools.pending_cpp_cases import write, sha

INPUT_FILES=('tools/pending_protocol_draft.py','tools/pending_trace_review.py','tools/pending_trace_cases.py',
    'tools/pending_protocol_cpp_draft.hh','tools/pending_protocol_cpp_reference.hh',
    'tools/pending_trace_cpp_json.hh','tools/pending_trace_cpp_review.hh','tools/pending_trace_cpp_probe.cc',
    'tools/pending_trace_cpp_cases.py','docs/PENDING_CPP_TRACE_REVIEW.md','docs/PRECOMPUTE_INTEGRATION_REVIEW.json',
    'reports/straight-followup-20261103-26/SUMMARY.json',
    'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample/input.json',
    'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample/result.json')
canonical=lambda x:json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False)


def corpus():
    rows=[(n,old.fixture(n),n in old.VALID) for n in old.VALID+old.INVALID]
    def add(name,doc,valid):rows.append(('edge--'+name,old.wire(doc).encode(),valid))
    for action in ('freeze','send','reply','hash_begin','hash_done','save_begin','save_done','compute_begin','compute_done','result','finish'):
        for field in ('actor','target'):
            doc=old.normal();doc['events'][old.locate(doc,action)][field]='wrong-role';add(action+'_'+field,doc,False)
    for count in (0,1,127,128,129):
        doc=old.normal();doc['events']=[]
        if count==1:doc['events']=[dict(action='finish',actor='owner',target='local',at_ns=0,outcome='completed_claim')]
        elif count>1:
            doc['events']=[dict(action='failure',actor='owner',target='local',at_ns=10_000_000_000+i,error='kept') for i in range(count-6)]
            old.cleanup(doc,'kept')
        add('event_count_'+str(count),doc,count in (127,128))
    raw=old.fixture('normal');rows.append(('edge--exact_trace_bytes',raw+b' '*(t.MAX_BYTES-len(raw)),True))
    for name,raw in [('empty',b''),('utf8',b'\xff'),('utf8_surrogate',b'"\xed\xa0\x80"'),
        ('bom',b'\xef\xbb\xbf'+old.fixture('normal')),('trailing',old.fixture('normal')+b'{}'),
        ('depth_exact_schema',b'['*12+b'0'+b']'*12),('depth_excess',b'['*13+b'0'+b']'*13)]:
        rows.append(('edge--'+name,raw,False))
    for size in (8192,8193):
        doc=json.loads(old.fixture('stop_no_frame'))
        for e in doc['events']:
            if 'wire' in e:e['wire']+=' '*(size-len(e['wire']))
        add('stop_wire_chars_'+str(size),doc,size==8192)
    for label,value in [('lone_high','\ud800'),('lone_low','\udc00'),('high_then_ascii','\ud800a'),('pair','\U0001f600')]:
        for field in ('wire','target'):
            doc=old.normal();doc['events'][old.locate(doc,'send')][field]=value;add(label+'_'+field,doc,False)
    doc=old.normal();doc['events'][1]['wire']='非'*8192;add('wire_char_bound_not_byte_bound',doc,False)
    doc=old.normal()
    for e in doc['events']:
        if e['action']=='reply':
            d=json.loads(e['wire']);d['kind']='status';d.pop('accepted');d.pop('for_kind');e['wire']=old.wire(d)
    add('status_claims_not_poll',doc,True)
    doc=json.loads(old.fixture('stop_after_old_expiry'))
    for i,b in enumerate(t.BACKENDS):
        e=dict(action='reply',actor=b,target='owner',at_ns=doc['events'][2]['at_ns'],wire=old.wire(old.message('ack',9,
            for_kind='stop',accepted=True,status=dict(backend=b,phase='tripped',record=None,view_digests=[],seal_request_id=None,
                observed_ns=doc['events'][2]['at_ns']))))
        doc['events'].insert(2+i,e)
    add('stop_reply_after_expiry_no_frame',doc,True)
    for mutation in ('future_observation','wrong_original_record'):
        doc=json.loads(old.fixture('late_reply_retains_unknown'));index=old.locate(doc,'late_reply');d=json.loads(doc['events'][index]['wire'])
        if mutation=='future_observation':d['status']['observed_ns']=p.MAX_NS
        else:d['status']['record']['frame_id']='caller-replacement'
        doc['events'][index]['wire']=old.wire(d);add('late_unupgraded_'+mutation,doc,True)
    for action in ('cancel','diagnostic'):
        doc=json.loads(old.fixture('hash_failure'));i=old.locate(doc,action);doc['events'].insert(i+1,dict(doc['events'][i]));add('duplicate_'+action,doc,False)
    doc=old.normal()
    for e in doc['events']:
        if 'wire' in e:
            d=json.loads(e['wire']);d['request_id']+=p.MAX_ID-3
            c=d['status'] if d['kind']=='ack' else d
            if c.get('seal_request_id') is not None:c['seal_request_id']+=p.MAX_ID-3
            e['wire']=old.wire(d)
    add('maximum_request_sequence',doc,True)
    for delta in (1,0):
        doc=old.normal();offset=p.MAX_NS-p.LEASE_NS-10_000_000_000
        for e in doc['events']:
            e['at_ns']+=offset
            if 'wire' in e:
                d=json.loads(e['wire']);c=d['status'] if d['kind']=='ack' else d
                if c.get('record') is not None:c['record']['available_ns']+=offset;c['record']['deadline_ns']+=offset
                if 'observed_ns' in c:c['observed_ns']+=offset
                e['wire']=old.wire(d)
        doc['events'][-1]['at_ns']=p.MAX_NS-delta;add('max_ns_finish_'+str(delta),doc,delta==1)
    for label,mutate in [('old_hold_minus_one',lambda d:d.update(old_hold_deadline_ns=10_019_000_001)),
        ('old_hold_exact',lambda d:d.update(old_hold_deadline_ns=10_019_000_000)),
        ('bool_actor',lambda d:d['events'][0].update(actor=True)),
        ('unknown_backend',lambda d:d['events'][0].update(backend='native'))]:
        doc=old.normal();mutate(doc);add(label,doc,label=='old_hold_minus_one')
    return tuple(rows)


def python_review(raw):
    try:
        result=old.evaluate(raw,True)
        return dict(schema_valid=True,consistent=result['review']['consistent'],result=result,error=None)
    except p.DraftError as exc:return dict(schema_valid=False,consistent=False,result=None,error=str(exc))


def saved_run(command,output,prefix,timeout,**kwargs):
    try:
        r=subprocess.run(command,capture_output=True,check=False,timeout=timeout,**kwargs)
        code=r.returncode;stdout=r.stdout;stderr=r.stderr;error=None
    except subprocess.TimeoutExpired as exc:
        code=None;stdout=exc.stdout or b'';stderr=exc.stderr or b'';error='timeout'
    (output/(prefix+'.stdout.bin')).write_bytes(stdout);(output/(prefix+'.stderr.bin')).write_bytes(stderr)
    write(output/(prefix+'.exit.json'),dict(returncode=code,error=error));p.check(code==0,prefix+'_failed')


def compile_probe(root,output):
    root=Path(root);output=Path(output);compiler=shutil.which('g++');p.check(compiler is not None,'compiler_missing')
    binary=output/'pending-trace-probe'
    command=[compiler,'-std=c++17','-O2','-Wall','-Wextra','-Werror','-pedantic',str(root/'tools/pending_trace_cpp_probe.cc'),'-o',str(binary)]
    write(output/'compile.command.json',dict(command=command,cwd=str(root),compiler_path=compiler,
        compiler_resolved_path=str(Path(compiler).resolve()),compiler_sha256=sha(Path(compiler).resolve()),no_gazebo_or_production_headers=True))
    saved_run([compiler,'--version'],output,'compiler-version',20)
    saved_run(command,output,'compile',60,cwd=root);return binary


def cpp_review(binary,raw):
    try:
        r=subprocess.run([str(binary)],input=raw,capture_output=True,check=False,timeout=10)
        code=r.returncode;stdout=r.stdout;stderr=r.stderr;failure=None
    except subprocess.TimeoutExpired as exc:
        code=None;stdout=exc.stdout or b'';stderr=exc.stderr or b'';failure='trace_timeout'
    try:doc=json.loads(stdout)
    except (ValueError,UnicodeError):doc=None;failure=failure or 'trace_invalid_output'
    if type(doc) is not dict:doc=None;failure=failure or 'trace_output_not_object'
    if doc is not None:
        base=asdict(p.DraftReview('offline_transcript_not_admission',None,False))
        base.update(runtime_one_time_consumption_enforced=False,cross_run_replay_prevented=False)
        if set(doc)!=set(base)|{'schema_valid','consistent','result','error'} or type(doc.get('schema_valid')) is not bool or type(doc.get('consistent')) is not bool:
            failure='trace_output_contract'
        elif canonical({k:doc[k] for k in base})!=canonical(base):failure='trace_authority_claim'
        elif code!=(2 if not doc['schema_valid'] else 0 if doc['consistent'] else 3):failure='trace_exit_contract'
        elif not doc['schema_valid']:
            if doc['consistent'] or doc['result'] is not None or type(doc['error']) is not str or not doc['error']:failure='trace_refusal_contract'
        else:
            r=doc['result']
            if doc['error'] is not None or type(r) is not dict or set(r)!={'review','repeated_review'}:failure='trace_result_contract'
            else:
                for review in (r['review'],r['repeated_review']):
                    if type(review) is not dict or review.get('actual_exposure_wall_s','missing') is not None or any(review.get(k) is not False for k in t.FALSE_FIELDS):failure='trace_nested_authority_claim'
                    elif type(review.get('consistent')) is not bool or review['consistent'] is not doc['consistent']:failure='trace_nested_consistency'
    return dict(returncode=code,stdout=stdout,stderr=stderr,document=doc,failure=failure)


def run(root,output,pins_path,pins_digest):
    root=Path(root);output=Path(output);pin_raw=read_index(pins_path)
    p.check(hashlib.sha256(pin_raw).hexdigest()==pins_digest,'input_pin_file_hash');pins=parse(pin_raw)
    p.check(type(pins) is dict and set(pins)==set(INPUT_FILES),'input_pin_schema');raw={n:read_index(root/n) for n in INPUT_FILES}
    p.check(all(hashlib.sha256(raw[n]).hexdigest()==pins[n] for n in INPUT_FILES),'input_pin_mismatch')
    design=parse(raw[INPUT_FILES[10]]);prior=parse(raw[INPUT_FILES[11]]);counter=parse(raw[INPUT_FILES[13]])
    p.check(len(design['requirements'])==14 and all(x['status']=='not_implemented' for x in design['requirements']),'runtime_requirements')
    p.check(len(design['test_plan'])==25 and all(x['status']=='planned_not_executed' for x in design['test_plan']),'runtime_scenarios')
    p.check(prior['task']=='2Q' and not prior['production_guard_fixed'] and not prior['one_time_consumption_implemented'],'prior_limits')
    p.check(hashlib.sha256(raw[INPUT_FILES[12]]).hexdigest()==counter['input_sha256'] and all(
        counter[b][0]['accepted'] and counter[b][0]['deadline_steady_s']>10.8 for b in ('python','receiver')),'prior_counterexample')
    output.mkdir(exist_ok=False);binary=compile_probe(root,output);binary_digest=sha(binary);rows=[]
    cases=corpus();p.check(len({n for n,_,_ in cases})==len(cases),'duplicate_case')
    for name,data,expected in cases:
        folder=output/name;folder.mkdir();(folder/'trace.bin').write_bytes(data)
        py=python_review(data);cpp=cpp_review(binary,data);write(folder/'python.json',py)
        (folder/'cpp.stdout.bin').write_bytes(cpp['stdout']);(folder/'cpp.stderr.bin').write_bytes(cpp['stderr']);doc=cpp['document']
        matches=cpp['failure'] is None and py['schema_valid']==doc['schema_valid'] and py['consistent']==doc['consistent']==expected and canonical(py['result'])==canonical(doc['result'])
        row=dict(case=name,expected_consistent=expected,schema_valid=py['schema_valid'],consistent=py['consistent'],outcome_matches=matches,
            cpp_returncode=cpp['returncode'],cpp_failure=cpp['failure'],cpp_document=doc,
            evidence_sha256={f.name:sha(f) for f in sorted(folder.iterdir())},synthetic_only=True,
            actual_runtime_backend_executed=False,production_changes=False,native_transport=False,movement=False)
        write(folder/'result.json',row);rows.append(row)
    p.check(read_index(pins_path)==pin_raw and all(read_index(root/n)==raw[n] for n in INPUT_FILES),'input_changed')
    p.check(sha(binary)==binary_digest,'binary_changed')
    summary=dict(cases=rows,case_count=len(rows),schema_valid=sum(r['schema_valid'] for r in rows),schema_refused=sum(not r['schema_valid'] for r in rows),
        consistent=sum(r['consistent'] for r in rows),semantic_rejected=sum(r['schema_valid'] and not r['consistent'] for r in rows),
        outcomes_match=all(r['outcome_matches'] for r in rows),input_sha256=pins,binary_sha256=binary_digest,formal_runs=1,formal_retries=0,
        portable_cpp_trace_executions=len(rows),pure_cpp_reviews_per_schema_valid_process=2,new_runtime_backend_experiments=0,
        prior_runtime_scenarios_still_not_executed=25,production_changes=False,native_transport=False,movement=False,
        scope='Fixed synthetic caller-authored transcript order and claim ledger only; NOT runtime owner, endpoints, trusted poll, origin or consumption')
    write(output/'summary.json',summary);return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--input-pins',type=Path,required=True);parser.add_argument('--input-pins-sha256',required=True);args=parser.parse_args()
    result=run(Path(__file__).resolve().parents[1],args.output,args.input_pins,args.input_pins_sha256)
    print(json.dumps({k:result[k] for k in ('case_count','schema_valid','schema_refused','consistent','semantic_rejected','outcomes_match')}))
    raise SystemExit(0 if result['outcomes_match'] else 1)
