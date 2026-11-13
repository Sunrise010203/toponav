"""Opt-in C++/Python OFFLINE contextual review; no runtime endpoint or owner."""
import argparse
import copy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import struct
import subprocess

from tools.offline_scope_index import read_index
from tools.offline_scope_review import parse
from tools import pending_protocol_draft as p
from tools import pending_protocol_cases as old
from tools.pending_cpp_cases import write, sha

INPUT_FILES=('tools/pending_protocol_draft.py','tools/pending_protocol_cases.py',
    'tools/pending_protocol_cpp_draft.hh','tools/pending_protocol_cpp_reference.hh',
    'tools/pending_protocol_cpp_reference_probe.cc','tools/pending_cpp_reference_cases.py',
    'docs/PENDING_CPP_REFERENCE_REVIEW.md','docs/PRECOMPUTE_INTEGRATION_REVIEW.json',
    'reports/straight-followup-20261103-25/SUMMARY.json',
    'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample/input.json',
    'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample/result.json')
ENVELOPE_LIMIT=4+5*4+4096+4*(p.MAX_BYTES+1)
canonical=lambda value:json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)


def corpus():
    rows=[(old.fixture(n),n in old.VALID) for n in old.VALID+old.INVALID]
    def add(name,base,valid,*,operation=None,kwargs=None,raw=None,changes=None):
        f=copy.deepcopy(old.fixture(base));f['case']='edge--'+name
        if operation is not None:f['operation']=operation
        if kwargs:f['kwargs'].update(kwargs)
        for role,update in (changes or {}).items():
            doc=json.loads(f[role]);update(doc);f[role]=old.wire(doc)
        if raw:f.update(raw)
        rows.append((f,valid))
    for role in ('reserve','seal'):
        for name,data in [('utf8',b'\xff'),('duplicate',b'{"x":1,"x":2}'),('empty',b''),('overflow',b' '*8193)]:
            add(role+'_'+name,'result_reference',False,raw={role:data})
        add('reserve_reference_validates_optional_'+role,'reserve',False,raw={role:b'\xff'})
    add('reserve_missing','result_reference',False,raw={'reserve':None})
    for role in ('reserve','seal','message'):
        for field,value in [('session_id','another-session'),('request_id',1)]:
            # The request-id=1 reserve is unchanged; all other mutations break order/reference.
            add(role+'_'+field,'result_reference',role=='reserve' and field=='request_id',
                changes={role:lambda d,k=field,v=value:d.update({k:v})})
    for field,value in [('host_boot_id','other'),('generation',2),('frame_id','other'),
                        ('captured_sim_ns',101),('available_ns',10_100_000_000)]:
        def mutate(d,k=field,v=value):
            d['record'][k]=v
            if k=='available_ns':d['record']['deadline_ns']=v+p.LEASE_NS
        add('seal_record_'+field,'result_reference',False,changes={'seal':mutate})
    add('seal_digest_differs','result_reference',False,
        changes={'seal':lambda d:d['view_digests'][0].__setitem__(1,'e'*64)})
    add('reserve_request_differs','reserve',False,changes={'message':lambda d:d.update(request_id=2)})
    add('seal_not_after_reserve','seal',False,changes={'message':lambda d:d.update(request_id=1)})
    add('seal_wrong_kind','result_reference',False,raw={'seal':old.fixture('reserve')['message']})
    add('context_wrong_kind','result_reference',False,raw={'reserve':old.fixture('seal')['message']})
    for config in (['front','left','right','back'],None,(),('front','left','right','front')):
        add('bad_config_'+str(len(rows)),'result_reference',False,kwargs={'required_views':config})
    for source in ('stop_no_frame','stop_ack_after_deadline','declared_refusal'):
        add(source+'_ignores_bad_context',''+source,True,
            raw={'reserve':b'\xff','seal':b'{bad'},kwargs={'required_views':None})
    add('refusal_wrong_original','declared_refusal',True,
        changes={'reply':lambda d:d['status']['record'].update(frame_id='different')})
    for field,value in [('backend','python'),('review_ns',True),('review_ns',-1),
        ('review_ns',p.MAX_NS+1),('review_ns','10'),('review_ns',10.0),
        ('old_hold_deadline_ns',True),('old_hold_deadline_ns',-1),('old_hold_deadline_ns',p.MAX_NS+1)]:
        add('bad_control_'+field+'_'+str(len(rows)),'result_ack_native',False,kwargs={field:value})
    for source in ('declared_refusal','stop_ack_after_deadline'):
        add(source+'_future',''+source,False,kwargs={'review_ns':99_999_999_999})
        add(source+'_backend',''+source,False,kwargs={'backend':'python'})
        add(source+'_bool_time',''+source,False,kwargs={'review_ns':True})
        add(source+'_identity',''+source,False,changes={'reply':lambda d:d.update(session_id='other')})
    for backend in ('python','native'):
        for base in ('reserve_ack_native','seal_status_native','result_ack_native','stop_ack_after_deadline'):
            for style in ('ack','status'):
                def change(d,b=backend,s=style,source=base):
                    d['status']['backend']=b
                    if s=='status':d['kind']='status';d.pop('accepted',None);d.pop('for_kind',None)
                    elif d['kind']=='status':d.update(kind='ack',accepted=True,for_kind='seal')
                add(backend+'_'+base+'_'+style,base,True,kwargs={'backend':backend},changes={'reply':change})
    for observed,review,oldhold,valid,label in [
        (10_000_000_000,10_000_000_000,None,True,'availability_exact'),
        (9_999_999_999,10_700_000_000,None,False,'before_available'),
        (10_799_999_999,10_799_999_999,None,True,'D_minus_one'),
        (10_800_000_000,10_800_000_000,None,False,'D_exact'),
        (10_800_000_001,10_800_000_001,None,False,'D_plus_one'),
        (10_300_000_000,10_699_999_999,10_700_000_000,True,'hold_minus_one'),
        (10_300_000_000,10_700_000_000,10_700_000_000,False,'hold_exact'),
        (10_300_000_000,10_700_000_001,10_700_000_000,False,'hold_plus_one'),
        (10_300_000_000,10_700_000_000,p.MAX_NS,True,'hold_does_not_extend_D')]:
        add(label,'result_ack_native',valid,kwargs={'review_ns':review,'old_hold_deadline_ns':oldhold},
            changes={'reply':lambda d,n=observed:d['status'].update(observed_ns=n)})
    for delta,valid in [(1,True),(0,False)]:
        def maximum_record(d):
            r=d['status']['record'] if d['kind']=='ack' else d['record']
            r.update(available_ns=p.MAX_NS-p.LEASE_NS,deadline_ns=p.MAX_NS)
            if d['kind']=='ack':d['status']['observed_ns']=p.MAX_NS-delta
        add('max_ns_'+str(delta),'result_ack_native',valid,kwargs={'review_ns':p.MAX_NS-delta},
            changes={role:maximum_record for role in ('message','reserve','seal','reply')})
    add('expired_reference_still_consistent','expired_schema_is_not_admission',True,operation='reference',
        raw={'reserve':old.fixture('expired_schema_is_not_admission')['message'],'seal':None})
    add('expired_reply_refused','reserve_ack_native',False)
    f=rows[-1][0]
    for role in ('message','reserve','seal','reply'):
        d=json.loads(f[role]);(d['status']['record'] if role=='reply' else d['record']).update(available_ns=0,deadline_ns=p.LEASE_NS);f[role]=old.wire(d)
    add('stop_status_bad_phase','stop_ack_after_deadline',False,changes={'reply':lambda d:(
        d.update(kind='status'),d.pop('accepted'),d.pop('for_kind'),d['status'].update(phase='disarmed'))})
    add('reference_reply_kind','disarmed_status',False,operation='reference')
    return tuple(rows)


def controls(f):
    k=f['kwargs'];views=k.get('required_views',('front','left','right','back'))
    return dict(operation=f['operation'],required_views=views,views_frozen=type(views) is tuple,
        backend=k.get('backend'),review_ns=k.get('review_ns'),old_hold_deadline_ns=k.get('old_hold_deadline_ns'))


def envelope(f):
    fields=(old.wire(controls(f)),f['message'],f['reserve'],f['seal'],f['reply'])
    return b'TPQ1'+b''.join(struct.pack('>I',0xffffffff if raw is None else len(raw))+(raw or b'') for raw in fields)


def python_review(f):
    try:return dict(consistent=True,result=old.evaluate(f),error=None)
    except p.DraftError as exc:return dict(consistent=False,result=None,error=str(exc))


def compile_probe(root,output):
    root=Path(root);output=Path(output);compiler=shutil.which('g++');p.check(compiler is not None,'compiler_missing')
    version=subprocess.run([compiler,'--version'],capture_output=True,check=False,timeout=20)
    (output/'compiler-version.stdout.bin').write_bytes(version.stdout);(output/'compiler-version.stderr.bin').write_bytes(version.stderr)
    p.check(version.returncode==0,'compiler_version_failed');binary=output/'pending-reference-probe'
    command=[compiler,'-std=c++17','-O2','-Wall','-Wextra','-Werror','-pedantic',
        str(root/'tools/pending_protocol_cpp_reference_probe.cc'),'-o',str(binary)]
    write(output/'compile.command.json',dict(command=command,cwd=str(root),compiler_path=compiler,
        compiler_resolved_path=str(Path(compiler).resolve()),compiler_sha256=sha(Path(compiler).resolve()),no_gazebo_or_production_headers=True))
    result=subprocess.run(command,cwd=root,capture_output=True,check=False,timeout=60)
    (output/'compile.stdout.bin').write_bytes(result.stdout);(output/'compile.stderr.bin').write_bytes(result.stderr)
    write(output/'compile.exit.json',dict(returncode=result.returncode));p.check(result.returncode==0,'compile_failed');return binary


def cpp_review(binary,data):
    try:
        run=subprocess.run([str(binary)],input=data,capture_output=True,check=False,timeout=10)
        code=run.returncode;stdout=run.stdout;stderr=run.stderr;failure=None
    except subprocess.TimeoutExpired as exc:
        code=None;stdout=exc.stdout or b'';stderr=exc.stderr or b'';failure='review_timeout'
    try:doc=json.loads(stdout)
    except (ValueError,UnicodeError):doc=None;failure=failure or 'review_invalid_output'
    if type(doc) is not dict:doc=None;failure=failure or 'review_output_not_object'
    if doc is not None:
        base=asdict(p.DraftReview('offline_caller_snapshot_not_admission',None,False))
        expected=set(base)|{'consistent','result','error'}
        if set(doc)!=expected or type(doc.get('consistent')) is not bool or code!=(0 if doc['consistent'] else 2):failure='review_output_contract'
        elif canonical({k:doc[k] for k in base})!=canonical(base):failure='review_authority_claim'
        elif doc['consistent']:
            r=doc['result']
            if doc['error'] is not None or type(r) is not dict or set(r)!={'review','repeated_review'}:failure='review_result_contract'
            else:
                for review in (r['review'],r['repeated_review']):
                    if review is None:continue
                    if type(review) is not dict or set(review)!=set(base) or review['actual_exposure_wall_s'] is not None or any(review[k] is not False for k in p.FALSE_FIELDS):failure='review_nested_authority_claim'
                if r['review'] is None:failure='review_missing'
        elif doc['result'] is not None or type(doc['error']) is not str or not doc['error']:failure='review_refusal_contract'
    return dict(returncode=code,stdout=stdout,stderr=stderr,document=doc,failure=failure)


def run(root,output,pins_path,pins_digest):
    root=Path(root);output=Path(output);pin_raw=read_index(pins_path)
    p.check(hashlib.sha256(pin_raw).hexdigest()==pins_digest,'input_pin_file_hash');pins=parse(pin_raw)
    p.check(type(pins) is dict and set(pins)==set(INPUT_FILES),'input_pin_schema')
    raw={n:read_index(root/n) for n in INPUT_FILES}
    p.check(all(hashlib.sha256(raw[n]).hexdigest()==pins[n] for n in INPUT_FILES),'input_pin_mismatch')
    design=parse(raw[INPUT_FILES[7]]);prior=parse(raw[INPUT_FILES[8]]);counter=parse(raw[INPUT_FILES[10]])
    p.check(len(design['requirements'])==14 and all(x['status']=='not_implemented' for x in design['requirements']),'runtime_requirements')
    p.check(len(design['test_plan'])==25 and all(x['status']=='planned_not_executed' for x in design['test_plan']),'runtime_scenarios')
    p.check(prior['task']=='2P' and not prior['production_guard_fixed'],'prior_limits')
    p.check(hashlib.sha256(raw[INPUT_FILES[9]]).hexdigest()==counter['input_sha256'] and all(
        counter[b][0]['accepted'] and counter[b][0]['deadline_steady_s']>10.8 for b in ('python','receiver')),'prior_counterexample')
    output.mkdir(exist_ok=False);binary=compile_probe(root,output);binary_digest=sha(binary);rows=[]
    cases=corpus();p.check(len({f['case'] for f,_ in cases})==len(cases),'duplicate_case')
    for f,expected in cases:
        folder=output/f['case'];folder.mkdir();data=envelope(f);(folder/'envelope.bin').write_bytes(data)
        for role in ('message','reserve','seal','reply'):
            if f[role] is not None:(folder/(role+'.bin')).write_bytes(f[role])
        write(folder/'context.json',controls(f));py=python_review(f);cpp=cpp_review(binary,data);write(folder/'python.json',py)
        (folder/'cpp.stdout.bin').write_bytes(cpp['stdout']);(folder/'cpp.stderr.bin').write_bytes(cpp['stderr']);doc=cpp['document']
        matches=cpp['failure'] is None and py['consistent']==doc['consistent']==expected and canonical(py['result'])==canonical(doc['result'])
        row=dict(case=f['case'],operation=f['operation'],expected_consistent=expected,consistent=py['consistent'],outcome_matches=matches,
            cpp_returncode=cpp['returncode'],cpp_failure=cpp['failure'],cpp_document=doc,
            evidence_sha256={p.name:sha(p) for p in sorted(folder.iterdir())},synthetic_only=True,
            actual_runtime_backend_executed=False,production_changes=False,native_transport=False,movement=False)
        write(folder/'result.json',row);rows.append(row)
    p.check(read_index(pins_path)==pin_raw and all(read_index(root/n)==raw[n] for n in INPUT_FILES),'input_changed')
    p.check(sha(binary)==binary_digest,'binary_changed')
    summary=dict(cases=rows,case_count=len(rows),consistent=sum(r['consistent'] for r in rows),refused=sum(not r['consistent'] for r in rows),
        outcomes_match=all(r['outcome_matches'] for r in rows),input_sha256=pins,binary_sha256=binary_digest,formal_runs=1,formal_retries=0,
        portable_cpp_review_executions=len(rows),new_runtime_backend_experiments=0,prior_runtime_scenarios_still_not_executed=25,
        production_changes=False,native_transport=False,movement=False,
        scope='Fixed synthetic caller-snapshot reference/reply review only; NOT runtime endpoints, origin authentication, trace or consumption')
    write(output/'summary.json',summary);return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--input-pins',type=Path,required=True);parser.add_argument('--input-pins-sha256',required=True);args=parser.parse_args()
    result=run(Path(__file__).resolve().parents[1],args.output,args.input_pins,args.input_pins_sha256)
    print(json.dumps({k:result[k] for k in ('case_count','consistent','refused','outcomes_match')}))
    raise SystemExit(0 if result['outcomes_match'] else 1)
