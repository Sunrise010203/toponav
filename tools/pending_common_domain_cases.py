"""Opt-in fixed TPT1 corpus/audit. No endpoints, old validator fixes or motion."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys
from tools import pending_common_domain as c
from tools import pending_cpp_differential_cases as old
from tools import pending_cpp_reference_cases as q
from tools import pending_protocol_cases as p
from tools.offline_scope_index import read_index

SOURCE_FILES=('tools/pending_common_domain.py','tools/pending_common_domain.hh',
    'tools/pending_common_domain_probe.cc','tools/pending_common_domain_cases.py',
    'docs/PENDING_COMMON_DOMAIN_V1.json','docs/PENDING_COMMON_DOMAIN_V1.md','tests/test_pending_common_domain.py')
INPUT_FILES=tuple(dict.fromkeys((*old.INPUT_FILES,*SOURCE_FILES,
    'reports/straight-followup-20261103-28/SUMMARY.json',
    'reports/straight-followup-20261103-28/CPP_ANALYSIS.json',
    'reports/straight-followup-20261103-28/cpp-differential-audit/summary.json',
    *(f'reports/straight-followup-20261103-28/cpp-differential-audit/preserve--{n}/python.json' for n in
      ('unknown_result_native','late_reply_retains_unknown','edge--late_unupgraded_future_observation','edge--late_unupgraded_wrong_original_record')))))
sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
canonical=lambda doc:json.dumps(doc,sort_keys=True,separators=(',',':'),ensure_ascii=True,allow_nan=False)
write=p.write


def pack(fields):
    return c.MAGIC+b''.join(struct.pack('>I',0xffffffff if raw is None else len(raw))+(raw or b'') for raw in fields)


def encode_fixture(f):
    # Serialization is explicit; raw packets NEVER pass through JSON conversion.
    controls=json.dumps(q.controls(f),sort_keys=True,separators=(',',':'),ensure_ascii=True).encode()
    return pack((controls,*(f[k] for k in ('message','reserve','seal','reply'))))


def corpus(root):
    root=Path(root);rows=[]
    def add(name,data,stage,pair=None):
        rows.append(dict(case=name,data=data,expected_stage=stage,error_pair=pair))
    for name in p.VALID:add('original--'+name,encode_fixture(p.fixture(name)),'review_returned')
    reg=old.registry((root/'docs/PENDING_CPP_DIFFERENCES.json').read_bytes())
    for f in old.corpus(reg):
        if f['mode']=='trace':continue
        fixture=f['fixture']
        if f['mode']=='schema':
            fixture=p.fixture('reserve');fixture.update(operation='schema',message=f['data'])
        wrapper=f['rule']['relation']=='reference_envelope_refusal'
        stage='domain_refused' if wrapper else 'review_returned' if f['rule']['expected_returned_result'] else 'review_refused'
        pair=([('domain_field_bound' if f['family']=='stop_large_ignored_reserve' else 'domain_controls')]*2 if wrapper else
              f['rule']['error_pair'] if stage=='review_refused' else None)
        add('sealed2S--'+f['case'],encode_fixture(fixture),stage,pair)
    base=p.fixture('stop_no_frame');controls=q.controls(base);message=base['message']
    def control_case(name,change):
        obj=copy.deepcopy(controls);change(obj)
        add('controls--'+name,pack((json.dumps(obj,sort_keys=True,separators=(',',':'),ensure_ascii=True).encode(),message,None,None,None)),
            'domain_refused',['domain_controls']*2)
    for name,key,value in [
        ('frozen_false','views_frozen',False),('frozen_integer','views_frozen',1),
        ('bad_operation','operation','trace'),('missing_views','required_views',None),
        ('empty_views','required_views',[]),('reorder_views','required_views',['left','front','right','back']),
        ('duplicate_views','required_views',['front','left','right','front']),
        ('bool_views','required_views',[True]),('float_views','required_views',[1.5]),
        ('surrogate_views','required_views',['\ud800']),('nonascii_views','required_views',['非']),
        ('ignored_backend','backend','native'),('ignored_review','review_ns',1),
        ('ignored_oldhold','old_hold_deadline_ns',1),('negative_ignored','review_ns',-1),
        ('huge_ignored','review_ns',2**63),('bool_ignored','review_ns',True)]:
        control_case(name,lambda d,k=key,v=value:d.update({k:v}))
    control_case('extra',lambda d:d.update(extra=1));control_case('missing',lambda d:d.pop('backend'))
    good=canonical(controls).encode()
    for name,raw in [('space',b' '+good),('unsorted',json.dumps(controls).encode()),
        ('duplicate',good[:-1]+b',"backend":null}'),('minus_zero',good.replace(b'null',b'-0',1)),
        ('nested',b'['*1001+b'0'+b']'*1001),('huge_integer',b'9'*4000),
        ('nonascii',b'\xff'),('empty',b''),('float',b'1.5'),('null',b'null'),
        ('escaped',good.replace(b'front',b'fr\\u006fnt'))]:
        add('controls-raw--'+name,pack((raw,message,None,None,None)),'domain_refused',['domain_controls']*2)
    for name,factor in [('review_bool',True),('review_float',1.0),('review_negative',-1),('review_overflow',2**63),('review_null',None)]:
        f=p.fixture('declared_refusal');f['kwargs']['review_ns']=factor
        add('reply-control--'+name,encode_fixture(f),'domain_refused',['domain_controls']*2)
    for name,key,value in [('backend','backend','unknown'),('hold_bool','old_hold_deadline_ns',True),
        ('hold_overflow','old_hold_deadline_ns',2**63),('hold_negative','old_hold_deadline_ns',-1)]:
        f=p.fixture('declared_refusal');f['kwargs'][key]=value
        add('reply-control--'+name,encode_fixture(f),'domain_refused',['domain_controls']*2)
    f=p.fixture('declared_refusal');f['reply']=None
    add('reply-null',encode_fixture(f),'domain_refused',['domain_reply_required']*2)
    for role in ('reserve','seal','reply'):
        for label,raw in [('null',None),('empty',b''),('utf8',b'\xff'),('cap',b'x'*8193),('overcap',b'x'*8194)]:
            f=p.fixture('stop_no_frame');f[role]=raw
            add('ignored--'+role+'--'+label,encode_fixture(f),'domain_refused' if label=='overcap' else 'review_returned',
                ['domain_field_bound']*2 if label=='overcap' else None)
    for op in ('schema','reference'):
        f=p.fixture('reserve');f['operation']=op;f['message']=b' '*8193
        add('schema-bound--'+op,encode_fixture(f),'review_refused',['message_byte_bound']*2)
    f=p.fixture('reserve');f['message']=b' '*8194
    add('message-overcap',encode_fixture(f),'domain_refused',['domain_field_bound']*2)
    raw=encode_fixture(base)
    for name,data,error in [('empty',b'','domain_magic'),('old_magic',b'TPQ1'+raw[4:],'domain_magic'),
        ('truncated',raw[:-1],'domain_field_bound'),('truncated_header',c.MAGIC+b'\x00','domain_length'),('trailing',raw+b'x','domain_trailing'),
        ('null_controls',c.MAGIC+struct.pack('>I',0xffffffff),'domain_required_field'),
        ('null_message',pack((good,None,None,None,None)),'domain_required_field'),
        ('control_cap',pack((b'x'*4097,message,None,None,None)),'domain_field_bound'),
        ('total_cap',b'x'*(c.LIMIT+1),'domain_byte_bound')]:
        add('envelope--'+name,data,'domain_refused',[error]*2)
    for count in (4,5,6):
        f=p.fixture('stop_no_frame');f['kwargs']['required_views']=tuple(['front','left','right','back','up','down'][:count])
        add('views-valid--'+str(count),encode_fixture(f),'review_returned')
    f=p.fixture('stop_no_frame');f['kwargs']['required_views']=('front','left','right','back','down')
    add('views-valid--down',encode_fixture(f),'review_returned')
    for n in (0,2**63-1):
        f=p.fixture('stop_ack_after_deadline');f['kwargs']['review_ns']=n
        add('reply-time--'+str(n),encode_fixture(f),'review_returned' if n else 'review_refused',None if n else ['future_reply_claim']*2)
    assert len({r['case'] for r in rows})==len(rows)
    return tuple(rows)


def compile_probe(root,output):
    output=Path(output);compiler=shutil.which('g++');assert compiler
    binary=output/'pending-common-domain-probe'
    command=[compiler,'-std=c++17','-O2','-Wall','-Wextra','-Werror','-pedantic',str(Path(root)/'tools/pending_common_domain_probe.cc'),'-o',str(binary)]
    write(output/'compile.command.json',dict(command=command,compiler_resolved_path=str(Path(compiler).resolve()),compiler_sha256=sha(Path(compiler).resolve())))
    for name,args in [('compiler-version',[compiler,'--version']),('compile',command)]:
        run=subprocess.run(args,capture_output=True,timeout=60,check=False)
        (output/(name+'.stdout.bin')).write_bytes(run.stdout);(output/(name+'.stderr.bin')).write_bytes(run.stderr)
        write(output/(name+'.exit.json'),dict(returncode=run.returncode));assert run.returncode==0
    return binary


def cpp_run(binary,data):
    run=subprocess.run([str(binary)],input=data,capture_output=True,timeout=10,check=False)
    return dict(returncode=run.returncode,stdout=run.stdout,stderr=run.stderr,document=json.loads(run.stdout))


def compare(f,py,cpp):
    assert py['stage']==cpp['stage']==f['expected_stage'],(f['case'],py,cpp)
    exact=canonical(py)==canonical(cpp)
    if not exact:
        assert f['expected_stage']=='review_refused' and [py['error'],cpp['error']]==f['error_pair'],(f['case'],py,cpp)
        assert canonical({k:v for k,v in py.items() if k!='error'})==canonical({k:v for k,v in cpp.items() if k!='error'})
    if f['error_pair'] is not None:assert [py['error'],cpp['error']]==f['error_pair'],(f['case'],py,cpp)
    for doc in (py,cpp):
        assert doc['actual_exposure_wall_s'] is None and all(doc[k] is False for k in c.FALSE_FIELDS)
        assert doc['domain_accepted'] is (doc['stage']!='domain_refused')
        assert doc['consistent'] is (doc['stage']=='review_returned')
        if doc['result']:
            for r in doc['result'].values():
                if r:assert r['actual_exposure_wall_s'] is None and all(r[k] is False for k in c.p.FALSE_FIELDS)
    return dict(exact=exact,relation='exact' if exact else 'registered_inner_error_difference')


def run(root,output,pins_path,pins_digest):
    root=Path(root);output=Path(output);pinraw=read_index(pins_path)
    assert hashlib.sha256(pinraw).hexdigest()==pins_digest
    pins=json.loads(pinraw);assert set(pins)==set(INPUT_FILES)
    assert all(sha(root/n)==h for n,h in pins.items())
    assert sys.get_int_max_str_digits()==4300
    output.mkdir(exist_ok=False);binary=compile_probe(root,output);binary_hash=sha(binary);rows=[]
    for f in corpus(root):
        folder=output/f['case'];folder.mkdir();(folder/'input.bin').write_bytes(f['data'])
        write(folder/'case.json',{k:v for k,v in f.items() if k!='data'})
        py=c.review_envelope(f['data']);cpp=cpp_run(binary,f['data'])
        write(folder/'python.json',py);(folder/'cpp.stdout.bin').write_bytes(cpp['stdout']);(folder/'cpp.stderr.bin').write_bytes(cpp['stderr'])
        write(folder/'cpp.exit.json',dict(returncode=cpp['returncode']))
        assert not cpp['stderr'] and cpp['returncode']==(0 if cpp['document']['consistent'] else 2)
        comparison=compare(f,py,cpp['document'])
        row=dict(case=f['case'],stage=py['stage'],comparison=comparison,evidence_sha256={x.name:sha(x) for x in folder.iterdir()})
        write(folder/'result.json',row);rows.append(row)
    assert sha(binary)==binary_hash and all(sha(root/n)==h for n,h in pins.items())
    summary=dict(profile=c.PROFILE,cases=rows,case_count=len(rows),stages={s:sum(r['stage']==s for r in rows) for s in ('domain_refused','review_refused','review_returned')},
        exact=sum(r['comparison']['exact'] for r in rows),registered_inner_error_differences=sum(not r['comparison']['exact'] for r in rows),
        input_sha256=pins,binary_sha256=binary_hash,formal_runs=1,formal_retries=0,formal_compilations=1,portable_cpp_executions=len(rows),
        new_runtime_backend_experiments=0,old_validators_fixed=False,universal_equivalence_proved=False,production_changes=False,native_transport=False,movement=False)
    write(output/'summary.json',summary);return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--input-pins',type=Path,required=True);parser.add_argument('--input-pins-sha256',required=True);args=parser.parse_args()
    result=run(Path(__file__).resolve().parents[1],args.output,args.input_pins,args.input_pins_sha256)
    print(json.dumps({k:v for k,v in result.items() if k not in ('cases','input_sha256')}))
