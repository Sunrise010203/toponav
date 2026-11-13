"""Opt-in fixed-seed boundaries/multi-fault audit of SEALED TPT1; no new endpoint."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import random
import struct
import sys
from tools import pending_common_domain as c
from tools import pending_common_domain_cases as prior
from tools import pending_common_boundary_review as review
from tools.offline_scope_index import read_index

SEED=2026110330
SOURCE_FILES=('tools/pending_common_boundary_review.py','tools/pending_common_boundary_cases.py',
    'docs/PENDING_COMMON_BOUNDARY_AUDIT.json','docs/PENDING_COMMON_BOUNDARY_AUDIT.md','tests/test_pending_common_boundaries.py')
INPUT_FILES=tuple(dict.fromkeys((*prior.INPUT_FILES,*SOURCE_FILES,
    'reports/straight-followup-20261103-29/SUMMARY.json','reports/straight-followup-20261103-29/COMMON_ANALYSIS.json',
    'reports/straight-followup-20261103-29/common-domain-audit/summary.json','reports/straight-followup-20261103-29/PLAN.json')))
FAULTS=('oversize','bad_magic','controls_null','message_null','reserve_over','seal_over','reply_over',
    'trailing','controls_invalid','reply_null','packet_utf8')
ERRORS=('domain_byte_bound','domain_magic','domain_required_field','domain_required_field',
    'domain_field_bound','domain_field_bound','domain_field_bound','domain_trailing','domain_controls',
    'domain_reply_required','message_utf8')
CONFLICTS=(frozenset(('controls_null','controls_invalid')),frozenset(('message_null','packet_utf8')),frozenset(('reply_over','reply_null')))
sha=prior.sha;write=prior.write;pack=prior.pack;canonical=prior.canonical


def compatible(mask):return not any(pair.issubset(mask) for pair in CONFLICTS)


def priority_masks():
    rng=random.Random(SEED)
    singles=[(f,) for f in FAULTS]
    pairs=[m for m in itertools.combinations(FAULTS,2) if compatible(m)]
    triples=rng.sample([m for m in itertools.combinations(FAULTS,3) if compatible(m)],24)
    return tuple(singles+pairs+triples)


def render_priority(mask,variant):
    # Stable bytes per variant permit removal-of-first-fault counterfactuals.
    rng=random.Random(SEED+variant);noise=bytes(rng.randrange(256) for _ in range(128))
    f=prior.p.fixture('result_ack_native');fields=[canonical(prior.q.controls(f)).encode(),*(f[k] for k in ('message','reserve','seal','reply'))]
    for fault,index in [('controls_null',0),('message_null',1),('reply_null',4)]:
        if fault in mask:fields[index]=None
    for fault,index in [('reserve_over',2),('seal_over',3),('reply_over',4)]:
        if fault in mask:fields[index]=(noise*65)[:8194]
    if 'controls_invalid' in mask:fields[0]=b'{}'
    if 'packet_utf8' in mask:fields[1]=b'\xff'
    raw=pack(fields)
    if 'trailing' in mask:raw+=noise[:3]
    if 'bad_magic' in mask:raw=b'BAD!'+raw[4:]
    if 'oversize' in mask:raw+=(noise*((c.LIMIT+1-len(raw)+127)//128))[:c.LIMIT+1-len(raw)]
    return raw


def corpus(root):
    root=Path(root);rows=[]
    def add(name,data,stage,error=None,**meta):
        rows.append(dict(case=name,data=data,expected_stage=stage,error_pair=None if error is None else [error,error],**meta))
    for f in prior.corpus(root):
        rows.append(dict(f,case='baseline--'+f['case'],family='sealed_2T',seed=None))
    for variant in range(2):
        for mask in priority_masks():
            first=min(mask,key=FAULTS.index);error=ERRORS[FAULTS.index(first)]
            add('priority--'+str(variant)+'--'+'+'.join(mask),render_priority(mask,variant),
                'review_refused' if first=='packet_utf8' else 'domain_refused',error,
                family='priority',seed=SEED,variant=variant,faults=list(mask),first_fault=first)
    base=prior.p.fixture('stop_no_frame');good=canonical(prior.q.controls(base)).encode();message=base['message']
    minimal=[good,message,None,None,None]
    # Genuine payload lengths, including controls, not merely announced caps.
    for index in range(5):
        cap=4096 if index==0 else 8193
        for n in (0,1,cap-1,cap,cap+1,0xfffffffe,0xffffffff):
            fields=list(minimal)
            if n==0xffffffff:
                fields[index]=None;raw=pack(fields)
                stage='domain_refused' if index<2 else 'review_returned';error='domain_required_field' if index<2 else None
            elif n==0xfffffffe:
                prefix=pack(fields[:index])[4:]
                raw=c.MAGIC+prefix+struct.pack('>I',n)+b'\xff'
                stage='domain_refused';error='domain_field_bound'
            else:
                fields[index]=b'x'*n;raw=pack(fields)
                if n>cap:stage='domain_refused';error='domain_field_bound'
                elif index==0:stage='domain_refused';error='domain_controls'
                elif index==1:stage='review_refused';error='message_byte_bound' if n in (0,8193) else 'message_json'
                else:stage='review_returned';error=None
            add(f'field-length--{index}--{n}',raw,stage,error,family='field_length',seed=SEED,field=index,declared_length=n)
    # Null, empty, invalid and exact-cap contexts are ignored only by old branches.
    for source in ('stop_no_frame','declared_refusal'):
        for role in ('reserve','seal','reply'):
            for label,data in [('null',None),('empty',b''),('utf8',b'\xff'),('8192',b'x'*8192),('8193',b'x'*8193),('8194',b'x'*8194)]:
                f=prior.p.fixture(source);f[role]=data
                if label=='8194':stage='domain_refused';error='domain_field_bound'
                elif source=='declared_refusal' and role=='reply':
                    stage='domain_refused' if data is None else 'review_refused'
                    error='domain_reply_required' if data is None else 'message_byte_bound' if len(data) in (0,8193) else 'message_utf8' if data==b'\xff' else 'message_json'
                else:stage='review_returned';error=None
                add('ignored--'+source+'--'+role+'--'+label,prior.encode_fixture(f),stage,error,family='ignored_context',seed=SEED)
    for source in ('stop_no_frame','reserve'):
        for n in (8191,8192,8193,8194):
            f=prior.p.fixture(source);f['message']+=b' '*(n-len(f['message']))
            add('message-padding--'+source+'--'+str(n),prior.encode_fixture(f),
                'domain_refused' if n==8194 else 'review_refused' if n==8193 else 'review_returned',
                'domain_field_bound' if n==8194 else 'message_byte_bound' if n==8193 else None,family='message_padding',seed=SEED)
    for n in (4095,4096,4097):
        fields=list(minimal);fields[0]=good+b' '*(n-len(good))
        add('controls-padding--'+str(n),pack(fields),'domain_refused','domain_field_bound' if n==4097 else 'domain_controls',family='controls_padding',seed=SEED)
    for delta in (-1,0,1):
        # At total cap every individual field fits; controls are intentionally
        # noncanonical. This is NOT a claim that cap-sized controls are valid.
        fields=[good+b' '*(4096-len(good)),b'x'*(8193+delta),b'x'*8193,b'x'*8193,b'x'*8193]
        raw=pack(fields);assert len(raw)==c.LIMIT+delta
        add('total-length--'+str(delta),raw,'domain_refused','domain_byte_bound' if delta==1 else 'domain_controls',family='total_length',seed=SEED,delta=delta)
    raw=pack(minimal);offsets=[];pos=4;cuts={0,1,3,4,len(raw)}
    for item in minimal:
        h=pos;pos+=4;end=pos+(0 if item is None else len(item));offsets.append((h,end,item is None))
        cuts.update(range(h,h+4));cuts.add(end)
        if item is not None:cuts.update((h+4,h+5,end-1))
        pos=end
    for cut in sorted(cuts):
        error=None
        if cut<4:error='domain_magic'
        else:
            for h,end,is_null in offsets:
                if cut<h+4:error='domain_length';break
                if not is_null and cut<end:error='domain_field_bound';break
        add('prefix-cut--'+str(cut),raw[:cut],'review_returned' if error is None else 'domain_refused',error,
            family='prefix_cut',seed=SEED,cut=cut,full_length=len(raw),frontiers=[list(x) for x in offsets])
    assert len({r['case'] for r in rows})==len(rows)
    return tuple(rows)


def run(root,output,pins_path,pins_digest):
    root=Path(root);output=Path(output);pinraw=read_index(pins_path)
    c.need(hashlib.sha256(pinraw).hexdigest()==pins_digest,'input_pin_hash')
    pins=json.loads(pinraw);c.need(set(pins)==set(INPUT_FILES),'input_pin_schema')
    c.need(all(sha(root/n)==h for n,h in pins.items()),'input_pin_mismatch')
    c.need(sys.get_int_max_str_digits()==4300,'python_integer_limit_4300')
    output.mkdir(exist_ok=False);binary=prior.compile_probe(root,output);binary_hash=sha(binary);rows=[]
    for f in corpus(root):
        folder=output/f['case'];folder.mkdir();(folder/'input.bin').write_bytes(f['data'])
        meta={k:v for k,v in f.items() if k!='data'};write(folder/'case.json',meta)
        py=c.review_envelope(f['data']);py2=c.review_envelope(f['data'])
        write(folder/'python.json',py);write(folder/'python-repeat.json',py2)
        cpp=prior.cpp_run(binary,f['data']);cpp2=prior.cpp_run(binary,f['data'])
        for name,run in [('cpp',cpp),('cpp-repeat',cpp2)]:
            (folder/(name+'.stdout.bin')).write_bytes(run['stdout']);(folder/(name+'.stderr.bin')).write_bytes(run['stderr'])
            write(folder/(name+'.exit.json'),dict(returncode=run['returncode']))
        observation=review.assess(meta,py,cpp['document'],py2,cpp2['document'])
        execution_ok=all(not r['stderr'] and r['returncode']==(0 if r['document']['consistent'] else 2) for r in (cpp,cpp2))
        row=dict(case=f['case'],family=f['family'],stage=py['stage'],execution_ok=execution_ok,observation=observation,
            evidence_sha256={p.name:sha(p) for p in sorted(folder.iterdir())})
        write(folder/'result.json',row);rows.append(row)
    c.need(sha(binary)==binary_hash and all(sha(root/n)==h for n,h in pins.items()),'input_or_binary_changed')
    summary=dict(case_count=len(rows),seed=SEED,cases=rows,input_sha256=pins,binary_sha256=binary_hash,
        families={s:sum(r['family']==s for r in rows) for s in sorted({r['family'] for r in rows})},
        stages={s:sum(r['stage']==s for r in rows) for s in ('domain_refused','review_refused','review_returned')},
        exact=sum(r['observation']['exact_equal'] for r in rows),registered_inner_error_differences=sum(r['observation']['relation']=='registered_inner_error_difference' for r in rows),
        expectations_match=all(r['execution_ok'] and r['observation']['expectation_matches'] for r in rows),all_repeated_calls_equal=all(r['observation']['repeated_calls_equal'] for r in rows),
        formal_runs=1,formal_retries=0,formal_compilations=1,portable_cpp_executions=2*len(rows),new_cpp_implementations=0,
        new_runtime_backend_experiments=0,old_validators_fixed=False,universal_equivalence_proved=False,production_changes=False,native_transport=False,movement=False)
    write(output/'summary.json',summary);return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--input-pins',type=Path,required=True);parser.add_argument('--input-pins-sha256',required=True);args=parser.parse_args()
    result=run(Path(__file__).resolve().parents[1],args.output,args.input_pins,args.input_pins_sha256)
    print(json.dumps({k:v for k,v in result.items() if k not in ('cases','input_sha256')}))
    raise SystemExit(0 if result['expectations_match'] else 1)
