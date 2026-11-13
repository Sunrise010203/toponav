"""Opt-in fixed-seed differential audit of SEALED portable tools, NOT endpoints.

Known differences remain differences; this driver never repairs or re-signs
old inputs, changes old validators, or strips indices/ledgers for equivalence.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import random
import shutil
import string
import sys

from tools import pending_protocol_draft as p
from tools import pending_protocol_cases as protocol
from tools import pending_trace_cases as traces
from tools import pending_cpp_cases as s
from tools import pending_cpp_reference_cases as q
from tools import pending_trace_cpp_cases as r
from tools import pending_cpp_difference_review as d
from tools.offline_scope_index import read_index
from tools.offline_scope_review import parse

INPUT_FILES=('tools/pending_protocol_draft.py','tools/pending_protocol_cases.py','tools/pending_trace_review.py','tools/pending_trace_cases.py',
    'tools/pending_protocol_cpp_draft.hh','tools/pending_protocol_cpp_probe.cc','tools/pending_cpp_cases.py',
    'tools/pending_protocol_cpp_reference.hh','tools/pending_protocol_cpp_reference_probe.cc','tools/pending_cpp_reference_cases.py',
    'tools/pending_trace_cpp_json.hh','tools/pending_trace_cpp_review.hh','tools/pending_trace_cpp_probe.cc','tools/pending_trace_cpp_cases.py',
    'tools/pending_cpp_difference_review.py','tools/pending_cpp_differential_cases.py','docs/PENDING_CPP_DIFFERENCES.json',
    'docs/PENDING_CPP_DIFFERENTIAL_REVIEW.md','docs/PRECOMPUTE_INTEGRATION_REVIEW.json',
    'reports/straight-followup-20261103-27/SUMMARY.json',
    'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample/input.json',
    'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample/result.json')
MODES=('schema','reference','trace')
SOURCES=dict(schema='pending_protocol_cpp_probe.cc',reference='pending_protocol_cpp_reference_probe.cc',trace='pending_trace_cpp_probe.cc')
BINARIES=dict(schema='pending-schema-probe',reference='pending-reference-probe',trace='pending-trace-probe')
SEED=2026110328
sha=s.sha;write=s.write


def registry(raw):
    doc=parse(raw)
    p.check(type(doc) is dict and doc.get('format')=='pending_cpp_registered_differences_v1' and doc.get('scope')=='offline_fixed_cases_only','registry_schema')
    p.check(doc.get('seed')==SEED and doc.get('variants_per_family')==3,'registry_seed')
    p.check(doc.get('runtime_owner_implemented') is doc.get('production_guard_fixed') is doc.get('movement_authorized') is False,'registry_authority')
    return doc


def mutate(family,nonce,k,pad):
    raw=protocol.fixture('reserve')['message'];doc=json.loads(raw)
    if family=='unchanged':return raw
    if family=='whitespace':return pad+raw+b'\n'
    if family=='escaped_ascii':return raw.replace(b'fictional-session',b'fictional-\\u0073ession')
    if family=='identity_surrogate':return raw.replace(b'fictional-session',b'\\ud800')
    if family=='record_overflow':doc['record']['captured_sim_ns']=p.MAX_NS+k
    if family=='signed_minimum':doc['record']['available_ns']=-2**63
    if family=='huge_integer':return raw.replace(b'"request_id":1',b'"request_id":'+str(k%9+1).encode()*5000)
    if family in ('infinity','nan','exponent'):
        token={'infinity':b'Infinity','nan':b'NaN','exponent':b'1e3'}[family]
        return raw.replace(b'"available_ns":10000000000',b'"available_ns":'+token)
    if family=='view_element_type':doc['record']['required_views'][3]=k
    if family=='refusal_phase':
        doc=json.loads(protocol.fixture('declared_refusal')['reply']);doc['status']['phase']='consumed'
    if family=='unicode_unknown_key':doc['非'*(33+k%8)]=nonce
    if family=='surrogate_unknown_key':doc['\ud800']=nonce
    if family=='duplicate_then_capacity':return b'{'+b','.join(b'"x":'+str(i).encode() for i in range(17))+b'}'
    if family=='malformed_json':return b'{'
    if family=='duplicate_escaped_key':return raw.replace(b'"request_id":1',b'"request_id":1,"request_\\u0069d":1')
    if family=='unknown_kind':doc['kind']='bad-'+nonce
    if family=='missing_kind':doc.pop('kind')
    if family=='caps_downgrade':doc['capabilities'].pop()
    if family=='zero_request':doc['request_id']=0
    if family=='extra_motion':doc['motion_authority']=True
    if family=='raw_invalid_utf8':return b'\xff'+nonce.encode()
    if family=='negative_zero_available':
        doc['record'].update(available_ns=0,deadline_ns=p.LEASE_NS)
        return protocol.wire(doc).replace(b'"available_ns":0',b'"available_ns":-0')
    if family=='maximum_deadline':doc['record'].update(available_ns=p.MAX_NS-p.LEASE_NS,deadline_ns=p.MAX_NS)
    p.check(family in ('record_overflow','signed_minimum','view_element_type','refusal_phase','unicode_unknown_key',
        'surrogate_unknown_key','unknown_kind','missing_kind','caps_downgrade','zero_request','extra_motion','maximum_deadline'),'unknown_mutation')
    return protocol.wire(doc)


def reference_fixture(raw):
    f=protocol.fixture('reserve');f.update(message=raw,reserve=raw,seal=None,reply=None);return f


def corpus(reg):
    p.check(sys.get_int_max_str_digits()==4300,'python_integer_digit_limit_must_be_4300')
    rng=random.Random(reg['seed']);rows=[]
    for family,errors in reg['packet_families'].items():
        pair=[errors['python_error'],errors['cpp_error']];invalid=pair[0] is not None
        for variant in range(reg['variants_per_family']):
            nonce=''.join(rng.choice(string.ascii_lowercase) for _ in range(16));k=rng.randrange(1,10000)
            pad=b' '*rng.randrange(1,8)+b'\t';anchor=rng.choice((1,2));packet=mutate(family,nonce,k,pad)
            for mode in MODES:
                if mode=='trace' and family=='raw_invalid_utf8':continue
                f=reference_fixture(packet) if mode=='reference' else None
                if mode=='trace':
                    trace=traces.normal();trace['events'][anchor]['wire']=packet.decode('utf-8');data=traces.wire(trace).encode()
                else:data=q.envelope(f) if f else packet
                relation='exact' if pair[0]==pair[1] else 'trace_violation_codes_only' if mode=='trace' else 'refusal_error_only'
                rule=dict(relation=relation,error_pair=pair,violation_index=anchor if mode=='trace' else None,
                    expected_returned_result=True if mode=='trace' else not invalid,
                    expected_consistent=family=='unchanged' if mode=='trace' else not invalid,
                    expected_first_codes=(pair if invalid else ['send_unprepared_or_changed']*2) if mode=='trace' and family!='unchanged' else None)
                rows.append(dict(case=mode+'--'+family+'--'+str(variant),mode=mode,data=data,fixture=f,rule=rule,
                    family=family,variant=variant,seed=reg['seed']))
    for family,errors in reg['outer_trace_families'].items():
        doc=traces.normal();raw=None
        if family=='empty':raw=b''
        if family=='utf8':raw=b'\xff'
        if family=='bad_json':raw=b'{'
        if family=='depth':raw=b'['*13+b'0'+b']'*13
        if family=='huge_at':doc['events'][0]['at_ns']=p.MAX_NS+1
        if family=='unicode_key':doc['非'*40]=1
        if family=='duplicate_capacity':raw=b'{'+b','.join(b'"x":'+str(i).encode() for i in range(17))+b'}'
        if family=='bool_time':doc['events'][0]['at_ns']=True
        if family=='unknown_action':doc['events'][0]['action']='not-a-runtime-action'
        if family=='float_time':doc['events'][0]['at_ns']=1.0
        if family=='huge_old_hold':doc['old_hold_deadline_ns']=p.MAX_NS+1
        pair=[errors['python_error'],errors['cpp_error']]
        rows.append(dict(case='outer--'+family,mode='trace',data=traces.wire(doc).encode() if raw is None else raw,fixture=None,
            rule=dict(relation='exact' if pair[0]==pair[1] else 'refusal_error_only',error_pair=pair,
                expected_returned_result=False,expected_consistent=False,expected_first_codes=None),family=family,variant=None,seed=None))
    for name,entry in reg['envelope_cases'].items():
        f=protocol.fixture('declared_refusal' if name.startswith('refusal') else 'stop_no_frame')
        if name.endswith('float_views'):f['kwargs']['required_views']=(1.5,)
        if name=='stop_surrogate_views':f['kwargs']['required_views']=('\ud800',)
        if name=='stop_large_ignored_reserve':f['reserve']=b'x'*8194
        rows.append(dict(case='wrapper--'+name,mode='reference',data=q.envelope(f),fixture=f,
            rule=dict(relation='reference_envelope_refusal',expected_returned_result=True,expected_consistent=True,expected_first_codes=None,**entry),
            family=name,variant=None,seed=None))
    names=('joint_context_replacement','repeat_document','late_reply_retains_unknown','unknown_result_native','declared_refusal',
        'stop_no_frame','finish_after_replaced_old_hold','success_suffix_after_failure',
        'edge--late_unupgraded_future_observation','edge--late_unupgraded_wrong_original_record')
    available={n:(raw,e) for n,raw,e in r.corpus()}
    for n in names:
        raw,e=available[n];rows.append(dict(case='preserve--'+n,mode='trace',data=raw,fixture=None,
            rule=dict(relation='exact',expected_returned_result=True,expected_consistent=e,expected_first_codes=None),family=n,variant=None,seed=None))
    p.check(len({f['case'] for f in rows})==len(rows),'duplicate_case');return tuple(rows)


def python_fact(f):
    mode=f['mode']
    if mode=='schema':
        a=s.python_decode(f['data']);fact=dict(returned_result=a['schema_consistent'],consistent=a['schema_consistent'],result=a['decoded'],error=a['error'])
    elif mode=='reference':
        a=q.python_review(f['fixture']);fact=dict(returned_result=a['consistent'],consistent=a['consistent'],result=a['result'],error=a['error'])
    else:
        a=r.python_review(f['data']);fact=dict(returned_result=a['schema_valid'],consistent=a['consistent'],result=a['result'],error=a['error'])
    return json.loads(d.canonical(fact))


def cpp_fact(mode,doc):
    key={'schema':'schema_consistent','reference':'consistent','trace':'schema_valid'}[mode]
    return dict(returned_result=doc[key],consistent=doc[key] if mode=='schema' else doc['consistent'],
        result=doc['decoded'] if mode=='schema' else doc['result'],error=doc['error'])


def evaluate_observation(f,py,cpp,failure):
    if failure is not None or cpp is None:
        return dict(relation='execution_failure',registered_expectation_matches=False,exact_equal=False,ledger_equal=None,authority_safe=False,changes=[])
    review=d.assess(f['mode'],py,cpp,f['rule']);rule=f['rule'];checks=[]
    checks.append(py['returned_result'] is rule['expected_returned_result'] and py['consistent'] is rule['expected_consistent'])
    # Explicit envelope differences have their OWN expected C++ stage/outcome.
    if rule['relation']=='reference_envelope_refusal':checks.append(cpp['returned_result'] is False and cpp['consistent'] is False)
    else:checks.append(cpp['returned_result'] is py['returned_result'] and cpp['consistent'] is py['consistent'])
    if not py['returned_result'] and 'error_pair' in rule:checks.append([py['error'],cpp['error']]==rule['error_pair'])
    first=rule.get('expected_first_codes')
    if first is not None:
        a=py['result']['review']['violations'];b=cpp['result']['review']['violations']
        checks.append(bool(a and b) and a[0][0]==b[0][0]==rule['violation_index'] and [a[0][1],b[0][1]]==first)
    review['registered_expectation_matches']=review['registered_expectation_matches'] and all(checks)
    return review


def compile_probes(root,output):
    compiler=shutil.which('g++');p.check(compiler is not None,'compiler_missing');binaries={}
    for mode in MODES:
        folder=output/mode;folder.mkdir();binary=folder/BINARIES[mode]
        command=[compiler,'-std=c++17','-O2','-Wall','-Wextra','-Werror','-pedantic',str(root/'tools'/SOURCES[mode]),'-o',str(binary)]
        write(folder/'compile.command.json',dict(command=command,cwd=str(root),compiler_path=compiler,
            compiler_resolved_path=str(Path(compiler).resolve()),compiler_sha256=sha(Path(compiler).resolve()),
            sealed_probe_source=True,no_gazebo_or_production_headers=True))
        r.saved_run([compiler,'--version'],folder,'compiler-version',20)
        r.saved_run(command,folder,'compile',60,cwd=root);binaries[mode]=binary
    return binaries


def cpp_run(binary,f):
    return (s.cpp_decode if f['mode']=='schema' else q.cpp_review if f['mode']=='reference' else r.cpp_review)(binary,f['data'])


def run(root,output,pins_path,pins_digest):
    root=Path(root);output=Path(output);pin_raw=read_index(pins_path)
    p.check(hashlib.sha256(pin_raw).hexdigest()==pins_digest,'input_pin_file_hash');pins=parse(pin_raw)
    p.check(type(pins) is dict and set(pins)==set(INPUT_FILES),'input_pin_schema');raw={n:read_index(root/n) for n in INPUT_FILES}
    p.check(all(hashlib.sha256(raw[n]).hexdigest()==pins[n] for n in INPUT_FILES),'input_pin_mismatch')
    reg=registry(raw['docs/PENDING_CPP_DIFFERENCES.json']);design=parse(raw['docs/PRECOMPUTE_INTEGRATION_REVIEW.json'])
    prior=parse(raw['reports/straight-followup-20261103-27/SUMMARY.json']);counter=parse(raw[INPUT_FILES[-1]])
    p.check(prior['task']=='2R' and prior['production_guard_fixed'] is prior['one_time_consumption_implemented'] is False,'prior_limits')
    p.check(len(design['requirements'])==14 and all(x['status']=='not_implemented' for x in design['requirements']),'runtime_requirements')
    p.check(len(design['test_plan'])==25 and all(x['status']=='planned_not_executed' for x in design['test_plan']),'runtime_scenarios')
    p.check(hashlib.sha256(raw[INPUT_FILES[-2]]).hexdigest()==counter['input_sha256'] and all(
        counter[b][0]['accepted'] and counter[b][0]['deadline_steady_s']>10.8 for b in ('python','receiver')),'prior_counterexample')
    # Assert environment before output/compiler; do not silently change Python.
    p.check(sys.get_int_max_str_digits()==4300,'python_integer_digit_limit_must_be_4300')
    output.mkdir(exist_ok=False);binaries=compile_probes(root,output);binary_hashes={k:sha(v) for k,v in binaries.items()};rows=[]
    for f in corpus(reg):
        folder=output/f['case'];folder.mkdir();(folder/'input.bin').write_bytes(f['data'])
        if f['fixture'] is not None:
            for role in ('message','reserve','seal','reply'):
                if f['fixture'][role] is not None:(folder/(role+'.bin')).write_bytes(f['fixture'][role])
            write(folder/'controls.json',q.controls(f['fixture']))
        write(folder/'case.json',{k:v for k,v in f.items() if k not in ('data','fixture')})
        py=python_fact(f);cpp=cpp_run(binaries[f['mode']],f);doc=cpp['document'];fact=None if doc is None or cpp['failure'] is not None else cpp_fact(f['mode'],doc)
        write(folder/'python.json',py);write(folder/'cpp-fact.json',fact)
        (folder/'cpp.stdout.bin').write_bytes(cpp['stdout']);(folder/'cpp.stderr.bin').write_bytes(cpp['stderr'])
        review=evaluate_observation(f,py,fact,cpp['failure'])
        row=dict(case=f['case'],mode=f['mode'],review=review,cpp_returncode=cpp['returncode'],cpp_failure=cpp['failure'],cpp_document=doc,
            evidence_sha256={p.name:sha(p) for p in sorted(folder.iterdir())},synthetic_only=True,actual_runtime_backend_executed=False,
            production_changes=False,native_transport=False,movement=False)
        write(folder/'result.json',row);rows.append(row)
    p.check(read_index(pins_path)==pin_raw and all(read_index(root/n)==raw[n] for n in INPUT_FILES),'input_changed')
    p.check(all(sha(binaries[k])==v for k,v in binary_hashes.items()),'binary_changed')
    relations={n:sum(x['review']['relation']==n for x in rows) for n in sorted({x['review']['relation'] for x in rows})}
    summary=dict(cases=rows,case_count=len(rows),seed=SEED,relations=relations,registered_expectations_match=all(x['review']['registered_expectation_matches'] for x in rows),
        all_results_exactly_equivalent=all(x['review']['exact_equal'] for x in rows),
        trace_ledgers_compared=sum(x['review']['ledger_equal'] is not None for x in rows),trace_ledger_mismatches=sum(x['review']['ledger_equal'] is False for x in rows),
        input_sha256=pins,binary_sha256=binary_hashes,formal_runs=1,formal_retries=0,formal_compilations=3,portable_cpp_executions=len(rows),
        new_runtime_backend_experiments=0,prior_runtime_scenarios_still_not_executed=25,production_changes=False,native_transport=False,movement=False,
        scope='Fixed-seed observations and registered differences of SEALED portable drafts; known non-equivalence retained, no validator/production fix or runtime endpoints')
    write(output/'summary.json',summary);return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--input-pins',type=Path,required=True);parser.add_argument('--input-pins-sha256',required=True);args=parser.parse_args()
    result=run(Path(__file__).resolve().parents[1],args.output,args.input_pins,args.input_pins_sha256)
    print(json.dumps({k:result[k] for k in ('case_count','relations','registered_expectations_match','all_results_exactly_equivalent','trace_ledger_mismatches')}))
    raise SystemExit(0 if result['registered_expectations_match'] else 1)
