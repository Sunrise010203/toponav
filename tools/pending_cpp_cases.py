"""Opt-in portable C++ single-packet schema comparison, NOT runtime backends."""
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
from tools import pending_protocol_cases as old

INPUT_FILES=('tools/pending_protocol_draft.py','tools/pending_protocol_cases.py','docs/PENDING_PROTOCOL_DRAFT.md',
    'docs/PRECOMPUTE_INTEGRATION_REVIEW.json','reports/straight-followup-20261103-24/SUMMARY.json',
    'tools/pending_protocol_cpp_draft.hh','tools/pending_protocol_cpp_probe.cc','tools/pending_cpp_cases.py',
    'docs/PENDING_CPP_SCHEMA_REVIEW.md')
EDGE_NAMES=('escaped_ascii_identity','escaped_root_key','escaped_slash','negative_zero_available',
    'maximum_record_ns','maximum_request_id','optional_up_only','optional_down_only','whitespace',
    'exact_byte_bound','expired_record_schema','duplicate_escaped_key','duplicate_escaped_record_key',
    'unpaired_high_surrogate','unpaired_low_surrogate','surrogate_pair_identity','raw_nonascii_identity',
    'overlong_utf8','utf8_surrogate','utf8_too_large','truncated_utf8','bom','trailing_document','trailing_comma',
    'leading_zero','plus_number','negative_request','request_overflow','record_addition_overflow',
    'signed_minimum','huge_integer','exponent_number','infinity','nul_string','raw_control','bad_escape',
    'object_field_overflow','long_unknown_key','deep_array','extra_bracket','empty','whitespace_only')
sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()


def edge(name):
    p.check(name in EDGE_NAMES,'unknown_edge');raw=old.fixture('reserve')['message'];doc=json.loads(raw)
    if name=='escaped_ascii_identity':return raw.replace(b'fictional-session',b'fictional-\\u0073ession')
    if name=='escaped_root_key':return raw.replace(b'"request_id"',b'"request_\\u0069d"')
    if name=='escaped_slash':doc['session_id']='fictional/session';return old.wire(doc).replace(b'fictional/session',b'fictional\\/session')
    if name=='negative_zero_available':doc['record'].update(available_ns=0,deadline_ns=p.LEASE_NS);return old.wire(doc).replace(b'"available_ns":0',b'"available_ns":-0')
    if name=='maximum_record_ns':doc['record'].update(available_ns=p.MAX_NS-p.LEASE_NS,deadline_ns=p.MAX_NS)
    if name=='maximum_request_id':doc['request_id']=p.MAX_ID
    if name=='optional_up_only':doc['record']['required_views'].append('up')
    if name=='optional_down_only':doc['record']['required_views'].append('down')
    if name=='whitespace':return b' \t\r\n'+raw+b'\n'
    if name=='exact_byte_bound':return raw+b' '*(p.MAX_BYTES-len(raw))
    if name=='expired_record_schema':doc['record'].update(available_ns=0,deadline_ns=p.LEASE_NS)
    if name=='duplicate_escaped_key':return raw.replace(b'"request_id":1',b'"request_id":1,"request_\\u0069d":1')
    if name=='duplicate_escaped_record_key':return raw.replace(b'"frame_id":',b'"frame_\\u0069d":"other","frame_id":')
    if name=='unpaired_high_surrogate':return raw.replace(b'fictional-session',b'\\ud800')
    if name=='unpaired_low_surrogate':return raw.replace(b'fictional-session',b'\\udc00')
    if name=='surrogate_pair_identity':return raw.replace(b'fictional-session',b'\\ud83d\\ude00')
    if name=='raw_nonascii_identity':return raw.replace(b'fictional-session','非ASCII'.encode())
    if name=='overlong_utf8':return raw.replace(b'fictional-session',b'\xc0\xaf')
    if name=='utf8_surrogate':return raw.replace(b'fictional-session',b'\xed\xa0\x80')
    if name=='utf8_too_large':return raw.replace(b'fictional-session',b'\xf4\x90\x80\x80')
    if name=='truncated_utf8':return raw+b'\xe2\x82'
    if name=='bom':return b'\xef\xbb\xbf'+raw
    if name=='trailing_document':return raw+b'{}'
    if name=='trailing_comma':return raw[:-1]+b',}'
    if name=='leading_zero':return raw.replace(b'"request_id":1',b'"request_id":01')
    if name=='plus_number':return raw.replace(b'"request_id":1',b'"request_id":+1')
    if name=='negative_request':doc['request_id']=-1
    if name=='request_overflow':doc['request_id']=p.MAX_ID+1
    if name=='record_addition_overflow':doc['record'].update(available_ns=p.MAX_NS,deadline_ns=p.MAX_NS)
    if name=='signed_minimum':doc['record']['available_ns']=-2**63
    if name=='huge_integer':return raw.replace(b'"request_id":1',b'"request_id":'+b'9'*5000)
    if name=='exponent_number':return raw.replace(b'"request_id":1',b'"request_id":1e0')
    if name=='infinity':return raw.replace(b'"request_id":1',b'"request_id":Infinity')
    if name=='nul_string':return raw.replace(b'fictional-session',b'fictional\\u0000session')
    if name=='raw_control':return raw.replace(b'fictional-session',b'fictional\x01session')
    if name=='bad_escape':return raw.replace(b'fictional-session',b'fictional\\qsession')
    if name=='object_field_overflow':return old.wire({f'x{i}':i for i in range(17)})
    if name=='long_unknown_key':return old.wire({'x'*65:0})
    if name=='deep_array':return b'['*(p.MAX_DEPTH+1)+b'0'+b']'*(p.MAX_DEPTH+1)
    if name=='extra_bracket':return raw+b']'
    if name=='empty':return b''
    if name=='whitespace_only':return b' \n\t'
    return old.wire(doc)


def corpus():
    rows=[]
    for name in old.VALID+old.INVALID:
        f=old.fixture(name)
        for role in ('message','reserve','seal','reply'):
            if f[role] is not None:rows.append((name+'--'+role,f[role]))
    rows.extend(('edge--'+name,edge(name)) for name in EDGE_NAMES)
    return tuple(rows)


def python_decode(raw):
    try:
        decoded=json.loads(json.dumps(asdict(p.decode_message(raw)),allow_nan=False))
        return dict(schema_consistent=True,decoded=decoded,error=None)
    except p.DraftError as exc:return dict(schema_consistent=False,decoded=None,error=str(exc))


def write(path,value):
    with Path(path).open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')


def compile_probe(root,output):
    root=Path(root);output=Path(output);compiler=shutil.which('g++')
    p.check(compiler is not None,'compiler_missing')
    version=subprocess.run([compiler,'--version'],capture_output=True,check=False,timeout=20)
    (output/'compiler-version.stdout.bin').write_bytes(version.stdout)
    (output/'compiler-version.stderr.bin').write_bytes(version.stderr)
    p.check(version.returncode==0,'compiler_version_failed')
    binary=output/'pending-schema-probe'
    command=[compiler,'-std=c++17','-O2','-Wall','-Wextra','-Werror','-pedantic',
        str(root/'tools/pending_protocol_cpp_probe.cc'),'-o',str(binary)]
    write(output/'compile.command.json',dict(command=command,cwd=str(root),compiler_path=compiler,
        compiler_resolved_path=str(Path(compiler).resolve()),compiler_sha256=sha(Path(compiler).resolve()),
        no_gazebo_or_production_headers=True))
    result=subprocess.run(command,cwd=root,capture_output=True,check=False,timeout=60)
    (output/'compile.stdout.bin').write_bytes(result.stdout);(output/'compile.stderr.bin').write_bytes(result.stderr)
    write(output/'compile.exit.json',dict(returncode=result.returncode))
    p.check(result.returncode==0,'compile_failed');return binary


def cpp_decode(binary,raw):
    try:
        run=subprocess.run([str(binary)],input=raw,capture_output=True,check=False,timeout=10)
        code=run.returncode;stdout=run.stdout;stderr=run.stderr;failure=None
    except subprocess.TimeoutExpired as exc:
        code=None;stdout=exc.stdout or b'';stderr=exc.stderr or b'';failure='parser_timeout'
    try:doc=json.loads(stdout)
    except (ValueError,UnicodeError):doc=None;failure=failure or 'parser_invalid_output'
    if type(doc) is not dict:doc=None;failure=failure or 'parser_output_not_object'
    if doc is not None:
        expected={'schema_consistent','decoded','error','classification','actual_exposure_wall_s',*p.FALSE_FIELDS}
        if set(doc)!=expected or type(doc['schema_consistent']) is not bool or code!=(0 if doc['schema_consistent'] else 2):failure='parser_output_contract'
        elif doc['classification']!='offline_schema_not_admission' or doc['actual_exposure_wall_s'] is not None or any(doc[k] is not False for k in p.FALSE_FIELDS):failure='parser_authority_claim'
        elif (doc['schema_consistent'] and (type(doc['decoded']) is not dict or doc['error'] is not None)) or (
            not doc['schema_consistent'] and (doc['decoded'] is not None or type(doc['error']) is not str or not doc['error'])):failure='parser_result_contract'
    return dict(returncode=code,stdout=stdout,stderr=stderr,document=doc,failure=failure)


def run(root,output,pins_path,pins_digest):
    root=Path(root);output=Path(output);pin_raw=read_index(pins_path)
    p.check(hashlib.sha256(pin_raw).hexdigest()==pins_digest,'input_pin_file_hash')
    pins=parse(pin_raw);p.check(type(pins) is dict and set(pins)==set(INPUT_FILES),'input_pin_schema')
    raw={n:read_index(root/n) for n in INPUT_FILES}
    p.check(all(hashlib.sha256(raw[n]).hexdigest()==pins[n] for n in INPUT_FILES),'input_pin_mismatch')
    design=parse(raw[INPUT_FILES[3]]);prior=parse(raw[INPUT_FILES[4]])
    p.check(len(design['test_plan'])==25 and all(x['status']=='planned_not_executed' for x in design['test_plan']),
        'prior_runtime_scenarios_unexecuted')
    p.check(prior['task']=='2O' and prior['production_guard_fixed'] is False and prior['one_time_consumption_implemented'] is False,
        'prior_limits_preserved')
    output.mkdir(exist_ok=False);binary=compile_probe(root,output);binary_digest=sha(binary);rows=[]
    packets=corpus();p.check(len({name for name,_ in packets})==len(packets),'duplicate_corpus_case')
    for name,data in packets:
        folder=output/name;folder.mkdir();(folder/'message.bin').write_bytes(data)
        python=python_decode(data);cpp=cpp_decode(binary,data);write(folder/'python.json',python)
        (folder/'cpp.stdout.bin').write_bytes(cpp['stdout']);(folder/'cpp.stderr.bin').write_bytes(cpp['stderr'])
        doc=cpp['document'];matches=cpp['failure'] is None and python['schema_consistent']==doc['schema_consistent'] and python['decoded']==doc['decoded']
        row=dict(case=name,schema_consistent=python['schema_consistent'],outcome_matches=matches,
            cpp_returncode=cpp['returncode'],cpp_failure=cpp['failure'],cpp_document=doc,
            input_sha256=sha(folder/'message.bin'),python_sha256=sha(folder/'python.json'),
            cpp_stdout_sha256=sha(folder/'cpp.stdout.bin'),cpp_stderr_sha256=sha(folder/'cpp.stderr.bin'),
            synthetic_only=True,actual_runtime_backend_executed=False,production_changes=False,native_transport=False,movement=False)
        write(folder/'result.json',row);rows.append(row)
    p.check(read_index(pins_path)==pin_raw and all(read_index(root/n)==raw[n] for n in INPUT_FILES),'input_changed')
    p.check(sha(binary)==binary_digest,'binary_changed')
    summary=dict(cases=rows,packet_count=len(rows),edge_cases=len(EDGE_NAMES),schema_consistent=sum(r['schema_consistent'] for r in rows),
        schema_refused=sum(not r['schema_consistent'] for r in rows),outcomes_match=all(r['outcome_matches'] for r in rows),
        input_sha256=pins,binary_sha256=binary_digest,formal_runs=1,formal_retries=0,
        portable_cpp_parser_executions=len(rows),new_runtime_backend_experiments=0,prior_runtime_scenarios_still_not_executed=25,
        reference_or_trace_review_reimplemented=False,production_changes=False,native_transport=False,movement=False,
        scope='fixed single-packet decode_message schema equivalence only; NOT 2N reference/reply checks, 2O traces, runtime backends or authentication')
    write(output/'summary.json',summary);return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--input-pins',type=Path,required=True);parser.add_argument('--input-pins-sha256',required=True);args=parser.parse_args()
    result=run(Path(__file__).resolve().parents[1],args.output,args.input_pins,args.input_pins_sha256)
    print(json.dumps({k:result[k] for k in ('packet_count','schema_consistent','schema_refused','outcomes_match')}))
    raise SystemExit(0 if result['outcomes_match'] else 1)
