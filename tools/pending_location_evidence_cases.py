"""Opt-in CLOSED snapshot/byte evidence audit; NO old validator/compiler calls."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import struct
from tools import pending_location_evidence as e

ARCHIVE='reports/straight-followup-20261103-30'
SOURCE_FILES=('tools/pending_location_evidence.py','tools/pending_location_evidence_cases.py',
    'docs/PENDING_LOCATION_EVIDENCE_V1.json','docs/PENDING_LOCATION_EVIDENCE_V1.md','tests/test_pending_location_evidence.py')
BASE_INPUTS=(*SOURCE_FILES,'tools/pending_common_boundary_review.py','tools/pending_common_boundary_cases.py',
    'tools/pending_common_domain.py','tools/pending_common_domain.hh','tools/pending_common_domain_probe.cc',
    'docs/PENDING_COMMON_DOMAIN_V1.json','docs/PENDING_COMMON_DOMAIN_V1.md',
    'docs/PENDING_COMMON_BOUNDARY_AUDIT.json','docs/PENDING_COMMON_BOUNDARY_AUDIT.md',
    *(ARCHIVE+'/'+n for n in ('PLAN.json','SUMMARY.json','DELIVERY_MANIFEST.json','BOUNDARY_ANALYSIS.json','COUNTERFACTUAL_LINKS.json','boundary-audit/summary.json')),
    'docs/PENDING_CPP_DIFFERENCES.json','reports/straight-followup-20261103-28/SUMMARY.json',
    'reports/straight-followup-20261103-28/cpp-differential-audit/summary.json',
    *(f'reports/straight-followup-20261103-28/cpp-differential-audit/preserve--{n}/python.json' for n in
      ('unknown_result_native','late_reply_retains_unknown','edge--late_unupgraded_future_observation','edge--late_unupgraded_wrong_original_record')),
    'docs/PRECOMPUTE_INTEGRATION_REVIEW.json',
    *(f'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample/{n}.json' for n in ('input','result')))
STRUCTURAL_ERRORS={'domain_byte_bound','domain_magic','domain_length','domain_required_field','domain_field_bound','domain_trailing'}
sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
canonical=e.canonical


def write(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')


def input_files(root):
    plan=json.loads((Path(root)/ARCHIVE/'PLAN.json').read_bytes())
    names=list(plan['cases']);assert len(names)==532
    assert all(n and all(c.isascii() and (c.isalnum() or c in '-_+.') for c in n) and n not in ('.','..') for n in names)
    return tuple(dict.fromkeys((*BASE_INPUTS,*(f'{ARCHIVE}/boundary-audit/{n}/{part}' for n in names for part in ('input.bin','python.json','cpp.stdout.bin')))))


def _parts(raw):
    pos=4;fields=[]
    for _ in range(5):
        n=struct.unpack('>I',raw[pos:pos+4])[0];pos+=4
        if n==0xffffffff:fields.append(None)
        else:fields.append(raw[pos:pos+n]);pos+=n
    return fields


def _pack(fields):
    return b'TPT1'+b''.join(struct.pack('>I',0xffffffff if p is None else len(p))+(p or b'') for p in fields)


def corpus(root):
    root=Path(root);oldplan=json.loads((root/ARCHIVE/'PLAN.json').read_bytes());rows=[];arch={}
    def add(name,raw,py,cpp=None,*,stage='evidence_returned',error=None,prediction=None,agrees=None,family='declared_snapshot',source_case=None):
        rows.append(dict(case=name,raw=raw,observations={'python':py,'cpp':copy.deepcopy(py) if cpp is None else cpp},
            expected_stage=stage,expected_error=error,expected_prediction_reason=prediction,expected_agreement=agrees,family=family,source_case=source_case))
    for name in oldplan['cases']:
        folder=root/ARCHIVE/'boundary-audit'/name;raw=(folder/'input.bin').read_bytes()
        py=json.loads((folder/'python.json').read_bytes());cpp=json.loads((folder/'cpp.stdout.bin').read_bytes())
        prediction=py['error'] if py['stage']=='domain_refused' and py['error'] in STRUCTURAL_ERRORS else None
        add('archive--'+name,raw,py,cpp,prediction=prediction,agrees=True if prediction else None,family='sealed_2U',source_case=name)
        arch[name]=rows[-1]
    positive=arch['baseline--original--reserve'];py=positive['observations']['python'];raw=positive['raw']
    bad=arch['priority--0--controls_null+message_null'];bad_doc=bad['observations']['python']
    add('claim--positive_on_required_null_bytes',bad['raw'],py,prediction='domain_required_field',agrees=False)
    wrong=copy.deepcopy(bad_doc);wrong['error']='domain_magic'
    add('claim--wrong_reported_error_on_required_null',bad['raw'],wrong,prediction='domain_required_field',agrees=False)
    add('claim--refusal_on_structurally_valid_bytes',raw,bad_doc)
    joint=arch['baseline--original--joint_context_replacement_counterexample']
    add('claim--joint_replacement_still_not_authenticated',joint['raw'],joint['observations']['python'])
    fields=_parts(raw);assert fields[4];fields[4]=b'\xff'+fields[4][1:]
    add('claim--ignored_reply_byte_mutation',_pack(fields),py)
    for name,blob,prediction in [('empty',b'','domain_magic'),('magic',b'BAD!'+raw[4:],'domain_magic'),
        ('header',b'TPT1','domain_length'),('null_then_missing_header',b'TPT1'+struct.pack('>I',0xffffffff),'domain_required_field'),
        ('inspection_cap_minus_one',b'x'*e.ENVELOPE_LIMIT,'domain_magic'),
        ('inspection_cap',b'x'*e.INSPECTION_LIMIT,'domain_byte_bound')]:
        add('scan--'+name,blob,py,prediction=prediction,agrees=False,family='scan_frontier')
    add('scan--inspection_overcap',b'x'*(e.INSPECTION_LIMIT+1),py,stage='evidence_refused',error='evidence_byte_bound',family='scan_frontier')
    add('scan--opaque_controls_and_packets',_pack((b'\xff',b'\x00',b'',None,b'\xff')),py,family='scan_frontier')
    max_error=copy.deepcopy(bad_doc);max_error['error']='e'*e.OBSERVATION_ERROR_MAX_CHARS
    add('claim--max_error_chars',raw,max_error)
    field_bound=copy.deepcopy(bad_doc);field_bound['error']='domain_field_bound'
    add('scan--huge_declared_length',b'TPT1'+struct.pack('>I',0xfffffffe)+b'\xff',field_bound,
        prediction='domain_field_bound',agrees=True,family='scan_frontier')
    add('scan--continue_facts_after_first_prediction',_pack((b'x'*4097,None,b'\xff',None,b'')),field_bound,
        prediction='domain_field_bound',agrees=True,family='scan_frontier')
    mutations=('outer_motion','outer_exposure','fake_site','wrong_profile','bool_stage','integer_consistent',
        'missing_result','inner_motion','inner_exposure','missing_repeat','repeat_type','error_on_success','backend_number',
        'refusal_with_result','error_overbound','null','list','string','empty')
    for name in mutations:
        doc=copy.deepcopy(py)
        if name=='outer_motion':doc['motion_authority']=True
        if name=='outer_exposure':doc['actual_exposure_wall_s']=0.0
        if name=='fake_site':doc['observed_validation_site']={'field_index':0}
        if name=='wrong_profile':doc['profile']='production'
        if name=='bool_stage':doc['stage']=True
        if name=='integer_consistent':doc['consistent']=1
        if name=='missing_result':doc.pop('result')
        if name=='inner_motion':doc['result']['review']['motion_authority']=True
        if name=='inner_exposure':doc['result']['review']['actual_exposure_wall_s']=0
        if name=='missing_repeat':doc['result'].pop('repeated_review')
        if name=='repeat_type':doc['result']['repeated_review']=[]
        if name=='error_on_success':doc['error']='fake'
        if name=='backend_number':doc['result']['review']['backend_claim']=1
        if name=='refusal_with_result':doc.update(stage='review_refused',consistent=False,error='fake')
        if name=='error_overbound':doc=copy.deepcopy(bad_doc);doc['error']='e'*(e.OBSERVATION_ERROR_MAX_CHARS+1)
        if name=='null':doc=None
        if name=='list':doc=[]
        if name=='string':doc='claim'
        if name=='empty':doc={}
        add('invalid-snapshot--'+name,raw,doc,stage='evidence_refused',error='evidence_observation_shape',family='invalid_snapshot')
    assert len({f['case'] for f in rows})==len(rows)
    return tuple(rows)


def evaluate(f):
    outputs={};repeated={}
    for language,snapshot in f['observations'].items():
        outputs[language]=e.inspect_evidence(f['raw'],snapshot)
        repeated[language]=e.inspect_evidence(f['raw'],snapshot)
    return outputs,repeated


def check(f,outputs,repeated):
    for language,doc in outputs.items():
        assert canonical(doc)==canonical(repeated[language])
        assert doc['evidence_stage']==f['expected_stage'] and doc['error']==f['expected_error'],(f['case'],language,doc)
        assert doc['observed_validation_site'] is None and doc['actual_exposure_wall_s'] is None
        assert all(doc[k] is False for k in (*e.FALSE_FIELDS,*e.EXTRA_FALSE))
        if doc['evidence_stage']=='evidence_returned':
            assert canonical(doc['observation_snapshot'])==canonical(f['observations'][language])
            prediction=doc['predicted_first_structural_failure']
            assert (None if prediction is None else prediction['reason'])==f['expected_prediction_reason']
            assert doc['reported_reason_agrees_with_prediction'] is f['expected_agreement']
            assert doc['input_sha256']==hashlib.sha256(f['raw']).hexdigest()
        else:
            assert all(doc[k] is None for k in ('input_sha256','observation_sha256','observation_snapshot','byte_facts','predicted_first_structural_failure','reported_reason_agrees_with_prediction'))
    return True


def run(root,output,pins_path,pins_digest):
    root=Path(root);output=Path(output);pinraw=Path(pins_path).read_bytes()
    if hashlib.sha256(pinraw).hexdigest()!=pins_digest:raise ValueError('input_pin_hash')
    pins=json.loads(pinraw)
    plan_name=ARCHIVE+'/PLAN.json'
    if type(pins) is not dict or pins.get(plan_name)!=sha(root/plan_name):raise ValueError('archive_plan_pin')
    if set(pins)!=set(input_files(root)) or any(sha(root/n)!=h for n,h in pins.items()):raise ValueError('input_pin_mismatch')
    output.mkdir(exist_ok=False);rows=[]
    for f in corpus(root):
        folder=output/f['case'];folder.mkdir();(folder/'input.bin').write_bytes(f['raw'])
        meta={k:v for k,v in f.items() if k not in ('raw','observations')};write(folder/'case.json',meta);write(folder/'observation.json',f['observations'])
        outputs,repeated=evaluate(f);write(folder/'evidence.json',outputs);write(folder/'evidence-repeat.json',repeated)
        ok=check(f,outputs,repeated)
        row=dict(case=f['case'],family=f['family'],expectations_match=ok,evidence_sha256={p.name:sha(p) for p in sorted(folder.iterdir())})
        write(folder/'result.json',row);rows.append(row)
    if any(sha(root/n)!=h for n,h in pins.items()):raise ValueError('input_changed')
    summary=dict(profile=e.PROFILE,case_count=len(rows),cases=rows,input_sha256=pins,
        families={s:sum(r['family']==s for r in rows) for s in sorted({r['family'] for r in rows})},
        expectations_match=all(r['expectations_match'] for r in rows),formal_runs=1,formal_retries=0,
        pure_diagnostic_calls=4*len(rows),old_protocol_validator_calls=0,formal_compilations=0,portable_cpp_executions=0,new_cpp_implementations=0,
        new_runtime_backend_experiments=0,old_validators_fixed=False,execution_site_observed=False,universal_equivalence_proved=False,
        production_changes=False,native_transport=False,movement=False)
    write(output/'summary.json',summary);return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--input-pins',type=Path,required=True);parser.add_argument('--input-pins-sha256',required=True);args=parser.parse_args()
    result=run(Path(__file__).resolve().parents[1],args.output,args.input_pins,args.input_pins_sha256)
    print(json.dumps({k:v for k,v in result.items() if k not in ('cases','input_sha256')}))
