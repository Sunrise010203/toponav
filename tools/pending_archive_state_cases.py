"""Closed 2AA decisions folded only in a bounded offline invocation-local model."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
from tools import pending_archive_state as model

PREVIOUS = 'reports/straight-followup-20261103-36'
SOURCE_FILES = ('tools/pending_archive_state.py','tools/pending_archive_state_cases.py',
    'docs/PENDING_ARCHIVE_STATE_V1.json','docs/PENDING_ARCHIVE_STATE_V1.md','tests/test_pending_archive_state.py')
sha = lambda b: hashlib.sha256(b).hexdigest()
encode = lambda v: json.dumps(v,sort_keys=True,separators=(',', ':'),allow_nan=False).encode()


def input_files(root):
    root=Path(root);s=json.loads((root/PREVIOUS/'decision-audit/summary.json').read_bytes());assert s['case_count']==1435
    refs=[*SOURCE_FILES,'tools/pending_archive_decision.py',*(PREVIOUS+'/'+n for n in ('DELIVERY_MANIFEST.json','SUMMARY.json',
        'DECISION_ANALYSIS.json','CLOSED_READER_DECISIONS.json','TOP_LEVEL_REFUSALS.json','LIMITATIONS.json','decision-audit/summary.json'))]
    for row in s['cases']:
        name=row['case'];assert name and all(c.isascii() and (c.isalnum() or c in '-_+.') for c in name)
        refs.extend(f'{PREVIOUS}/decision-audit/{name}/{part}' for part in ('case.json','decision.json','decision-repeat.json'))
    return tuple(dict.fromkeys(refs))


def trace(events,scope='caller-scope'):
    return encode(dict(profile=model.PROFILE,scope=scope,events=[dict(slot=slot,decision_utf8=text) for slot,text in events]))


def corpus(root):
    root=Path(root);audit=root/PREVIOUS/'decision-audit';s=json.loads((audit/'summary.json').read_bytes());rows=[]
    for row in s['cases']:
        folder=audit/row['case'];a=(folder/'decision.json').read_text();b=(folder/'decision-repeat.json').read_text()
        phase='local_recorded' if row['stage']=='offline_reference_consistent' else 'local_refused'
        final='local_conflict' if phase=='local_recorded' and a!=b else phase
        rows.append(dict(case='closed--'+row['case'],family='sealed_2AA',source_case=row['case'],
            trace_raw=trace([('slot',a),('slot',a),('slot',b),('slot',a)]),expected_error=None,
            expected_phases=[final],expected_history=[phase,phase,final,final]))
    base=(audit/'closed--fixture--regular/decision.json').read_text();refused=(audit/'closed--fixture--bad_parent_pin/decision.json').read_text()
    unknown=json.loads(base);unknown['stage']='unknown_outcome';unknown['error']=None;unknown=encode(unknown).decode()
    repinned=(audit/'limitation--coherent-forgery-and-repin/decision.json').read_text()
    def add(name,events=None,phases=None,history=None,raw=None,error=None,family='model_boundary',scope='caller-scope'):
        rows.append(dict(case=name,family=family,source_case=None,trace_raw=trace(events,scope) if raw is None else raw,
            expected_error=error,expected_phases=[] if phases is None else phases,expected_history=[] if history is None else history))
    def sequence(name,texts,history,family='model_boundary',scope='caller-scope'):
        add(name,[('slot',t) for t in texts],[history[-1]],history,family=family,scope=scope)
    sequence('sequence--exact-repeat',[base]*3,['local_recorded']*3)
    sequence('sequence--whitespace-is-different',[base,base+' ',base],['local_recorded','local_conflict','local_conflict'])
    sequence('sequence--refused-terminal',[refused,base,refused],['local_refused']*3)
    sequence('sequence--null-unknown-terminal',[None,base,None],['local_unknown']*3)
    sequence('sequence--stage-unknown-terminal',[unknown,base,refused],['local_unknown']*3)
    sequence('sequence--positive-then-refusal',[base,refused,base],['local_recorded','local_conflict','local_conflict'])
    sequence('sequence--positive-then-unknown',[base,None,base],['local_recorded','local_conflict','local_conflict'])
    sequence('sequence--repin-same-slot',[base,repinned,repinned],['local_recorded','local_conflict','local_conflict'])
    add('limitation--alias-new-slot',[('a',base),('b',base)],['local_recorded','local_recorded'],['local_recorded']*2,family='explicit_limit')
    sequence('limitation--fresh-invocation-rebuild',[base,base],['local_recorded']*2,family='explicit_limit')
    sequence('limitation--renamed-scope',[base],['local_recorded'],family='explicit_limit',scope='other-caller-scope')
    sequence('limitation--coherent-forgery',[repinned],['local_recorded'],family='explicit_limit')
    for label in ('fabricated-metadata','cross-call-consistent-splice','same-record-replay'):
        text=(audit/('limitation--'+label)/'decision.json').read_text()
        sequence('limitation--'+label,[text],['local_recorded'],family='explicit_limit')
    def changed(name,changes,phase='local_refused'):
        value=json.loads(base)
        for k,v in changes.items():value[k]=v
        sequence(name,[encode(value).decode()],[phase])
    for k in (*model.FALSE_FIELDS,*model.decision.EXTRA_FALSE):
        for v in (True,0):changed('decision-flag--'+k+'--'+str(v),{k:v})
    for k,v in [('error','synthetic'),('offline_reference_review_ready',1),('diagnostic_only',False),
                ('stateful_consumption',True),('decision_basis','authenticated'),('reader_stage','unknown'),
                ('reader_error','old_error'),('reader_local_reads_complete',1),('observed_validation_site','invented'),
                ('actual_exposure_wall_s',1),('input_record_sha256','not-a-digest'),('profile','other')]:
        changed('decision-field--'+k,{k:v})
    changed('decision-field--extra',dict(runtime_permit=True))
    value=json.loads(base);del value['error'];sequence('decision-field--missing',[encode(value).decode()],['local_refused'])
    changed('decision-claim--missing',dict(expected_claim_snapshot={}))
    value=json.loads(base);value['sampled_artifact_sha256']['parent']='0'*64
    sequence('decision-claim--pin-mismatch',[encode(value).decode()],['local_refused'])
    changed('decision-claim--hash-shape',dict(sampled_artifact_sha256={}))
    for label,text in [('empty',''),('list','[]'),('null','null'),('duplicate','{"profile":"x","profile":"y"}'),
        ('float','{"profile":1.0}'),('nan','{"x":NaN}'),('bom','\ufeff{}'),('deep','['*1500+']'*1500)]:
        sequence('decision-json--'+label,[text],['local_refused'])
    changed('decision-unknown--null-stage',dict(stage=None),phase='local_unknown')
    changed('decision-unknown--unexpected-stage',dict(stage='future_stage'),phase='local_unknown')
    add('bound--empty-events',[],[],[])
    sequence('bound--events-exact',[base]*256,['local_recorded']*256)
    add('bound--events-over',events=[('slot',None)]*257,error='state_event_bound')
    add('bound--slots-exact',[(str(i),None) for i in range(128)],['local_unknown']*128,['local_unknown']*128)
    add('bound--slots-over',[(str(i),None) for i in range(129)],error='state_slot_bound')
    sequence('bound--decision-exact',[base+' '*(model.DECISION_LIMIT-len(base.encode()))],['local_recorded'])
    add('bound--decision-over',[('slot','x'*(model.DECISION_LIMIT+1))],error='state_decision_byte_bound')
    add('bound--decision-unicode-over',[('slot','界'*(model.DECISION_LIMIT//3+1))],error='state_decision_byte_bound')
    good=trace([('slot',base)])
    add('bound--trace-exact',raw=good+b' '*(model.TRACE_LIMIT-len(good)),phases=['local_recorded'],history=['local_recorded'])
    add('bound--trace-over',raw=b' '*(model.TRACE_LIMIT+1),error='state_trace_byte_bound')
    for label,v in [('null',None),('bool',True),('slash','a/b'),('dot','.'),('parent','..'),('empty',''),('unicode','调用者'),('over','x'*65)]:
        add('shape--scope--'+label,raw=encode(dict(profile=model.PROFILE,scope=v,events=[])),error='state_scope_shape')
    for label,v in [('null',None),('bool',True),('slash','a/b'),('dot','.'),('empty',''),('unicode','槽'),('over','x'*129)]:
        add('shape--slot--'+label,raw=encode(dict(profile=model.PROFILE,scope='scope',events=[dict(slot=v,decision_utf8=base)])),error='state_event_shape')
    for label,v in [('list',[]),('number',1),('bool',True)]:
        add('shape--decision-text--'+label,[('slot',v)],error='state_decision_text')
    add('shape--decision-surrogate',[('slot','\ud800')],error='state_decision_utf8')
    for label,raw,err in [('root-list',b'[]','state_trace_shape'),('root-null',b'null','state_trace_shape'),
        ('json-empty',b'','state_trace_json'),('json-utf8',b'\xff','state_trace_json'),('json-bom',b'\xef\xbb\xbf{}','state_trace_json'),
        ('json-duplicate',b'{"events":[],"events":[]}','state_trace_json'),('json-float',b'{"x":1e999}','state_trace_json'),
        ('json-deep',b'['*1500+b']'*1500,'state_trace_json')]:add('shape--'+label,raw=raw,error=err)
    for label,change,err in [('profile',dict(profile='other'),'state_trace_shape'),('extra',dict(owner=True),'state_trace_shape'),
        ('events',dict(events=None),'state_event_bound')]:
        doc=json.loads(good);doc.update(change);add('shape--'+label,raw=encode(doc),error=err)
    for label,suffix,err in [('bad-slot',dict(slot='../bad',decision_utf8=base),'state_event_shape'),
        ('extra-key',dict(slot='new',decision_utf8=base,permit=True),'state_event_shape'),
        ('bad-text',dict(slot='slot',decision_utf8=True),'state_decision_text')]:
        doc=json.loads(good);doc['events'].append(suffix);add('suffix--'+label,raw=encode(doc),error=err)
    assert len({r['case'] for r in rows})==len(rows)
    return rows


def check(row,a,b):
    assert a==b,row['case']
    assert a['error']==row['expected_error'],row['case']
    assert a['stage']==('state_trace_reviewed' if row['expected_error'] is None else 'state_trace_refused')
    assert [s['phase'] for s in a['slots']]==row['expected_phases'],row['case']
    assert [e['after_phase'] for e in a['events']]==row['expected_history'],row['case']
    assert all(a[k] is False for k in (*model.FALSE_FIELDS,*model.EXTRA_FALSE))
    assert a['observed_validation_site'] is a['actual_exposure_wall_s'] is None
    assert a['model_only'] is True and a['fresh_invocation_starts_empty'] is True
    if a['error'] is not None:assert a['slots']==a['events']==[] and not a['local_model_review_complete']
    return True


def write(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')


def run(root,output,pinpath,pinhash):
    root,output=Path(root),Path(output);pinraw=Path(pinpath).read_bytes();pins=json.loads(pinraw)
    if sha(pinraw)!=pinhash:raise ValueError('pin_digest')
    if set(pins)!=set(input_files(root)) or any(sha((root/n).read_bytes())!=h for n,h in pins.items()):raise ValueError('input_pin_mismatch')
    output.mkdir(exist_ok=False);rows=[]
    for row in corpus(root):
        folder=output/row['case'];folder.mkdir();raw=row['trace_raw'];a=model.review_state_trace(raw);b=model.review_state_trace(raw);check(row,a,b)
        write(folder/'case.json',{k:v for k,v in row.items() if k!='trace_raw'})
        with (folder/'input.bin').open('xb') as f:f.write(raw)
        write(folder/'state.json',a);write(folder/'state-repeat.json',b)
        rows.append(dict(case=row['case'],family=row['family'],stage=a['stage'],error=a['error'],input_sha256=sha(raw),
            output_sha256=sha((folder/'state.json').read_bytes())))
    assert all(sha((root/n).read_bytes())==h for n,h in pins.items())
    summary=dict(case_count=len(rows),model_calls=2*len(rows),cases=rows,formal_runs=1,formal_retries=0,
        old_2AA_review_calls=0,sealed_2AA_constants_and_claim_shape_helper_reused=True,old_2Z_reader_calls=0,old_2Y_projection_calls=0,
        old_2W_binding_calls=0,old_2X_pair_reader_calls=0,old_2V_diagnostic_calls=0,old_protocol_validator_calls=0,
        formal_compilations=0,portable_cpp_executions=0,new_runtime_backend_experiments=0,production_changes=False,native_transport=False,movement=False)
    write(output/'summary.json',summary);return {k:v for k,v in summary.items() if k!='cases'}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--input-pins',type=Path,required=True);p.add_argument('--input-pins-sha256',required=True);a=p.parse_args()
    print(json.dumps(run(Path(__file__).resolve().parents[1],a.output,a.input_pins,a.input_pins_sha256)))
