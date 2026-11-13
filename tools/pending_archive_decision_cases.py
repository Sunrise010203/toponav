"""Closed 2Z records only. No old reader/contract/protocol or filesystem fixture."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
from tools import pending_archive_decision as decision

PREVIOUS = 'reports/straight-followup-20261103-35'
SOURCE_FILES = ('tools/pending_archive_decision.py', 'tools/pending_archive_decision_cases.py',
    'docs/PENDING_ARCHIVE_DECISION_V1.json', 'docs/PENDING_ARCHIVE_DECISION_V1.md', 'tests/test_pending_archive_decision.py')
sha = lambda raw: hashlib.sha256(raw).hexdigest()
encode = lambda v: json.dumps(v, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def input_files(root):
    root = Path(root); summary = json.loads((root / PREVIOUS / 'projection-reader-audit/summary.json').read_bytes())
    assert summary['case_count'] == 1123
    refs = [*SOURCE_FILES, *(PREVIOUS+'/'+n for n in ('DELIVERY_MANIFEST.json', 'SUMMARY.json', 'READER_ANALYSIS.json',
        'REFERENCE_MATRIX.json', 'SAME_LABEL_REFERENCES.json', 'FILESYSTEM_CASES.json', 'projection-reader-audit/summary.json'))]
    for row in summary['cases']:
        name = row['case'];assert name and all(c.isascii() and (c.isalnum() or c in '-_+.') for c in name)
        refs.extend(f'{PREVIOUS}/projection-reader-audit/{name}/{part}' for part in ('case.json','read.json','read-repeat.json'))
    return tuple(dict.fromkeys(refs))


def corpus(root):
    root = Path(root); audit = root / PREVIOUS / 'projection-reader-audit'
    summary = json.loads((audit / 'summary.json').read_bytes());rows = []
    for row in summary['cases']:
        folder = audit / row['case'];a=(folder/'read.json').read_bytes();b=(folder/'read-repeat.json').read_bytes()
        rows.append(dict(case='closed--'+row['case'],family='sealed_2Z',source_case=row['case'],
            record_raw=a,repeat_raw=b,expected_claim=json.loads(a)['claim'],
            expected_error=None if row['stage']=='projection_reader_checked' else 'reader_top_level_not_checked'))
    base = json.loads((audit/'fixture--regular/read.json').read_bytes());baseclaim=base['claim']
    def add(name, rec=base, claim=baseclaim, error=None, family='structural_mutation', repeat=None):
        raw=rec if type(rec) is bytes else encode(rec)
        rows.append(dict(case=name,family=family,source_case=None,record_raw=raw,
            repeat_raw=raw if repeat is None else repeat,expected_claim=copy.deepcopy(claim),expected_error=error))
    def change(name,path,value,error,*,delete=False):
        rec=copy.deepcopy(base);target=rec
        for key in path[:-1]:target=target[key]
        if delete:del target[path[-1]]
        else:target[path[-1]]=value
        add(name,rec,error=error)
    for name,v in [('refused','projection_reader_refused'),('reference_refused','projection_reader_reference_refused'),
                   ('unknown','unknown_outcome'),('null',None),('bool',True)]:
        change('top-priority--'+name,['result','stage'],v,'reader_top_level_not_checked')
    change('top-priority--missing',['result','stage'],None,'reader_top_level_not_checked',delete=True)
    for k,v in [('error','archive_close'),('filesystem_error_role','cleanup'),('os_errno',5),
                ('local_regular_file_reads_complete',False),('local_regular_file_reads_complete',1),('sequential_samples_only',1),
                ('observed_validation_site','invented'),('actual_exposure_wall_s',1)]:
        change('reader-state--'+k+'--'+str(v),['result',k],v,'decision_reader_state')
    change('reader-shape--profile',['result','profile'],'other','decision_reader_shape')
    change('reader-shape--extra',['result','runtime_permit'],True,'decision_reader_shape')
    change('reader-shape--missing',['result','error'],None,'decision_reader_shape',delete=True)
    for layer, path, flags, error in [('reader',['result'],(*decision.FALSE_FIELDS,*decision.READER_FALSE),'decision_reader_state'),
        ('projection',['result','projection_result'],(*decision.FALSE_FIELDS,*decision.PROJECTION_FALSE),'decision_projection_state'),
        ('binding',['result','projection_result','reference_result'],decision.FALSE_FIELDS,'decision_binding_state')]:
        for flag in flags:
            for value in (True,0):change(f'flags--{layer}--{flag}--{value}',path+[flag],value,error)
        change('flags--'+layer+'--missing',path+[flags[0]],None,'decision_'+('reader' if layer=='reader' else layer)+'_shape',delete=True)
    for layer,path,fields in [('projection',['result','projection_result'],('parent_digest_matches','projection_digest_matches',
            'raw_parent_projection_digest_matches','observation_parent_projection_digest_matches','projection_members_agree_with_parent')),
        ('binding',['result','projection_result','reference_result'],('manifest_digest_matches','raw_reference_digest_matches',
            'observation_reference_digest_matches','file_references_agree','pairing_declared'))]:
        for field in fields:
            for value in (None,False,1):change(f'true--{layer}--{field}--{value}',path+[field],value,'decision_'+layer+'_state')
        for k,v in [('stage','refused'),('error','old_exact_error'),('profile','other'),('observed_validation_site','invented'),
                    ('actual_exposure_wall_s',1),('pairing_basis','authenticated'),('classification','authority')]:
            change(f'nested--{layer}--{k}',path+[k],v,'decision_'+layer+'_state')
        change('nested--'+layer+'--null',path,None,'decision_'+layer+'_shape')
        change('nested--'+layer+'--extra',path+['runtime_permit'],True,'decision_'+layer+'_shape')
    for i,role in enumerate(decision.ROLES):
        path=['result','files',i];f=base['result']['files'][i]
        for name,key,v in [('negative','byte_count',-1),('bool_count','byte_count',True),('oversize','byte_count',decision.CAPS[i]+1),
            ('role','role','wrong'),('ref','reference','other/path'),('sha','sha256','bad'),('source','source','trusted'),
            ('final_null','final_recheck_agreed',None),('final_false','final_recheck_agreed',False),('final_one','final_recheck_agreed',1),
            ('short_signature','before_signature',[]),('after_changed','after_signature',[*f['after_signature'][:-1],9])]:
            change('fact--'+role+'--'+name,path+[key],v,'decision_file_facts')
        sig=list(f['before_signature']);sig[0]=True
        rec=copy.deepcopy(base);rec['result']['files'][i]['before_signature']=sig;rec['result']['files'][i]['after_signature']=sig
        add('fact--'+role+'--bool_signature',rec,error='decision_file_facts')
        sig=list(f['before_signature']);sig[2]=0o040755
        rec=copy.deepcopy(base);rec['result']['files'][i]['before_signature']=sig;rec['result']['files'][i]['after_signature']=sig
        add('fact--'+role+'--directory_signature',rec,error='decision_file_facts')
    change('fact--missing-file',['result','files'],base['result']['files'][:3],'decision_file_facts')
    change('fact--extra-key',['result','files',0,'lease'],True,'decision_file_facts')
    change('fact--pin-mismatch',['result','files',0,'sha256'],'0'*64,'decision_pin_mismatch')
    change('projection--hash-mismatch',['result','projection_result','artifact_sha256','raw'],'0'*64,'decision_projection_state')
    change('projection--bool-count',['result','projection_result','artifact_byte_counts','raw'],True,'decision_projection_state')
    change('binding--hash-mismatch',['result','projection_result','reference_result','artifact_sha256','raw'],'0'*64,'decision_binding_state')
    change('binding--bool-count',['result','projection_result','reference_result','artifact_byte_counts','raw'],True,'decision_binding_state')
    change('claim--record-mismatch',['claim','raw_ref'],'data/other','decision_claim_mismatch')
    change('claim--reader-mismatch',['result','claim_snapshot','raw_ref'],'data/other','decision_claim_mismatch')
    change('claim--projection-mismatch',['result','projection_result','claim_snapshot','raw_ref'],'data/other','decision_projection_state')
    change('claim--binding-mismatch',['result','projection_result','reference_result','claim_snapshot','raw_ref'],'data/other','decision_binding_state')
    for idx,(key,value) in enumerate([('profile','bad'),('pairing_basis','trusted'),('raw_ref','../a'),('raw_ref','a/'*32+'x'),
                      ('raw_ref','x'*513),('projection_sha256',True),('parent_manifest_sha256','F'*64)]):
        add('expected-claim--'+key+'--'+str(idx),claim=dict(baseclaim,**{key:value}),error='decision_expected_claim_shape')
    add('expected-claim--missing',claim={},error='decision_expected_claim_shape')
    add('expected-claim--wrong-selection',claim=dict(baseclaim,raw_ref='data/other'),error='decision_claim_mismatch')
    change('metadata--events',['events'],None,'decision_record_metadata')
    change('metadata--events-over',['events'],[{}]*65,'decision_record_metadata')
    change('metadata--initial',['initial_sha256'],{},'decision_record_metadata')
    add('json--list',b'[]',error='decision_record_shape');add('json--null',b'null',error='decision_record_shape')
    add('json--empty',b'',error='decision_record_json');add('json--utf8',b'\xff',error='decision_record_json')
    add('json--bom',b'\xef\xbb\xbf{}',error='decision_record_json')
    add('json--duplicate',b'{"result":null,"result":{}}',error='decision_record_json')
    add('json--nested-duplicate',b'{"result":{"x":1,"x":2}}',error='decision_record_json')
    for v in (b'NaN',b'Infinity',b'1e999',b'1.0'):add('json--number--'+v.decode(),b'{"x":'+v+b'}',error='decision_record_json')
    add('json--deep',b'['*1500+b']'*1500,error='decision_record_json')
    add('json--record-cap',encode(base)+b' '*(decision.RECORD_LIMIT-len(encode(base))))
    add('json--record-over',b' '*(decision.RECORD_LIMIT+1),error='decision_record_byte_bound')
    change('record--missing',['events'],None,'decision_record_shape',delete=True)
    change('record--extra',['authority'],True,'decision_record_shape')
    change('record--reader-null',['result'],None,'decision_reader_shape')
    other=json.loads((audit/'fixture--joint_repin/read.json').read_bytes())
    change('splice--partial-projection',['result','projection_result'],other['result']['projection_result'],'decision_projection_state')
    change('splice--partial-fact',['result','files',2],other['result']['files'][2],'decision_projection_state')
    repeat=json.loads((audit/'fixture--regular/read-repeat.json').read_bytes())
    mixed=copy.deepcopy(base);mixed['result']['files'][2:]=repeat['result']['files'][2:]
    add('limitation--cross-call-consistent-splice',mixed,family='explicit_limit')
    fake=copy.deepcopy(base)
    for f in fake['result']['files']:
        sig=list(f['before_signature']);sig[1]+=123456;sig[4]+=987654;sig[5]+=987654
        f['before_signature']=sig;f['after_signature']=list(sig)
    add('limitation--fabricated-metadata',fake,family='explicit_limit')
    fake=copy.deepcopy(base);newhashes=dict(zip(decision.ARTIFACT_ROLES,['a'*64,'b'*64,'c'*64,'d'*64]));c=fake['claim']
    c['parent_manifest_sha256']=newhashes['parent'];c['projection_sha256']=newhashes['projection']
    r=fake['result'];p=r['projection_result'];b=p['reference_result'];r['claim_snapshot']=dict(c);p['claim_snapshot']=dict(c)
    for f,h in zip(r['files'],newhashes.values()):f['sha256']=h
    fake['initial_sha256']={role:h for role,h in zip(decision.ROLES,newhashes.values())}
    p['artifact_sha256']=newhashes;b['artifact_sha256']=dict(manifest=newhashes['projection'],raw=newhashes['raw'],observation=newhashes['observation'])
    b['claim_snapshot']['manifest_sha256']=newhashes['projection']
    add('limitation--coherent-forgery-and-repin',fake,claim=c,family='explicit_limit')
    add('limitation--same-record-replay',family='explicit_limit')
    assert len({r['case'] for r in rows})==len(rows)
    return rows


def evaluate(row, repeat=False):
    return decision.review_reader_decision(row['repeat_raw'] if repeat else row['record_raw'],row['expected_claim'])


def check(row,a,b):
    assert a['error']==b['error']==row['expected_error'],row['case']
    assert a['stage']==b['stage']==('offline_reference_consistent' if row['expected_error'] is None else 'decision_refused')
    assert {k:v for k,v in a.items() if k!='input_record_sha256'}=={k:v for k,v in b.items() if k!='input_record_sha256'},row['case']
    assert all(a[k] is False and b[k] is False for k in (*decision.FALSE_FIELDS,*decision.EXTRA_FALSE))
    assert a['observed_validation_site'] is a['actual_exposure_wall_s'] is None
    assert a['diagnostic_only'] is True and a['stateful_consumption'] is False
    assert a['offline_reference_review_ready'] is (row['expected_error'] is None)
    return True


def write(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')


def run(root,output,pinpath,pinhash):
    root,output=Path(root),Path(output);pinraw=Path(pinpath).read_bytes();pins=json.loads(pinraw)
    if sha(pinraw)!=pinhash:raise ValueError('pin_digest')
    if set(pins)!=set(input_files(root)) or any(sha((root/n).read_bytes())!=h for n,h in pins.items()):raise ValueError('input_pin_mismatch')
    output.mkdir(exist_ok=False);cases=[]
    for row in corpus(root):
        folder=output/row['case'];folder.mkdir();a=evaluate(row);b=evaluate(row,True);check(row,a,b)
        write(folder/'case.json',{k:v for k,v in row.items() if k not in ('record_raw','repeat_raw','expected_claim')})
        for part,key in [('input.bin','record_raw'),('input-repeat.bin','repeat_raw')]:
            with (folder/part).open('xb') as f:f.write(row[key])
        write(folder/'expected-claim.json',row['expected_claim']);write(folder/'decision.json',a);write(folder/'decision-repeat.json',b)
        cases.append(dict(case=row['case'],family=row['family'],error=a['error'],stage=a['stage'],input_sha256=sha(row['record_raw']),
            repeat_input_sha256=sha(row['repeat_raw']),output_sha256=sha((folder/'decision.json').read_bytes())))
    assert all(sha((root/n).read_bytes())==h for n,h in pins.items())
    summary=dict(case_count=len(cases),decision_calls=2*len(cases),cases=cases,formal_runs=1,formal_retries=0,
        old_2Z_reader_calls=0,old_2Y_projection_calls=0,old_2W_binding_calls=0,old_2X_pair_reader_calls=0,
        old_2V_diagnostic_calls=0,old_protocol_validator_calls=0,formal_compilations=0,portable_cpp_executions=0,
        new_runtime_backend_experiments=0,production_changes=False,native_transport=False,movement=False)
    write(output/'summary.json',summary);return {k:v for k,v in summary.items() if k!='cases'}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--input-pins',type=Path,required=True);p.add_argument('--input-pins-sha256',required=True);a=p.parse_args()
    print(json.dumps(run(Path(__file__).resolve().parents[1],a.output,a.input_pins,a.input_pins_sha256)))
