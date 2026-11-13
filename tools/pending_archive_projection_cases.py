"""Closed large-parent/minimal-projection byte corpus, no filesystem adapter."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
from tools import pending_archive_projection as p

ARCHIVE='reports/straight-followup-20261103-30'
PREVIOUS='reports/straight-followup-20261103-33'
PARENT='reports/straight-followup-20261103-32/DELIVERY_MANIFEST.json'
SOURCE_FILES=('tools/pending_archive_projection.py','tools/pending_archive_projection_cases.py',
    'docs/PENDING_ARCHIVE_PROJECTION_V1.json','docs/PENDING_ARCHIVE_PROJECTION_V1.md','tests/test_pending_archive_projection.py')
PRESERVED=(PREVIOUS+'/SUMMARY.json',PREVIOUS+'/READER_ANALYSIS.json',PREVIOUS+'/FILESYSTEM_CASES.json',
    PREVIOUS+'/archive-reader-audit/archive-catalog--sealed_2W_over_entry_cap/read.json',
    PREVIOUS+'/development.xml',PREVIOUS+'/development.exit.json',
    'reports/straight-followup-20261103-31/SAME_LABEL_EVIDENCE.json',
    'reports/straight-followup-20261103-28/SUMMARY.json','docs/PRECOMPUTE_INTEGRATION_REVIEW.json',
    *(f'reports/straight-followup-20261103-28/cpp-differential-audit/preserve--{n}/python.json' for n in
      ('unknown_result_native','late_reply_retains_unknown','edge--late_unupgraded_future_observation','edge--late_unupgraded_wrong_original_record')),
    *(f'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample/{n}.json' for n in ('input','result')))
sha=lambda blob:hashlib.sha256(blob).hexdigest()
encode=lambda value:json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()


def input_files(root):
    names=json.loads((Path(root)/ARCHIVE/'PLAN.json').read_bytes())['cases'];assert len(names)==532
    assert all(n and all(c.isascii() and (c.isalnum() or c in '-_+.') for c in n) and n not in ('.','..') for n in names)
    return tuple(dict.fromkeys((*SOURCE_FILES,'tools/pending_archive_binding.py',PARENT,PREVIOUS+'/DELIVERY_MANIFEST.json',
        ARCHIVE+'/PLAN.json',*PRESERVED,*(f'{ARCHIVE}/boundary-audit/{name}/{part}' for name in names
        for part in ('input.bin','python.json','cpp.stdout.bin')))))


def minimal_catalog(files,raw_ref,obs_ref):
    return encode(dict(scope=p.PROJECTION_SCOPE,files={raw_ref:files[raw_ref],obs_ref:files[obs_ref]}))


def claim_for(parent,projection,raw_ref,obs_ref,parent_ref=PARENT):
    return dict(profile=p.PROFILE,parent_manifest_ref=parent_ref,parent_manifest_sha256=sha(parent),
        projection_manifest_ref='derived/minimal-catalog.json',projection_sha256=sha(projection),
        raw_ref=raw_ref,observation_ref=obs_ref,pairing_basis='caller_declared_only')


def corpus(root):
    root=Path(root);parent=(root/PARENT).read_bytes();catalog=json.loads(parent)['files']
    names=json.loads((root/ARCHIVE/'PLAN.json').read_bytes())['cases'];rows=[]
    def add(name,par,proj,claim,raw,obs,*,family='contract_boundary',error=None,nested_error=None,source_case=None,origin=None):
        rows.append(dict(case=name,family=family,parent_raw=par,projection_raw=proj,claim=claim,raw=raw,observation_raw=obs,
            expected_error=error,expected_nested_error=nested_error,source_case=source_case,origin=origin))
    for name in names:
        rawref=f'{ARCHIVE}/boundary-audit/{name}/input.bin';raw=(root/rawref).read_bytes()
        for origin,part in (('python','python.json'),('cpp','cpp.stdout.bin')):
            obsref=f'{ARCHIVE}/boundary-audit/{name}/{part}';proj=minimal_catalog(catalog,rawref,obsref)
            add('archive--'+name+'--'+origin,parent,proj,claim_for(parent,proj,rawref,obsref),raw,(root/obsref).read_bytes(),
                family='sealed_large_parent',source_case=name,origin=origin)
    base=rows[0];small_ref=PREVIOUS+'/DELIVERY_MANIFEST.json';small=(root/small_ref).read_bytes()
    add('claim--sealed_smaller_parent',small,base['projection_raw'],
        claim_for(small,base['projection_raw'],base['claim']['raw_ref'],base['claim']['observation_ref'],small_ref),
        base['raw'],base['observation_raw'],family='caller_selection')
    raw,obs=b'opaque raw fixture',b'opaque non-JSON observation';rawref,obsref='artifacts/input.bin','artifacts/observation.bin'
    doc=dict(scope='caller-owned parent, not trusted provenance',files={rawref:sha(raw),obsref:sha(obs),'unselected/other.bin':'0'*64})
    par=encode(doc);proj=minimal_catalog(doc['files'],rawref,obsref);claim=claim_for(par,proj,rawref,obsref)
    def var(name,*,par2=par,proj2=proj,c2=None,raw2=raw,obs2=obs,error=None,nested_error=None,family='contract_boundary'):
        add(name,par2,proj2,copy.deepcopy(claim if c2 is None else c2),raw2,obs2,error=error,nested_error=nested_error,family=family)
    var('claim--baseline',family='caller_selection');var('claim--repeat',family='caller_selection')
    var('mismatch--parent_pin',c2=dict(claim,parent_manifest_sha256='0'*64),error='parent_digest_mismatch')
    var('mismatch--projection_pin',c2=dict(claim,projection_sha256='0'*64),error='projection_digest_mismatch')
    var('mismatch--raw',raw2=raw+b'!',error='projection_reference_refused',nested_error='raw_digest_mismatch')
    var('mismatch--observation',obs2=obs+b' ',error='projection_reference_refused',nested_error='observation_digest_mismatch')
    for role,ref in (('raw',rawref),('observation',obsref)):
        changed=copy.deepcopy(doc);del changed['files'][ref];blob=encode(changed)
        var('missing--'+role,par2=blob,c2=dict(claim,parent_manifest_sha256=sha(blob)),error=role+'_parent_reference_missing')
        changed=json.loads(proj);changed['files'][ref]='f'*64;blob=encode(changed)
        var('mismatch--'+role+'_parent_projection',proj2=blob,c2=dict(claim,projection_sha256=sha(blob)),error=role+'_parent_projection_digest_mismatch')
    pretty=json.dumps(doc,indent=2).encode()
    var('encoding--parent_without_repin',par2=pretty,error='parent_digest_mismatch')
    var('encoding--parent_with_repin',par2=pretty,c2=dict(claim,parent_manifest_sha256=sha(pretty)),family='caller_selection')
    pretty=json.dumps(json.loads(proj),indent=2).encode()
    var('encoding--projection_without_repin',proj2=pretty,error='projection_digest_mismatch')
    var('encoding--projection_with_repin',proj2=pretty,c2=dict(claim,projection_sha256=sha(pretty)),family='caller_selection')
    var('claim--parent_name_is_declarative',c2=dict(claim,parent_manifest_ref='not-opened/parent.json'),family='caller_selection')
    other=next(r for r in rows if r['source_case']=='priority--0--message_null' and r['origin']=='python')
    unpaired=minimal_catalog(catalog,base['claim']['raw_ref'],other['claim']['observation_ref'])
    var('claim--unrelated_sealed_pair',par2=parent,proj2=unpaired,
        c2=claim_for(parent,unpaired,base['claim']['raw_ref'],other['claim']['observation_ref']),
        raw2=base['raw'],obs2=other['observation_raw'],family='caller_selection')
    for name,r,o in [('joint_replacement',b'unrelated replacement',b'not JSON'),('empty_artifacts',b'',b''),
                     ('artifact_caps',b'x'*p.binding.RAW_LIMIT,b'y'*p.binding.OBSERVATION_LIMIT)]:
        newdoc=dict(scope='replacement',files={rawref:sha(r),obsref:sha(o)});newpar=encode(newdoc);newproj=minimal_catalog(newdoc['files'],rawref,obsref)
        var('claim--'+name,par2=newpar,proj2=newproj,c2=claim_for(newpar,newproj,rawref,obsref),raw2=r,obs2=o,family='caller_selection')
    for name,changes in [('profile',dict(profile='old_profile')),('authority',dict(motion_authority=True)),
                         ('pairing',dict(pairing_basis='authenticated')),('alias',dict(projection_manifest_ref=PARENT)),
                         ('upper_digest',dict(parent_manifest_sha256='F'*64)),('bool_digest',dict(projection_sha256=True))]:
        var('shape--'+name,c2=dict(claim,**changes),error='projection_claim_shape')
    var('shape--not_dict',c2=[],error='projection_claim_shape')
    missing=dict(claim);del missing['pairing_basis'];var('shape--missing_key',c2=missing,error='projection_claim_shape')
    for name,v in [('absolute','/etc/passwd'),('parent','../a'),('empty_component','a//b'),('dot','a/./b'),
                   ('backslash','a\\b'),('unicode','档案'),('nul','a\0b'),('overlong','a'*513)]:
        var('path--'+name,c2=dict(claim,raw_ref=v),error='projection_claim_shape')
    malformed={'list':b'[]','utf8':b'\xff','bom':b'\xef\xbb\xbf{}','duplicate':b'{"scope":"x","scope":"y","files":{}}',
        'duplicate_entry':b'{"scope":"x","files":{"a":"x","a":"y"}}','nonfinite':b'{"scope":NaN,"files":{}}',
        'deep':b'['*1500+b'0'+b']'*1500,'empty':encode(dict(scope='x',files={})),
        'unselected_unsafe_path':encode(dict(scope='x',files={**doc['files'],'../bad':'0'*64})),
        'unselected_bad_digest':encode(dict(scope='x',files={**doc['files'],'unselected/invalid':'z'*64})),
        'scope_bound':encode(dict(scope='x'*4097,files=doc['files']))}
    for role in ('parent','projection'):
        for name,blob in malformed.items():
            if role=='parent':var('catalog--parent_'+name,par2=blob,c2=dict(claim,parent_manifest_sha256=sha(blob)),error='parent_catalog_shape')
            else:var('catalog--projection_'+name,proj2=blob,c2=dict(claim,projection_sha256=sha(blob)),error='projection_catalog_shape')
    for name,change in [('extra_member',dict(scope=p.PROJECTION_SCOPE,files=doc['files'])),
                        ('missing_member',dict(scope=p.PROJECTION_SCOPE,files={rawref:sha(raw)})),
                        ('wrong_scope',dict(scope='trusted',files=json.loads(proj)['files']))]:
        blob=encode(change);var('catalog--projection_'+name,proj2=blob,c2=dict(claim,projection_sha256=sha(blob)),error='projection_catalog_shape')
    large=copy.deepcopy(doc)
    for i in range(p.PARENT_ENTRY_LIMIT-len(large['files'])):large['files'][f'padding/{i}']='0'*64
    blob=encode(large);blob+=b' '*(p.PARENT_LIMIT-len(blob))
    var('bound--inclusive_parent',par2=blob,c2=dict(claim,parent_manifest_sha256=sha(blob)),family='caller_selection')
    large['files']['padding/overflow']='0'*64;blob=encode(large)
    var('catalog--parent_entry_over',par2=blob,c2=dict(claim,parent_manifest_sha256=sha(blob)),error='parent_catalog_shape')
    blob=proj+b' '*(p.PROJECTION_LIMIT-len(proj))
    var('bound--inclusive_projection',proj2=blob,c2=dict(claim,projection_sha256=sha(blob)),family='caller_selection')
    for role,key,cap in [('parent','par2',p.PARENT_LIMIT),('projection','proj2',p.PROJECTION_LIMIT),
                          ('raw','raw2',p.binding.RAW_LIMIT),('observation','obs2',p.binding.OBSERVATION_LIMIT)]:
        var('bound--'+role,**{key:b'x'*(cap+1)},error=role+'_byte_bound')
        value=dict(parent=par,projection=proj,raw=raw,observation=obs)[role]
        var('bytes--'+role,**{key:bytearray(value)},error=role+'_bytes')
    return rows


def evaluate(row):
    args=(row['parent_raw'],row['projection_raw'],row['claim'],row['raw'],row['observation_raw'])
    return p.review_projection(*args),p.review_projection(*args)


def check(row,out,repeat):
    assert out==repeat and out['error']==row['expected_error'],(row['case'],out['error'],row['expected_error'])
    assert out['stage']==('projection_reference_checked' if row['expected_error'] is None else 'projection_refused')
    assert all(out[k] is False for k in (*p.binding.FALSE_FIELDS,*p.EXTRA_FALSE))
    assert out['actual_exposure_wall_s'] is out['observed_validation_site'] is None
    nested=out['reference_result']
    if nested is not None:assert nested['error']==row['expected_nested_error']
    if row['expected_error'] is None:assert out['projection_members_agree_with_parent'] and nested['file_references_agree']
    return True


def write(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')


def run(root,output,pinpath,pinhash):
    root,output=Path(root),Path(output);pinraw=Path(pinpath).read_bytes()
    if sha(pinraw)!=pinhash:raise ValueError('pin_digest')
    pins=json.loads(pinraw)
    if set(pins)!=set(input_files(root)) or any(sha((root/n).read_bytes())!=h for n,h in pins.items()):raise ValueError('input_pin_mismatch')
    output.mkdir(exist_ok=False);blobs=output/'blobs';blobs.mkdir();results=[];calls=0
    for row in corpus(root):
        folder=output/row['case'];folder.mkdir()
        meta={k:v for k,v in row.items() if k not in ('parent_raw','projection_raw','claim','raw','observation_raw')}
        for role,key in [('parent','parent_raw'),('projection','projection_raw'),('raw','raw'),('observation','observation_raw')]:
            blob=row[key];h=sha(blob);meta[role+'_sha256']=h;meta[role+'_type']=type(blob).__name__
            dest=blobs/(h+'.bin')
            if not dest.exists():
                with dest.open('xb') as f:f.write(blob)
        write(folder/'case.json',meta);write(folder/'claim.json',row['claim'])
        out,repeat=evaluate(row);write(folder/'projection.json',out);write(folder/'projection-repeat.json',repeat);check(row,out,repeat)
        calls+=2*(out['reference_result'] is not None)
        results.append(dict(case=row['case'],family=row['family'],error=out['error'],output_sha256=sha((folder/'projection.json').read_bytes())))
    assert all(sha((root/n).read_bytes())==h for n,h in pins.items())
    summary=dict(case_count=len(results),cases=results,projection_calls=2*len(results),sealed_2W_pure_contract_calls=calls,
        checked=sum(r['error'] is None for r in results),refused=sum(r['error'] is not None for r in results),formal_runs=1,formal_retries=0,
        old_2X_reader_calls=0,old_2V_diagnostic_calls=0,old_protocol_validator_calls=0,formal_compilations=0,portable_cpp_executions=0,
        new_runtime_backend_experiments=0,old_contract_relaxed=False,input_binding_authenticated=False,production_changes=False,native_transport=False,movement=False)
    write(output/'summary.json',summary);return {k:v for k,v in summary.items() if k!='cases'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--input-pins',type=Path,required=True);parser.add_argument('--input-pins-sha256',required=True);a=parser.parse_args()
    print(json.dumps(run(Path(__file__).resolve().parents[1],a.output,a.input_pins,a.input_pins_sha256)))
