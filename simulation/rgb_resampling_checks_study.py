"""Frozen native-sampling study; public RGB prediction precedes truth audit."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import re
import subprocess
import sys

from roadvision.corridor_rgb_probe import CALIBRATION_ID
from simulation.rgb_observability_run import ROOT, verify_freeze, generate_suite
from simulation.rgb_motion_checks_study import predict_index as predict_h_index
from simulation.rgb_ground_rig_study import audit_region


def predict_index(index_path, output):
    data=json.loads(Path(index_path).read_text())
    if (not isinstance(data,dict) or set(data)!={'version','sequences'} or
            data['version']!='rgb_observability_public_index_v1' or not isinstance(data['sequences'],list)):
        raise ValueError('Invalid public sequence index')
    seen=set()
    for item in data['sequences']:
        if not isinstance(item,dict) or set(item)!={'sequence_id','public_root'}:
            raise ValueError('Only public paths and external report IDs allowed')
        name=item['sequence_id']
        if not isinstance(name,str) or not re.fullmatch(r'[a-z0-9_]{1,64}',name) or name in seen:
            raise ValueError('Invalid/duplicate report ID')
        if not isinstance(item['public_root'],str) or not Path(item['public_root']).is_absolute():
            raise ValueError('Expected absolute public root')
        seen.add(name)
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    statuses=[]
    env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1','OMP_NUM_THREADS':'2','OPENBLAS_NUM_THREADS':'2','MKL_NUM_THREADS':'2'}
    for item in data['sequences']:
        public=Path(item['public_root']);directory=output/item['sequence_id'];directory.mkdir()
        process=subprocess.run([sys.executable,'-m','roadvision.rgb_resampling_checks_run',
                                '--manifest',str(public/'sequence.json'),'--rgb-root',str(public),
                                '--output',str(directory/'diagnostics')],cwd=ROOT,env=env,text=True,capture_output=True)
        (directory/'stdout.log').write_text(process.stdout);(directory/'stderr.log').write_text(process.stderr)
        statuses.append(dict(sequence_id=item['sequence_id'],exit_code=process.returncode))
    phase=dict(scope='all public-RGB-only predictor children completed',sequences=statuses)
    (output/'phase-complete.json').write_text(json.dumps(phase,indent=2)+'\n')
    return phase


def _records(root, name):
    return [json.loads(s) for s in (root/name/'diagnostics/windows.jsonl').read_text().splitlines()]


def _physical_unknown(result):
    p,v=result['evidence_packet'],result['review']
    return (v['motion_authorized'] is False and v['arrival_status']=='unverified' and
            p['corridor_boundary']['state']==p['front_surface']['state']=='unknown' and
            all(p['corridor_boundary'][k] is None for k in ('offset_m','heading_rad')) and
            p['front_surface']['range_m'] is None)


def _counts(result, prefix, counter):
    counter[prefix+'_any']+=any(v['status']=='planar_flow_candidate' for v in result['regions'].values())
    counter[prefix+'_ground']+=any(v['status']=='planar_flow_candidate' for k,v in result['regions'].items() if k.endswith('_ground'))
    shared=result['shared_motion']['status']=='shared_motion_candidate'
    anchored=result['anchored_support']['status']=='anchored_motion_candidate'
    counter[prefix+'_shared']+=shared;counter[prefix+'_anchored']+=anchored;counter[prefix+'_either']+=shared or anchored


def audit_study(generated, baseline, predictions, output):
    generated,baseline,predictions=map(Path,(generated,baseline,predictions))
    # Required completion markers checked BEFORE any case/truth reads.
    phases=[json.loads((p/'phase-complete.json').read_text()) for p in (baseline,predictions)]
    sequences=json.loads((generated/'public-index.json').read_text())['sequences']
    statuses=[{v['sequence_id']:v['exit_code'] for v in p['sequences']} for p in phases]
    expected={v['sequence_id'] for v in sequences}
    if any(set(s)!=expected or len(p['sequences'])!=len(expected) for s,p in zip(statuses,phases)):
        raise ValueError('Incomplete/duplicate prediction phase')
    rows=[];errors=[];totals=Counter()
    for item in sequences:
        identity=item['sequence_id'];public=Path(item['public_root'])
        case=json.loads((public.parent/'audit/case.json').read_text())
        truth=json.loads((public.parent/'audit/truth.json').read_text())
        row=dict(sequence_id=identity,case=case,exit_codes=[s[identity] for s in statuses])
        if any(row['exit_codes']):
            row['expected_input_rejection']=case['declared_calibration_id']!=CALIBRATION_ID and row['exit_codes']==[2,2]
            if not row['expected_input_rejection']:errors.append(dict(sequence_id=identity,error='unexpected subprocess exit'))
            rows.append(row);continue
        old,new=_records(baseline,identity),_records(predictions,identity)
        if len(old)!=3 or len(new)!=3:errors.append(dict(sequence_id=identity,error='missing windows'))
        count=Counter();descriptors=Counter();transfers=[];changes=[]
        for a,b in zip(old,new):
            r=b['result'];previous=a['result'];ablation=r['ablations']['without_rectified_repetition_veto']
            if r['baseline']!=previous:errors.append(dict(sequence_id=identity,error='H baseline differs'))
            if not _physical_unknown(r):errors.append(dict(sequence_id=identity,error='physical claim made'))
            count['windows']+=1
            for label,z in (('old',previous),('new',r),('noveto',ablation)):_counts(z,label,count)
            changed={}
            for key,region in r['regions'].items():
                count['input_tracks']+=region['stage_counts']['input_tracks'];count['kept_tracks']+=len(region['tracks'])
                before=previous['regions'][key];old_ok=before['status']=='planar_flow_candidate';new_ok=region['status']=='planar_flow_candidate'
                if old_ok!=new_ok:changed[key]=dict(old=before['status'],new=region['status'])
                count['region_gained']+=new_ok and not old_ok;count['region_lost']+=old_ok and not new_ok
                count['candidate_'+key]+=new_ok
                descriptors.update(e['reason'] for e in region['descriptor_evidence'] if not e['accepted'])
                if region['rectified_ground_assumption']:
                    count['rectified_veto_rejections']+=sum(not e.get('rectified_repetition',{}).get('accepted',False) for e in region['descriptor_evidence'])
                    for e in region['descriptor_evidence']:
                        if e['accepted'] and not e['rectified_repetition']['accepted']:
                            errors.append(dict(sequence_id=identity,error='repetition veto bypass'))
                m=audit_region(truth,b['index'],region['view'],region)
                if m is not None:transfers.append(dict(window=b['index'],region=key,status=region['status'],**m))
            changes.append(dict(window=b['index'],regions=changed))
            for z in (r,ablation):
                for h in z['anchored_support']['hypotheses']:
                    if not h['accepted'] or not h['anchor_validation']['accepted'] or not h['support_validation']['accepted']:
                        errors.append(dict(sequence_id=identity,error='invalid heldout acceptance'))
                    for key,fit in h['fit_indices'].items():
                        if set(fit)&set(h['validation_indices'][key]):errors.append(dict(sequence_id=identity,error='fit/check overlap'))
        row.update(counts=dict(count),descriptor_rejection_counts=dict(descriptors),image_transfer=transfers,changes=changes)
        rows.append(row);totals.update(count)
    result=dict(version='rgb_resampling_checks_study_v1',scope='targeted frozen texture study, not navigation/generalization accuracy',
                case_count=len(rows),totals=dict(totals),errors=errors,cases=rows,
                expected_input_rejections=sum(r.get('expected_input_rejection',False) for r in rows),
                physical_geometry='unknown',motion_authorized=False,arrival_status='unverified')
    with Path(output).open('x') as f:f.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--suite',required=True,type=Path);p.add_argument('--freeze',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    args=p.parse_args(argv);verify_freeze(args.freeze);args.output.mkdir(parents=True,exist_ok=False)
    generation=generate_suite(args.suite,args.output/'generated')
    index=args.output/'generated/public-index.json'
    predict_h_index(index,args.output/'baseline');predict_index(index,args.output/'predictions')
    result=audit_study(args.output/'generated',args.output/'baseline',args.output/'predictions',args.output/'audit-result.json')
    verify_freeze(args.freeze)
    print(json.dumps({k:v for k,v in result.items() if k!='cases'},indent=2))
    return int(bool(generation['errors'] or result['errors']))


if __name__=='__main__':raise SystemExit(main())
