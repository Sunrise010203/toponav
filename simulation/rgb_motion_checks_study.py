"""Frozen ambiguity/low-parallax study: public RGB prediction before truth audit."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import re
import subprocess
import sys

from roadvision.corridor_rgb_probe import CALIBRATION_ID
from roadvision.rgb_motion_checks import CHECK_POLICY
from simulation.rgb_observability_run import ROOT,verify_freeze,generate_suite
from simulation.rgb_ground_rig_study import predict_ground_index,audit_region


def predict_index(index_path, output):
    data=json.loads(Path(index_path).read_text())
    if (not isinstance(data,dict) or set(data)!={'version','sequences'} or
            data['version']!='rgb_observability_public_index_v1' or not isinstance(data['sequences'],list)):
        raise ValueError('Invalid public sequence index')
    seen=set()
    for item in data['sequences']:
        if not isinstance(item,dict) or set(item)!={'sequence_id','public_root'}:
            raise ValueError('Only public paths and external IDs permitted')
        name=item['sequence_id']
        if not isinstance(name,str) or not re.fullmatch(r'[a-z0-9_]{1,64}',name) or name in seen:
            raise ValueError('Invalid/duplicate external report ID')
        if not isinstance(item['public_root'],str) or not Path(item['public_root']).is_absolute():
            raise ValueError('Expected absolute public RGB root')
        seen.add(name)
    output=Path(output); output.mkdir(parents=True,exist_ok=False)
    results=[]
    env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1','OMP_NUM_THREADS':'2','OPENBLAS_NUM_THREADS':'2','MKL_NUM_THREADS':'2'}
    for item in data['sequences']:
        public=Path(item['public_root']); directory=output/item['sequence_id']; directory.mkdir()
        command=[sys.executable,'-m','roadvision.rgb_motion_checks_run','--manifest',str(public/'sequence.json'),
                 '--rgb-root',str(public),'--output',str(directory/'diagnostics')]
        process=subprocess.run(command,cwd=ROOT,env=env,text=True,capture_output=True)
        (directory/'stdout.log').write_text(process.stdout)
        (directory/'stderr.log').write_text(process.stderr)
        results.append({'sequence_id':item['sequence_id'],'exit_code':process.returncode})
    phase={'scope':'all public-RGB-only children completed','sequences':results}
    (output/'phase-complete.json').write_text(json.dumps(phase,indent=2)+'\n')
    return phase


def _records(root,identity):
    return [json.loads(s) for s in (root/identity/'diagnostics/windows.jsonl').read_text().splitlines()]


def audit_study(generated,baseline,predictions,output):
    generated,baseline,predictions=map(Path,(generated,baseline,predictions))
    # Completion markers before *any* audit/case/truth reads.
    phases=[json.loads((p/'phase-complete.json').read_text()) for p in (baseline,predictions)]
    sequences=json.loads((generated/'public-index.json').read_text())['sequences']
    statuses=[{v['sequence_id']:v['exit_code'] for v in p['sequences']} for p in phases]
    if any(set(s)!={v['sequence_id'] for v in sequences} for s in statuses):raise ValueError('Incomplete prediction phase')
    rows,errors=[],[];totals=Counter()
    for item in sequences:
        identity=item['sequence_id'];public=Path(item['public_root'])
        case=json.loads((public.parent/'audit/case.json').read_text())
        truth=json.loads((public.parent/'audit/truth.json').read_text())
        row={'sequence_id':identity,'case':case,'exit_codes':[s[identity] for s in statuses]}
        if any(s[identity]!=0 for s in statuses):
            expected=case['declared_calibration_id']!=CALIBRATION_ID and row['exit_codes']==[2,2]
            row['expected_input_rejection']=expected
            if not expected:errors.append({'sequence_id':identity,'error':'unexpected subprocess exit'})
            rows.append(row);continue
        old,new=_records(baseline,identity),_records(predictions,identity)
        if len(old)!=3 or len(new)!=3:errors.append({'sequence_id':identity,'error':'missing windows'})
        count=Counter();descriptors=Counter();transfer=[];validation=[]
        for a,b in zip(old,new):
            r=b['result'];original=a['result']
            if original!=r['baseline']:errors.append({'sequence_id':identity,'error':'G baseline differs'})
            p,v=r['evidence_packet'],r['review']
            if (v['motion_authorized'] is not False or v['arrival_status']!='unverified' or
                p['corridor_boundary']['state']!='unknown' or p['front_surface']['state']!='unknown' or
                any(p['corridor_boundary'][k] is not None for k in ('offset_m','heading_rad')) or p['front_surface']['range_m'] is not None):
                errors.append({'sequence_id':identity,'error':'physical claim made'})
            count['windows']+=1
            for name,regions in (('old',original['regions']),('new',r['regions'])):
                count[name+'_any']+=any(v['status']=='planar_flow_candidate' for v in regions.values())
                count[name+'_ground']+=any(v['status']=='planar_flow_candidate' for k,v in regions.items() if k.endswith('_ground'))
            count['old_shared']+=original['shared_motion']['status']=='shared_motion_candidate'
            count['new_shared']+=r['shared_motion']['status']=='shared_motion_candidate'
            count['anchored']+=r['anchored_support']['status']=='anchored_motion_candidate'
            count['shared_without_upper']+=r['ablations']['shared_without_upper']['status']=='shared_motion_candidate'
            count['anchored_without_upper']+=r['ablations']['anchored_without_upper']
            count['either_shared']+=(r['shared_motion']['status']=='shared_motion_candidate' or r['anchored_support']['status']=='anchored_motion_candidate')
            for key,region in r['regions'].items():
                count['candidate_'+key]+=region['status']=='planar_flow_candidate'
                count['input_tracks']+=region['stage_counts']['input_tracks']
                count['kept_tracks']+=region['stage_counts']['retained']
                descriptors.update(e['reason'] for e in region['descriptor_evidence'] if not e['accepted'])
                m=audit_region(truth,b['index'],region['view'],region)
                if m is not None:transfer.append({'window':b['index'],'region':key,'status':region['status'],**m})
            anchored=r['anchored_support']
            validation.append({'window':b['index'],'region_checks':anchored['region_checks'],'pair_checks':anchored['pair_checks']})
            for h in anchored['hypotheses']:
                for key,fit in h['fit_indices'].items():
                    if set(fit)&set(h['validation_indices'][key]):errors.append({'sequence_id':identity,'error':'train/validation overlap'})
                for metric in (h['anchor_validation'],h['support_validation']):
                    if not metric['accepted']:errors.append({'sequence_id':identity,'error':'invalid heldout acceptance'})
        row.update(counts=dict(count),descriptor_rejection_counts=dict(descriptors),image_transfer=transfer,heldout_checks=validation)
        rows.append(row);totals.update(count)
    degeneracies=[r for r in rows if r['case']['texture'] in ('repeated','uniform') or r['case']['motion'] in ('static','yaw','tiny_forward')]
    slow=[r for r in rows if r['case']['scene']=='room' and r['case']['motion']=='slow_forward' and r['case']['texture']=='unique']
    result={'version':'rgb_motion_checks_study_v1','scope':'targeted frozen new-seed study, not navigation/generalization rate',
            'case_count':len(rows),'totals':dict(totals),'errors':errors,'cases':rows,
            'expected_input_rejections':sum(r.get('expected_input_rejection',False) for r in rows),
            'degeneracy_cases':len(degeneracies),'degeneracy_counts':dict(sum((Counter(r.get('counts',{})) for r in degeneracies),Counter())),
            'slow_room_counts':dict(sum((Counter(r.get('counts',{})) for r in slow),Counter())),
            'physical_geometry':'unknown','motion_authorized':False,'arrival_status':'unverified'}
    with Path(output).open('x') as f:f.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--suite',required=True,type=Path);p.add_argument('--freeze',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    args=p.parse_args(argv);verify_freeze(args.freeze);args.output.mkdir(parents=True,exist_ok=False)
    generation=generate_suite(args.suite,args.output/'generated')
    predict_ground_index(args.output/'generated/public-index.json',args.output/'baseline')
    predict_index(args.output/'generated/public-index.json',args.output/'predictions')
    result=audit_study(args.output/'generated',args.output/'baseline',args.output/'predictions',args.output/'audit-result.json')
    verify_freeze(args.freeze)
    print(json.dumps({k:v for k,v in result.items() if k!='cases'},indent=2))
    return int(bool(generation['errors'] or result['errors']))


if __name__=='__main__':raise SystemExit(main())
