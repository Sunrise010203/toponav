"""Frozen RGB-only ground/rig comparison; truth is opened after predictions.

This is logical input separation, not an OS sandbox or a claim of blind study.
The fixed generator supplies scripted camera samples, never wheel execution.
"""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys

import numpy as np

from roadvision.corridor_rgb_probe import CALIBRATION_ID, _project
from simulation.rgb_observability import plane_transfer, view_basis
from simulation.rgb_observability_run import ROOT, verify_freeze, generate_suite, predict_index


def predict_ground_index(index_path, output):
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
        command=[sys.executable,'-m','roadvision.rgb_ground_rig_run','--manifest',str(public/'sequence.json'),
                 '--rgb-root',str(public),'--output',str(directory/'diagnostics')]
        process=subprocess.run(command,cwd=ROOT,env=env,text=True,capture_output=True)
        (directory/'stdout.log').write_text(process.stdout)
        (directory/'stderr.log').write_text(process.stderr)
        results.append({'sequence_id':item['sequence_id'],'exit_code':process.returncode})
    phase={'scope':'all public-RGB-only children completed','sequences':results}
    (output/'phase-complete.json').write_text(json.dumps(phase,indent=2)+'\n')
    return phase


def audit_region(truth, index, view, diagnostic):
    """Finite nearest-visible-plane image transfer on tracked model inliers.

    Camera poses/planes are used here only, never in tracking or acceptance.
    This is not an independent pixel correspondence annotation or pose score.
    """
    if 'homography' not in diagnostic['models']: return None
    first,last=truth['captures'][index],truth['captures'][index+2]
    points=np.array([t['first'] for t in diagnostic['tracks']],float)
    inliers=np.array(diagnostic['models']['homography_inlier_mask'],bool)
    f=64/math.tan(math.radians(first['rendered_fov_deg'])/2)
    k=np.array([[f,0,63.5],[0,f,63.5],[0,0,1.]])
    _,_,basis=view_basis(first,view)
    rays=np.column_stack((points,np.ones(len(points))))@np.linalg.inv(k).T@basis.T
    origin=np.array(first['position_m'])
    distances=np.full(len(points),np.inf); selected=np.full(len(points),-1,int)
    surfaces=truth['surfaces']
    for si,surface in enumerate(surfaces):
        n=np.array(surface['normal_world'])
        v=np.array(surface['vertices_world_m'])+np.array(first['surface_shift_m'])
        den=rays@n
        distance=np.divide(n@(v[0]-origin),den,out=np.full(len(points),np.inf),where=np.abs(den)>1e-9)
        # Avoid invalid inf*zero in rays that do not intersect this plane.
        hit=origin+rays*np.where(np.isfinite(distance),distance,0.)[:,None]
        good=(distance>.05)&np.isfinite(distance)&inliers
        for edge in (v[1]-v[0],v[3]-v[0]):
            t=(hit-v[0])@edge/(edge@edge)
            good&=(t>=0)&(t<=1)
        good&=distance<distances
        distances[good]=distance[good];selected[good]=si
    predicted=_project(np.array(diagnostic['models']['homography']),points)
    errors=[]; labels=Counter()
    for si,surface in enumerate(surfaces):
        good=selected==si
        if not good.any(): continue
        expected=_project(plane_transfer(surface,first,last,view),points[good])
        if expected is None or predicted is None: continue
        errors.extend(np.linalg.norm(predicted[good]-expected,axis=1).tolist())
        labels[surface['name']]+=int(good.sum())
    return {'visible_plane_points':len(errors),'surface_counts':dict(labels),
            'transfer_error_median_px':float(np.median(errors)) if errors else None,
            'transfer_error_max_px':max(errors) if errors else None,
            'scope':'audit-only image transfer on model inliers; not metric pose accuracy'}


def _records(root, identity):
    return [json.loads(line) for line in (root/identity/'diagnostics/windows.jsonl').read_text().splitlines()]


def audit_study(generated,baseline,prediction,output):
    generated,baseline,prediction=map(Path,(generated,baseline,prediction))
    # Both completion markers must be opened before any audit truth or case.
    phases=[json.loads((root/'phase-complete.json').read_text()) for root in (baseline,prediction)]
    sequences=json.loads((generated/'public-index.json').read_text())['sequences']
    statuses=[{v['sequence_id']:v['exit_code'] for v in phase['sequences']} for phase in phases]
    if any(set(s)!={v['sequence_id'] for v in sequences} for s in statuses):
        raise ValueError('Incomplete prediction phase')
    rows,errors=[],[]
    total=Counter()
    for sequence in sequences:
        identity=sequence['sequence_id']; public=Path(sequence['public_root'])
        case=json.loads((public.parent/'audit/case.json').read_text())
        truth=json.loads((public.parent/'audit/truth.json').read_text())
        row={'sequence_id':identity,'case':case,'exit_codes':[s[identity] for s in statuses]}
        if any(s[identity]!=0 for s in statuses):
            expected=case['declared_calibration_id']!=CALIBRATION_ID and row['exit_codes']==[2,2]
            row['expected_input_rejection']=expected
            if not expected: errors.append({'sequence_id':identity,'error':'unexpected subprocess rejection'})
            rows.append(row); continue
        old,new=_records(baseline,identity),_records(prediction,identity)
        if len(old)!=3 or len(new)!=3: errors.append({'sequence_id':identity,'error':'missing windows'})
        counts=Counter(); transfer=[]; stages=[]
        for a,b in zip(old,new):
            result=b['result']; original=a['result']
            if original!=result['baseline']:
                errors.append({'sequence_id':identity,'error':'embedded frozen baseline differs'})
            packet,review=result['evidence_packet'],result['review']
            if (review['motion_authorized'] is not False or review['arrival_status']!='unverified' or
                    packet['corridor_boundary']['state']!='unknown' or packet['front_surface']['state']!='unknown' or
                    any(packet['corridor_boundary'][k] is not None for k in ('offset_m','heading_rad')) or
                    packet['front_surface']['range_m'] is not None):
                errors.append({'sequence_id':identity,'error':'physical claim made'})
            counts['windows']+=1
            counts['baseline_any']+=original['diagnostic_status']=='planar_flow_candidate'
            counts['raw_any']+=any(v['status']=='planar_flow_candidate' for k,v in result['regions'].items() if k.endswith('_raw'))
            counts['ground_any']+=any(v['status']=='planar_flow_candidate' for k,v in result['regions'].items() if k.endswith('_ground'))
            counts['shared']+=result['shared_motion']['status']=='shared_motion_candidate'
            for kind,value in result['ablations'].items(): counts['shared_'+kind]+=value['status']=='shared_motion_candidate'
            for key,region in result['regions'].items():
                stages.append({'window':b['index'],'region':key,'counts':region['stage_counts'],'reasons':region['reasons']})
                counts[key]+=region['status']=='planar_flow_candidate'
                metric=audit_region(truth,b['index'],region['view'],region)
                if metric is not None:
                    transfer.append({'window':b['index'],'region':key,'status':region['status'],**metric})
        row.update(counts=dict(counts),image_transfer_audit=transfer,tracking_stages=stages)
        total.update(counts);rows.append(row)
    degeneracies=[r for r in rows if r['case']['motion'] in {'static','yaw','tiny_forward'} or r['case']['texture'] in {'uniform','repeated'}]
    result={'version':'rgb_ground_rig_study_v1','scope':'targeted new-seed validation after development; overlapping windows, not navigation/generalization rate',
            'case_count':len(rows),'totals':dict(total),'errors':errors,'cases':rows,
            'expected_input_rejections':sum(r.get('expected_input_rejection',False) for r in rows),
            'degeneracy_cases':len(degeneracies),
            'degeneracy_counts':dict(sum((Counter(r.get('counts',{})) for r in degeneracies),Counter())),
            'physical_geometry':'unknown','motion_authorized':False,'arrival_status':'unverified'}
    with Path(output).open('x') as f: f.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite',required=True,type=Path)
    parser.add_argument('--freeze',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args(argv)
    verify_freeze(args.freeze)
    args.output.mkdir(parents=True,exist_ok=False)
    generation=generate_suite(args.suite,args.output/'generated')
    predict_index(args.output/'generated/public-index.json',args.output/'baseline')
    predict_ground_index(args.output/'generated/public-index.json',args.output/'predictions')
    result=audit_study(args.output/'generated',args.output/'baseline',args.output/'predictions',args.output/'audit-result.json')
    verify_freeze(args.freeze)
    print(json.dumps({k:v for k,v in result.items() if k not in ('cases',)},indent=2))
    return int(bool(generation['errors'] or result['errors']))


if __name__=='__main__': raise SystemExit(main())
