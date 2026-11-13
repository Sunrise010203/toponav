"""RGB history development/acceptance harness, not a navigation controller.

Generation owns truth in audit/. Predictor children receive only public/
paths. Audits are a later, explicit phase after ALL children complete.
Camera-only counterfactuals are labelled as such, never wheel execution.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
import os
import re
from pathlib import Path
import subprocess
import sys
import numpy as np
from PIL import Image
import pybullet as p

from simulation.textured_corridor import TexturedCorridorWorld
from simulation.corridor_motion import CorridorConfig
from roadvision.corridor_rgb_probe import CALIBRATION_ID

ROOT=Path(__file__).resolve().parents[1]


def render_camera(world,position,yaw,*,fov=90.,pitch=0.):
    """Generator-only camera. No images or poses are supplied to decision code."""
    origin=np.array(position);forward=np.array([math.cos(yaw)*math.cos(pitch),math.sin(yaw)*math.cos(pitch),math.sin(pitch)])
    left=np.array([-math.sin(yaw),math.cos(yaw),0.]);up=np.cross(forward,left)
    projection=p.computeProjectionMatrixFOV(fov,1,.05,30);images=[]
    for angle in (0,math.pi/2,-math.pi/2,math.pi):
        direction=math.cos(angle)*forward+math.sin(angle)*left
        view=p.computeViewMatrix(origin.tolist(),(origin+direction).tolist(),up.tolist())
        pixels=p.getCameraImage(128,128,view,projection,renderer=p.ER_TINY_RENDERER,flags=p.ER_NO_SEGMENTATION_MASK,
                               lightDirection=(1,-1,3),physicsClientId=world.client)[2]
        images.append(np.asarray(pixels,np.uint8).reshape(128,128,4)[...,:3])
    return np.concatenate((np.concatenate(images[:2],axis=1),np.concatenate(images[2:],axis=1)),axis=0)


def generate_case(case,directory):
    d=Path(directory);(d/'public/frames').mkdir(parents=True,exist_ok=False);(d/'audit').mkdir()
    refs=[];truth=[];config=CorridorConfig(**case.get('scene',{}))
    with TexturedCorridorWorld(config,case.get('appearance'),assets=d/'audit/assets',seed=case['seed'],
                               texture_cell_m=case.get('texture_cell_m',.05),pattern=case.get('pattern','unique')) as w:
        if case['mode']=='wheels':
            for _ in range(case.get('preroll_ticks',0)):w.drive(case.get('preroll_v',.3),0.);w.step_control()
        for i in range(case.get('frame_count',17)):
            t=i*.1
            if case['mode']=='wheels':
                frame=w.capture();audit=w.audit_state();audit['yaw_rad']=math.radians(audit['yaw_deg']);audit['camera_xyz']=audit['camera_position_m']
            elif case['mode']=='camera_only':
                start=case.get('camera_start_m',[2.,0.,.45]);velocity=case.get('camera_velocity_m_s',[.15,0.,0.])
                position=(np.array(start)+t*np.array(velocity)).tolist();yaw=case.get('camera_yaw_rad',0.)+t*case.get('camera_yaw_rate_rad_s',0.)
                frame=dict(frame_id=f'camera-only-{i}',captured_sim_s=round(t,10),rgb=render_camera(w,position,yaw,fov=case.get('fov_deg',90.),pitch=case.get('pitch_rad',0.)))
                audit=dict(camera_xyz=position,yaw_rad=yaw,sim_s=round(t,10),wall_contacts=None)
            else:raise ValueError('Only wheels or explicit camera_only generation')
            relative=f'frames/{i:04d}.png';Image.fromarray(frame['rgb']).save(d/'public'/relative)
            refs.append({k:v for k,v in frame.items() if k!='rgb'}|dict(rgb_path=relative,rgb_sha256=hashlib.sha256(frame['rgb'].tobytes()).hexdigest()))
            truth.append(audit)
            if case['mode']=='wheels':w.drive(case.get('v',.15),case.get('omega',0.));w.step_control()
    (d/'public/sequence.json').write_text(json.dumps(dict(version='rgb_probe_sequence_v1',calibration_id=case.get('calibration_id',CALIBRATION_ID),frames=refs),indent=2)+'\n')
    (d/'audit/truth.json').write_text(json.dumps(truth,indent=2)+'\n');(d/'audit/case.json').write_text(json.dumps(case,indent=2)+'\n')


def generate(suite_path,output):
    suite=json.loads(Path(suite_path).read_text());out=Path(output);out.mkdir(parents=True,exist_ok=False);index=[]
    for case in suite['cases']:
        name=case['id']
        if not isinstance(name,str) or not name.replace('_','').isalnum() or any(x['sequence_id']==name for x in index):raise ValueError('Invalid case ID')
        d=out/'cases'/name;generate_case(case,d);index.append(dict(sequence_id=name,public_root=str((d/'public').resolve())));print(name,'generated',flush=True)
    (out/'public-index.json').write_text(json.dumps(dict(version='rgb_observability_public_index_v1',sequences=index),indent=2)+'\n')


def predict(index_path,output,*,method='history',stride=4):
    index=json.loads(Path(index_path).read_text())
    if not isinstance(index,dict) or set(index)!={'version','sequences'} or index['version']!='rgb_observability_public_index_v1' or not isinstance(index['sequences'],list):
        raise ValueError('Invalid public index')
    seen=set()
    for item in index['sequences']:
        if not isinstance(item,dict) or set(item)!={'sequence_id','public_root'}:raise ValueError('Only public path and external reporting ID allowed')
        name=item['sequence_id'];public=item['public_root']
        if not isinstance(name,str) or not re.fullmatch('[a-z0-9_]{1,64}',name) or name in seen:raise ValueError('Invalid or duplicate external ID')
        if not isinstance(public,str) or not Path(public).is_absolute():raise ValueError('Public root must be absolute')
        seen.add(name)
    modules=dict(history='roadvision.ground_history_run',adaptive='roadvision.ground_history_run',H='roadvision.rgb_motion_checks_run',I='roadvision.rgb_resampling_checks_run')
    if method not in modules:raise ValueError('Unsupported predictor method')
    out=Path(output);out.mkdir(parents=True,exist_ok=False);statuses=[]
    for item in index['sequences']:
        public=Path(item['public_root']).resolve();name=item['sequence_id']
        if not public.is_absolute() or not name.replace('_','').isalnum():raise ValueError('Invalid public index')
        cmd=[sys.executable,'-m',modules[method],'--manifest',str(public/'sequence.json'),'--rgb-root',str(public),'--output',str(out/name)]
        if method in ('history','adaptive'):cmd+=['--stride',str(stride)]
        if method=='adaptive':cmd+=['--adaptive']
        process=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,
            env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1','OMP_NUM_THREADS':'2','OPENBLAS_NUM_THREADS':'2','MKL_NUM_THREADS':'2'})
        (out/(name+'-stdout.log')).write_text(process.stdout);(out/(name+'-stderr.log')).write_text(process.stderr)
        statuses.append(dict(sequence_id=name,exit_code=process.returncode));print(name,method,process.returncode,flush=True)
    (out/'phase-complete.json').write_text(json.dumps(dict(method=method,sequences=statuses),indent=2)+'\n')


def audit(generated,predicted,output):
    generated=Path(generated);predicted=Path(predicted);phase=json.loads((predicted/'phase-complete.json').read_text())
    assert phase['method'] in ('history','adaptive')
    index=json.loads((generated/'public-index.json').read_text())['sequences']
    if {x['sequence_id'] for x in phase['sequences']}!={x['sequence_id'] for x in index}:raise ValueError('Incomplete predictions')
    status={x['sequence_id']:x['exit_code'] for x in phase['sequences']};summaries=[]
    for item in index:
        name=item['sequence_id'];d=generated/'cases'/name;case=json.loads((d/'audit/case.json').read_text())
        if status[name]:summaries.append(dict(case=case,exit_code=status[name]));continue
        truth=json.loads((d/'audit/truth.json').read_text());rows=[json.loads(s)['result'] for s in (predicted/name/'windows.jsonl').read_text().splitlines()]
        summary=json.loads((predicted/name/'summary.json').read_text());stride=summary['stride'];errors=[];unsafe=[]
        for i,r in enumerate(rows):
            a,b=truth[i],truth[i+2*stride];yaw=a['yaw_rad'];rotation=np.array([[math.cos(yaw),math.sin(yaw)],[-math.sin(yaw),math.cos(yaw)]])
            t=rotation@(np.array(b['camera_xyz'])[:2]-np.array(a['camera_xyz'])[:2]);m=r.get('ground_motion',{});c=r.get('corridor_geometry',{});f=r.get('front_surface',{});e=dict(index=i)
            if m.get('status')=='conditional_ground_motion':e.update(translation_error_m=float(np.linalg.norm(np.array(m['translation_m'])-t)),yaw_error_rad=abs(m['yaw_delta_rad']-(b['yaw_rad']-a['yaw_rad'])))
            width=case.get('scene',{}).get('width_m',1.8)
            if c.get('status')=='conditional_corridor_geometry':e.update(offset_error_m=abs(c['offset_m']-b['camera_xyz'][1]),heading_error_rad=abs(c['heading_error_rad']-b['yaw_rad']),width_error_m=abs(c['width_m']-width))
            if f.get('status')=='conditional_front_surface':
                surface=case.get('audit_front_x_m',case.get('scene',{}).get('length_m',6.))
                actual=(surface-b['camera_xyz'][0])/math.cos(b['yaw_rad']);e.update(front_error_m=abs(f['range_m']-actual),actual_front_range_m=actual)
                if 'range_interval_m' in f:e['front_actual_inside_conditional_interval']=f['range_interval_m'][0]<=actual<=f['range_interval_m'][1]
            if r.get('motion_authorized') is not False or r.get('arrival_status')!='unverified':unsafe.append(i)
            errors.append(e)
        metrics={k:dict(count=sum(k in e for e in errors),max=max((e[k] for e in errors if k in e),default=None)) for k in ('translation_error_m','yaw_error_rad','offset_error_m','heading_error_rad','width_error_m','front_error_m')}
        summaries.append(dict(case=case,summary=summary,metrics=metrics,unauthorized_claims=unsafe,errors=errors))
    result=dict(scope='conditional geometry/motion RGB study, NOT navigation/control acceptance',cases=summaries)
    Path(output).write_text(json.dumps(result,indent=2)+'\n');return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    g=sub.add_parser('generate');g.add_argument('--suite',required=True,type=Path);g.add_argument('--output',required=True,type=Path)
    p1=sub.add_parser('predict');p1.add_argument('--index',required=True,type=Path);p1.add_argument('--output',required=True,type=Path);p1.add_argument('--method',choices=['history','adaptive','H','I'],default='history');p1.add_argument('--stride',type=int,default=4)
    a=sub.add_parser('audit');a.add_argument('--generated',required=True,type=Path);a.add_argument('--predicted',required=True,type=Path);a.add_argument('--output',required=True,type=Path)
    args=parser.parse_args(argv)
    if args.command=='generate':generate(args.suite,args.output)
    elif args.command=='predict':predict(args.index,args.output,method=args.method,stride=args.stride)
    else:audit(args.generated,args.predicted,args.output)


if __name__=='__main__':main()
