"""Serial real-ROS launch harness, public replay and independent physics audit.

No ROS import is needed for schema checks or replay. Rendering lives ONLY in
prepare(); neither controller process nor replayer reads fixture/audit files.
"""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
import uuid

from .protocol import VERSION,decode,encode,load_public,command_from_decision,validate_command,number,identity

ROOT=Path(__file__).resolve().parents[4]
FAULTS=('none','camera_silence','navigation_hang','feedback_silence','replay_command','bad_version','future_camera')


def write(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x') as f:json.dump(data,f,indent=2,allow_nan=False);f.write('\n')


def read(path):return json.loads(Path(path).read_text())


def lines(path):return [json.loads(l) for l in Path(path).read_text().splitlines()]


def validate_case(case):
    import re
    from simulation.topology import validate_pair
    from simulation.path_planning import plan_from_estimate
    fields={'id','seed','graph','physical_scene','start_node','goal_node','heading_deg','fault','fault_tick','max_ticks'}
    if not isinstance(case,dict) or set(case)!=fields:raise ValueError('Invalid S7 case fields')
    if not isinstance(case['id'],str) or not re.fullmatch('[a-z0-9_-]{1,64}',case['id']):raise ValueError('Invalid S7 case ID')
    if type(case['seed']) is not int or not 0<=case['seed']<2**31 or case['fault'] not in FAULTS:raise ValueError('Invalid S7 seed/fault')
    if type(case['max_ticks']) is not int or case['max_ticks']!=1200:raise ValueError('S7 execution budget changed')
    if case['fault_tick'] is not None and (type(case['fault_tick']) is not int or case['fault_tick']!=180):raise ValueError('S7 partial-turn fault tick changed')
    if (case['fault']=='none')!=(case['fault_tick'] is None):raise ValueError('Missing/unexpected S7 fault tick')
    validate_pair(case['graph'],case['physical_scene'])
    if type(case['heading_deg']) not in (int,float) or case['heading_deg'] not in (0,90,180,270):raise ValueError('Invalid cardinal task prior')
    p=plan_from_estimate(case['graph'],dict(status='localized',node_id=case['start_node']),case['goal_node'])
    if p['status']!='reachable' or p['hop_count']!=2:raise ValueError('S7 requires a full two-edge route')
    if case['graph']['task']!=dict(start_node=case['start_node'],goal_node=case['goal_node'],initial_heading_deg=case['heading_deg']):raise ValueError('Public task conflict')
    return case


def load_suite(path):
    suite=read(path)
    if set(suite)!={'version','status','scope','cases'} or suite['version']!='ros2_s7_v1' or suite['status']!='FROZEN_FOR_FIRST_GENERATION':raise ValueError('Invalid S7 suite envelope')
    cases=suite['cases']
    if not isinstance(cases,list) or len(cases)!=8:raise ValueError('Exactly8 S7 conditions required')
    for c in cases:validate_case(c)
    if len({c['id'] for c in cases})!=8 or len({c['seed'] for c in cases})!=8:raise ValueError('Duplicate S7 IDs/seeds')
    if Counter(c['fault'] for c in cases)!=Counter(dict(none=2,**{f:1 for f in FAULTS if f!='none'})):raise ValueError('S7 safety/normal denominators changed')
    normals=[c for c in cases if c['fault']=='none']
    if len({json.dumps(c['physical_scene']['geometry'],sort_keys=True) for c in normals})!=2:raise ValueError('Two distinct S7 normal layouts required')
    return suite


def prepare(case,base):
    """Fixture/map acquisition BEFORE launch, never a navigation callback."""
    from PIL import Image
    from roadvision.corridor_rgb_probe import CALIBRATION_ID
    from simulation.continuous_audit_world import RouteAuditWorld
    validate_case(case);base=Path(base).resolve();base.mkdir(parents=True,exist_ok=False)
    public=base/'public';public.mkdir();(public/'references').mkdir();(base/'fixture').mkdir();(base/'logs').mkdir()
    task=dict(version=VERSION,session_id='ros2-route:'+uuid.uuid4().hex,start_node=case['start_node'],goal_node=case['goal_node'],heading_deg=case['heading_deg'],operator_confirmed=True,calibration_id=CALIBRATION_ID)
    write(public/'task.json',task);write(public/'topology.json',case['graph']);write(base/'fixture/scenario.json',case)
    manifest=[]
    with RouteAuditWorld(physical_scene=case['physical_scene'],assets=base/'fixture/map-assets',seed=case['seed'],spawn_node=case['start_node'],spawn_heading_deg=case['heading_deg']) as world:
        for i,ref in enumerate(world.reference_atlas()):
            name=f'references/{i:03d}.png';Image.fromarray(ref['rgb']).save(public/name)
            manifest.append(dict(node_id=ref['node_id'],heading_deg=ref['heading_deg'],rgb_path=name,rgb_sha256=hashlib.sha256(ref['rgb'].tobytes()).hexdigest()))
    write(public/'references.json',manifest)
    return base


def run_case(case,base,*,timeout_s=180.):
    base=prepare(case,base);fault=case['fault'];events=[];stopped_pid=None;continued=False
    env={**os.environ,'ROS_DOMAIN_ID':str(40+case['seed']%160),'ROS_LOCALHOST_ONLY':'1',
         'RMW_IMPLEMENTATION':'rmw_fastrtps_cpp','ROS_LOG_DIR':str(base/'logs/ros'),
         'PYTHONDONTWRITEBYTECODE':'1','OMP_NUM_THREADS':'2','OPENBLAS_NUM_THREADS':'2','MKL_NUM_THREADS':'2',
         'PYTHONPATH':str(ROOT)+os.pathsep+os.environ.get('PYTHONPATH','')}
    command=['ros2','launch','toponav_bridge','continuous_route.launch.py',
             'public:='+str(base/'public'),'output:='+str(base),'scenario:='+str(base/'fixture/scenario.json'),
             'drop_feedback:='+str(fault=='feedback_silence').lower(),
             'inject_command:='+str(fault in ('replay_command','bad_version')).lower(),
             'fault_mode:='+(fault if fault in ('replay_command','bad_version') else 'replay_command'),
             'fault_tick:=180']
    started=time.monotonic()
    write(base/'environment.json',dict(command=command,python=sys.executable,ros_distro=env.get('ROS_DISTRO'),domain_id=env['ROS_DOMAIN_ID'],rmw=env['RMW_IMPLEMENTATION'],clock='lock-step simulation + independent steady wall lease',prior_clearance='operator-confirmed .30m sweep; explicit fixture assumption'))
    with (base/'logs/launch.log').open('x') as log:
        process=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        error=None
        try:
            while process.poll() is None:
                if time.monotonic()-started>timeout_s:raise TimeoutError('S7 ROS launch wall budget exceeded')
                progress=base/'world/progress.json';info=base/'navigation/process.json'
                if fault=='navigation_hang' and progress.exists() and info.exists():
                    state=read(progress)
                    if stopped_pid is None and state['phase']=='running' and state['tick']>=case['fault_tick']:
                        stopped_pid=read(info)['pid'];os.kill(stopped_pid,signal.SIGSTOP)
                        events.append(dict(signal='SIGSTOP',pid=stopped_pid,tick=state['tick'],steady_s=time.monotonic()))
                    if stopped_pid is not None and not continued and state['latched']:
                        os.kill(stopped_pid,signal.SIGCONT);continued=True
                        events.append(dict(signal='SIGCONT',pid=stopped_pid,tick=state['tick'],steady_s=time.monotonic()))
                time.sleep(.02)
            if process.returncode!=0:raise RuntimeError('ROS launch failed: '+str(process.returncode))
            if not (base/'world/result.json').exists() or not (base/'navigation/result.json').exists():raise RuntimeError('Missing ROS process results')
        except Exception:
            error=traceback.format_exc();raise
        finally:
            if stopped_pid is not None and not continued:
                try:os.kill(stopped_pid,signal.SIGCONT)
                except ProcessLookupError:pass
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=12)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            write(base/'orchestration.json',dict(exit_code=process.returncode,elapsed_wall_s=time.monotonic()-started,signals=events,error=error))
    return base


def terminal_inventory(base):
    """Bind exactly three declared camera probes, separate from queued RGB.

    Legacy controller-only fixtures lack a camera inventory and retain the
    original exact-three-terminal-images rule. Actual camera runs must bind
    every received image and all three declared probes to public metadata.
    """
    path=Path(base)/'public/frames.json'
    if not path.exists():return None
    frames=read(path)
    if not isinstance(frames,list):raise ValueError('Invalid public frame inventory')
    inventory={}
    required={'frame_id','captured_sim_s','rgb_sha256','terminal','suppressed'}
    for f in frames:
        if (not isinstance(f,dict) or not required<=set(f) or not identity(f['frame_id']) or
                not number(f['captured_sim_s']) or f['captured_sim_s']<0 or
                type(f['terminal']) is not bool or type(f['suppressed']) is not bool or
                not isinstance(f['rgb_sha256'],str) or len(f['rgb_sha256'])!=64):
            raise ValueError('Invalid public frame metadata')
        if f['frame_id'] in inventory:raise ValueError('Duplicate camera frame identity')
        inventory[f['frame_id']]=f
    probes=[f for f in frames if f['terminal']]
    if len(probes)!=3 or any(f['suppressed'] for f in probes):raise ValueError('Three published terminal probes required')
    return inventory


def replay_navigation(base):
    """NO world, fixture, physics, audit or live ROS/DDS reads."""
    import numpy as np
    from PIL import Image
    from roadvision.continuous_straight import ContinuousRouteController,InitialClearancePrior
    from simulation.path_planning import plan_from_estimate,next_action
    base=Path(base);root=base/'navigation';task,graph,references=load_public(base/'public')
    inventory=terminal_inventory(base);terminal_ids=[]
    c=ContinuousRouteController(graph=graph,references=references,plan_from_estimate=plan_from_estimate,next_action=next_action,
        start=task['start_node'],goal=task['goal_node'],heading=task['heading_deg'],initial_clearance=InitialClearancePrior(confirmed=task['operator_confirmed']),session_id=task['session_id'])
    sequence=0;images=0;terminal=0;feedbacks=0;rows=[];bridge_reason=None
    events=lines(root/'events.jsonl')
    for e in events:
        now=e['now_sim_s'];ref=None
        if e['kind']=='image':
            ref=e['input'];p=(root/ref['rgb_path']).resolve()
            if Path(ref['rgb_path']).is_absolute() or not p.is_relative_to(root.resolve()):raise ValueError('Received RGB path escaped root')
            rgb=np.array(Image.open(p))
            if rgb.shape!=(256,256,3) or rgb.dtype!=np.uint8 or hashlib.sha256(rgb.tobytes()).hexdigest()!=ref['rgb_sha256']:raise ValueError('Received RGB pixel mismatch')
            if inventory is not None:
                published=inventory.get(ref['frame_id'])
                if published is None or published['suppressed'] or any(published[k]!=ref[k] for k in ('captured_sim_s','rgb_sha256')):raise ValueError('Received RGB not bound to public camera inventory')
            before=c.result();was_terminal=before['state'] in ('visual_stop','aborted')
            d=c.update(dict(frame_id=ref['frame_id'],captured_sim_s=ref['captured_sim_s'],rgb=rgb),now);images+=1
            if e['terminal']!=was_terminal:raise ValueError('Wrong terminal input claim')
            if was_terminal:
                terminal+=1
                terminal_ids.append(ref['frame_id'])
                if not e['result_unchanged'] or c.result()!=before or d['v_m_s'] or d['omega_rad_s']:raise ValueError('Terminal good-frame latching failed')
        elif e['kind']=='feedback':
            f=e['feedback']
            if set(f)!={'version','session_id','applied_seq','applied_sim_s','event'} or type(f['version']) is not int or f['version']!=VERSION or f['session_id']!=task['session_id'] or type(f['applied_seq']) is not int or f['applied_seq']!=sequence or not number(f['applied_sim_s']) or not 0<=now-f['applied_sim_s']<=.250001:raise ValueError('Feedback transport binding differs on replay')
            d=c.feedback(f['event'],now);feedbacks+=1
        elif e['kind']=='bridge_abort':
            bridge_reason=e['transport_reason'];d=c.cancel(now)
        else:raise ValueError('Unknown public navigation event')
        if d!=e['decision']:raise ValueError('Exact public replay differs')
        if 'command' in e:
            sequence+=1
            if e['command']!=command_from_decision(task['session_id'],sequence,d):raise ValueError('Command not bound to exact public decision')
        rows.append(dict(tick=len(rows),sim_s=now,input=ref,decision=d))
    saved=read(root/'result.json')
    if c.result()!=saved['controller_result'] or saved['bridge_reason']!=bridge_reason or saved['images']!=images or saved['commands']!=sequence:raise ValueError('Final ROS public replay differs')
    summary=dict(images=images,decisions=len(events),terminal_frames=terminal,feedback_deliveries=feedbacks,commands=sequence)
    if inventory is None:
        if terminal!=3:raise ValueError('Three actual terminal good frames required')
    else:
        probes={name for name,f in inventory.items() if f['terminal']}
        if len(set(terminal_ids))!=len(terminal_ids) or not probes<=set(terminal_ids):raise ValueError('Missing or duplicate terminal probe delivery')
        summary.update(terminal_probe_frames=len(probes),queued_terminal_frames=terminal-len(probes))
    return summary,rows


def watchdog_deadline(frames,commands,lease_s):
    """The held command can expire before the newer pending RGB lease.

    Receipt records are taken just after accept(); retain the original .01s
    timestamp tolerance and .20s poll budget, not a larger timing allowance.
    """
    if not number(lease_s) or lease_s!=.8:raise ValueError('Actuator lease differs from restricted .8s contract')
    primary=[f for f in frames if not f['terminal']]
    accepted=[r for r in commands if not r['already_latched'] and r['reason']=='accepted']
    if not primary or not accepted:raise ValueError('Missing pending/held watchdog evidence')
    pending=primary[-1]['lease_deadline_steady_s'];held=accepted[-1]['steady_s']+lease_s
    if not number(pending) or not number(held):raise ValueError('Invalid watchdog deadline evidence')
    return dict(pending_frame_deadline_s=pending,held_velocity_deadline_s=held,effective_deadline_s=min(pending,held))


def assess_case(case,base):
    import numpy as np
    from simulation.continuous_straight_acceptance import audit_safety_physics,assess_route_case
    base=Path(base);replay,rows=replay_navigation(base);w=read(base/'world/result.json');n=read(base/'navigation/result.json')
    errors=[];task=read(base/'public/task.json');c=n['controller_result']
    if read(base/'fixture/scenario.json')!=case or read(base/'public/topology.json')!=case['graph'] or any(task[k]!=case[k] for k in ('start_node','goal_node','heading_deg')) or task['operator_confirmed'] is not True:raise ValueError('Run differs from declared S7 case')
    processes=[read(base/role/'process.json') for role in ('world','navigation','feedback')]
    if len({p['pid'] for p in processes})!=3:errors.append('not_three_distinct_ros_processes')
    if read(base/'orchestration.json')['exit_code']!=0:errors.append('ros_launch_failed')
    if not w['gate_latched'] or len(w['terminal_probes'])!=3 or not all(p['processed'] for p in w['terminal_probes']):errors.append('terminal_probe_transport_incomplete')
    frames=read(base/'public/frames.json');inventory={f['frame_id']:f for f in frames}
    if {p['frame_id'] for p in w['terminal_probes']}!={f['frame_id'] for f in frames if f['terminal']}:errors.append('terminal_probe_identity_mismatch')
    for row in rows:
        ref=row['input']
        if ref is not None:
            f=inventory.get(ref['frame_id'])
            if f is None or f['suppressed'] or any(f[k]!=ref[k] for k in ('frame_id','captured_sim_s','rgb_sha256')):errors.append('received_image_not_bound_to_camera_publication')
    commands=lines(base/'world/commands.jsonl');nav_commands=[e['command'] for e in lines(base/'navigation/events.jsonl') if 'command' in e]
    for record in commands:
        if not record['already_latched'] and record['reason']=='accepted':
            validate_command(record['command'])
            if record['command'] not in nav_commands:errors.append('motor_command_without_public_navigation_decision')
        if record['already_latched'] and (record['applied_v_m_s'] or record['applied_omega_rad_s']):errors.append('motor_restarted_after_latch')
    for e in lines(base/'navigation/events.jsonl'):
        if e['kind']=='feedback':
            f=e['feedback']
            bound=[r for r in commands if not r['already_latched'] and r['reason']=='accepted' and r['command']['seq']==f['applied_seq']]
            if len(bound)!=1 or bound[0]['command']['segment_event']!=f['event'] or bound[0]['sim_s']!=f['applied_sim_s']:errors.append('feedback_not_bound_to_motor_zero_receipt')
    audit=w['physics_audit'];result=dict(controller_result=c,audit=audit,exception=None)
    applied=read(base/'world/applied-rows.json');physics=audit_safety_physics(result,applied,read(base/'world/audit/physics-trace.json'));errors.extend(physics['errors'])
    audit=dict(audit,safe_final_stop=not physics['errors'],physics_audit_dt_s=1/120,braking_episodes=physics['braking'],displacement_m=float(np.linalg.norm(np.array(audit['final']['position_m'])[:2]-np.array(audit['initial']['position_m'])[:2])))
    route_case=dict(case,expectation='normal' if case['fault']=='none' else 's7_fault')
    route=assess_route_case(route_case,dict(controller_result=c,audit=audit,exception=None),rows,read(base/'world/audit/truth.json'));errors.extend(route['errors'])
    normal=case['fault']=='none';completed=route['functional_completed']
    if normal and (not completed or w['gate_reason']!='remote_visual_stop' or c['transition_count']!=2):errors.append('normal_task_not_completed')
    if not normal:
        if c['state']!='aborted' or c['node_estimate'] is not None or c['arrival_status']!='unverified':errors.append('fault_false_arrival_or_not_aborted')
        if not c['motion_commands'] or c['command_count']<1:errors.append('fault_did_not_exercise_partial_route')
        fault=case['fault']
        if fault!='feedback_silence':
            yaw_delta=abs((audit['final']['yaw_deg']-case['heading_deg']+180)%360-180)
            if not 5.<yaw_delta<85.:errors.append('fault_not_during_a_real_partial_turn')
        if fault in ('camera_silence','future_camera'):
            found=[f for f in frames if f['tick']==case['fault_tick'] and not f['terminal']]
            if len(found)!=1 or (fault=='camera_silence' and not found[0]['suppressed']):errors.append('camera_fault_not_injected')
            if fault=='future_camera' and abs(found[0]['captured_sim_s']-(audit['initial']['sim_s']+.1*case['fault_tick']))<.5:errors.append('future_timestamp_not_injected')
        if fault=='navigation_hang':
            signals=read(base/'orchestration.json')['signals']
            if [s['signal'] for s in signals]!=['SIGSTOP','SIGCONT'] or w['gate_reason']!='command_lease_expired':errors.append('independent_hung_navigation_watchdog_not_exercised')
            stops=[r for r in applied if r['source']=='gate_stop']
            deadline=watchdog_deadline(frames,commands,read(base/'world/process.json')['lease_wall_s'])
            if not stops or not -.01<=stops[0]['steady_s']-deadline['effective_deadline_s']<=.20:errors.append('steady_watchdog_deadline_missed')
        if fault=='feedback_silence':
            feedback=lines(base/'feedback/events.jsonl')
            if not feedback or not feedback[0]['dropped'] or n['bridge_reason']!='feedback_wall_timeout':errors.append('feedback_silence_not_exercised')
            if w['gate_reason']!='remote_abort':errors.append('feedback_timeout_abort_wire_not_accepted')
        if fault in ('replay_command','bad_version'):
            injected=read(base/'fault/injected.json')
            if injected['mode']!=fault or not any(r['command']==injected['command'] and r['reason'] in ('replayed_or_out_of_order_command','command_without_frame_lease','invalid_command_envelope') for r in commands):errors.append('real_dds_bad_command_not_rejected')
    return dict(case=case['id'],fault=case['fault'],errors=sorted(set(errors)),functional_completed=completed,gate_reason=w['gate_reason'],controller_reason=c['reason'],bridge_reason=n['bridge_reason'],replay=replay,physics=physics,
                maximum_node_proof_distance_m=route['maximum_node_proof_distance_m'],elapsed_wall_s=read(base/'orchestration.json')['elapsed_wall_s'])


def run_suite(suite_path,freeze,output):
    from simulation.rgb_observability_run import verify_freeze
    suite_path=Path(suite_path).resolve();suite=load_suite(suite_path);lock=verify_freeze(freeze);out=Path(output).resolve()
    if not lock.get('frozen_before_generation') or str(suite_path.relative_to(ROOT)) not in lock['files']:raise ValueError('S7 frozen source/suite required')
    if out.exists():raise FileExistsError('Fresh S7 batch output required')
    digest=hashlib.sha256(suite_path.read_bytes()).hexdigest()
    write(Path(freeze).parent/'FIRST_S7_GENERATION_STARTED.json',dict(suite_sha256=digest,output=str(out),started_wall_s=time.time()))
    out.mkdir();finished=[];failures=[]
    for case in suite['cases']:
        verify_freeze(freeze)
        try:run_case(case,out/case['id'])
        except Exception:failures.append(dict(case=case['id'],exception=traceback.format_exc()))
        finished.append(case['id']);print('S7_CASE_COMPLETE',case['id'],flush=True)
    verify_freeze(freeze);write(out/'phase-complete.json',dict(cases=finished,suite_sha256=digest,infrastructure_errors=failures))


def evaluate(suite_path,freeze,primary,output):
    from simulation.rgb_observability_run import verify_freeze
    suite=load_suite(suite_path);lock=verify_freeze(freeze);root=Path(primary).resolve()
    if not lock.get('frozen_before_generation') or str(Path(suite_path).resolve().relative_to(ROOT)) not in lock['files']:raise ValueError('S7 bound freeze required')
    digest=hashlib.sha256(Path(suite_path).read_bytes()).hexdigest();receipt=read(Path(freeze).parent/'FIRST_S7_GENERATION_STARTED.json');phase=read(root/'phase-complete.json')
    if receipt['suite_sha256']!=digest or receipt['output']!=str(root) or phase['suite_sha256']!=digest or phase['cases']!=[c['id'] for c in suite['cases']]:raise ValueError('Bound full first-generation S7 phase required')
    assessment=[];failures=list(phase['infrastructure_errors']);sessions=set()
    for case in suite['cases']:
        try:
            session=read(root/case['id']/'public/task.json')['session_id']
            if session in sessions:raise ValueError('Reused session')
            sessions.add(session);assessment.append(assess_case(case,root/case['id']))
        except Exception:failures.append(dict(case=case['id'],exception=traceback.format_exc()))
    verify_freeze(freeze)
    normals=[a for a in assessment if a['fault']=='none']
    passed=not failures and len(assessment)==8 and all(not a['errors'] for a in assessment) and len(normals)==2 and all(a['functional_completed'] for a in normals)
    report=dict(stage='S7',passed=passed,assessment=assessment,infrastructure_errors=failures,normal_completion=dict(completed=sum(a['functional_completed'] for a in normals),total=2),
                safety_conditions=dict(passed=sum(not a['errors'] for a in assessment if a['fault']!='none'),total=6),whole_block6_complete=False,
                limitations=['S6 formal67 and S2 repeated-texture boundary remain unaccepted','accelerated lock-step PyBullet simulation, not wall-time physical/hardware watchdog guarantee','world/OS/actuator-process hang not covered; no real robot','restricted static calibrated blue-grey-floor orthogonal reference-map domain'])
    write(output,report);return report


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('phase',choices=['validate','prepare','run-case','assess-case','control','evaluate'])
    p.add_argument('--suite',type=Path);p.add_argument('--case',type=Path);p.add_argument('--freeze',type=Path);p.add_argument('--output',type=Path);p.add_argument('--primary',type=Path)
    a=p.parse_args(argv)
    if a.phase=='validate':print(json.dumps(dict(cases=len(load_suite(a.suite)['cases']),generated=False)));return 0
    if a.phase=='prepare':prepare(read(a.case),a.output)
    elif a.phase=='run-case':run_case(read(a.case),a.output)
    elif a.phase=='assess-case':
        result=assess_case(read(a.case),a.primary);write(a.output,result);print(encode(result));return 1 if result['errors'] else 0
    elif a.phase=='control':run_suite(a.suite,a.freeze,a.output)
    else:
        result=evaluate(a.suite,a.freeze,a.primary,a.output);print(encode(dict(passed=result['passed'],normal_completion=result['normal_completion'],safety_conditions=result['safety_conditions'])));return 0 if result['passed'] else 1
    return 0


if __name__=='__main__':raise SystemExit(main())
