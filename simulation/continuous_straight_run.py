"""Actual wheel-driven new RGB straight controller + one-way post-run audit.

No truth fields enter the controller. A confirmed .30m initial swept-path
clearance is an explicit task/installation prerequisite, never a map distance.
Simulation-clock synchronous loop; wall compute logs are NOT hard realtime.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import time
import traceback
import uuid
import cv2
import numpy as np
from PIL import Image

from roadvision.continuous_straight import StraightController,CornerController,ContinuousRouteController,InitialClearancePrior
from simulation.continuous_audit_world import ContinuousAuditWorld,CornerAuditWorld,RouteAuditWorld
from simulation.corridor_motion import CorridorConfig
from roadvision.corridor_rgb_probe import CALIBRATION_ID


def run_case(output,*,scene=None,appearance=None,seed=313,pattern='unique',max_ticks=400,operator_confirmed=False,fault=None,narrow_obstacle_width_m=.5,corner_direction=None,corner_geometry=None,layout_direction=None,junction='l',blocked_branch=None,clear_blocker_tick=None,route_graph=None,route_scene=None,start_node=None,goal_node=None,heading_deg=None,physical_start_node=None,physical_heading_deg=None,transport_fault=None,terminal_check_ticks=0,motion_permission='no_motion',straight_variant='baseline'):
    declared_arguments=locals().copy();declared_arguments.pop('output')
    if declared_arguments['scene'] is not None:declared_arguments['scene']=asdict(declared_arguments['scene'])
    if type(max_ticks) is not int or max_ticks<1:raise ValueError('Expected a positive integer tick budget')
    if type(terminal_check_ticks) is not int or not 0<=terminal_check_ticks<=10:raise ValueError('Invalid terminal verification budget')
    if fault is not None and (not isinstance(fault,dict) or set(fault)!={'kind','tick'} or fault['kind'] not in ('missing','replay','stale','occlusion','future','out_of_order','silence','blank','back_occlusion','bad_envelope','freeze','back_freeze') or type(fault['tick']) is not int or not 0<=fault['tick']<max_ticks or (fault['kind'] in ('replay','out_of_order','freeze','back_freeze') and fault['tick']==0)):raise ValueError('Unsupported or unreachable declared input fault')
    if corner_direction not in (None,'left','right'):raise ValueError('Expected a planned corner direction')
    if layout_direction not in (None,'left','right') or (layout_direction is not None and corner_direction is None):raise ValueError('Invalid fixture-only layout direction')
    if corner_direction is not None and scene is not None:raise ValueError('Use explicit corner geometry, not straight scene configuration')
    if corner_geometry is not None and (corner_direction is None or not isinstance(corner_geometry,dict) or set(corner_geometry)-{'width_m','corner_x_m','exit_length_m'}):raise ValueError('Invalid corner setup geometry')
    if junction not in ('l','t') or (junction=='t' and (corner_direction is None or layout_direction is not None)):raise ValueError('T junction needs a left/right task, not a fixture direction')
    if blocked_branch not in (None,'left','right') or (blocked_branch is not None and junction!='t'):raise ValueError('Invalid T branch blocker')
    if clear_blocker_tick is not None and (blocked_branch is None or type(clear_blocker_tick) is not int or not 0<=clear_blocker_tick<max_ticks):raise ValueError('Invalid fixture-only clearing event')
    route_mode=route_graph is not None or route_scene is not None
    if route_mode:
        from simulation.topology import validate_pair,task_values
        if route_graph is None or route_scene is None:raise ValueError('Route needs independent public graph and physical fixture')
        if scene is not None or corner_direction is not None or corner_geometry is not None or junction!='l' or appearance is not None or blocked_branch is not None:raise ValueError('Do not mix route and primitive configurations')
        validate_pair(route_graph,route_scene)
        task_start,task_goal=task_values(route_graph);task_heading=route_graph.get('task',{}).get('initial_heading_deg')
        start_node=task_start if start_node is None else start_node;goal_node=task_goal if goal_node is None else goal_node
        heading_deg=task_heading if heading_deg is None else heading_deg
        ids={n['node_id'] for n in route_graph['nodes']}
        if start_node not in ids or goal_node not in ids or start_node==goal_node or type(heading_deg) not in (int,float) or heading_deg not in (0,90,180,270):raise ValueError('Invalid route task')
        physical_start_node=start_node if physical_start_node is None else physical_start_node
        physical_heading_deg=heading_deg if physical_heading_deg is None else physical_heading_deg
        if physical_start_node not in ids or type(physical_heading_deg) not in (int,float) or physical_heading_deg not in (0,90,180,270):raise ValueError('Invalid physical spawn override')
    elif any(v is not None for v in (start_node,goal_node,heading_deg,physical_start_node,physical_heading_deg,transport_fault)):
        raise ValueError('Route-only arguments require a route')
    if transport_fault is not None and (not isinstance(transport_fault,dict) or set(transport_fault)!={'kind','segment'} or transport_fault['kind'] not in ('duplicate','drop','stale','unknown_command','conflict') or type(transport_fault['segment']) is not int or not 1<=transport_fault['segment']<=12):raise ValueError('Invalid declared feedback injection')
    if straight_variant not in ('baseline','history','control','combined','swept','rear','rear_active','reference','wide_reference','gradient_transfer','gradient_history'):raise ValueError('Unknown straight variant')
    if (route_mode or corner_direction is not None) and straight_variant!='baseline':raise ValueError('Straight-only ablation cannot change route/corner control')
    if motion_permission not in ('no_motion','bounded_probe'):raise ValueError('Invalid straight motion permission')
    if (route_mode or corner_direction is not None) and motion_permission!='no_motion':raise ValueError('Straight-only probe authorization cannot arm route/corner control')
    out=Path(output);(out/'public/frames').mkdir(parents=True,exist_ok=False);(out/'audit').mkdir()
    scene=scene or CorridorConfig()
    prior=InitialClearancePrior(confirmed=operator_confirmed)
    if not route_mode and corner_direction is None:
        task=dict(version='straight_motion_permission_v1',motion_permission=motion_permission,initial_clearance=asdict(prior),calibration_id=CALIBRATION_ID)
        if straight_variant!='baseline':task['straight_variant']=straight_variant
        if straight_variant=='swept':
            from roadvision.swept_ground_path import task_contract
            task['trajectory_contract']=task_contract()
        if straight_variant in ('rear','rear_active'):
            from roadvision.rear_observation import task_contract as rear_contract
            from roadvision.swept_ground_path import task_contract as sweep_contract
            task['rear_contract']=rear_contract(straight_variant=='rear_active')
            task['trajectory_contract']=sweep_contract()
        (out/'public/task.json').write_text(json.dumps(task,indent=2)+'\n')
    controller=None if route_mode else StraightController(initial_clearance=prior,motion_permission=motion_permission) if corner_direction is None else CornerController(direction=corner_direction,initial_clearance=prior,junction=junction)
    if straight_variant in ('history','control','combined'):
        from roadvision.straight_ablation import AblationStraightController
        controller=AblationStraightController(variant=straight_variant,initial_clearance=prior,motion_permission=motion_permission)
    if straight_variant=='swept':
        from roadvision.swept_straight import SweptStraightController
        controller=SweptStraightController(initial_clearance=prior,motion_permission=motion_permission)
    if straight_variant in ('rear','rear_active'):
        from roadvision.rear_straight import RearStraightController
        controller=RearStraightController(active_review=straight_variant=='rear_active',initial_clearance=prior,motion_permission=motion_permission)
    if straight_variant=='gradient_transfer':
        from roadvision.gradient_straight import GradientTransferStraightController
        controller=GradientTransferStraightController(initial_clearance=prior,motion_permission=motion_permission)
    if straight_variant=='gradient_history':
        from roadvision.gradient_history_straight import GradientHistoryStraightController
        controller=GradientHistoryStraightController(initial_clearance=prior,motion_permission=motion_permission)
    if straight_variant=='wide_reference':
        from roadvision.wide_reference_straight import WideReferenceStraightController
        controller=WideReferenceStraightController(initial_clearance=prior,motion_permission=motion_permission)
    if straight_variant=='reference':
        from roadvision.reference_straight import ReferenceStraightController
        controller=ReferenceStraightController(initial_clearance=prior,motion_permission=motion_permission)
    feedback_rows=[];delivered=set();reference_manifest=[];terminal_rows=[];caught_input_errors=[]
    session_id="continuous-route:"+uuid.uuid4().hex if route_mode else None
    refs=[];truth=[];rows=[];fixture_events=[];previous=None;frozen_rgb=None;exception=None;started=time.perf_counter();cv2.setNumThreads(1)
    fixture=(RouteAuditWorld(physical_scene=route_scene,assets=out/'audit/assets',seed=seed,spawn_node=physical_start_node,spawn_heading_deg=physical_heading_deg,pattern=pattern) if route_mode else ContinuousAuditWorld(scene,appearance,assets=out/'audit/assets',seed=seed,pattern=pattern,narrow_obstacle_width_m=narrow_obstacle_width_m) if corner_direction is None else
             CornerAuditWorld(assets=out/'audit/assets',direction=layout_direction or corner_direction,appearance=appearance,seed=seed,pattern=pattern,junction=junction,blocked_branch=blocked_branch,**(corner_geometry or {})))
    with fixture as world, (out/'decisions.live.jsonl').open('x') as live, (out/'audit/truth.live.jsonl').open('x') as truth_live:
        if route_mode:
            from simulation.path_planning import plan_from_estimate,next_action
            (out/'public/references').mkdir()
            references=[]
            for i,ref in enumerate(world.reference_atlas()):
                relative=f'references/{i:03d}.png';Image.fromarray(ref['rgb']).save(out/'public'/relative)
                # Decode the exact public assets the controller and replayer use.
                pixels=np.array(Image.open(out/'public'/relative))
                references.append(dict(node_id=ref['node_id'],heading_deg=ref['heading_deg'],rgb=pixels))
                reference_manifest.append(dict(node_id=ref['node_id'],heading_deg=ref['heading_deg'],rgb_path=relative,rgb_sha256=hashlib.sha256(pixels.tobytes()).hexdigest()))
            (out/'public/references.json').write_text(json.dumps(reference_manifest,indent=2)+'\n')
            (out/'public/topology.json').write_text(json.dumps(route_graph,indent=2)+'\n')
            (out/'public/task.json').write_text(json.dumps(dict(start_node=start_node,goal_node=goal_node,heading_deg=heading_deg,session_id=session_id),indent=2)+'\n')
            controller=ContinuousRouteController(session_id=session_id,graph=route_graph,references=references,plan_from_estimate=plan_from_estimate,next_action=next_action,start=start_node,goal=goal_node,heading=heading_deg,initial_clearance=prior)
        initial=world.audit_state()
        try:
            for tick in range(max_ticks):
                if clear_blocker_tick==tick:
                    world.clear_branch_blocker();fixture_events.append(dict(tick=tick,sim_s=world.sim_s,event='branch_blocker_removed'))
                now_s=world.sim_s;frame=None;injected=None
                # Scheduler still runs on missing camera delivery; no fake RGB
                # capture is created for a silent producer. Not a wall-time
                # hardware watchdog or a claim about a hung process.
                if fault is None or tick!=fault['tick'] or fault['kind']!='silence':
                    raw=world.capture();frame={**raw,'rgb':raw['rgb'].copy()}
                if fault is not None and (tick==fault['tick'] or fault['kind'] in ('freeze','back_freeze') and tick>=fault['tick']):
                    injected=fault['kind']
                    if injected in ('missing','silence'):frame=None
                    elif injected=='replay' and previous is not None:frame={**previous,'rgb':previous['rgb'].copy()}
                    elif injected=='stale':frame['captured_sim_s']-=.4
                    elif injected=='occlusion':frame['rgb'][:128,:128]=0
                    elif injected=='back_occlusion':frame['rgb'][128:,128:]=0
                    elif injected=='blank':frame['rgb'][:]=128
                    elif injected=='future':frame['captured_sim_s']+=.1
                    elif injected=='out_of_order':frame['captured_sim_s']=previous['captured_sim_s']-.01
                    elif injected=='bad_envelope':frame['unexpected_field']='not_a_public_rgb_field'
                    elif injected=='back_freeze':
                        if frozen_rgb is None:frozen_rgb=previous['rgb'].copy()
                        frame['rgb'][128:,128:]=frozen_rgb[128:,128:]
                    elif injected=='freeze':
                        # Frozen sensor payload with new delivery IDs/times:
                        # unlike replay, metadata checks alone cannot catch it.
                        if frozen_rgb is None:frozen_rgb=previous['rgb'].copy()
                        frame['rgb']=frozen_rgb.copy()
                elapsed=time.perf_counter()
                input_error=None
                try:
                    decision=controller.missing(now_s) if frame is None else controller.update(frame,now_s)
                except Exception as error:
                    # update() latches an abort before raising. Always submit a
                    # physical ZERO now, and durably record the exact failure.
                    world.drive(0.,0.)
                    decision=controller.missing(now_s)
                    input_error=dict(type=type(error).__name__,message=str(error))
                    caught_input_errors.append(dict(tick=tick,**input_error))
                segment_output=None;deliveries=[]
                if route_mode and decision['pending_feedback'] is not None:
                    event=dict(decision['pending_feedback']);identity=event['event_id']
                    if identity not in delivered:
                        delivered.add(identity);segment_output=decision
                        kind=transport_fault['kind'] if transport_fault is not None and len(delivered)==transport_fault['segment'] else None
                        events=[event]
                        if kind=='drop':events=[]
                        elif kind=='duplicate':events=[event,dict(event)]
                        elif kind=='stale':events=[dict(event,captured_sim_s=event['captured_sim_s']-.4)]
                        elif kind=='unknown_command':events=[dict(event,command_id='unknown-command')]
                        elif kind=='conflict':events=[event,dict(event,status='conflicting')]
                        for value in events:
                            decision=controller.feedback(value,now_s)
                            record=dict(tick=tick,sim_s=now_s,event=value,decision=decision)
                            deliveries.append(record);feedback_rows.append(record)
                        if not events:feedback_rows.append(dict(tick=tick,sim_s=now_s,event=None,injection='drop',bound_event=event))
                elapsed=time.perf_counter()-elapsed
                if frame is not None:
                    relative=f'frames/{tick:04d}.png';Image.fromarray(frame['rgb']).save(out/'public'/relative)
                    reference={k:v for k,v in frame.items() if k!='rgb'}|dict(rgb_path=relative,rgb_sha256=hashlib.sha256(frame['rgb'].tobytes()).hexdigest())
                    refs.append(reference);previous=frame
                else:reference=None
                row=dict(tick=tick,sim_s=now_s,input=reference,injected_fault=injected,compute_wall_s=elapsed,decision=decision)
                if route_mode:row.update(segment_output=segment_output,feedback_deliveries=deliveries)
                if input_error is not None:row['input_error']=input_error
                rows.append(row)
                # Durable public decision trail even if a run is interrupted.
                # It is output-only, never read to select future commands.
                live.write(json.dumps(row,allow_nan=False,separators=(',',':'))+'\n');live.flush()
                world.drive(decision['v_m_s'],decision['omega_rad_s']);world.step_control()
                # Audit collection is one-way and never branches control.
                truth.append(world.audit_state())
                truth_live.write(json.dumps(truth[-1],allow_nan=False)+'\n');truth_live.flush()
                if tick%10==0 or decision['state'] in ('visual_stop','aborted'):
                    print(tick,decision['state'],decision['reason'],round(decision['v_m_s'],4),round(decision['omega_rad_s'],4),flush=True)
                if decision['state'] in ('visual_stop','aborted'):break
            else:
                decision=controller.cancel(world.sim_s)
                row=dict(tick=max_ticks,sim_s=world.sim_s,input=None,injected_fault=None,compute_wall_s=0.,decision=decision,external_event='tick_budget_exhausted')
                rows.append(row);live.write(json.dumps(row,allow_nan=False,separators=(',',':'))+'\n');live.flush()
        except Exception:
            exception=traceback.format_exc();world.drive(0.,0.)
            decision=controller.missing(world.sim_s)
            row=dict(tick=tick,sim_s=world.sim_s,input=None,injected_fault=None,compute_wall_s=0.,decision=decision,external_event='runner_exception_zero_stop')
            rows.append(row);live.write(json.dumps(row,allow_nan=False,separators=(',',':'))+'\n');live.flush()
        finally:
            world.drive(0.,0.)
            for _ in range(10):world.step_control();truth.append(world.audit_state())
        # Probe terminal latching with REAL later good captures; zero motors
        # stay zero independently even if a bug returns a nonzero decision.
        terminal_result=controller.result()
        for index in range(terminal_check_ticks):
            f=world.capture();d=controller.update(f,world.sim_s)
            relative=f'frames/terminal-{index:02d}.png';Image.fromarray(f['rgb']).save(out/'public'/relative)
            ref={k:v for k,v in f.items() if k!='rgb'}|dict(rgb_path=relative,rgb_sha256=hashlib.sha256(f['rgb'].tobytes()).hexdigest())
            terminal_rows.append(dict(input=ref,sim_s=world.sim_s,decision=d,result_unchanged=controller.result()==terminal_result))
            world.drive(0.,0.);world.step_control();truth.append(world.audit_state())
        final=world.audit_state();contacts=world.wall_contact_physics_steps
        physics_trace=list(world.physics_trace)
    # Decision loop finished before reading/comparing any collected poses.
    displacement=float(np.linalg.norm(np.array(final['position_m'])[:2]-np.array(initial['position_m'])[:2]))
    end_gap=scene.length_m-final['camera_position_m'][0];stationary=final['speed_m_s']<.01 and abs(final['yaw_rate_rad_s'])<.02
    geometric_stop=(controller.state=='visual_stop' and scene.obstacle=='none' and .8<=end_gap<=1.7 and abs(final['position_m'][1])<.2 and abs(final['yaw_deg'])<6 and stationary and contacts==0)
    physical_audit=dict(initial=initial,final=final,displacement_m=displacement,wall_contact_physics_steps=contacts,
        stationary=stationary,end_camera_gap_m=end_gap,normal_geometric_stop_pass=geometric_stop,
        node_arrival_status='unverified',safe_final_stop=stationary and contacts==0,
        scope='post-run only; physical stop/end location are not controller node identity')
    if corner_direction is not None:
        geometry=world.corner_geometry;sign=1 if corner_direction=='left' else -1
        end_gap=geometry['exit_length_m']-sign*final['camera_position_m'][1]
        heading_error=abs((final['yaw_deg']-sign*90.+180.)%360.-180.)
        geometric_stop=(controller.state=='visual_stop' and sign*final['position_m'][1]>geometry['width_m']/2+.20 and
                        abs(final['position_m'][0]-geometry['corner_x_m'])<.20 and heading_error<6. and
                        stationary and contacts==0)
        physical_audit.update(normal_geometric_stop_pass=geometric_stop,corner_geometric_exit_pass=geometric_stop,
            end_camera_gap_m=end_gap,outgoing_heading_error_deg=heading_error,
            exit_body_progress_m=sign*final['position_m'][1],lateral_exit_error_m=final['position_m'][0]-geometry['corner_x_m'])
    if route_mode:
        confirmations=[]
        # This post-run comparison never affects control or topology promotion.
        for proof in controller.result()['node_confirmations']:
            sample=min([initial,*truth],key=lambda t:abs(t['sim_s']-proof['captured_sim_s']))
            centre=np.array(world.node_positions[proof['node_id']],float)
            distance=float(np.linalg.norm(np.array(sample['position_m'])[:2]-centre))
            angle=abs((sample['yaw_deg']-proof['heading_deg']+180)%360-180)
            ok=distance<.20 and angle<6 and sample['speed_m_s']<.01 and abs(sample['yaw_rate_rad_s'])<.02
            confirmations.append(dict(node_id=proof['node_id'],captured_sim_s=proof['captured_sim_s'],sample_sim_s=sample['sim_s'],body_distance_m=distance,heading_error_deg=angle,passed=ok))
        goal_distance=float(np.linalg.norm(np.array(final['position_m'])[:2]-np.array(world.node_positions[goal_node])))
        goal_heading_error=abs((final['yaw_deg']-controller.heading+180)%360-180)
        geometric_stop=controller.state=='visual_stop' and goal_distance<.20 and goal_heading_error<6 and stationary and contacts==0 and bool(confirmations) and all(c['passed'] for c in confirmations)
        physical_audit.pop('end_camera_gap_m',None) # no fictitious straight-end distance for a route
        physical_audit['scope']='post-run station/heading/stop audit only; no physical values entered the controller'
        physical_audit.update(normal_geometric_stop_pass=geometric_stop,route_geometric_goal_pass=geometric_stop,
            goal_body_distance_m=goal_distance,goal_heading_error_deg=goal_heading_error,node_confirmation_audit=confirmations,
            node_arrival_status='independently_audited_map_node' if geometric_stop else 'unverified')
    braking=[]
    for i,row in enumerate(rows):
        d=row['decision'];previous_decision=rows[i-1]['decision'] if i else None
        if not previous_decision or d['v_m_s'] or d['omega_rad_s'] or not (previous_decision['v_m_s'] or previous_decision['omega_rad_s']):continue
        end=next((rows[j]['sim_s'] for j in range(i+1,len(rows)) if rows[j]['decision']['v_m_s'] or rows[j]['decision']['omega_rad_s']),final['sim_s']+world.physics_dt)
        episode=[t for t in physics_trace if row['sim_s']<t['sim_s']<end]
        stopped=next((t for t in episode if t['speed_m_s']<.01 and abs(t['yaw_rate_rad_s'])<.02),None)
        before=next((t for t in reversed(physics_trace) if t['sim_s']<=row['sim_s']+1e-9),None)
        path=([before]+[t for t in episode if stopped is None or t['sim_s']<=stopped['sim_s']]) if before is not None else []
        arc=sum(float(np.linalg.norm(np.array(b['position_m'])[:2]-np.array(a['position_m'])[:2])) for a,b in zip(path,path[1:]))
        braking.append(dict(tick=row['tick'],requested_sim_s=row['sim_s'],reason=d['reason'],
            latency_s=stopped['sim_s']-row['sim_s'] if stopped is not None else None,
            distance_m=arc if stopped is not None and before is not None else None,
            stop_request_duration_s=end-row['sim_s'],resumed_before_stationary=stopped is None and end<final['sim_s'],
            distance_until_resume_or_end_m=arc))
    run_trace=[t for t in physics_trace if t['sim_s']>=initial['sim_s']]
    physical_audit.update(braking_episodes=braking,physics_audit_dt_s=world.physics_dt,
        maximum_audited_speed_m_s=max(t['speed_m_s'] for t in run_trace),
        maximum_audited_yaw_rate_rad_s=max(abs(t['yaw_rate_rad_s']) for t in run_trace),
        bootstrap_max_displacement_m=max((float(np.linalg.norm(np.array(t['position_m'])[:2]-np.array(initial['position_m'])[:2])) for t in run_trace if any(r['decision']['state']=='bootstrapping' and r['sim_s']<t['sim_s']<=r['sim_s']+world.control_dt+1e-9 for r in rows)),default=0.))
    result=dict(version='continuous_straight_run_v1' if corner_direction is None else 'continuous_corner_run_v1',scene=asdict(scene) if corner_direction is None else None,appearance=appearance,
        seed=seed,pattern=pattern,corner_direction=corner_direction,corner_geometry=world.corner_geometry if corner_direction is not None else None,operator_confirmed_initial_clearance=operator_confirmed,motion_permission=motion_permission,fault=fault,narrow_obstacle_width_m=narrow_obstacle_width_m,
        frame_count=len(refs),decision_count=len(rows),controller_result=controller.result(),audit=physical_audit,
        elapsed_wall_s=time.perf_counter()-started,exception=exception,clock_domain='synchronous simulation seconds; not wall-clock hard realtime')
    if junction=='t':
        result.update(version='continuous_t_junction_run_v1',junction='t',blocked_branch=blocked_branch,clear_blocker_tick=clear_blocker_tick)
        physical_audit['fixture_events']=fixture_events
    if route_mode:
        result.update(version='continuous_route_run_v1',session_id=session_id,scene=None,route_graph=route_graph,route_scene=route_scene,
            start_node=start_node,goal_node=goal_node,heading_deg=heading_deg,physical_start_node=physical_start_node,
            physical_heading_deg=physical_heading_deg,transport_fault=transport_fault,max_ticks=max_ticks)
        (out/'feedback.jsonl').write_text(''.join(json.dumps(r,allow_nan=False,separators=(',',':'))+'\n' for r in feedback_rows))
    if terminal_check_ticks or caught_input_errors:
        result['safety_checks']=dict(configuration=declared_arguments,terminal_good_frame_checks=len(terminal_rows),terminal_latched=all(r['result_unchanged'] and r['decision']['v_m_s']==r['decision']['omega_rad_s']==0. for r in terminal_rows),caught_input_errors=caught_input_errors)
        (out/'public/terminal-checks.json').write_text(json.dumps(terminal_rows,indent=2,allow_nan=False)+'\n')
    with (out/'decisions.jsonl').open('x') as f:
        for row in rows:f.write(json.dumps(row,allow_nan=False,separators=(',',':'))+'\n')
    (out/'public/sequence.json').write_text(json.dumps(dict(version='rgb_probe_sequence_v1',calibration_id=CALIBRATION_ID,frames=refs),indent=2)+'\n')
    (out/'audit/truth.json').write_text(json.dumps(truth,indent=2)+'\n');(out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    (out/'audit/physics-trace.json').write_text(json.dumps(physics_trace,separators=(',',':'))+'\n')
    print(json.dumps(result,indent=2),flush=True);return result


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True,type=Path);p.add_argument('--seed',type=int,default=313)
    p.add_argument('--length-m',type=float,default=4.);p.add_argument('--offset-m',type=float,default=0.);p.add_argument('--yaw-deg',type=float,default=0.)
    p.add_argument('--obstacle',choices=['none','narrow','full_width'],default='none');p.add_argument('--max-ticks',type=int,default=400)
    p.add_argument('--narrow-obstacle-width-m',type=float,default=.5)
    p.add_argument('--corner',choices=['left','right']);p.add_argument('--corner-width-m',type=float,default=1.8)
    p.add_argument('--junction',choices=['l','t'],default='l');p.add_argument('--blocked-branch',choices=['left','right']);p.add_argument('--clear-blocker-tick',type=int)
    p.add_argument('--corner-x-m',type=float,default=2.5);p.add_argument('--exit-length-m',type=float,default=3.5)
    p.add_argument('--route-map',type=Path);p.add_argument('--route-scene',type=Path);p.add_argument('--start-node');p.add_argument('--goal-node');p.add_argument('--heading-deg',type=int)
    p.add_argument('--straight-variant',choices=['baseline','history','control','combined','swept','rear','rear_active','reference','wide_reference','gradient_transfer','gradient_history'],default='baseline')
    p.add_argument('--motion-permission',choices=['no_motion','bounded_probe'],default='no_motion')
    p.add_argument('--confirm-initial-clearance',action='store_true');a=p.parse_args(argv)
    r=run_case(a.output,scene=None if a.corner or a.route_map else CorridorConfig(length_m=a.length_m,initial_offset_m=a.offset_m,initial_yaw_deg=a.yaw_deg,obstacle=a.obstacle),seed=a.seed,max_ticks=a.max_ticks,operator_confirmed=a.confirm_initial_clearance,motion_permission=a.motion_permission,straight_variant=a.straight_variant,narrow_obstacle_width_m=a.narrow_obstacle_width_m,corner_direction=a.corner,corner_geometry=dict(width_m=a.corner_width_m,corner_x_m=a.corner_x_m,exit_length_m=a.exit_length_m) if a.corner else None,junction=a.junction,blocked_branch=a.blocked_branch,clear_blocker_tick=a.clear_blocker_tick,route_graph=json.loads(a.route_map.read_text()) if a.route_map else None,route_scene=json.loads(a.route_scene.read_text()) if a.route_scene else None,start_node=a.start_node,goal_node=a.goal_node,heading_deg=a.heading_deg)
    return 0 if r['audit']['normal_geometric_stop_pass'] else 1


if __name__=='__main__':raise SystemExit(main())
