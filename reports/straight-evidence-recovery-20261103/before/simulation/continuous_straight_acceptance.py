"""Post-run S2 gates; audit never returns to a controller.

Coverage and safety have separate denominators. All-refusal does NOT pass
normal navigation. A physical surface stop is NEVER a topology-node claim.
The caller must freeze this evaluator/suite/source before fresh generation.
"""
import json
import math
from collections import Counter
from pathlib import Path
import hashlib
from simulation.rgb_observability_run import verify_freeze
from simulation.frozen_straight_comparison import compare_initial
import numpy as np
from PIL import Image


def verify_public_records(root,decisions):
    """Post-phase public PNG/hash and decision-input binding integrity."""
    public=Path(root)/'public';manifest=json.loads((public/'sequence.json').read_text())
    inputs=[r['input'] for r in decisions if r['input'] is not None]
    if manifest['frames']!=inputs:raise ValueError('Manifest does not match exact ordered decision inputs')
    for ref in inputs:
        path=(public/ref['rgb_path']).resolve()
        if Path(ref['rgb_path']).is_absolute() or not path.is_relative_to(public.resolve()):raise ValueError('RGB path outside public root')
        pixels=np.array(Image.open(path))
        if pixels.shape!=(256,256,3) or pixels.dtype!=np.uint8 or hashlib.sha256(pixels.tobytes()).hexdigest()!=ref['rgb_sha256']:raise ValueError('Saved RGB differs from decision input')
    return len(inputs)


def assess_case(case,result,decisions):
    errors=[];audit=result['audit'];feedback=result['controller_result'];kind=case['expectation']
    if result.get('exception') is not None:errors.append('execution_exception')
    if not decisions:errors.append('missing_decisions')
    if not audit['safe_final_stop'] or audit['wall_contact_physics_steps']!=0:errors.append('physical_stop_or_contact_failed')
    if audit.get('physics_audit_dt_s')!=1/120:errors.append('missing_physics_rate_speed_and_brake_audit')
    if audit['maximum_audited_speed_m_s']>.18:errors.append('speed_exceeds_declared_initial_clearance_domain')
    if audit['bootstrap_max_displacement_m']>.25:errors.append('bootstrap_consumes_initial_clearance_margin')
    for episode in audit['braking_episodes']:
        if episode['latency_s'] is not None:
            if episode['latency_s']>.30 or episode['distance_m'] is None or episode['distance_m']>.05:errors.append('braking_latency_or_distance_exceeds_domain')
        elif episode['stop_request_duration_s']>=.30 or not episode['resumed_before_stationary']:errors.append('sustained_zero_command_without_bounded_stop')
    if feedback['arrival_status']!='unverified' or feedback['atomic_edge_compatible'] is not False or audit['node_arrival_status']!='unverified':errors.append('false_node_or_atomic_claim')
    for row in decisions:
        d=row['decision']
        if d['arrival_status']!='unverified' or d['atomic_edge_compatible'] is not False:errors.append('false_intermediate_node_or_atomic_claim')
        v,w=d['v_m_s'],d['omega_rad_s']
        if not all(type(x) in (int,float) and math.isfinite(x) for x in (v,w)) or not 0<=v<=.15 or abs(w)>.200001:errors.append('unsafe_or_nonfinite_command')
        if (v or w) and (row['input'] is None or not 0<=row['sim_s']-row['input']['captured_sim_s']<=.25):errors.append('motion_without_fresh_input')
        if (v or w) and row['input'] is not None and (d['control_frame_id']!=row['input']['frame_id'] or d['captured_sim_s']!=row['input']['captured_sim_s'] or d['decision_sim_s']!=row['sim_s']):errors.append('motion_not_bound_to_current_rgb_and_clock')
    if kind=='fault_stop':
        fault=case['fault'];events=[r for r in decisions if r.get('injected_fault')==fault['kind'] and r['tick']==fault['tick']]
        if len(events)!=1:errors.append('fault_not_reached_and_injected')
        elif events[0]['decision']['v_m_s'] or events[0]['decision']['omega_rad_s']:errors.append('nonzero_command_at_fault')
        if feedback['state']!='aborted':errors.append('fault_did_not_abort')
        if fault['tick']>=20 and not feedback['motion_commands']:errors.append('late_fault_never_exercised_partial_motion')
    elif kind=='no_motion':
        if feedback['motion_commands'] or audit['displacement_m']>.001 or feedback['state']!='aborted':errors.append('unsupported_launch_moved')
    elif kind=='obstacle_stop':
        if feedback['state'] not in ('visual_stop','aborted'):errors.append('obstacle_run_did_not_terminate')
        if audit['normal_geometric_stop_pass']:errors.append('obstacle_miscounted_as_normal_endpoint')
    elif kind=='cancel_stop':
        if feedback['state']!='aborted' or feedback.get('reason')!='execution_cancelled' or not feedback['motion_commands']:errors.append('finite_budget_did_not_cancel_partial_motion')
        if not decisions or decisions[-1].get('external_event')!='tick_budget_exhausted' or decisions[-1]['decision']['v_m_s'] or decisions[-1]['decision']['omega_rad_s']:errors.append('missing_explicit_budget_zero_command_event')
    elif kind!='normal':raise ValueError('Unsupported expected outcome')
    normal_completed=(kind=='normal' and audit['normal_geometric_stop_pass'] and feedback['result_type']=='completed_visual_stop')
    return dict(case=case['id'],expectation=kind,errors=sorted(set(errors)),normal_completed=normal_completed,
        result_type=feedback['result_type'],physical_stop_safe=audit['safe_final_stop'])


def summarize_suite(cases,assessments):
    by_id={a['case']:a for a in assessments};errors=[]
    if len(by_id)!=len(assessments) or set(by_id)!={c['id'] for c in cases}:errors.append('missing_or_duplicate_case_results')
    groups=Counter(c['expectation'] for c in cases)
    for kind in ('normal','obstacle_stop','fault_stop','no_motion','cancel_stop'):
        if not groups[kind]:errors.append('missing_required_denominator_'+kind)
    normal=[a for a in assessments if a['expectation']=='normal'];success=sum(a['normal_completed'] for a in normal)
    if not normal or success<math.ceil(.8*len(normal)):errors.append('normal_completion_below80percent')
    # Offset and yaw cannot be hidden in a center-only success denominator.
    for family in ('offset','yaw'):
        ids={c['id'] for c in cases if c['expectation']=='normal' and family in c.get('tags',[])}
        if not ids or not any(by_id.get(i,{}).get('normal_completed') for i in ids):errors.append('no_successful_'+family+'_correction')
    failures=[a for a in assessments if a['errors']]
    return dict(stage='S2',passed=not errors and not failures,failures=failures,suite_errors=errors,
        normal_completion=dict(completed=success,total=len(normal)),expectation_denominators=dict(groups),
        assessment=assessments,node_arrival_status='unverified',
        scope='restricted calibrated static textured parallel corridors with explicit operator initial clearance; not node navigation or certified real-car safety')


def evaluate(suite_path,runs,frozen,freeze,output):
    verify_freeze(freeze)
    suite=json.loads(Path(suite_path).read_text());root=Path(runs)
    digest=hashlib.sha256(Path(suite_path).read_bytes()).hexdigest()
    for phase,method in ((root,'S2_control'),(Path(frozen),'frozen_colour_control')):
        marker=json.loads((phase/'phase-complete.json').read_text())
        if marker['method']!=method or marker['suite_sha256']!=digest or marker['cases']!=[c['id'] for c in suite['cases']]:raise ValueError('Both complete bound phases required before final suite audit')
    out=Path(output);out.parent.mkdir(parents=True,exist_ok=True)
    assessments=[]
    comparison=[]
    for case in suite['cases']:
        p=root/case['id'];result=json.loads((p/'result.json').read_text());decisions=[json.loads(s) for s in (p/'decisions.jsonl').read_text().splitlines()]
        verify_public_records(p,decisions)
        assessments.append(assess_case(case,result,decisions))
        comparison.append(compare_initial(p,Path(frozen)/case['id'],out.parent/(case['id']+'-comparison.json')))
    result=summarize_suite(suite['cases'],assessments)
    result['frozen_colour_comparison']=comparison
    with out.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    return result


def evaluate_corner_suite(suite_path,runs,freeze,output):
    """S3 post-only assessment. No audit value is returned to any controller."""
    lock=verify_freeze(freeze)
    from simulation.rgb_observability_run import ROOT
    relative=str(Path(suite_path).resolve().relative_to(ROOT))
    if not lock.get('frozen_before_generation') or relative not in lock['files']:
        raise ValueError('Suite and source must be bound before first generation')
    suite=json.loads(Path(suite_path).read_text());root=Path(runs)
    if suite['version']!='single_corner_s3_v1' or len(suite['cases'])!=20:
        raise ValueError('Expected the predeclared20-case S3 suite')
    ids=[c['id'] for c in suite['cases']]
    if len(set(ids))!=len(ids):raise ValueError('Duplicate case ID')
    marker=json.loads((root/'phase-complete.json').read_text())
    digest=hashlib.sha256(Path(suite_path).read_bytes()).hexdigest()
    if marker['cases']!=ids or marker['suite_sha256']!=digest:
        raise ValueError('Complete bound control phase required')
    assessments=[]
    for c in suite['cases']:
        base=root/c['id'];result=json.loads((base/'result.json').read_text())
        decisions=[json.loads(s) for s in (base/'decisions.jsonl').read_text().splitlines()]
        expected_geometry={**c['geometry'],'direction':c.get('layout_direction') or c['direction']}
        if (result['seed']!=c['seed'] or result['corner_direction']!=c['direction'] or
            result['corner_geometry']!=expected_geometry or result['fault']!=c.get('fault') or
            result['operator_confirmed_initial_clearance'] is not c['operator_confirmed'] or result['pattern']!='unique'):
            raise ValueError('Run setup differs from predeclared case')
        verify_public_records(base,decisions)
        assessments.append(assess_corner_case(c,result,decisions))
    normals=[a for a in assessments if a['expectation']=='normal']
    groups=Counter(c['expectation'] for c in suite['cases'])
    errors=[]
    if dict(groups)!=dict(normal=8,fault_stop=8,wrong_branch=2,no_motion=2):errors.append('changed_denominators')
    if len(normals)!=8 or not all(a['normal_completed'] for a in normals):errors.append('normal_completion_not8of8')
    # Both turn directions in BOTH layouts, not centre-only or one-way coverage.
    for direction in ('left','right'):
        for layout in ('base','wide'):
            family={c['id'] for c in suite['cases'] if c['expectation']=='normal' and c['direction']==direction and c['layout']==layout}
            if len(family)!=2 or not all(a['normal_completed'] for a in normals if a['case'] in family):errors.append('missing_direction_layout_coverage')
    report=dict(stage='S3',passed=not errors and all(not a['errors'] for a in assessments),
                suite_errors=errors,normal_completion=dict(completed=sum(a['normal_completed'] for a in normals),total=len(normals)),
                denominators=dict(groups),assessment=assessments,arrival_status='unverified',
                scope='restricted colour/flat/static L-corner simulator task; NOT S2 material generalisation, whole Block6 or real-car certification')
    out=Path(output);out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
    return report


def assess_corner_case(case,result,decisions,*,junction=False):
    """Safety and geometric functional completion have separate denominators."""
    audit=result['audit'];feedback=result['controller_result'];errors=[];kind=case['expectation']
    if result.get('exception') is not None:errors.append('execution_exception')
    if not decisions:errors.append('missing_decisions')
    if not audit['safe_final_stop'] or audit['wall_contact_physics_steps']!=0:errors.append('physical_stop_or_contact_failed')
    if audit['physics_audit_dt_s']!=1/120:errors.append('missing_120hz_audit')
    if audit['maximum_audited_speed_m_s']>.18 or audit['maximum_audited_yaw_rate_rad_s']>.35:errors.append('physical_speed_outside_domain')
    for b in audit['braking_episodes']:
        if b['latency_s'] is None or b['distance_m'] is None or b['latency_s']>.30 or b['distance_m']>.05:errors.append('unbounded_physical_braking')
    if feedback['arrival_status']!='unverified' or feedback['atomic_edge_compatible'] is not False or audit['node_arrival_status']!='unverified':errors.append('false_node_or_atomic_claim')
    for row in decisions:
        d=row['decision'];v,w=d['v_m_s'],d['omega_rad_s']
        if d['arrival_status']!='unverified' or d['atomic_edge_compatible'] is not False:errors.append('false_intermediate_arrival')
        if not all(type(n) in (int,float) and math.isfinite(n) for n in (v,w)) or not 0<=v<=.15 or abs(w)>.300001:errors.append('invalid_command')
        if v or w:
            ref=row['input']
            if ref is None or not 0<=row['sim_s']-ref['captured_sim_s']<=.25:errors.append('motion_without_fresh_rgb')
            elif (d['control_frame_id'],d['captured_sim_s'],d['decision_sim_s'])!=(ref['frame_id'],ref['captured_sim_s'],row['sim_s']):errors.append('unbound_motion')
    if kind=='fault_stop':
        fault=case['fault'];events=[r for r in decisions if r['tick']==fault['tick'] and r.get('injected_fault')==fault['kind']]
        if len(events)!=1:errors.append('fault_not_reached')
        else:
            event=events[0]['decision']
            if event['v_m_s'] or event['omega_rad_s'] or event['state']!='aborted':errors.append('fault_did_not_zero_abort')
            if event['phase']!='turn' or not .1<event['turn_angle_rad']<math.pi/2-.1:errors.append('fault_not_during_partial_turn')
        if not feedback['motion_commands'] or feedback['state']!='aborted':errors.append('no_partial_motion_before_fault')
        # The truth audit must independently show a real partial physical turn.
        if not 5.<abs(audit['final']['yaw_deg'])<85.:errors.append('physical_partial_turn_not_exercised')
    elif kind=='wrong_branch':
        if feedback['state']!='aborted' or feedback['turn_angle_rad']!=0.:errors.append('wrong_branch_not_refused_before_turn')
    elif kind=='blocked':
        if not junction:raise ValueError('Unexpected S3 denominator')
        if feedback['state']!='aborted' or feedback['turn_angle_rad']!=0. or feedback['reason']!='branch_or_brake_confirmation_timeout':errors.append('blocked_branch_not_refused_before_turn')
    elif kind=='cancel_stop':
        if not junction:raise ValueError('Unexpected S3 denominator')
        if feedback['state']!='aborted' or feedback['reason']!='execution_cancelled' or not feedback['motion_commands']:errors.append('partial_motion_cancel_not_exercised')
        if not decisions or decisions[-1].get('external_event')!='tick_budget_exhausted':errors.append('missing_budget_cancel_event')
    elif kind=='no_motion':
        if feedback['motion_commands'] or audit['displacement_m']>.001 or feedback['state']!='aborted':errors.append('unsupported_start_moved')
    elif kind not in (('normal','clear_after_wait','alternative_blocked') if junction else ('normal',)):raise ValueError('Unexpected task denominator')
    # Independent post-run position/orientation/velocity/contact check, not
    # merely the controller's own completed flag. Scene values stay here only.
    geometry=result['corner_geometry'];sign=1 if case['direction']=='left' else -1
    final=audit['final'];heading=abs((final['yaw_deg']-sign*90.+180.)%360.-180.)
    geometric=(sign*final['position_m'][1]>geometry['width_m']/2+.20 and
               abs(final['position_m'][0]-geometry['corner_x_m'])<.20 and heading<6. and
               final['speed_m_s']<.01 and abs(final['yaw_rate_rad_s'])<.02 and audit['wall_contact_physics_steps']==0)
    functional=kind in (('normal','clear_after_wait','alternative_blocked') if junction else ('normal',))
    completion='completed_visual_junction' if junction else 'completed_visual_corner'
    completed=(functional and feedback['result_type']==completion and geometric)
    if not functional and feedback['result_type']==completion:errors.append('false_completion_on_counterexample')
    if junction:
        if result.get('version')!='continuous_t_junction_run_v1' or feedback.get('junction')!='t':errors.append('not_a_t_junction_run')
        if feedback['direction']!=case['direction']:errors.append('silently_changed_planned_direction')
        turns=[i for i,row in enumerate(decisions) if row['decision']['omega_rad_s'] and row['decision']['phase']=='turn']
        if any((1 if case['direction']=='left' else -1)*decisions[i]['decision']['omega_rad_s']<=0 for i in turns):errors.append('turned_towards_unplanned_branch')
        if turns:
            i=turns[0];evidence=decisions[max(0,i-2):i+1]
            observations=[row['decision']['perception'].get('branches') for row in evidence]
            if len(observations)!=3 or any(b!=observations[0] or not b or b[case['direction']]!='open' or 'unknown' in b.values() for b in observations):errors.append('turn_without_three_consistent_branch_observations')
            if kind=='normal' and any(b!={'left':'open','right':'open'} for b in observations):errors.append('normal_t_branches_not_both_open')
            if kind=='alternative_blocked' and not any(b and b.get('right' if case['direction']=='left' else 'left')=='blocked' for b in observations):errors.append('unselected_block_not_exercised')
        waits=[row for row in decisions if row['decision']['reason']=='waiting_for_planned_branch_clear']
        if any(row['decision']['v_m_s'] or row['decision']['omega_rad_s'] for row in waits):errors.append('moved_while_waiting_for_blocked_branch')
        if kind in ('blocked','clear_after_wait') and not waits:errors.append('selected_block_never_observed')
        if kind=='blocked' and turns:errors.append('blocked_branch_turned')
        if kind=='clear_after_wait':
            events=audit.get('fixture_events',[])
            if len(events)!=1 or events[0]['tick']!=case['clear_blocker_tick'] or events[0]['event']!='branch_blocker_removed':errors.append('temporary_block_not_cleared_at_declared_tick')
            if not turns or decisions[turns[0]]['tick']<case['clear_blocker_tick']+2:errors.append('resumed_without_new_post_clear_observations')
        if functional:
            phases=[]
            for row in decisions:
                phase=row['decision']['phase']
                if not phases or phases[-1]!=phase:phases.append(phase)
            if phases!=['entry','entry_stop','turn','turn_stop','exit','exit_stop']:errors.append('incomplete_visual_phase_sequence')
    return dict(case=case['id'],expectation=kind,normal_completed=completed,errors=sorted(set(errors)),
                result_type=feedback['result_type'],physical_safe=audit['safe_final_stop'],
                exit_progress_m=sign*final['position_m'][1],lateral_error_m=final['position_m'][0]-geometry['corner_x_m'],heading_error_deg=heading)


def evaluate_junction_suite(suite_path,runs,freeze,output):
    """Independent post-phase S4 gates, never a controller input."""
    from simulation.continuous_straight_suite import load_junction_suite
    from simulation.rgb_observability_run import ROOT
    lock=verify_freeze(freeze);suite=load_junction_suite(suite_path)
    if not lock.get('frozen_before_generation') or str(Path(suite_path).resolve().relative_to(ROOT)) not in lock['files']:
        raise ValueError('Bound source/suite freeze required')
    root=Path(runs);ids=[c['id'] for c in suite['cases']]
    digest=hashlib.sha256(Path(suite_path).read_bytes()).hexdigest()
    receipt=json.loads((Path(freeze).parent/'FIRST_S4_GENERATION_STARTED.json').read_text())
    if receipt['suite_sha256']!=digest or receipt['output']!=str(root.resolve()):raise ValueError('Wrong first-generation receipt')
    marker=json.loads((root/'phase-complete.json').read_text())
    if marker!=dict(method='S4_control',cases=ids,suite_sha256=digest):raise ValueError('Complete bound S4 phase required')
    assessments=[]
    for c in suite['cases']:
        base=root/c['id'];result=json.loads((base/'result.json').read_text())
        decisions=[json.loads(s) for s in (base/'decisions.jsonl').read_text().splitlines()]
        if (result['seed']!=c['seed'] or result['corner_direction']!=c['direction'] or result['corner_geometry']!={**c['geometry'],'direction':c['direction']} or
            result['fault']!=c['fault'] or result['operator_confirmed_initial_clearance'] is not c['operator_confirmed'] or
            result['blocked_branch']!=c['blocked_branch'] or result['clear_blocker_tick']!=c['clear_blocker_tick'] or result['pattern']!='unique' or result['appearance'] is not None):
            raise ValueError('Run setup differs from frozen case')
        if max(row['tick'] for row in decisions)>c['max_ticks']:raise ValueError('Run exceeds frozen budget')
        verify_public_records(base,decisions)
        assessments.append(assess_corner_case(c,result,decisions,junction=True))
    errors=[];functional={}
    for kind,total in [('normal',8),('clear_after_wait',2),('alternative_blocked',2)]:
        group=[a for a in assessments if a['expectation']==kind]
        functional[kind]=dict(completed=sum(a['normal_completed'] for a in group),total=len(group))
        if len(group)!=total or not all(a['normal_completed'] for a in group):errors.append('incomplete_functional_group_'+kind)
    report=dict(stage='S4',passed=not errors and all(not a['errors'] for a in assessments),suite_errors=errors,
                functional_completion=functional,denominators=dict(Counter(c['expectation'] for c in suite['cases'])),
                assessment=assessments,arrival_status='unverified',whole_block6_complete=False,
                scope='restricted colour/static/flat/calibrated stem-entry T junction, planned left/right only; not node identity, S2 material generalisation or real-car safety')
    out=Path(output);out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
    return report


def verify_route_replay(base,result,rows):
    """Re-execute from ONLY hash-checked public RGB/map/references/events.

    Does not open audit/, instantiate a world, or use physical_scene. Binds
    every node proof and transport response to exactly the saved input stream.
    """
    from roadvision.continuous_straight import ContinuousRouteController,InitialClearancePrior
    from simulation.path_planning import plan_from_estimate,next_action
    base=Path(base);public=base/'public';references=[]
    graph=json.loads((public/'topology.json').read_text())
    if graph!=result['route_graph']:raise ValueError('Public topology differs from controller input')
    manifest=json.loads((public/'references.json').read_text())
    for ref in manifest:
        if set(ref)!={'node_id','heading_deg','rgb_path','rgb_sha256'}:raise ValueError('Reference contains nonpublic fields')
        path=(public/ref['rgb_path']).resolve()
        if Path(ref['rgb_path']).is_absolute() or not path.is_relative_to(public.resolve()):raise ValueError('Reference path escaped public root')
        rgb=np.array(Image.open(path))
        if hashlib.sha256(rgb.tobytes()).hexdigest()!=ref['rgb_sha256']:raise ValueError('Public reference pixel hash mismatch')
        references.append(dict(node_id=ref['node_id'],heading_deg=ref['heading_deg'],rgb=rgb))
    task=dict(start_node=result['start_node'],goal_node=result['goal_node'],heading_deg=result['heading_deg'],session_id=result.get('session_id','continuous-route'))
    if (public/'task.json').exists() and json.loads((public/'task.json').read_text())!=task:raise ValueError('Public task/session differs from controller prior')
    c=ContinuousRouteController(session_id=task['session_id'],graph=graph,references=references,plan_from_estimate=plan_from_estimate,next_action=next_action,
        start=result['start_node'],goal=result['goal_node'],heading=result['heading_deg'],initial_clearance=InitialClearancePrior(confirmed=result['operator_confirmed_initial_clearance']))
    deliveries=[]
    for row in rows:
        if row.get('external_event')=='tick_budget_exhausted':d=c.cancel(row['sim_s'])
        elif row['input'] is None:d=c.missing(row['sim_s'])
        else:
            ref=row['input'];rgb=np.array(Image.open(public/ref['rgb_path']))
            d=c.update(dict(frame_id=ref['frame_id'],captured_sim_s=ref['captured_sim_s'],rgb=rgb),row['sim_s'])
        if row.get('segment_output') is not None and row['segment_output']!=d:raise ValueError('Unbound pre-transport segment output')
        for record in row.get('feedback_deliveries',[]):
            if record['tick']!=row['tick'] or record['sim_s']!=row['sim_s']:raise ValueError('Transport time binding mismatch')
            d=c.feedback(record['event'],row['sim_s'])
            if d!=record['decision']:raise ValueError('Transport response differs on public replay')
            deliveries.append(record)
        if d!=row['decision']:raise ValueError('Decision differs on exact public replay at '+str(row['tick']))
    if c.result()!=result['controller_result']:raise ValueError('Final result differs on exact public replay')
    saved=[json.loads(l) for l in (base/'feedback.jsonl').read_text().splitlines()]
    if [r for r in saved if r['event'] is not None]!=deliveries:raise ValueError('Feedback trail differs from decision deliveries')
    drops=[r for r in saved if r['event'] is None]
    expected_drop=result['transport_fault'] is not None and result['transport_fault']['kind']=='drop'
    if len(drops)!=(1 if expected_drop else 0):raise ValueError('Missing/unexpected drop receipt')
    for drop in drops:
        row=next(r for r in rows if r['tick']==drop['tick'])
        if row['segment_output']['pending_feedback']!=drop['bound_event'] or row['feedback_deliveries']:raise ValueError('Drop not bound to actual segment output')
    return dict(decisions=len(rows),references=len(references),feedback_deliveries=len(deliveries))


def assess_route_case(case,result,rows,truth):
    """S5 independent physical/node proof gates; normal refusal FAILS function."""
    c=result['controller_result'];a=result['audit'];errors=[];kind=case['expectation']
    if result.get('exception') is not None:errors.append('execution_exception')
    if not rows:errors.append('missing_decisions')
    if not a['safe_final_stop'] or a['wall_contact_physics_steps']!=0:errors.append('physical_stop_or_contact_failed')
    if a['physics_audit_dt_s']!=1/120 or a['maximum_audited_speed_m_s']>.18 or a['maximum_audited_yaw_rate_rad_s']>.35:errors.append('physical_speed_or_audit_rate_invalid')
    for b in a['braking_episodes']:
        if b['latency_s'] is None or b['distance_m'] is None or b['latency_s']>.30 or b['distance_m']>.05:errors.append('unbounded_physical_braking')
    if c['atomic_edge_compatible'] is not False:errors.append('false_atomic_semantics')
    completed=c['result_type']=='completed_continuous_route'
    if not completed and (c['node_estimate'] is not None or c['position_status']!='uncertain' or c['arrival_status']!='unverified'):errors.append('aborted_pose_wrongly_certain')
    for row in rows:
        d=row['decision'];v,w=d['v_m_s'],d['omega_rad_s'];ref=row['input']
        if d['atomic_edge_compatible'] is not False:errors.append('atomic_decision_claim')
        if not all(type(n) in (int,float) and math.isfinite(n) for n in (v,w)) or not 0<=v<=.15 or abs(w)>.300001:errors.append('invalid_route_command')
        if v or w:
            if ref is None or not 0<=row['sim_s']-ref['captured_sim_s']<=.25:errors.append('motion_without_fresh_rgb')
            elif (d['control_frame_id'],d['captured_sim_s'],d['decision_sim_s'])!=(ref['frame_id'],ref['captured_sim_s'],row['sim_s']):errors.append('unbound_motion')
        if d['state']!='visual_stop' and d['arrival_status']!='unverified':errors.append('premature_arrival_claim')
        if d['state']=='visual_stop' and (d['node_estimate'] is None or d['node_estimate']['node_id']!=case['goal_node']):errors.append('wrong_terminal_node')
        if d['pending_command'] is not None and d['node_estimate'] is not None:errors.append('pending_motion_retains_certain_source')
        segment=row.get('segment_output')
        if segment is not None and (segment['node_estimate'] is not None or segment['arrival_status']!='unverified'):errors.append('visual_exit_promoted_to_node')
    # Recompute station checks from separately collected physical samples, for
    # ALL three fresh proof frames, not just the controller's final flag.
    inputs={r['input']['frame_id']:r['input'] for r in rows if r['input'] is not None}
    samples={round(t['sim_s'],8):t for t in [a['initial'],*truth]}
    positions=case['physical_scene']['geometry']['node_positions_m'];proof_errors=[]
    for proof in c['node_confirmations']:
        ids=proof['frame_ids']
        if len(ids)!=3 or len(set(ids))!=3 or any(f not in inputs for f in ids):errors.append('invalid_three_frame_node_proof');continue
        times=[inputs[f]['captured_sim_s'] for f in ids]
        if not times[0]<times[1]<times[2] or times[-1]!=proof['captured_sim_s']:errors.append('nonfresh_node_proof')
        for t in times:
            sample=samples.get(round(t,8))
            if sample is None:errors.append('missing_independent_node_sample');continue
            distance=float(np.linalg.norm(np.array(sample['position_m'])[:2]-np.array(positions[proof['node_id']])))
            heading=abs((sample['yaw_deg']-proof['heading_deg']+180)%360-180)
            proof_errors.append(distance)
            if distance>=.20 or heading>=6 or sample['speed_m_s']>=.01 or abs(sample['yaw_rate_rad_s'])>=.02:errors.append('false_or_moving_node_confirmation')
    commands=c['commands'];events=c['consumed_segment_events'];proofs=c['node_confirmations']
    if len(commands)!=c['command_count'] or len(events)!=len({e['event_id'] for e in events}) or len(proofs)!=len({p['captured_sim_s'] for p in proofs}):errors.append('duplicate_commands_events_or_proofs')
    if c['transition_count']!=max(0,len(proofs)-1):errors.append('transition_count_not_visual_node_count')
    if c['transition_count']>len(events):errors.append('node_transition_without_bound_segment')
    for i,proof in enumerate(proofs):
        if i==0:
            if (proof['node_id'],proof['heading_deg'],proof['command_id'])!=(case['start_node'],case['heading_deg'],None):errors.append('wrong_initial_confirmation')
        else:
            cmd=commands[i-1];event=next((e for e in events if e['command_id']==cmd['command_id']),None)
            if proof['command_id']!=cmd['command_id'] or proof['node_id']!=cmd['target'] or event is None or proof['captured_sim_s']<=event['captured_sim_s']:errors.append('unbound_or_pre_exit_node_confirmation')
    final=a['final'];goal_distance=float(np.linalg.norm(np.array(final['position_m'])[:2]-np.array(positions[case['goal_node']])))
    goal_heading_error=abs((final['yaw_deg']-c['heading_deg']+180)%360-180)
    geometric=goal_distance<.20 and goal_heading_error<6 and final['speed_m_s']<.01 and abs(final['yaw_rate_rad_s'])<.02 and a['wall_contact_physics_steps']==0
    functional=kind in ('normal','duplicate')
    success=functional and completed and geometric and c['transition_count']>=2 and len(events)==c['transition_count'] and proofs[-1]['node_id']==case['goal_node']
    if completed and not functional:errors.append('false_completion_on_counterexample')
    if kind=='duplicate' and c['duplicate_feedback']!=1:errors.append('duplicate_not_idempotently_exercised')
    if kind=='camera':
        f=case['fault'];found=[r for r in rows if r['tick']==f['tick'] and r.get('injected_fault')==f['kind']]
        if len(found)!=1 or found[0]['decision']['state']!='aborted' or found[0]['decision']['v_m_s'] or found[0]['decision']['omega_rad_s']:errors.append('camera_fault_not_reached_or_zero_aborted')
        if not c['motion_commands'] or (f['tick']==700 and c['command_count']<2):errors.append('camera_did_not_exercise_later_partial_edge')
    if kind=='transport':
        reasons={'drop':'segment_feedback_timeout_pose_uncertain','stale':'stale_segment_feedback_pose_uncertain','unknown_command':'unbound_segment_feedback_pose_uncertain','conflict':'conflicting_duplicate_feedback_pose_uncertain'}
        if c['reason']!=reasons[case['transport_fault']['kind']] or c['command_count']!=2 or c['transition_count']!=1:errors.append('second_segment_transport_failure_not_exercised')
    if kind in ('wrong_start','wrong_heading') and (c['reason']!='node_or_heading_prior_conflict' or proofs or commands):errors.append('wrong_identity_not_independently_refused')
    if kind=='cancel' and (c['reason']!='execution_cancelled' or not c['motion_commands'] or rows[-1].get('external_event')!='tick_budget_exhausted' or (case['max_ticks']==700 and c['command_count']<2)):errors.append('partial_route_cancel_not_exercised')
    if kind=='no_prior' and (c['motion_commands'] or a['displacement_m']>.001 or c['reason']!='primitive_initial_clearance_prior_not_confirmed'):errors.append('unconfirmed_route_launch_moved')
    if not functional and c['state']!='aborted':errors.append('counterexample_not_aborted')
    return dict(case=case['id'],expectation=kind,functional_completed=success,errors=sorted(set(errors)),result_type=c['result_type'],reason=c['reason'],transition_count=c['transition_count'],goal_body_distance_m=goal_distance,maximum_node_proof_distance_m=max(proof_errors,default=0.),physical_safe=a['safe_final_stop'])


def evaluate_route_suite(suite_path,runs,freeze,output):
    from simulation.continuous_straight_suite import load_route_suite
    from simulation.rgb_observability_run import ROOT
    lock=verify_freeze(freeze);suite=load_route_suite(suite_path);root=Path(runs)
    if not lock.get('frozen_before_generation') or str(Path(suite_path).resolve().relative_to(ROOT)) not in lock['files']:raise ValueError('Bound S5 source/suite required')
    digest=hashlib.sha256(Path(suite_path).read_bytes()).hexdigest();ids=[c['id'] for c in suite['cases']]
    receipt=json.loads((Path(freeze).parent/'FIRST_S5_GENERATION_STARTED.json').read_text())
    if receipt['suite_sha256']!=digest or receipt['output']!=str(root.resolve()):raise ValueError('Wrong S5 generation receipt')
    if json.loads((root/'phase-complete.json').read_text())!=dict(method='S5_control',cases=ids,suite_sha256=digest):raise ValueError('Full bound S5 phase required before assessment')
    assessments=[];replays=[];sessions=set()
    for case in suite['cases']:
        base=root/case['id'];result=json.loads((base/'result.json').read_text());rows=[json.loads(l) for l in (base/'decisions.jsonl').read_text().splitlines()]
        bindings={'route_graph':'graph','route_scene':'physical_scene','operator_confirmed_initial_clearance':'operator_confirmed',**{k:k for k in ('seed','start_node','goal_node','heading_deg','physical_start_node','physical_heading_deg','max_ticks','fault','transport_fault')}}
        if result.get('version')!='continuous_route_run_v1' or any(result.get(k)!=case[v] for k,v in bindings.items()) or result['appearance'] is not None or result['pattern']!='unique':raise ValueError('Route setup differs from frozen case')
        if max(r['tick'] for r in rows)>case['max_ticks']:raise ValueError('Route exceeded frozen budget')
        session=result.get('session_id')
        if not isinstance(session,str) or not session.startswith('continuous-route:') or session in sessions:raise ValueError('Missing/reused mission session ID')
        sessions.add(session)
        if not (base/'public/task.json').is_file():raise ValueError('Missing public task/session prior')
        verify_public_records(base,rows);replays.append(dict(case=case['id'],**verify_route_replay(base,result,rows)))
        truth=json.loads((base/'audit/truth.json').read_text())
        assessments.append(assess_route_case(case,result,rows,truth))
    errors=[];functional={}
    for kind,total in [('normal',6),('duplicate',2)]:
        group=[a for a in assessments if a['expectation']==kind];functional[kind]=dict(completed=sum(a['functional_completed'] for a in group),total=len(group))
        if len(group)!=total or not all(a['functional_completed'] for a in group):errors.append('incomplete_functional_group_'+kind)
    verify_freeze(freeze)
    report=dict(stage='S5',passed=not errors and all(not a['errors'] for a in assessments),suite_errors=errors,functional_completion=functional,
        denominators=dict(Counter(c['expectation'] for c in suite['cases'])),assessment=assessments,public_replays=replays,whole_block6_complete=False,
        scope='restricted static blue-grey-floor calibrated orthogonal stem-entry L/T/dead-end routes with public natural-wall reference RGB and known cardinal start task; not S2 material generalisation, arbitrary-viewpoint localisation or real-car certification')
    out=Path(output);out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
    return report


def replay_safety_public(base,result,rows):
    """S6 deterministic replay: NO audit/scene geometry/physics reads here."""
    from roadvision.continuous_straight import StraightController,CornerController,ContinuousRouteController,InitialClearancePrior
    from simulation.path_planning import plan_from_estimate,next_action
    base=Path(base);public=base/'public';config=result['safety_checks']['configuration']
    prior=InitialClearancePrior(confirmed=config['operator_confirmed'])
    def pixels(ref):
        path=(public/ref['rgb_path']).resolve()
        if Path(ref['rgb_path']).is_absolute() or not path.is_relative_to(public.resolve()):raise ValueError('Public RGB path escaped root')
        rgb=np.array(Image.open(path))
        if rgb.shape!=(256,256,3) or rgb.dtype!=np.uint8 or hashlib.sha256(rgb.tobytes()).hexdigest()!=ref['rgb_sha256']:raise ValueError('Public pixel hash mismatch')
        return rgb
    if config['route_graph'] is not None:
        graph=json.loads((public/'topology.json').read_text());task=json.loads((public/'task.json').read_text())
        if graph!=config['route_graph'] or task!={k:result[k] for k in ('start_node','goal_node','heading_deg','session_id')}:raise ValueError('Public task/map differs from route prior')
        references=[]
        for ref in json.loads((public/'references.json').read_text()):
            if set(ref)!={'node_id','heading_deg','rgb_path','rgb_sha256'}:raise ValueError('Nonpublic reference fields')
            references.append(dict(node_id=ref['node_id'],heading_deg=ref['heading_deg'],rgb=pixels(ref)))
        c=ContinuousRouteController(graph=graph,references=references,plan_from_estimate=plan_from_estimate,next_action=next_action,
            start=task['start_node'],goal=task['goal_node'],heading=task['heading_deg'],session_id=task['session_id'],initial_clearance=prior)
    elif config['corner_direction'] is not None:c=CornerController(direction=config['corner_direction'],junction=config['junction'],initial_clearance=prior)
    else:
        task=json.loads((public/'task.json').read_text())
        from dataclasses import asdict
        from roadvision.corridor_rgb_probe import CALIBRATION_ID
        expected=dict(version='straight_motion_permission_v1',motion_permission=config.get('motion_permission','no_motion'),initial_clearance=asdict(prior),calibration_id=CALIBRATION_ID)
        variant=config.get('straight_variant','baseline')
        if variant not in ('baseline','history','control','combined','swept','rear','rear_active','reference','wide_reference','gradient_transfer'):raise ValueError('Unknown straight replay variant')
        if variant!='baseline':expected['straight_variant']=variant
        if variant=='swept':
            from roadvision.swept_ground_path import task_contract
            expected['trajectory_contract']=task_contract()
        if variant in ('rear','rear_active'):
            from roadvision.rear_observation import task_contract as rear_contract
            from roadvision.swept_ground_path import task_contract as sweep_contract
            expected['rear_contract']=rear_contract(variant=='rear_active')
            expected['trajectory_contract']=sweep_contract()
        if task!=expected:raise ValueError('Public straight permission differs from declared task')
        c=StraightController(initial_clearance=prior,motion_permission=task['motion_permission'])
        if variant=='gradient_transfer':
            from roadvision.gradient_straight import GradientTransferStraightController
            c=GradientTransferStraightController(initial_clearance=prior,motion_permission=task['motion_permission'])
        if variant=='wide_reference':
            from roadvision.wide_reference_straight import WideReferenceStraightController
            c=WideReferenceStraightController(initial_clearance=prior,motion_permission=task['motion_permission'])
        if variant=='reference':
            from roadvision.reference_straight import ReferenceStraightController
            c=ReferenceStraightController(initial_clearance=prior,motion_permission=task['motion_permission'])
        if variant in ('history','control','combined'):
            from roadvision.straight_ablation import AblationStraightController
            c=AblationStraightController(variant=variant,initial_clearance=prior,motion_permission=task['motion_permission'])
        if variant=='swept':
            from roadvision.swept_straight import SweptStraightController
            c=SweptStraightController(initial_clearance=prior,motion_permission=task['motion_permission'])
        if variant in ('rear','rear_active'):
            from roadvision.rear_straight import RearStraightController
            c=RearStraightController(active_review=variant=='rear_active',initial_clearance=prior,motion_permission=task['motion_permission'])
    deliveries=[]
    for row in rows:
        error_record=None
        if row.get('external_event')=='tick_budget_exhausted':d=c.cancel(row['sim_s'])
        elif row.get('external_event')=='runner_exception_zero_stop':raise ValueError('Unexpected infrastructure exception is not an accepted replay input')
        elif row['input'] is None:d=c.missing(row['sim_s'])
        else:
            ref=row['input'];frame={k:v for k,v in ref.items() if k not in ('rgb_path','rgb_sha256')};frame['rgb']=pixels(ref)
            try:d=c.update(frame,row['sim_s'])
            except Exception as error:
                error_record=dict(type=type(error).__name__,message=str(error));d=c.missing(row['sim_s'])
        if error_record!=row.get('input_error'):raise ValueError('Input exception trail differs on public replay')
        if row.get('segment_output') is not None and row['segment_output']!=d:raise ValueError('Segment feedback not bound to RGB output')
        for record in row.get('feedback_deliveries',[]):
            if record['tick']!=row['tick'] or record['sim_s']!=row['sim_s']:raise ValueError('Feedback delivery clock differs')
            d=c.feedback(record['event'],row['sim_s'])
            if d!=record['decision']:raise ValueError('Feedback response differs on replay')
            deliveries.append(record)
        if d!=row['decision']:raise ValueError('Public decision replay differs at '+str(row['tick']))
    if c.result()!=result['controller_result']:raise ValueError('Final public replay differs')
    terminal=json.loads((public/'terminal-checks.json').read_text());original=c.result()
    if len(terminal)!=3:raise ValueError('Three real terminal good frames required')
    for record in terminal:
        ref=record['input'];d=c.update(dict(frame_id=ref['frame_id'],captured_sim_s=ref['captured_sim_s'],rgb=pixels(ref)),record['sim_s'])
        if d!=record['decision'] or c.result()!=original or not record['result_unchanged'] or d['v_m_s'] or d['omega_rad_s']:raise ValueError('Terminal latching failed')
    if config['route_graph'] is not None:
        saved=[json.loads(l) for l in (base/'feedback.jsonl').read_text().splitlines()]
        if [r for r in saved if r['event'] is not None]!=deliveries:raise ValueError('Saved feedback trail differs')
        drops=[r for r in saved if r['event'] is None]
        expected=config['transport_fault'] is not None and config['transport_fault']['kind']=='drop'
        if len(drops)!=(1 if expected else 0):raise ValueError('Drop receipt mismatch')
        for drop in drops:
            row=next(r for r in rows if r['tick']==drop['tick'])
            if row['segment_output']['pending_feedback']!=drop['bound_event'] or row['feedback_deliveries']:raise ValueError('Drop not bound to current event')
    return dict(decisions=len(rows),terminal_frames=len(terminal),feedback_deliveries=len(deliveries))


def audit_safety_physics(result,rows,trace):
    """Recompute peak speeds/each braking path from 120Hz one-way trace.

    Does not trust runner pass flags or runner precomputed braking summaries.
    A final 1s settling period cannot erase an excessive stopping distance.
    """
    errors=[];audit=result['audit'];initial=audit['initial'];final=audit['final'];episodes=[]
    samples=[t for t in trace if t['sim_s']>=initial['sim_s']-1e-8]
    if not samples:return dict(errors=['missing_physics_trace'],braking=[],maximum_speed_m_s=None,maximum_yaw_rate_rad_s=None)
    required={'sim_s','position_m','speed_m_s','yaw_rate_rad_s','wall_contact_physics_steps'}
    if any(not required<=set(t) or not all(type(t[k]) in (int,float) and math.isfinite(t[k]) for k in ('sim_s','speed_m_s','yaw_rate_rad_s','wall_contact_physics_steps')) or len(t['position_m'])!=3 or not all(type(x) in (int,float) and math.isfinite(x) for x in t['position_m']) for t in samples):raise ValueError('Invalid physics trace')
    if any(abs(b['sim_s']-a['sim_s']-1/120)>1e-7 for a,b in zip(samples,samples[1:])):errors.append('noncontinuous_120hz_trace')
    if abs(samples[0]['sim_s']-initial['sim_s'])>1e-7 or abs(samples[-1]['sim_s']-final['sim_s'])>1e-7:errors.append('trace_does_not_cover_entire_run')
    if any(t['speed_m_s']<0 or type(t['wall_contact_physics_steps']) is not int or t['wall_contact_physics_steps']<0 for t in samples):errors.append('invalid_physical_speed_or_contact_counter')
    if any(b['wall_contact_physics_steps']<a['wall_contact_physics_steps'] for a,b in zip(samples,samples[1:])):errors.append('contact_counter_regressed')
    for key in ('position_m','speed_m_s','yaw_rate_rad_s'):
        if samples[-1][key]!=final[key]:errors.append('final_audit_differs_from_last_physics_step')
    peak=max(t['speed_m_s'] for t in samples);yaw=max(abs(t['yaw_rate_rad_s']) for t in samples)
    if peak>.18 or yaw>.35:errors.append('physical_speed_outside_declared_domain')
    contacts=max(t['wall_contact_physics_steps'] for t in samples)
    if contacts!=0 or audit['wall_contact_physics_steps']!=contacts:errors.append('physical_contact_or_mismatched_contact_audit')
    if final['speed_m_s']>=.01 or abs(final['yaw_rate_rad_s'])>=.02:errors.append('final_not_stationary')
    if abs(peak-audit['maximum_audited_speed_m_s'])>1e-10 or abs(yaw-audit['maximum_audited_yaw_rate_rad_s'])>1e-10:errors.append('peak_audit_mismatch')
    for i,row in enumerate(rows):
        d=row['decision'];prev=rows[i-1]['decision'] if i else None
        if not prev or d['v_m_s'] or d['omega_rad_s'] or not (prev['v_m_s'] or prev['omega_rad_s']):continue
        requested=row['sim_s'];resume=next((r['sim_s'] for r in rows[i+1:] if r['decision']['v_m_s'] or r['decision']['omega_rad_s']),final['sim_s']+1/120)
        after=[t for t in samples if requested<t['sim_s']<resume]
        stop=next((t for t in after if t['speed_m_s']<.01 and abs(t['yaw_rate_rad_s'])<.02),None)
        before=next((t for t in reversed(samples) if t['sim_s']<=requested+1e-9),None)
        path=[before]+[t for t in after if stop is None or t['sim_s']<=stop['sim_s']] if before is not None else []
        distance=sum(float(np.linalg.norm(np.array(b['position_m'])[:2]-np.array(a['position_m'])[:2])) for a,b in zip(path,path[1:]))
        latency=stop['sim_s']-requested if stop is not None else None
        if stop is None or before is None or latency>.30 or distance>.05:errors.append('unbounded_physical_braking')
        episodes.append(dict(tick=row['tick'],latency_s=latency,distance_m=distance))
    if result['controller_result']['motion_commands'] and not episodes:errors.append('partial_motion_has_no_audited_stop_request')
    return dict(errors=sorted(set(errors)),braking=episodes,maximum_speed_m_s=peak,maximum_yaw_rate_rad_s=yaw)


def assess_safety_case(case,result,rows,truth,trace):
    """S6 unified safety gates + separate functional and known-boundary results."""
    mode=case['mode'];kind=case['expectation'];c=result['controller_result'];audit=result['audit'];errors=[]
    physics=audit_safety_physics(result,rows,trace);errors.extend(physics['errors'])
    if result.get('exception') is not None:errors.append('infrastructure_exception')
    checks=result.get('safety_checks',{})
    if checks.get('terminal_good_frame_checks')!=3 or not checks.get('terminal_latched'):errors.append('terminal_latch_not_checked')
    if checks.get('caught_input_errors') and kind!='input_error':errors.append('unexpected_input_or_perception_exception')
    for row in rows:
        d=row['decision'];v,w=d['v_m_s'],d['omega_rad_s'];ref=row['input']
        if d['atomic_edge_compatible'] is not False:errors.append('atomic_rollback_claim')
        if not all(type(n) in (int,float) and math.isfinite(n) for n in (v,w)) or not 0<=v<=.15 or abs(w)>(.200001 if mode=='straight' else .300001):errors.append('unsafe_command')
        if v or w:
            if ref is None or not 0<=row['sim_s']-ref['captured_sim_s']<=.25:errors.append('motion_without_fresh_rgb')
            elif (d['control_frame_id'],d['captured_sim_s'],d['decision_sim_s'])!=(ref['frame_id'],ref['captured_sim_s'],row['sim_s']):errors.append('motion_not_bound_to_current_input')
        if d['arrival_status']!='unverified' and not (mode=='route' and d['state']=='visual_stop'):errors.append('premature_topological_arrival')
    if c['atomic_edge_compatible'] is not False:errors.append('atomic_result_claim')
    if c['state']=='aborted' and c['arrival_status']!='unverified':errors.append('aborted_claims_arrival')
    if mode=='route' and c['state']=='aborted' and (c['node_estimate'] is not None or c['position_status']!='uncertain'):errors.append('partial_route_pose_not_uncertain')
    functional=kind in ('normal','duplicate');completed=False
    if mode=='route':
        # Reuse S5 independent ALL-three-frame station/command/event audit.
        # S6 adds its own fault checks below, not S5's different case budgets.
        route_case=dict(id=case['id'],expectation='normal',physical_scene=result['route_scene'],start_node=result['start_node'],goal_node=result['goal_node'],heading_deg=result['heading_deg'])
        route_assessment=assess_route_case(route_case,result,rows,truth)
        errors.extend(route_assessment['errors']);completed=route_assessment['functional_completed']
    else:
        final=audit['final'];stationary=final['speed_m_s']<.01 and abs(final['yaw_rate_rad_s'])<.02
        if mode=='straight':
            scene=result['scene'];gap=scene['length_m']-final['camera_position_m'][0]
            geometric=scene['obstacle']=='none' and .8<=gap<=1.7 and abs(final['position_m'][1])<.2 and abs(final['yaw_deg'])<6 and stationary
            completed=c['result_type']=='completed_visual_stop' and geometric
        else:
            g=result['corner_geometry'];sign=1 if result['corner_direction']=='left' else -1
            heading=abs((final['yaw_deg']-sign*90+180)%360-180)
            geometric=sign*final['position_m'][1]>g['width_m']/2+.20 and abs(final['position_m'][0]-g['corner_x_m'])<.20 and heading<6 and stationary
            completed=c['result_type']==('completed_visual_junction' if mode=='t' else 'completed_visual_corner') and geometric
    if kind in ('camera','input_error'):
        f=case['arguments']['fault'];events=[r for r in rows if r['tick']==f['tick'] and r.get('injected_fault')==f['kind']]
        if len(events)!=1:errors.append('declared_fault_not_reached')
        else:
            d=events[0]['decision']
            if d['v_m_s'] or d['omega_rad_s'] or d['state']!='aborted':errors.append('fault_not_immediate_zero_abort')
            if kind=='input_error' and (events[0].get('input_error',{}).get('type')!='ValueError' or len(checks.get('caught_input_errors',[]))!=1):errors.append('input_exception_not_durably_recorded')
            if mode in ('l','t'):
                angle=d.get('turn_angle_rad',0.)
                if not .1<angle<math.pi/2-.1 or not 5.<abs(audit['final']['yaw_deg'])<85.:errors.append('fault_not_during_real_partial_turn')
            if mode=='route' and d['command_count']<2:errors.append('fault_did_not_exercise_second_route_command')
        if not c['motion_commands']:errors.append('late_fault_without_partial_motion')
    if kind=='cancel':
        if c['state']!='aborted' or c['reason']!='execution_cancelled' or not c['motion_commands'] or rows[-1].get('external_event')!='tick_budget_exhausted':errors.append('partial_execution_cancel_not_exercised')
        if mode=='route' and c['command_count']<2:errors.append('cancel_not_during_later_route_edge')
    if kind=='no_prior' and (c['state']!='aborted' or c['motion_commands'] or audit['displacement_m']>.001):errors.append('unconfirmed_initial_clearance_moved')
    if kind=='blocked':
        if mode=='t' and (c['state']!='aborted' or c['turn_angle_rad'] or c['reason']!='branch_or_brake_confirmation_timeout'):errors.append('blocked_selected_branch_not_refused')
        if c['arrival_status']!='unverified' or audit['normal_geometric_stop_pass']:errors.append('blocker_stop_claimed_as_goal')
    if kind in ('transport','duplicate'):
        t=case['arguments']['transport_fault']
        if kind=='duplicate':
            if c['duplicate_feedback']!=1 or not completed:errors.append('duplicate_not_idempotent_full_route')
        else:
            reasons={'drop':'segment_feedback_timeout_pose_uncertain','stale':'stale_segment_feedback_pose_uncertain','unknown_command':'unbound_segment_feedback_pose_uncertain','conflict':'conflicting_duplicate_feedback_pose_uncertain'}
            if c['state']!='aborted' or c['reason']!=reasons[t['kind']] or c['command_count']!=2 or c['transition_count']!=1:errors.append('second_segment_feedback_failure_not_exercised')
    if not functional and kind not in ('blocked','texture_boundary','uniform_boundary') and c['state']!='aborted':errors.append('counterexample_not_aborted')
    if not functional and mode=='route' and c['state']=='visual_stop':errors.append('false_route_arrival_on_counterexample')
    limitations=[]
    if kind=='texture_boundary':
        # Do NOT reinstate the user-removed repeated-texture startup guard.
        # Observing a safe trajectory is not proof that aliasing is detectable.
        limitations.append('repeated_texture_observability_not_certified')
        if mode=='straight' and c['motion_commands']:limitations.append('legacy_straight_authorises_motion_on_repeated_texture')
    if kind=='uniform_boundary' and mode in ('straight','route') and c['motion_commands'] and mode=='straight':limitations.append('untextured_straight_motion_needs_review')
    return dict(case=case['id'],mode=mode,expectation=kind,errors=sorted(set(errors)),limitations=limitations,
        functional_completed=functional and completed,physical_safe=not physics['errors'],result_type=c['result_type'],reason=c['reason'],motion_commands=c['motion_commands'],physics=physics)


def evaluate_safety_suite(suite_path,runs,freeze,output):
    from simulation.continuous_straight_suite import load_safety_suite,safety_case_arguments
    from simulation.rgb_observability_run import ROOT
    from dataclasses import asdict
    lock=verify_freeze(freeze);suite=load_safety_suite(suite_path);root=Path(runs)
    if not lock.get('frozen_before_generation') or str(Path(suite_path).resolve().relative_to(ROOT)) not in lock['files']:raise ValueError('Bound S6 source/suite required')
    digest=hashlib.sha256(Path(suite_path).read_bytes()).hexdigest();ids=[c['id'] for c in suite['cases']]
    receipt=json.loads((Path(freeze).parent/'FIRST_S6_GENERATION_STARTED.json').read_text())
    if receipt['suite_sha256']!=digest or receipt['output']!=str(root.resolve()):raise ValueError('Wrong S6 first-generation receipt')
    if json.loads((root/'phase-complete.json').read_text())!=dict(method='S6_control',cases=ids,suite_sha256=digest):raise ValueError('Full S6 phase required before assessment')
    assessments=[];replays=[];sessions=set()
    for case in suite['cases']:
        base=root/case['id'];result=json.loads((base/'result.json').read_text());rows=[json.loads(l) for l in (base/'decisions.jsonl').read_text().splitlines()]
        config=safety_case_arguments(case)
        if config['scene'] is not None:config['scene']=asdict(config['scene'])
        if result.get('safety_checks',{}).get('configuration')!=config:raise ValueError('S6 run setup differs from frozen declaration')
        if max(r['tick'] for r in rows)>config['max_ticks']:raise ValueError('S6 budget exceeded')
        if case['mode']=='route':
            if not isinstance(result.get('session_id'),str) or result['session_id'] in sessions:raise ValueError('Missing/reused route session ID')
            sessions.add(result['session_id'])
        verify_public_records(base,rows);replays.append(dict(case=case['id'],**replay_safety_public(base,result,rows)))
        assessments.append(assess_safety_case(case,result,rows,json.loads((base/'audit/truth.json').read_text()),json.loads((base/'audit/physics-trace.json').read_text())))
    functional={};errors=[]
    for mode in ('straight','l','t','route'):
        group=[a for a in assessments if a['mode']==mode and a['expectation'] in ('normal','duplicate')]
        functional[mode]=dict(completed=sum(a['functional_completed'] for a in group),total=len(group))
        if not group or not all(a['functional_completed'] for a in group):errors.append('normal_refusal_or_failure_'+mode)
    limitations=sorted({l for a in assessments for l in a['limitations']})
    # S2 is explicitly deferred by the user. Never silently close it or claim
    # S6's entire observability/general-material boundary is now accepted.
    limitations.extend(['S2_full_straight_domain_unaccepted','synchronous_simulation_not_hardware_watchdog'])
    development_passed=not errors and all(not a['errors'] for a in assessments)
    blocking_boundaries=[l for l in limitations if l in ('repeated_texture_observability_not_certified','legacy_straight_authorises_motion_on_repeated_texture','untextured_straight_motion_needs_review')]
    report=dict(stage='S6',passed=development_passed and not blocking_boundaries,blocking_boundaries=blocking_boundaries,development_checks_passed=development_passed,suite_errors=errors,
        functional_completion=functional,denominators=dict(Counter(c['expectation'] for c in suite['cases'])),
        limitations=limitations,assessment=assessments,public_replays=replays,whole_block6_complete=False,
        scope='cross-mode continuous-control safety DEVELOPMENT verification, static calibrated simulation; known deferred S2/repeated texture boundaries explicitly NOT accepted')
    verify_freeze(freeze);out=Path(output);out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
    return report
