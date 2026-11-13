import copy
import pytest
from simulation.continuous_straight_acceptance import assess_case,summarize_suite


def fixture():
    case={'id':'normal','expectation':'normal','tags':['offset','yaw']}
    r=dict(exception=None,audit=dict(safe_final_stop=True,wall_contact_physics_steps=0,physics_audit_dt_s=1/120,maximum_audited_speed_m_s=.15,bootstrap_max_displacement_m=.1,braking_episodes=[],node_arrival_status='unverified',normal_geometric_stop_pass=True,displacement_m=2.),controller_result=dict(arrival_status='unverified',atomic_edge_compatible=False,state='visual_stop',result_type='completed_visual_stop',motion_commands=10))
    decisions=[dict(sim_s=1.,input={'captured_sim_s':1.,'frame_id':'f1'},decision=dict(v_m_s=.1,omega_rad_s=0.,arrival_status='unverified',atomic_edge_compatible=False,control_frame_id='f1',captured_sim_s=1.,decision_sim_s=1.))]
    return case,r,decisions


def test_normal_all_refusal_is_not_functional_completion():
    c,r,d=fixture();r['audit']['normal_geometric_stop_pass']=False;r['controller_result']['result_type']='aborted_motion_uncertain'
    a=assess_case(c,r,d);out=summarize_suite([c],[a])
    assert not out['passed'] and out['normal_completion']=={'completed':0,'total':1}
    assert 'normal_completion_below80percent' in out['suite_errors']


@pytest.mark.parametrize('fault',['contact','speed','rate','bootstrap','claim','age','brake'])
def test_audited_hazards_never_get_counted_as_passing(fault):
    c,r,d=fixture()
    if fault=='contact':r['audit']['wall_contact_physics_steps']=1
    elif fault=='speed':r['audit']['maximum_audited_speed_m_s']=.2
    elif fault=='rate':r['audit'].pop('physics_audit_dt_s')
    elif fault=='bootstrap':r['audit']['bootstrap_max_displacement_m']=.29
    elif fault=='claim':r['controller_result']['arrival_status']='arrived'
    elif fault=='age':d[0]['input']['captured_sim_s']=.6
    elif fault=='brake':r['audit']['braking_episodes']=[dict(latency_s=.4,distance_m=.02)]
    assert assess_case(c,r,d)['errors']


def test_missing_safety_groups_fail_even_with_a_normal_success():
    c,r,d=fixture();a=assess_case(c,r,d)
    out=summarize_suite([c],[a]);assert not out['passed']
    assert out['normal_completion']['completed']==1
    assert any(x.startswith('missing_required_denominator') for x in out['suite_errors'])


def test_external_budget_has_its_own_partial_motion_stop_semantics():
    c,r,d=fixture();c.update(id='budget',expectation='cancel_stop',max_ticks=12)
    r['audit']['normal_geometric_stop_pass']=False
    r['controller_result'].update(state='aborted',reason='execution_cancelled',result_type='aborted_motion_uncertain')
    d[-1].update(external_event='tick_budget_exhausted',input=None)
    d[-1]['decision'].update(v_m_s=0.,omega_rad_s=0.,reason='execution_cancelled')
    a=assess_case(c,r,d)
    assert a['errors']==[] and not a['normal_completed']


def test_corner_gate_never_counts_a_safe_refusal_as_normal_completion():
    from simulation.continuous_straight_acceptance import assess_corner_case
    result=dict(exception=None,corner_geometry=dict(width_m=1.8,corner_x_m=2.5),controller_result=dict(
        result_type='aborted_motion_uncertain',state='aborted',turn_angle_rad=0.,motion_commands=1,
        arrival_status='unverified',atomic_edge_compatible=False),audit=dict(
        safe_final_stop=True,wall_contact_physics_steps=0,physics_audit_dt_s=1/120,
        maximum_audited_speed_m_s=.15,maximum_audited_yaw_rate_rad_s=.3,braking_episodes=[],
        node_arrival_status='unverified',displacement_m=.1,
        final=dict(position_m=[.6,0.,.16],yaw_deg=0.,speed_m_s=0.,yaw_rate_rad_s=0.)))
    a=assess_corner_case(dict(id='normal',direction='left',expectation='normal'),result,[])
    assert not a['normal_completed']
    assert a['physical_safe']


def test_corner_midturn_fault_gate_requires_real_partial_rotation():
    from simulation.continuous_straight_acceptance import assess_corner_case
    result=dict(exception=None,corner_geometry=dict(width_m=1.8,corner_x_m=2.5),controller_result=dict(
        result_type='aborted_motion_uncertain',state='aborted',turn_angle_rad=.5,motion_commands=1,
        arrival_status='unverified',atomic_edge_compatible=False),audit=dict(
        safe_final_stop=True,wall_contact_physics_steps=0,physics_audit_dt_s=1/120,
        maximum_audited_speed_m_s=.15,maximum_audited_yaw_rate_rad_s=.3,braking_episodes=[],
        node_arrival_status='unverified',displacement_m=.1,
        final=dict(position_m=[.6,0.,.16],yaw_deg=0.,speed_m_s=0.,yaw_rate_rad_s=0.)))
    d=dict(state='aborted',phase='turn',turn_angle_rad=.5,v_m_s=0.,omega_rad_s=0.,arrival_status='unverified',atomic_edge_compatible=False)
    rows=[dict(tick=165,injected_fault='missing',decision=d)]
    case=dict(id='fault',direction='left',expectation='fault_stop',fault=dict(kind='missing',tick=165))
    a=assess_corner_case(case,result,rows)
    assert 'physical_partial_turn_not_exercised' in a['errors']


def junction_assessment_fixture():
    c=dict(id='normal-t',direction='left',expectation='normal')
    r=dict(version='continuous_t_junction_run_v1',exception=None,corner_geometry=dict(width_m=1.8,corner_x_m=2.5),controller_result=dict(
        result_type='completed_visual_junction',state='visual_stop',junction='t',direction='left',turn_angle_rad=1.55,motion_commands=30,
        arrival_status='unverified',atomic_edge_compatible=False),audit=dict(
        safe_final_stop=True,wall_contact_physics_steps=0,physics_audit_dt_s=1/120,
        maximum_audited_speed_m_s=.15,maximum_audited_yaw_rate_rad_s=.3,braking_episodes=[],fixture_events=[],
        node_arrival_status='unverified',displacement_m=2.,
        final=dict(position_m=[2.5,1.3,.16],yaw_deg=90.,speed_m_s=0.,yaw_rate_rad_s=0.)))
    rows=[]
    for i,phase in enumerate(('entry','entry_stop','entry_stop','entry_stop','turn','turn_stop','exit','exit_stop')):
        perception={'branches':{'left':'open','right':'open'}} if 2<=i<=4 else {}
        d=dict(state='moving',phase=phase,reason='fixture',turn_angle_rad=0.,v_m_s=.1 if phase in ('entry','exit') else 0.,omega_rad_s=.3 if phase=='turn' else 0.,
               arrival_status='unverified',atomic_edge_compatible=False,control_frame_id=f'f{i}',captured_sim_s=i*.1,decision_sim_s=i*.1,perception=perception)
        rows.append(dict(tick=i,sim_s=i*.1,input=dict(frame_id=f'f{i}',captured_sim_s=i*.1),injected_fault=None,decision=d))
    return c,r,rows


def test_t_completion_requires_independent_geometry_and_full_visual_sequence():
    from simulation.continuous_straight_acceptance import assess_corner_case
    c,r,d=junction_assessment_fixture()
    a=assess_corner_case(c,r,d,junction=True)
    assert not a['errors'] and a['normal_completed']
    r['audit']['final']['position_m'][1]=.9
    assert not assess_corner_case(c,r,d,junction=True)['normal_completed']


@pytest.mark.parametrize('mutation',['one-frame','unknown','closed','reroute','switched-plan','phase','node','contact','brake','refusal'])
def test_t_evaluator_does_not_accept_script_flags_or_weakened_branch_evidence(mutation):
    from simulation.continuous_straight_acceptance import assess_corner_case
    c,r,d=junction_assessment_fixture()
    if mutation=='one-frame':d[2]['decision']['perception']={}
    elif mutation=='unknown':d[3]['decision']['perception']['branches']['right']='unknown'
    elif mutation=='closed':
        for i in (2,3,4):d[i]['decision']['perception']['branches']['left']='blocked'
    elif mutation=='reroute':d[4]['decision']['omega_rad_s']=-.3
    elif mutation=='switched-plan':r['controller_result']['direction']='right'
    elif mutation=='phase':d[6]['decision']['phase']='exit_stop'
    elif mutation=='node':r['controller_result']['arrival_status']='arrived'
    elif mutation=='contact':r['audit']['wall_contact_physics_steps']=1
    elif mutation=='brake':r['audit']['braking_episodes']=[dict(latency_s=None,distance_m=None)]
    elif mutation=='refusal':r['controller_result'].update(result_type='aborted_motion_uncertain',state='aborted')
    a=assess_corner_case(c,r,d,junction=True)
    assert a['errors'] or not a['normal_completed']


def test_t_temporary_obstruction_must_be_observed_waited_cleared_and_reobserved():
    from simulation.continuous_straight_acceptance import assess_corner_case
    c,r,d=junction_assessment_fixture();c.update(expectation='clear_after_wait',clear_blocker_tick=2)
    a=assess_corner_case(c,r,d,junction=True)
    assert 'selected_block_never_observed' in a['errors']
    assert 'temporary_block_not_cleared_at_declared_tick' in a['errors']


def route_assessment_fixture():
    positions={'S':[0.,0.],'A':[0.,1.],'G':[1.,1.]}
    case=dict(id='route',expectation='normal',start_node='S',goal_node='G',heading_deg=0,physical_scene=dict(geometry=dict(node_positions_m=positions)))
    rows=[];truth=[];proofs=[]
    for j,(node,heading) in enumerate([('S',0),('A',90),('G',0)]):
        ids=[]
        for k in range(3):
            t=j*3.+k*.1;fid=f'{j}-{k}';ids.append(fid)
            d=dict(state='observing',v_m_s=0.,omega_rad_s=0.,atomic_edge_compatible=False,arrival_status='unverified',node_estimate=None,pending_command=None)
            rows.append(dict(tick=len(rows),sim_s=t,input=dict(frame_id=fid,captured_sim_s=t),decision=d))
            truth.append(dict(sim_s=t,position_m=[*positions[node],.16],yaw_deg=heading,speed_m_s=0.,yaw_rate_rad_s=0.))
        proofs.append(dict(node_id=node,heading_deg=heading,frame_ids=ids,captured_sim_s=t,command_id=None if j==0 else f'cmd{j}'))
    commands=[dict(command_id='cmd1',source='S',target='A'),dict(command_id='cmd2',source='A',target='G')]
    events=[dict(event_id='e1',command_id='cmd1',captured_sim_s=1.),dict(event_id='e2',command_id='cmd2',captured_sim_s=4.)]
    c=dict(result_type='completed_continuous_route',state='visual_stop',reason='goal',node_estimate=dict(node_id='G'),position_status='visually_confirmed_map_node',arrival_status='visually_confirmed_map_node',atomic_edge_compatible=False,heading_deg=0,node_confirmations=proofs,commands=commands,consumed_segment_events=events,command_count=2,transition_count=2,duplicate_feedback=0,motion_commands=20)
    rows[-1]['decision'].update(state='visual_stop',node_estimate=dict(node_id='G'),arrival_status='visually_confirmed_map_node')
    a=dict(safe_final_stop=True,wall_contact_physics_steps=0,physics_audit_dt_s=1/120,maximum_audited_speed_m_s=.15,maximum_audited_yaw_rate_rad_s=.3,braking_episodes=[],initial=truth[0],final=truth[-1],displacement_m=1.4)
    return case,dict(exception=None,controller_result=c,audit=a),rows,truth


def test_route_assessment_requires_real_multinode_completion_not_safe_refusal():
    from simulation.continuous_straight_acceptance import assess_route_case
    case,r,rows,truth=route_assessment_fixture()
    a=assess_route_case(case,r,rows,truth)
    assert a['functional_completed'] and not a['errors']
    r['controller_result'].update(result_type='aborted_motion_uncertain',state='aborted',node_estimate=None,position_status='uncertain',arrival_status='unverified')
    assert not assess_route_case(case,r,rows,truth)['functional_completed']


@pytest.mark.parametrize('hazard',['wrong_node_pose','moving_proof','pre_exit','reused_frames','premature_arrival','segment_arrival','duplicate_event','stale_motion','atomic','contact','brake','transition','certain_abort'])
def test_route_assessment_rejects_false_or_unsafe_node_claims(hazard):
    from simulation.continuous_straight_acceptance import assess_route_case
    case,r,rows,truth=route_assessment_fixture();c=r['controller_result'];a=r['audit']
    if hazard=='wrong_node_pose':truth[3]['position_m']=[9.,9.,.16]
    elif hazard=='moving_proof':truth[3]['speed_m_s']=.1
    elif hazard=='pre_exit':c['consumed_segment_events'][0]['captured_sim_s']=5.
    elif hazard=='reused_frames':c['node_confirmations'][1]['frame_ids']=['1-0']*3
    elif hazard=='premature_arrival':rows[0]['decision']['arrival_status']='visually_confirmed_map_node'
    elif hazard=='segment_arrival':rows[0]['segment_output']=dict(node_estimate={'node_id':'A'},arrival_status='visually_confirmed_map_node')
    elif hazard=='duplicate_event':c['consumed_segment_events'].append(c['consumed_segment_events'][0])
    elif hazard=='stale_motion':rows[0]['decision']['v_m_s']=.1;rows[0]['sim_s']=1.
    elif hazard=='atomic':c['atomic_edge_compatible']=True
    elif hazard=='contact':a['wall_contact_physics_steps']=1
    elif hazard=='brake':a['braking_episodes']=[dict(latency_s=.4,distance_m=.01)]
    elif hazard=='transition':c['transition_count']=3
    else:c.update(result_type='aborted_motion_uncertain',state='aborted')
    assert assess_route_case(case,r,rows,truth)['errors']


def safety_physics_fixture():
    # Perfectly regular 120Hz trace with one explicit stop request at .10s.
    trace=[]
    for i in range(37):
        t=i/120;moving=i<=12
        trace.append(dict(sim_s=t,position_m=[min(t,.10)*.1,0.,.16],speed_m_s=.1 if moving else 0.,yaw_rate_rad_s=0.,wall_contact_physics_steps=0))
    a=dict(initial=dict(trace[0]),final=dict(trace[-1]),wall_contact_physics_steps=0,maximum_audited_speed_m_s=.1,maximum_audited_yaw_rate_rad_s=0.)
    result=dict(audit=a,controller_result=dict(motion_commands=1))
    rows=[dict(tick=0,sim_s=0.,decision=dict(v_m_s=.1,omega_rad_s=0.)),dict(tick=1,sim_s=.1,decision=dict(v_m_s=0.,omega_rad_s=0.))]
    return result,rows,trace


def test_s6_braking_is_recomputed_from_physics_not_pass_booleans():
    from simulation.continuous_straight_acceptance import audit_safety_physics
    r,rows,trace=safety_physics_fixture()
    a=audit_safety_physics(r,rows,trace)
    assert not a['errors'] and a['braking'][0]['latency_s']==pytest.approx(1/120)
    # Falsifying the runner's summary does not alter the independent result.
    r['audit']['braking_episodes']=[dict(latency_s=999,distance_m=999)]
    assert audit_safety_physics(r,rows,trace)==a


@pytest.mark.parametrize('hazard',['gap','speed','contact','final_motion','no_stop','missing_trace','bad_peak','bad_end','distance'])
def test_s6_independent_trace_gate_rejects_concealed_hazards(hazard):
    from simulation.continuous_straight_acceptance import audit_safety_physics
    r,rows,t=safety_physics_fixture()
    if hazard=='gap':t.pop(6)
    elif hazard=='speed':t[6]['speed_m_s']=.3
    elif hazard=='contact':t[6]['wall_contact_physics_steps']=1
    elif hazard=='final_motion':r['audit']['final']['speed_m_s']=.2
    elif hazard=='no_stop':
        for p in t:p['speed_m_s']=.1
    elif hazard=='missing_trace':t=[]
    elif hazard=='bad_peak':r['audit']['maximum_audited_speed_m_s']=0.
    elif hazard=='bad_end':t.pop()
    elif hazard=='distance':t[13]['position_m'][0]=1.
    assert audit_safety_physics(r,rows,t)['errors']


def test_s6_an_untextured_or_repeated_refusal_cannot_be_relabelled_normal_success():
    from simulation.continuous_straight_acceptance import assess_safety_case
    r,rows,trace=safety_physics_fixture()
    # A safely rejected normal task remains a FUNCTIONAL failure.
    c=r['controller_result'];c.update(state='aborted',result_type='aborted_motion_uncertain',reason='unknown',atomic_edge_compatible=False,arrival_status='unverified')
    r.update(exception=None,scene=dict(length_m=4.,obstacle='none'),safety_checks=dict(terminal_good_frame_checks=3,terminal_latched=True,caught_input_errors=[]))
    r['audit']['final'].update(camera_position_m=[.2,0.,.45],yaw_deg=0.)
    for row in rows:
        row.update(input=dict(frame_id='f'+str(row['tick']),captured_sim_s=row['sim_s']))
        row['decision'].update(atomic_edge_compatible=False,arrival_status='unverified',control_frame_id=row['input']['frame_id'],captured_sim_s=row['sim_s'],decision_sim_s=row['sim_s'])
    a=assess_safety_case(dict(id='normal',mode='straight',expectation='normal',arguments={}),r,rows,[],trace)
    assert not a['functional_completed']
    a=assess_safety_case(dict(id='boundary',mode='straight',expectation='texture_boundary',arguments={}),r,rows,[],trace)
    assert not a['functional_completed'] and 'repeated_texture_observability_not_certified' in a['limitations']
