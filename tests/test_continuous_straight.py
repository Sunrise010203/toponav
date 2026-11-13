import numpy as np
import pytest
from roadvision.continuous_straight import StraightController,InitialClearancePrior,StraightPolicy


def frame(i=0):return dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb=np.zeros((256,256,3),np.uint8))


def test_no_task_clearance_prior_or_missing_rgb_never_launches():
    c=StraightController();r=c.update(frame(),0.)
    assert r['v_m_s']==r['omega_rad_s']==0.
    assert r['arrival_status']=='unverified'
    assert c.missing(.1)['v_m_s']==0.


def test_truth_fields_are_rejected_and_partial_motion_not_atomic_failed():
    c=StraightController(initial_clearance=InitialClearancePrior(confirmed=True))
    with pytest.raises(ValueError):c.update({**frame(),'true_pose':[0,0]},0.)
    assert c.result()['atomic_edge_compatible'] is False
    assert c.result()['arrival_status']=='unverified'


def test_uniform_camera_cannot_bootstrap_even_with_operator_prior():
    c=StraightController(initial_clearance=InitialClearancePrior(confirmed=True))
    for i in range(4):
        f=frame(i);f['rgb'][:]=128;r=c.update(f,i*.1)
        assert r['v_m_s']==0.


@pytest.mark.parametrize('field,value',[('max_speed_m_s',.6),('max_omega_rad_s',1.),('bootstrap_duration_s',2.),('max_frame_age_s',1.),('stationary_confirmations',1),('max_duration_s',float('nan'))])
def test_control_overrides_cannot_bypass_validated_limits(field,value):
    with pytest.raises(ValueError):StraightPolicy(**{field:value})


# S3 source-level tests: old StraightController assertions above remain intact.
def corner_perception(distance=2.,width=1.8,wall_angle=None,open_branch=True):
    import math
    def line(n,d):return dict(normal=n,distance_m=d,count=17)
    views={k:dict(line=line(n,d),points=[(n[0]*d,n[1]*d)]) for k,n,d in (
        ('front',[1.,0.],distance),('left',[0.,1.],width/2),
        ('right',[0.,-1.],width/2),('back',[-1.,0.],1.))}
    if wall_angle is None:views['left']['line']['distance_m']=width/2
    wall=line([math.cos(wall_angle or 0.),math.sin(wall_angle or 0.)],.75)
    return dict(status='conditional_colour_footprint',front=views['front']['line'],views=views,lines=[wall],path_clear=True)


def corner_started(monkeypatch,direction='left'):
    from roadvision import continuous_straight as m
    current=[corner_perception()]
    monkeypatch.setattr(m,'analyze_corner_rgb',lambda _:current[0])
    c=m.CornerController(direction=direction,initial_clearance=m.InitialClearancePrior(confirmed=True))
    image=np.random.default_rng(55).integers(0,255,(256,256,3),dtype=np.uint8)
    def step(i):return c.update(dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb=image),i*.1)
    for i in range(3):r=step(i)
    assert r['v_m_s']>0 and c.phase=='entry'
    return c,current,step,image


def test_corner_pixel_quantisation_is_fitted_in_image_not_staircase_metres():
    from roadvision.continuous_straight import _corner_footprint_line
    u=np.arange(8,121,2)
    pixels=np.column_stack((u,63.5+np.round(8.+.02*(u-63.5))))
    result=_corner_footprint_line(pixels,'front')
    assert result is not None and result['pixel_p90']<=.65
    assert result['normal'][0]>.98 and 3.<result['distance_m']<4.


def test_corner_curved_or_unsupported_boundary_is_unknown():
    from roadvision.continuous_straight import _corner_footprint_line
    u=np.arange(8,121,2)
    assert _corner_footprint_line(np.column_stack((u,70.+.01*(u-64.)**2)),'front') is None
    assert _corner_footprint_line(np.zeros((7,2)),'front') is None


@pytest.mark.parametrize('direction',['left','right'])
def test_corner_entry_turn_exit_sequence_is_visual_not_timed(monkeypatch,direction):
    import math
    c,p,step,_=corner_started(monkeypatch,direction)
    sign=1 if direction=='left' else -1
    p[0]=corner_perception(distance=.72,wall_angle=0.)
    p[0]['views'][direction]['line']['distance_m']=3.
    assert step(3)['phase']=='entry_stop'
    for i in range(4,7):r=step(i)
    assert r['phase']=='turn' and sign*r['omega_rad_s']>0
    for j,i in enumerate(range(7,17),1):
        p[0]=corner_perception(distance=3.,wall_angle=-sign*j*math.pi/20)
        r=step(i)
    assert r['phase']=='turn_stop' and r['omega_rad_s']==0.
    for i in range(17,20):r=step(i)
    assert r['phase']=='exit' and r['v_m_s']>0
    p[0]=corner_perception(distance=1.5,wall_angle=-sign*math.pi/2)
    for view,side in [('left',1),('right',-1)]:
        p[0]['views'][view]['points']=[(x,side*.9) for x in (-.62,-.58,-.54,.44,.48,.52)]
    assert step(20)['phase']=='exit_stop'
    for i in range(21,24):r=step(i)
    assert r['state']=='visual_stop'
    assert c.result()['result_type']=='completed_visual_corner'
    assert not c.result()['atomic_edge_compatible'] and c.result()['arrival_status']=='unverified'


@pytest.mark.parametrize('fault',['missing','replay','stale','future','order','gap','clock','truth','unknown','exception','cancel'])
def test_corner_partial_motion_faults_zero_and_remain_uncertain(monkeypatch,fault):
    from roadvision import continuous_straight as m
    c,p,step,image=corner_started(monkeypatch)
    f=dict(frame_id='f3',captured_sim_s=.3,rgb=image);now=.3
    if fault=='missing':r=c.missing(now)
    elif fault=='cancel':r=c.cancel(now)
    elif fault in ('truth','exception'):
        if fault=='truth':f['true_pose']=0;error=ValueError
        else:
            def broken(_):raise RuntimeError('perception failed')
            monkeypatch.setattr(m,'analyze_corner_rgb',broken);error=RuntimeError
        with pytest.raises(error):c.update(f,now)
        r=c.missing(now)
    else:
        if fault=='replay':f['frame_id']='f2'
        elif fault=='stale':f['captured_sim_s']=0.
        elif fault=='future':f['captured_sim_s']=.4
        elif fault=='order':f['captured_sim_s']=.2
        elif fault=='gap':now=.6;f['captured_sim_s']=.6
        elif fault=='clock':now=.1
        elif fault=='unknown':p[0]=dict(status='unknown',reason='no_boundary')
        r=c.update(f,now)
    assert r['state']=='aborted' and r['v_m_s']==r['omega_rad_s']==0.
    assert c.result()['result_type']=='aborted_motion_uncertain'
    assert c.update(dict(frame_id='later',captured_sim_s=now+.1,rgb=image),now+.1)['v_m_s']==0.
    assert c.result()['arrival_status']=='unverified'


def test_corner_elapsed_time_alone_cannot_complete(monkeypatch):
    c,p,step,image=corner_started(monkeypatch)
    for i in range(3,50):r=step(i)
    assert c.phase=='entry' and c.state=='moving'
    r=c.update(dict(frame_id='late',captured_sim_s=75.,rgb=image),75.)
    assert r['reason']=='motion_timeout' and r['v_m_s']==r['omega_rad_s']==0.
    assert c.result()['result_type']=='aborted_motion_uncertain'


def test_corner_rejects_visually_closed_planned_branch(monkeypatch):
    c,p,step,_=corner_started(monkeypatch)
    p[0]=corner_perception(distance=.72,wall_angle=0.,open_branch=False)
    step(3)
    for i in range(4,7):r=step(i)
    assert r['reason']=='planned_branch_not_visually_open' and r['omega_rad_s']==0.


def test_corner_without_prior_or_bad_rgb_never_moves():
    from roadvision.continuous_straight import CornerController
    c=CornerController(direction='left')
    assert c.update(frame(),0.)['v_m_s']==0.
    assert c.result()['result_type']=='rejected_no_motion'
    with pytest.raises(ValueError):CornerController(direction='straight')


def test_corner_far_range_jump_cannot_replace_observed_clear_exit(monkeypatch):
    c,p,step,_=corner_started(monkeypatch)
    c.phase='exit';c.exit_initial_range=3.
    p[0]=corner_perception(distance=1.5,wall_angle=0.)
    for side in ('left','right'):
        p[0]['views'][side]['line']['support_min_forward_m']=0.
        p[0]['views'][side]['line']['support_max_forward_m']=.8
    # Even a large far-range jump cannot show that the opening is behind us.
    r=step(3)
    assert r['phase']=='exit' and r['state']=='moving' and r['v_m_s']>0.


def test_t_junction_approach_uses_actual_rear_incoming_boundaries(monkeypatch):
    from roadvision import continuous_straight as m
    current=[corner_perception()]
    monkeypatch.setattr(m,'analyze_corner_rgb',lambda _,**kwargs:current[0])
    c=m.CornerController(direction='left',junction='t',initial_clearance=InitialClearancePrior(confirmed=True))
    for i in range(3):r=c.update(frame(i),i*.1)
    assert r['v_m_s']>0
    p=corner_perception(distance=1.2)
    p['lines']=[p['views']['left']['line'],p['views']['right']['line']]
    p['views']['left']['line']=p['views']['right']['line']=None
    current[0]=p
    r=c.update(frame(3),3*.1)
    assert r['v_m_s']>0 and r['phase']=='entry'
    current[0]['lines']=[]
    r=c.update(frame(4),.4)
    assert r['state']=='aborted' and r['v_m_s']==r['omega_rad_s']==0.


def test_t_junction_requires_both_branch_observations_and_consistency(monkeypatch):
    from roadvision import continuous_straight as m
    current=[corner_perception()]
    monkeypatch.setattr(m,'analyze_corner_rgb',lambda _,**kwargs:current[0])
    monkeypatch.setattr(m,'_junction_branches',lambda *args:current[0]['branches'])
    c=m.CornerController(direction='right',junction='t',initial_clearance=InitialClearancePrior(confirmed=True))
    for i in range(3):c.update(frame(i),i*.1)
    c.state='braking';c.phase='entry_stop';c.brake_started=.3
    current[0]=corner_perception(distance=.75)
    current[0]['branches']={'left':'open','right':'open'}
    for i in range(3,7):r=c.update(frame(i),i*.1)
    assert r['omega_rad_s']==0. and c.branch_observations<3
    r=c.update(frame(7),7*.1)
    assert r['omega_rad_s']<0 and r['phase']=='turn'


@pytest.mark.parametrize('direction',['left','right'])
def test_t_junction_planned_block_waits_zero_then_needs_three_clear_frames(monkeypatch,direction):
    from roadvision import continuous_straight as m
    current=corner_perception(distance=.75)
    monkeypatch.setattr(m,'analyze_corner_rgb',lambda _,**kwargs:current)
    branches={'left':'open','right':'open'};branches[direction]='blocked'
    monkeypatch.setattr(m,'_junction_branches',lambda *args:branches.copy())
    c=m.CornerController(direction=direction,junction='t',initial_clearance=InitialClearancePrior(confirmed=True))
    c.width_m=1.8;c.state='braking';c.phase='entry_stop';c.brake_started=0.
    for i in range(6):
        r=c.update(frame(i),i*.1)
        assert r['v_m_s']==r['omega_rad_s']==0.
    assert r['reason']=='waiting_for_planned_branch_clear' and c.branch_observations==0
    branches[direction]='open'
    for i in range(6,8):
        r=c.update(frame(i),i*.1)
        assert r['v_m_s']==r['omega_rad_s']==0.
    r=c.update(frame(8),8*.1)
    assert r['phase']=='turn' and (1 if direction=='left' else -1)*r['omega_rad_s']>0


def test_t_junction_branch_unknown_and_persistent_block_never_reroute(monkeypatch):
    from roadvision import continuous_straight as m
    current=corner_perception(distance=.75)
    monkeypatch.setattr(m,'analyze_corner_rgb',lambda _,**kwargs:current)
    for status in ('unknown','blocked'):
        monkeypatch.setattr(m,'_junction_branches',lambda *args:{'left':status,'right':'open'})
        c=m.CornerController(direction='left',junction='t',initial_clearance=InitialClearancePrior(confirmed=True))
        c.width_m=1.8;c.state='braking';c.phase='entry_stop';c.brake_started=0.
        for i in range(52):r=c.update(frame(i),i*.1)
        assert r['state']=='aborted' and c.motion_commands==0
        assert c.direction=='left' and c.turn_angle_rad==0.


@pytest.mark.parametrize('fault',['missing','replay','stale','order','gap','truth','cancel'])
def test_t_junction_has_same_fresh_frame_and_partial_motion_semantics(monkeypatch,fault):
    from roadvision import continuous_straight as m
    monkeypatch.setattr(m,'analyze_corner_rgb',lambda _,**kwargs:corner_perception())
    c=m.CornerController(direction='left',junction='t',initial_clearance=InitialClearancePrior(confirmed=True))
    for i in range(3):c.update(frame(i),i*.1)
    f=frame(3);now=3*.1
    if fault=='missing':r=c.missing(now)
    elif fault=='cancel':r=c.cancel(now)
    elif fault=='truth':
        with pytest.raises(ValueError):c.update({**f,'node_id':'truth'},now)
        r=c.missing(now)
    else:
        if fault=='replay':f['frame_id']='f2'
        elif fault=='stale':f['captured_sim_s']=0.
        elif fault=='order':f['captured_sim_s']=.2
        elif fault=='gap':now=.6;f['captured_sim_s']=.6
        r=c.update(f,now)
    assert r['state']=='aborted' and r['v_m_s']==r['omega_rad_s']==0.
    assert c.result()['result_type']=='aborted_motion_uncertain'
    assert c.result()['arrival_status']=='unverified' and not c.result()['atomic_edge_compatible']


def test_t_junction_has_no_straight_exit_and_rejects_unknown_installation_mode():
    from roadvision.continuous_straight import CornerController
    with pytest.raises(ValueError):CornerController(direction='straight',junction='t')
    with pytest.raises(ValueError):CornerController(direction='left',junction='crossroads')


@pytest.mark.parametrize('direction',['left','right'])
@pytest.mark.parametrize('visible_side',['left','right'])
def test_t_can_track_either_observed_incoming_wall_not_the_task_outer_side(monkeypatch,direction,visible_side):
    from roadvision import continuous_straight as m
    current=[corner_perception()]
    monkeypatch.setattr(m,'analyze_corner_rgb',lambda _,**kwargs:current[0])
    c=m.CornerController(direction=direction,junction='t',initial_clearance=InitialClearancePrior(confirmed=True))
    for i in range(3):c.update(frame(i),i*.1)
    current[0]=corner_perception(distance=1.1)
    current[0]['lines']=[current[0]['views'][visible_side]['line']]
    for side in ('left','right'):current[0]['views'][side]['line']=None
    r=c.update(frame(3),3*.1)
    assert r['v_m_s']>0 and abs(r['omega_rad_s'])<.01


def test_t_branch_evidence_uses_full_projected_floor_region_not_a_far_wall_fit():
    from roadvision.continuous_straight import _junction_branches
    rgb=np.full((256,256,3),[80,90,120],dtype=np.uint8)
    p=corner_perception()
    for side in ('left','right'):p['views'][side]['line']['distance_m']=3.
    assert _junction_branches(rgb,1.8)=={'left':'open','right':'open'}
    rgb[:128,128:]=128
    assert _junction_branches(rgb,1.8)=={'left':'blocked','right':'open'}
    p['views']['right']['line']=None
    assert _junction_branches(rgb,1.8)['right']=='open'
    assert _junction_branches(rgb,-2.)['right']=='unknown'


def test_t_single_nonfloor_pixel_inside_projected_lane_cannot_hide_between_sparse_probes():
    from roadvision.continuous_straight import _junction_branches
    rgb=np.full((256,256,3),[80,90,120],dtype=np.uint8)
    assert _junction_branches(rgb,1.8)['left']=='open'
    rgb[100,128+68]=128
    assert _junction_branches(rgb,1.8)['left']=='blocked'


def test_t_interrupted_stationarity_discards_previously_counted_branch_frames(monkeypatch):
    from roadvision import continuous_straight as m
    current=corner_perception(distance=.75)
    monkeypatch.setattr(m,'analyze_corner_rgb',lambda _,**kwargs:current)
    monkeypatch.setattr(m,'_junction_branches',lambda *args:{'left':'open','right':'open'})
    c=m.CornerController(direction='left',junction='t',initial_clearance=InitialClearancePrior(confirmed=True))
    c.width_m=1.8;c.state='braking';c.phase='entry_stop';c.brake_started=0.
    for i in range(5):r=c.update(frame(i),i*.1)
    assert c.branch_observations==2 and r['omega_rad_s']==0.
    # A changing image breaks consecutive stationary branch evidence.
    for i in range(5,11):
        f=frame(i);f['rgb'][:]=10
        r=c.update(f,i*.1)
        if i<10:assert r['omega_rad_s']==0.
    assert r['phase']=='turn' and r['omega_rad_s']>0.


def test_route_can_hold_a_visually_reached_junction_without_a_premature_turn(monkeypatch):
    from roadvision import continuous_straight as m
    current=corner_perception(distance=.72)
    monkeypatch.setattr(m,'analyze_corner_rgb',lambda _,**kwargs:current)
    c=m.CornerController(direction=None,junction='t',pause_at_entry=True,initial_clearance=InitialClearancePrior(confirmed=True))
    for i in range(10):r=c.update(frame(i),i*.1)
    assert c.entry_ready and r['v_m_s']==r['omega_rad_s']==0.
    assert c.turn_angle_rad==0. and c.direction is None
    c.resume_turn('right')
    assert c.direction=='right' and not c.entry_ready


def route_unit(monkeypatch):
    """Only perception mocked here; actual public planner and executor state."""
    from roadvision import continuous_straight as m
    from simulation.path_planning import plan_from_estimate,next_action
    nodes=[dict(node_id='S',exits=[dict(neighbor_id='A',channel_id='sa',direction='north')]),
           dict(node_id='A',exits=[dict(neighbor_id='S',channel_id='sa',direction='south'),dict(neighbor_id='G',channel_id='ag',direction='east')]),
           dict(node_id='G',exits=[dict(neighbor_id='A',channel_id='ag',direction='west')])]
    for n in nodes:n.update(neighbors=[e['neighbor_id'] for e in n['exits']],node_type='corner' if n['node_id']=='A' else 'dead_end')
    graph=dict(schema_version='2.0',scene_id='unit-route',exit_direction_frame='map_xy_east_north',directed=False,nodes=nodes,edges=[dict(channel_id='sa',source='S',target='A',bidirectional=True,enabled=True),dict(channel_id='ag',source='A',target='G',bidirectional=True,enabled=True)],task=dict(start_node='S',goal_node='G',initial_heading_deg=0))
    refs=[dict(node_id=n['node_id'],heading_deg=h,rgb=frame()['rgb']) for n in nodes for h in (0,90,180,270)]
    monkeypatch.setattr(m,'_route_features',lambda rgb:[])
    calls=[]
    def plan(*args):calls.append(args[1].copy());return plan_from_estimate(*args)
    c=m.ContinuousRouteController(graph=graph,references=refs,plan_from_estimate=plan,next_action=next_action,initial_clearance=InitialClearancePrior(confirmed=True))
    observed=['S',0]
    monkeypatch.setattr(m,'_route_place_matches',lambda *args:dict(matches=[dict(node_id=observed[0],heading_deg=observed[1])],best=[]))
    monkeypatch.setattr(m,'_route_openings',lambda *args:c._map_structure(*observed))
    return c,observed,calls


def route_hold(c):
    c.leaf.entry_ready=True;c.leaf.width_m=1.8;c.leaf.state='braking';c.leaf.phase='entry_stop'
    c.leaf.update=lambda f,t:dict(state='braking',phase='entry_stop',reason='test_rgb_hold',v_m_s=0.,omega_rad_s=0.)


def route_exit(c,i):
    c.leaf.update=lambda f,t:dict(state='visual_stop',phase='exit_stop',reason='test_rgb_exit',v_m_s=0.,omega_rad_s=0.)
    r=c.update(frame(i),i*.1)
    assert r['arrival_status']=='unverified' and r['node_estimate'] is None
    return r['pending_feedback']


def test_route_three_fresh_proofs_planner_and_two_edges_not_atomic_exits(monkeypatch):
    c,obs,calls=route_unit(monkeypatch);route_hold(c)
    for i in range(3):r=c.update(frame(i),i*.1)
    assert len(calls)==1 and c.command_count==1 and c.transitions==0
    assert c.pending['action']=='left' and r['node_estimate'] is None
    event=route_exit(c,3)
    c.feedback(event,3*.1);c.feedback(dict(event),3*.1)
    assert c.duplicate_feedback==1 and c.transitions==0 and c.node_estimate is None
    obs[:]=['A',90];route_hold(c)
    for i in range(4,7):c.update(frame(i),i*.1)
    assert c.transitions==1 and c.command_count==2 and len(calls)==2
    assert c.pending['action']=='right'
    event=route_exit(c,7);c.feedback(event,7*.1)
    assert c.transitions==1
    obs[:]=['G',0];route_hold(c)
    for i in range(8,11):r=c.update(frame(i),i*.1)
    assert c.transitions==2 and c.state=='visual_stop' and len(calls)==3
    assert c.result()['result_type']=='completed_continuous_route'
    assert r['arrival_status']=='visually_confirmed_map_node' and not r['atomic_edge_compatible']
    assert [p['frame_ids'] for p in c.node_confirmations]==[['f0','f1','f2'],['f4','f5','f6'],['f8','f9','f10']]


@pytest.mark.parametrize('kind',['missing','cancel','replay','stale','gap','extra_truth','invalid_time'])
def test_route_partial_failure_clears_pose_without_rollback(monkeypatch,kind):
    c,_,_=route_unit(monkeypatch);route_hold(c)
    for i in range(3):c.update(frame(i),i*.1)
    c.motion_commands=1
    if kind=='missing':r=c.missing(.3)
    elif kind=='cancel':r=c.cancel(.3)
    elif kind=='invalid_time':r=c.update(frame(3),float('nan'))
    elif kind=='extra_truth':
        with pytest.raises(ValueError):c.update(dict(frame(3),true_node='A'),3*.1)
        r=c.result()
    else:
        f=frame(2) if kind=='replay' else frame(3) if kind=='stale' else frame(8)
        r=c.update(f,.8 if kind in ('gap','stale') else .3)
    assert r['state']=='aborted' and c.node_estimate is None and c.transitions==0
    assert c.last_confirmed_node=='S' and c.result()['position_status']=='uncertain'
    assert c.result()['result_type']=='aborted_motion_uncertain'


@pytest.mark.parametrize('kind',['stale','unknown','conflict','drop','schema'])
def test_route_transport_rejects_unbound_feedback_and_never_promotes_node(monkeypatch,kind):
    c,_,_=route_unit(monkeypatch);route_hold(c)
    for i in range(3):c.update(frame(i),i*.1)
    event=route_exit(c,3)
    if kind=='drop':
        for i in range(4,14):r=c.update(frame(i),i*.1)
    else:
        if kind=='stale':event=dict(event,captured_sim_s=-.1)
        elif kind=='unknown':event=dict(event,command_id='other')
        elif kind=='schema':event=dict(event,node_id='A')
        elif kind=='conflict':c.feedback(event,3*.1);event=dict(event,status='other')
        r=c.feedback(event,3*.1)
    assert r['state']=='aborted' and c.node_estimate is None and c.transitions==0
    assert c.result()['arrival_status']=='unverified'


@pytest.mark.parametrize('observation',[['A',0],['S',90]])
def test_route_wrong_start_or_heading_not_corrected_by_task_label(monkeypatch,observation):
    c,obs,calls=route_unit(monkeypatch);obs[:]=observation;route_hold(c)
    r=c.update(frame(),0.)
    assert r['state']=='aborted' and c.reason=='node_or_heading_prior_conflict'
    assert c.command_count==0 and not calls and not c.node_confirmations


def test_route_ambiguous_reference_and_structure_disagreement_refuse(monkeypatch):
    from roadvision import continuous_straight as m
    for mode in ('ambiguous','structure'):
        c,_,calls=route_unit(monkeypatch);route_hold(c)
        if mode=='ambiguous':monkeypatch.setattr(m,'_route_place_matches',lambda *args:dict(matches=[dict(node_id='S',heading_deg=0)]*2,best=[]))
        else:monkeypatch.setattr(m,'_route_openings',lambda *args:dict(front=True,left=True,right=True,back=True))
        r=c.update(frame(),0.)
        assert r['state']=='aborted' and c.reason=='node_reference_or_structure_not_unique' and not calls


def test_route_root_freshness_survives_leaf_replacement(monkeypatch):
    c,_,_=route_unit(monkeypatch);route_hold(c)
    for i in range(3):c.update(frame(i),i*.1)
    event=route_exit(c,3);c.feedback(event,3*.1)
    r=c.update(frame(1),.4)
    assert r['state']=='aborted' and c.reason=='replayed_frame'


def test_route_public_references_require_exact_inventory_and_no_pose(monkeypatch):
    from roadvision import continuous_straight as m
    from simulation.path_planning import plan_from_estimate,next_action
    c,_,_=route_unit(monkeypatch)
    refs=[dict(node_id=n['node_id'],heading_deg=h,rgb=frame()['rgb']) for n in c.graph['nodes'] for h in (0,90,180,270)]
    for bad in (refs[:-1],refs+[refs[0]],[dict(r,position_m=[0,0]) for r in refs]):
        with pytest.raises(ValueError):m.ContinuousRouteController(graph=c.graph,references=bad,plan_from_estimate=plan_from_estimate,next_action=next_action)


def test_route_natural_appearance_requires_two_views_and_unique_place():
    from roadvision.continuous_straight import _route_features,_route_place_matches
    import cv2
    rng=np.random.default_rng(15)
    gray=rng.integers(50,190,(256,256),dtype=np.uint8)
    rgb=np.repeat(cv2.GaussianBlur(gray,(3,3),.5)[:,:,None],3,axis=2)
    features=_route_features(rgb)
    reference=dict(node_id='S',heading_deg=0,features=features)
    assert _route_place_matches(features,[reference])['matches'][0]['node_id']=='S'
    assert len(_route_place_matches(features,[reference,dict(reference,node_id='other')])['matches'])==2
    one=[features[0],*( [(np.empty((0,2)),None)]*3 )]
    assert not _route_place_matches(one,[reference])['matches']
    uniform=_route_features(np.full((256,256,3),128,np.uint8))
    assert not _route_place_matches(uniform,[reference])['matches']


def test_route_other_session_feedback_is_not_a_current_exit(monkeypatch):
    c,_,_=route_unit(monkeypatch);route_hold(c)
    for i in range(3):c.update(frame(i),i*.1)
    event=route_exit(c,3)
    event=dict(event,event_id='different-mission:cmd-0001:visual-exit',command_id='different-mission:cmd-0001')
    r=c.feedback(event,3*.1)
    assert r['state']=='aborted' and c.reason=='unbound_segment_feedback_pose_uncertain' and c.transitions==0


def test_route_graph_cannot_smuggle_metric_or_current_truth_fields(monkeypatch):
    from roadvision import continuous_straight as m
    from simulation.path_planning import plan_from_estimate,next_action
    import copy
    c,_,_=route_unit(monkeypatch)
    refs=[dict(node_id=n['node_id'],heading_deg=h,rgb=frame()['rgb']) for n in c.graph['nodes'] for h in (0,90,180,270)]
    for field in ('graph','node','edge','task'):
        g=copy.deepcopy(c.graph)
        if field=='graph':g['actual_node']='S'
        elif field=='node':g['nodes'][0]['position_m']=[0,0]
        elif field=='edge':g['edges'][0]['length_m']=3.
        else:g['task']['current_pose']=[0,0]
        with pytest.raises(ValueError):m.ContinuousRouteController(graph=g,references=refs,plan_from_estimate=plan_from_estimate,next_action=next_action)


def test_s6_feedback_timestamp_bool_cannot_alias_a_consumed_float_timestamp(monkeypatch):
    c,_,_=route_unit(monkeypatch);route_hold(c)
    for i in range(10):c.update(frame(i),i*.1)
    # A legitimate RGB exit captured at exactly 1.0s, not a truth arrival.
    event=route_exit(c,10);c.feedback(event,1.)
    c.feedback(dict(event,captured_sim_s=True),1.)
    assert c.state=='aborted' and c.duplicate_feedback==0
