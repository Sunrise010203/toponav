"""State-machine faults after partial motion; geometric outputs are stubbed."""
import numpy as np
import pytest
from roadvision import continuous_straight as module


def moving_controller(monkeypatch):
    monkeypatch.setattr(module,'_ground_evidence',lambda *_:({'status':'conditional_ground_motion','translation_m':[0.,0.],'yaw_delta_rad':0.},None,None))
    monkeypatch.setattr(module,'probe_straight_history',lambda *a,**k:bind_evidence(a[0], dict(corridor_geometry={'status':'conditional_corridor_geometry','heading_error_rad':0.,'offset_m':0.},front_surface={'status':'unknown'},ground_motion={'status':'unknown'})))
    monkeypatch.setattr(module,'assess_free_ground',lambda *a,**k:bind_evidence(a[0], {'status':'conditional_free_ground_path','conditional_stopping_clearance_m':.6}))
    c=module.StraightController(initial_clearance=module.InitialClearancePrior(confirmed=True),motion_permission='bounded_probe')
    rgb=np.random.default_rng(328).integers(0,255,(256,256,3),dtype=np.uint8)
    for i in range(9):
        rgb[255,255,0]=i
        r=c.update(dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb=rgb),i*.1)
    assert r['v_m_s']>0 and c.motion_commands>0
    return c,rgb


@pytest.mark.parametrize('fault',['missing','replay','stale','future','order','gap','time_regression','shape','flat','truth','exception'])
def test_faults_stop_after_partial_motion_without_atomic_or_arrival_claim(monkeypatch,fault):
    c,rgb=moving_controller(monkeypatch);now=.9
    f=dict(frame_id='f9',captured_sim_s=.9,rgb=rgb)
    if fault=='missing':r=c.missing(now)
    elif fault in ('truth','exception'):
        if fault=='truth':f['true_pose']=[1,2,3];error=ValueError
        else:
            def broken(*a,**k):raise RuntimeError('perception failure')
            monkeypatch.setattr(module,'probe_straight_history',broken);error=RuntimeError
        with pytest.raises(error):c.update(f,now)
        r=c.missing(now)
    else:
        if fault=='replay':f['frame_id']='f8'
        elif fault=='stale':f['captured_sim_s']=.5
        elif fault=='future':f['captured_sim_s']=1.
        elif fault=='order':f['captured_sim_s']=.8
        elif fault=='gap':now=1.1;f['captured_sim_s']=1.1
        elif fault=='time_regression':now=.7
        elif fault=='shape':f['rgb']=np.zeros((128,128,3),np.uint8)
        elif fault=='flat':f['rgb']=np.zeros_like(rgb)
        r=c.update(f,now)
    assert r['state']=='aborted' and r['v_m_s']==r['omega_rad_s']==0.
    assert c.result()['result_type']=='aborted_motion_uncertain'
    assert not c.result()['atomic_edge_compatible'] and c.result()['arrival_status']=='unverified'
    # Terminal is sticky; a later good frame cannot restart the controller.
    assert c.update(dict(frame_id='next',captured_sim_s=now+.1,rgb=rgb),now+.1)['v_m_s']==0.


def test_unknown_perception_immediately_zeroes_motion_and_cannot_reset_recovery_budget(monkeypatch):
    c,rgb=moving_controller(monkeypatch)
    monkeypatch.setattr(module,'probe_straight_history',lambda *a,**k:bind_evidence(a[0], dict(corridor_geometry={'status':'unknown'},front_surface={'status':'unknown'},ground_motion={'status':'unknown'})))
    for i in range(9,21):
        r=c.update(dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb=rgb),i*.1)
        assert r['v_m_s']==r['omega_rad_s']==0.
    assert c.state=='aborted'
    assert 'recovery_timeout' in c.reason


def test_external_execution_budget_is_not_disguised_as_missing_camera(monkeypatch):
    c,rgb=moving_controller(monkeypatch);r=c.cancel(.9)
    assert r['state']=='aborted' and r['reason']=='execution_cancelled'
    assert r['v_m_s']==r['omega_rad_s']==0.
    assert c.result()['result_type']=='aborted_motion_uncertain'


@pytest.mark.parametrize('dt',[.05,.1,.15])
def test_new_ids_and_times_cannot_disguise_frozen_pixels_while_moving(monkeypatch,dt):
    c,rgb=moving_controller(monkeypatch)
    start=c.last_capture
    for i in range(1,20):
        now=start+i*dt
        r=c.update(dict(frame_id=f'fresh-freeze-{i}',captured_sim_s=now,rgb=rgb.copy()),now)
        if r['state']=='aborted':break
    assert r['reason']=='commanded_motion_without_rgb_response'
    assert .3-1e-8<=now-start<=.3+dt+1e-8
    assert r['v_m_s']==r['omega_rad_s']==0.
    assert c.result()['result_type']=='aborted_motion_uncertain'
    terminal=c.result()
    rgb[255,255,0]=200
    assert c.update(dict(frame_id='restored',captured_sim_s=now+dt,rgb=rgb),now+dt)['v_m_s']==0.
    assert c.result()==terminal


def test_unchanged_startup_rgb_is_allowed_before_motion_not_a_texture_guard(monkeypatch):
    monkeypatch.setattr(module,'_ground_evidence',lambda *_:({'status':'conditional_ground_motion','translation_m':[0.,0.],'yaw_delta_rad':0.},None,None))
    c=module.StraightController(initial_clearance=module.InitialClearancePrior(confirmed=True),motion_permission='bounded_probe')
    rgb=np.random.default_rng(200).integers(0,255,(256,256,3),dtype=np.uint8)
    for i in range(3):d=c.update(dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb=rgb),i*.1)
    assert d['state']=='bootstrapping' and d['v_m_s']>0
    assert c.frozen_rgb_motion_s==0.


def test_observed_image_response_resets_only_the_frozen_motion_timer(monkeypatch):
    c,rgb=moving_controller(monkeypatch)
    for i in range(9,18):
        if i%3==2:rgb[255,255,0]=i
        d=c.update(dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb=rgb),i*.1)
        assert d['state']=='following' and d['v_m_s']>0


@pytest.mark.parametrize('state',['visual_stop','aborted'])
@pytest.mark.parametrize('bad',[None,{}, {'unexpected':'field'}])
def test_terminal_result_is_immutable_even_for_malformed_later_packets(state,bad):
    c=module.StraightController();c.state=state;c.reason='latched_test_terminal'
    before=c.result();d=c.update(bad,float('nan'))
    assert d['v_m_s']==d['omega_rad_s']==0. and c.result()==before


@pytest.mark.parametrize('kwargs',[{'frozen_rgb_motion_timeout_s':.31}, {'front_approach_minimum_fraction':1.1},
                                  {'frozen_rgb_motion_timeout_s':True},{'front_slowdown_band_m':float('nan')}])
def test_response_and_approach_policy_limits_are_strict(kwargs):
    with pytest.raises(ValueError):module.StraightPolicy(**kwargs)


@pytest.mark.parametrize('mode',['l','t'])
@pytest.mark.parametrize('fault',['missing','replay','stale','future','order','gap','clock','shape','flat','truth'])
def test_s6_corner_modes_zero_latch_after_actual_motion_commands(monkeypatch,mode,fault):
    # Real state machine, synthetic geometry only. Physical acceptance is separate.
    from test_continuous_straight import corner_perception,frame
    monkeypatch.setattr(module,'analyze_corner_rgb',lambda *a,**kw:corner_perception(distance=2.))
    c=module.CornerController(direction='left',junction=mode,initial_clearance=module.InitialClearancePrior(confirmed=True))
    for i in range(3):r=c.update(frame(i),i*.1)
    assert r['v_m_s']>0
    f=frame(3);now=.3
    if fault=='missing':r=c.missing(now)
    elif fault=='truth':
        with pytest.raises(ValueError):c.update(dict(f,true_node='A'),now)
        r=c.missing(now)
    else:
        if fault=='replay':f['frame_id']='f2'
        elif fault=='stale':now=.7
        elif fault=='future':f['captured_sim_s']=.4
        elif fault=='order':f['captured_sim_s']=.19
        elif fault=='gap':f['captured_sim_s']=now=.6
        elif fault=='clock':now=.1
        elif fault=='shape':f['rgb']=np.zeros((10,10,3),np.uint8)
        elif fault=='flat':
            monkeypatch.undo();f['rgb'][:]=128
        if fault=='shape':
            # Analyzer has its own envelope validator; don't stub that guard.
            monkeypatch.undo()
        r=c.update(f,now)
    assert r['state']=='aborted' and r['v_m_s']==r['omega_rad_s']==0.
    r=c.update(frame(4),.4)
    assert r['state']=='aborted' and r['v_m_s']==r['omega_rad_s']==0.
    assert c.result()['result_type']=='aborted_motion_uncertain'


@pytest.mark.parametrize('mode',['straight','l','t','route'])
def test_s6_real_clock_deadline_never_turns_timeout_into_node_arrival(monkeypatch,mode):
    from test_continuous_straight import route_unit,route_hold,frame,corner_perception
    if mode=='route':
        c,_,_=route_unit(monkeypatch);route_hold(c);timeout=240.;expected='route_timeout'
    elif mode=='straight':c=module.StraightController();timeout=60.;expected='motion_timeout'
    else:
        monkeypatch.setattr(module,'analyze_corner_rgb',lambda *a,**kw:corner_perception())
        c=module.CornerController(direction='left',junction=mode,initial_clearance=module.InitialClearancePrior(confirmed=True));timeout=75.;expected='motion_timeout'
    if mode=='straight':c.started=0.;c.last_now=0.
    else:c.update(frame(),0.)
    r=c.update(dict(frame(1),captured_sim_s=timeout),timeout)
    assert r['state']=='aborted' and r['reason']==expected and r['arrival_status']=='unverified'
    assert r['v_m_s']==r['omega_rad_s']==0.


def bind_evidence(frames,packet):
    return {**packet,'geometry_reference_frame_id':frames[-1]['frame_id'],'geometry_time_s':frames[-1]['captured_sim_s']}


@pytest.fixture(autouse=True)
def synthetic_preflight(monkeypatch):
    # These state-machine tests stub geometry; actual RGB veto is tested separately.
    monkeypatch.setattr(module,'assess_startup_texture',lambda *_:dict(accepted=True,reason='unit_stub'))
