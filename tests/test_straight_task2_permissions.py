"""Task-2 permission contracts; unit fits are synthetic, not RGB validation."""
import copy
import numpy as np
import pytest
from roadvision import continuous_straight as m
from ros2_ws.src.toponav_bridge.toponav_bridge.protocol import MotorGate, command_from_decision


def frame(i):
    return dict(frame_id=f'f{i}',captured_sim_s=i*.1,
                rgb=np.random.default_rng(i).integers(0,255,(256,256,3),dtype=np.uint8))


def setup(monkeypatch,permission='bounded_probe'):
    control=m.StraightController(initial_clearance=m.InitialClearancePrior(confirmed=True),motion_permission=permission)
    fits=dict(good=True,stable=True,preflight=True)
    monkeypatch.setattr(m,'assess_startup_texture',lambda *_:dict(accepted=fits['preflight'],reason='synthetic'))
    monkeypatch.setattr(control,'_local_stationary_motion',lambda *_:dict(status='conditional_ground_motion',translation_m=[0. if fits['stable'] else .02,0.],yaw_delta_rad=0.))
    def probe(frames,**kw):
        return dict(geometry_time_s=kw['now_s'],geometry_reference_frame_id=frames[-1]['frame_id'],
                    corridor_geometry=dict(status='conditional_corridor_geometry' if fits['good'] else 'unknown',heading_error_rad=0.,offset_m=0.),
                    front_surface=dict(status='unknown'),ground_motion=dict(status='unknown'))
    monkeypatch.setattr(m,'probe_straight_history',probe)
    monkeypatch.setattr(m,'assess_free_ground',lambda frames,*a,**kw:dict(status='conditional_free_ground_path',conditional_stopping_clearance_m=.6,geometry_time_s=kw['now_s'],geometry_reference_frame_id=frames[-1]['frame_id']))
    return control,fits


def test_confirmed_clearance_alone_no_longer_authorizes_motion():
    c=m.StraightController(initial_clearance=m.InitialClearancePrior(confirmed=True))
    out=c.update(frame(0),0.)
    assert out['reason']=='motion_not_authorized' and c.motion_commands==0


def test_explicit_no_motion_overrides_good_fits(monkeypatch):
    c,_=setup(monkeypatch,'no_motion')
    for i in range(12):out=c.update(frame(i),i*.1)
    assert out['state']=='aborted' and out['v_m_s']==out['omega_rad_s']==0 and c.motion_commands==0


@pytest.mark.parametrize('mode',[None,True,'follow','probe',''])
def test_unknown_permission_fails_closed(mode):
    with pytest.raises(ValueError):m.StraightController(motion_permission=mode)


def test_ambiguous_preflight_cannot_use_operator_clearance_to_probe(monkeypatch):
    c,fits=setup(monkeypatch);fits['preflight']=False
    for i in range(3):out=c.update(frame(i),i*.1)
    assert out['v_m_s']==0 and c.motion_commands==0 and out['reason']=='startup_texture_unverified'


def test_valid_bounded_probe_is_not_following_before_complete_fits(monkeypatch):
    c,_=setup(monkeypatch)
    for i in range(3):out=c.update(frame(i),i*.1)
    assert out['state']=='bootstrapping' and out['v_m_s']>0
    assert out['permission']['authorization']=='bounded_probe'
    assert out['permission']['probe_distance_bound_m']+out['permission']['stopping_reserve_m']<=.30


def test_complete_geometry_rejection_ends_probe_immediately(monkeypatch):
    c,fits=setup(monkeypatch);fits['good']=False
    for i in range(9):out=c.update(frame(i),i*.1)
    assert out['state']=='waiting_evidence' and out['v_m_s']==0 and c.motion_commands==6


def test_good_following_is_not_disabled_for_every_case(monkeypatch):
    c,_=setup(monkeypatch)
    for i in range(9):out=c.update(frame(i),i*.1)
    assert out['state']=='following' and out['v_m_s']>0


def test_return_of_geometry_does_not_interrupt_braking(monkeypatch):
    c,fits=setup(monkeypatch)
    for i in range(9):c.update(frame(i),i*.1)
    fits['good']=False;c.update(frame(9),.9)
    fits.update(good=True,stable=False)
    for i in range(10,17):
        out=c.update(frame(i),i*.1)
        assert out['v_m_s']==out['omega_rad_s']==0
    assert out['state']=='waiting_evidence' and out['stationary_frames']==0


def test_rearm_requires_post_zero_stationarity_and_consecutive_good_evidence(monkeypatch):
    c,fits=setup(monkeypatch)
    # RGB change is separately part of the visual stop test.
    pixels=frame(0)['rgb']
    def step(i):
        pixels[255,255,0]=i
        return c.update(dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb=pixels),i*.1)
    for i in range(9):step(i)
    fits['good']=False;step(9)
    fits['good']=True
    for i in range(10,14):assert step(i)['v_m_s']==0
    out=step(14)
    assert out['state']=='following' and out['v_m_s']>0 and out['stationary_frames']>=3
    assert out['permission']['rearm_confirmations']>=3


def test_intermittent_geometry_cannot_extend_wait_budget(monkeypatch):
    c,fits=setup(monkeypatch)
    for i in range(9):c.update(frame(i),i*.1)
    fits['good']=False;c.update(frame(9),.9)
    for i in range(10,21):
        fits['good']=i%2==0
        out=c.update(frame(i),i*.1)
    assert out['state']=='aborted' and 'recovery_timeout' in out['reason']


def test_budget_reserves_next_command_interval_and_braking(monkeypatch):
    c,_=setup(monkeypatch)
    # Sparse delivery reaches command-distance cap before nine frames exist.
    for i in range(10):
        f=frame(i);f['captured_sim_s']=i*.25;out=c.update(f,i*.25)
        assert out['v_m_s']==0 or out['permission']['probe_distance_bound_m']<=c.policy.probe_distance_limit_m
    assert c.motion_commands>0 and out['v_m_s']==0


def test_held_motor_command_expires_even_without_next_frame_arm():
    gate=MotorGate('t2');gate.arm('f1',1.,10.)
    d=dict(control_frame_id='f1',captured_sim_s=1.,decision_sim_s=1.,v_m_s=.15,omega_rad_s=0.,state='moving')
    assert gate.accept(command_from_decision('t2',1,d),1.,10.1)==(.15,0.)
    assert gate.poll(10.91)==(0.,0.) and gate.latched


def test_arming_a_new_frame_does_not_extend_old_command_lifetime():
    gate=MotorGate('t2');gate.arm('f1',1.,10.)
    d=dict(control_frame_id='f1',captured_sim_s=1.,decision_sim_s=1.,v_m_s=.15,omega_rad_s=0.,state='moving')
    gate.accept(command_from_decision('t2',1,d),1.,10.1)
    gate.arm('f2',1.1,10.89)
    assert gate.poll(10.91)==(0.,0.) and gate.latched


@pytest.mark.parametrize('target',['probe_straight_history','assess_free_ground'])
@pytest.mark.parametrize('field,value',[('geometry_time_s',0.),('geometry_reference_frame_id','old')])
def test_stale_or_wrong_frame_evidence_cannot_authorize_following(monkeypatch,target,field,value):
    c,_=setup(monkeypatch)
    original=getattr(m,target)
    def stale(*a,**kw):
        result=original(*a,**kw);result[field]=value;return result
    monkeypatch.setattr(m,target,stale)
    for i in range(9):out=c.update(frame(i),i*.1)
    assert out['state']=='waiting_evidence' and out['v_m_s']==0


@pytest.mark.parametrize('clearance',[None,True,.01,float('nan'),float('inf')])
def test_insufficient_or_invalid_stopping_support_forbids_following(monkeypatch,clearance):
    c,_=setup(monkeypatch);original=m.assess_free_ground
    def short(*a,**kw):
        result=original(*a,**kw);result['conditional_stopping_clearance_m']=clearance;return result
    monkeypatch.setattr(m,'assess_free_ground',short)
    for i in range(9):out=c.update(frame(i),i*.1)
    assert out['v_m_s']==0 and out['state']=='waiting_evidence'


@pytest.mark.parametrize('kwargs',[{'probe_distance_limit_m':.141},{'actuator_speed_bound_m_s':.179},{'command_horizon_s':.24},
    {'minimum_brake_deceleration_m_s2':.26},{'stopping_margin_m':.049},
    {'rearm_confirmations':2},{'max_recoveries':4},{'max_recoveries':True}])
def test_conservative_bound_cannot_be_weakened(kwargs):
    with pytest.raises(ValueError):m.StraightPolicy(**kwargs)


def test_recovery_count_is_not_reset_by_transient_success(monkeypatch):
    c,fits=setup(monkeypatch);c.policy=m.StraightPolicy(max_recoveries=0)
    for i in range(9):c.update(frame(i),i*.1)
    fits['good']=False;c.update(frame(9),.9);fits['good']=True
    out=c.update(frame(10),1.)
    assert out['state']=='aborted' and out['reason']=='recovery_count_exhausted'


def test_a_larger_latency_assumption_requires_more_prior_clearance(monkeypatch):
    c,_=setup(monkeypatch);c.policy=m.StraightPolicy(command_horizon_s=1.)
    for i in range(3):out=c.update(frame(i),i*.1)
    assert out['reason']=='initial_clearance_insufficient_for_probe_and_braking' and c.motion_commands==0


def test_stale_fit_breaks_the_consecutive_rearm_sequence(monkeypatch):
    c,fits=setup(monkeypatch)
    for i in range(9):c.update(frame(i),i*.1)
    fits['good']=False;c.update(frame(9),.9);fits['good']=True
    c.update(frame(10),1.);assert c.rearm_frames==1
    original=m.probe_straight_history
    def stale(*a,**kw):
        packet=original(*a,**kw);packet['geometry_time_s']=0.;return packet
    monkeypatch.setattr(m,'probe_straight_history',stale)
    c.update(frame(11),1.1);assert c.rearm_frames==0


@pytest.mark.parametrize('value',[float('nan'),float('inf'),None,True])
def test_invalid_supported_geometry_never_becomes_nonfinite_motion(monkeypatch,value):
    c,_=setup(monkeypatch);original=m.probe_straight_history
    def invalid(*a,**kw):
        packet=original(*a,**kw);packet['corridor_geometry']['heading_error_rad']=value;return packet
    monkeypatch.setattr(m,'probe_straight_history',invalid)
    for i in range(9):out=c.update(frame(i),i*.1)
    assert out['v_m_s']==out['omega_rad_s']==0 and out['reason']=='invalid_geometry_values'


def test_fast_packets_cannot_shorten_stationary_confirmation_time(monkeypatch):
    c,fits=setup(monkeypatch)
    pixels=frame(0)['rgb']
    def step(t):
        pixels[255,255,0]=(int(t*1000)%255)
        return c.update(dict(frame_id=f'fast-{t}',captured_sim_s=t,rgb=pixels),t)
    for i in range(9):step(i*.1)
    fits['good']=False;step(.9);fits['good']=True
    for i in range(1,31):
        out=step(round(.9+i*.01,6))
        assert out['v_m_s']==0, 'fresh IDs must not turn 3 fast packets into a stop proof'
    assert c.stationary_frames<3
