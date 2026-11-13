"""Control integration, not geometric/physical validation (perception is stubbed)."""
import numpy as np
import pytest
from roadvision import continuous_straight as module


def test_one_fixed_nine_capture_history_without_same_frame_retry(monkeypatch):
    calls=[]
    monkeypatch.setattr(module,'_ground_evidence',lambda *_:({'status':'conditional_ground_motion','translation_m':[0.,0.],'yaw_delta_rad':0.},None,None))
    def unknown(frames,**kwargs):
        calls.append([f['frame_id'] for f in frames])
        return bind_evidence(frames, dict(corridor_geometry={'status':'unknown'},front_surface={'status':'unknown'},ground_motion={'status':'unknown'}))
    monkeypatch.setattr(module,'probe_straight_history',unknown)
    monkeypatch.setattr(module,'assess_free_ground',lambda *a,**k:bind_evidence(a[0], {'status':'unknown'}))
    c=module.StraightController(initial_clearance=module.InitialClearancePrior(confirmed=True),motion_permission='bounded_probe')
    rgb=np.random.default_rng(18).integers(0,255,(256,256,3),dtype=np.uint8)
    for i in range(22):
        # Geometry is stubbed, but delivery must not imitate a frozen camera.
        rgb[255,255,0]=i
        c.update(dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb=rgb),i*.1)
    # Fixed window policy is chosen before any perception result.
    assert [len(x) for x in calls]==[9]*10  # first full rejection now starts bounded stop
    for ids in calls:
        assert len(set(ids))==len(ids)
    assert len({ids[-1] for ids in calls})==len(calls)
    assert calls[-1][0]=='f9'
    assert c.state=='aborted'
    assert c.result()['arrival_status']=='unverified'


def test_feedback_is_gentle_and_slew_limited_but_missing_brakes_immediately(monkeypatch):
    monkeypatch.setattr(module,'_ground_evidence',lambda *_:({'status':'conditional_ground_motion','translation_m':[0.,0.],'yaw_delta_rad':0.},None,None))
    def good(frames,**kwargs):
        return bind_evidence(frames, dict(corridor_geometry={'status':'conditional_corridor_geometry','heading_error_rad':0.,'offset_m':.16},front_surface={'status':'unknown'},ground_motion={'status':'unknown'}))
    monkeypatch.setattr(module,'probe_straight_history',good)
    monkeypatch.setattr(module,'assess_free_ground',lambda *a,**k:bind_evidence(a[0], {'status':'conditional_free_ground_path','conditional_stopping_clearance_m':.6}))
    c=module.StraightController(initial_clearance=module.InitialClearancePrior(confirmed=True),motion_permission='bounded_probe')
    rgb=np.random.default_rng(18).integers(0,255,(256,256,3),dtype=np.uint8)
    outputs=[]
    for i in range(10):
        rgb[255,255,0]=i
        outputs.append(c.update(dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb=rgb),i*.1))
    assert outputs[8]['v_m_s']>0 and outputs[8]['omega_rad_s']<0
    assert abs(outputs[8]['omega_rad_s'])<=.030001
    assert abs(outputs[9]['omega_rad_s']-outputs[8]['omega_rad_s'])<=.030001
    assert abs(outputs[9]['omega_rad_s'])<=.064001
    assert c.missing(1.)['omega_rad_s']==0.


def test_fresh_front_surface_gradually_limits_speed_before_unchanged_stop_gate(monkeypatch):
    monkeypatch.setattr(module,'_ground_evidence',lambda *_:({'status':'conditional_ground_motion','translation_m':[0.,0.],'yaw_delta_rad':0.},None,None))
    front={'status':'conditional_front_surface','range_interval_m':[2.3,2.4]}
    monkeypatch.setattr(module,'probe_straight_history',lambda *a,**k:bind_evidence(a[0], dict(
        corridor_geometry={'status':'conditional_corridor_geometry','heading_error_rad':0.,'offset_m':0.},
        front_surface=dict(front),ground_motion={'status':'unknown'})))
    monkeypatch.setattr(module,'assess_free_ground',lambda *a,**k:bind_evidence(a[0], {'status':'conditional_free_ground_path','conditional_stopping_clearance_m':.6}))
    c=module.StraightController(initial_clearance=module.InitialClearancePrior(confirmed=True),motion_permission='bounded_probe')
    rgb=np.random.default_rng(19).integers(0,255,(256,256,3),dtype=np.uint8)
    def step(i):
        rgb[255,255,0]=i
        return c.update(dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb=rgb),i*.1)
    for i in range(9):d=step(i)
    assert d['v_m_s']==.15
    speeds=[]
    for i,lower in enumerate((1.9,1.7,1.5,1.31),9):
        front['range_interval_m']=[lower,lower+.1];d=step(i);speeds.append(d['v_m_s'])
        assert d['state']=='following'
    assert all(a>b for a,b in zip(speeds,speeds[1:]))
    assert .075<speeds[-1]<.08
    front['range_interval_m']=[1.30,1.4];d=step(13)
    assert d['state']=='braking' and d['v_m_s']==d['omega_rad_s']==0.


def test_unknown_front_never_supplies_a_stale_speed_or_distance(monkeypatch):
    # With no metric front evidence, existing max speed and zero-on-unknown
    # geometry semantics stay intact; an old front must not be propagated.
    monkeypatch.setattr(module,'_ground_evidence',lambda *_:({'status':'conditional_ground_motion','translation_m':[0.,0.],'yaw_delta_rad':0.},None,None))
    packet=dict(corridor_geometry={'status':'conditional_corridor_geometry','heading_error_rad':0.,'offset_m':0.},
                front_surface={'status':'conditional_front_surface','range_interval_m':[1.4,1.5]},ground_motion={'status':'unknown'})
    monkeypatch.setattr(module,'probe_straight_history',lambda *a,**k:bind_evidence(a[0], packet))
    monkeypatch.setattr(module,'assess_free_ground',lambda *a,**k:bind_evidence(a[0], {'status':'conditional_free_ground_path','conditional_stopping_clearance_m':.6}))
    c=module.StraightController(initial_clearance=module.InitialClearancePrior(confirmed=True),motion_permission='bounded_probe')
    rgb=np.random.default_rng(41).integers(0,255,(256,256,3),dtype=np.uint8)
    for i in range(9):
        rgb[255,255,0]=i;d=c.update(dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb=rgb),i*.1)
    assert d['v_m_s']<.1
    packet['front_surface']={'status':'unknown'}
    rgb[255,255,0]=9;d=c.update(dict(frame_id='f9',captured_sim_s=.9,rgb=rgb),.9)
    assert d['v_m_s']==.15
    packet['corridor_geometry']={'status':'unknown'}
    rgb[255,255,0]=10;d=c.update(dict(frame_id='f10',captured_sim_s=1.,rgb=rgb),1.)
    assert d['v_m_s']==d['omega_rad_s']==0.


@pytest.mark.parametrize('offset,heading',[(.051,0.),(-.16,0.),(0.,.051),(0.,-.08)])
def test_front_slowdown_does_not_consume_alignment_recovery_parallax(monkeypatch,offset,heading):
    monkeypatch.setattr(module,'_ground_evidence',lambda *_:({'status':'conditional_ground_motion','translation_m':[0.,0.],'yaw_delta_rad':0.},None,None))
    monkeypatch.setattr(module,'probe_straight_history',lambda *a,**k:bind_evidence(a[0], dict(
        corridor_geometry={'status':'conditional_corridor_geometry','heading_error_rad':heading,'offset_m':offset},
        front_surface={'status':'conditional_front_surface','range_interval_m':[1.4,1.5]},ground_motion={'status':'unknown'})))
    monkeypatch.setattr(module,'assess_free_ground',lambda *a,**k:bind_evidence(a[0], {'status':'conditional_free_ground_path','conditional_stopping_clearance_m':.6}))
    c=module.StraightController(initial_clearance=module.InitialClearancePrior(confirmed=True),motion_permission='bounded_probe')
    rgb=np.random.default_rng(62).integers(0,255,(256,256,3),dtype=np.uint8)
    for i in range(9):
        rgb[255,255,0]=i;d=c.update(dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb=rgb),i*.1)
    assert d['v_m_s']==.15*max(.5,1-abs(heading)/.4)
    assert d['reason']=='fresh_two_side_geometry_and_ground_transfer'


def bind_evidence(frames,packet):
    return {**packet,'geometry_reference_frame_id':frames[-1]['frame_id'],'geometry_time_s':frames[-1]['captured_sim_s']}


@pytest.fixture(autouse=True)
def synthetic_preflight(monkeypatch):
    # These state-machine tests stub geometry; actual RGB veto is tested separately.
    monkeypatch.setattr(module,'assess_startup_texture',lambda *_:dict(accepted=True,reason='unit_stub'))
