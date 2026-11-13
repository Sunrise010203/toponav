"""Task4 integration; mock geometry is safety-state testing, not RGB proof."""
import json
import numpy as np
import pytest
from roadvision import continuous_straight as base
from roadvision import swept_straight as guarded
from roadvision import swept_ground_path as swept


def controller(monkeypatch,supported=False):
    from test_straight_task2_permissions import setup
    b,fits=setup(monkeypatch)
    c=guarded.SweptStraightController(initial_clearance=base.InitialClearancePrior(confirmed=True),motion_permission='bounded_probe')
    c._local_stationary_motion=b._local_stationary_motion
    calls=[]
    def check(frames,packet,v,w,**kw):
        calls.append((frames,packet,v,w,kw))
        return dict(status='conditional_patch_supported_sweep' if supported else 'unknown',requested_v_m_s=v,requested_omega_rad_s=w,
            geometry_reference_frame_id=frames[-1]['frame_id'],geometry_time_s=frames[-1]['captured_sim_s'])
    monkeypatch.setattr(guarded,'assess_swept_ground',check)
    return c,fits,calls


def test_extra_rejection_stops_immediately_and_cannot_reset_wait_budget(monkeypatch):
    from test_straight_task2_permissions import frame
    c,_,calls=controller(monkeypatch)
    for i in range(9):out=c.update(frame(i),i*.1)
    assert out['state']=='waiting_evidence' and out['v_m_s']==0 and len(calls)==1
    started=c.unverified_started
    for i in range(9,20):
        out=c.update(frame(i),i*.1)
        assert out['v_m_s']==0 and c.unverified_started==started
    assert out['state']=='aborted' and out['reason']=='evidence_recovery_timeout_swept_path_unverified'


def test_guard_pass_does_not_change_baseline_candidate_or_allow_topological_arrival(monkeypatch):
    from test_straight_task2_permissions import frame
    c,_,calls=controller(monkeypatch,True)
    for i in range(9):out=c.update(frame(i),i*.1)
    assert out['state']=='following' and out['v_m_s']==calls[-1][2] and out['omega_rad_s']==calls[-1][3]
    assert out['arrival_status']=='unverified' and not out['atomic_edge_compatible']
    assert len(calls[-1][0])==3 and calls[-1][0][-1]['frame_id']==out['control_frame_id']


@pytest.mark.parametrize('field,value',[('geometry_reference_frame_id','old'),('geometry_time_s',0.),('requested_v_m_s',.14)])
def test_stale_or_different_command_proof_never_passes(monkeypatch,field,value):
    from test_straight_task2_permissions import frame
    c,_,_=controller(monkeypatch,True);original=guarded.assess_swept_ground
    def wrong(*args,**kw):
        result=original(*args,**kw);result[field]=value;return result
    monkeypatch.setattr(guarded,'assess_swept_ground',wrong)
    for i in range(9):out=c.update(frame(i),i*.1)
    assert out['state']=='waiting_evidence' and out['v_m_s']==0


@pytest.mark.parametrize('confirmed,permission',[(True,'no_motion'),(False,'bounded_probe')])
def test_swept_pass_cannot_bypass_startup_authorization(monkeypatch,confirmed,permission):
    from test_straight_task2_permissions import frame
    c,_,calls=controller(monkeypatch,True);c.initial_clearance=base.InitialClearancePrior(confirmed=confirmed);c.motion_permission=permission
    out=c.update(frame(0),0.)
    assert out['state']=='aborted' and out['v_m_s']==0 and not calls
    assert c.update({},float('nan'))['v_m_s']==0


def test_public_contract_and_variant_are_bound_to_replay(tmp_path):
    from simulation.continuous_straight_run import run_case
    from simulation.continuous_straight_acceptance import replay_safety_public
    p=tmp_path/'run';result=run_case(p,straight_variant='swept',operator_confirmed=True,terminal_check_ticks=3,max_ticks=3)
    rows=[json.loads(l) for l in (p/'decisions.jsonl').read_text().splitlines()]
    assert replay_safety_public(p,result,rows)['terminal_frames']==3
    path=p/'public/task.json';task=json.loads(path.read_text());task['trajectory_contract']['installation']['half_width_m']=.11;path.write_text(json.dumps(task))
    with pytest.raises(ValueError,match='differs'):replay_safety_public(p,result,rows)


def test_rgb_near_field_is_unknown_even_with_accepted_far_lookahead():
    from test_free_ground_path import synthetic_window
    from roadvision.corridor_rgb_probe import CALIBRATION_ID
    frames,m=synthetic_window();m['conditional_uncertainty']=dict(covariance_se2=(np.eye(3)*1e-6).tolist())
    planes={v:dict(status='conditional_vertical_plane',normal_rig=n,distance_m=d,
        conditional_interval=dict(distance_half_width_m=.01,normal_angle_half_width_rad=.01)) for v,n,d in [('left',[0,1,0],.9),('right',[0,-1,0],.9)]}
    packet=dict(ground_motion=m,vertical_planes=planes,geometry_reference_frame_id='f2',geometry_time_s=.8)
    out=swept.assess_swept_ground(frames,packet,.15,.1,now_s=.8,calibration_id=CALIBRATION_ID)
    assert out['status']=='unknown' and out['reason']=='swept_area_contains_unobserved_or_unverified_ground'
    assert out['coverage']['unknown_cells']>0 and not out['certified_clearance']
    assert out['rgb_support']['outside_all_supported_views']>0


def test_ground_patch_footprints_are_finite_native_patches_not_one_point():
    from test_free_ground_path import synthetic_window
    from roadvision.calibrated_ground_motion import GroundCalibration
    frames,m=synthetic_window()
    out=swept.ground_patch_support(frames,m,[[.85,0.]],calibration=GroundCalibration())
    assert out['queries']==1 and len(out['patches'])==1
    assert out['patches'][0]['accepted'] and len(out['patches'][0]['polygon'])==4
    assert not swept.coverage_audit([[-.4,-.25],[.2,-.25],[.2,.25],[-.4,.25]],out['patches'],cell_m=.02)['covered']


@pytest.mark.parametrize('name',[
    'test_ambiguous_preflight_cannot_use_operator_clearance_to_probe',
    'test_complete_geometry_rejection_ends_probe_immediately',
    'test_return_of_geometry_does_not_interrupt_braking',
    'test_rearm_requires_post_zero_stationarity_and_consecutive_good_evidence',
    'test_intermittent_geometry_cannot_extend_wait_budget',
    'test_budget_reserves_next_command_interval_and_braking',
    'test_recovery_count_is_not_reset_by_transient_success',
    'test_a_larger_latency_assumption_requires_more_prior_clearance',
    'test_stale_fit_breaks_the_consecutive_rearm_sequence',
    'test_fast_packets_cannot_shorten_stationary_confirmation_time'])
def test_existing_safety_contracts_remain_when_extra_guard_passes(monkeypatch,name):
    import test_straight_task2_permissions as tests
    monkeypatch.setattr(guarded.SweptStraightController,'_following_guard',lambda *args:True)
    monkeypatch.setattr(base,'StraightController',lambda **kw:guarded.SweptStraightController(**kw))
    getattr(tests,name)(monkeypatch)


@pytest.mark.parametrize('bad',['truth','stale','reference','covariance','negative_covariance'])
def test_ground_support_binding_and_covariance_fail_closed(bad):
    from test_free_ground_path import synthetic_window
    from roadvision.corridor_rgb_probe import CALIBRATION_ID
    frames,m=synthetic_window();m['conditional_uncertainty']=dict(covariance_se2=(np.eye(3)*1e-6).tolist())
    packet=dict(ground_motion=m,geometry_reference_frame_id='f2',geometry_time_s=.8)
    if bad=='truth':
        frames[1]['pose']=[0,0]
        with pytest.raises(ValueError):swept.assess_swept_ground(frames,packet,.1,0.,now_s=.8,calibration_id=CALIBRATION_ID)
        return
    if bad=='stale':packet['geometry_time_s']=.7
    if bad=='reference':m['translation_reference_frame_id']='foreign'
    if bad=='covariance':m['conditional_uncertainty']={}
    if bad=='negative_covariance':m['conditional_uncertainty']['covariance_se2'][0][0]=-.01
    out=swept.assess_swept_ground(frames,packet,.1,0.,now_s=.8,calibration_id=CALIBRATION_ID)
    assert out['status']=='unknown' and not out['motion_authorized']


@pytest.mark.parametrize('surface',['flat','vertical'])
def test_native_patch_ambiguity_and_vertical_transfer_do_not_supply_ground(surface):
    from test_free_ground_path import synthetic_window
    from roadvision.calibrated_ground_motion import GroundCalibration
    frames,m=synthetic_window(plane_normal=(1.,0.,0.),distance=.6) if surface=='vertical' else synthetic_window()
    if surface=='flat':
        for f in frames:f['rgb'][:]=128
    r=swept.ground_patch_support(frames,m,[[.85,0.]],calibration=GroundCalibration())
    assert not any(p['accepted'] for p in r['patches'])


def test_colour_is_not_a_semantic_obstacle_label():
    from test_free_ground_path import synthetic_window
    from roadvision.calibrated_ground_motion import GroundCalibration
    frames,m=synthetic_window()
    # Identical grayscale texture in blue-channel panels: no colour veto.
    for f in frames:f['rgb'][:,:,0]=0;f['rgb'][:,:,1]=0
    r=swept.ground_patch_support(frames,m,[[.85,0.]],calibration=GroundCalibration())
    assert r['patches'][0]['accepted']


def test_complete_synthetic_patch_and_plane_coverage_can_pass_without_claiming_certification(monkeypatch):
    from test_free_ground_path import synthetic_window
    from roadvision.corridor_rgb_probe import CALIBRATION_ID
    frames,m=synthetic_window();m['conditional_uncertainty']=dict(covariance_se2=(np.eye(3)*1e-6).tolist())
    planes={v:dict(status='conditional_vertical_plane',normal_rig=n,distance_m=.9,
        conditional_interval=dict(distance_half_width_m=.01,normal_angle_half_width_rad=.01)) for v,n in [('left',[0,1,0]),('right',[0,-1,0])]}
    packet=dict(ground_motion=m,vertical_planes=planes,geometry_reference_frame_id='f2',geometry_time_s=.8)
    monkeypatch.setattr(swept,'ground_patch_support',lambda *args,**kw:dict(patches=[dict(accepted=True,polygon=[[-1,-1],[1,-1],[1,1],[-1,1]])],scope='SYNTHETIC unit proof, NOT RGB result'))
    r=swept.assess_swept_ground(frames,packet,.15,.1,now_s=.8,calibration_id=CALIBRATION_ID)
    assert r['status']=='conditional_patch_supported_sweep' and r['coverage']['covered']
    assert not r['motion_authorized'] and not r['certified_clearance'] and r['arrival_status']=='unverified'
    packet['vertical_planes']['left']['distance_m']=.2
    assert swept.assess_swept_ground(frames,packet,.15,.1,now_s=.8,calibration_id=CALIBRATION_ID)['status']=='unknown'
