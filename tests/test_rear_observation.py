import copy
from dataclasses import replace
import cv2
import numpy as np
import pytest
from roadvision import rear_observation as rear
from roadvision.calibrated_ground_motion import GroundCalibration,_camera_motion,ground_pixels,ground_points
from roadvision.corridor_rgb_probe import CALIBRATION_ID
from roadvision.rgb_ground_rig import rig_basis
from roadvision.swept_ground_path import ground_patch_support,predict_sweep,required_cells,coverage_audit
from roadvision.continuous_straight import StraightPolicy


def rear_window(translation=.06,yaw=0.,reflect=True):
    rng=np.random.default_rng(9137)
    image=cv2.GaussianBlur(rng.integers(20,240,(128,128),dtype=np.uint8),(3,3),.6)
    k=np.array([[64.,0,64.],[0,64.,64.],[0,0,1.]])
    frames=[]
    for i,fraction in enumerate([0.,.5,1.]):
        m=dict(translation_m=[translation*fraction,0.],yaw_delta_rad=yaw*fraction)
        r,t=_camera_motion('back',m)
        h=k@(r+np.outer(t,rig_basis('back').T@np.array([0.,0.,-1.])/.45))@np.linalg.inv(k)
        panel=cv2.warpPerspective(image,h,(128,128),borderMode=cv2.BORDER_REFLECT101 if reflect else cv2.BORDER_CONSTANT)
        rgb=np.repeat(np.tile(panel,(2,2))[:,:,None],3,axis=2)
        frames.append(dict(frame_id=f'f{i}',captured_sim_s=.4*i,rgb=rgb))
    motion=dict(status='conditional_ground_motion',translation_m=[translation,0.],yaw_delta_rad=yaw,
        translation_reference_frame_id='f0',from_time_s=0.,to_time_s=.8,
        conditional_uncertainty=dict(covariance_se2=(np.eye(3)*1e-6).tolist()))
    packet=dict(ground_motion=motion,geometry_reference_frame_id='f2',geometry_time_s=.8)
    return frames,packet


def assess(frames,packet):
    return rear.assess_rear_consistency(frames,packet,now_s=.8,calibration_id=CALIBRATION_ID)


def test_back_basis_and_ground_inverse():
    b=rig_basis('back');assert np.allclose(b.T@b,np.eye(3)) and np.linalg.det(b)==pytest.approx(1.)
    p=np.array([[-.85,0.],[-1.,.15],[-1.2,-.2]])
    assert np.allclose(ground_points(ground_pixels(p,'back',GroundCalibration()),'back',GroundCalibration()),p)
    assert np.isnan(ground_pixels([[.8,0.]],'back',GroundCalibration())).all()


def test_real_synthetic_back_patch_has_native_footprint_but_does_not_fill_near_blind_zone():
    f,p=rear_window(.12,reflect=False)
    r=ground_patch_support(f,p['ground_motion'],[[-.85,0.]],calibration=GroundCalibration(),include_back=True)
    assert r['patches'][0]['view']=='back' and r['patches'][0]['accepted']
    assert len(r['patches'][0]['polygon'])==4
    s=predict_sweep(.08,0.,age_s=0.,policy=StraightPolicy())
    cells=required_cells(s['polygon'],.02)
    r=ground_patch_support(f,p['ground_motion'],[c.mean(axis=0) for c in cells],calibration=GroundCalibration(),include_back=True)
    c=coverage_audit(s['polygon'],r['patches'],cell_m=.02)
    assert c['unknown_cells']==c['required_cells'] and r['outside_all_supported_views']==len(cells)


def test_synthetic_rear_motion_passes_without_refit_or_authorization():
    f,p=rear_window();saved=copy.deepcopy(p);r=assess(f,p)
    assert r['status']=='conditional_rear_consistent',r
    assert p==saved and not r['motion_reestimated'] and not r['motion_averaged']
    assert not r['motion_authorized'] and not r['certified_clearance']
    assert set(r['pixel_checks']['training']['indices']).isdisjoint(r['pixel_checks']['heldout']['indices'])


@pytest.mark.parametrize('mode',['flat','repeat','frozen','wrong_motion','wrong_yaw','foreign','stale','pure_rotation'])
def test_unsupported_or_conflicting_back_never_passes(mode):
    f,p=rear_window()
    if mode=='flat':
        for x in f:x['rgb'][128:,128:]=128
    if mode=='repeat':
        yy,xx=np.indices((128,128));im=((xx//4+yy//4)%2*160+40).astype(np.uint8)
        for x in f:x['rgb'][128:,128:]=im[:,:,None]
    if mode=='frozen':
        for x in f:x['rgb'][128:,128:]=f[0]['rgb'][128:,128:]
    if mode=='wrong_motion':p['ground_motion']['translation_m']=[-.12,0.]
    if mode=='wrong_yaw':p['ground_motion']['yaw_delta_rad']=.2
    if mode=='foreign':p['ground_motion']['translation_reference_frame_id']='foreign'
    if mode=='stale':p['geometry_time_s']=.7
    if mode=='pure_rotation':p['ground_motion']['translation_m']=[0.,0.]
    assert assess(f,p)['status']!='conditional_rear_consistent'


@pytest.mark.parametrize('mode',['truth','gap','duplicate','future'])
def test_public_envelope_validation(mode):
    f,p=rear_window()
    if mode=='truth':f[1]['pose']=[0,0]
    if mode=='gap':f[1]['captured_sim_s']=.99
    if mode=='duplicate':f[1]['frame_id']='f0'
    if mode=='future':f[-1]['captured_sim_s']=.9
    with pytest.raises(ValueError):assess(f,p)


def proposal(**changes):
    binding=dict(geometry_reference_frame_id='now',geometry_time_s=.8)
    args=dict(now_s=.8,frame_id='now',captured_s=.8,state='waiting_evidence',permission='bounded_probe',
        stationary_frames=3,recovery_count=0,attempts=0,wait_started=.8,probe_distance_bound_m=0.,policy=StraightPolicy(),
        sweep=dict(status='conditional_patch_supported_sweep',requested_v_m_s=.08,requested_omega_rad_s=0.,**binding),
        rear=dict(status='conditional_rear_consistent',**binding))
    args.update(changes);return rear.review_active_proposal(**args)


def test_original_one_second_deadline_cannot_fit_new_action_plus_conservative_braking():
    r=proposal()
    assert r['status']=='ineligible' and r['failed_checks']==['original_time_budget']
    assert r['stop_reserve_time_s']==pytest.approx(.97)
    assert not r['motion_authorized'] and r['actuator_command']==dict(v_m_s=0.,omega_rad_s=0.)


def test_counterfactual_larger_wait_is_unit_branch_only_never_actual_configuration():
    # NOT the running controller policy; no extension of the production 1s.
    r=proposal(policy=replace(StraightPolicy(),unverified_timeout_s=2.))
    assert r['status']=='eligible_proposal_only' and r['evaluated_not_executed']
    assert not r['motion_authorized'] and r['actuator_command']['v_m_s']==0


@pytest.mark.parametrize('change,check',[
    ({'state':'following'},'current_wait_and_fresh_capture'),
    ({'state':'aborted'},'current_wait_and_fresh_capture'),
    ({'permission':'no_motion'},'original_motion_permission'),
    ({'stationary_frames':2},'confirmed_stationary'),
    ({'recovery_count':3},'original_recovery_limit'),
    ({'attempts':2},'proposal_attempt_limit'),
    ({'wait_started':None},'original_time_budget'),
    ({'wait_started':0.},'original_time_budget'),
    ({'now_s':1.2},'current_wait_and_fresh_capture'),
    ({'probe_distance_bound_m':.135},'original_distance_budget'),
    ({'sweep':None},'current_requested_swept_coverage'),
    ({'rear':None},'current_rear_consistent'),
    ({'captured_s':float('nan')},'current_wait_and_fresh_capture'),
])
def test_proposal_preconditions_cannot_be_bypassed(change,check):
    r=proposal(**change);assert r['status']=='ineligible' and check in r['failed_checks']
    assert not r['motion_authorized']


@pytest.mark.parametrize('which,field,value',[
    ('sweep','geometry_reference_frame_id','old'),('sweep','geometry_time_s',.7),
    ('sweep','requested_v_m_s',.15),('sweep','requested_omega_rad_s',.1),
    ('rear','geometry_reference_frame_id','old'),('rear','status','conflict')])
def test_exact_action_and_current_binding_required(which,field,value):
    binding=dict(geometry_reference_frame_id='now',geometry_time_s=.8)
    item=dict(status='conditional_patch_supported_sweep',requested_v_m_s=.08,requested_omega_rad_s=0.,**binding) if which=='sweep' else dict(status='conditional_rear_consistent',**binding)
    item[field]=value
    assert proposal(**{which:item})['status']=='ineligible'


@pytest.mark.parametrize('value',['yes',1,None,{}])
def test_extra_view_must_be_explicit_bool(value):
    f,p=rear_window()
    with pytest.raises(ValueError):ground_patch_support(f,p['ground_motion'],[[-.85,0.]],calibration=GroundCalibration(),include_back=value)
