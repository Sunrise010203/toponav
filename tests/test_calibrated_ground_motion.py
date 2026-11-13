"""Synthetic geometry tests, not evaluation scenes."""
import copy
import math

import numpy as np
import pytest

from roadvision.calibrated_ground_motion import (
    GroundCalibration, ground_points, ground_pixels, fit_se2, pixel_predictions,
    estimate_ground_motion, select_descriptor, probe_calibrated_ground,
    estimate_vertical_plane, corridor_from_planes,
)
from roadvision.corridor_rgb_probe import CALIBRATION_ID


def rotation(theta):
    return np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])


def observations(translation=(.12,.02), yaw=.025):
    calibration=GroundCalibration();regions={}
    for name in ('front','left','right'):
        a=np.array([[u,v] for v in (82.,90.,100.,112.) for u in (20.,40.,60.,80.,100.)])
        xy=ground_points(a,name,calibration)
        target=(xy-np.array(translation))@rotation(yaw)
        c=ground_pixels(target,name,calibration)
        good=np.isfinite(c).all(axis=1)&(c>=6).all(axis=1)&(c<122).all(axis=1)
        tracks=[dict(first=p.tolist(),last=q.tolist(),feature_split='train' if i%2==0 else 'validation') for i,(p,q) in enumerate(zip(a[good],c[good]))]
        regions[name]=dict(view=name,tracks=tracks)
    return regions


@pytest.mark.parametrize('name',['front','left','right'])
def test_ground_ray_projection_round_trip(name):
    p=np.array([[15.,82.],[64.,100.],[110.,115.]])
    c=GroundCalibration()
    np.testing.assert_allclose(ground_pixels(ground_points(p,name,c),name,c),p,atol=1e-10)


@pytest.mark.parametrize('field',['height_m','focal_px','principal_px'])
@pytest.mark.parametrize('value',[float('nan'),True,0.,-1.])
def test_invalid_calibration(field,value):
    with pytest.raises(ValueError):GroundCalibration(**{field:value})


def test_above_horizon_cannot_provide_ground_points():
    with pytest.raises(ValueError):ground_points(np.array([[50.,60.]]),'front',GroundCalibration())


def test_fit_sign_convention_and_translation_scale():
    a=np.array([[1.,0.],[2.,1.],[3.,-1.],[2.5,.5]])
    t=np.array([.1,-.03]);yaw=.04;b=(a-t)@rotation(yaw)
    estimate=fit_se2(a,b)
    np.testing.assert_allclose(estimate[:2],t,atol=1e-10)
    assert estimate[2]==pytest.approx(yaw,abs=1e-10)


@pytest.mark.parametrize('t,yaw',[((.12,.02),.025),((0.,0.),.04),((0.,0.),0.),((.035,0.),0.)])
def test_shared_se2_recovers_known_motion(t,yaw):
    r=estimate_ground_motion(observations(t,yaw),GroundCalibration())
    assert r['status']=='conditional_ground_motion'
    np.testing.assert_allclose(r['translation_m'],t,atol=.002)
    assert r['yaw_delta_rad']==pytest.approx(yaw,abs=.001)
    assert r['motion_authorized'] is False and r['arrival_status']=='unverified'
    assert not set(r['fit_track_ids'])&set(r['validation_track_ids'])


def test_validation_corruption_never_repaired_by_ransac():
    regions=observations()
    for r in regions.values():
        for t in r['tracks']:
            if t['feature_split']=='validation':t['last'][0]+=8
    result=estimate_ground_motion(regions,GroundCalibration())
    assert result['status']=='unknown' and result['reason']=='heldout_prediction_failed'


def test_one_view_is_not_cross_view_evidence():
    regions=observations();r=estimate_ground_motion({'front':regions['front']},GroundCalibration())
    assert r['status']=='unknown' and r['motion_authorized'] is False


def test_validation_descriptor_scores_cannot_select_channel():
    a=[dict(first=[20+i*10,20+i*5],last=[20+i*10,20+i*5]) for i in range(8)]
    channels={name:dict(tracks=[],descriptor_evidence=[dict(track_index=i,accepted=(i%2==0 if name=='H' else i%2!=0)) for i in range(8)]) for name in ('H','I')}
    selected=select_descriptor(a,channels,np.eye(3))
    assert selected['channel']=='H'
    modified=copy.deepcopy(channels)
    for x in modified['I']['descriptor_evidence']:
        if x['track_index']%2:x['accepted']=False
    assert select_descriptor(a,modified,np.eye(3))==selected


def test_empty_and_malformed_inputs_fail_closed():
    assert estimate_ground_motion({},GroundCalibration())['status']=='unknown'
    frames=[dict(frame_id=str(i),captured_sim_s=i*.1,rgb=np.zeros((256,256,3),np.uint8)) for i in range(3)]
    result=probe_calibrated_ground(frames,now_s=.2,calibration_id=CALIBRATION_ID)
    assert result['motion_authorized'] is False and result['arrival_status']=='unverified'
    assert result['ground_motion']['status']=='unknown'
    with pytest.raises(ValueError):probe_calibrated_ground(frames,now_s=.2,calibration_id='wrong')
    with pytest.raises(ValueError):probe_calibrated_ground([{**frames[0],'true_pose':[]}]+frames[1:],now_s=.2,calibration_id=CALIBRATION_ID)


def test_metric_motion_is_explicitly_conditional_on_calibration_and_static_ground():
    r=estimate_ground_motion(observations(),GroundCalibration())
    assert r['scale_source']=='declared_camera_height_m'
    assert r['calibration_verified'] is False and r['static_scene_verified'] is False


def plane_region(view='left',normal=(0.,1.,0.),distance=1.2):
    from roadvision.rgb_ground_rig import rig_basis
    c=GroundCalibration();k=np.array([[c.focal_px,0,c.principal_px],[0,c.focal_px,c.principal_px],[0,0,1.]])
    basis=rig_basis(view);rotation3=np.eye(3);translation=-np.array([.12,0.,0.])
    q=basis.T@np.array(normal)/distance
    h=k@(rotation3+np.outer(basis.T@translation,q))@np.linalg.inv(k)
    from roadvision.corridor_rgb_probe import _project
    rows=(82.,90.,100.,112.) if abs(normal[2])>.5 else (12.,28.,44.,60.)
    a=np.array([[u,v] for v in rows for u in (15.,35.,55.,75.,95.,110.)])
    target=_project(h,a)
    return dict(view=view,tracks=[dict(first=p.tolist(),last=t.tolist(),feature_split='train' if i%2==0 else 'validation') for i,(p,t) in enumerate(zip(a,target))])


def test_vertical_plane_metric_scale_and_ground_rejection():
    motion=dict(status='conditional_ground_motion',translation_m=[.12,0.],yaw_delta_rad=0.)
    r=estimate_vertical_plane(plane_region(),motion,GroundCalibration())
    assert r['status']=='conditional_vertical_plane'
    assert r['distance_m']==pytest.approx(1.2,abs=.001)
    np.testing.assert_allclose(r['normal_rig'],[0.,1.,0.],atol=1e-5)
    floor=estimate_vertical_plane(plane_region(normal=(0.,0.,-1.),distance=.45),motion,GroundCalibration())
    assert floor['status']=='unknown' and floor['reason']=='not_vertical_plane'


def test_plane_needs_translation_and_disjoint_checks():
    motion=dict(status='conditional_ground_motion',translation_m=[0.,0.],yaw_delta_rad=.02)
    assert estimate_vertical_plane(plane_region(),motion,GroundCalibration())['status']=='unknown'
    motion.update(translation_m=[.12,0.],yaw_delta_rad=0.)
    region=plane_region()
    for t in region['tracks']:
        if t['feature_split']=='validation':t['last'][0]+=6
    result=estimate_vertical_plane(region,motion,GroundCalibration())
    assert result['status']=='unknown' and result['reason']=='heldout_plane_failed'


def test_paired_planes_center_offset_is_not_paint_center():
    planes={v:dict(status='conditional_vertical_plane',distance_m=d,normal_rig=n)
            for v,d,n in [('left',.7,[0.,1.,0.]),('right',1.1,[0.,-1.,0.])]}
    r=corridor_from_planes(planes)
    assert r['status']=='conditional_corridor_geometry'
    assert r['offset_m']==pytest.approx(.2)
    assert r['width_m']==pytest.approx(1.8)
    assert r['heading_error_rad']==pytest.approx(0.)
    assert r['motion_authorized'] is False


def test_corridor_does_not_exist_from_one_plane_or_inconsistent_normals():
    assert corridor_from_planes({'left':dict(status='unknown')})['status']=='unknown'
    planes={v:dict(status='conditional_vertical_plane',distance_m=.9,normal_rig=n)
            for v,n in [('left',[.6,.8,0.]),('right',[.6,-.8,0.])]}
    assert corridor_from_planes(planes)['status']=='unknown'


def test_distant_plane_with_tiny_flow_cannot_claim_metric_range():
    # The plane exactly matches its correspondences but is not sufficiently
    # distinguishable from a separately fitted rotation on held-out points.
    from roadvision.corridor_rgb_probe import _project
    c=GroundCalibration();k=np.array([[64.,0,64.],[0,64.,64.],[0,0,1.]])
    a=np.array([[x,y] for y in (15.,30.,45.,60.) for x in (20.,40.,60.,80.,100.)])
    target=_project(k@(np.eye(3)+np.outer([0.,0.,-.03],[0.,0.,1/6]))@np.linalg.inv(k),a)
    region=dict(view='front',tracks=[dict(first=p.tolist(),last=q.tolist(),feature_split='train' if i%2==0 else 'validation') for i,(p,q) in enumerate(zip(a,target))])
    motion=dict(status='conditional_ground_motion',translation_m=[.03,0.],yaw_delta_rad=0.)
    r=estimate_vertical_plane(region,motion,c)
    assert r['status']=='unknown' and r['reason']=='plane_rotation_competition_unresolved'


@pytest.mark.parametrize('reason',['poor_spatial_support','planar_model_unsupported','temporal_models_inconsistent'])
def test_metric_plane_cannot_bypass_existing_spatial_and_temporal_rejections(reason):
    region=plane_region();region['reasons']=[reason]
    motion=dict(status='conditional_ground_motion',translation_m=[.12,0.],yaw_delta_rad=0.)
    assert estimate_vertical_plane(region,motion,GroundCalibration())['status']=='unknown'


def test_two_side_walls_cannot_masquerade_as_a_near_front_plane():
    a=np.array([[u,v] for v in (12.,28.,44.,60.) for u in (14.,22.,30.,38.,46.,82.,90.,98.,106.,114.)])
    rays=(a-64.)/64.;depth=.9/np.abs(rays[:,0]);target=64.+64.*rays*depth[:,None]/(depth[:,None]-.03)
    region=dict(view='front',tracks=[dict(first=p.tolist(),last=q.tolist(),feature_split='train' if i%2==0 else 'validation') for i,(p,q) in enumerate(zip(a,target))])
    motion=dict(status='conditional_ground_motion',translation_m=[.03,0.],yaw_delta_rad=0.)
    r=estimate_vertical_plane(region,motion,GroundCalibration())
    assert r['status']=='unknown'


def test_a_view_cannot_be_supported_only_by_validation_tracks():
    regions=observations()
    # Make two views dominate the global count, hiding a rejected third
    # training population. The third holdout still follows the true motion.
    for v in ('front','left'):regions[v]['tracks']*=5
    for t in regions['right']['tracks']:
        if t['feature_split']=='train':t['last'][0]+=7
    r=estimate_ground_motion(regions,GroundCalibration())
    assert r['status']=='unknown'
    assert r['reason']=='per_view_training_support_lost'


def test_conditional_ground_estimate_reports_model_conditioning_not_certification():
    r=estimate_ground_motion(observations(),GroundCalibration())
    u=r['conditional_uncertainty']
    assert u['pixel_noise_floor_px']>=.25
    assert u['translation_std_m']>0 and u['yaw_std_rad']>0
    assert not u['certified_error_bound']
