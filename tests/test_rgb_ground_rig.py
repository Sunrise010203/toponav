"""Independent algebra and conservative interface tests for the new prototype."""
import numpy as np
import pytest
from roadvision.rgb_ground_rig import (ground_map, track_region, shared_motion, rig_basis,
                                       probe_ground_rig)
from roadvision.corridor_rgb_probe import CALIBRATION_ID


def test_rectification_roundtrip():
    h = ground_map()
    p = np.array([[-1., 2., 1.], [.7, 4., 1.]])
    uv = p @ h.T
    uv /= uv[:, 2:]
    back = uv @ np.linalg.inv(h).T
    back /= back[:, 2:]
    np.testing.assert_allclose(back, p, atol=1e-10)
    assert (uv[:, 1] > 64).all()


def test_basis_is_right_handed():
    for view in ['front', 'left', 'right']:
        b = rig_basis(view)
        np.testing.assert_allclose(b.T @ b, np.eye(3), atol=1e-10)
        assert np.linalg.det(b) == pytest.approx(1.)


def model(view, normal, direction, rotation=None):
    b = rig_basis(view)
    r = np.eye(3) if rotation is None else rotation
    return {'view': view, 'plane_hypotheses': [
        {'rotation': (b.T @ r @ b).tolist(),
         'translation_over_plane_distance': (b.T @ np.array(direction)).tolist(),
         'normal_camera': (b.T @ np.array(normal)).tolist()}]}


def test_two_planes_same_motion():
    a = model('front', [0,0,1], [1,0,0])
    b = model('left', [0,1,0], [-2,0,0])  # sign/scale are unresolved
    r = shared_motion({'a': a, 'b': b})
    assert r['status'] == 'shared_motion_candidate'
    assert r['max_supported_regions'] == 2
    assert r['metric_scale'] is None


@pytest.mark.parametrize('kind', ['same_plane', 'same_view', 'different_axis', 'different_rotation'])
def test_degenerate_or_conflicting_planes(kind):
    a = model('front', [0,0,1], [1,0,0])
    b = model('left', [0,1,0], [1,0,0])
    if kind == 'same_plane': b = model('left', [0,0,1], [1,0,0])
    if kind == 'same_view': b = model('front', [0,1,0], [1,0,0])
    if kind == 'different_axis': b = model('left', [0,1,0], [0,1,0])
    if kind == 'different_rotation':
        angle = .2
        r = np.array([[np.cos(angle), -np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
        b = model('left', [0,1,0], [1,0,0], r)
    assert shared_motion({'a':a, 'b':b})['status'] == 'unknown'


def test_blank_track_and_safety():
    images = [np.zeros((128,128,3), np.uint8)]*3
    r = track_region(images, rectified=True)
    assert r['status'] == 'unknown'
    assert r['stage_counts']['retained'] == 0
    frames = [dict(frame_id=str(i), captured_sim_s=i*.1,
                   rgb=np.zeros((256,256,3), np.uint8)) for i in range(3)]
    result = probe_ground_rig(frames, now_s=.2, calibration_id=CALIBRATION_ID)
    assert not result['review']['motion_authorized']
    assert result['review']['arrival_status'] == 'unverified'
    assert result['evidence_packet']['front_surface']['range_m'] is None


def test_wrong_calibration():
    with pytest.raises(ValueError):
        probe_ground_rig([], now_s=0, calibration_id='wrong')


def test_all_branches_and_joint_not_merely_pairwise_chain():
    a=model('front',[0,0,1],[1,0,0])
    b=model('left',[0,1,0],[0,1,0])
    b['plane_hypotheses'] += model('left',[0,1,0],[1,0,0])['plane_hypotheses']
    r=shared_motion({'a':a,'b':b})
    assert r['status']=='shared_motion_candidate'
    assert all(h['branches'][1]['branch']==1 for h in r['hypotheses'])
    def direction(deg):
        t=np.radians(deg); return [np.cos(t),np.sin(t),0]
    regions={k:model(v,n,direction(deg)) for k,v,n,deg in [
        ('a','front',[0,0,1],0),('b','left',[0,1,0],10),('c','right',[1,0,0],20)]}
    assert shared_motion(regions)['max_supported_regions']==2  # A-C disagree


@pytest.mark.parametrize('kind',['zero_t','zero_n','nan'])
def test_invalid_or_rotation_only_branches_not_motion(kind):
    a=model('front',[0,0,1],[1,0,0])
    b=model('left',[0,1,0],[1,0,0])
    branch=b['plane_hypotheses'][0]
    if kind=='zero_t': branch['translation_over_plane_distance']=[0,0,0]
    if kind=='zero_n': branch['normal_camera']=[0,0,0]
    if kind=='nan': branch['normal_camera']=[float('nan'),0,0]
    assert shared_motion({'a':a,'b':b})['status']=='unknown'


def test_failed_lk_points_not_sent_downstream(monkeypatch):
    import roadvision.rgb_ground_rig as module
    points=np.array([[20,20],[40,40],[60,60]],np.float32)
    monkeypatch.setattr(module,'_unique_corners',lambda gray:(points,points.copy(),0))
    sizes=[]
    def lk(a,b,p):
        sizes.append(len(p))
        valid=np.ones(len(p),bool)
        if len(sizes)==1: valid[0]=False
        return p.copy(),valid
    monkeypatch.setattr(module,'_lk',lk)
    r=module.track_region([np.zeros((128,128,3),np.uint8)]*3)
    assert sizes==[3,2,2,2,2,2]
    assert r['stage_counts']['retained']==2


@pytest.mark.parametrize('kind',['extra_truth','stale','order','dtype','few'])
def test_inherited_strict_contract(kind):
    frames=[dict(frame_id=str(i),captured_sim_s=i*.1,rgb=np.zeros((256,256,3),np.uint8)) for i in range(3)]
    if kind=='extra_truth': frames[0]['pose']=[0,0,0]
    if kind=='stale': frames[0]['captured_sim_s']=-1.
    if kind=='order': frames.reverse()
    if kind=='dtype': frames[0]['rgb']=frames[0]['rgb'].astype(float)
    if kind=='few': frames.pop()
    with pytest.raises(ValueError): probe_ground_rig(frames,now_s=.2,calibration_id=CALIBRATION_ID)


def test_raw_stage_diagnostics_match_frozen_pixel_algorithm():
    from test_corridor_rgb_probe import window
    frames=window()
    result=probe_ground_rig(frames,now_s=.2,calibration_id=CALIBRATION_ID)
    assert result==probe_ground_rig(frames,now_s=.2,calibration_id=CALIBRATION_ID)
    for view in ('front','left','right'):
        raw=result['regions'][view+'_raw']
        assert raw['status']==result['baseline']['views'][view]['status']
        counts=raw['stage_counts']
        ordered=[counts[k] for k in ('unique','roi_valid','lk01','lk10','lk12','lk21','lk02','lk20','original_bounds','fb','path','retained')]
        assert ordered==sorted(ordered,reverse=True)
    assert result['evidence_packet']['corridor_boundary']['heading_rad'] is None


def test_region_coverage_is_explicit_not_silent_old_threshold_change():
    from roadvision.rgb_ground_rig import _rectification,_region_models
    from roadvision.corridor_rgb_probe import _project,K,_grid_count,_models
    canvas=np.array([[x,y] for x in (42,54,78,86) for y in (15,48,80,110)],float)
    warp=_rectification()
    a=_project(warp,canvas)
    hs=[K@(np.eye(3)+np.outer([0,0,-d],[0,1,0]))@np.linalg.inv(K) for d in (.1,.2)]
    b,c=[_project(h,a) for h in hs]
    assert _grid_count(a)<6
    assert 'planar_inliers_poorly_distributed' in _models(a,b,c)[1]
    result,_=_region_models(a,b,c,np.linalg.inv(warp))
    assert 'homography' in result
