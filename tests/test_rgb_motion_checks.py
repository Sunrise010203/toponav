import copy
import numpy as np
import pytest
import cv2
from roadvision.rgb_motion_checks import (patch_evidence, fit_plane_vector,
    plane_projection, split_tracks, anchored_support, probe_motion_checks)
from roadvision.corridor_rgb_probe import CALIBRATION_ID,K,_project


def texture(seed=3):
    rng=np.random.default_rng(seed)
    return cv2.GaussianBlur(rng.integers(0,255,(128,128),dtype=np.uint8),(3,3),.6)


def test_unique_patch_and_displaced_correspondence():
    a=texture(); b=np.roll(a,4,axis=1)
    e=patch_evidence(a,b,[50.,50.],[54.,50.])
    assert e['accepted'] and e['observed_ncc']>.99
    assert not patch_evidence(a,b,[50.,50.],[70.,60.])['accepted']


def test_repeated_and_flat_patches_reject():
    tile=texture()[:16,:16]
    a=np.tile(tile,(8,8))
    e=patch_evidence(a,a,[40.,40.],[40.,40.])
    assert not e['accepted'] and e['alternative_ncc']>.99
    assert not patch_evidence(np.zeros((128,128),np.uint8),a,[40.,40.],[40.,40.])['accepted']


def test_boundary_is_unknown_not_padding_match():
    a=texture()
    assert not patch_evidence(a,a,[1.,1.],[1.,1.])['accepted']


def test_plane_vector_fit_independent_geometry():
    p=np.array([[x,y] for x in (10.,40.,80.,110.) for y in (20.,50.,90.,115.)])
    r=cv2.Rodrigues(np.array([.01,.02,-.01]))[0]
    t=np.array([.2,.03,.1]);t/=np.linalg.norm(t)
    q=np.array([.01,.04,.12])
    h=K@(r+np.outer(t,q))@np.linalg.inv(K)
    target=_project(h,p)
    estimate=fit_plane_vector(p,target,r,t)
    np.testing.assert_allclose(estimate,q,atol=1e-10)
    np.testing.assert_allclose(plane_projection(p,r,t,estimate),target,atol=1e-9)


def test_split_disjoint_deterministic_and_no_point_loss():
    tracks=[dict(first=[i%5*20+10,i//5*20+10],last=[0,0]) for i in range(20)]
    a,b=split_tracks(tracks)
    assert not set(a)&set(b) and set(a)|set(b)==set(range(20))
    assert (a,b)==split_tracks(tracks)


def test_empty_and_invalid_inputs_conservative():
    assert anchored_support({})['status']=='unknown'
    frames=[dict(frame_id=str(i),captured_sim_s=i*.1,rgb=np.zeros((256,256,3),np.uint8)) for i in range(3)]
    before=copy.deepcopy(frames)
    result=probe_motion_checks(frames,now_s=.2,calibration_id=CALIBRATION_ID)
    assert result['review']['motion_authorized'] is False
    assert result['review']['arrival_status']=='unverified'
    assert result['evidence_packet']['front_surface']['range_m'] is None
    for a,b in zip(frames,before): assert np.array_equal(a['rgb'],b['rgb'])
    with pytest.raises(ValueError):probe_motion_checks(frames,now_s=.2,calibration_id='wrong')


def region(view,normal,magnitude,*,candidate=True):
    from roadvision.rgb_ground_rig import rig_basis
    b=rig_basis(view);r=np.eye(3);t=b.T@np.array([-1.,0,0]);q=b.T@np.array(normal,float)*magnitude
    a=np.array([[x,y] for y in (15.,36.,57.,78.,99.,115.) for x in (12.,35.,58.,81.,105.)])
    c=plane_projection(a,r,t,q)
    middle=plane_projection(a,r,t,q*.5)
    return dict(view=view,rectified_ground_assumption=False,
                status='planar_flow_candidate' if candidate else 'unknown',
                reasons=[] if candidate else ['rotation_or_low_parallax'],
                tracks=[dict(first=p.tolist(),middle=m.tolist(),last=s.tolist()) for p,m,s in zip(a,middle,c)])


def test_weak_side_plane_supports_high_parallax_anchor():
    a=region('front',[0,0,1],.15)
    b=region('left',[0,1,0],.03,candidate=False)
    r=anchored_support({'anchor':a,'weak':b})
    assert r['status']=='anchored_motion_candidate'
    assert any(h['support']=='weak' for h in r['hypotheses'])
    for h in r['hypotheses']:
        for key,train in h['fit_indices'].items(): assert not set(train)&set(h['validation_indices'][key])
        assert h['support_validation']['rotation_gain_px']>=.2


@pytest.mark.parametrize('kind',['same_plane','same_view','no_anchor','corrupt_validation'])
def test_conditional_support_rejections(kind):
    a=region('front',[0,0,1],.15)
    b=region('left',[0,1,0],.03,candidate=False)
    if kind=='same_plane':b=region('left',[0,0,1],.15)
    if kind=='same_view':b=region('front',[0,1,0],.08)
    if kind=='no_anchor':a=region('front',[0,0,1],.03,candidate=False)
    if kind=='corrupt_validation':
        _,validation=split_tracks(b['tracks'])
        for i in validation:b['tracks'][i]['last'][0]+=10
    assert anchored_support({'a':a,'b':b})['status']=='unknown'


def test_fit_rejects_degenerate_training():
    a=np.ones((5,2))*20
    assert fit_plane_vector(a,a,np.eye(3),[1,0,0]) is None


def test_preserve_feature_assignment_after_descriptor_filtering():
    tracks=[dict(first=[i,i],feature_split='train' if i%3==0 else 'validation') for i in range(20)]
    a,b=split_tracks(tracks[3:])
    assert all(tracks[3:][i]['feature_split']=='train' for i in a)
    assert all(tracks[3:][i]['feature_split']=='validation' for i in b)


@pytest.mark.parametrize('kind',['few','extra_truth','stale','order','dtype'])
def test_strict_inputs(kind):
    frames=[dict(frame_id=str(i),captured_sim_s=i*.1,rgb=np.zeros((256,256,3),np.uint8)) for i in range(3)]
    if kind=='few':frames.pop()
    if kind=='extra_truth':frames[0]['pose']=[0,0,0]
    if kind=='stale':frames[0]['captured_sim_s']=-1
    if kind=='order':frames.reverse()
    if kind=='dtype':frames[0]['rgb']=frames[0]['rgb'].astype(float)
    with pytest.raises(ValueError):probe_motion_checks(frames,now_s=.2,calibration_id=CALIBRATION_ID)


def test_optimizer_exhaustion_remains_unknown(monkeypatch):
    from types import SimpleNamespace
    import roadvision.rgb_motion_checks as module
    monkeypatch.setattr(module,'least_squares',lambda *a,**k:SimpleNamespace(success=False,nfev=60))
    r=anchored_support({'a':region('front',[0,0,1],.15),'b':region('left',[0,1,0],.03,candidate=False)})
    assert r['status']=='unknown'
    assert r['pair_checks'] and all(p['reason']=='optimizer_not_converged' for p in r['pair_checks'])


def test_fixed_upper_roi_never_uses_truth_mask(monkeypatch):
    import roadvision.rgb_motion_checks as module
    tracks=[dict(first=[30,y],middle=[30,y],last=[30,y]) for y in (5,20,79,80,110)]
    source=dict(view='front',rectified_ground_assumption=False,tracks=tracks)
    baseline={'regions':{'front_raw':source},'evidence_packet':{},'review':{}}
    monkeypatch.setattr(module,'probe_ground_rig',lambda *a,**k:baseline)
    seen=[]
    def filtered(images,r):
        seen.append(r)
        return dict(view=r['view'],rectified_ground_assumption=False,status='unknown',reasons=['mock'],tracks=[])
    monkeypatch.setattr(module,'filter_region',filtered)
    frames=[{'rgb':np.zeros((256,256,3),np.uint8)}]*3
    result=module.probe_motion_checks(frames,now_s=.2,calibration_id=CALIBRATION_ID)
    assert len(seen)==2 and [t['first'][1] for t in seen[1]['tracks']]==[20,79]
    assert len(source['tracks'])==5
    assert 'front_upper' in result['regions']


@pytest.mark.parametrize('kind',['large_residual','no_rotation_gain','invalid_projection'])
def test_heldout_residual_rejections(kind):
    from roadvision.rgb_motion_checks import _residual_summary
    a=np.zeros((8,2));pred=np.zeros_like(a);rot=np.ones_like(a)*2
    if kind=='large_residual':pred+=3
    if kind=='no_rotation_gain':rot[:]=0
    if kind=='invalid_projection':pred=None
    assert not _residual_summary(pred,a,rot)['accepted']
