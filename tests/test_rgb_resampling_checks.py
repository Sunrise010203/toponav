import copy

import cv2
import numpy as np
import pytest

from roadvision.rgb_resampling_checks import (
    appearance_evidence, prepare_canvases, filter_region, probe_resampling_checks,
)
from roadvision.corridor_rgb_probe import CALIBRATION_ID


def texture(seed=73):
    return cv2.GaussianBlur(np.random.default_rng(seed).integers(0, 255, (128,128), dtype=np.uint8), (3,3), .6)


def test_unique_and_wrong_correspondence():
    a=texture().astype(np.float32);b=np.roll(a,4,axis=1)
    mask=np.ones((128,128),np.uint8)
    assert appearance_evidence(a,b,[50.,50.],[54.,50.],mask,mask)['accepted']
    assert not appearance_evidence(a,b,[50.,50.],[74.,60.],mask,mask)['accepted']


def test_repeated_still_searches_full_canvas_both_directions():
    a=np.tile(texture()[:16,:16],(8,8)).astype(np.float32)
    mask=np.ones((128,128),np.uint8)
    e=appearance_evidence(a,a,[40.,40.],[40.,40.],mask,mask)
    assert not e['accepted'] and e['alternative_ncc']>.99
    assert e['reverse_alternative_ncc']>.99 and e['self_alternative_ncc']>.99


@pytest.mark.parametrize('bad',[np.nan,np.inf,-np.inf])
def test_invalid_coordinates_fail_closed(bad):
    a=texture().astype(np.float32);mask=np.ones((128,128),np.uint8)
    assert not appearance_evidence(a,a,[bad,40.],[40.,40.],mask,mask)['accepted']


@pytest.mark.parametrize('kind',['flat','boundary','invalid_mask'])
def test_unsupported_patches_do_not_match_padding(kind):
    a=texture().astype(np.float32);mask=np.ones((128,128),np.uint8);point=[50.,50.]
    if kind=='flat':a[:]=0
    if kind=='boundary':point=[1.,1.]
    if kind=='invalid_mask':mask[48,48]=0
    assert not appearance_evidence(a,a,point,point,mask,mask)['accepted']


def test_no_competing_valid_area_rejects_not_assume_unique():
    a=texture().astype(np.float32);mask=np.zeros((128,128),np.uint8);mask[40:61,40:61]=1
    assert not appearance_evidence(a,a,[50.,50.],[50.,50.],mask,mask)['accepted']


def test_native_sampling_composes_warps_once_and_inputs_unchanged():
    images=[np.repeat(texture()[:,:,None],3,axis=2) for _ in range(3)]
    original=copy.deepcopy(images)
    warp=np.array([[1.1,0,5],[0,1.2,8],[0,0,1.]])
    align=np.array([[1.,0,4],[0,1,2],[0,0,1.]])
    gray,masks=prepare_canvases(images,warp,align)
    expected=cv2.warpPerspective(cv2.cvtColor(images[2],cv2.COLOR_RGB2GRAY),warp@align@np.linalg.inv(warp),(128,128),flags=cv2.INTER_LINEAR|cv2.WARP_INVERSE_MAP)
    np.testing.assert_array_equal(gray[1],expected)
    np.testing.assert_array_equal(gray[0],cv2.cvtColor(images[0],cv2.COLOR_RGB2GRAY))
    assert masks[0][10,10]
    for a,b in zip(images,original):np.testing.assert_array_equal(a,b)


def test_warp_padding_mask_propagates():
    images=[np.repeat(texture()[:,:,None],3,axis=2) for _ in range(3)]
    align=np.array([[1.,0,25],[0,1,0],[0,0,1]])
    _,masks=prepare_canvases(images,np.eye(3),align)
    assert masks[1][50,90] and not masks[1][50,103]


def test_filter_split_is_fixed_before_appearance(monkeypatch):
    import roadvision.rgb_resampling_checks as module
    tracks=[dict(first=[20+i*2.,20+i*2.],middle=[21+i*2.,20+i*2.],last=[22+i*2.,20+i*2.]) for i in range(20)]
    region=dict(view='front',rectified_ground_assumption=False,tracks=tracks)
    n=[0]
    def evidence(*args,**kwargs):
        i=n[0];n[0]+=1
        return dict(accepted=i%3!=0,reason='test')
    monkeypatch.setattr(module,'appearance_evidence',evidence)
    images=[np.repeat(texture()[:,:,None],3,axis=2) for _ in range(3)]
    r=filter_region(images,region)
    assert all(t['feature_split']==('train' if int((t['first'][0]-20)/2)%2==0 else 'validation') for t in r['tracks'])
    assert r['descriptor_alignment']['fit_track_indices']==list(range(0,20,2))


@pytest.mark.parametrize('kind',['few','truth','stale','order','dtype','calibration'])
def test_strict_input_contract(kind):
    frames=[dict(frame_id=str(i),captured_sim_s=i*.1,rgb=np.zeros((256,256,3),np.uint8)) for i in range(3)]
    calibration=CALIBRATION_ID
    if kind=='few':frames.pop()
    if kind=='truth':frames[0]['pose']=[0,0,0]
    if kind=='stale':frames[0]['captured_sim_s']=-1
    if kind=='order':frames.reverse()
    if kind=='dtype':frames[0]['rgb']=frames[0]['rgb'].astype(float)
    if kind=='calibration':calibration='wrong'
    with pytest.raises(ValueError):probe_resampling_checks(frames,now_s=.2,calibration_id=calibration)


def test_physical_outputs_unknown_and_baseline_unchanged():
    frames=[dict(frame_id=str(i),captured_sim_s=i*.1,rgb=np.zeros((256,256,3),np.uint8)) for i in range(3)]
    r=probe_resampling_checks(frames,now_s=.2,calibration_id=CALIBRATION_ID)
    assert r['review']['motion_authorized'] is False and r['review']['arrival_status']=='unverified'
    assert r['evidence_packet']['front_surface']['range_m'] is None
    assert r['baseline']['version']=='rgb_motion_checks_v1'
    assert len(r['regions'])==9 and r['shared_motion']['status']=='unknown'
    assert r['ablations']['without_rectified_repetition_veto']['shared_motion']['status']=='unknown'


def test_rectified_repetition_veto_cannot_be_bypassed(monkeypatch):
    import roadvision.rgb_resampling_checks as module
    images=[np.repeat(texture()[:,:,None],3,axis=2) for _ in range(3)]
    tracks=[dict(first=[60.,100.],middle=[60.,100.],last=[60.,100.])]
    region=dict(view='front',rectified_ground_assumption=True,tracks=tracks)
    monkeypatch.setattr(module,'appearance_evidence',lambda *a,**k:dict(accepted=True,reason='unique_patch_support'))
    monkeypatch.setattr(module,'rectified_repetition',lambda *a,**k:dict(accepted=False,reason='self_repetition',self_alternative_ncc=.99))
    r=filter_region(images,region)
    assert not r['tracks'] and r['descriptor_evidence'][0]['reason']=='rectified_self_repetition'
    ablation=filter_region(images,region,repetition_veto=False)
    assert len(ablation['tracks'])==1


def test_validation_correspondences_do_not_fit_prealignment():
    images=[np.repeat(texture()[:,:,None],3,axis=2) for _ in range(3)]
    tracks=[dict(first=[x,y],middle=[x+1,y],last=[x+2,y])
            for y in (20.,40.,60.,80.) for x in (20.,40.,60.,80.,100.)]
    from roadvision.rgb_motion_checks import split_tracks
    _,validation=split_tracks(tracks)
    region=dict(view='front',rectified_ground_assumption=False,tracks=tracks)
    before=filter_region(images,region)['descriptor_alignment']
    modified=copy.deepcopy(region)
    for i in validation:modified['tracks'][i]['last'][0]+=15
    after=filter_region(images,modified)['descriptor_alignment']
    assert before==after


def test_physical_packet_equal_to_frozen_h():
    frames=[dict(frame_id=str(i),captured_sim_s=i*.1,rgb=np.zeros((256,256,3),np.uint8)) for i in range(3)]
    r=probe_resampling_checks(frames,now_s=.2,calibration_id=CALIBRATION_ID)
    assert r['evidence_packet']==r['baseline']['evidence_packet']
    assert r['review']==r['baseline']['review']
