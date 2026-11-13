"""Exact full-population scores with overwritten, call-local scratch."""
import copy

import cv2
import numpy as np
import pytest

from roadvision.descriptor_scratch_search import PreparedAlternative
from roadvision import rgb_motion_checks as h, rgb_resampling_checks as i
from roadvision.prepared_descriptor_checks import filter_h, filter_i


def image(seed=771):
    return np.random.default_rng(seed).integers(0, 256, (128,128), dtype=np.uint8)


@pytest.mark.parametrize('masked', [False, True])
@pytest.mark.parametrize('point', [[64.,64.], [5.,5.], [121.999,64.], [44.125,60.875],
    [-1000.,64.], [np.inf,64.], [np.nan,64.], [1e100,-1e100]])
def test_broadcast_coordinates_and_search_population_are_exact(masked, point):
    target=image(); mask=np.ones_like(target);mask[:20,:]=0;mask[:,80:85]=0
    c=PreparedAlternative(target, patch_px=11, half=5, exclude_radius=11., mask=mask if masked else None)
    assert c.x.shape == (1,118) and c.y.shape == (118,1)
    y,x=np.indices((118,118))
    a=(c.x-point[0])**2+(c.y-point[1])**2>11.**2
    b=(x+5-point[0])**2+(y+5-point[1])**2>11.**2
    np.testing.assert_array_equal(a,b)
    for seed in (72,81,99):
        template=image(seed)[30:41,40:51].astype(np.float32)
        expected=i._masked_alternative(template,target,point,mask) if masked else h._alternative(template,target,point)
        assert c.at(point)(template) == expected


@pytest.mark.parametrize('masked', [False, True])
def test_each_full_match_overwrites_poisoned_workspaces(masked, monkeypatch):
    target=image();mask=np.ones_like(target);point=[63.5,62.25]
    c=PreparedAlternative(target,patch_px=11,half=5,exclude_radius=11.,mask=mask if masked else None)
    search=c.at(point);score_id=id(c._scores);valid_id=id(c._valid)
    original=cv2.matchTemplate;seen=[]
    def match(target_arg,template,method,result):
        assert result is c._scores and method == cv2.TM_CCOEFF_NORMED
        output=original(target_arg,template,method,result)
        assert np.shares_memory(output,result)
        seen.append(1)
        return output
    for seed in range(6):
        template=image(seed)[30:41,40:51].astype(np.float32)
        expected=i._masked_alternative(template,target,point,mask) if masked else h._alternative(template,target,point)
        c._scores.fill(np.nan if seed%2 else np.inf);c._valid.fill(bool(seed%2))
        with monkeypatch.context() as patch:
            patch.setattr(cv2,'matchTemplate',match)
            assert search(template) == expected
        assert id(c._scores)==score_id and id(c._valid)==valid_id
    assert len(seen)==6  # NO read-from-prior-score shortcut, even same target.


@pytest.mark.parametrize('masked', [False, True])
def test_nonfinite_population_is_fully_recomputed_every_search(masked,monkeypatch):
    target=image();mask=np.ones_like(target);point=[64.,64.]
    c=PreparedAlternative(target,patch_px=11,half=5,exclude_radius=11.,mask=mask if masked else None)
    scores=np.full((118,118),np.nan,np.float32);template=target[20:31,30:41].astype(np.float32)
    monkeypatch.setattr(cv2,'matchTemplate',lambda *args,**kw:scores.copy())
    for value in (np.nan,np.inf,.94,-.2,np.nan):
        scores[10,10]=value
        expected=i._masked_alternative(template,target,point,mask) if masked else h._alternative(template,target,point)
        assert c.at(point)(template)==expected


@pytest.mark.parametrize('masked',[False,True])
def test_saved_endpoint_closures_do_not_reuse_previous_validity(masked):
    target=image();mask=np.ones_like(target);mask[:,40:60]=0
    c=PreparedAlternative(target,patch_px=11,half=5,exclude_radius=11.,mask=mask if masked else None)
    points=([64.,64.],[24.5,77.25],[105.5,18.125]);searches=[c.at(p) for p in points]
    for index in (0,2,1,0,1,2):
        template=image(index)[30:41,40:51].astype(np.float32);point=points[index]
        expected=i._masked_alternative(template,target,point,mask) if masked else h._alternative(template,target,point)
        assert searches[index](template)==expected


@pytest.mark.parametrize('rectified',[False,True])
@pytest.mark.parametrize('channel',['H','I'])
def test_complete_region_unchanged_and_every_buffer_is_call_local(rectified,channel,monkeypatch):
    import roadvision.prepared_descriptor_search as module
    images=[np.repeat(np.roll(image(),n,axis=1)[:,:,None],3,axis=2) for n in (0,1,2)]
    tracks=[dict(first=[float(x),float(y)],middle=[float(x+1),float(y)],last=[float(x+2),float(y)])
        for y in (20,40,60,80,100) for x in (20,40,60,80,100)]
    region=dict(view='front',rectified_ground_assumption=rectified,tracks=tracks)
    raw=h.filter_region if channel=='H' else i.filter_region
    fast=filter_h if channel=='H' else filter_i
    want=raw(images,region);original=copy.deepcopy(region);buffers=[];cls=PreparedAlternative
    def context(*args,**kwargs):
        c=cls(*args,**kwargs);buffers.append((c._scores,c._valid));return c
    monkeypatch.setattr(module,'PreparedAlternative',context)
    for _ in range(2):assert fast(images,region)==want
    assert region==original and len(buffers)==4
    for j,(scores,valid) in enumerate(buffers):
        for other_scores,other_valid in buffers[j+1:]:
            assert not np.shares_memory(scores,other_scores) and not np.shares_memory(valid,other_valid)
