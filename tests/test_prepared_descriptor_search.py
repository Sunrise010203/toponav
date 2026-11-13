"""Exact call-local preparation contracts, not navigation acceptance."""
import copy

import cv2
import numpy as np
import pytest

from roadvision.prepared_descriptor_search import PreparedAlternative
from roadvision import rgb_motion_checks as h
from roadvision import rgb_resampling_checks as i
from roadvision.prepared_descriptor_checks import filter_h, filter_i


def texture(kind='random', seed=1721):
    rng = np.random.default_rng(seed)
    image = rng.integers(0, 256, (128, 128), dtype=np.uint8)
    if kind == 'float':
        image = image.astype(np.float32) + rng.normal(0, .1, image.shape).astype(np.float32)
    elif kind == 'blur':
        image = cv2.GaussianBlur(image, (3, 3), .6)
    elif kind == 'flat':
        image[:] = 127
    elif kind == 'repeated':
        image = np.tile(image[:16, :16], (8, 8))
    elif kind == 'low_contrast':
        image = (image % 5 + 120).astype(np.uint8)
    return image


def prepared(a, b, masks=None):
    return tuple(PreparedAlternative(im, patch_px=11, half=5, exclude_radius=11.,
        mask=None if masks is None else mask) for im, mask in
        zip((a, b), (None, None) if masks is None else masks))


@pytest.mark.parametrize('kind', ['random', 'float', 'blur', 'flat', 'repeated', 'low_contrast'])
@pytest.mark.parametrize('point', [[64.,64.], [6.,6.], [121.999,64.], [62.125,81.875], [30.5,30.5]])
def test_raw_primitive_scores_are_bit_exact(kind, point):
    target = texture(kind);template = texture('float', 39)[44:55, 50:61].copy()
    search = PreparedAlternative(target, patch_px=11, half=5, exclude_radius=11.).at(point)
    assert search(template) == h._alternative(template, target, point)


@pytest.mark.parametrize('kind', ['random', 'float', 'flat', 'repeated'])
@pytest.mark.parametrize('mask_kind', ['all', 'none', 'padding', 'disconnected', 'one_patch'])
def test_masked_primitive_scores_and_exact_valid_population(kind, mask_kind):
    target = texture(kind);mask = np.ones((128,128), np.uint8)
    if mask_kind == 'none':mask[:] = 0
    elif mask_kind == 'padding':mask[:, :41] = 0
    elif mask_kind == 'disconnected':mask[20:50, :] = 0;mask[:, 80:100] = 0
    elif mask_kind == 'one_patch':mask[:] = 0;mask[59:70, 59:70] = 1
    template = texture('float', 39)[44:55,50:61].copy()
    context = PreparedAlternative(target, patch_px=11, half=5, exclude_radius=11., mask=mask)
    for point in ([64.,64.], [44.125,60.875]):
        assert context.at(point)(template) == i._masked_alternative(template, target, point, mask)


@pytest.mark.parametrize('masked', [False, True])
@pytest.mark.parametrize('kind', ['random', 'float', 'flat', 'repeated', 'low_contrast', 'blur'])
@pytest.mark.parametrize('source,target', [([64.,64.],[66.25,64.5]), ([6.,6.],[6.,6.]),
    ([5.999,64.],[64.,64.]), ([np.nan,64.],[64.,64.])])
def test_complete_evidence_and_failure_reasons_exact(masked, kind, source, target):
    a = texture(kind);b = np.roll(a, 2, axis=1);masks = [np.ones((128,128),np.uint8)] * 2
    if masked:
        expected = i.appearance_evidence(a,b,source,target,*masks)
        actual = i.appearance_evidence(a,b,source,target,*masks,_searches=prepared(a,b,masks))
    else:
        expected = h.patch_evidence(a,b,source,target)
        actual = h.patch_evidence(a,b,source,target,_searches=prepared(a,b))
    assert actual == expected


@pytest.mark.parametrize('masked', [False, True])
def test_keeps_four_actual_template_searches_and_prepares_only_two_masks(monkeypatch, masked):
    a=texture();b=np.roll(a,2,axis=1);mask=np.ones_like(a);calls=[];erosions=[]
    match=cv2.matchTemplate;erode=cv2.erode
    def counted(*args,**kwargs):calls.append(1);return match(*args,**kwargs)
    def erosion(*args,**kwargs):erosions.append(1);return erode(*args,**kwargs)
    monkeypatch.setattr(cv2,'matchTemplate',counted);monkeypatch.setattr(cv2,'erode',erosion)
    contexts=prepared(a,b,[mask,mask] if masked else None)
    for point in ([64.,64.],[70.125,71.5],[80.,80.]):
        if masked:i.appearance_evidence(a,b,point,point,mask,mask,_searches=contexts)
        else:h.patch_evidence(a,b,point,point,_searches=contexts)
    assert len(calls)==12
    assert len(erosions)==(2 if masked else 0)


@pytest.mark.parametrize('masked', [False, True])
def test_nonfinite_score_population_and_no_alternatives_match(monkeypatch, masked):
    image=texture();mask=np.ones_like(image);point=[64.,64.];template=image[59:70,59:70].astype(np.float32)
    score=np.full((118,118),np.nan,np.float32);score[0,0]=np.inf;score[10,10]=.94
    monkeypatch.setattr(cv2,'matchTemplate',lambda *a,**k:score.copy())
    context=PreparedAlternative(image,patch_px=11,half=5,exclude_radius=11.,mask=mask if masked else None)
    oracle=(lambda: i._masked_alternative(template,image,point,mask)) if masked else (lambda:h._alternative(template,image,point))
    assert context.at(point)(template)==oracle()
    score[:]=np.nan
    assert context.at(point)(template)==oracle()


@pytest.mark.parametrize('masked', [False, True])
@pytest.mark.parametrize('observed,alternative', [(None,.5),(.7,.64), (np.nextafter(.7,0.),.5),
    (.9,np.nextafter(.84,0.)),(.9,np.nextafter(.84,1.)),(1.,.94),(1.,np.nextafter(.94,0.)),(.69,.95)])
def test_gate_boundaries_are_not_rounded_or_replaced(monkeypatch,masked,observed,alternative):
    module=i if masked else h;image=texture();mask=np.ones_like(image);p=[64.,64.]
    monkeypatch.setattr(module,'_ncc',lambda *args:observed)
    value=(alternative,123) if masked else alternative
    monkeypatch.setattr(module,'_masked_alternative' if masked else '_alternative',lambda *args:value)
    class Context:
        def at(self,point):return lambda template:value
    contexts=(Context(),Context())
    if masked:
        assert i.appearance_evidence(image,image,p,p,mask,mask)==i.appearance_evidence(image,image,p,p,mask,mask,_searches=contexts)
    else:
        assert h.patch_evidence(image,image,p,p)==h.patch_evidence(image,image,p,p,_searches=contexts)


@pytest.mark.parametrize('rectified', [False, True])
@pytest.mark.parametrize('kind', ['random','flat','repeated'])
@pytest.mark.parametrize('channel', ['H','I'])
def test_whole_region_alignment_evidence_models_and_inputs_unchanged(rectified,kind,channel):
    a=texture(kind);images=[np.repeat(np.roll(a,n,axis=1)[:,:,None],3,axis=2) for n in (0,1,2)]
    tracks=[dict(first=[float(x),float(y)],middle=[float(x+1),float(y)],last=[float(x+2),float(y)])
        for y in (20,40,60,80,100) for x in (20,40,60,80,100)]
    region=dict(view='front',rectified_ground_assumption=rectified,tracks=tracks)
    original_images=copy.deepcopy(images);original_region=copy.deepcopy(region)
    old=h.filter_region if channel=='H' else i.filter_region
    fast=filter_h if channel=='H' else filter_i
    assert fast(images,region)==old(images,region)
    assert region==original_region
    for actual,expected in zip(images,original_images):np.testing.assert_array_equal(actual,expected)


def test_default_s1_filter_does_not_construct_prepared_context(monkeypatch):
    import roadvision.prepared_descriptor_search as module
    def forbidden(*a,**k):raise AssertionError('default S1 selected preparation')
    monkeypatch.setattr(module,'PreparedAlternative',forbidden)
    image=np.repeat(texture()[:,:,None],3,axis=2)
    region=dict(view='front',rectified_ground_assumption=False,tracks=[])
    h.filter_region([image]*3,region);i.filter_region([image]*3,region)


def test_context_is_never_reused_by_a_new_region_call(monkeypatch):
    import roadvision.prepared_descriptor_search as module
    contexts=[];cls=module.PreparedAlternative
    def counted(*a,**k):
        c=cls(*a,**k);contexts.append(c);return c
    monkeypatch.setattr(module,'PreparedAlternative',counted)
    image=np.repeat(texture()[:,:,None],3,axis=2)
    region=dict(view='front',rectified_ground_assumption=False,tracks=[])
    for fn in (filter_h,filter_i):
        fn([image]*3,region);image[:]=128;fn([image]*3,region)
    assert len(contexts)==8 and len({id(c) for c in contexts})==8
