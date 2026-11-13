"""Rejected pure-cache experiment; default integration has been restored.

Frozen hook/integration tests are retained with the measured candidate source.
These delivered tests exercise the standalone context and unchanged defaults.
"""
import gc
import weakref

import cv2
import numpy as np
import pytest

from roadvision import paired_descriptor_checks as pair
from roadvision import rgb_motion_checks as h, rgb_resampling_checks as i
from roadvision.paired_patch_preparation import TrackAppearance
from tools.straight_paired_population_benchmark import local_cases


@pytest.mark.parametrize('case_index', range(6))
def test_complete_local_packets_equal_independent(case_index):
    _, images, region = local_cases()[case_index]
    want = dict(H=pair.filter_h(images, region), I=pair.filter_i(images, region))
    assert pair.filter_pair(images, region) == want


@pytest.mark.parametrize('point', [[5., 64.], [6., 64.], [121.999, 64.], [122., 64.],
                                  [44.125, 60.875], [np.nan, 64.], [np.inf, 64.]])
@pytest.mark.parametrize('kind', ['random', 'flat', 'padding', 'fractional_padding'])
def test_standalone_preparation_exact_with_boundary_flat_and_padding(kind, point):
    rng=np.random.default_rng(801)
    a=rng.integers(0,256,(128,128),dtype=np.uint8)
    b=cv2.warpAffine(a,np.array([[1.,0.,1.25],[0.,1.,.5]]),(128,128))
    if kind == 'flat': a.fill(70); b.fill(70)
    mask=np.ones_like(a)
    if kind == 'padding': mask[:, :50]=0
    if kind == 'fractional_padding': mask[:, :39]=0
    context=TrackAppearance([a,b])
    prepared=[context.patch(fn,gray,point) for fn,gray in [(h._patch,a),(i._patch,b)]]
    expected=[h._patch(a,point),i._patch(b,point)]
    for got,want in zip(prepared,expected):
        if want is None: assert got is None
        else: np.testing.assert_array_equal(got,want)
    if all(patch is not None for patch in prepared):
        assert context.ncc(h._ncc,*prepared)==h._ncc(*expected)
    # Support rejection remains external to the standalone raw patch context.
    for gray,raw in zip([a,b],prepared):
        supported=i._supported_patch(gray,point,mask)
        if raw is None: assert supported is None
        else:
            valid=cv2.getRectSubPix(mask.astype(np.float32),(11,11),tuple(map(float,point)))
            if np.all(valid>=1.-1e-6): np.testing.assert_array_equal(supported,raw)
            else: assert supported is None


def test_exact_canvas_point_and_function_required():
    gray=np.zeros((128,128),np.uint8)
    changed=gray.copy(); changed[0,0]=1
    context=TrackAppearance([gray, gray.copy(), changed, gray.astype(np.float32)])
    calls=[]
    def patch(g,p): calls.append(1); return np.ones((11,11),np.float32)
    point=np.array([64.,64.])
    first=context.patch(patch,gray,point)
    equal=context.canvases[next(key for key,entry in context.canvases.items() if entry[0] is not gray and entry[1] == 0)][0]
    assert context.patch(patch,equal,point.copy()) is first
    context.patch(patch,changed,point)
    floating=next(entry[0] for entry in context.canvases.values() if entry[0].dtype==np.float32)
    context.patch(patch,floating,point)
    context.patch(patch,gray,np.array([64.,64.],np.float32))
    context.patch(patch,gray,point+np.array([.125,0.]))
    context.patch(lambda g,p: patch(g,p),gray,point)
    assert len(calls)==6 and len(context.patches)<=4


def test_unknown_canvas_and_malformed_points_use_original_errors():
    gray=np.zeros((128,128),np.uint8); context=TrackAppearance([gray])
    other=gray.copy(); calls=[]
    def patch(g,p): calls.append(1); return h._patch(g,p)
    assert context.patch(patch,other,[64.,64.]) is None
    assert context.patch(patch,other,[64.,64.]) is None
    assert not context.patches
    assert context.patch(patch,gray,[64.]) is None
    assert not context.patches
    with pytest.raises(ValueError): context.patch(patch,gray,['bad','data'])
    assert len(calls)==4


def test_none_patch_cached_and_signed_zero_point_not_conflated():
    gray=np.zeros((128,128),np.uint8); context=TrackAppearance([gray]); calls=[]
    def patch(g,p): calls.append(1); return None
    for _ in range(2): assert context.patch(patch,gray,[0.,64.]) is None
    assert context.patch(patch,gray,[-0.,64.]) is None
    assert len(calls)==2


def test_ncc_reuse_requires_same_function_and_ordered_immutable_patches():
    context=TrackAppearance([]); rng=np.random.default_rng(922)
    a=rng.normal(size=(11,11)).astype(np.float32); b=a[::-1].copy()
    calls=[]
    def ncc(a,b): calls.append(1); return h._ncc(a,b)
    expected=ncc(a,b); calls.clear()
    assert context.ncc(ncc,a,b)==context.ncc(ncc,a,b)==expected
    context.ncc(ncc,b,a)
    context.ncc(ncc,a.copy(),b)
    context.ncc(lambda a,b:ncc(a,b),a,b)
    assert len(calls)==4 and len(context.correlations)==2


@pytest.mark.parametrize('bad', [False,True])
def test_ncc_none_and_nonfinite_values_not_repaired(bad):
    context=TrackAppearance([]); calls=[]; a=np.ones((11,11),np.float32)
    def ncc(a,b): calls.append(1); return float('nan') if bad else None
    first=context.ncc(ncc,a,a); second=context.ncc(ncc,a,a)
    assert len(calls)==1
    if bad: assert np.isnan(first) and np.isnan(second)
    else: assert first is second is None


def test_cache_clear_releases_patch_and_ncc_references():
    gray=np.zeros((128,128),np.uint8); context=TrackAppearance([gray])
    a=context.patch(lambda g,p:np.ones((11,11),np.float32),gray,[64.,64.])
    reference=weakref.ref(a); context.ncc(lambda a,b:1.,a,a)
    del a; context.clear(); gc.collect()
    assert not context.patches and not context.correlations and reference() is None


@pytest.mark.parametrize('rectified', [False,True])
def test_default_filter_does_not_construct_rejected_context(rectified,monkeypatch):
    _,images,region=local_cases()[5 if rectified else 0]
    region['tracks']=region['tracks'][:2]
    def forbidden(*args,**kwargs): raise AssertionError('rejected context selected')
    monkeypatch.setattr(TrackAppearance,'__init__',forbidden)
    assert 'TrackAppearance' not in vars(pair)
    assert pair.filter_pair(images,region)==dict(H=pair.filter_h(images,region),I=pair.filter_i(images,region))


def test_padding_support_is_still_sampled_even_with_shared_raw_patches(monkeypatch):
    _,images,region=local_cases()[0];region['tracks']=region['tracks'][:2]
    original=i.prepare_canvases
    def prepare(*args):
        gray,masks=original(*args);masks[0][:,:25]=0;return gray,masks
    monkeypatch.setattr(i,'prepare_canvases',prepare)
    got=pair.filter_pair(images,region)
    assert got == dict(H=pair.filter_h(images,region),I=pair.filter_i(images,region))
    assert got['H']['descriptor_evidence'][0]['accepted']
    assert got['I']['descriptor_evidence'][0]['reason']=='patch_boundary_flat_or_padding'


def test_standalone_pure_patch_and_ncc_actually_run_once_per_identical_input(monkeypatch):
    _,images,region=local_cases()[0];region['tracks']=region['tracks'][:1]
    gray=[cv2.cvtColor(images[j],cv2.COLOR_RGB2GRAY) for j in [0,2]]
    point=np.array(region['tracks'][0]['first'],float)
    context=TrackAppearance(gray)
    original_patch=h._patch;original_ncc=h._ncc;calls=[]
    def patch(*args): calls.append('patch'); return original_patch(*args)
    def ncc(*args): calls.append('ncc'); return original_ncc(*args)
    a=context.patch(patch,gray[0],point);b=context.patch(patch,gray[1],point)
    assert context.patch(patch,gray[0],point) is a
    assert context.patch(patch,gray[1],point) is b
    assert context.ncc(ncc,a,b)==context.ncc(ncc,a,b)
    assert calls==['patch','ncc']
    monkeypatch.setattr(h,'_patch',patch);monkeypatch.setattr(i,'_patch',patch)
    monkeypatch.setattr(h,'_ncc',ncc);monkeypatch.setattr(i,'_ncc',ncc)
    calls.clear();pair.filter_h(images,region);pair.filter_i(images,region)
    assert calls==['patch','patch','ncc','patch','patch','ncc']


def test_separate_patch_ncc_functions_are_not_assumed_equal(monkeypatch):
    _,images,region=local_cases()[0];region['tracks']=region['tracks'][:1]
    monkeypatch.setattr(h,'_ncc',lambda *args:.99)
    monkeypatch.setattr(i,'_ncc',lambda *args:.69)
    original=i._patch
    monkeypatch.setattr(i,'_patch',lambda *args: original(*args).copy())
    assert pair.filter_pair(images,region)==dict(H=pair.filter_h(images,region),I=pair.filter_i(images,region))
