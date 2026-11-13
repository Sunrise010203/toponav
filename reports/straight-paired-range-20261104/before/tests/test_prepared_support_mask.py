"""Archived candidate checks; default remains the pre-experiment implementation."""
import gc
import importlib.util
from pathlib import Path
import weakref

import cv2
import numpy as np
import pytest

from roadvision import native_patch_contrast as default_fast, paired_descriptor_checks as default_pair

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT/'reports/straight-paired-support-20261104'


def archived(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# Explicit snapshots, not default monkeypatching and not a new performance run.
fast = archived('_support_candidate_contrast', ARCHIVE/'candidate/roadvision/native_patch_contrast.py')
pair = archived('_support_candidate_pair', ARCHIVE/'candidate/roadvision/paired_descriptor_checks.py')
pair.h_evidence, pair.i_evidence = fast.h_evidence, fast.i_evidence
from roadvision import rgb_resampling_checks as i
from roadvision.prepared_support_mask import PreparedSupportMask, prepare_support_masks, original_support_mask


def assert_patch(a,b):
    if a is None:assert b is None
    else:assert b is not None and a.dtype==b.dtype and a.shape==b.shape and a.tobytes()==b.tobytes()


@pytest.mark.parametrize('layout',['C','F','reverse','strided'])
@pytest.mark.parametrize('kind',['ones','padding','random'])
def test_prepared_sampling_values_and_footprints_exact(layout,kind,monkeypatch):
    rng=np.random.default_rng(7281)
    gray=rng.integers(0,256,(128,128),dtype=np.uint8)
    source=np.ones((128,128),np.uint8)
    if kind=='padding':source[:,:64]=0
    if kind=='random':source=rng.integers(0,3,(128,128),dtype=np.uint8)
    if layout=='F':source=np.asfortranarray(source)
    if layout=='reverse':source=source[::-1,::-1]
    if layout=='strided':
        storage=np.zeros((256,256),np.uint8);storage[::2,::2]=source;source=storage[::2,::2]
    before=source.tobytes();prepared=prepare_support_masks([source])[0]
    assert type(prepared) is PreparedSupportMask and prepared._canvas is None
    original=cv2.getRectSubPix;sampled=[]
    def sample(a,*args):
        out=original(a,*args)
        if a.dtype==np.float32 and a.shape==(128,128):sampled.append(out.copy())
        return out
    monkeypatch.setattr(cv2,'getRectSubPix',sample)
    for point in ([64.,64.],[64.125,64.375],[6.,6.],[121.,121.],[122.,64.],[np.nan,64.]):
        expected=i._supported_patch(gray,point,source)
        actual=fast.supported_patch(gray,point,prepared)
        assert_patch(expected,actual)
    assert prepared.sampling_canvas().tobytes()==source.astype(np.float32).tobytes()
    assert prepared.sampling_canvas() is prepared.sampling_canvas()
    assert len(sampled)==8  # Four valid gray footprints, original+prepared support samples.
    assert all(a.tobytes()==b.tobytes() for a,b in zip(sampled[::2],sampled[1::2]))
    assert source.tobytes()==before


@pytest.mark.parametrize('kind',['float32','shape','subclass'])
def test_ineligible_masks_keep_original_object(kind):
    mask=np.ones((128,128),np.uint8)
    if kind=='float32':mask=mask.astype(np.float32)
    if kind=='shape':mask=mask[:127]
    if kind=='subclass':
        class Other(np.ndarray):pass
        mask=mask.view(Other)
    assert prepare_support_masks([mask])[0] is mask
    assert original_support_mask(mask) is mask


@pytest.mark.parametrize('kind',['boundary','flat'])
def test_conversion_remains_lazy_after_patch_rejection(kind):
    gray=np.zeros((128,128),np.uint8)
    if kind=='boundary':gray=np.random.default_rng(90).integers(0,256,gray.shape,dtype=np.uint8)
    prepared=PreparedSupportMask(np.ones_like(gray))
    assert fast.supported_patch(gray,[122.,64.] if kind=='boundary' else [64.,64.],prepared) is None
    assert prepared._canvas is None


def test_changed_support_callback_receives_original_mask(monkeypatch):
    mask=np.ones((128,128),np.uint8);prepared=PreparedSupportMask(mask);seen=[]
    def callback(gray,point,raw):seen.append(raw);return 'custom'
    monkeypatch.setattr(i,'_supported_patch',callback)
    assert fast.supported_patch(None,None,prepared)=='custom'
    assert seen==[mask] and seen[0] is mask and prepared._canvas is None


@pytest.mark.parametrize('keywords',[False,True])
def test_changed_evidence_callback_receives_original_masks(monkeypatch,keywords):
    a=np.ones((128,128),np.uint8);b=a.copy();pa,pb=prepare_support_masks([a,b]);seen=[]
    def callback(*args,**kwargs):seen.append((args,kwargs));return 'custom'
    monkeypatch.setattr(i,'appearance_evidence',callback)
    if keywords:
        assert fast.i_evidence(None,None,None,None,first_mask=pa,second_mask=pb)=='custom'
        assert seen[0][1]['first_mask'] is a and seen[0][1]['second_mask'] is b
    else:
        assert fast.i_evidence(None,None,None,None,pa,pb)=='custom'
        assert seen[0][0][4] is a and seen[0][0][5] is b
    assert pa._canvas is pb._canvas is None


def test_no_searches_unwrap_before_original_alternative_path():
    gray=np.random.default_rng(91).integers(0,256,(128,128),dtype=np.uint8)
    mask=np.ones_like(gray);prepared=PreparedSupportMask(mask);point=[64.125,64.375]
    assert fast.i_evidence(gray,gray,point,point,prepared,prepared)==i.appearance_evidence(gray,gray,point,point,mask,mask)
    assert prepared._canvas is None


def test_region_and_masks_release_without_global_retention():
    mask=np.ones((128,128),np.uint8);prepared=PreparedSupportMask(mask);canvas=prepared.sampling_canvas()
    refs=[weakref.ref(x) for x in (mask,prepared,canvas)]
    del mask,prepared,canvas;gc.collect()
    assert all(r() is None for r in refs)


@pytest.mark.parametrize('raises',[False,True])
def test_pair_owns_at_most_two_conversions_and_still_samples_each_patch(monkeypatch,raises):
    from tools.straight_paired_population_benchmark import local_cases
    _,images,region=local_cases()[0]
    original_init=PreparedSupportMask.__init__;original_canvas=PreparedSupportMask.sampling_canvas
    refs=[];calls=[];converted=[]
    def init(self,*args):original_init(self,*args);refs.append(weakref.ref(self))
    def canvas(self):
        calls.append(1)
        if self._canvas is None:converted.append(1)
        return original_canvas(self)
    monkeypatch.setattr(PreparedSupportMask,'__init__',init)
    monkeypatch.setattr(PreparedSupportMask,'sampling_canvas',canvas)
    if raises:
        def fail(*args):raise RuntimeError('injected')
        monkeypatch.setattr(i,'_ncc',fail)
        with pytest.raises(RuntimeError,match='injected'):pair.filter_pair(images,region)
    else:
        pair.filter_pair(images,region)
        assert len(calls)==50
    assert len(converted)==2 and len(refs)==2
    gc.collect();assert all(r() is None for r in refs)


def test_default_pair_and_contrast_remain_original_bytes():
    for module, name in ((default_pair, 'paired_descriptor_checks.py'),
                         (default_fast, 'native_patch_contrast.py')):
        assert Path(module.__file__).read_bytes() == (ARCHIVE/'before/roadvision'/name).read_bytes()
