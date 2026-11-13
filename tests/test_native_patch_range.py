"""Exact range proof and explicitly archived candidate; not default promotion."""
import importlib.util
from pathlib import Path

import cv2
import numpy as np
import pytest

from roadvision.native_patch_range import sufficient_range_contrast

ROOT=Path(__file__).resolve().parents[1]
ARCHIVE=ROOT/'reports/straight-paired-range-20261104'
spec=importlib.util.spec_from_file_location('_range_candidate_contrast',ARCHIVE/'candidate/roadvision/native_patch_contrast.py')
fast=importlib.util.module_from_spec(spec)
spec.loader.exec_module(fast)


def exact_numerator(raw):
    a=raw.astype(np.int64)
    return 121*int((a*a).sum())-int(a.sum())**2


def verify(raw):
    before=raw.tobytes();decision=sufficient_range_contrast(raw)
    patch=raw.astype(np.float32)
    expected=float(patch.std())>=2.
    if decision is not None:
        assert decision==expected
        n=exact_numerator(raw)
        assert (n>9*121**2) if decision else (n<121**2)
    assert fast._contrast_gate(raw,patch)==expected
    assert raw.tobytes()==before


@pytest.mark.parametrize('kind',['two_extrema','balanced','one_high','one_low'])
@pytest.mark.parametrize('base',['zero','top'])
def test_every_uint8_span_with_extremal_populations(kind,base):
    for span in range(256):
        low=0 if base=='zero' else 255-span
        a=np.full(121,low+span//2,np.uint8)
        if kind=='two_extrema':a[0]=low;a[-1]=low+span
        elif kind=='balanced':a[:60]=low;a[60:]=low+span
        elif kind=='one_high':a[:]=low;a[-1]=low+span
        else:a[:]=low+span;a[0]=low
        verify(a.reshape(11,11))


@pytest.mark.parametrize('layout',['C','F','reverse','strided'])
def test_layouts_and_random_near_range_boundary(layout):
    rng=np.random.default_rng(110447)
    for span in (1,2,22,45,46,47,48,255):
        for _ in range(50):
            low=int(rng.integers(0,256-span))
            raw=rng.integers(low,low+span+1,(11,11),dtype=np.uint8)
            raw[0,0],raw[-1,-1]=low,low+span
            if layout=='F':raw=np.asfortranarray(raw)
            if layout=='reverse':raw=raw[::-1,::-1]
            if layout=='strided':
                storage=np.zeros((22,22),np.uint8);storage[::2,::2]=raw;raw=storage[::2,::2]
            verify(raw)


@pytest.mark.parametrize('span',[0,1,47,48,255])
def test_sufficient_span_does_not_allocate_integrals_or_call_std(span,monkeypatch):
    raw=np.full((11,11),span//2,np.uint8);raw[0,0]=0;raw[-1,-1]=span
    class Observed(np.ndarray):
        def std(self,*args,**kwargs):pytest.fail('proved far band must not need std')
    monkeypatch.setattr(cv2,'integral2',lambda *a,**k:pytest.fail('range shortcut should not allocate integrals'))
    assert fast._contrast_gate(raw,raw.astype(np.float32).view(Observed)) == (span>=47)


@pytest.mark.parametrize('span',[2,22,46])
def test_ambiguous_span_keeps_original_integral_gate(span,monkeypatch):
    raw=np.zeros((11,11),np.uint8);raw[0,0]=span
    original=cv2.integral2;calls=[]
    def integral(*args,**kwargs):calls.append(1);return original(*args,**kwargs)
    monkeypatch.setattr(cv2,'integral2',integral)
    assert sufficient_range_contrast(raw) is None
    assert fast._contrast_gate(raw,raw.astype(np.float32)) == (float(raw.astype(np.float32).std())>=2.)
    assert len(calls)==1


@pytest.mark.parametrize('kind',['float32','shape','subclass'])
def test_ineligible_inputs_do_not_use_opencv_range(kind,monkeypatch):
    raw=np.zeros((11,11),np.uint8)
    if kind=='float32':raw=raw.astype(np.float32)
    if kind=='shape':raw=raw[:10]
    if kind=='subclass':
        class Other(np.ndarray):pass
        raw=raw.view(Other)
    monkeypatch.setattr(cv2,'minMaxLoc',lambda *a,**k:pytest.fail('ineligible'))
    assert sufficient_range_contrast(raw) is None
    assert fast._contrast_gate(raw,raw.astype(np.float32)) is False


def test_existing_original_std_band_is_retained(monkeypatch):
    raw=np.zeros((11,11),np.uint8);raw[0,0]=22;calls=[]
    class Observed(np.ndarray):
        def std(self,*args,**kwargs):calls.append(1);return super().std(*args,**kwargs)
    result=fast._contrast_gate(raw,raw.astype(np.float32).view(Observed))
    assert result==(float(raw.astype(np.float32).std())>=2.) and calls==[1]
