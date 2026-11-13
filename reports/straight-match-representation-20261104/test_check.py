"""Comparator, eligibility, baseline isolation and restoration; not performance."""
import hashlib
import importlib.util
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

spec = importlib.util.spec_from_file_location('_representation_check', Path(__file__).with_name('check.py'))
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


def test_every_uint8_value_roundtrips():
    a = np.arange(256,dtype=np.float32).reshape(16,16)
    b = check.to_uint8_exact(a)
    assert b.dtype == np.uint8 and b.astype(np.float32).tobytes() == a.tobytes()


@pytest.mark.parametrize('value', [0.25,-0.,-1.,256.,np.nan,np.inf,-np.inf])
def test_unsafe_conversion_ineligible(value):
    assert check.to_uint8_exact(np.array([value],np.float32)) is None


@pytest.mark.parametrize('dtype',[np.float64,np.uint8,np.int32])
def test_wrong_dtype_ineligible(dtype):
    assert check.to_uint8_exact(np.zeros((2,2),dtype)) is None


def test_noncontiguous_exact_conversion():
    a=np.arange(36,dtype=np.float32).reshape(6,6)[::2,::2]
    assert check.to_uint8_exact(a).astype(np.float32).tobytes() == a.tobytes()


def test_identical_maps():
    a=np.arange(12,dtype=np.float32).reshape(3,4)
    result=check.compare_maps(a,a.copy())
    assert result['bit_exact'] and result['different_elements'] == 0
    assert result['first_difference'] is None


def test_one_ulp_is_rejected_without_tolerance():
    a=np.array([[.25]],np.float32)
    b=np.nextafter(a,np.float32(1.))
    result=check.compare_maps(a,b)
    assert not result['bit_exact'] and result['different_elements'] == 1
    assert result['maximum_finite_absolute_difference'] > 0


def test_signed_zero_is_rejected():
    result=check.compare_maps(np.array([[0.]],np.float32),np.array([[-0.]],np.float32))
    assert not result['bit_exact'] and result['maximum_finite_absolute_difference'] == 0.


def test_nan_payload_is_not_hidden():
    a=np.array([0x7fc00001],np.uint32).view(np.float32)
    b=np.array([0x7fc00002],np.uint32).view(np.float32)
    result=check.compare_maps(a,b)
    assert not result['bit_exact'] and result['baseline_nonfinite'] == 1
    assert result['maximum_finite_absolute_difference'] is None
    json.dumps(result,allow_nan=False)


@pytest.mark.parametrize('other',[np.zeros((2,3),np.float32),np.zeros((2,2),np.float64)])
def test_map_contract_fails_closed(other):
    with pytest.raises(ValueError,match='map_contract'):
        check.compare_maps(np.zeros((2,2),np.float32),other)


def test_alternate_never_returned_and_one_witness_per_group(tmp_path):
    baseline=np.zeros((2,2),np.float32);alternate=np.ones((2,2),np.float32)
    seen=[]
    def matcher(target,template,method):
        seen.append(target.dtype)
        return baseline if target.dtype == np.float32 else alternate
    observer=check.Comparison(tmp_path)
    for _ in range(2):
        result=observer.compare(matcher,'group',np.zeros((12,12),np.float32),np.zeros((11,11),np.float32),5)
        assert result is baseline
    assert seen == [np.float32,np.uint8,np.float32,np.uint8]
    assert len(list(tmp_path.glob('*.npz'))) == 1
    assert len(observer.records) == 2
    witness=np.load(tmp_path/observer.records[0]['witness'],allow_pickle=False)
    assert witness['baseline'].tobytes() == baseline.tobytes()
    assert witness['alternate'].tobytes() == alternate.tobytes()


def test_ineligible_calls_only_baseline(tmp_path):
    calls=[];baseline=np.zeros((2,2),np.float32)
    def matcher(*args):calls.append(args);return baseline
    observer=check.Comparison(tmp_path)
    assert observer.compare(matcher,'group',np.full((12,12),.5,np.float32),np.zeros((11,11),np.float32),5) is baseline
    assert len(calls)==1 and not observer.records[0]['eligible']


@pytest.mark.parametrize('raises',[False,True])
def test_attach_restores_on_exit_and_exception(tmp_path,monkeypatch,raises):
    def matcher(*args):return np.zeros((2,2),np.float32)
    monkeypatch.setattr(cv2,'matchTemplate',matcher)
    observer=check.Comparison(tmp_path)
    try:
        with observer.attach('group'):
            assert cv2.matchTemplate is not matcher
            cv2.matchTemplate(np.zeros((12,12),np.float32),np.zeros((11,11),np.float32),5)
            if raises:raise RuntimeError('expected')
    except RuntimeError:
        assert raises
    assert cv2.matchTemplate is matcher


def test_freeze_detects_changed_bytes(tmp_path):
    source=tmp_path/'input';source.write_bytes(b'original')
    pin=tmp_path/'freeze.json';pin.write_text(json.dumps(dict(files={str(source):hashlib.sha256(source.read_bytes()).hexdigest()})))
    check.verify_pins(pin)
    source.write_bytes(b'changed')
    with pytest.raises(AssertionError,match='frozen_file_changed'):check.verify_pins(pin)
