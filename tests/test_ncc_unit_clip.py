"""Exact NCC clip identity, always against the explicitly archived candidate."""
import importlib.util
from pathlib import Path
import struct
import warnings

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]/'reports/straight-ncc-unit-clip-20261104'


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


old = load(HERE/'before/roadvision/rgb_motion_checks.py', '_old_ncc_clip')
new = load(HERE/'candidate/roadvision/rgb_motion_checks.py', '_candidate_ncc_clip')


def observed(function, a, b):
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter('always')
        try:
            value = function(a, b)
            result = ('none',) if value is None else ('value', type(value), struct.pack('=d', value))
        except Exception as exc:
            result = ('exception', type(exc), str(exc))
    return result, [(type(item.message), str(item.message)) for item in caught]


def exact(a, b):
    before = a.tobytes(), b.tobytes()
    assert observed(old._ncc, a, b) == observed(new._ncc, a, b)
    assert (a.tobytes(), b.tobytes()) == before


@pytest.mark.parametrize('dtype', [np.uint8, np.int16, np.float16, np.float32, np.float64, np.longdouble])
@pytest.mark.parametrize('layout', ['C', 'F', 'reverse', 'strided'])
def test_random_original_dtypes_layouts_and_inputs_unchanged(dtype, layout):
    rng = np.random.default_rng(441105)
    for _ in range(15):
        a, b = [rng.integers(0, 256, (11, 11)).astype(dtype) for _ in range(2)]
        if layout == 'F': a, b = np.asfortranarray(a), np.asfortranarray(b)
        if layout == 'reverse': a, b = a[::-1, ::-1], b[::-1, ::-1]
        if layout == 'strided': a, b = a[::2, ::2], b[::2, ::2]
        exact(a, b)


@pytest.mark.parametrize('shape', [(11, 11), (7, 7, 3), (2,)])
@pytest.mark.parametrize('kind', ['random', 'flat', 'tiny_boundary', 'anti', 'orthogonal',
                                  'nonfinite', 'overflow', 'underflow', 'mixed_scale'])
def test_degenerate_nonfinite_extreme_and_clamp_boundary(shape, kind):
    rng = np.random.default_rng(33017)
    a, b = [rng.normal(size=shape) for _ in range(2)]
    if kind == 'flat': a[:], b[:] = 7., 3.
    if kind == 'tiny_boundary': a *= np.sqrt(1e-9/len(a.ravel())); b = a.copy()
    if kind == 'anti': b = -a
    if kind == 'orthogonal': b = np.roll(a.ravel(), 1).reshape(shape)
    if kind == 'nonfinite': a.ravel()[0] = np.nan; b.ravel()[-1] = np.inf
    if kind == 'overflow': a *= 1e300; b *= 1e300
    if kind == 'underflow': a *= 1e-300; b *= 1e-300
    if kind == 'mixed_scale': a *= 1e100; b *= 1e-100
    exact(a, b)


@pytest.mark.parametrize('bits', [
    0, 0x8000000000000000, 0x3ff0000000000000, 0xbff0000000000000,
    0x3fefffffffffffff, 0xbfefffffffffffff, 0x3ff0000000000001, 0xbff0000000000001,
    1, 0x8000000000000001, 0x7ff0000000000000, 0xfff0000000000000,
    0x7ff8000000000013, 0xfff8000000000013, 0x7ff0000000000013])
def test_forced_scalar_bits_original_fallback_and_clip_calls(bits, monkeypatch):
    score = np.array([bits], np.uint64).view(np.float64)[0]
    class Dot(np.ndarray):
        def __matmul__(self, other): return score
    a = np.array([0., 1.]).view(Dot)
    original_clip = np.clip
    calls = []
    def clip(*args, **kw): calls.append(1); return original_clip(*args, **kw)
    monkeypatch.setattr(np, 'clip', clip)
    monkeypatch.setattr(np.linalg, 'norm', lambda a: 1.)
    expected = observed(old._ncc, a, a); old_calls = len(calls); calls.clear()
    actual = observed(new._ncc, a, a); new_calls = len(calls)
    assert expected == actual
    assert old_calls == 1
    # Division quiets a signaling NaN identically before either clamp branch.
    interior = np.isfinite(score) and -1. <= score <= 1.
    assert new_calls == (0 if interior else 1)


@pytest.mark.parametrize('kind', ['interior', 'outside', 'nonfinite', 'tiny_denominator'])
def test_norm_operands_and_call_counts_not_cached_or_reduced(kind, monkeypatch):
    a = np.arange(121, dtype=np.float32).reshape(11, 11)
    b = a[::-1].copy()
    if kind == 'outside': b = a.copy()
    if kind == 'nonfinite': b[0, 0] = np.nan
    if kind == 'tiny_denominator': a[:], b[:] = 1., 2.
    original_norm = np.linalg.norm
    records = []
    def norm(x):
        records.append((x.dtype.str, x.shape, x.strides, x.tobytes()))
        return original_norm(x)
    monkeypatch.setattr(np.linalg, 'norm', norm)
    expected = observed(old._ncc, a, b); before = records[:]; records.clear()
    actual = observed(new._ncc, a, b)
    assert actual == expected and records == before and len(records) == 2


@pytest.mark.parametrize('score', [0.25, np.float32(.25), complex(.25, 0)])
def test_unusual_score_types_preserve_original_result_or_exception(score, monkeypatch):
    class Dot(np.ndarray):
        def __matmul__(self, other): return score
    a = np.array([0., 1.]).view(Dot)
    monkeypatch.setattr(np.linalg, 'norm', lambda a: 1)
    exact(a, a)


@pytest.mark.parametrize('kind', ['empty', 'length_mismatch', 'complex', 'read_only', 'big_endian', 'object'])
def test_non_native_arguments_warnings_results_exceptions(kind):
    a, b = np.arange(10.), np.arange(10.)[::-1]
    if kind == 'empty': a, b = np.array([]), np.array([])
    if kind == 'length_mismatch': b = b[:-1]
    if kind == 'complex': a = a.astype(complex)+1j
    if kind == 'read_only': a.setflags(write=False); b.setflags(write=False)
    if kind == 'big_endian': a, b = a.astype('>f8'), b.astype('>f8')
    if kind == 'object': a, b = a.astype(object), b.astype(object)
    exact(a, b)
