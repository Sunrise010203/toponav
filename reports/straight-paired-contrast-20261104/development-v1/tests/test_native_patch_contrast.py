"""Integer contrast shortcuts must preserve original patches and evidence."""
import ast
import gc
import inspect
import weakref

import cv2
import numpy as np
import pytest

from roadvision import native_patch_contrast as fast
from roadvision import rgb_motion_checks as h, rgb_resampling_checks as i


def gate(raw):
    patch = raw.astype(np.float32)
    assert fast._contrast_gate(raw, patch) == (float(patch.std()) >= 2.)


@pytest.mark.parametrize('base', [0, 100, 247])
def test_all_two_level_multiplicities_near_threshold(base):
    for difference in range(9):
        for count in range(122):
            raw = np.full(121, base, np.uint8)
            raw[:count] += difference
            gate(raw.reshape(11, 11))


@pytest.mark.parametrize('layout', ['C', 'F', 'reverse', 'strided'])
def test_random_full_range_and_near_threshold_integer_patches(layout):
    rng = np.random.default_rng(110406)
    for limit in [4, 5, 7, 9, 256]:
        for _ in range(100):
            raw = rng.integers(0, limit, (11, 11), dtype=np.uint8)
            if layout == 'F': raw = np.asfortranarray(raw)
            elif layout == 'reverse': raw = raw[::-1, ::-1]
            elif layout == 'strided':
                storage = np.zeros((22, 22), np.uint8)
                storage[::2, ::2] = raw
                raw = storage[::2, ::2]
            before = raw.copy()
            gate(raw)
            np.testing.assert_array_equal(raw, before)


def test_opencv_integer_moments_are_exact_for_every_constant_level():
    for level in range(256):
        raw = np.full((11, 11), level, np.uint8)
        assert cv2.sumElems(raw)[0] == 121*level
        assert cv2.norm(raw, cv2.NORM_L2SQR) == 121*level*level
    rng = np.random.default_rng(406)
    for _ in range(500):
        raw = rng.integers(0, 256, (11, 11), dtype=np.uint8)
        exact = raw.astype(np.int64)
        assert cv2.sumElems(raw)[0] == int(exact.sum())
        assert cv2.norm(raw, cv2.NORM_L2SQR) == int((exact*exact).sum())


@pytest.mark.parametrize('outlier,uses_original', [(0, False), (11, False),
                                                   (12, True), (22, True),
                                                   (23, True), (33, True), (34, False)])
def test_conservative_band_uses_original_std(outlier, uses_original):
    calls = []
    class Observed(np.ndarray):
        def std(self, *args, **kwargs):
            calls.append(1)
            return super().std(*args, **kwargs)
    raw = np.zeros((11, 11), np.uint8); raw[0, 0] = outlier
    patch = raw.astype(np.float32).view(Observed)
    assert fast._contrast_gate(raw, patch) == (float(np.asarray(patch).std()) >= 2.)
    assert bool(calls) == uses_original


@pytest.mark.parametrize('kind', ['float', 'wrong_shape', 'subclass'])
def test_ineligible_raw_patch_uses_original_std(kind):
    raw = np.arange(121, dtype=np.uint8).reshape(11, 11)
    if kind == 'float': raw = raw.astype(np.float32)+.125
    elif kind == 'wrong_shape': raw = raw[:10]
    else:
        class Other(np.ndarray): pass
        raw = raw.view(Other)
    gate(raw)


def assert_patch_equal(a, b):
    if a is None:
        assert b is None
    else:
        assert b.dtype == a.dtype and b.shape == a.shape and b.tobytes() == a.tobytes()


@pytest.mark.parametrize('point', [[64., 64.], [64.125, 64.375], [6., 6.],
    [121., 121.], [122., 64.], [5.999, 64.], [np.nan, 64.], [np.inf, 64.],
    [64.], [], [[64., 64.]]])
def test_original_point_validation_and_fractional_extraction(point):
    gray = np.random.default_rng(24).integers(0, 256, (128, 128), dtype=np.uint8)
    before = gray.copy()
    assert_patch_equal(h._patch(gray, point), fast.prepared_patch(h._patch, gray, point))
    np.testing.assert_array_equal(gray, before)


@pytest.mark.parametrize('kind', ['float32', 'shape', 'subclass', 'policy', 'callback'])
def test_changed_callback_canvas_or_policy_falls_back(kind, monkeypatch):
    gray = np.random.default_rng(35).integers(0, 256, (128, 128), dtype=np.uint8)
    function = h._patch
    if kind == 'float32': gray = gray.astype(np.float32)
    elif kind == 'shape': gray = gray[:120]
    elif kind == 'subclass':
        class Other(np.ndarray): pass
        gray = gray.view(Other)
    elif kind == 'policy': monkeypatch.setitem(h.CHECK_POLICY, 'patch_px', 9)
    else:
        function = lambda *args: 'custom'
        assert fast.prepared_patch(function, gray, [64., 64.]) == 'custom'
        return
    monkeypatch.setattr(fast, '_contrast_gate', lambda *args: pytest.fail('ineligible fast gate'))
    assert_patch_equal(function(gray, [64., 64.]), fast.prepared_patch(function, gray, [64., 64.]))


@pytest.mark.parametrize('dtype', [np.uint8, np.float32])
@pytest.mark.parametrize('padding', [False, True])
def test_supported_patch_retains_original_mask_sampling(dtype, padding):
    gray = np.random.default_rng(49).integers(0, 256, (128, 128), dtype=np.uint8)
    mask = np.ones((128, 128), dtype=dtype)
    if padding: mask[:, :65] = 0
    for point in [[64., 64.], [64.125, 64.375], [70., 64.], [122., 64.]]:
        before = mask.copy()
        assert_patch_equal(i._supported_patch(gray, point, mask), fast.supported_patch(gray, point, mask))
        np.testing.assert_array_equal(mask, before)


def test_evidence_source_is_original_except_patch_reader_and_namespace():
    class Normalize(ast.NodeTransformer):
        def visit_Attribute(self, node):
            if isinstance(node.value, ast.Name) and node.value.id in ('h', 'i'):
                return ast.Name(id=node.attr, ctx=node.ctx)
            return self.generic_visit(node)
        def visit_Call(self, node):
            if isinstance(node.func, ast.Name) and node.func.id == 'prepared_patch':
                return ast.Call(func=ast.Name(id='_patch', ctx=ast.Load()), args=node.args[1:], keywords=node.keywords)
            return self.generic_visit(node)
        def visit_Name(self, node):
            if node.id == 'supported_patch': node.id = '_supported_patch'
            return node
    for original, specialized in [(h.patch_evidence, fast._native_h_evidence),
                                  (i.appearance_evidence, fast._native_i_evidence)]:
        before = ast.parse(inspect.getsource(original))
        after = Normalize().visit(ast.parse(inspect.getsource(specialized)))
        after.body[0].name = before.body[0].name
        assert ast.dump(before, include_attributes=False) == ast.dump(after, include_attributes=False)


@pytest.mark.parametrize('callback', ['patch', 'supported_patch', 'ncc', 'h_evidence', 'i_evidence'])
def test_altered_original_callbacks_are_honored(callback, monkeypatch):
    gray = np.random.default_rng(64).integers(0, 256, (128, 128), dtype=np.uint8)
    mask = np.ones_like(gray)
    point = [64.125, 64.375]
    if callback == 'patch':
        monkeypatch.setattr(h, '_patch', lambda *a: None)
        monkeypatch.setattr(i, '_patch', lambda *a: None)
    elif callback == 'supported_patch': monkeypatch.setattr(i, '_supported_patch', lambda *a: None)
    elif callback == 'ncc':
        monkeypatch.setattr(h, '_ncc', lambda *a: None)
        monkeypatch.setattr(i, '_ncc', lambda *a: None)
    elif callback == 'h_evidence': monkeypatch.setattr(h, 'patch_evidence', lambda *a, **kw: {'custom': 'H'})
    else: monkeypatch.setattr(i, 'appearance_evidence', lambda *a, **kw: {'custom': 'I'})
    assert fast.h_evidence(gray, gray, point, point) == h.patch_evidence(gray, gray, point, point)
    assert fast.i_evidence(gray, gray, point, point, mask, mask) == i.appearance_evidence(gray, gray, point, point, mask, mask)


def test_no_patch_or_input_references_retained_across_calls():
    gray = np.random.default_rng(82).integers(0, 256, (128, 128), dtype=np.uint8)
    patches = [fast.prepared_patch(h._patch, gray, [64., 64.]) for _ in range(2)]
    assert patches[0] is not patches[1]
    patches[0].fill(0)
    assert np.any(patches[1])
    refs = [weakref.ref(p) for p in [gray]+patches]
    del gray, patches
    gc.collect()
    assert all(ref() is None for ref in refs)
