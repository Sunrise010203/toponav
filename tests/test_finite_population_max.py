"""Compare Python-float bytes, including signed zero, to the original gather."""
import gc
import struct
import weakref

import numpy as np
import pytest

from roadvision.finite_population_max import finite_population_max


def compare(scores, outside):
    before_scores, before_outside = scores.copy(), outside.copy()
    valid = outside & np.isfinite(scores)
    before_valid = valid.copy()
    count = int(np.count_nonzero(valid))
    expected = float(scores[valid].max()) if count else None
    actual = finite_population_max(scores, valid, count)
    if expected is None:
        assert actual is None
    else:
        assert struct.pack('>d', actual) == struct.pack('>d', expected)
    np.testing.assert_array_equal(scores, before_scores)
    np.testing.assert_array_equal(outside, before_outside)
    np.testing.assert_array_equal(valid, before_valid)


@pytest.mark.parametrize('shape', [(1, 1), (2, 3), (11, 11), (118, 118)])
@pytest.mark.parametrize('layout', ['C', 'F', 'negative_stride', 'strided'])
def test_random_float32_bit_patterns_and_sparse_dense_support(shape, layout):
    rng = np.random.default_rng(110405)
    for probability in (0., .01, .5, .98, 1.):
        for _ in range(8):
            scores = rng.integers(0, 2**32, shape, dtype=np.uint32).view(np.float32)
            if layout == 'F':
                scores = np.asfortranarray(scores)
            elif layout == 'negative_stride':
                scores = scores[::-1, ::-1]
            elif layout == 'strided':
                storage = np.zeros((shape[0]*2, shape[1]*2), np.float32)
                storage[::2, ::2] = scores
                scores = storage[::2, ::2]
            compare(scores, rng.random(shape) < probability)


@pytest.mark.parametrize('zeros', [[0.], [-0.], [0., -0.], [-0., 0.],
                                  [-0., 0., -0.], [0., -0., 0.]])
def test_zero_sign_is_exact_original_reduction_order(zeros):
    for size in (1, 2, 7, 16, 33, 118):
        scores = np.resize(np.array(zeros, np.float32), (size, size))
        compare(scores, np.ones(scores.shape, bool))
        scores.flat[0] = -1.
        outside = np.ones(scores.shape, bool)
        outside.flat[-1] = False
        compare(scores, outside)
        compare(scores.T, outside.T)


@pytest.mark.parametrize('maximum', [np.float32(-1.), np.float32(1.),
    np.nextafter(np.float32(0), np.float32(1)),
    np.nextafter(np.float32(0), np.float32(-1)),
    np.nextafter(np.float32(1), np.float32(0)), np.finfo(np.float32).max,
    -np.finfo(np.float32).max])
def test_exact_finite_extrema_with_masked_nonfinite_competitors(maximum):
    scores = np.full((118, 118), np.nan, np.float32)
    scores[0, 0], scores[0, 1] = np.inf, -np.inf
    scores[50, 50] = scores[60, 60] = maximum
    outside = np.ones(scores.shape, bool)
    compare(scores, outside)
    outside[50, 50] = outside[60, 60] = False
    compare(scores, outside)


@pytest.mark.parametrize('dtype', ['float16', 'float64', 'longdouble', 'int32', '>f4'])
def test_other_dtypes_retain_original_gather(dtype):
    scores = np.array([[-1., 0., 1.], [1., -1., 0.]], dtype=dtype)
    compare(scores, np.array([[True, False, True], [False, True, False]]))


def test_array_subclass_is_not_sent_to_masked_reduction():
    class OriginalOnly(np.ndarray):
        def max(self, *args, **kwargs):
            assert 'where' not in kwargs and 'initial' not in kwargs
            return super().max(*args, **kwargs)
    scores = np.arange(100, dtype=np.float32).reshape(10, 10).view(OriginalOnly)
    compare(scores, np.ones(scores.shape, bool))


def test_empty_count_returns_none_without_touching_scores():
    class Forbidden:
        def __getattribute__(self, name):
            raise AssertionError(name)
    assert finite_population_max(Forbidden(), Forbidden(), 0) is None


def test_fallback_keeps_malformed_boolean_mask_exception_type():
    scores = np.ones((2, 3), np.float32)
    mask = np.ones((1, 3), bool)
    with pytest.raises(Exception) as error:
        scores[mask].max()
    with pytest.raises(type(error.value)):
        finite_population_max(scores, mask, 3)


def test_helper_retains_no_scores_mask_or_result_array():
    scores = np.ones((118, 118), np.float32)
    valid = np.ones(scores.shape, bool)
    refs = [weakref.ref(scores), weakref.ref(valid)]
    result = finite_population_max(scores, valid, scores.size)
    assert type(result) is float and result == 1.
    del scores, valid
    gc.collect()
    assert all(ref() is None for ref in refs)
