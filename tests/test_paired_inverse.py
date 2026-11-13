"""Archived REJECTED inverse experiment; default implementation is unchanged.

The frozen candidate remains independently testable, not wired into production.
The original measured test file is retained as candidate/tests/*.py.snapshot.
"""
import gc
from pathlib import Path
import weakref

import numpy as np
import pytest

from roadvision import paired_descriptor_checks as default_pair
from tools.straight_paired_population_benchmark import load_before
from test_paired_descriptor_checks import inputs, independent


ROOT = Path(__file__).resolve().parents[1]
pair = load_before(ROOT/'reports/straight-paired-inverse-20261104/candidate/roadvision/paired_descriptor_checks.py')


def test_default_does_not_use_archived_candidate_inverse(monkeypatch):
    images, region = inputs(count=2)
    expected = pair.filter_pair(images, region)
    prepare, invert = default_pair._prepare_h, np.linalg.inv
    aligned, calls = [], []
    def prepared(*args, **kw):
        result = prepare(*args, **kw); aligned.append(result[3]); return result
    def inverse(matrix):
        if any(matrix is m for m in aligned): calls.append(1)
        return invert(matrix)
    monkeypatch.setattr(default_pair, '_prepare_h', prepared)
    monkeypatch.setattr(np.linalg, 'inv', inverse)
    assert default_pair.filter_pair(images, region) == expected
    assert len(calls) == 2


def watch_inverse(monkeypatch, *, singular=False):
    prepare, invert = pair._prepare_h, np.linalg.inv
    alignments, calls, references = [], [], []
    def prepared(*args, **kw):
        values = list(prepare(*args, **kw))
        if singular:
            values[3] = np.zeros((3, 3))
        alignments.append(values[3])
        return tuple(values)
    def inverse(matrix):
        tracked = any(matrix is m for m in alignments)
        if tracked:
            calls.append(matrix.copy())
        value = invert(matrix)
        if tracked:
            references.append(weakref.ref(value))
        return value
    monkeypatch.setattr(pair, '_prepare_h', prepared)
    monkeypatch.setattr(np.linalg, 'inv', inverse)
    return alignments, calls, references


@pytest.mark.parametrize('count', [0, 1, 2, 16])
@pytest.mark.parametrize('shift', [0., 2.25])
def test_one_inverse_per_nonempty_region_exact_output_and_no_retention(count, shift, monkeypatch):
    images, region = inputs(count=count, shift=shift)
    expected = independent(images, region)
    alignments, calls, references = watch_inverse(monkeypatch)
    for repeat in range(2):
        assert pair.filter_pair(images, region) == expected
        assert len(calls) == (repeat+1 if count else 0)
    assert len(alignments) == 2 and alignments[0] is not alignments[1]
    gc.collect()
    assert all(ref() is None for ref in references)


@pytest.mark.parametrize('count', [0, 1, 16])
def test_singular_inverse_only_raises_at_original_first_use(count, monkeypatch):
    images, region = inputs(count=count)
    _, calls, _ = watch_inverse(monkeypatch, singular=True)
    evidence_calls = []
    monkeypatch.setattr(pair.h, 'patch_evidence', lambda *a, **kw: evidence_calls.append(1))
    if count:
        with pytest.raises(np.linalg.LinAlgError, match='Singular matrix'):
            pair.filter_pair(images, region)
        assert len(calls) == 1
    else:
        value = pair.filter_pair(images, region)
        assert value['H']['tracks'] == value['I']['tracks'] == []
        assert calls == []
    assert evidence_calls == []


def test_projection_exception_precedes_first_inversion_and_finally_cleans(monkeypatch):
    images, region = inputs(count=2)
    _, calls, _ = watch_inverse(monkeypatch)
    original, providers = pair._TrackScores.__init__, []
    def initialize(self, *a, **kw):
        original(self, *a, **kw)
        providers.append(self)
    monkeypatch.setattr(pair._TrackScores, '__init__', initialize)
    error = RuntimeError('first projection')
    def fail(*a, **kw): raise error
    monkeypatch.setattr(pair.h, '_project', fail)
    with pytest.raises(RuntimeError) as actual:
        pair.filter_pair(images, region)
    assert actual.value is error and calls == []
    assert providers and all(not p.entries and not p.populations for p in providers)


def test_later_evidence_error_still_cleans_and_does_not_reinvert(monkeypatch):
    images, region = inputs(count=2)
    _, calls, references = watch_inverse(monkeypatch)
    original, providers = pair._TrackScores.__init__, []
    def initialize(self, *a, **kw):
        original(self, *a, **kw); providers.append(self)
    monkeypatch.setattr(pair._TrackScores, '__init__', initialize)
    evidence, seen = pair.h.patch_evidence, []
    def second_fails(*a, **kw):
        seen.append(1)
        if len(seen) == 2:
            raise RuntimeError('second evidence')
        return evidence(*a, **kw)
    monkeypatch.setattr(pair.h, 'patch_evidence', second_fails)
    with pytest.raises(RuntimeError, match='second evidence'):
        pair.filter_pair(images, region)
    assert len(calls) == 1
    assert all(not p.entries and not p.populations for p in providers)


@pytest.mark.parametrize('custom', [False, True])
def test_fallback_does_not_enter_native_preparation(custom, monkeypatch):
    images, region = inputs(count=1, rectified=not custom)
    expected = independent(images, region) if not custom else dict(H='customH', I='customI')
    def fail(*a, **kw): raise AssertionError('native preparation on fallback')
    monkeypatch.setattr(pair, '_prepare_h', fail)
    kwargs = dict(filters=(lambda *a: 'customH', lambda *a: 'customI')) if custom else {}
    assert pair.filter_pair(images, region, **kwargs) == expected
