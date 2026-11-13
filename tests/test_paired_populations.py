"""Exact-population sharing does not share H/I acceptance or stale maps."""
import gc
import weakref

import cv2
import numpy as np
import pytest

from roadvision import paired_descriptor_checks as pair


def make(mask=None):
    target = np.arange(128 * 128, dtype=np.uint8).reshape(128, 128)
    provider = pair._TrackScores(target)
    a = pair._ChannelSearch(target, provider)
    b = pair._ChannelSearch(target, provider, mask=np.ones_like(target) if mask is None else mask)
    template = target[30:41, 40:51].astype(np.float32)
    return provider, a, b, template


@pytest.mark.parametrize('value', [np.nan, np.inf, -np.inf, -.5, .94, -0.])
def test_exact_support_endpoint_map_reuses_finite_reduction(value, monkeypatch):
    provider, a, b, template = make()
    scores = np.full((118, 118), np.nan, np.float32)
    scores[0, 0] = value
    monkeypatch.setattr(cv2, 'matchTemplate', lambda *args: scores.copy())
    original, calls = np.isfinite, []
    def finite(array):
        calls.append(1)
        return original(array)
    monkeypatch.setattr(np, 'isfinite', finite)
    point = np.array([64.125, 60.875])
    h = a.at(point)(template)
    i = b.at(point.copy())(template.copy())
    assert len(provider.populations) == 1 and len(calls) == 1
    assert i[0] == h
    assert i[1] == int(np.isfinite(value))
    if i[1]:
        assert np.signbit(i[0]) == np.signbit(value)


def test_masks_with_same_eroded_support_can_share_but_not_padding(monkeypatch):
    target = np.zeros((128, 128), np.uint8)
    provider = pair._TrackScores(target)
    mask = np.ones_like(target); mask[:20] = 0
    a = pair._ChannelSearch(target, provider, mask=mask)
    b = pair._ChannelSearch(target, provider, mask=mask.copy())
    different = mask.copy(); different[:, :20] = 0
    c = pair._ChannelSearch(target, provider, mask=different)
    assert a.support_key == b.support_key != c.support_key
    assert a.population_support is b.population_support
    monkeypatch.setattr(cv2, 'matchTemplate', lambda *args: np.ones((118, 118), np.float32))
    template = np.ones((11, 11), np.float32)
    result = a.at([64., 64.])(template)
    assert result == b.at([64., 64.])(template)
    assert c.at([64., 64.])(template)[1] < result[1]


@pytest.mark.parametrize('change', ['endpoint', 'dtype', 'support', 'template', 'role'])
def test_no_reduction_hit_when_population_or_score_changes(change, monkeypatch):
    mask = np.ones((128, 128), np.uint8)
    if change == 'support': mask[:30] = 0
    provider, a, b, template = make(mask)
    counter = []
    def match(*args):
        counter.append(1)
        scores = np.zeros((118, 118), np.float32)
        scores[5, 5] = .9
        scores[90, 90] = .7
        return scores
    monkeypatch.setattr(cv2, 'matchTemplate', match)
    original, finite_calls = np.isfinite, []
    def finite(scores):
        finite_calls.append(1); return original(scores)
    monkeypatch.setattr(np, 'isfinite', finite)
    point = np.array([64., 64.])
    search = a.at(point)
    assert search(template) == float(np.float32(.9))
    if change == 'endpoint': point = np.array([10., 10.])
    if change == 'dtype': point = point.astype(np.float32)
    if change == 'template': template = template.copy(); template[0, 0] += 1.
    if change == 'role':
        search(template)
    else:
        b.at(point)(template)
    assert len(finite_calls) == 2
    assert len(counter) == (2 if change in ('template', 'role') else 1)


def test_clear_releases_populations_and_saved_closure_never_uses_stale_map(monkeypatch):
    provider, a, b, template = make()
    calls = []
    def match(*args):
        calls.append(1)
        return np.full((118, 118), .1 * len(calls), np.float32)
    monkeypatch.setattr(cv2, 'matchTemplate', match)
    search = a.at([64., 64.])
    assert search(template) == float(np.float32(.1))
    scores_ref = weakref.ref(provider.entries[0][1])
    provider.clear(); gc.collect()
    assert not provider.entries and not provider.populations and scores_ref() is None
    assert b.at([64., 64.])(template) == (float(np.float32(.2)), 13547)
    assert search(template) == float(np.float32(.3))  # saved closure's next role


def test_replaced_template_maps_are_not_retained_by_populations(monkeypatch):
    provider, a, b, template = make()
    monkeypatch.setattr(cv2, 'matchTemplate', lambda *args: np.ones((118, 118), np.float32))
    a.at([64., 64.])(template)
    ref = weakref.ref(provider.entries[0][1])
    changed = template.copy(); changed[0, 0] += 1.
    b.at([64., 64.])(changed)
    gc.collect()
    assert ref() is None


def test_many_endpoints_and_saved_closures_keep_bounded_provider_state(monkeypatch):
    provider, a, b, template = make()
    monkeypatch.setattr(cv2, 'matchTemplate', lambda *args: np.ones((118, 118), np.float32))
    saved = a.at([64., 64.])
    for n in range(10):
        assert b.at([float(n), float(n)])(template)[0] == 1.
        assert len(provider.populations) <= 2
        assert len(provider.entries) <= 2
    assert saved(template) == 1.


def test_populations_cleared_even_when_channel_evidence_raises(monkeypatch):
    from roadvision import rgb_resampling_checks as i
    rng = np.random.default_rng(711)
    image = rng.integers(0, 256, (128, 128, 3), dtype=np.uint8)
    point = [64., 64.]
    region = dict(view='front', rectified_ground_assumption=False,
                  tracks=[dict(first=point, middle=point, last=point)])
    original, providers = pair._TrackScores.__init__, []
    def init(self, *args):
        original(self, *args); providers.append(self)
    monkeypatch.setattr(pair._TrackScores, '__init__', init)
    def fail(*args, **kwargs):
        assert any(p.populations for p in providers)
        raise RuntimeError('stop')
    monkeypatch.setattr(i, 'appearance_evidence', fail)
    with pytest.raises(RuntimeError, match='stop'):
        pair.filter_pair([image] * 3, region)
    assert providers and all(not p.entries and not p.populations for p in providers)


def test_changed_template_invalidates_reduction_even_if_match_reuses_array(monkeypatch):
    provider, a, b, template = make()
    scores = np.zeros((118, 118), np.float32)
    calls = []
    def match(*args):
        calls.append(1); scores.fill(.1 * len(calls)); return scores
    monkeypatch.setattr(cv2, 'matchTemplate', match)
    assert a.at([64., 64.])(template) == float(np.float32(.1))
    changed = template.copy(); changed[0, 0] += 1.
    assert b.at([64., 64.])(changed) == (float(np.float32(.2)), 13547)
