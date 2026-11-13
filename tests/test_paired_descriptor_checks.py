"""Two independent decisions; only exact per-track raw score maps shared."""
import copy
import json

import cv2
import numpy as np
import pytest

from roadvision import paired_descriptor_checks as pair
from roadvision import rgb_motion_checks as h, rgb_resampling_checks as i
from roadvision.prepared_descriptor_checks import filter_h, filter_i


def inputs(kind='random', shift=0., rectified=False, count=25):
    rng = np.random.default_rng(817)
    gray = rng.integers(0, 256, (128, 128), dtype=np.uint8)
    if kind == 'flat':
        gray.fill(70)
    elif kind == 'repeated':
        gray = np.tile(gray[:8, :8], (16, 16))
    elif kind == 'blur':
        gray = cv2.GaussianBlur(gray, (7, 7), 1.2)
    first = np.repeat(gray[:, :, None], 3, axis=2)
    images = [cv2.warpAffine(first, np.array([[1., 0., s], [0., 1., 0.]]), (128, 128))
              for s in (0., shift / 2, shift)]
    tracks = [dict(first=[x + .125, y + .375], middle=[x + .125 + shift / 2, y + .375],
                   last=[x + .125 + shift, y + .375])
              for y in (15., 35., 60., 85., 110.) for x in (15., 35., 60., 85., 110.)][:count]
    region = dict(view='front', rectified_ground_assumption=rectified, tracks=tracks,
                  coverage_map=[[1., 0., 0.], [0., 1.7, -13.], [0., 0., 1.]])
    return images, region


def independent(images, region):
    return dict(H=filter_h(images, region), I=filter_i(images, region))


@pytest.mark.parametrize('rectified', [False, True])
@pytest.mark.parametrize('kind', ['random', 'flat', 'repeated', 'blur'])
@pytest.mark.parametrize('shift', [0., 2.25])
def test_complete_regions_models_and_inputs_exact(kind, shift, rectified):
    images, region = inputs(kind, shift, rectified)
    original_region, original_images = copy.deepcopy(region), [a.copy() for a in images]
    want = independent(images, region)
    for _ in range(2):
        got = pair.filter_pair(images, region)
        assert got == want
        json.dumps(got, allow_nan=False)  # no raw score maps/arrays escape
        assert got['H']['models'] is not got['I']['models']
        assert got['H']['tracks'] is not got['I']['tracks']
        assert got['H']['descriptor_evidence'] is not got['I']['descriptor_evidence']
    assert region == original_region
    for a, b in zip(images, original_images):
        np.testing.assert_array_equal(a, b)


@pytest.mark.parametrize('point', [[6., 6.], [121.999, 64.], [122., 64.], [0., 64.],
                                  [64.125, 60.875], [float('inf'), 4.], [float('nan'), 4.]])
def test_boundary_and_nonfinite_endpoints_exact(point):
    images, region = inputs(count=1)
    region['tracks'][0] = dict(first=point, middle=point, last=point)
    if np.isfinite(point).all():
        assert pair.filter_pair(images, region) == independent(images, region)
    else:
        # H's original malformed-region failure is preserved, not repaired here.
        with pytest.raises(TypeError) as original:
            independent(images, region)
        with pytest.raises(TypeError) as candidate:
            pair.filter_pair(images, region)
        assert str(candidate.value) == str(original.value)


def test_empty_region_exact():
    images, region = inputs(count=0)
    assert pair.filter_pair(images, region) == independent(images, region)


@pytest.mark.parametrize('count', [1, 2, 16])
def test_four_actual_matches_eight_logical_checks_per_equal_pair(count, monkeypatch):
    images, region = inputs(count=count)
    actual, logical = [], []
    original_match, original_search = cv2.matchTemplate, pair._TrackScores.match
    def match(*args, **kw):
        actual.append(1)
        return original_match(*args, **kw)
    def search(self, role, template):
        logical.append(role)
        return original_search(self, role, template)
    monkeypatch.setattr(cv2, 'matchTemplate', match)
    monkeypatch.setattr(pair._TrackScores, 'match', search)
    pair.filter_pair(images, region)
    assert len(actual) == 4 * count  # stationary identical templates do NOT merge roles
    assert len(logical) == 8 * count
    assert logical == [0, 0, 1, 1, 0, 0, 1, 1] * count


@pytest.mark.parametrize('fn', [filter_h, filter_i])
def test_individual_entrypoints_still_do_four_full_matches(fn, monkeypatch):
    images, region = inputs(count=1)
    original, calls = cv2.matchTemplate, []
    def match(*a, **kw):
        calls.append(1)
        return original(*a, **kw)
    monkeypatch.setattr(cv2, 'matchTemplate', match)
    fn(images, region)
    assert len(calls) == 4


def test_separate_fits_canvases_and_templates_not_assumed_equal(monkeypatch):
    images, region = inputs(count=16)
    calls = []
    def fit_h(a, b):
        calls.append('H')
        return np.eye(3), np.ones(len(a), bool)
    def fit_i(a, b):
        calls.append('I')
        return np.array([[1., 0., 1.25], [0., 1., .75], [0., 0., 1.]]), np.ones(len(a), bool)
    monkeypatch.setattr(h, '_homography', fit_h)
    monkeypatch.setattr(i, '_homography', fit_i)
    want = independent(images, region)
    calls.clear()
    got = pair.filter_pair(images, region)
    assert got == want and calls == ['H', 'I']
    assert got['H']['descriptor_alignment']['homography'] != got['I']['descriptor_alignment']['homography']


def test_canvas_mismatch_gets_private_providers(monkeypatch):
    images, region = inputs(count=1)
    original_prepare = i.prepare_canvases
    def prepare(*args):
        gray, masks = original_prepare(*args)
        return [255 - g for g in gray], masks
    monkeypatch.setattr(i, 'prepare_canvases', prepare)
    want = independent(images, region)
    original, calls = cv2.matchTemplate, []
    def match(*a, **kw):
        calls.append(1)
        return original(*a, **kw)
    monkeypatch.setattr(cv2, 'matchTemplate', match)
    assert pair.filter_pair(images, region) == want
    assert len(calls) == 8


def test_masks_counts_and_padding_rejections_are_not_shared(monkeypatch):
    images, region = inputs(count=2)
    original = i.prepare_canvases
    def prepare(*args):
        gray, masks = original(*args)
        masks[0][:, :25] = 0
        masks[1][:, 75:90] = 0
        return gray, masks
    monkeypatch.setattr(i, 'prepare_canvases', prepare)
    got = pair.filter_pair(images, region)
    assert got == independent(images, region)
    assert got['H']['descriptor_evidence'][0]['accepted']
    assert got['I']['descriptor_evidence'][0]['reason'] == 'patch_boundary_flat_or_padding'
    assert got['I']['descriptor_evidence'][1]['alternative_search_counts'][0] < 118 ** 2


@pytest.mark.parametrize('value', [np.nan, np.inf, -.5, .94])
def test_shared_raw_nonfinite_scores_still_use_private_populations(value, monkeypatch):
    images, region = inputs(count=1)
    scores = np.full((118, 118), np.nan, np.float32)
    scores[0, 0] = value
    monkeypatch.setattr(cv2, 'matchTemplate', lambda *a, **kw: scores.copy())
    assert pair.filter_pair(images, region) == independent(images, region)


@pytest.mark.parametrize('gate', ['minimum_ncc', 'minimum_margin', 'self_similarity_max'])
@pytest.mark.parametrize('direction', [-1, 0, 1])
def test_threshold_neighbors_preserve_separate_reasons(gate, direction, monkeypatch):
    images, region = inputs(count=1)
    threshold = h.CHECK_POLICY[gate]
    if gate == 'self_similarity_max':
        value = np.float32(threshold)
        if direction:
            value = np.nextafter(value, np.float32(np.inf if direction > 0 else -np.inf))
    else:
        value = threshold if not direction else np.nextafter(threshold, np.inf if direction > 0 else -np.inf)
    observed = value if gate in ('minimum_ncc', 'minimum_margin') else .9
    alternative = 0. if gate == 'minimum_margin' else .1
    self_score = value if gate == 'self_similarity_max' else .1
    monkeypatch.setattr(h, '_ncc', lambda *a: observed)
    monkeypatch.setattr(i, '_ncc', lambda *a: observed)
    # Distinct templates aren't needed: mock scores by logical role per target.
    counters = {}
    def match(target, template, method):
        key = id(target)
        role = counters.get(key, 0) % 2
        counters[key] = role + 1
        return np.full((118, 118), alternative if role == 0 else self_score, np.float32)
    monkeypatch.setattr(cv2, 'matchTemplate', match)
    counters.clear()
    got = pair.filter_pair(images, region)
    counters.clear()
    assert got == independent(images, region)


def test_exact_template_bytes_and_bounded_roles(monkeypatch):
    target = inputs()[0][0][:, :, 0]
    provider = pair._TrackScores(target)
    original, calls = cv2.matchTemplate, []
    def match(*a, **kw):
        calls.append(1)
        return original(*a, **kw)
    monkeypatch.setattr(cv2, 'matchTemplate', match)
    a = np.zeros((11, 11), np.float32)
    a[1, 1] = 1.
    b = a.copy(); b[0, 0] = -0.
    x = provider.match(0, a)
    assert provider.match(0, a.copy()) is x
    provider.match(0, b)  # signed zero is NOT a byte-identical template
    provider.match(1, b)  # distinct roles aren't merged
    provider.match(2, b); provider.match(2, b)
    assert len(calls) == 5 and len(provider.entries) == 2
    provider.clear()
    assert not provider.entries
    provider.match(0, b)
    assert len(calls) == 6


def test_maps_cleared_per_track_region_and_on_exception(monkeypatch):
    images, region = inputs(count=2)
    original, providers, clear_sizes = pair._TrackScores.__init__, [], []
    def init(self, *a, **kw):
        original(self, *a, **kw); providers.append(self)
    clear = pair._TrackScores.clear
    def cleared(self):
        clear_sizes.append(len(self.entries)); clear(self)
    monkeypatch.setattr(pair._TrackScores, '__init__', init)
    monkeypatch.setattr(pair._TrackScores, 'clear', cleared)
    for _ in range(2):
        pair.filter_pair(images, region)
        assert all(not p.entries for p in providers)
    assert len(providers) == 4 and max(clear_sizes) == 2
    monkeypatch.setattr(i, 'appearance_evidence', lambda *a, **kw: (_ for _ in ()).throw(RuntimeError('stop')))
    with pytest.raises(RuntimeError, match='stop'):
        pair.filter_pair(images, region)
    assert all(not p.entries for p in providers)


def test_alternate_filters_dispatch_before_requiring_region_schema():
    events = []
    def one(images, region):
        events.append('H'); return region
    def two(images, region):
        events.append('I'); return images
    assert pair.filter_pair([], {}, filters=(one, two)) == dict(H={}, I=[])
    assert events == ['H', 'I']


@pytest.mark.parametrize('policy_key,value', [('patch_px', 9), ('exclude_radius_px', 12.)])
def test_changed_policy_uses_original_separate_filters(policy_key, value, monkeypatch):
    images, region = inputs(count=1)
    monkeypatch.setitem(h.CHECK_POLICY, policy_key, value)
    monkeypatch.setattr(pair, '_prepare_h', lambda *a: pytest.fail('specialization must not run'))
    if policy_key == 'patch_px':
        # I's fixed 11px search cannot consume H's mutated 9px patch. Preserve
        # the existing error, and verify specialization wasn't used.
        with pytest.raises(ValueError) as original:
            independent(images, region)
        with pytest.raises(ValueError) as candidate:
            pair.filter_pair(images, region)
        assert str(candidate.value) == str(original.value)
    else:
        assert pair.filter_pair(images, region) == independent(images, region)


@pytest.mark.parametrize('point', [[64., 64.], [5., 5.], [121.999, 64.], [44.125, 60.875],
                                  [-1000., 64.], [np.inf, 64.], [np.nan, 64.], [1e100, -1e100]])
@pytest.mark.parametrize('masked', [False, True])
def test_private_exclusion_population_exact(point, masked):
    target = inputs()[0][0][:, :, 0]
    mask = np.ones_like(target); mask[:20] = 0; mask[:, 80:85] = 0
    provider = pair._TrackScores(target)
    search = pair._ChannelSearch(target, provider, mask=mask if masked else None)
    template = target[30:41, 40:51].astype(np.float32)
    expected = (i._masked_alternative(template, target, point, mask) if masked
                else h._alternative(template, target, point))
    assert search.at(point)(template) == expected


def test_same_raw_map_is_reduced_with_different_support_and_endpoint(monkeypatch):
    target = inputs()[0][0][:, :, 0]
    provider = pair._TrackScores(target)
    template = target[30:41, 40:51].astype(np.float32)
    scores = np.full((118, 118), np.nan, np.float32)
    scores[10, 10] = .9; scores[80, 80] = .7
    calls = []
    def match(*a, **kw):
        calls.append(1); return scores.copy()
    monkeypatch.setattr(cv2, 'matchTemplate', match)
    first_mask = np.ones_like(target); second_mask = first_mask.copy(); second_mask[:30] = 0
    a = pair._ChannelSearch(target, provider, mask=first_mask)
    b = pair._ChannelSearch(target, provider, mask=second_mask)
    assert a.at([64., 64.])(template) == (float(np.float32(.9)), 2)
    assert b.at([64., 64.])(template) == (float(np.float32(.7)), 1)
    assert a.at([15., 15.])(template) == (float(np.float32(.7)), 1)
    assert len(calls) == 1
