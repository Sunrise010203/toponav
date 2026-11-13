"""Call-local preparation equivalence; not straight-task safety acceptance."""
import copy

import cv2
import numpy as np
import pytest

from roadvision import free_ground_path as ground
from roadvision import reference_patch_evidence as reference
from roadvision import rgb_resampling_checks as original
from roadvision.calibrated_ground_motion import GroundCalibration
from roadvision.prepared_descriptor_search import PreparedAlternative
from tests.test_reference_patch_equivalence import image_for, oracle


def prepared(image, mask):
    context = PreparedAlternative(image, patch_px=11, half=5, exclude_radius=11., mask=mask)
    return lambda patch, point: context.at(point)(patch)


@pytest.mark.parametrize('kind', ['random', 'blur', 'flat', 'repeated', 'low_contrast', 'float'])
@pytest.mark.parametrize('point', [[64., 64.], [62.125, 81.875], [6., 6.], [121.999, 64.], [5.999, 64.], [np.nan, 64.]])
def test_prepared_reference_full_evidence_exact(kind, point):
    image = image_for(kind); mask = np.ones((128, 128), np.uint8)
    assert reference.reference_appearance_evidence(image, point, mask, _search=prepared(image, mask)) == oracle(image, point, mask)


@pytest.mark.parametrize('mask_kind', ['all', 'none', 'padding', 'one_footprint', 'disconnected'])
def test_prepared_support_populations_and_invalid_search_exact(mask_kind):
    image = image_for('random'); mask = np.ones_like(image)
    if mask_kind == 'none': mask[:] = 0
    elif mask_kind == 'padding': mask[:, :61] = 0
    elif mask_kind == 'one_footprint': mask[:] = 0; mask[59:70, 59:70] = 1
    elif mask_kind == 'disconnected': mask[:, 20:40] = 0; mask[90:, :] = 0
    for point in ([64., 64.], [44.125, 60.875]):
        assert reference.reference_appearance_evidence(image, point, mask, _search=prepared(image, mask)) == oracle(image, point, mask)


@pytest.mark.parametrize('value', [np.nan, np.inf, -.5, .94])
def test_nonfinite_full_score_maps_do_not_become_support(monkeypatch, value):
    image = image_for('random'); mask = np.ones_like(image)
    scores = np.full((118, 118), np.nan, np.float32); scores[0, 0] = value
    monkeypatch.setattr(cv2, 'matchTemplate', lambda *a, **kw: scores.copy())
    assert reference.reference_appearance_evidence(image, [64., 64.], mask, _search=prepared(image, mask)) == oracle(image, [64., 64.], mask)


@pytest.mark.parametrize('observed,alternative', [
    (None, .5), (.9, None), (.7, .64), (np.nextafter(.7, 0.), .5),
    (np.nextafter(.7, 1.), .5), (.9, .84), (.9, np.nextafter(.84, 0.)),
    (.9, np.nextafter(.84, 1.)), (1., .94), (1., np.nextafter(.94, 0.)),
    (1., np.nextafter(.94, 1.)), (.69, .95),
])
def test_prepared_hook_retains_measured_ncc_and_gate_neighbors(monkeypatch, observed, alternative):
    image = image_for('random'); mask = np.ones_like(image)
    for module in (original, reference):
        monkeypatch.setattr(module, '_ncc', lambda *a: observed)
    monkeypatch.setattr(original, '_masked_alternative', lambda *a: (alternative, 31))
    search = lambda patch, point: (alternative, 31)
    assert reference.reference_appearance_evidence(image, [64., 64.], mask, _search=search) == oracle(image, [64., 64.], mask)


@pytest.mark.parametrize('kind', ['random', 'blur', 'flat', 'repeated', 'float'])
@pytest.mark.parametrize('masked', [False, True])
def test_81_candidates_and_stable_selection_exact_without_mutation(monkeypatch, kind, masked):
    image = image_for(kind); mask = np.ones((128, 128), np.uint8)
    if masked: mask[:, 40:57] = 0; mask[92:, :] = 0
    saved_image, saved_mask = image.copy(), mask.copy()
    calibration = GroundCalibration()
    candidates = ground.reference_cell_candidates(image, mask, calibration)
    cells = ground.select_reference_cells(image, mask, calibration)
    monkeypatch.setattr(ground, 'reference_appearance_evidence', oracle)
    assert candidates == ground.reference_cell_candidates(image, mask, calibration)
    assert cells == ground.select_reference_cells(image, mask, calibration)
    np.testing.assert_array_equal(image, saved_image)
    np.testing.assert_array_equal(mask, saved_mask)


def test_one_preparation_but_still_81_full_template_matches(monkeypatch):
    image = image_for('random'); mask = np.ones_like(image)
    counts = dict(prepare=0, match=0, erode=0)
    cls, match, erode = ground.PreparedAlternative, cv2.matchTemplate, cv2.erode
    def prepare(*a, **kw): counts['prepare'] += 1; return cls(*a, **kw)
    def matched(*a, **kw): counts['match'] += 1; return match(*a, **kw)
    def eroded(*a, **kw): counts['erode'] += 1; return erode(*a, **kw)
    monkeypatch.setattr(ground, 'PreparedAlternative', prepare)
    monkeypatch.setattr(cv2, 'matchTemplate', matched)
    monkeypatch.setattr(cv2, 'erode', eroded)
    ground.reference_cell_candidates(image, mask, GroundCalibration())
    assert counts == dict(prepare=1, match=81, erode=1)


@pytest.mark.parametrize('kind', ['flat', 'padding'])
def test_unavailable_patches_do_not_even_prepare_search(monkeypatch, kind):
    image = image_for('flat' if kind == 'flat' else 'random')
    mask = np.ones_like(image) if kind == 'flat' else np.zeros_like(image)
    def forbidden(*a, **kw): pytest.fail('no supported patch must mean no prepared search')
    monkeypatch.setattr(ground, 'PreparedAlternative', forbidden)
    monkeypatch.setattr(cv2, 'matchTemplate', forbidden)
    result = ground.reference_cell_candidates(image, mask, GroundCalibration())
    assert all(not c['reference_accepted'] for group in result for c in group['candidates'])


def test_new_call_never_reuses_old_image_or_mask_context(monkeypatch):
    contexts = []; cls = ground.PreparedAlternative
    def prepare(*a, **kw):
        result = cls(*a, **kw); contexts.append(result); return result
    monkeypatch.setattr(ground, 'PreparedAlternative', prepare)
    image = image_for('random'); mask = np.ones_like(image)
    cal = GroundCalibration()
    first = ground.reference_cell_candidates(image, mask, cal)
    saved = copy.deepcopy(first)
    image[:] = image_for('random', 519); mask[:20, :] = 0
    second = ground.reference_cell_candidates(image, mask, cal)
    assert len(contexts) == 2 and contexts[0] is not contexts[1]
    assert first == saved and first != second
    monkeypatch.setattr(ground, 'reference_appearance_evidence', oracle)
    assert second == ground.reference_cell_candidates(image, mask, cal)


def test_prepared_search_failure_is_not_retried_or_converted_to_acceptance(monkeypatch):
    image = image_for('random'); mask = np.ones_like(image); calls = []
    class Broken:
        def __init__(self, *a, **kw): pass
        def at(self, point):
            calls.append(1)
            raise RuntimeError('search failed')
    monkeypatch.setattr(ground, 'PreparedAlternative', Broken)
    with pytest.raises(RuntimeError, match='search failed'):
        ground.reference_cell_candidates(image, mask, GroundCalibration())
    assert calls == [1]
