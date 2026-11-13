"""Before timing: full local output/work equality and all-project exactness."""
from contextlib import contextmanager
import hashlib
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(HERE))
from baseline import ProjectionBaseline, ORIGINAL_REPLACE, CURRENT_PAIR
from roadvision import paired_descriptor_checks as pair
from roadvision.prepared_descriptor_checks import filter_h, filter_i
from tools.straight_paired_population_benchmark import encode, local_cases, write


def digest(array):
    return (array.dtype.str, array.shape, array.strides,
            hashlib.sha256(array.tobytes()).hexdigest())


@contextmanager
def observed(baseline, old):
    original_match, original_sample = cv2.matchTemplate, cv2.getRectSubPix
    hncc, incc = pair.h._ncc, pair.i._ncc
    stats = dict(projections=0, eligible_projections=0, projection_inputs=[], projection_outputs=[],
                 matches=0, support_samples=0, support_sha256=[], h_ncc=0, i_ncc=0)
    function = baseline.old if old else baseline.new
    def projected(matrix, points):
        stats['projections'] += 1
        if (type(points) is np.ndarray and points.dtype == np.float64 and points.ndim == 2
                and points.shape[1] == 2 and points.flags.c_contiguous):
            stats['eligible_projections'] += 1
        stats['projection_inputs'].append((digest(matrix), digest(points)))
        value = function(matrix, points)
        reference = baseline.old(matrix, points)
        assert (value is None) == (reference is None)
        if value is not None:
            assert digest(value) == digest(reference)
        stats['projection_outputs'].append(None if value is None else digest(value))
        return value
    def matched(*a, **kw):
        stats['matches'] += 1
        return original_match(*a, **kw)
    def sampled(image, *a, **kw):
        value = original_sample(image, *a, **kw)
        if image.dtype == np.float32 and image.shape == (128, 128):
            stats['support_samples'] += 1
            stats['support_sha256'].append(hashlib.sha256(value.tobytes()).hexdigest())
        return value
    def h_call(*a): stats['h_ncc'] += 1; return hncc(*a)
    def i_call(*a): stats['i_ncc'] += 1; return incc(*a)
    try:
        cv2.matchTemplate, cv2.getRectSubPix = matched, sampled
        pair.h._ncc, pair.i._ncc = h_call, i_call
        with ORIGINAL_REPLACE(baseline.bindings, projected):
            yield stats
    finally:
        cv2.matchTemplate, cv2.getRectSubPix = original_match, original_sample
        pair.h._ncc, pair.i._ncc = hncc, incc


if __name__ == '__main__':
    baseline = ProjectionBaseline(HERE/'before/roadvision/corridor_rgb_probe.py')
    results = []
    for name, images, region in local_cases():
        images_before = [digest(im) for im in images]
        region_before = encode(region)
        expected = encode(baseline.local_before(images, region))
        variants = {}
        for label, old, kw in (
                ('before_implicit', True, {}), ('before_explicit', True, dict(filters=(filter_h, filter_i))),
                ('after_implicit', False, {}), ('after_explicit', False, dict(filters=(filter_h, filter_i)))):
            with observed(baseline, old) as counters:
                assert encode(CURRENT_PAIR(images, region, **kw)) == expected
            variants[label] = counters
        assert all(v == variants['before_implicit'] for v in variants.values())
        assert images_before == [digest(im) for im in images]
        assert region_before == encode(region)
        results.append(dict(case=name, variants=variants, all_outputs_exact=True, inputs_unchanged=True))
    assert all(vars(owner)[name] is baseline.new for owner, name, _ in baseline.bindings)
    write(HERE/'PREFLIGHT.json', dict(cases=results, baseline_binding=baseline.record(),
          complete_outputs_and_projection_bits_exact=True, logical_and_actual_work_unchanged=True,
          aliases_restored=True))
    print('All six local cases exact in four dispatches; projection bits, match/NCC/support unchanged')
