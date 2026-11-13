"""Real frozen baseline dispatch/counts and full-bitmaps, outside timers."""
from contextlib import contextmanager
import json
from pathlib import Path

import cv2
import numpy as np

from roadvision import paired_descriptor_checks as candidate
from roadvision.local_exclusion_population import LocalExclusion
from roadvision.prepared_descriptor_checks import filter_h, filter_i
from tools.straight_paired_population_benchmark import encode, load_before, local_cases, write

HERE = Path(__file__).resolve().parent
before = load_before(HERE/'before/roadvision/paired_descriptor_checks.py')
assert before.h is candidate.h and before.i is candidate.i
assert before.filter_h is candidate.filter_h is filter_h
assert before.filter_i is candidate.filter_i is filter_i


@contextmanager
def counts():
    match, outside, window = cv2.matchTemplate, LocalExclusion.outside, LocalExclusion._window
    counter = dict(actual_match_template=0, checked_full_bitmaps=0, full_distance_positions=0,
                   local_window_calls=0, local_distance_positions=0)
    def matched(*args, **kw):
        counter['actual_match_template'] += 1
        return match(*args, **kw)
    def bounded(self, *args):
        slices = window(self, *args)
        counter['local_window_calls'] += 1
        counter['local_distance_positions'] += self.x[slices].size
        return slices
    def checked(self, point, radius):
        value = outside(self, point, radius)
        original = (self.x-point[0])**2+(self.y-point[1])**2 > radius
        np.testing.assert_array_equal(value, original)
        counter['checked_full_bitmaps'] += 1
        counter['full_distance_positions'] += self.x.size
        return value
    try:
        cv2.matchTemplate, LocalExclusion.outside, LocalExclusion._window = matched, checked, bounded
        yield counter
    finally:
        cv2.matchTemplate, LocalExclusion.outside, LocalExclusion._window = match, outside, window


records = []
for index in range(6):
    name, images, region = local_cases()[index]
    expected = encode(before.filter_pair(images, region))
    item = dict(case=name, variants={})
    for label, module, kwargs in [
        ('before_implicit', before, {}), ('before_explicit', before, dict(filters=(filter_h, filter_i))),
        ('after_implicit', candidate, {}), ('after_explicit', candidate, dict(filters=(filter_h, filter_i))),
    ]:
        with counts() as counter:
            assert encode(module.filter_pair(images, region, **kwargs)) == expected
        item['variants'][label] = counter
    variants = item['variants']
    assert variants['before_implicit'] == variants['before_explicit']
    assert variants['after_implicit'] == variants['after_explicit']
    assert variants['before_explicit']['actual_match_template'] == variants['after_explicit']['actual_match_template']
    if index == 0:
        assert variants['after_explicit']['actual_match_template'] == 100  # 25 * 4 actual roles/canvases
        assert variants['after_explicit']['local_window_calls'] > 0
        assert variants['after_explicit']['local_distance_positions'] < variants['after_explicit']['full_distance_positions']
    records.append(item)
write(HERE/'PREFLIGHT.json', dict(records=records, all_full_outputs_exact=True,
    all_evaluated_full_exclusion_bitmaps_exact=True, raw_template_matches_unchanged=True,
    explicit_filters_keep_native_pair_path=True, h_i_modules_shared_unchanged=True))
print('preflight exact all 6 cases; full searches/counts unchanged, bitmap arithmetic locally bounded', flush=True)
