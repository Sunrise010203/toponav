"""Verify the real baseline dispatch and every new max, outside all timers."""
from contextlib import contextmanager
import hashlib
import struct
from pathlib import Path

import cv2
import numpy as np

from roadvision import paired_descriptor_checks as candidate
from roadvision.prepared_descriptor_checks import filter_h, filter_i
from tools.straight_paired_population_benchmark import encode, load_before, local_cases, write

HERE = Path(__file__).resolve().parent
before = load_before(HERE/'before/roadvision/paired_descriptor_checks.py')
assert before.h is candidate.h and before.i is candidate.i
assert before.filter_h is candidate.filter_h is filter_h
assert before.filter_i is candidate.filter_i is filter_i


@contextmanager
def counts():
    match, maximum = cv2.matchTemplate, candidate.finite_population_max
    counter = dict(actual_match_template=0, checked_maxima=0, checked_nonempty_maxima=0,
                   checked_finite_population_positions=0, zero_original_fallbacks=0)
    def matched(*args, **kw):
        counter['actual_match_template'] += 1
        return match(*args, **kw)
    def checked(scores, valid, count):
        assert count == int(np.count_nonzero(valid))
        assert np.isfinite(scores[valid]).all()
        original = float(scores[valid].max()) if count else None
        value = maximum(scores, valid, count)
        if original is None:
            assert value is None
        else:
            assert struct.pack('>d', original) == struct.pack('>d', value)
            counter['checked_nonempty_maxima'] += 1
            counter['zero_original_fallbacks'] += int(original == 0.)
        counter['checked_maxima'] += 1
        counter['checked_finite_population_positions'] += count
        return value
    try:
        cv2.matchTemplate, candidate.finite_population_max = matched, checked
        yield counter
    finally:
        cv2.matchTemplate, candidate.finite_population_max = match, maximum


records = []
for name, images, region in local_cases():
    hashes = [hashlib.sha256(im.tobytes()).hexdigest() for im in images]
    expected = encode(before.filter_pair(images, region))
    item = dict(case=name, variants={})
    for label, module, kwargs in [
        ('before_implicit', before, {}), ('before_explicit', before, dict(filters=(filter_h, filter_i))),
        ('after_implicit', candidate, {}), ('after_explicit', candidate, dict(filters=(filter_h, filter_i))),
    ]:
        with counts() as counter:
            assert encode(module.filter_pair(images, region, **kwargs)) == expected
        item['variants'][label] = counter
    v = item['variants']
    assert v['before_implicit'] == v['before_explicit']
    assert v['after_implicit'] == v['after_explicit']
    assert v['before_explicit']['actual_match_template'] == v['after_explicit']['actual_match_template']
    assert v['before_explicit']['checked_maxima'] == 0
    if name == 'random':
        assert v['after_explicit']['actual_match_template'] == 100
        assert v['after_explicit']['checked_maxima'] == 100
    if name == 'rectified':
        assert v['after_explicit']['checked_maxima'] == 0
    assert hashes == [hashlib.sha256(im.tobytes()).hexdigest() for im in images]
    records.append(item)
write(HERE/'PREFLIGHT.json', dict(records=records, all_full_outputs_exact=True,
    all_checked_maxima_bit_exact=True, finite_masks_and_counts_verified=True,
    raw_template_matches_unchanged=True, input_arrays_unchanged=True,
    explicit_filters_keep_native_pair_path=True, h_i_modules_shared_unchanged=True))
print('preflight exact all 6; raw template counts unchanged; finite maxima bit exact', flush=True)
