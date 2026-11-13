"""Check real frozen baseline dispatch and actual work, outside timers."""
from contextlib import contextmanager
import json
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
def count(module):
    original_prepare, original_inv, original_match = module._prepare_h, np.linalg.inv, cv2.matchTemplate
    targets = []
    counts = dict(aligned_inverses=0, actual_match_template=0)
    def prepare(*args, **kwargs):
        result = original_prepare(*args, **kwargs)
        targets.append(result[3])
        return result
    def inverse(matrix):
        counts['aligned_inverses'] += int(any(matrix is target for target in targets))
        return original_inv(matrix)
    def match(*args, **kwargs):
        counts['actual_match_template'] += 1
        return original_match(*args, **kwargs)
    try:
        module._prepare_h = prepare
        np.linalg.inv = inverse
        cv2.matchTemplate = match
        yield counts
    finally:
        module._prepare_h, np.linalg.inv, cv2.matchTemplate = original_prepare, original_inv, original_match


records = []
for index, n in [(0, 0), (0, 1), (0, 16), (4, 16), (5, 1)]:
    name, images, region = local_cases()[index]
    region['tracks'] = region['tracks'][:n]
    expected = encode(before.filter_pair(images, region))
    item = dict(case=name, tracks=n, variants={})
    for label, module, kwargs in [
        ('before_implicit', before, {}), ('before_explicit', before, dict(filters=(filter_h, filter_i))),
        ('after_implicit', candidate, {}), ('after_explicit', candidate, dict(filters=(filter_h, filter_i))),
    ]:
        with count(module) as counts:
            assert encode(module.filter_pair(images, region, **kwargs)) == expected
        item['variants'][label] = counts
    variants = item['variants']
    assert variants['before_implicit'] == variants['before_explicit']
    assert variants['after_implicit'] == variants['after_explicit']
    assert variants['before_explicit']['actual_match_template'] == variants['after_explicit']['actual_match_template']
    if index != 5:
        assert variants['before_explicit']['aligned_inverses'] == n
        assert variants['after_explicit']['aligned_inverses'] == int(n > 0)
    if index == 0 and n:
        assert variants['before_explicit']['actual_match_template'] == 4*n
    records.append(item)
for module in (before, candidate):
    assert module.filter_pair([], {}, filters=(lambda *a: 'customH', lambda *a: 'customI')) == dict(H='customH', I='customI')
write(HERE/'PREFLIGHT.json', dict(records=records, full_outputs_exact=True,
    unchanged_shared_h_i=True, explicit_filters_keep_native_pair_path=True,
    custom_filters_unchanged=True, no_clone_identity_remapping=True))
print('preflight exact; native actual match counts unchanged, regional inverses n -> 1 (empty stays 0)', flush=True)
