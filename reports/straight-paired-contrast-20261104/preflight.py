"""Real baseline dispatch, raw patch bytes, contrast gates and work counts."""
from contextlib import contextmanager
from pathlib import Path
import hashlib

import cv2
import numpy as np

from roadvision import paired_descriptor_checks as candidate, native_patch_contrast as fast
from roadvision.prepared_descriptor_checks import filter_h, filter_i
from tools.straight_paired_population_benchmark import encode, load_before, local_cases, write

HERE = Path(__file__).resolve().parent
before = load_before(HERE/'before/roadvision/paired_descriptor_checks.py')
assert before.h is candidate.h and before.i is candidate.i
assert before.filter_h is candidate.filter_h is filter_h
assert before.filter_i is candidate.filter_i is filter_i


@contextmanager
def counts():
    match, reader, gate = cv2.matchTemplate, fast.prepared_patch, fast._contrast_gate
    ncc_h, ncc_i = fast.h._ncc, fast.i._ncc
    counter = dict(actual_match_template=0, h_ncc=0, i_ncc=0,
                   checked_patch_requests=0, checked_contrast_gates=0,
                   far_accept=0, far_reject=0, original_std_band=0)
    def matched(*args, **kw):
        counter['actual_match_template'] += 1
        return match(*args, **kw)
    def hncc(*args):
        counter['h_ncc'] += 1
        return ncc_h(*args)
    def incc(*args):
        counter['i_ncc'] += 1
        return ncc_i(*args)
    def checked_reader(function, gray, point):
        value = reader(function, gray, point)
        expected = function(gray, point)
        if expected is None:
            assert value is None
        else:
            assert value.dtype == expected.dtype and value.shape == expected.shape
            assert value.tobytes() == expected.tobytes()
        counter['checked_patch_requests'] += 1
        return value
    def checked_gate(raw, patch):
        value = gate(raw, patch)
        assert value == (float(patch.std()) >= 2.)
        exact = raw.astype(np.int64)
        numerator = 121*int((exact*exact).sum())-int(exact.sum())**2
        counter['checked_contrast_gates'] += 1
        counter['far_reject' if numerator < 121**2 else
                'far_accept' if numerator > 9*121**2 else 'original_std_band'] += 1
        return value
    try:
        cv2.matchTemplate, fast.prepared_patch, fast._contrast_gate = matched, checked_reader, checked_gate
        fast.h._ncc, fast.i._ncc = hncc, incc
        yield counter
    finally:
        cv2.matchTemplate, fast.prepared_patch, fast._contrast_gate = match, reader, gate
        fast.h._ncc, fast.i._ncc = ncc_h, ncc_i


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
    assert v['before_implicit'] == v['before_explicit'] and v['after_implicit'] == v['after_explicit']
    for field in ['actual_match_template', 'h_ncc', 'i_ncc']:
        assert v['before_explicit'][field] == v['after_explicit'][field]
    assert v['before_explicit']['checked_patch_requests'] == 0
    if name == 'random':
        assert v['after_explicit']['actual_match_template'] == 100
        assert v['after_explicit']['h_ncc'] == v['after_explicit']['i_ncc'] == 25
        assert v['after_explicit']['checked_contrast_gates'] == 100
        assert v['after_explicit']['far_accept'] == 100
    if name == 'rectified': assert v['after_explicit']['checked_patch_requests'] == 0
    assert hashes == [hashlib.sha256(im.tobytes()).hexdigest() for im in images]
    records.append(item)
write(HERE/'PREFLIGHT.json', dict(records=records, all_full_outputs_exact=True,
    all_checked_patches_bit_exact=True, all_checked_contrast_gates_exact=True,
    raw_template_matches_and_ncc_calls_unchanged=True, input_arrays_unchanged=True,
    explicit_filters_keep_native_pair_path=True, h_i_modules_shared_unchanged=True,
    original_std_threshold_retained=True, patch_or_ncc_cache=False))
print('preflight exact all 6; patch bytes/gates exact; native full matching and NCC counts unchanged', flush=True)
