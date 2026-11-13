"""Fixed numerical experiment: compare uint8 maps, always return original maps."""
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))


def digest(a):
    return hashlib.sha256(a.tobytes()).hexdigest()


def write(path, value):
    with path.open('x') as f:
        f.write(json.dumps(value, indent=2, allow_nan=False)+'\n')


def to_uint8_exact(a):
    if type(a) is not np.ndarray or a.dtype != np.float32:
        return None
    if not np.isfinite(a).all() or np.any(a < 0) or np.any(a > 255):
        return None
    b = a.astype(np.uint8)
    return b if b.astype(np.float32).tobytes() == a.tobytes() else None


def compare_maps(first, second):
    if first.dtype != np.float32 or second.dtype != first.dtype or first.shape != second.shape:
        raise ValueError('map_contract')
    a = np.ascontiguousarray(first).view(np.uint32)
    b = np.ascontiguousarray(second).view(np.uint32)
    different = a != b
    indices = np.flatnonzero(different)
    finite = np.isfinite(first) & np.isfinite(second)
    delta = np.abs(first[finite].astype(np.float64)-second[finite].astype(np.float64))
    out = dict(shape=list(first.shape), elements=first.size,
               baseline_sha256=digest(first), alternate_sha256=digest(second),
               bit_exact=not bool(len(indices)), different_elements=len(indices),
               baseline_nonfinite=int(np.count_nonzero(~np.isfinite(first))),
               alternate_nonfinite=int(np.count_nonzero(~np.isfinite(second))),
               maximum_finite_absolute_difference=float(delta.max()) if delta.size else None,
               first_difference=None)
    if len(indices):
        index = np.unravel_index(int(indices[0]), first.shape)
        out['first_difference'] = dict(index=[int(v) for v in index],
            baseline_bits=f'{int(a[index]):08x}', alternate_bits=f'{int(b[index]):08x}',
            baseline_float_hex=float(first[index]).hex(), alternate_float_hex=float(second[index]).hex())
    return out


class Comparison:
    def __init__(self, output):
        self.output = output
        self.records = []
        self.witness_groups = set()

    def compare(self, original, group, target, template, method):
        hashes = digest(target), digest(template)
        baseline = original(target, template, method)
        target8, template8 = to_uint8_exact(target), to_uint8_exact(template)
        row = dict(number=len(self.records), group=group, method=int(method),
                   target_dtype=str(target.dtype), template_dtype=str(template.dtype),
                   target_shape=list(target.shape), template_shape=list(template.shape),
                   input_sha256=list(hashes), eligible=target8 is not None and template8 is not None)
        if row['eligible']:
            alternate = original(target8, template8, method)
            row['comparison'] = compare_maps(baseline, alternate)
            if not row['comparison']['bit_exact'] and group not in self.witness_groups:
                witness = f'witness-{row["number"]:04d}.npz'
                with (self.output/witness).open('xb') as f:
                    np.savez_compressed(f, target=target, template=template, baseline=baseline, alternate=alternate)
                row['witness'] = witness
                self.witness_groups.add(group)
        assert (digest(target), digest(template)) == hashes
        self.records.append(row)
        return baseline  # Alternate data NEVER enters evidence/filter execution.

    @contextmanager
    def attach(self, group):
        original = cv2.matchTemplate
        def observed(image, templ, method, *args, **kwargs):
            if args or kwargs:
                raise ValueError('unexpected_match_arguments')
            return self.compare(original, group, image, templ, method)
        try:
            cv2.matchTemplate = observed
            yield
        finally:
            cv2.matchTemplate = original


def synthetics(plan):
    rng = np.random.default_rng(plan['synthetic_seed'])
    y, x = np.indices((128, 128))
    random = rng.integers(0, 256, (128, 128), dtype=np.uint8)
    targets = dict(constant_0=np.zeros_like(random), constant_70=np.full_like(random, 70),
        constant_255=np.full_like(random, 255), random_full=random,
        random_low=rng.integers(120, 126, (128, 128), dtype=np.uint8),
        ramp_x=(2*x).astype(np.uint8), ramp_xy=((x+y)%256).astype(np.uint8),
        checker=(((x+y)%2)*255).astype(np.uint8),
        periodic=np.tile(random[:8,:8], (16,16)), fractional_extraction=random)
    assert list(targets) == plan['synthetic_cases']
    for name, target in targets.items():
        point = (62.125, 67.375) if name == 'fractional_extraction' else (62., 67.)
        template = cv2.getRectSubPix(target, (11,11), point)
        yield name, target.astype(np.float32), template.astype(np.float32)


def verify_pins(path):
    for p, sha in json.loads(path.read_bytes())['files'].items():
        if hashlib.sha256((ROOT/p).read_bytes()).hexdigest() != sha:
            raise AssertionError(('frozen_file_changed',p))


def run():
    from roadvision import paired_descriptor_checks as pd
    from roadvision.prepared_descriptor_checks import filter_h, filter_i
    from tools.straight_paired_population_benchmark import local_cases, encode
    verify_pins(HERE/'FREEZE.json')
    plan = json.loads((HERE/'PLAN.json').read_bytes())
    output = HERE/'numerical'
    output.mkdir(exist_ok=False)
    comparison = Comparison(output)
    match, threads = cv2.matchTemplate, cv2.getNumThreads()
    cases = []
    cv2.setNumThreads(plan['opencv_threads'])
    try:
        for name, target, template in synthetics(plan):
            comparison.compare(match, 'synthetic/'+name, target, template, cv2.TM_CCOEFF_NORMED)
        local = local_cases()
        assert [name for name, _, _ in local] == plan['local_cases']
        for name, images, region in local:
            original_inputs = [digest(im) for im in images]
            original_region = encode(region)
            for dispatch in ('implicit', 'explicit'):
                group = 'local/'+name+'/'+dispatch
                kwargs = {} if dispatch == 'implicit' else dict(filters=(filter_h,filter_i))
                expected = encode(pd.filter_pair(images,region,**kwargs))
                start = len(comparison.records)
                with comparison.attach(group):
                    actual = encode(pd.filter_pair(images,region,**kwargs))
                assert cv2.matchTemplate is match and actual == expected
                assert [digest(im) for im in images] == original_inputs
                assert encode(region) == original_region
                cases.append(dict(case=name,dispatch=dispatch,full_serialized_output_exact=True,
                    match_calls=len(comparison.records)-start,output_sha256=hashlib.sha256(actual).hexdigest()))
        verify_pins(HERE/'FREEZE.json')
    finally:
        cv2.setNumThreads(threads)
    assert cv2.matchTemplate is match and cv2.getNumThreads() == threads
    with (output/'rows.jsonl').open('x') as f:
        for r in comparison.records:
            f.write(json.dumps(r, allow_nan=False)+'\n')
    eligible = [r for r in comparison.records if r['eligible']]
    mismatch = [r for r in eligible if not r['comparison']['bit_exact']]
    result = dict(scope=plan['scope'], records=len(comparison.records), eligible_maps=len(eligible),
        ineligible_maps=len(comparison.records)-len(eligible), different_maps=len(mismatch),
        different_elements=sum(r['comparison']['different_elements'] for r in mismatch),
        compared_elements=sum(r['comparison']['elements'] for r in eligible),
        dtype_substitution_rejected=bool(mismatch), production_changes=False,
        alternate_maps_used_by_filters=False, all_local_outputs_exact=True, local_cases=cases,
        aliases_restored=True, opencv_threads_restored=True, input_arrays_unchanged=True,
        freeze_unchanged=True, ordinary_timing_runs=0, controller_sequences=0,
        performance_claim=False, physical_execution=False,
        versions=dict(numpy=np.__version__,opencv=cv2.__version__),
        opencv_threads=plan['opencv_threads'],opencv_optimized=cv2.useOptimized(),
        opencv_ipp=cv2.ipp.useIPP())
    write(output/'SUMMARY.json',result)
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    run()
