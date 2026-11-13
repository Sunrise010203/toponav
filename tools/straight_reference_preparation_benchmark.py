"""Bounded offline old/new reference-selection benchmark, never an actuator.

Uses explicit pre-change source snapshots and exposed RGB. Preparation timing
is not whole-controller latency, independent navigation or real-time proof.
Run with python -m tools.straight_reference_preparation_benchmark.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import time
from types import FunctionType

import cv2
import numpy as np
from PIL import Image

from roadvision import free_ground_path as candidate
from roadvision import straight_permission, reference_straight
from roadvision.calibrated_ground_motion import GroundCalibration
from roadvision.corridor_rgb_probe import CALIBRATION_ID

ROOT = Path(__file__).resolve().parents[1]


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def encode(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def load_source(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def clone(fn, **replacements):
    return FunctionType(fn.__code__, {**fn.__globals__, **replacements}, fn.__name__, fn.__defaults__)


def targets(before):
    """Same deterministic synthetic/exposed inputs for both implementations."""
    cal = GroundCalibration()
    rng = np.random.default_rng(110303)
    image = rng.integers(0, 256, (128, 128), dtype=np.uint8)
    mask = np.ones_like(image)
    examples = [('synthetic_random', image, mask),
                ('synthetic_blur', cv2.GaussianBlur(image, (3, 3), .6), mask),
                ('synthetic_flat', np.full_like(image, 127), mask),
                ('synthetic_repeated', np.tile(image[:8, :8], (16, 16)), mask)]
    padded = mask.copy(); padded[:, 40:57] = 0; padded[92:, :] = 0
    examples.append(('synthetic_padding', image, padded))
    fixture = ROOT / 'tests/fixtures'
    manifest = json.loads((fixture / 'straight_startup/manifest.json').read_bytes())
    rgbs = []
    for entry in manifest:
        rgb = np.array(Image.open(fixture / 'straight_startup' / entry['file']))
        if digest(rgb.tobytes()) != entry['rgb_sha256']: raise ValueError('fixture_pixel_pin')
        rgbs.append((Path(entry['file']).stem, rgb))
    rgb = np.array(Image.open(fixture / 'straight_reference/default_start.png'))
    pin = json.loads((fixture / 'straight_reference/manifest.json').read_bytes())
    if digest(rgb.tobytes()) != pin['rgb_sha256']: raise ValueError('fixture_pixel_pin')
    rgbs.append(('default_start', rgb))
    examples.extend((name, cv2.cvtColor(np.ascontiguousarray(rgb[:128, :128]), cv2.COLOR_RGB2GRAY), mask)
                    for name, rgb in rgbs)
    rows = []
    for name, image, support in examples:
        rows.append((f'candidates--{name}', '81_reference_candidates',
                     lambda g=before, im=image, m=support: g.reference_cell_candidates(im, m, cal),
                     lambda im=image, m=support: candidate.reference_cell_candidates(im, m, cal),
                     dict(gray_sha256=digest(image.tobytes()), mask_sha256=digest(support.tobytes()))))
    old_startup = clone(straight_permission.assess_startup_texture, select_reference_cells=before.select_reference_cells)
    old_joint = clone(reference_straight.assess_joint_startup_texture, reference_cell_candidates=before.reference_cell_candidates)
    for name, rgb in rgbs:
        for label, old, new in [('startup', old_startup, straight_permission.assess_startup_texture),
                                ('joint_startup', old_joint, reference_straight.assess_joint_startup_texture)]:
            rows.append((label + '--' + name, label,
                         lambda fn=old, im=rgb: fn(im, cal), lambda fn=new, im=rgb: fn(im, cal),
                         dict(rgb_sha256=digest(rgb.tobytes()))))
    meta = json.loads((fixture / 'straight_gradient/manifest.json').read_bytes())
    frames = []
    for entry in meta['frames']:
        rgb = np.array(Image.open(fixture / 'straight_gradient' / entry['rgb_path']))
        if digest(rgb.tobytes()) != entry['rgb_sha256']: raise ValueError('fixture_pixel_pin')
        frames.append(dict(frame_id=entry['frame_id'], captured_sim_s=entry['captured_sim_s'], rgb=rgb))
    kw = dict(now_s=frames[-1]['captured_sim_s'], calibration_id=CALIBRATION_ID)
    rows.append(('free_ground--exposed_gradient', 'full_free_ground_packet',
                 lambda: before.assess_free_ground(frames, meta['motion'], **kw),
                 lambda: candidate.assess_free_ground(frames, meta['motion'], **kw),
                 dict(frame_rgb_sha256=[digest(f['rgb'].tobytes()) for f in frames])))
    return rows


def statistics(rows):
    a = np.array(rows)
    return dict(n=len(rows), median_s=float(np.median(a)), p95_s=float(np.quantile(a, .95)),
                maximum_s=float(a.max()), mean_s=float(a.mean()))


def run(before_dir, pin_path, pin_sha256, output, rounds=8, samples=3):
    if type(rounds) is not int or not 2 <= rounds <= 32 or type(samples) is not int or not 1 <= samples <= 16:
        raise ValueError('bounded_rounds_samples')
    raw = pin_path.read_bytes()
    if digest(raw) != pin_sha256: raise ValueError('pin_digest')
    pins = json.loads(raw)
    if not pins or any(digest((ROOT / path).read_bytes()) != sha for path, sha in pins.items()):
        raise ValueError('input_pin_mismatch')
    # Pin all executed local Python sources, not just the two changed files.
    required = {str(p.relative_to(ROOT)) for p in (ROOT / 'roadvision').glob('*.py')}
    required.add(str(Path(__file__).resolve().relative_to(ROOT)))
    required.update(str(p.relative_to(ROOT)) for p in before_dir.rglob('*.py'))
    required.update(str(p.relative_to(ROOT)) for p in (ROOT / 'tests/fixtures').glob('straight_*/*') if p.is_file())
    if not required.issubset(pins): raise ValueError('incomplete_pins')
    before = load_source('_before_free_ground', before_dir / 'roadvision/free_ground_path.py')
    old_reference = load_source('_before_reference', before_dir / 'roadvision/reference_patch_evidence.py')
    before.reference_appearance_evidence = old_reference.reference_appearance_evidence
    cases = targets(before)
    output.mkdir(parents=True, exist_ok=False)
    write(output / 'plan.json', dict(cases=[dict(case=n, family=f, input=i) for n,f,_,_,i in cases],
         rounds=rounds, samples_per_round=samples, warmups_per_implementation=1,
         pin_file_sha256=pin_sha256, old_dir=str(before_dir), opencv_threads=1,
         timed_scope='in-memory function call only; output equality/JSON outside timer',
         alternating_order=True, independent_acceptance=False, controller_latency_measured=False))
    threads = cv2.getNumThreads(); rows = []
    try:
        cv2.setNumThreads(1)
        for name, family, old, new, inputs in cases:
            expected = encode(old())
            if encode(new()) != expected: raise AssertionError('warmup_equivalence:' + name)
            samples_out = []
            for round_index in range(rounds):
                order = [('before', old), ('after', new)]
                if round_index % 2: order.reverse()
                for label, fn in order:
                    for sample_index in range(samples):
                        cpu_start = time.process_time_ns(); start = time.perf_counter_ns()
                        value = fn()
                        elapsed = time.perf_counter_ns() - start; cpu_elapsed = time.process_time_ns() - cpu_start
                        if encode(value) != expected: raise AssertionError('sample_equivalence:' + name)
                        samples_out.append(dict(round=round_index, implementation=label, sample=sample_index,
                                                wall_s=elapsed / 1e9, process_cpu_s=cpu_elapsed / 1e9))
            result = dict(case=name, family=family, input=inputs, output_sha256=digest(expected), all_outputs_exact=True,
                          samples=samples_out)
            for label in ('before', 'after'):
                result[label] = {clock: statistics([s[clock] for s in samples_out if s['implementation'] == label])
                                 for clock in ('wall_s', 'process_cpu_s')}
            result['wall_median_reduction_fraction'] = 1 - result['after']['wall_s']['median_s'] / result['before']['wall_s']['median_s']
            write(output / (name + '.json'), result)
            rows.append({k: v for k, v in result.items() if k != 'samples'})
    finally:
        cv2.setNumThreads(threads)
    if any(digest((ROOT / path).read_bytes()) != sha for path, sha in pins.items()): raise ValueError('inputs_changed')
    summary = dict(cases=rows, case_count=len(rows), all_outputs_exact=True,
                   rounds=rounds, samples_per_implementation_per_case=rounds*samples,
                   input_pins_unchanged=True, opencv_threads_restored=cv2.getNumThreads() == threads,
                   versions=dict(numpy=np.__version__, opencv=cv2.__version__),
                   timing_scope='reference candidates, startup checks and free-ground packet; NOT whole controller',
                   navigation_acceptance=False, physical_stop_proved=False, production_gate_changed=False)
    write(output / 'summary.json', summary)
    return {k: v for k, v in summary.items() if k != 'cases'}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--before', type=Path, required=True)
    p.add_argument('--pins', type=Path, required=True)
    p.add_argument('--pins-sha256', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--rounds', type=int, default=8)
    p.add_argument('--samples', type=int, default=3)
    args = p.parse_args()
    print(json.dumps(run(args.before.resolve(), args.pins, args.pins_sha256, args.output, args.rounds, args.samples)))
