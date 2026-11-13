"""Offline current-only full-sequence diagnostics, NOT a performance gate.

Two fixed instrumented passes, no candidate, warmup or ordinary timing repeat.
Only saved public task/RGB/decisions enter the controller. Counters observe input
sizes and native match calls; original arguments, arrays and returns are intact.
"""
from contextlib import contextmanager
import cProfile
from functools import wraps
import hashlib
import json
import os
from pathlib import Path
import platform
import pstats
import sys
import threading

from tools.straight_pipeline_profile import (
    StageProbe, controller_for_task, decision_call, discover_bindings, load_case,
    pipeline_bindings, verify_freeze,
)


def encode(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def write(path, value):
    with path.open('x') as stream:
        stream.write(json.dumps(value, indent=2, allow_nan=False)+'\n')


class WorkloadProbe:
    def __init__(self):
        self.active = self.attached = False
        self.thread = None
        self.stack = []
        self.pairs = []
        self.matches = {}

    def _wrapper(self, fn, label):
        @wraps(fn)
        def observed(*args, **kwargs):
            if not self.active or threading.get_ident() != self.thread:
                return fn(*args, **kwargs)
            if label == 'pair':
                images = args[0] if args else kwargs['images']
                region = args[1] if len(args) > 1 else kwargs['region']
                item = dict(view=region['view'], rectified=region['rectified_ground_assumption'],
                            input_tracks=len(region['tracks']),
                            image_shapes=[list(im.shape) for im in images])
                self.stack.append(item)
                try:
                    result = fn(*args, **kwargs)
                    item['retained'] = {k: len(v['tracks']) for k, v in result.items()}
                    self.pairs.append(item)
                    return result
                finally:
                    assert self.stack.pop() is item
            if label != 'match':
                raise ValueError('unknown_workload_label')
            image = args[0] if args else kwargs['image']
            templ = args[1] if len(args) > 1 else kwargs['templ']
            method = args[2] if len(args) > 2 else kwargs['method']
            result = fn(*args, **kwargs)
            current = self.stack[-1] if self.stack else None
            footprint = dict(scope='outside_pair' if current is None else 'paired_descriptor',
                             view=None if current is None else current['view'],
                             rectified=None if current is None else current['rectified'],
                             image_shape=list(image.shape), template_shape=list(templ.shape),
                             result_shape=list(result.shape), method=int(method))
            key = encode(footprint)
            entry = self.matches.setdefault(key, dict(**footprint, calls=0, result_elements=0))
            entry['calls'] += 1
            entry['result_elements'] += int(result.size)
            return result
        return observed

    @contextmanager
    def attach(self, bindings):
        if self.attached or self.active:
            raise ValueError('workload_already_attached')
        patches, wrappers, seen = [], {}, set()
        for owner, name, label in bindings:
            key = id(owner), name
            if key in seen:
                raise ValueError('duplicate_workload_binding')
            seen.add(key)
            original = vars(owner)[name]
            if label not in ('pair', 'match') or not callable(original):
                raise ValueError('invalid_workload_binding')
            wrapper_key = id(original), label
            wrappers.setdefault(wrapper_key, self._wrapper(original, label))
            patches.append((owner, name, original, wrappers[wrapper_key]))
        self.attached, self.thread = True, threading.get_ident()
        changed = []
        try:
            for owner, name, original, wrapper in patches:
                setattr(owner, name, wrapper)
                changed.append((owner, name, original))
            yield self
        finally:
            for owner, name, original in reversed(changed):
                setattr(owner, name, original)
            self.attached, self.thread = False, None

    @contextmanager
    def capture(self):
        if not self.attached or self.active or self.stack or threading.get_ident() != self.thread:
            raise ValueError('workload_not_idle_on_calling_thread')
        self.pairs, self.matches = [], {}
        self.active = True
        try:
            yield self
        finally:
            self.active = False
            assert not self.stack

    def snapshot(self):
        if self.active:
            raise ValueError('workload_still_active')
        # Copy via JSON so later captures cannot mutate an earlier record.
        return json.loads(encode(dict(pairs=self.pairs, matches=[self.matches[k] for k in sorted(self.matches)])))


def bindings():
    import cv2
    from roadvision import paired_descriptor_checks as pd, rgb_motion_checks as h, rgb_resampling_checks as i
    from roadvision import native_patch_contrast as contrast
    stages = pipeline_bindings()
    stages += discover_bindings([
        (h.patch_evidence, 'h_patch_evidence'), (i.appearance_evidence, 'i_appearance_evidence'),
        (contrast.h_evidence, 'h_patch_evidence'), (contrast.i_evidence, 'i_appearance_evidence'),
        (pd._prepare_h, 'h_pair_preparation'), (pd._prepare_i, 'i_pair_preparation'),
        (pd._finish_h, 'h_pair_finish'), (i._finish, 'i_descriptor_finish'),
    ])
    workload = discover_bindings([(pd.filter_pair, 'pair'), (cv2.matchTemplate, 'match')], explicit=(cv2,))
    pair_modules = {owner.__name__ for owner, _, label in workload if label == 'pair'}
    if not {'roadvision.ground_history', 'roadvision.straight_history_support'}.issubset(pair_modules):
        raise ValueError('missing_explicit_pair_aliases')
    return stages, workload


def functions(stats):
    return [dict(file=path, line=line, function=name, calls=nc, primitive_calls=cc,
                 self_s=tt, cumulative_s=ct)
            for (path, line, name), (cc, nc, tt, ct, _) in sorted(stats.stats.items())]


def run(root, freeze, plan, output):
    import cv2
    root, output = Path(root).resolve(), Path(output)
    verify_freeze(root, freeze)
    case = load_case(root/plan['case'])
    if len(case['rows']) != plan['expected_decisions_per_pass']:
        raise ValueError('unexpected_sequence_length')
    if any(str(p.relative_to(root)) not in freeze['files'] for p in case['files']):
        raise ValueError('unfrozen_public_input')
    stages, workload_bindings = bindings()
    if any(str(Path(sys.modules[owner.__module__ if isinstance(owner, type) else owner.__name__].__file__).resolve().relative_to(root))
           not in freeze['files'] for owner, _, _ in stages if getattr(owner, '__module__', getattr(owner, '__name__', '')).startswith('roadvision.')):
        raise ValueError('unfrozen_bound_source')
    originals = {(id(owner), name): (owner, name, vars(owner)[name]) for owner, name, _ in stages+workload_bindings}
    previous_threads = cv2.getNumThreads()
    output.mkdir(parents=True, exist_ok=False)
    records = []
    summaries = []
    environment = dict(python=sys.version, platform=platform.platform(), opencv=cv2.__version__,
                       measured_opencv_threads=1, host_exclusive=False,
                       thread_environment={k: os.environ.get(k) for k in
                           ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS')})
    cv2.setNumThreads(1)
    try:
        for mode in ('stage', 'cprofile'):
            controller = controller_for_task(case['task'])
            workload, stage = WorkloadProbe(), StageProbe()
            aggregate = None
            with workload.attach(workload_bindings):
                @contextmanager
                def timing_scope():
                    if mode == 'stage':
                        with stage.attach(stages):
                            yield
                    else:
                        yield
                with timing_scope():
                    for row, frame in zip(case['rows'], case['frames']):
                        before = controller.state
                        with workload.capture():
                            if mode == 'stage':
                                value, timing = stage.measure(decision_call, controller, row, frame)
                                profile_functions = None
                            else:
                                profile = cProfile.Profile()
                                profile.enable()
                                try:
                                    value = decision_call(controller, row, frame)
                                finally:
                                    profile.disable()
                                stats = pstats.Stats(profile)
                                profile_functions = functions(stats)
                                aggregate = stats if aggregate is None else aggregate.add(stats)
                                timing = None
                        if encode(value) != encode(row['decision']):
                            raise AssertionError((mode, row['tick'], 'full_serialized_output_differs'))
                        record = dict(mode=mode, tick=row['tick'], state_before=before,
                                      state_after=value['state'], reason=value['reason'], exact=True,
                                      workload=workload.snapshot(), timing=timing, functions=profile_functions)
                        records.append(record)
                        with (output/'rows.jsonl').open('a') as stream:
                            stream.write(encode(record)+'\n')
                        if (row['tick']+1) % 20 == 0:
                            print(mode, 'exact_through_tick', row['tick'], flush=True)
            if aggregate is not None:
                aggregate.dump_stats(str(output/'current-offset.prof'))
                write(output/'CPROFILE.json', dict(functions=functions(aggregate), total_self_s=aggregate.total_tt))
                for sort in ('cumulative', 'tottime'):
                    with (output/f'current-offset-{sort}.txt').open('x') as stream:
                        pstats.Stats(str(output/'current-offset.prof'), stream=stream).sort_stats(sort).print_stats(100)
            summaries.append(dict(mode=mode, exact_decisions=len(case['rows'])))
        first = [r for r in records if r['mode'] == 'stage']
        second = [r for r in records if r['mode'] == 'cprofile']
        if [(r['tick'], r['workload']) for r in first] != [(r['tick'], r['workload']) for r in second]:
            raise AssertionError('cross_pass_workload_differs')
        for frame, row in zip(case['frames'], case['rows']):
            if frame is not None and hashlib.sha256(frame['rgb'].tobytes()).hexdigest() != row['input']['rgb_sha256']:
                raise AssertionError('input_array_mutated')
        verify_freeze(root, freeze)
    finally:
        cv2.setNumThreads(previous_threads)
    restored = all(vars(owner)[name] is fn for owner, name, fn in originals.values())
    if not restored or cv2.getNumThreads() != previous_threads:
        raise AssertionError('runtime_binding_or_threads_not_restored')
    result = dict(scope='Current restored implementation, two instrumented diagnostic passes only',
                  environment=environment, passes=summaries, all_serialized_outputs_exact=True,
                  workload_cross_pass_identical=True, arrays_unchanged=True, aliases_restored=True,
                  opencv_threads_restored=True, measured_rows=len(records), frozen_files_unchanged=True,
                  instrumented_times_not_ordinary_latency=True, candidate_executed=False,
                  performance_improvement_claim=False, navigation_acceptance=False,
                  physical_execution=False, hardware_end_to_end_latency_measured=False)
    write(output/'SUMMARY.json', result)
    return result


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--freeze', required=True, type=Path)
    parser.add_argument('--plan', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    frozen = json.loads(args.freeze.read_bytes())
    verify_freeze(root, frozen)
    if str(args.plan.resolve().relative_to(root)) not in frozen['files']:
        raise ValueError('unfrozen_plan')
    print(json.dumps(run(root, frozen, json.loads(args.plan.read_bytes()), args.output), indent=2))
