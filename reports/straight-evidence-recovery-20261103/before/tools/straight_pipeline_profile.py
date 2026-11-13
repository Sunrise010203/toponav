"""Opt-in, offline CURRENT straight-pipeline profiling; never an actuator.

Timing boundaries are controller calls only (not capture, decode, transport or
physics). Stage wrappers are scoped to one synchronous thread and restore all
aliases on exit. No production entrypoint imports this module.
"""
from contextlib import contextmanager
import cProfile
from dataclasses import asdict
from functools import wraps
import hashlib
import json
import os
from pathlib import Path
import platform
import pstats
import sys
import threading
import time

import numpy as np
from PIL import Image


class StageProbe:
    """Aggregate only selected calls, not an unbounded event trace.

    Inclusive intervals overlap; exclusive intervals subtract ALL instrumented
    children, including recursion. Profiler bookkeeping can appear in parent
    exclusive/uncovered time. This is attribution, not uninstrumented latency.
    """
    def __init__(self, *, wall=time.perf_counter, cpu=time.process_time):
        self.wall, self.cpu = wall, cpu
        self.stack = []
        self.active = False
        self.attached = False
        self.thread = None
        self.stats = {}

    def _wrapper(self, fn, label):
        @wraps(fn)
        def measured(*args, **kwargs):
            if not self.active or threading.get_ident() != self.thread:
                return fn(*args, **kwargs)
            node = [self.wall(), self.cpu(), 0., 0.]
            self.stack.append(node)
            try:
                return fn(*args, **kwargs)
            finally:
                cpu = self.cpu() - node[1]
                wall = self.wall() - node[0]
                assert self.stack.pop() is node
                if self.stack:
                    self.stack[-1][2] += wall
                    self.stack[-1][3] += cpu
                item = self.stats.setdefault(label, dict(calls=0, wall_inclusive_s=0., cpu_inclusive_s=0.,
                                                        wall_exclusive_s=0., cpu_exclusive_s=0.))
                item['calls'] += 1
                item['wall_inclusive_s'] += wall
                item['cpu_inclusive_s'] += cpu
                item['wall_exclusive_s'] += wall - node[2]
                item['cpu_exclusive_s'] += cpu - node[3]
        measured._straight_pipeline_probe = self
        return measured

    @contextmanager
    def attach(self, bindings):
        """Bindings: (owner, attribute, label); duplicate originals share wrapper.

        Collect aliases before entering. Late imports are not guaranteed to be
        patched; the profiling runner preloads its complete supported pipeline.
        """
        if self.attached or self.active:
            raise ValueError('Probe is already attached or active')
        patches, wrappers, seen = [], {}, {}
        for owner, name, label in bindings:
            key = (id(owner), name)
            if key in seen:
                if seen[key] != label:
                    raise ValueError('Conflicting labels for one binding')
                continue
            seen[key] = label
            original = vars(owner)[name]
            if not callable(original):
                raise ValueError('Stage binding must be a direct callable attribute')
            if hasattr(original, '_straight_pipeline_probe'):
                raise ValueError('Stage binding is already instrumented')
            existing = wrappers.get(id(original))
            if existing is not None and existing[0] != label:
                raise ValueError('One original callable cannot have two stage labels')
            if existing is None:
                wrappers[id(original)] = (label, self._wrapper(original, label))
            patches.append((owner, name, original, wrappers[id(original)][1]))
        changed = []
        self.attached, self.thread = True, threading.get_ident()
        try:
            for owner, name, original, wrapper in patches:
                setattr(owner, name, wrapper)
                changed.append((owner, name, original))
            yield self
        finally:
            for owner, name, original in reversed(changed):
                setattr(owner, name, original)
            self.attached, self.thread = False, None

    def measure(self, fn, *args, **kwargs):
        if not self.attached or self.active or self.stack or threading.get_ident() != self.thread:
            raise ValueError('Measurement requires one attached, idle calling thread')
        self.stats = {}
        start_cpu, start_wall = self.cpu(), self.wall()
        self.active = True
        try:
            result = fn(*args, **kwargs)
        finally:
            self.active = False
        wall, cpu = self.wall() - start_wall, self.cpu() - start_cpu
        assert not self.stack
        stats = {key: dict(value) for key, value in self.stats.items()}
        record = dict(wall_s=wall, cpu_s=cpu, stages=stats,
                      uncovered_wall_s=wall - sum(v['wall_exclusive_s'] for v in stats.values()),
                      uncovered_cpu_s=cpu - sum(v['cpu_exclusive_s'] for v in stats.values()))
        return result, record


def discover_bindings(targets, *, modules=None, explicit=()):
    """Discover direct function aliases by identity, without evaluating properties."""
    registry = {}
    for fn, label in targets:
        if id(fn) in registry and registry[id(fn)][1] != label:
            raise ValueError('Conflicting stage labels')
        registry[id(fn)] = (fn, label)
    if modules is None:
        modules = [m for name, m in list(sys.modules.items()) if name.startswith('roadvision.') and m is not None]
    bindings, seen = [], set()
    def inspect(owner):
        for name, value in list(vars(owner).items()):
            match = registry.get(id(value))
            if match is not None and value is match[0] and (id(owner), name) not in seen:
                seen.add((id(owner), name)); bindings.append((owner, name, match[1]))
    for module in modules:
        inspect(module)
        for value in list(vars(module).values()):
            if isinstance(value, type) and value.__module__.startswith('roadvision.'):
                inspect(value)
    for owner in explicit:
        inspect(owner)
    return bindings


def pipeline_bindings():
    """Preload all four supported variants BEFORE discovering aliases."""
    import cv2
    from scipy.optimize import least_squares
    from roadvision import continuous_straight as c, reference_straight as r, wide_reference_straight as w
    from roadvision import gradient_straight  # preload subclass and its producer
    from roadvision import ground_history as gh, straight_history_support as sh, ground_band as gb
    from roadvision import paired_descriptor_checks as pd, calibrated_ground_motion as gm
    from roadvision import joint_straight_model as joint, paired_corridor as paired, front_support as front
    from roadvision import rgb_ground_rig as rig
    targets = [
        (c.StraightController._local_stationary_motion, 'stationary_motion'),
        (c.StraightController._assess_startup_texture, 'startup_texture'),
        (r.ReferenceStraightController._assess_startup_texture, 'startup_texture'),
        (c.StraightController._navigation_history, 'history_selection'),
        (c.StraightController._probe_navigation_history, 'navigation_geometry'),
        (w.WideReferenceStraightController._probe_navigation_history, 'navigation_geometry'),
        (c.StraightController._assess_navigation_free_ground, 'free_path'),
        (gradient_straight.GradientTransferStraightController._assess_navigation_free_ground, 'free_path'),
        (c.StraightController._following_command, 'command_computation'),
        (sh.prepare_straight_history, 'history_preparation'),
        (gh._ground_evidence, 'ground_evidence'),
        (gb.track_fixed_roi, 'roi_tracking'),
        (pd.filter_pair, 'paired_descriptors'),
        (gm.estimate_ground_motion, 'metric_ground_fit'),
        (joint.estimate_joint_corridor, 'joint_corridor_fit'),
        (paired.estimate_paired_corridor, 'paired_corridor_fit'),
        (front.estimate_perpendicular_front, 'front_fit'),
        (rig._region_models, 'region_models'),
        (least_squares, 'nonlinear_solver'),
        (cv2.matchTemplate, 'template_matching'),
        (cv2.calcOpticalFlowPyrLK, 'optical_flow'),
        (cv2.warpPerspective, 'perspective_warp'),
    ]
    bindings = discover_bindings(targets, explicit=(cv2,))
    missing = {label for _, label in targets} - {label for _, _, label in bindings}
    if missing:
        raise ValueError('Missing stage bindings: ' + repr(missing))
    return bindings


def controller_for_task(task):
    from roadvision.continuous_straight import StraightController, InitialClearancePrior
    from roadvision.reference_straight import ReferenceStraightController
    from roadvision.wide_reference_straight import WideReferenceStraightController
    from roadvision.gradient_straight import GradientTransferStraightController
    from roadvision.corridor_rgb_probe import CALIBRATION_ID
    if not isinstance(task, dict):
        raise ValueError('Expected public straight task')
    variant = task.get('straight_variant', 'baseline')
    classes = dict(baseline=StraightController, reference=ReferenceStraightController,
                   wide_reference=WideReferenceStraightController, gradient_transfer=GradientTransferStraightController)
    if type(variant) is not str or variant not in classes:
        raise ValueError('Unsupported straight variant')
    prior = InitialClearancePrior(**task['initial_clearance'])
    expected = dict(version='straight_motion_permission_v1', motion_permission=task['motion_permission'],
                    initial_clearance=asdict(prior), calibration_id=CALIBRATION_ID)
    if variant != 'baseline':
        expected['straight_variant'] = variant
    if task != expected:
        raise ValueError('Public task differs from exact declared straight contract')
    return classes[variant](initial_clearance=prior, motion_permission=task['motion_permission'])


def load_case(base):
    """Read only decisions, public task/sequence and bound RGB; no result/audit."""
    base = Path(base).resolve(); public = (base / 'public').resolve()
    rows = [json.loads(line) for line in (base / 'decisions.jsonl').read_text().splitlines()]
    if not rows:
        raise ValueError('Empty public sequence')
    sequence = json.loads((public / 'sequence.json').read_text())
    task = json.loads((public / 'task.json').read_text())
    controller_for_task(task)  # validate public priors before profiling
    if sequence['frames'] != [row['input'] for row in rows if row['input'] is not None]:
        raise ValueError('Public sequence is not the exact ordered input list')
    from roadvision.corridor_rgb_probe import CALIBRATION_ID
    if sequence['calibration_id'] != CALIBRATION_ID:
        raise ValueError('Unsupported public calibration')
    frames, files = [], {base / 'decisions.jsonl', public / 'sequence.json', public / 'task.json'}
    for row in rows:
        ref = row['input']
        if ref is None:
            frames.append(None)
            continue
        if set(ref) != {'frame_id', 'captured_sim_s', 'rgb_path', 'rgb_sha256'}:
            raise ValueError('Unexpected public RGB reference fields')
        path = (public / ref['rgb_path']).resolve()
        if Path(ref['rgb_path']).is_absolute() or not path.is_relative_to(public):
            raise ValueError('Public RGB path escaped root')
        rgb = np.array(Image.open(path))
        if rgb.shape != (256, 256, 3) or rgb.dtype != np.uint8 or hashlib.sha256(rgb.tobytes()).hexdigest() != ref['rgb_sha256']:
            raise ValueError('Public RGB differs from bound input')
        files.add(path)
        frames.append(dict(frame_id=ref['frame_id'], captured_sim_s=ref['captured_sim_s'], rgb=rgb))
    for row in rows:
        if row.get('external_event') not in (None, 'tick_budget_exhausted'):
            raise ValueError('Unsupported external event')
    return dict(task=task, rows=rows, frames=frames, files=files)


def decision_call(controller, row, frame):
    if row.get('external_event') == 'tick_budget_exhausted':
        return controller.cancel(row['sim_s'])
    return controller.missing(row['sim_s']) if frame is None else controller.update(frame, row['sim_s'])


def distribution(values):
    values = np.asarray(values, float)
    if not len(values):
        return dict(count=0, p50_s=None, p90_s=None, p95_s=None, p99_s=None, mean_s=None,
                    max_s=None, over_0_1s_count=0, over_0_1s_fraction=None)
    if values.ndim != 1 or not np.isfinite(values).all() or (values < 0).any():
        raise ValueError('Expected finite nonnegative durations')
    return dict(count=len(values), **{f'p{p}_s': float(np.percentile(values, p)) for p in (50, 90, 95, 99)},
                mean_s=float(values.mean()), max_s=float(values.max()),
                over_0_1s_count=int((values > .1).sum()), over_0_1s_fraction=float((values > .1).mean()))


def verify_freeze(root, freeze):
    root = Path(root).resolve()
    for name, expected in freeze['files'].items():
        path = (root / name).resolve()
        if Path(name).is_absolute() or not path.is_relative_to(root):
            raise ValueError('Freeze path escaped project root')
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('Frozen file changed: ' + name)


def run_cases(cases, output, *, repeats=3, warmups=1, root, freeze):
    """One frozen current implementation, no before/after performance claim."""
    if type(repeats) is not int or repeats < 1 or type(warmups) is not int or warmups < 0:
        raise ValueError('Invalid repeat count')
    if not cases or len({name for name, _ in cases}) != len(cases):
        raise ValueError('Nonempty unique cases required')
    # Names become filenames; do not accept separators or aliases like '..'.
    if any(not name or any(c not in 'abcdefghijklmnopqrstuvwxyz0123456789_-' for c in name) for name, _ in cases):
        raise ValueError('Invalid case name')
    import cv2
    root = Path(root).resolve(); output = Path(output)
    verify_freeze(root, freeze)
    data = {name: load_case(base) for name, base in cases}
    for case in data.values():
        if any(str(p.relative_to(root)) not in freeze['files'] for p in case['files']):
            raise ValueError('Profile input was not frozen')
    bindings = pipeline_bindings()
    original_threads = cv2.getNumThreads()
    output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    rows_out = []
    def execute(name, mode, repeat):
        case = data[name]; controller = controller_for_task(case['task'])
        probe = StageProbe(); profiler = cProfile.Profile() if mode == 'cprofile' else None
        before = len(rows_out)
        def updates():
            for row, frame in zip(case['rows'], case['frames']):
                state_before = controller.state
                if mode == 'stage':
                    decision, timing = probe.measure(decision_call, controller, row, frame)
                else:
                    cpu = time.process_time(); wall = time.perf_counter()
                    if profiler is not None:
                        profiler.enable()
                    try:
                        decision = decision_call(controller, row, frame)
                    finally:
                        if profiler is not None:
                            profiler.disable()
                    timing = dict(wall_s=time.perf_counter() - wall, cpu_s=time.process_time() - cpu)
                if decision != row['decision']:
                    raise AssertionError((name, mode, repeat, row['tick'], 'full decision differs'))
                if mode != 'warmup':
                    item = dict(case=name, variant=case['task'].get('straight_variant', 'baseline'), mode=mode,
                                repeat=repeat, tick=row['tick'], state_before=state_before,
                                state_after=decision['state'], reason=decision['reason'],
                                command_nonzero=bool(decision['v_m_s'] or decision['omega_rad_s']),
                                frame_id=None if frame is None else frame['frame_id'],
                                exact=True, **timing)
                    rows_out.append(item)
                    with (output / 'rows.jsonl').open('a') as stream:
                        stream.write(json.dumps(item, allow_nan=False) + '\n')
        if mode == 'stage':
            with probe.attach(bindings):
                updates()
        else:
            updates()
        if profiler is not None:
            profiler.dump_stats(str(output / (name + '.prof')))
            stats = pstats.Stats(profiler)
            with (output / (name + '-cumulative.txt')).open('x') as stream:
                pstats.Stats(profiler, stream=stream).sort_stats('cumulative').print_stats(100)
            with (output / (name + '-self.txt')).open('x') as stream:
                pstats.Stats(profiler, stream=stream).sort_stats('tottime').print_stats(100)
            functions = []
            for (filename, line, function), (primitive, calls, own, cumulative, callers) in stats.stats.items():
                functions.append(dict(file=filename, line=line, function=function, primitive_calls=primitive,
                                      calls=calls, self_s=own, cumulative_s=cumulative,
                                      callers=[dict(file=k[0], line=k[1], function=k[2], values=list(v))
                                               for k, v in callers.items()]))
            payload = dict(total_self_s=stats.total_tt, total_calls=stats.total_calls,
                           functions=sorted(functions, key=lambda v: v['self_s'], reverse=True))
            (output / (name + '-cprofile.json')).write_text(json.dumps(payload, indent=2) + '\n')
        print(name, mode, repeat, 'exact', len(case['rows']), flush=True)
        return len(rows_out) - before
    try:
        for name, _ in cases:
            for repeat in range(warmups):
                execute(name, 'warmup', repeat)
        for repeat in range(repeats):
            order = cases if repeat % 2 == 0 else list(reversed(cases))
            for name, _ in order:
                execute(name, 'ordinary', repeat)
        for mode in ('stage', 'cprofile'):
            for name, _ in cases:
                execute(name, mode, 0)
        verify_freeze(root, freeze)
    finally:
        cv2.setNumThreads(original_threads)
    result = dict(scope='controller computation only; current optimized implementation, exposed saved inputs',
                  environment=dict(platform=platform.platform(), python=sys.version, numpy=np.__version__,
                                   opencv=cv2.__version__, measured_opencv_threads=1,
                                   restored_opencv_threads=cv2.getNumThreads(),
                                   thread_environment={key: os.environ.get(key) for key in
                                       ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS')},
                                   cpu_clock='process_time: all threads in this process, may exceed wall time'),
                  ordinary_repeats=repeats, warmups=warmups, instrumented_passes_per_case=2,
                  all_outputs_exact=True, measured_rows=len(rows_out),
                  warmup_exact_decisions=warmups * sum(len(v['rows']) for v in data.values()),
                  stage_bindings=len(bindings), instrumented_times_not_ordinary_latency=True,
                  host_exclusive=False, hardware_end_to_end_latency_measured=False,
                  acquisition_decode_serialization_transport_physics_excluded=True, cases={})
    for name, case in data.items():
        ordinary = [r for r in rows_out if r['case'] == name and r['mode'] == 'ordinary']
        groups = {'all_updates': ordinary,
                  'nonzero_command_updates': [r for r in ordinary if r['command_nonzero']]}
        for state in sorted({r['state_before'] for r in ordinary}):
            groups['state_before_' + state] = [r for r in ordinary if r['state_before'] == state]
        entry = dict(inputs=len(case['rows']), variant=case['task'].get('straight_variant', 'baseline'), distributions={})
        for group, items in groups.items():
            entry['distributions'][group] = {metric: distribution([r[metric] for r in items]) for metric in ('wall_s', 'cpu_s')}
        entry['slowest_ordinary_rows'] = sorted(ordinary, key=lambda r: r['wall_s'], reverse=True)[:10]
        result['cases'][name] = entry
    with (output / 'SUMMARY.json').open('x') as stream:
        json.dump(result, stream, allow_nan=False, indent=2); stream.write('\n')
    return result


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--freeze', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    plan = json.loads(args.plan.read_text()); freeze = json.loads(args.freeze.read_text())
    run_cases([(c['name'], root / c['path']) for c in plan['cases']], args.output,
              repeats=plan['ordinary_repeats'], warmups=plan['ordinary_warmups'], root=root, freeze=freeze)


if __name__ == '__main__':
    # Direct script execution must resolve the same local production package.
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    main()
