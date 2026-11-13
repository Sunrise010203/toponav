"""Read-only timing attribution, public-only inputs and unchanged decisions."""
import hashlib
import json
from pathlib import Path
import types

import numpy as np
from PIL import Image
import pytest

from tools.straight_pipeline_profile import (StageProbe, discover_bindings, pipeline_bindings,
    controller_for_task, load_case, decision_call, distribution, verify_freeze, run_cases)
from roadvision.continuous_straight import InitialClearancePrior
from roadvision.corridor_rgb_probe import CALIBRATION_ID
from dataclasses import asdict


class Clock:
    def __init__(self): self.value = 0.
    def __call__(self): return self.value
    def advance(self, seconds): self.value += seconds


def task(variant='baseline'):
    result = dict(version='straight_motion_permission_v1', motion_permission='no_motion',
                  initial_clearance=asdict(InitialClearancePrior(confirmed=True)), calibration_id=CALIBRATION_ID)
    if variant != 'baseline': result['straight_variant'] = variant
    return result


def fixture_case(tmp_path, variant='baseline'):
    base = tmp_path / 'input'; public = base / 'public'; public.mkdir(parents=True)
    rgb = np.random.default_rng(716).integers(0, 256, (256, 256, 3), dtype=np.uint8)
    Image.fromarray(rgb).save(public / 'frame.png')
    ref = dict(frame_id='one', captured_sim_s=.5, rgb_path='frame.png', rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest())
    frame = dict(frame_id='one', captured_sim_s=.5, rgb=rgb)
    c = controller_for_task(task(variant))
    rows = [dict(tick=0, sim_s=.5, input=ref), dict(tick=1, sim_s=.6, input=None),
            dict(tick=2, sim_s=.7, input=None, external_event='tick_budget_exhausted')]
    for row, f in zip(rows, [frame, None, None]): row['decision'] = decision_call(c, row, f)
    (base / 'decisions.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
    (public / 'task.json').write_text(json.dumps(task(variant)))
    (public / 'sequence.json').write_text(json.dumps(dict(calibration_id=CALIBRATION_ID, frames=[ref])))
    return base, rows, rgb


def test_nested_aliases_partition_and_original_return_identity():
    clock = Clock(); ns = types.ModuleType('roadvision.profile_fixture'); sentinel = object()
    def child(): clock.advance(2); return sentinel
    def parent():
        clock.advance(1); assert ns.alias() is sentinel; clock.advance(3); return sentinel
    ns.child = ns.alias = child; ns.parent = parent
    probe = StageProbe(wall=clock, cpu=clock)
    bindings = discover_bindings([(child, 'child'), (parent, 'parent')], modules=[ns])
    with probe.attach(bindings):
        assert ns.alias is ns.child and ns.alias is not child
        result, record = probe.measure(ns.parent)
    assert result is sentinel and ns.child is ns.alias is child and ns.parent is parent
    assert record['wall_s'] == record['cpu_s'] == 6.
    assert record['stages']['parent']['wall_inclusive_s'] == 6.
    assert record['stages']['parent']['wall_exclusive_s'] == 4.
    assert record['stages']['child']['wall_exclusive_s'] == 2.
    assert record['uncovered_wall_s'] == record['uncovered_cpu_s'] == 0.


def test_recursion_same_label_subtracts_children_without_double_counting():
    clock = Clock(); ns = types.SimpleNamespace()
    def recurse(n):
        clock.advance(1)
        return ns.fn(n - 1) if n else 7
    ns.fn = recurse; probe = StageProbe(wall=clock, cpu=clock)
    with probe.attach([(ns, 'fn', 'recursive')]): result, record = probe.measure(ns.fn, 3)
    assert result == 7 and record['wall_s'] == 4.
    assert record['stages']['recursive']['calls'] == 4
    assert record['stages']['recursive']['wall_inclusive_s'] == 10.
    assert record['stages']['recursive']['wall_exclusive_s'] == 4.


def test_uncovered_time_and_each_update_stats_reset():
    clock = Clock(); ns = types.SimpleNamespace()
    def child(): clock.advance(2)
    ns.fn = child; probe = StageProbe(wall=clock, cpu=clock)
    def caller(): clock.advance(3); ns.fn()
    with probe.attach([(ns, 'fn', 'child')]):
        _, first = probe.measure(caller)
        _, second = probe.measure(lambda: clock.advance(1))
    assert first['wall_s'] == 5. and first['uncovered_wall_s'] == 3.
    assert second['stages'] == {} and second['uncovered_wall_s'] == 1.
    assert first['stages']['child']['calls'] == 1  # snapshot isn't mutated


def test_exception_cleanup_and_unmeasured_calls_are_unchanged():
    ns = types.SimpleNamespace(); error = RuntimeError('original')
    def failure(): raise error
    ns.fn = failure; probe = StageProbe()
    with pytest.raises(RuntimeError) as actual:
        with probe.attach([(ns, 'fn', 'failure')]): probe.measure(ns.fn)
    assert actual.value is error and ns.fn is failure and probe.stack == []
    assert not probe.active and not probe.attached
    with probe.attach([(ns, 'fn', 'failure')]):
        with pytest.raises(RuntimeError): ns.fn()  # outside measure: no attribution
        assert probe.stats['failure']['calls'] == 1


def test_class_method_and_imported_alias_discovery():
    ns = types.ModuleType('roadvision.profile_fixture')
    class Example:
        def method(self, n): return n + 1
    Example.__module__ = ns.__name__
    ns.Example = Example; ns.alias = Example.method
    original = Example.method; probe = StageProbe()
    bindings = discover_bindings([(original, 'method')], modules=[ns])
    with probe.attach(bindings):
        value, record = probe.measure(Example().method, 4)
        assert value == 5 and ns.alias is Example.method
    assert record['stages']['method']['calls'] == 1 and Example.method is original


def test_attach_and_measure_reject_reentry_and_other_probe():
    ns = types.SimpleNamespace(fn=lambda: 1); probe = StageProbe(); other = StageProbe()
    with pytest.raises(ValueError): probe.measure(ns.fn)
    with probe.attach([(ns, 'fn', 'one')]):
        with pytest.raises(ValueError):
            with probe.attach([]): pass
        with pytest.raises(ValueError):
            with other.attach([(ns, 'fn', 'two')]): pass
        with pytest.raises(ValueError): probe.measure(lambda: probe.measure(ns.fn))
        assert not probe.active and not probe.stack
        assert probe.measure(ns.fn)[0] == 1


def test_partial_attach_rolls_back():
    class Locked:
        def __init__(self): object.__setattr__(self, 'fn', lambda: None)
        def __setattr__(self, name, value): raise RuntimeError('locked')
    a = types.SimpleNamespace(fn=lambda: 2); original = a.fn; probe = StageProbe()
    with pytest.raises(RuntimeError):
        with probe.attach([(a, 'fn', 'a'), (Locked(), 'fn', 'b')]): pass
    assert a.fn is original and not probe.attached


@pytest.mark.parametrize('kind', ['duplicate_label', 'same_fn_label', 'not_callable'])
def test_invalid_bindings_fail_without_patches(kind):
    fn = lambda: 1; ns = types.SimpleNamespace(a=fn, b=fn, bad=17)
    bindings = {'duplicate_label': [(ns, 'a', 'one'), (ns, 'a', 'two')],
                'same_fn_label': [(ns, 'a', 'one'), (ns, 'b', 'two')],
                'not_callable': [(ns, 'bad', 'bad')]}[kind]
    with pytest.raises(ValueError):
        with StageProbe().attach(bindings): pass
    assert ns.a is ns.b is fn


@pytest.mark.parametrize('variant', ['baseline', 'reference', 'wide_reference', 'gradient_transfer'])
def test_public_task_only_controls_factory_and_permission(variant):
    c = controller_for_task(task(variant))
    assert c.motion_permission == 'no_motion' and c.initial_clearance.confirmed
    assert c.motion_commands == 0


@pytest.mark.parametrize('change', [dict(truth_pose=[1, 2]), dict(calibration_id='other'),
    dict(straight_variant='unknown'), dict(motion_permission='drive'), dict(straight_variant='baseline')])
def test_extra_or_changed_task_fields_rejected(change):
    with pytest.raises(ValueError): controller_for_task(dict(task(), **change))


def test_input_loading_does_not_read_result_or_audit(tmp_path):
    base, rows, rgb = fixture_case(tmp_path)
    (base / 'result.json').write_text('not JSON and must not be opened')
    case = load_case(base)
    assert case['rows'] == rows and len(case['frames']) == 3
    np.testing.assert_array_equal(case['frames'][0]['rgb'], rgb)
    assert case['frames'][1:] == [None, None]
    assert not any(p.name == 'result.json' for p in case['files'])


@pytest.mark.parametrize('damage', ['pixels', 'manifest', 'escape', 'absolute', 'extra_ref', 'event', 'calibration', 'empty'])
def test_corrupt_or_nonpublic_inputs_rejected(tmp_path, damage):
    base, rows, _ = fixture_case(tmp_path); public = base / 'public'
    sequence = json.loads((public / 'sequence.json').read_text())
    if damage == 'pixels': Image.fromarray(np.zeros((256, 256, 3), np.uint8)).save(public / 'frame.png')
    elif damage == 'manifest': sequence['frames'] = []
    elif damage in ('escape', 'absolute', 'extra_ref'):
        if damage == 'extra_ref': rows[0]['input']['truth'] = 1
        else: rows[0]['input']['rgb_path'] = '../secret.png' if damage == 'escape' else '/tmp/frame.png'
        sequence['frames'] = [rows[0]['input']]
    elif damage == 'event': rows[-1]['external_event'] = 'runner_exception_zero_stop'
    elif damage == 'calibration': sequence['calibration_id'] = 'other'
    elif damage == 'empty': rows = []; sequence['frames'] = []
    (public / 'sequence.json').write_text(json.dumps(sequence))
    (base / 'decisions.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
    with pytest.raises(ValueError): load_case(base)


def test_summary_percentiles_denominators_and_empty_group():
    s = distribution([0., .1, .2, 1.])
    assert s['count'] == 4 and s['p50_s'] == pytest.approx(.15)
    assert s['p99_s'] == pytest.approx(.976)
    assert s['over_0_1s_count'] == 2 and s['over_0_1s_fraction'] == .5
    assert distribution([])['over_0_1s_fraction'] is None


@pytest.mark.parametrize('bad', [[np.inf], [np.nan], [-1.], [[1., 2.]]])
def test_bad_durations_not_silently_discarded(bad):
    with pytest.raises(ValueError): distribution(bad)


def test_freeze_mutation_and_escape_fail_closed(tmp_path):
    p = tmp_path / 'source.py'; p.write_text('before')
    frozen = dict(files={p.name: hashlib.sha256(p.read_bytes()).hexdigest()})
    verify_freeze(tmp_path, frozen)
    p.write_text('after')
    with pytest.raises(ValueError, match='changed'): verify_freeze(tmp_path, frozen)
    with pytest.raises(ValueError, match='escaped'): verify_freeze(tmp_path, dict(files={'../source.py': '0'}))


def test_pipeline_aliases_and_opencv_restore_exactly():
    import cv2
    from roadvision import ground_history as gh, straight_history_support as sh, continuous_straight as c
    bindings = pipeline_bindings(); original = cv2.matchTemplate; ground = gh._ground_evidence
    probe = StageProbe()
    with probe.attach(bindings):
        assert cv2.matchTemplate is not original
        assert gh._ground_evidence is sh._ground_evidence is c._ground_evidence
        assert gh._ground_evidence is not ground
    assert cv2.matchTemplate is original
    assert gh._ground_evidence is sh._ground_evidence is c._ground_evidence is ground


def test_end_to_end_all_modes_exact_no_motion_no_image_mutation(tmp_path):
    import cv2
    base, rows, rgb = fixture_case(tmp_path, 'gradient_transfer'); case = load_case(base)
    frozen = dict(files={str(p.relative_to(tmp_path)): hashlib.sha256(p.read_bytes()).hexdigest() for p in case['files']})
    threads = cv2.getNumThreads(); original = cv2.matchTemplate
    output = tmp_path / 'profile'
    result = run_cases([('one', base)], output, repeats=2, warmups=1, root=tmp_path, freeze=frozen)
    assert result['all_outputs_exact'] and result['measured_rows'] == 12 and result['warmup_exact_decisions'] == 3
    assert result['cases']['one']['distributions']['all_updates']['wall_s']['count'] == 6
    assert result['cases']['one']['distributions']['nonzero_command_updates']['wall_s']['count'] == 0
    assert cv2.matchTemplate is original and cv2.getNumThreads() == threads
    np.testing.assert_array_equal(load_case(base)['frames'][0]['rgb'], rgb)
    assert (output / 'one-cprofile.json').exists() and (output / 'one.prof').exists()
    with pytest.raises(FileExistsError): run_cases([('one', base)], output, root=tmp_path, freeze=frozen)


def test_changed_decision_aborts_without_producing_success(tmp_path):
    base, rows, _ = fixture_case(tmp_path)
    rows[0]['decision']['v_m_s'] = .15
    (base / 'decisions.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
    case = load_case(base)
    freeze = dict(files={str(p.relative_to(tmp_path)): hashlib.sha256(p.read_bytes()).hexdigest() for p in case['files']})
    out = tmp_path / 'bad-output'
    with pytest.raises(AssertionError): run_cases([('one', base)], out, root=tmp_path, freeze=freeze)
    assert not (out / 'SUMMARY.json').exists()
