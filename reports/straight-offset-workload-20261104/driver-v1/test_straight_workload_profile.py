"""Synthetic workload observation; no physical or benchmark experiment."""
from contextlib import nullcontext
import hashlib
import json
from pathlib import Path
import types

import cv2
import numpy as np
import pytest

from roadvision import ground_history as gh, paired_descriptor_checks as pd
from roadvision.prepared_descriptor_checks import filter_h, filter_i
from tools.straight_pipeline_profile import StageProbe
from tools.straight_workload_profile import WorkloadProbe, bindings, run
from test_paired_descriptor_checks import inputs
from test_straight_pipeline_profile import fixture_case


@pytest.mark.parametrize('staged', [False, True])
@pytest.mark.parametrize('explicit', [False, True])
def test_native_pair_four_matches_no_identity_fallback(staged, explicit):
    images, region = inputs(count=1)
    before_images = [im.copy() for im in images]
    expected = pd.filter_pair(images, region)
    original = cv2.matchTemplate
    original_filters = (filter_h, filter_i)
    stage_bindings, workload_bindings = bindings()
    probe, stage = WorkloadProbe(), StageProbe()
    with probe.attach(workload_bindings):
        with stage.attach(stage_bindings) if staged else nullcontext():
            with probe.capture():
                kwargs = dict(filters=original_filters) if explicit else {}
                if staged:
                    actual, timing = stage.measure(gh.filter_pair, images, region, **kwargs)
                    assert timing['stages']['template_matching']['calls'] == 4
                    assert timing['stages']['h_patch_evidence']['calls'] == 1
                    assert timing['stages']['i_appearance_evidence']['calls'] == 1
                else:
                    actual = gh.filter_pair(images, region, **kwargs)
    snapshot = probe.snapshot()
    assert actual == expected
    assert sum(r['calls'] for r in snapshot['matches']) == 4
    assert snapshot['pairs'][0]['input_tracks'] == 1
    for row in snapshot['matches']:
        assert row['image_shape'] == [128, 128]
        assert row['template_shape'] == [11, 11]
        assert row['result_shape'] == [118, 118]
        assert row['result_elements'] == row['calls']*118*118
    assert cv2.matchTemplate is original
    assert (pd.filter_h, pd.filter_i) == original_filters
    for before, after in zip(before_images, images):
        np.testing.assert_array_equal(before, after)


def test_custom_filters_preserved_and_no_native_matches():
    images, region = inputs(count=1)
    sentinel_h, sentinel_i = dict(tracks=[]), dict(tracks=[])
    def h(*a, **kw): return sentinel_h
    def i(*a, **kw): return sentinel_i
    _, wb = bindings()
    probe = WorkloadProbe()
    with probe.attach(wb):
        with probe.capture():
            value = gh.filter_pair(images=images, region=region, filters=(h, i))
    assert value['H'] is sentinel_h and value['I'] is sentinel_i
    assert probe.snapshot()['matches'] == []


def test_real_rectified_fallback_still_exact():
    images, region = inputs(count=1, rectified=True)
    expected = pd.filter_pair(images, region, filters=(filter_h, filter_i))
    sb, wb = bindings()
    probe, stage = WorkloadProbe(), StageProbe()
    with probe.attach(wb), stage.attach(sb), probe.capture():
        value, _ = stage.measure(gh.filter_pair, images, region, filters=(filter_h, filter_i))
    assert value == expected
    assert probe.snapshot()['pairs'][0]['rectified'] is True


def test_outside_pair_return_identity_reset_and_no_inactive_observation():
    sentinel = np.zeros((3, 3), np.float32)
    ns = types.SimpleNamespace(fn=lambda **kw: sentinel)
    original = ns.fn
    probe = WorkloadProbe()
    with probe.attach([(ns, 'fn', 'match')]):
        assert ns.fn() is sentinel  # inactive, must not inspect arguments
        with probe.capture():
            got = ns.fn(image=np.zeros((4, 4)), templ=np.zeros((2, 2)), method=5)
        first = probe.snapshot()
        assert got is sentinel and first['matches'][0]['scope'] == 'outside_pair'
        assert first['matches'][0]['result_elements'] == 9
        with probe.capture(): pass
        assert probe.snapshot() == dict(pairs=[], matches=[])
        assert first['matches'][0]['calls'] == 1
    assert ns.fn is original


def test_exception_restores_aliases_and_nested_pair_stack():
    original_error = RuntimeError('original')
    def pair(*a, **kw): raise original_error
    ns = types.SimpleNamespace(pair=pair)
    images, region = inputs(count=1)
    probe = WorkloadProbe()
    with pytest.raises(RuntimeError) as actual:
        with probe.attach([(ns, 'pair', 'pair')]), probe.capture():
            ns.pair(images, region)
    assert actual.value is original_error
    assert ns.pair is pair and probe.stack == [] and not probe.active and not probe.attached


def test_reentry_and_incomplete_binding_rejected():
    ns = types.SimpleNamespace(fn=lambda: None)
    probe = WorkloadProbe()
    with pytest.raises(ValueError):
        with probe.capture(): pass
    with pytest.raises(ValueError):
        with probe.attach([(ns, 'fn', 'bad')]): pass
    with pytest.raises(ValueError):
        with probe.attach([(ns, 'fn', 'match'), (ns, 'fn', 'match')]): pass
    with probe.attach([]):
        with pytest.raises(ValueError):
            with probe.attach([]): pass
        with probe.capture():
            with pytest.raises(ValueError):
                with probe.capture(): pass
            with pytest.raises(ValueError): probe.snapshot()


@pytest.mark.parametrize('damage', [None, 'output', 'input_hash'])
def test_two_pass_runner_or_fail_closed(tmp_path, monkeypatch, damage):
    import tools.straight_workload_profile as driver
    base, rows, _ = fixture_case(tmp_path, 'gradient_transfer')
    if damage == 'output':
        rows[-1]['decision']['reason'] = 'deliberate_mismatch'
        (base/'decisions.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    monkeypatch.setattr(driver, 'bindings', lambda: ([], []))
    pins = {str(p.relative_to(tmp_path)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in base.rglob('*') if p.is_file()}
    if damage == 'input_hash': pins['input/public/task.json'] = 'wrong'
    plan = dict(case='input', expected_decisions_per_pass=3)
    original_threads, original_pair, original_match = cv2.getNumThreads(), pd.filter_pair, cv2.matchTemplate
    out = tmp_path/'out'
    if damage is None:
        result = run(tmp_path, dict(files=pins), plan, out)
        assert result['all_serialized_outputs_exact'] and result['measured_rows'] == 6
        assert result['workload_cross_pass_identical'] and result['aliases_restored']
        assert not result['candidate_executed'] and not result['performance_improvement_claim']
        recorded = [json.loads(r) for r in (out/'rows.jsonl').read_text().splitlines()]
        assert [r['mode'] for r in recorded] == ['stage']*3+['cprofile']*3
        assert (out/'current-offset.prof').is_file()
        with pytest.raises(FileExistsError): run(tmp_path, dict(files=pins), plan, out)
    else:
        with pytest.raises((ValueError, AssertionError)): run(tmp_path, dict(files=pins), plan, out)
        assert not (out/'SUMMARY.json').exists()
    assert cv2.getNumThreads() == original_threads
    assert pd.filter_pair is original_pair and cv2.matchTemplate is original_match
