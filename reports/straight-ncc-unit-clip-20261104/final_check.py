"""Read-only restoration, integer-nanosecond gate and experiment audit.

Creates only an exclusive output; never executes a controller or benchmark.
"""
import argparse
import ast
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import statistics
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tests(path):
    cases = list(ET.parse(path).iter('testcase'))
    failures = []
    skipped = 0
    for case in cases:
        for kind in ('failure', 'error'):
            item = case.find(kind)
            if item is not None:
                failures.append(dict(classname=case.get('classname'), name=case.get('name'),
                                     message=item.get('message')))
        skipped += case.find('skipped') is not None
    return (dict(total=len(cases), passed=len(cases)-len(failures)-skipped,
                 failed=len(failures), skipped=skipped), failures)


def integer_gate(directory):
    # Durations were recorded in nanoseconds/1e9. Reconstitute exact integer
    # nanoseconds via decimal strings, independent of float sum roundoff.
    from decimal import Decimal
    rows = [json.loads(line) for line in (directory/'rows.jsonl').read_text().splitlines()]
    assert len(rows) == 1950 and all(row['exact'] is True for row in rows)
    ns = []
    for row in rows:
        value = Decimal(str(row['wall_s']))*10**9
        assert value > 0 and value == value.to_integral_value()
        ns.append((row, int(value)))
    result = []
    for name, count in [('known_center', 75), ('known_default', 40), ('known_offset_plus', 80)]:
        subset = [(row, duration) for row, duration in ns if row['case'] == name]
        order = [(-1, label, tick) for label in ('before', 'after') for tick in range(count)]
        for repeat in range(4):
            order += [(repeat, label, tick) for label in
                      (('before', 'after') if repeat % 2 == 0 else ('after', 'before'))
                      for tick in range(count)]
        assert [(row['repeat'], row['implementation'], row['tick']) for row, _ in subset] == order
        totals = {label: [sum(duration for row, duration in subset
                             if row['repeat'] == repeat and row['implementation'] == label)
                          for repeat in range(4)] for label in ('before', 'after')}
        median = lambda values: Fraction(sum(sorted(values)[1:3]), 2)
        mean = lambda values: Fraction(sum(values), 4)
        median_pass = median(totals['before']) > median(totals['after'])
        mean_pass = mean(totals['before']) > mean(totals['after'])
        result.append(dict(case=name, totals_ns=totals, median_improved=median_pass, mean_improved=mean_pass))
    return all(item['median_improved'] and item['mean_improved'] for item in result), result


def main(output):
    previous = read(HERE/'BEFORE_PINS.json')['files']
    assert len(previous) == 1211
    for p, expected in previous.items():
        assert sha(ROOT/p) == expected, p
    changed = 'roadvision/rgb_motion_checks.py'
    assert (ROOT/changed).read_bytes() == (HERE/'before'/changed).read_bytes()
    archives = {p: HERE/'candidate'/p for p in (
        changed, 'tests/test_ncc_unit_clip.py', 'tools/straight_function_benchmark.py',
        'tests/test_straight_function_benchmark.py')}
    for p, archived in archives.items():
        if p != changed:
            assert (ROOT/p).read_bytes() == archived.read_bytes()
    counts = {}
    for name in ('FREEZE', 'REPLAY_FREEZE'):
        pins = read(HERE/(name+'.json'))['files']
        for p, expected in pins.items():
            assert sha(archives.get(p, ROOT/p)) == expected, p
        counts[name] = len(pins)
    assert counts == dict(FREEZE=647, REPLAY_FREEZE=1973)
    assert sha(HERE/'FREEZE.json') == (HERE/'FREEZE.sha256').read_text().strip()
    before, candidate = [ast.parse(path.read_bytes()) for path in
                         (HERE/'before'/changed, HERE/'candidate'/changed)]
    dump = lambda node: ast.dump(node, include_attributes=False)
    def other_functions(tree):
        return [dump(node) for node in tree.body
                if not isinstance(node, ast.FunctionDef) or node.name != '_ncc']
    assert other_functions(before) == other_functions(candidate)
    b, c = [next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == '_ncc')
            for tree in (before, candidate)]
    assert [dump(node) for node in b.body[:-1]] == [dump(node) for node in c.body[:5]]
    legacy = b.body[-1].value
    assert dump(legacy.test) == dump(c.body[5].test)
    assert dump(legacy.orelse.args[0].args[0]) == dump(c.body[6].value)
    assert [dump(node) for node in legacy.orelse.args[0].args[1:]] == [
        dump(node) for node in c.body[-1].value.args[0].args[1:]]
    assert 'candidate/roadvision/rgb_motion_checks.py' in (ROOT/'tests/test_ncc_unit_clip.py').read_text()
    assert 'straight_function_benchmark' not in (ROOT/changed).read_text()
    summary = read(HERE/'benchmark/SUMMARY.json')
    gate, integers = integer_gate(HERE/'benchmark')
    assert gate is False and summary['complete_controller_performance_gate_passed'] is False
    assert summary['all_outputs_exact'] and summary['frozen_files_unchanged'] and summary['opencv_threads_restored']
    assert summary['measured_decisions'] == 1560 and summary['warmup_decisions'] == 390
    assert not summary['navigation_acceptance'] and not summary['physical_audit_performed'] and not summary['real_time_budget_proved']
    binding = read(HERE/'BENCHMARK_BASELINE_BINDING.json')
    assert binding['baseline_pair_wrapper_symmetric'] and binding['per_controller_alias_switch_outside_timer']
    assert binding['before_and_after_wrapper_class'] == 'FunctionVariant'
    assert not binding['nested_per_pair_alias_switch_during_controller'] and binding['local_alias_switch_symmetric']
    assert set(binding['aliases']) == {'roadvision.rgb_motion_checks._ncc', 'roadvision.rgb_resampling_checks._ncc',
                                      'roadvision.reference_patch_evidence._ncc'}
    assert all(read(HERE/'BENCHMARK_RUNTIME_RESTORED.json').values())
    preflight = read(HERE/'PREFLIGHT.json')
    assert len(preflight['cases']) == 6 and preflight['all_outputs_exact'] and preflight['aliases_restored']
    for case in preflight['cases']:
        variants = case['variants']
        assert variants['before_implicit'] == variants['before_explicit']
        assert variants['after_implicit'] == variants['after_explicit']
        old, new = variants['before_explicit'], variants['after_explicit']
        for field in old:
            if field != 'ncc_clip_calls':
                assert old[field] == new[field], (case['case'], field)
        assert len(old['norm_operands']) == 2*old['ncc_calls']
        assert new['ncc_clip_calls'] <= old['ncc_clip_calls']
        assert case['all_outputs_exact'] and case['inputs_unchanged']
    assert len(preflight['symmetric_dispatch_checks']) == 6
    for case in preflight['symmetric_dispatch_checks']:
        assert len(case['records']) == 4 and all(row['local_and_controller_scope_exact'] for row in case['records'])
    assert not (HERE/'REPLAY.json').exists() and not (HERE/'replay-progress.jsonl').exists()
    assert not read(HERE/'REPLAY_FREEZE_CHECK.json')['conditional_replay_executed']
    assert (HERE/'TIMING_ANALYSIS.json').read_bytes() == (HERE/'TIMING_ANALYSIS_REPRODUCED.json').read_bytes()
    for p, expected in read(HERE/'ANALYSIS_INPUT_PINS.json')['files'].items():
        assert sha(ROOT/p) == expected, p
    initial, _ = tests(HERE/'targeted.xml')
    final, _ = tests(HERE/'final-targeted.xml')
    expanded, failures = tests(HERE/'final-expanded.xml')
    assert initial == final == dict(total=459, passed=459, failed=0, skipped=0)
    assert expanded == dict(total=1663, passed=1630, failed=33, skipped=0)
    assert failures == tests(ROOT/'reports/straight-project-allocation-20261104/final-expanded.xml')[1]
    comparison = read(HERE/'FAILURE_COMPARISON.json')
    assert comparison['failed_nodes_and_messages_unchanged'] and comparison['failures'] == failures
    full = read(ROOT/'reports/straight-ncc-unit-clip-full-current-20261104/summary.json')
    assert full['verification_complete'] is False and full['exit_code'] == 2
    assert full['groups']['current']['selected'] == 0
    result = dict(candidate_promoted=False, performance_gate=False,
        independent_integer_nanosecond_gate=integers,
        preexisting_files_byte_identical=1211, restored_files=[changed], old_test_files_unchanged=True,
        measured_freeze_verified_via_snapshots=647, replay_freeze_verified_via_snapshots=1973,
        default_source_does_not_import_new_tool=True, experimental_candidate_retained_only_in_archive=True,
        new_numeric_tests_explicitly_load_archived_candidate=True,
        normalization_and_dot_division_asts_unchanged=True, other_function_asts_unchanged=True,
        full_controller_alias_binding_symmetric=True, ncc_aliases=3,
        initial_targeted=initial, final_targeted=final, expanded=expanded, old_failures_unchanged=True,
        full_current_exit=2, full_current_selected=0, verification_complete=False,
        analysis_reproduced_byte_identical=True, measured_decisions=1560, warmup_decisions=390,
        all_samples_retained=True, all_serialized_outputs_exact=True,
        additional_candidate_timing_or_profile=False, conditional_replay_executed=False,
        new_physics_or_actuator_execution=False, navigation_acceptance=False,
        real_time_budget_proved=False, S2_passed=False, S6_completed=False)
    with output.open('x') as stream:
        stream.write(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=HERE/'FINAL_VERIFICATION.json')
    main(parser.parse_args().output)


