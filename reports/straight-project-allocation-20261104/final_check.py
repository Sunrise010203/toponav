"""Read-only exact-source, integer-nanosecond gate and delivery audit.

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
    old = read(HERE/'BEFORE_PINS.json')['files']
    changes = [p for p, expected in old.items() if sha(ROOT/p) != expected]
    assert changes == ['roadvision/corridor_rgb_probe.py'] and len(old) == 1092
    assert sha(HERE/'before/roadvision/corridor_rgb_probe.py') == old[changes[0]]
    for p in ('roadvision/corridor_rgb_probe.py', 'tests/test_homogeneous_projection.py'):
        assert (ROOT/p).read_bytes() == (HERE/'candidate'/p).read_bytes()
    before = ast.parse((HERE/'before/roadvision/corridor_rgb_probe.py').read_bytes())
    candidate = ast.parse((ROOT/'roadvision/corridor_rgb_probe.py').read_bytes())
    def rest(tree):
        return [ast.dump(node, include_attributes=False) for node in tree.body
                if not isinstance(node, ast.FunctionDef) or node.name != '_project']
    assert rest(before) == rest(candidate)
    b = next(node for node in before.body if isinstance(node, ast.FunctionDef) and node.name == '_project')
    c = next(node for node in candidate.body if isinstance(node, ast.FunctionDef) and node.name == '_project')
    assert [ast.dump(node) for node in b.body[1:]] == [ast.dump(node) for node in c.body[1:]]
    freezes = {}
    for name in ('FREEZE', 'FREEZE-v2', 'REPLAY_FREEZE', 'REPLAY_FREEZE-v2'):
        pins = read(HERE/(name+'.json'))['files']
        for p, expected in pins.items():
            assert sha(ROOT/p) == expected, p
        freezes[name] = len(pins)
    assert freezes == {'FREEZE': 646, 'FREEZE-v2': 681, 'REPLAY_FREEZE': 1911, 'REPLAY_FREEZE-v2': 1946}
    for name in ('FREEZE', 'FREEZE-v2'):
        assert sha(HERE/(name+'.json')) == (HERE/(name+'.sha256')).read_text().strip()
    for p, expected in read(HERE/'ANALYSIS_INPUT_PINS.json')['files'].items():
        assert sha(ROOT/p) == expected, p
    assert (HERE/'TIMING_ANALYSIS.json').read_bytes() == (HERE/'TIMING_ANALYSIS_INITIAL_REPRODUCED.json').read_bytes()
    assert (HERE/'TIMING_ANALYSIS_CORRECTED.json').read_bytes() == (HERE/'TIMING_ANALYSIS_CORRECTED_REPRODUCED.json').read_bytes()
    for directory in ('benchmark', 'benchmark-corrected'):
        summary = read(HERE/directory/'SUMMARY.json')
        gate, independently_computed = integer_gate(HERE/directory)
        assert gate is True and summary['complete_controller_performance_gate_passed'] is True
        assert summary['all_outputs_exact'] and summary['frozen_files_unchanged'] and summary['opencv_threads_restored']
        assert summary['measured_decisions'] == 1560 and summary['warmup_decisions'] == 390
        assert not summary['navigation_acceptance'] and not summary['physical_audit_performed'] and not summary['real_time_budget_proved']
        if directory == 'benchmark-corrected':
            corrected_gate = independently_computed
    correction = read(HERE/'MEASUREMENT_CORRECTION.json')
    assert not correction['first_comparison_valid_for_performance_acceptance']
    assert correction['first_gate_reported'] and not correction['original_and_corrected_samples_pooled']
    binding = read(HERE/'BENCHMARK_BASELINE_BINDING-v2.json')
    assert binding['baseline_pair_wrapper_symmetric'] and binding['per_controller_alias_switch_outside_timer']
    assert not binding['nested_per_pair_alias_switch_during_controller'] and binding['local_alias_switch_symmetric']
    assert len(binding['aliases']) == 11
    preflight = read(HERE/'PREFLIGHT.json')
    assert len(preflight['cases']) == 6
    for case in preflight['cases']:
        values = list(case['variants'].values())
        assert all(item == values[0] for item in values)
        assert case['all_outputs_exact'] and case['inputs_unchanged']
    second = read(HERE/'PREFLIGHT-v2.json')
    assert len(second['cases']) == 6 and second['all_outputs_exact'] and second['aliases_restored']
    for case in second['cases']:
        assert len(case['dispatches']) == 8 and all(row['exact'] for row in case['dispatches'])
        assert len({row['matches'] for row in case['dispatches']}) == 1
    replay = read(HERE/'REPLAY.json')
    assert len(replay['cases']) == 13 and replay['decisions'] == 581 and replay['terminal_frames'] == 39
    assert replay['all_public_outputs_and_final_results_equal'] and replay['old_physics_errors'] == []
    assert replay['frozen_files_unchanged'] and replay['opencv_threads_restored']
    assert all(row['old_physics']['errors'] == [] for row in replay['cases'])
    startup = read(HERE/'POSTPROCESS_STARTUP_CORRECTION.json')
    assert startup['replay_controller_calls_before_startup_fix'] == 0
    assert read(HERE/'replay-exit.json')['exit_code'] == 1 and read(HERE/'replay-v2-exit.json')['exit_code'] == 0
    initial, _ = tests(HERE/'targeted.xml')
    corrected, _ = tests(HERE/'targeted-v2.xml')
    callers, _ = tests(HERE/'projection-callers.xml')
    final, _ = tests(HERE/'final-targeted.xml')
    expanded, failed = tests(HERE/'final-expanded.xml')
    assert initial == dict(total=357, passed=357, failed=0, skipped=0)
    assert corrected == final == dict(total=368, passed=368, failed=0, skipped=0)
    assert callers == dict(total=107, passed=107, failed=0, skipped=0)
    assert expanded == dict(total=1572, passed=1539, failed=33, skipped=0)
    previous_failed = tests(ROOT/'reports/straight-paired-range-20261104/final-expanded.xml')[1]
    assert failed == previous_failed == read(HERE/'FAILURE_COMPARISON.json')['failures']
    assert read(HERE/'FAILURE_COMPARISON.json')['failed_nodes_and_messages_unchanged']
    full = read(ROOT/'reports/straight-project-allocation-full-current-20261104/summary.json')
    assert not full['verification_complete'] and full['exit_code'] == 2 and full['groups']['current']['selected'] == 0
    result = dict(candidate_promoted=True, acceptance_scope='Exact source optimization only, not navigation or physical acceptance',
                  performance_gate=True, independent_integer_nanosecond_gate=corrected_gate,
                  preexisting_files_byte_identical=1091, declared_changed_files=changes,
                  old_test_files_unchanged=True, original_other_function_asts_and_projection_checks_unchanged=True,
                  freeze_files_verified=freezes, full_controller_projection_aliases=11,
                  first_comparison_retained_but_invalid_for_performance_acceptance=True,
                  corrected_comparisons=1, original_and_corrected_samples_pooled=False,
                  total_controller_records_retained=3900, valid_measured_decisions=1560, valid_warmup_decisions=390,
                  all_serialized_outputs_exact=True, initial_targeted=initial, corrected_targeted=corrected,
                  projection_callers=callers, final_targeted=final, expanded=expanded, old_failures_unchanged=True,
                  old_recording_replay_cases=13, old_recording_replay_decisions=581, terminal_frames=39,
                  old_physics_errors=[], new_physics_or_actuator_execution=False,
                  full_current_exit=2, full_current_selected=0, verification_complete=False,
                  analyses_reproduced_byte_identical=True, navigation_acceptance=False,
                  real_time_budget_proved=False, S2_passed=False, S6_completed=False)
    with output.open('x') as stream:
        stream.write(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=HERE/'FINAL_VERIFICATION.json')
    main(parser.parse_args().output)
