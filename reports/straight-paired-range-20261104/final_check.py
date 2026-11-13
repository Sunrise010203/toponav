"""Read-only evidence checks; creates only a new final-verification output.

No controller execution, candidate timing, profiling, or replay.
"""
import argparse
import hashlib
import json
from pathlib import Path
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
            node = case.find(kind)
            if node is not None:
                failures.append(dict(classname=case.get('classname'), name=case.get('name'),
                                     message=node.get('message')))
        skipped += case.find('skipped') is not None
    return (dict(total=len(cases), passed=len(cases)-len(failures)-skipped,
                 failed=len(failures), skipped=skipped), failures)


def main(output):
    previous = read(HERE/'BEFORE_PINS.json')['files']
    for p, expected in previous.items():
        assert sha(ROOT/p) == expected, p
    assert len(previous) == 1013
    mapped_names = ('roadvision/native_patch_contrast.py', 'roadvision/native_patch_range.py',
                    'tests/test_native_patch_range.py', 'tests/test_prepared_support_mask.py')
    replacements = {p: HERE/'candidate'/p for p in mapped_names}
    counts = {}
    for name in ('FREEZE', 'REPLAY_FREEZE'):
        pins = read(HERE/(name+'.json'))['files']
        for p, expected in pins.items():
            assert sha(replacements.get(p, ROOT/p)) == expected, p
        counts[name] = len(pins)
    assert counts == dict(FREEZE=346, REPLAY_FREEZE=1744)
    assert sha(HERE/'FREEZE.json') == 'acfe56e2373825ba4c048660ff2e3ee5cd03dec0918e6ae995ef9fba583bc4cb'
    for p in ('roadvision/native_patch_contrast.py', 'tests/test_prepared_support_mask.py'):
        assert (ROOT/p).read_bytes() == (HERE/'before'/p).read_bytes()
    for p in ('roadvision/native_patch_range.py', 'tests/test_native_patch_range.py'):
        assert (ROOT/p).read_bytes() == (HERE/'candidate'/p).read_bytes()
    assert 'ARCHIVE/\'candidate/roadvision/native_patch_contrast.py\'' in (ROOT/'tests/test_native_patch_range.py').read_text()
    for p in ('roadvision/paired_descriptor_checks.py', 'roadvision/native_patch_contrast.py'):
        assert 'native_patch_range' not in (ROOT/p).read_text()
        assert 'prepared_support_mask' not in (ROOT/p).read_text()
    assert (HERE/'TIMING_ANALYSIS.json').read_bytes() == (HERE/'TIMING_ANALYSIS_REPRODUCED.json').read_bytes()
    gate = read(HERE/'benchmark/SUMMARY.json')
    assert gate['complete_controller_performance_gate_passed'] is False
    assert gate['all_outputs_exact'] and gate['frozen_files_unchanged'] and gate['opencv_threads_restored']
    assert gate['measured_decisions'] == 1560 and gate['warmup_decisions'] == 390
    rows = [json.loads(line) for line in (HERE/'benchmark/rows.jsonl').read_text().splitlines()]
    assert len(rows) == 1950 and all(row['exact'] is True for row in rows)
    assert not gate['navigation_acceptance'] and not gate['physical_audit_performed'] and not gate['real_time_budget_proved']
    assert not (HERE/'REPLAY.json').exists() and not (HERE/'replay-progress.jsonl').exists()
    assert not read(HERE/'REPLAY_FREEZE_CHECK.json')['conditional_replay_executed']
    preflight = read(HERE/'PREFLIGHT.json')
    assert len(preflight['cases']) == 6
    for case in preflight['cases']:
        variants = case['variants']
        for suffix in ('explicit', 'implicit'):
            old, new = variants['before_'+suffix], variants['after_'+suffix]
            for field in ('matches', 'support_samples', 'h_ncc', 'i_ncc', 'support_sha256'):
                assert old[field] == new[field], (case['case'], field)
            assert old['range_checks'] == 0
            assert new['range_checks'] == sum(new[k] for k in ('range_accept', 'range_reject', 'range_ambiguous'))
            if case['case'] == 'random':
                assert old['integrals'] == new['range_accept'] == old['matches'] == new['matches'] == 100
                assert new['integrals'] == 0
            if case['case'] == 'flat':
                assert new['range_reject'] == 100 and new['integrals'] == new['matches'] == 0
            if case['case'] == 'rectified':
                assert new['range_checks'] == 0
    initial, initial_failures = tests(HERE/'targeted.xml')
    corrected, _ = tests(HERE/'targeted-v2.xml')
    final, _ = tests(HERE/'final-targeted.xml')
    expanded, failures = tests(HERE/'final-expanded.xml')
    assert initial == dict(total=311, passed=310, failed=1, skipped=0)
    assert initial_failures[0]['name'] == 'test_default_pair_and_contrast_remain_original_bytes'
    assert corrected == final == dict(total=311, passed=311, failed=0, skipped=0)
    assert expanded == dict(total=1408, passed=1375, failed=33, skipped=0)
    old_failures = tests(ROOT/'reports/straight-paired-support-20261104/final-expanded.xml')[1]
    assert failures == old_failures
    comparison = read(HERE/'FAILURE_COMPARISON.json')
    assert comparison['failed_nodes_and_messages_unchanged'] and comparison['failures'] == failures
    for log in ('final-targeted.log', 'final-expanded.log'):
        assert '1 warning' in (HERE/log).read_text()
    full = read(ROOT/'reports/straight-paired-range-full-current-20261104/summary.json')
    assert full['verification_complete'] is False and full['exit_code'] == 2
    assert full['groups']['current']['selected'] == 0
    result = dict(candidate_promoted=False, performance_gate=False,
                  preexisting_files_byte_identical=len(previous),
                  restored_production_files=['roadvision/native_patch_contrast.py'],
                  restored_test_files=['tests/test_prepared_support_mask.py'],
                  no_old_test_assertion_permanently_changed=True,
                  measured_freeze_verified_via_snapshots=counts['FREEZE'],
                  replay_freeze_verified_via_snapshots=counts['REPLAY_FREEZE'],
                  default_does_not_import_new_helper=True, independent_experimental_helper_retained=True,
                  new_tests_explicitly_bind_archived_candidate=True,
                  initial_targeted=initial, corrected_targeted=corrected, final_targeted=final,
                  expanded=expanded, old_failures_unchanged=True,
                  full_current_exit=2, full_current_selected=0, verification_complete=False,
                  analysis_reproduced_byte_identical=True, measured_decisions=1560, warmup_decisions=390,
                  all_samples_retained=True, all_serialized_outputs_exact=True,
                  additional_candidate_timing_or_profile=False, conditional_replay_executed=False,
                  physical_execution=False, real_time_budget_proved=False, S2_passed=False, S6_completed=False)
    with output.open('x') as stream:
        stream.write(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=HERE/'FINAL_VERIFICATION.json')
    main(parser.parse_args().output)
