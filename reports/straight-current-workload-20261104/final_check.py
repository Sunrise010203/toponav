"""Read-only final checks of current paths, original bytes and diagnostic provenance."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_bytes())


def verify(name):
    files = read(HERE/name)['files']
    for p, sha in files.items():
        assert hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == sha, p
    return len(files)


def main():
    counts = {name: verify(name) for name in (
        'PREEXISTING_PINS.json', 'FREEZE.json', 'FREEZE-v2.json', 'ANALYSIS_INPUT_PINS.json')}
    prior = read(ROOT/'reports/straight-paired-contrast-20261104/FINAL_PINS.json')['files']
    for p, sha in prior.items():
        assert hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == sha, p
    assert read(HERE/'profile-exit.json')['exit_code'] == 0
    assert read(HERE/'tests-exit.json')['exit_code'] == 4  # Original collection failure retained.
    assert read(HERE/'tests-v2-exit.json')['exit_code'] == 0
    cases = list(ET.parse(HERE/'tests-v2.xml').iter('testcase'))
    assert len(cases) == 262 and all(len(c) == 0 for c in cases)
    profile = read(HERE/'profile/CPROFILE.json')['functions']
    tracks = read(HERE/'INDEPENDENT_CHECK.json')['total_pair_input_tracks']
    def calls(file, function):
        return sum(f['calls'] for f in profile if Path(f['file']).name == file and f['function'] == function)
    native = {name: calls('native_patch_contrast.py', name) for name in (
        'h_evidence', 'i_evidence', '_native_h_evidence', '_native_i_evidence', '_contrast_gate')}
    assert all(native[n] == tracks for n in ('h_evidence', 'i_evidence', '_native_h_evidence', '_native_i_evidence'))
    assert native['_contrast_gate'] > 0
    assert calls('~', '<integral2>') == native['_contrast_gate']
    assert calls('local_exclusion_population.py', 'outside') > 0
    summary = read(HERE/'profile/SUMMARY.json')
    assert summary['all_serialized_outputs_exact'] and summary['workload_cross_pass_identical']
    assert summary['arrays_unchanged'] and summary['aliases_restored'] and summary['opencv_threads_restored']
    result = dict(pin_counts_verified=counts, latest_accepted_final_pins_verified=len(prior),
                  default_source_unchanged=True, native_contrast_calls=native,
                  native_local_exclusion_calls=calls('local_exclusion_population.py', 'outside'),
                  targeted_passed=262, targeted_skipped=0, initial_collection_failure_retained=True,
                  profile_exit_code=0, diagnostic_passes=2, decisions_each_pass=80,
                  candidate_executed=False, ordinary_timing_runs=0, replay_runs=0,
                  performance_improvement_claim=False, full_current_reverified=False,
                  verification_complete=False, navigation_acceptance=False, physical_execution=False,
                  real_time_budget_proved=False)
    with (HERE/'FINAL_CHECK.json').open('x') as f:
        f.write(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
