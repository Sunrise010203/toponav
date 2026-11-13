"""Independent arithmetic/provenance audit, without rerunning the controller."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import pstats

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_bytes())


def close(a, b):
    assert abs(a-b) < 1e-8, (a, b)


def main():
    raw = [json.loads(line) for line in (HERE/'profile/rows.jsonl').read_text().splitlines()]
    stage, profile = raw[:80], raw[80:]
    assert len(raw) == 160
    assert [r['tick'] for r in stage] == [r['tick'] for r in profile] == list(range(80))
    assert all(r['mode'] == 'stage' for r in stage)
    assert all(r['mode'] == 'cprofile' for r in profile)
    functions = Counter()
    self_time = Counter()
    for first, second in zip(stage, profile):
        assert first['exact'] is True and second['exact'] is True
        assert first['workload'] == second['workload']
        counts = sum(m['calls'] for m in first['workload']['matches'])
        primitive = sum(f['calls'] for f in second['functions'] if f['file'] == '~' and f['function'] == '<matchTemplate>')
        assert counts == primitive == first['timing']['stages'].get('template_matching', {}).get('calls', 0)
        close(first['timing']['wall_s'], first['timing']['uncovered_wall_s']+
              sum(v['wall_exclusive_s'] for v in first['timing']['stages'].values()))
        for f in second['functions']:
            key = f['file'], f['line'], f['function']
            functions[key] += f['calls']
            self_time[key] += f['self_s']
    stored = pstats.Stats(str(HERE/'profile/current-offset.prof'))
    assert functions.keys() == stored.stats.keys()
    for key, value in stored.stats.items():
        assert functions[key] == value[1]
        close(self_time[key], value[2])
    analysis = read(HERE/'ANALYSIS.json')
    pair_calls = sum(len(r['workload']['pairs']) for r in stage)
    tracks = sum(p['input_tracks'] for r in stage for p in r['workload']['pairs'])
    pairs_matches = sum(m['calls'] for r in stage for m in r['workload']['matches'] if m['scope'] == 'paired_descriptor')
    all_matches = sum(m['calls'] for r in stage for m in r['workload']['matches'])
    assert (pair_calls, tracks, pairs_matches, all_matches) == (459, 28732, 114756, 126105)
    assert sum(r['timing']['stages'].get('h_patch_evidence', {}).get('calls', 0) for r in stage) == tracks
    close(sum(r['timing']['wall_s'] for r in stage), analysis['full_stage']['instrumented_wall_s'])
    for group_key in ('by_state_before', 'by_state_after_reason'):
        assert sum(g['decisions'] for g in analysis[group_key].values()) == 80
        close(sum(g['instrumented_wall_s'] for g in analysis[group_key].values()),
              analysis['full_stage']['instrumented_wall_s'])
    for pin_name in ('PREEXISTING_PINS.json', 'FREEZE.json', 'ANALYSIS_INPUT_PINS.json'):
        for path, sha in read(HERE/pin_name)['files'].items():
            assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest() == sha, path
    result = dict(complete_decisions_checked=160, workload_identical_every_tick=True,
                  binary_profile_matches_every_function_aggregate=True,
                  per_tick_exclusive_time_partition_verified=True,
                  total_pair_calls=pair_calls, total_pair_input_tracks=tracks,
                  total_pair_matches=pairs_matches, total_matches=all_matches,
                  original_gate_reopened=False, controller_or_physics_rerun=False,
                  all_preexisting_and_frozen_files_unchanged=True)
    with (HERE/'INDEPENDENT_CHECK.json').open('x') as stream:
        stream.write(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
