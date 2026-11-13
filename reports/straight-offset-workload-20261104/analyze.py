"""Summarize the two complete diagnostic passes without a performance claim."""
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_bytes())


def write(path, value):
    with path.open('x') as stream:
        stream.write(json.dumps(value, indent=2, allow_nan=False)+'\n')


def summarize(rows):
    stages = defaultdict(Counter)
    matches = Counter()
    populations = {}
    views = defaultdict(Counter)
    primitives = Counter()
    for r in rows:
        for label, values in (r['timing'] or {}).get('stages', {}).items():
            stages[label].update(values)
        for item in r['workload']['pairs']:
            view = item['view']+('/rectified' if item['rectified'] else '/native')
            views[view].update(calls=1, input_tracks=item['input_tracks'],
                               h_retained=item['retained']['H'], i_retained=item['retained']['I'])
        for item in r['workload']['matches']:
            footprint = {k: v for k, v in item.items() if k not in ('calls', 'result_elements')}
            key = json.dumps(footprint, sort_keys=True)
            previous = populations.setdefault(key, dict(**footprint, calls=0, result_elements=0))
            previous['calls'] += item['calls']
            previous['result_elements'] += item['result_elements']
            matches[item['scope']] += item['calls']
        for f in r['functions'] or []:
            if f['file'] == '~' or Path(f['file']).name in (
                'paired_descriptor_checks.py', 'rgb_motion_checks.py', 'rgb_resampling_checks.py',
                'prepared_descriptor_search.py', 'calibrated_ground_motion.py', 'ground_band.py'):
                key = Path(f['file']).name+':'+f['function']
                primitives[key] += f['calls']
    wall = sum((r['timing'] or {}).get('wall_s', 0.) for r in rows)
    cpu = sum((r['timing'] or {}).get('cpu_s', 0.) for r in rows)
    uncovered = sum((r['timing'] or {}).get('uncovered_wall_s', 0.) for r in rows)
    exclusive = sum(s['wall_exclusive_s'] for s in stages.values())
    if abs(wall-exclusive-uncovered) > 1e-8:
        raise AssertionError('exclusive_partition_failed')
    return dict(decisions=len(rows), instrumented_wall_s=wall, instrumented_cpu_s=cpu,
                uncovered_wall_s=uncovered,
                stages=[dict(stage=k, **v, exclusive_fraction_of_instrumented_wall=v['wall_exclusive_s']/wall)
                        for k, v in sorted(stages.items(), key=lambda kv: kv[1]['wall_exclusive_s'], reverse=True)],
                pair_regions_by_view={k: dict(v) for k, v in sorted(views.items())},
                match_calls_by_scope=dict(matches),
                match_footprints=[populations[k] for k in sorted(populations)],
                primitive_calls=dict(sorted(primitives.items())))


def main():
    summary = read(HERE/'profile/SUMMARY.json')
    assert summary['measured_rows'] == 160 and summary['all_serialized_outputs_exact']
    rows = [json.loads(line) for line in (HERE/'profile/rows.jsonl').read_text().splitlines()]
    assert [(r['mode'], r['tick']) for r in rows] == [(mode, t) for mode in ('stage', 'cprofile') for t in range(80)]
    stage, profile = rows[:80], rows[80:]
    for s, p in zip(stage, profile):
        assert s['exact'] is True and p['exact'] is True
        assert s['workload'] == p['workload']
        total_matches = sum(item['calls'] for item in s['workload']['matches'])
        actual_matches = sum(f['calls'] for f in p['functions'] if f['file'] == '~' and f['function'] == '<matchTemplate>')
        assert total_matches == actual_matches == s['timing']['stages'].get('template_matching', {}).get('calls', 0)
        assert len(s['workload']['pairs']) == s['timing']['stages'].get('paired_descriptors', {}).get('calls', 0)
    out = dict(scope='Instrumented attribution of current only; no before/after comparison or causality',
               full_stage=summarize(stage), full_profile_workload=summarize(profile),
               by_state_before={}, by_state_after_reason={}, contiguous_quarters=[], focus_ticks=[])
    for state in sorted({r['state_before'] for r in stage}):
        out['by_state_before'][state] = summarize([r for r in stage if r['state_before'] == state])
    for state, reason in sorted({(r['state_after'], r['reason']) for r in stage}):
        out['by_state_after_reason'][state+'/'+reason] = summarize([r for r in stage if (r['state_after'], r['reason']) == (state, reason)])
    out['contiguous_quarters'] = [dict(ticks=[q*20, q*20+19], **summarize(stage[q*20:q*20+20])) for q in range(4)]
    ticks = read(HERE/'PLAN.json')['focus_ticks']
    out['focus_ticks'] = [dict(tick=t, state_before=stage[t]['state_before'], state_after=stage[t]['state_after'],
                             stage=summarize([stage[t]]), profile_workload=summarize([profile[t]])) for t in ticks]
    write(HERE/'ANALYSIS.json', out)
    pins = read(HERE/'PREEXISTING_PINS.json')['files']
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == sha for p, sha in pins.items())
    write(HERE/'ANALYSIS_VERIFICATION.json', dict(
        measured_rows=160, decisions_each_pass=80, entire_ordered_sequences=True,
        all_exact_recorded=True, cross_pass_workload_equal=True, exclusive_partition_verified=True,
        primitive_match_counts_equal_stage_and_workload_every_tick=True,
        preexisting_files_unchanged=len(pins), new_candidate=False, performance_improvement_claim=False))


if __name__ == '__main__':
    main()
