"""Check the one frozen comparison without trimming or new acceptance rules."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_bytes())


def close(a, b):
    assert abs(a-b) < 1e-8, (a, b)


def main(output_path):
    # After rejection the production source is restored; validate the exact
    # measured source/test through their immutable candidate snapshots.
    archives = {'roadvision/corridor_rgb_probe.py': HERE/'candidate/roadvision/corridor_rgb_probe.py',
                'tests/test_homogeneous_projection.py': HERE/'candidate/tests/test_homogeneous_projection.py'}
    def measured_path(path):
        return archives.get(path, ROOT/path)
    frozen = read(HERE/'FREEZE-v2.json')['files']
    for p, sha in frozen.items():
        assert hashlib.sha256(measured_path(p).read_bytes()).hexdigest() == sha, p
    plan = read(HERE/'PLAN-v2.json')
    summary = read(HERE/'benchmark-corrected/SUMMARY.json')
    rows = [json.loads(line) for line in (HERE/'benchmark-corrected/rows.jsonl').read_text().splitlines()]
    assert len(rows) == 1950
    assert summary['all_outputs_exact'] is True
    assert summary['measured_decisions'] == 1560 and summary['warmup_decisions'] == 390
    result = []
    for case in plan['cases']:
        name = case['name']
        source = read(HERE/'benchmark-corrected'/f'{name}-controller.json')
        n = source['decisions_per_pass']
        selected = [r for r in rows if r['case'] == name]
        sequence = [(-1, label, tick) for label in ('before', 'after') for tick in range(n)]
        for r in range(4):
            sequence += [(r, label, tick) for label in (('before', 'after') if r % 2 == 0 else ('after', 'before')) for tick in range(n)]
        assert [(r['repeat'], r['implementation'], r['tick']) for r in selected] == sequence
        assert all(r['exact'] is True for r in selected)
        measured = [r for r in selected if r['repeat'] >= 0]
        totals = {}
        for label in ('before', 'after'):
            totals[label] = {}
            for metric in ('wall_s', 'cpu_s'):
                values = [sum(r[metric] for r in measured if r['implementation'] == label and r['repeat'] == repeat) for repeat in range(4)]
                totals[label][metric] = dict(median=statistics.median(values), mean=statistics.mean(values), values=values)
                for k in ('median', 'mean'):
                    close(totals[label][metric][k], source[label][metric][k])
                for repeat, v in enumerate(values):
                    close(v, next(s[metric] for s in source['samples'] if s['repeat'] == repeat and s['implementation'] == label))
            times = sorted(r['wall_s'] for r in measured if r['implementation'] == label)
            def quantile(q):
                index = (len(times)-1)*q
                floor = int(index)
                return times[floor]+(times[min(floor+1, len(times)-1)]-times[floor])*(index-floor)
            totals[label]['per_call_wall'] = dict(n=len(times), p50=quantile(.5), p95=quantile(.95),
                maximum=max(times), over_0_1_s=sum(t > .1 for t in times))
        median_gain = 1-totals['after']['wall_s']['median']/totals['before']['wall_s']['median']
        mean_gain = 1-totals['after']['wall_s']['mean']/totals['before']['wall_s']['mean']
        close(median_gain, source['wall_median_reduction_fraction'])
        close(mean_gain, source['wall_mean_reduction_fraction'])
        result.append(dict(case=name, decisions_per_pass=n, totals=totals,
            median_wall_reduction_fraction=median_gain, mean_wall_reduction_fraction=mean_gain))
    gate = all(c['median_wall_reduction_fraction'] > 0 and c['mean_wall_reduction_fraction'] > 0 for c in result)
    assert gate == summary['complete_controller_performance_gate_passed']
    baseline = read(HERE/'BEFORE_PINS.json')['files']
    changed = [p for p, sha in baseline.items() if hashlib.sha256(measured_path(p).read_bytes()).hexdigest() != sha]
    assert changed == ['roadvision/corridor_rgb_probe.py'], changed
    output = dict(measured_rows=1560, warmup_rows=390, all_samples_retained=True,
                  original_gate_passed=gate, cases=result,
                  baseline_pinned_changed_files=changed, frozen_files_unchanged=len(frozen),
                  physical_acceptance=False, real_time_budget_proved=False)
    with output_path.open('x') as stream:
        stream.write(json.dumps(output, indent=2)+'\n')
    print(json.dumps(dict(gate=gate, cases=[dict(case=c['case'], median_gain=c['median_wall_reduction_fraction'], mean_gain=c['mean_wall_reduction_fraction']) for c in result])), flush=True)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=HERE/'TIMING_ANALYSIS_CORRECTED.json')
    args = parser.parse_args()
    main(args.output)
