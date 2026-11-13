"""Independent arithmetic checks; imports no attribution or navigation code."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
BASE = ROOT/'reports/straight-paired-patches-20261103'


def read(path):
    return json.loads(path.read_bytes())


def close(a, b):
    assert abs(a-b) < 1e-9, (a, b)


def main():
    rows = [json.loads(line) for line in (BASE/'benchmark-corrected/rows.jsonl').read_text().splitlines()]
    analysis = read(HERE/'analysis/ANALYSIS.json')
    assert len(rows) == 1950
    assert analysis['measured_rows'] == 1560
    assert analysis['warmup_rows_retained_not_analyzed'] == 390
    assert analysis['original_performance_gate_passed'] is False
    pairs_checked = 0
    for case in analysis['cases']:
        name = case['case']
        measured = [r for r in rows if r['case'] == name and r['repeat'] >= 0]
        lookup = {(r['repeat'], r['implementation'], r['tick']): r for r in measured}
        assert len(lookup) == len(measured)
        by_tick = {t['tick']: t for t in case['per_tick']}
        for rnd in case['per_round']:
            r = rnd['repeat']
            d = [lookup[r, 'after', t]['wall_s']-lookup[r, 'before', t]['wall_s'] for t in by_tick]
            pairs_checked += len(d)
            close(sum(d), rnd['wall_delta_s'])
            close(statistics.median(d), rnd['median_paired_wall_delta_s'])
            assert sum(v > 0 for v in d) == rnd['slower_count']
            close(sum(rnd['quarters'][q]['wall_delta_s'] for q in range(4)), sum(d))
        all_deltas = []
        for t, item in by_tick.items():
            d = [lookup[r, 'after', t]['wall_s']-lookup[r, 'before', t]['wall_s'] for r in range(4)]
            assert d == item['wall_deltas_s']
            close(sum(d), item['wall_delta_s'])
            assert sum(v > 0 for v in d) == item['slower_count']
            all_deltas.extend(d)
        histogram = Counter(sum(lookup[r, 'after', t]['wall_s'] > lookup[r, 'before', t]['wall_s']
                                for r in range(4)) for t in by_tick)
        assert case['tick_slower_rounds_histogram'] == {str(k): histogram[k] for k in range(5)}
        positive = sorted([v for v in all_deltas if v > 0], reverse=True)
        close(sum(all_deltas), case['pooled_pairs']['wall_delta_s'])
        for n in (1, 5, 10):
            close(sum(positive[:n])/sum(positive), case['pooled_pairs']['top_positive_share'][str(n)])
        original = read(BASE/'benchmark-corrected'/f'{name}-controller.json')
        for label in ('before', 'after'):
            totals = [sum(r['wall_s'] for r in measured if r['repeat'] == repeat and r['implementation'] == label)
                      for repeat in range(4)]
            close(statistics.median(totals), original[label]['wall_s']['median'])
            close(statistics.mean(totals), original[label]['wall_s']['mean'])
        decisions = [json.loads(line) for line in (ROOT/f'reports/straight-evidence-recovery-20261103/comparison/{name}--gradient_transfer/decisions.jsonl').read_text().splitlines()]
        states = {d['tick']: (d['decision']['state'], d['decision']['reason']) for d in decisions}
        for group in case['by_recorded_state_reason']:
            subset = [r for r in measured if states[r['tick']] == (group['state'], group['reason'])]
            assert len(subset) == 2*group['n']
            close(sum((1 if r['implementation'] == 'after' else -1)*r['wall_s'] for r in subset), group['wall_delta_s'])
    assert pairs_checked == 780
    preexisting = read(HERE/'PREEXISTING_PINS.json')['files']
    for path, sha in preexisting.items():
        assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest() == sha, path
    for name in ('ANALYSIS.json', 'VERIFICATION.json'):
        assert (HERE/'analysis'/name).read_bytes() == (HERE/'reproduction'/name).read_bytes()
    result = dict(measured_pairs_independently_checked=pairs_checked, measured_rows=1560,
                  warmup_rows_retained=390, preexisting_files_byte_identical=len(preexisting),
                  old_gate_still_failed=True, analysis_byte_reproduced=True,
                  scope='Arithmetic/provenance only; no runtime or performance rerun')
    with (HERE/'INDEPENDENT_CHECK.json').open('x') as stream:
        stream.write(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
