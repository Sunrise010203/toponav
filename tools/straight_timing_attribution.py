"""Read-only descriptive attribution of already recorded serial AB/BA timings.

No roadvision, image decoding, runtime instrumentation or performance gate tuning.
Positive differences always mean after is slower. Rounds are repeated dependent
observations, not independent trials. Concentration uses gross positive deltas,
never the small signed net after cancellation.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics as st


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_bytes())


def lines(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def write(path, value):
    with path.open('x') as stream:
        stream.write(json.dumps(value, indent=2, allow_nan=False) + '\n')


def correlation(x, y):
    mx, my = st.mean(x), st.mean(y)
    xx = sum((v-mx)**2 for v in x)
    yy = sum((v-my)**2 for v in y)
    if xx == 0 or yy == 0:
        return None
    return sum((a-mx)*(b-my) for a, b in zip(x, y)) / math.sqrt(xx*yy)


def describe(pairs):
    dw = [p['after']['wall_s']-p['before']['wall_s'] for p in pairs]
    dc = [p['after']['cpu_s']-p['before']['cpu_s'] for p in pairs]
    positive = sorted((v for v in dw if v > 0), reverse=True)
    gross = sum(positive)
    totals = {label: {k: sum(p[label][k] for p in pairs)
                      for k in ('wall_s', 'cpu_s')}
              for label in ('before', 'after')}
    for v in totals.values():
        v['cpu_wall_ratio'] = v['cpu_s']/v['wall_s']
    def needed(fraction):
        total = 0.
        for i, v in enumerate(positive, 1):
            total += v
            if total >= fraction*gross:
                return i
        return 0
    return dict(
        n=len(pairs), totals=totals,
        wall_delta_s=sum(dw), cpu_delta_s=sum(dc),
        wall_delta_fraction=sum(dw)/totals['before']['wall_s'],
        median_paired_wall_delta_s=st.median(dw),
        median_paired_wall_ratio=st.median(p['after']['wall_s']/p['before']['wall_s'] for p in pairs),
        slower_count=sum(v > 0 for v in dw), faster_count=sum(v < 0 for v in dw),
        equal_count=sum(v == 0 for v in dw),
        gross_positive_wall_delta_s=gross,
        gross_negative_wall_delta_s=sum(v for v in dw if v < 0),
        top_positive_share={str(n): sum(positive[:n])/gross if gross else None for n in (1, 5, 10)},
        positive_pairs_to_half_gross=needed(.5), positive_pairs_to_80pct_gross=needed(.8),
        wall_cpu_delta_correlation=correlation(dw, dc),
    )


def analyze_case(name, rows, decisions, source, rounds):
    """Validate complete samples before joining to recorded output states."""
    ticks = [d['tick'] for d in decisions]
    if not ticks or any(type(t) is not int for t in ticks) or ticks != sorted(set(ticks)):
        raise ValueError('decision_ticks')
    if source['case'] != name or source['decisions_per_pass'] != len(ticks):
        raise ValueError('source_case_or_count')
    if source['all_outputs_exact'] is not True:
        raise ValueError('source_not_exact')
    expected = {(r, label, tick) for r in [-1, *range(rounds)]
                for label in ('before', 'after') for tick in ticks}
    index = {}
    for row in rows:
        if (row['case'] != name or type(row['repeat']) is not int
                or type(row['tick']) is not int or row['exact'] is not True):
            raise ValueError('invalid_row')
        key = row['repeat'], row['implementation'], row['tick']
        if key not in expected or key in index:
            raise ValueError('unexpected_or_duplicate_row')
        for k in ('wall_s', 'cpu_s'):
            if type(row[k]) not in (int, float) or not math.isfinite(row[k]) or row[k] <= 0:
                raise ValueError('nonpositive_or_nonfinite_timing')
        index[key] = row
    if index.keys() != expected:
        raise ValueError('incomplete_rows')
    expected_order = [(-1, 'before'), (-1, 'after')]
    for repeat in range(rounds):
        expected_order.extend((repeat, label) for label in
                              (('before', 'after') if repeat % 2 == 0 else ('after', 'before')))
    if [(r['repeat'], r['implementation'], r['tick']) for r in rows] != [
            (repeat, label, tick) for repeat, label in expected_order for tick in ticks]:
        raise ValueError('not_serial_ab_ba')
    sample_keys = [(s['repeat'], s['implementation']) for s in source['samples']]
    if sample_keys != expected_order[2:]:
        raise ValueError('source_sample_order')
    for sample in source['samples']:
        for k in ('wall_s', 'cpu_s'):
            total = sum(index[sample['repeat'], sample['implementation'], t][k] for t in ticks)
            if not math.isclose(total, sample[k], rel_tol=0, abs_tol=1e-9):
                raise ValueError('source_total_mismatch')
    pairs = []
    for repeat in range(rounds):
        for i, decision in enumerate(decisions):
            tick = decision['tick']
            pairs.append(dict(repeat=repeat, tick=tick, quarter=min(3, 4*i//len(ticks)),
                              state=decision['decision']['state'], reason=decision['decision']['reason'],
                              before=index[repeat, 'before', tick], after=index[repeat, 'after', tick]))
    grouped = defaultdict(list)
    for p in pairs:
        grouped[p['state'], p['reason']].append(p)
    per_round = [dict(repeat=r, order='AB' if r % 2 == 0 else 'BA',
                      **describe([p for p in pairs if p['repeat'] == r]),
                      quarters=[dict(quarter=q, **describe([p for p in pairs if p['repeat'] == r and p['quarter'] == q]))
                                for q in sorted({p['quarter'] for p in pairs})])
                 for r in range(rounds)]
    per_tick = [dict(tick=t, state=decisions[i]['decision']['state'], reason=decisions[i]['decision']['reason'],
                     wall_deltas_s=[index[r, 'after', t]['wall_s']-index[r, 'before', t]['wall_s'] for r in range(rounds)],
                     **describe([p for p in pairs if p['tick'] == t])) for i, t in enumerate(ticks)]
    consistency = Counter(t['slower_count'] for t in per_tick)
    totals = {}
    for label in ('before', 'after'):
        values = [s['wall_s'] for s in source['samples'] if s['implementation'] == label]
        totals[label] = dict(median=st.median(values), mean=st.mean(values),
                             minimum=min(values), maximum=max(values))
        for k in ('median', 'mean', 'minimum', 'maximum'):
            if not math.isclose(totals[label][k], source[label]['wall_s'][k], rel_tol=0, abs_tol=1e-9):
                raise ValueError('source_summary_mismatch')
    return dict(case=name, decisions_per_pass=len(ticks), measured_pairs=len(pairs),
                warmup_rows_retained_not_analyzed=2*len(ticks), round_total_wall=totals,
                wall_median_regression_fraction=totals['after']['median']/totals['before']['median']-1,
                wall_mean_regression_fraction=totals['after']['mean']/totals['before']['mean']-1,
                pooled_pairs=describe(pairs), per_round=per_round, per_tick=per_tick,
                tick_slower_rounds_histogram={str(k): consistency[k] for k in range(rounds+1)},
                by_recorded_state_reason=[dict(state=state, reason=reason, **describe(ps))
                                          for (state, reason), ps in sorted(grouped.items())],
                top_ten_ticks_by_mean_wall_delta=[dict(tick=t['tick'], state=t['state'], reason=t['reason'],
                    mean_wall_delta_s=t['wall_delta_s']/rounds, slower_rounds=t['slower_count'])
                    for t in sorted(per_tick, key=lambda t: t['wall_delta_s'], reverse=True)[:10]])


def run(root, freeze_path, output):
    freeze = read(freeze_path)['files']
    def verify():
        for path, sha in freeze.items():
            if digest(root/path) != sha:
                raise ValueError(('changed_frozen_input', path))
    verify()
    base = root/'reports/straight-paired-patches-20261103'
    def frozen(path):
        if str(path.relative_to(root)) not in freeze:
            raise ValueError(('unfrozen_input', str(path)))
        return path
    plan = read(frozen(base/'PLAN_CORRECTED.json'))
    original_freeze = read(frozen(base/'FREEZE_CORRECTED.json'))['files']
    recorded = lines(frozen(base/'benchmark-corrected/rows.jsonl'))
    if set(r['case'] for r in recorded) != {c['name'] for c in plan['cases']}:
        raise ValueError('unexpected_case')
    cases = []
    for case in plan['cases']:
        path = frozen(root/case['path']/'decisions.jsonl')
        if digest(path) != original_freeze[str(path.relative_to(root))]:
            raise ValueError('changed_historical_decisions')
        source = read(frozen(base/'benchmark-corrected'/f"{case['name']}-controller.json"))
        cases.append(analyze_case(case['name'], [r for r in recorded if r['case'] == case['name']],
                                  lines(path), source, plan['controller_rounds']))
    # Validate and reproduce the old gate, never introduce a new acceptance rule.
    gate = all(c['wall_median_regression_fraction'] < 0 and c['wall_mean_regression_fraction'] < 0 for c in cases)
    original_summary = read(frozen(base/'benchmark-corrected/SUMMARY.json'))
    if gate != original_summary['complete_controller_performance_gate_passed']:
        raise ValueError('historical_gate_disagreement')
    verify()
    output.mkdir(exist_ok=False, parents=True)
    write(output/'ANALYSIS.json', dict(
        methodology='Post-hoc descriptive; all measured samples retained. Deltas = after-before. No causal inference.',
        measured_rows=sum(2*c['measured_pairs'] for c in cases),
        warmup_rows_retained_not_analyzed=sum(c['warmup_rows_retained_not_analyzed'] for c in cases),
        original_performance_gate_passed=gate, candidate_promoted=False,
        new_performance_or_navigation_execution=False, cases=cases))
    write(output/'VERIFICATION.json', dict(frozen_files_checked=len(freeze),
        frozen_inputs_unchanged=True, original_decision_hashes_match=True,
        duplicate_or_missing_samples=False, all_recorded_outputs_exact=True,
        serial_ab_ba_order_verified=True, all_round_totals_match_original=True,
        original_gate_reproduced=True))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--freeze', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    run(args.root.resolve(), args.freeze, args.output)
