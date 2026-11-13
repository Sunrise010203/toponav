"""Synthetic checks only; no benchmark or navigation reruns."""
from copy import deepcopy
import math
import statistics as st

import pytest

from tools.straight_timing_attribution import analyze_case, describe


def fixture():
    decisions = [dict(tick=t, decision=dict(state='observing', reason='fixture')) for t in range(4)]
    rows, samples = [], []
    for repeat in (-1, 0, 1, 2, 3):
        order = ('before', 'after') if repeat % 2 == 0 or repeat == -1 else ('after', 'before')
        for label in order:
            wall = 1. if label == 'before' else 1.1
            rows.extend(dict(case='fixture', implementation=label, repeat=repeat, tick=t,
                             wall_s=wall, cpu_s=wall*2, exact=True) for t in range(4))
            if repeat >= 0:
                samples.append(dict(repeat=repeat, implementation=label, wall_s=wall*4, cpu_s=wall*8))
    source = dict(case='fixture', decisions_per_pass=4, samples=samples, all_outputs_exact=True)
    for label in ('before', 'after'):
        vals = [s['wall_s'] for s in samples if s['implementation'] == label]
        source[label] = dict(wall_s=dict(median=st.median(vals), mean=st.mean(vals),
                                         minimum=min(vals), maximum=max(vals)))
    return rows, decisions, source


def test_complete_join_and_warmup_separation():
    rows, decisions, source = fixture()
    for row in rows:
        if row['repeat'] == -1:
            row['wall_s'] = 900.
    result = analyze_case('fixture', rows, decisions, source, 4)
    assert result['measured_pairs'] == 16
    assert result['warmup_rows_retained_not_analyzed'] == 8
    assert result['wall_mean_regression_fraction'] == pytest.approx(.1)
    assert result['tick_slower_rounds_histogram']['4'] == 4
    assert result['by_recorded_state_reason'][0]['n'] == 16
    assert result['per_round'][0]['totals']['before']['cpu_wall_ratio'] == 2
    assert len(result['per_round'][0]['quarters']) == 4


@pytest.mark.parametrize('mutation', [
    'duplicate', 'missing', 'missing_warmup', 'wrong_tick', 'unexpected_repeat', 'false_exact',
    'nan', 'inf', 'negative', 'zero', 'boolean', 'order', 'total', 'summary', 'decision_tick',
    'duplicate_sample', 'source_not_exact',
])
def test_fail_closed(mutation):
    rows, decisions, source = fixture()
    if mutation == 'duplicate': rows.append(deepcopy(rows[-1]))
    elif mutation == 'missing': rows.pop()
    elif mutation == 'missing_warmup': rows.pop(0)
    elif mutation == 'wrong_tick': rows[-1]['tick'] = 9
    elif mutation == 'unexpected_repeat': rows[-1]['repeat'] = 5
    elif mutation == 'false_exact': rows[-1]['exact'] = False
    elif mutation == 'nan': rows[-1]['wall_s'] = math.nan
    elif mutation == 'inf': rows[-1]['cpu_s'] = math.inf
    elif mutation == 'negative': rows[-1]['cpu_s'] = -1.
    elif mutation == 'zero': rows[-1]['wall_s'] = 0.
    elif mutation == 'boolean': rows[-1]['tick'] = True
    elif mutation == 'order': rows[-1], rows[-2] = rows[-2], rows[-1]
    elif mutation == 'total': source['samples'][0]['wall_s'] += 1.
    elif mutation == 'summary': source['before']['wall_s']['mean'] += 1.
    elif mutation == 'decision_tick': decisions[-1]['tick'] = 0
    elif mutation == 'duplicate_sample': source['samples'].append(deepcopy(source['samples'][0]))
    elif mutation == 'source_not_exact': source['all_outputs_exact'] = False
    with pytest.raises(ValueError):
        analyze_case('fixture', rows, decisions, source, 4)


def test_concentration_uses_gross_not_canceled_net():
    pairs = [dict(before=dict(wall_s=10., cpu_s=20.), after=dict(wall_s=10.+d, cpu_s=20.+2*d))
             for d in (1., 1., -1.9)]
    result = describe(pairs)
    assert result['wall_delta_s'] == pytest.approx(.1)
    assert result['gross_positive_wall_delta_s'] == 2.
    assert result['gross_negative_wall_delta_s'] == pytest.approx(-1.9)
    assert result['top_positive_share']['1'] == .5
    assert result['positive_pairs_to_half_gross'] == 1
    assert result['positive_pairs_to_80pct_gross'] == 2
    assert result['wall_cpu_delta_correlation'] == pytest.approx(1.)


def test_equal_pairs_do_not_manufacture_concentration_or_correlation():
    result = describe([dict(before=dict(wall_s=1., cpu_s=2.), after=dict(wall_s=1., cpu_s=2.))])
    assert result['equal_count'] == 1
    assert result['top_positive_share']['1'] is None
    assert result['positive_pairs_to_half_gross'] == 0
    assert result['wall_cpu_delta_correlation'] is None
