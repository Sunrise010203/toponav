"""Offline benchmark aliases, fixtures and freeze checks remain explicit."""
from types import SimpleNamespace

import pytest

from roadvision.prepared_descriptor_checks import filter_h, filter_i
from tools import straight_paired_population_benchmark as bench


@pytest.mark.parametrize('raises', [False, True])
def test_aliases_restored_on_success_and_exception(raises):
    def original(): pass
    def replacement(): pass
    a=SimpleNamespace(filter_pair=original)
    b=SimpleNamespace(alias=original)
    bindings=[(a,'filter_pair','paired'),(b,'alias','paired')]
    def run():
        with bench.replace_aliases(bindings,replacement):
            assert a.filter_pair is replacement and b.alias is replacement
            if raises: raise RuntimeError('stop')
    if raises:
        with pytest.raises(RuntimeError,match='stop'): run()
    else:
        run()
    assert a.filter_pair is original and b.alias is original


@pytest.mark.parametrize('index', range(6))
def test_benchmark_local_fixtures_have_complete_independent_outputs(index):
    name,images,region=bench.local_cases()[index]
    expected=dict(H=filter_h(images,region),I=filter_i(images,region))
    assert bench.encode(bench.candidate.filter_pair(images,region)) == bench.encode(expected)
    assert name == bench.local_cases()[index][0]


def test_summary_preserves_regression_instead_of_clipping():
    samples=[dict(implementation=label,wall_s=value,cpu_s=value) for label,value in
             [('before',1.),('after',2.),('before',3.),('after',4.)]]
    summary=bench.summarize(samples)
    assert summary['before']['wall_s']['median'] == 2.
    assert summary['after']['wall_s']['median'] == 3.
    assert summary['wall_median_reduction_fraction'] == -.5
    assert summary['wall_mean_reduction_fraction'] == -.5


def test_bad_freeze_digest_fails_before_loading_inputs(tmp_path):
    freeze=tmp_path/'freeze.json'; freeze.write_text('{}')
    with pytest.raises(ValueError,match='freeze_digest'):
        bench.run(tmp_path/'missing-plan.json',freeze,'bad',tmp_path/'output')
    assert not (tmp_path/'output').exists()
