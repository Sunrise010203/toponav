"""Old/new NCC aliases for the ENTIRE preloaded controller chain."""
from pathlib import Path

from roadvision import rgb_motion_checks as h, rgb_resampling_checks as i, paired_descriptor_checks as pair
from tools import straight_paired_population_benchmark as bench
from tools.straight_function_benchmark import SymmetricFunctionBinding
from tools.straight_pipeline_profile import pipeline_bindings, discover_bindings

HERE = Path(__file__).resolve().parent


def make_binding():
    pipeline_bindings()
    before = bench.load_before(HERE/'before/roadvision/rgb_motion_checks.py')._ncc
    assert h._ncc is i._ncc
    aliases = discover_bindings([(h._ncc, 'ncc')])
    assert {'roadvision.rgb_motion_checks', 'roadvision.rgb_resampling_checks'}.issubset(
        {owner.__name__ for owner, _, _ in aliases})
    return SymmetricFunctionBinding(before, h._ncc, pair.filter_pair, aliases)
