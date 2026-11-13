"""Exactly one fixed original benchmark with symmetric NCC dispatch."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(HERE))
from ncc_binding import make_binding
from tools import straight_paired_population_benchmark as bench


if __name__ == '__main__':
    binding = make_binding()
    bench.write(HERE/'BENCHMARK_BASELINE_BINDING.json', binding.record())
    with binding.configure_benchmark(bench, HERE/'before/roadvision/rgb_motion_checks.py'):
        result = bench.run(HERE/'PLAN.json', HERE/'FREEZE.json',
                           (HERE/'FREEZE.sha256').read_text().strip(), HERE/'benchmark')
    assert not binding.before.active and not binding.after.active
    assert all(vars(owner)[name] is binding.after.function for owner, name, _ in binding.function_bindings)
    bench.write(HERE/'BENCHMARK_RUNTIME_RESTORED.json', dict(aliases_restored=True,
        dispatchers_inactive=True, benchmark_dispatch_bindings_restored=True))
    print(result)
