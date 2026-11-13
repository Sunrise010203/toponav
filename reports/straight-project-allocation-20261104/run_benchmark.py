"""One original fixed benchmark with full-controller old-project binding."""
from pathlib import Path
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE))
from baseline import ProjectionBaseline
from tools import straight_paired_population_benchmark as bench


if __name__ == '__main__':
    baseline = ProjectionBaseline(HERE/'before/roadvision/corridor_rgb_probe.py')
    bench.write(HERE/'BENCHMARK_BASELINE_BINDING.json', baseline.record())
    with patch.object(bench, 'load_before', baseline.load_before), \
            patch.object(bench, 'replace_aliases', baseline.replace_aliases):
        print(bench.run(HERE/'PLAN.json', HERE/'FREEZE.json',
                       (HERE/'FREEZE.sha256').read_text().strip(), HERE/'benchmark'))
