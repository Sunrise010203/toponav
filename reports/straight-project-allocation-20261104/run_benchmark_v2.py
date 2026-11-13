"""One corrected comparison; first comparison retained, not pooled."""
from pathlib import Path
from types import SimpleNamespace
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(HERE))
from baseline_v2 import SymmetricBaseline
from tools import straight_paired_population_benchmark as bench


if __name__ == '__main__':
    baseline = SymmetricBaseline(HERE/'before/roadvision/corridor_rgb_probe.py')
    bench.write(HERE/'BENCHMARK_BASELINE_BINDING-v2.json', baseline.record())
    with patch.object(bench, 'load_before', baseline.load_before), \
            patch.object(bench, 'replace_aliases', baseline.replace_aliases), \
            patch.object(bench, 'discover_bindings', baseline.discover), \
            patch.object(bench, 'candidate', SimpleNamespace(filter_pair=baseline.after)):
        print(bench.run(HERE/'PLAN-v2.json', HERE/'FREEZE-v2.json',
                       (HERE/'FREEZE-v2.sha256').read_text().strip(), HERE/'benchmark-corrected'))
