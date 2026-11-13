"""Reuse original serial benchmark, binding both original baseline files."""
from pathlib import Path
import sys
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(HERE))
from baseline import load_before
from tools import straight_paired_population_benchmark as bench

if __name__=='__main__':
    before=load_before(HERE/'before/roadvision/paired_descriptor_checks.py')
    bench.write(HERE/'BENCHMARK_BASELINE_BINDING.json',dict(
        original_pair_source=str(Path(before.__file__).resolve().relative_to(ROOT)),
        original_helper_source=str(Path(before.i_evidence.__globals__['__file__']).resolve().relative_to(ROOT)),
        same_original_filter_identities=True,same_h_i_namespaces=True,
        original_benchmark_unmodified=True,scope='Old helper must not accidentally use new raw-mask branch'))
    with patch.object(bench,'load_before',load_before):
        print(bench.run(HERE/'PLAN.json',HERE/'FREEZE.json',(HERE/'FREEZE.sha256').read_text().strip(),HERE/'benchmark'))
