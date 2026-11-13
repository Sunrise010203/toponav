"""Same fixed synthetic corpus with IPP disabled, never a production setting."""
from contextlib import contextmanager
import importlib.util
import json
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('_original_representation_check', HERE/'check.py')
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


@contextmanager
def generic_backend():
    threads, ipp = cv2.getNumThreads(), cv2.ipp.useIPP()
    try:
        cv2.setNumThreads(1)
        cv2.ipp.setUseIPP(False)
        assert not cv2.ipp.useIPP()
        yield
    finally:
        cv2.ipp.setUseIPP(ipp)
        cv2.setNumThreads(threads)
    assert cv2.getNumThreads() == threads and cv2.ipp.useIPP() == ipp


def run():
    check.verify_pins(HERE/'FALLBACK_FREEZE.json')
    plan = json.loads((HERE/'PLAN.json').read_bytes())
    output = HERE/'fallback-numerical'
    output.mkdir(exist_ok=False)
    observer = check.Comparison(output)
    original, threads, ipp = cv2.matchTemplate, cv2.getNumThreads(), cv2.ipp.useIPP()
    with generic_backend():
        for name, target, template in check.synthetics(plan):
            observer.compare(original, 'synthetic/'+name, target, template, cv2.TM_CCOEFF_NORMED)
    assert cv2.matchTemplate is original and cv2.getNumThreads() == threads and cv2.ipp.useIPP() == ipp
    assert len(observer.records) == len(plan['synthetic_cases'])
    check.verify_pins(HERE/'FALLBACK_FREEZE.json')
    with (output/'rows.jsonl').open('x') as f:
        for row in observer.records:
            f.write(json.dumps(row, allow_nan=False)+'\n')
    mismatch = [r for r in observer.records if not r['comparison']['bit_exact']]
    result = dict(scope='Source-directed generic backend audit only; no production configuration change',
        records=len(observer.records), eligible_maps=sum(r['eligible'] for r in observer.records),
        different_maps=len(mismatch), different_elements=sum(r['comparison']['different_elements'] for r in mismatch),
        compared_elements=sum(r['comparison']['elements'] for r in observer.records),
        unconditional_dtype_substitution_rejected=bool(mismatch),
        ipp_enabled_during_check=False, ipp_restored=True, threads_restored=True,
        match_alias_unchanged=True, freeze_unchanged=True, backend_configuration_deployed=False,
        ordinary_timing_runs=0,controller_sequences=0,physical_execution=False)
    check.write(output/'SUMMARY.json',result)
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    run()
