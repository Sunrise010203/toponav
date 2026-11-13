"""Original loader plus explicit old helper binding for a two-file baseline."""
from pathlib import Path
from tools import straight_paired_population_benchmark as bench
from roadvision import paired_descriptor_checks as current, native_patch_contrast as fast

HERE=Path(__file__).resolve().parent
ORIGINAL_LOAD_BEFORE=bench.load_before


def load_before(path):
    before=ORIGINAL_LOAD_BEFORE(path)
    helper=ORIGINAL_LOAD_BEFORE(HERE/'before/roadvision/native_patch_contrast.py')
    assert before.h is current.h is helper.h is fast.h
    assert before.i is current.i is helper.i is fast.i
    assert before.filter_h is current.filter_h and before.filter_i is current.filter_i
    before.h_evidence, before.i_evidence=helper.h_evidence,helper.i_evidence
    assert before.h_evidence.__globals__['__file__']==str(HERE/'before/roadvision/native_patch_contrast.py')
    assert before.i_evidence.__globals__ is before.h_evidence.__globals__
    assert before.i_evidence is not current.i_evidence
    return before
