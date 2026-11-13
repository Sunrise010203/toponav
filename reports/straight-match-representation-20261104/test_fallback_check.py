import importlib.util
from pathlib import Path

import cv2
import pytest

spec=importlib.util.spec_from_file_location('_fallback_representation_check',Path(__file__).with_name('fallback_check.py'))
check=importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


@pytest.mark.parametrize('initial',[False,True])
@pytest.mark.parametrize('raises',[False,True])
def test_backend_restores_prior_state_on_success_or_exception(initial,raises):
    original_ipp,original_threads=cv2.ipp.useIPP(),cv2.getNumThreads()
    try:
        cv2.ipp.setUseIPP(initial)
        before=cv2.ipp.useIPP()
        try:
            with check.generic_backend():
                assert cv2.getNumThreads()==1 and not cv2.ipp.useIPP()
                if raises:raise RuntimeError('injected')
        except RuntimeError:
            assert raises
        assert cv2.ipp.useIPP()==before and cv2.getNumThreads()==original_threads
    finally:
        cv2.ipp.setUseIPP(original_ipp)
        cv2.setNumThreads(original_threads)
