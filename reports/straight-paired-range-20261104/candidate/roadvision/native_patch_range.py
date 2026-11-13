"""Sufficient original-contrast decisions, not different acceptance gates.

For N=121 values with extrema m,M and range R: V >= R**2/(2*N),
because the two extremal squared deviations sum to at least R**2/2.
Also V <= R**2/4. Thus R>=47 implies V>9 and R<=1 implies V<1.
These lie in the existing native contrast proof's conservative far bands.
No integral/moment precision or approximate standard deviation is assumed.
"""
import cv2
import numpy as np


def sufficient_range_contrast(raw):
    if type(raw) is not np.ndarray or raw.dtype != np.uint8 or raw.shape != (11, 11):
        return None
    low, high, _, _ = cv2.minMaxLoc(raw)
    span = int(high)-int(low)  # uint8 extrema are exact values, not reductions of moments.
    if span >= 47:
        return True
    if span <= 1:
        return False
    return None
