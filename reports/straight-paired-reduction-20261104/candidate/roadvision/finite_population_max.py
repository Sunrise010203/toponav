"""Maximum of an already counted, finite population, without changing evidence.

The caller supplies the original boolean finite/support/exclusion mask and its
exact count_nonzero. There is no new filtering, search restriction or cache.
"""
import numpy as np


def finite_population_max(scores, valid, count):
    if not count:
        return None
    if (type(scores) is np.ndarray and scores.ndim == 2 and
            scores.dtype == np.dtype('float32') and type(valid) is np.ndarray and
            valid.dtype == np.dtype('bool') and valid.shape == scores.shape):
        # Selecting a finite, nonzero float32 maximum performs no arithmetic
        # on that value; conversion to Python float is the original conversion.
        maximum = scores.max(where=valid, initial=-np.inf)
        if maximum != 0:
            return float(maximum)
        # NumPy reduction order may choose a different sign among tied zeros.
        # Retain the original gather and reduction for either zero result.
    return float(scores[valid].max())
