import math
from collections.abc import Mapping
from numbers import Real

DIRECTIONS = ("front", "left", "right", "back")
CLASS_IDS = tuple(f"exits_{i:04b}" for i in range(16)) + ("unobservable",)


def _finite_real(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        return False
    try:
        return math.isfinite(value)
    except (TypeError, ValueError, OverflowError):
        return False


def to_observation(probabilities, threshold=0.6, valid=True, age_s=0.0):
    if isinstance(probabilities, (str, bytes, bytearray, memoryview, Mapping)):
        raise ValueError("Expected an ordered vector of 17 class probabilities")
    try:
        if len(probabilities) != 17:
            raise ValueError
        probabilities = tuple(probabilities[i] for i in range(17))
    except (TypeError, ValueError, KeyError, IndexError, AttributeError, OverflowError):
        raise ValueError("Expected an ordered vector of 17 class probabilities") from None
    if any(not _finite_real(p) or not 0 <= p <= 1 for p in probabilities):
        raise ValueError("Expected 17 finite real class probabilities between zero and one")
    if not math.isclose(sum(probabilities), 1.0, abs_tol=1e-5):
        raise ValueError("Class probabilities must sum to one")
    if not _finite_real(threshold) or not 0.6 <= threshold <= 1.0:
        raise ValueError("Threshold cannot be below the third block minimum of 0.6")
    index = max(range(17), key=probabilities.__getitem__)
    age_valid = (type(age_s) in (int, float) and _finite_real(age_s)
                 and 0 <= age_s <= 1.0)
    if valid is not True or not age_valid or index == 16 or probabilities[index] < threshold:
        return {"relative_open": dict.fromkeys(DIRECTIONS), "confidence": 0.0}
    return {"relative_open": dict(zip(DIRECTIONS, (bit == "1" for bit in f"{index:04b}"))),
            "confidence": float(probabilities[index])}
