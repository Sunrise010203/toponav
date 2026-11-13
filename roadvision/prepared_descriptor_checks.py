"""S2/history-only equivalent descriptor implementation selection.

Frozen S1 entrypoints keep their original default path. No OR, fallback,
track retry, score cache or new evidence/acceptance variant is introduced.
"""
from roadvision.rgb_motion_checks import filter_region as _filter_h
from roadvision.rgb_resampling_checks import filter_region as _filter_i


def filter_h(images, region):
    return _filter_h(images, region, _prepared_search=True)


def filter_i(images, region):
    return _filter_i(images, region, _prepared_search=True)
