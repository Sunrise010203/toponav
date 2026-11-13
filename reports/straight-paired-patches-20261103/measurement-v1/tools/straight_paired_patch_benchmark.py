"""Run the existing serial AB/BA harness with a full pre-hook H/I baseline."""
import argparse
from contextlib import contextmanager
from pathlib import Path
from types import FunctionType

from tools import straight_paired_population_benchmark as harness


# Keep the original loader even while the harness is temporarily bound below.
_load = harness.load_before


def load_baseline(path):
    """Use frozen old evidence bodies too, not new default-hook overhead."""
    pair = _load(path)
    h = _load(path.parent / 'rgb_motion_checks.py')
    i = _load(path.parent / 'rgb_resampling_checks.py')
    # Rebind I's original H imports to the same old module, without globals edits.
    from roadvision import rgb_motion_checks as current_h
    for name, value in list(vars(i).items()):
        if name in vars(h) and vars(current_h).get(name) is value:
            setattr(i, name, vars(h)[name])
    pair.h, pair.i = h, i
    from roadvision.prepared_descriptor_checks import filter_h, filter_i
    def clone(fn, region_fn):
        return FunctionType(fn.__code__, {**fn.__globals__, '_filter_h': region_fn,
                            '_filter_i': region_fn}, fn.__name__, fn.__defaults__)
    pair.filter_h = clone(filter_h, h.filter_region)
    pair.filter_i = clone(filter_i, i.filter_region)
    pair.filter_pair.__kwdefaults__ = dict(filters=(pair.filter_h, pair.filter_i))
    return pair


@contextmanager
def baseline_loader():
    previous = harness.load_before
    try:
        harness.load_before = load_baseline
        yield
    finally:
        harness.load_before = previous


def run(plan, freeze, freeze_sha256, output):
    with baseline_loader():
        return harness.run(plan, freeze, freeze_sha256, output)


if __name__ == '__main__':
    import json
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan', type=Path, required=True)
    p.add_argument('--freeze', type=Path, required=True)
    p.add_argument('--freeze-sha256', required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    print(json.dumps(run(args.plan,args.freeze,args.freeze_sha256,args.output)))
