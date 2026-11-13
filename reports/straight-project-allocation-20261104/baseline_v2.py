"""Symmetric projection dispatch; alias switching outside controller timers.

Both variants have the same per-pair wrapper. During complete controller
passes the wrapper only forwards; the whole pass has already bound the right
projection. Local cases enter the same context in BOTH variants.
"""
from contextlib import contextmanager
from types import SimpleNamespace

from baseline import ProjectionBaseline, CURRENT_PAIR, ORIGINAL_REPLACE
from tools import straight_paired_population_benchmark as bench

ORIGINAL_DISCOVER = bench.discover_bindings


class Variant:
    def __init__(self, baseline, project):
        self.baseline, self.project, self.active = baseline, project, False

    def __call__(self, *args, **kwargs):
        if self.active:
            return CURRENT_PAIR(*args, **kwargs)
        with ORIGINAL_REPLACE(self.baseline.bindings, self.project):
            return CURRENT_PAIR(*args, **kwargs)


class SymmetricBaseline(ProjectionBaseline):
    def __init__(self, path):
        super().__init__(path)
        self.before = Variant(self, self.old)
        self.after = Variant(self, self.new)

    def load_before(self, path):
        super().load_before(path)  # validate the unchanged source identity
        return SimpleNamespace(filter_pair=self.before)

    def discover(self, targets):
        # Pipeline modules still refer to the actual production pair function.
        # The variant callable is a dispatcher, not a new filter identity.
        return ORIGINAL_DISCOVER([(CURRENT_PAIR if fn is self.after else fn, label)
                                  for fn, label in targets])

    @contextmanager
    def replace_aliases(self, bindings, fn):
        if fn not in (self.before, self.after) or fn.active:
            raise ValueError('unknown_or_reentered_projection_variant')
        with ORIGINAL_REPLACE(bindings, fn), ORIGINAL_REPLACE(self.bindings, fn.project):
            fn.active = True
            try:
                yield
            finally:
                fn.active = False

    def record(self):
        return dict(super().record(), baseline_pair_wrapper_symmetric=True,
                    per_controller_alias_switch_outside_timer=True,
                    nested_per_pair_alias_switch_during_controller=False,
                    local_alias_switch_symmetric=True,
                    before_and_after_wrapper_class='Variant')
