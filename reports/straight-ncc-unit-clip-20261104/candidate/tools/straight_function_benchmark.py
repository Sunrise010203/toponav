"""Symmetric offline function-alias binding for the existing full benchmark.

Not imported by production. Both variants execute the exact same dispatcher;
complete-pass alias switches happen outside controller timers. Local cases
use the same context code in both variants. No instrumentation of math calls.
"""
from contextlib import contextmanager
from types import SimpleNamespace

from tools.straight_paired_population_benchmark import replace_aliases
from tools.straight_pipeline_profile import discover_bindings


class FunctionVariant:
    def __init__(self, binding, function):
        self.binding, self.function, self.active = binding, function, False

    def __call__(self, *args, **kwargs):
        if self.active:
            return self.binding.pair_function(*args, **kwargs)
        with replace_aliases(self.binding.function_bindings, self.function):
            return self.binding.pair_function(*args, **kwargs)


class SymmetricFunctionBinding:
    def __init__(self, before, after, pair_function, function_bindings):
        self.before = FunctionVariant(self, before)
        self.after = FunctionVariant(self, after)
        self.pair_function = pair_function
        self.function_bindings = list(function_bindings)
        keys = [(id(owner), name) for owner, name, _ in self.function_bindings]
        if len(keys) != len(set(keys)) or not keys:
            raise ValueError('empty_or_duplicate_function_binding')
        if any(vars(owner)[name] is not after for owner, name, _ in self.function_bindings):
            raise ValueError('function_alias_identity_mismatch')

    def discover_pairs(self, targets):
        return discover_bindings([(self.pair_function if fn is self.after else fn, label)
                                  for fn, label in targets])

    @contextmanager
    def controller_context(self, pair_bindings, variant):
        if variant not in (self.before, self.after) or variant.active:
            raise ValueError('unknown_or_reentered_function_variant')
        with replace_aliases(pair_bindings, variant), \
                replace_aliases(self.function_bindings, variant.function):
            variant.active = True
            try:
                yield
            finally:
                variant.active = False

    @contextmanager
    def configure_benchmark(self, benchmark, before_path):
        from pathlib import Path
        from unittest.mock import patch
        before_path = Path(before_path).resolve()
        def load_before(path):
            if Path(path).resolve() != before_path:
                raise ValueError('unexpected_baseline_source')
            return SimpleNamespace(filter_pair=self.before)
        with patch.object(benchmark, 'load_before', load_before), \
                patch.object(benchmark, 'replace_aliases', self.controller_context), \
                patch.object(benchmark, 'discover_bindings', self.discover_pairs), \
                patch.object(benchmark, 'candidate', SimpleNamespace(filter_pair=self.after)):
            yield

    def record(self):
        return dict(aliases=[owner.__name__+'.'+name for owner, name, _ in self.function_bindings],
                    baseline_pair_wrapper_symmetric=True, before_and_after_wrapper_class='FunctionVariant',
                    per_controller_alias_switch_outside_timer=True,
                    nested_per_pair_alias_switch_during_controller=False,
                    local_alias_switch_symmetric=True, actual_pair_implementation_unchanged=True,
                    original_benchmark_source_unmodified=True)
