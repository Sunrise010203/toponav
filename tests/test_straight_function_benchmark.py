"""Synthetic symmetric alias dispatch; no performance or navigation run."""
from contextlib import contextmanager
from pathlib import Path
import types

import pytest

from tools import straight_function_benchmark as harness


def fixture():
    before, after = lambda: 'old', lambda: 'new'
    ns = types.SimpleNamespace(fn=after, __name__='test_alias')
    pair = lambda *args, **kwargs: (ns.fn(), args, kwargs)
    binding = harness.SymmetricFunctionBinding(before, after, pair, [(ns, 'fn', 'function')])
    return binding, ns, pair


@pytest.mark.parametrize('label', ['before', 'after'])
@pytest.mark.parametrize('raises', [False, True])
def test_complete_controller_no_nested_alias_context_and_restore(label, raises, monkeypatch):
    b, ns, original_pair = fixture()
    variant = getattr(b, label)
    pair_alias = types.SimpleNamespace(pair=original_pair)
    def called(*args, **kw):
        assert ns.fn is variant.function and variant.active
        if raises: raise RuntimeError('original')
        return 'identity return'
    b.pair_function = called
    with b.controller_context([(pair_alias, 'pair', 'pair')], variant):
        def forbidden(*a): pytest.fail('alias switch inside timer')
        monkeypatch.setattr(harness, 'replace_aliases', forbidden)
        if raises:
            with pytest.raises(RuntimeError, match='original'): pair_alias.pair()
        else: assert pair_alias.pair() == 'identity return'
    assert ns.fn is b.after.function and pair_alias.pair is original_pair and not variant.active


@pytest.mark.parametrize('label', ['before', 'after'])
def test_local_variants_same_code_and_alias_scope(label):
    b, ns, original = fixture()
    variant = getattr(b, label)
    assert variant(3, key=4) == (variant.function(), (3,), dict(key=4))
    assert type(b.before) is type(b.after) is harness.FunctionVariant
    assert type(b.before).__call__ is type(b.after).__call__
    assert ns.fn is b.after.function and not variant.active


@pytest.mark.parametrize('label', ['before', 'after'])
def test_local_exception_and_controller_outer_exception_restore(label):
    b, ns, _ = fixture()
    variant = getattr(b, label)
    def error(*a, **kw): raise RuntimeError('same')
    b.pair_function = error
    with pytest.raises(RuntimeError, match='same'): variant()
    assert ns.fn is b.after.function
    with pytest.raises(RuntimeError, match='outer'):
        with b.controller_context([], variant):
            with pytest.raises(ValueError):
                with b.controller_context([], variant): pass
            raise RuntimeError('outer')
    assert ns.fn is b.after.function and not variant.active


def test_bad_bindings_and_unknown_variants_reject():
    b, ns, pair = fixture()
    with pytest.raises(ValueError):
        harness.SymmetricFunctionBinding(b.before.function, b.after.function, pair, [])
    with pytest.raises(ValueError):
        harness.SymmetricFunctionBinding(b.before.function, b.after.function, pair,
            [(ns, 'fn', 'x'), (ns, 'fn', 'x')])
    with pytest.raises(ValueError):
        harness.SymmetricFunctionBinding(b.before.function, lambda: None, pair, [(ns, 'fn', 'x')])
    with pytest.raises(ValueError):
        with b.controller_context([], object()): pass


@pytest.mark.parametrize('raises', [False, True])
def test_original_benchmark_dispatch_configuration_restored(tmp_path, raises):
    b, ns, _ = fixture()
    sentinel = lambda *a: 'original'
    benchmark = types.SimpleNamespace(load_before=sentinel, replace_aliases=sentinel,
        discover_bindings=sentinel, candidate=object())
    initial = vars(benchmark).copy()
    def execute():
        with b.configure_benchmark(benchmark, tmp_path/'before.py'):
            assert benchmark.load_before(tmp_path/'before.py').filter_pair is b.before
            with pytest.raises(ValueError): benchmark.load_before(tmp_path/'wrong.py')
            assert benchmark.candidate.filter_pair is b.after
            if raises: raise RuntimeError('body')
    if raises:
        with pytest.raises(RuntimeError, match='body'): execute()
    else: execute()
    assert all(vars(benchmark)[k] is v for k, v in initial.items())


def test_real_pair_and_ncc_alias_identities_are_bound_without_fallback():
    from roadvision import paired_descriptor_checks as pair, rgb_motion_checks as h, rgb_resampling_checks as i
    from tools.straight_pipeline_profile import discover_bindings, pipeline_bindings
    pipeline_bindings()
    bindings = discover_bindings([(h._ncc, 'ncc')])
    b = harness.SymmetricFunctionBinding(lambda *a: None, h._ncc, pair.filter_pair, bindings)
    pairs = b.discover_pairs([(b.after, 'pair')])
    assert {'roadvision.ground_history', 'roadvision.straight_history_support'}.issubset(
        {owner.__name__ for owner, _, _ in pairs})
    with b.controller_context(pairs, b.before):
        assert h._ncc is i._ncc is b.before.function
    assert h._ncc is i._ncc is b.after.function
