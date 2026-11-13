"""Synthetic fairness and restoration checks; never a performance comparison."""
import importlib.util
from pathlib import Path
import sys
import types

import pytest

HERE = Path(__file__).resolve().parents[1]/'reports/straight-project-allocation-20261104'
sys.path.insert(0, str(HERE))
from baseline_v2 import SymmetricBaseline, Variant
import baseline_v2 as binding


@pytest.mark.parametrize('which', ['before', 'after'])
@pytest.mark.parametrize('raises', [False, True])
def test_controller_variant_has_no_nested_alias_switch(which, raises, monkeypatch):
    b = SymmetricBaseline(HERE/'before/roadvision/corridor_rgb_probe.py')
    fn = getattr(b, which)
    project = fn.project
    from roadvision import corridor_rgb_probe as core
    initial = core._project
    calls = []
    def pair(*args, **kw):
        assert core._project is project
        assert fn.active
        calls.append((args, kw))
        if raises: raise RuntimeError('forwarded')
        return 'same return'
    monkeypatch.setattr(binding, 'CURRENT_PAIR', pair)
    ns = types.SimpleNamespace(pair=pair)
    original_context = binding.ORIGINAL_REPLACE
    with b.replace_aliases([(ns, 'pair', 'pair')], fn):
        def forbidden(*args): pytest.fail('context entered inside controller timer')
        monkeypatch.setattr(binding, 'ORIGINAL_REPLACE', forbidden)
        if raises:
            with pytest.raises(RuntimeError, match='forwarded'): ns.pair(1, k=2)
        else:
            assert ns.pair(1, k=2) == 'same return'
        monkeypatch.setattr(binding, 'ORIGINAL_REPLACE', original_context)
    assert calls == [((1,), dict(k=2))]
    assert not fn.active and core._project is initial and ns.pair is pair


@pytest.mark.parametrize('which', ['before', 'after'])
@pytest.mark.parametrize('raises', [False, True])
def test_local_switch_is_symmetric_and_restores(which, raises, monkeypatch):
    b = SymmetricBaseline(HERE/'before/roadvision/corridor_rgb_probe.py')
    fn = getattr(b, which)
    from roadvision import corridor_rgb_probe as core
    initial = core._project
    def pair(*args, **kw):
        assert core._project is fn.project and not fn.active
        if raises: raise RuntimeError('local forwarded')
        return 'identical'
    monkeypatch.setattr(binding, 'CURRENT_PAIR', pair)
    if raises:
        with pytest.raises(RuntimeError, match='local forwarded'): fn()
    else:
        assert fn() == 'identical'
    assert core._project is initial and not fn.active
    assert type(b.before) is type(b.after) is Variant
    assert type(b.before).__call__ is type(b.after).__call__


def test_alias_discovery_uses_actual_production_pair_identity():
    b = SymmetricBaseline(HERE/'before/roadvision/corridor_rgb_probe.py')
    aliases = b.discover([(b.after, 'pair')])
    assert {'roadvision.ground_history', 'roadvision.straight_history_support'}.issubset(
        {owner.__name__ for owner, _, _ in aliases})
    assert all(vars(owner)[name] is binding.CURRENT_PAIR for owner, name, _ in aliases)


@pytest.mark.parametrize('which', ['before', 'after'])
def test_context_exception_and_reentry_reset_both_types(which):
    b = SymmetricBaseline(HERE/'before/roadvision/corridor_rgb_probe.py')
    fn = getattr(b, which)
    with pytest.raises(RuntimeError, match='outer'):
        with b.replace_aliases([], fn):
            with pytest.raises(ValueError):
                with b.replace_aliases([], fn): pass
            raise RuntimeError('outer')
    assert not b.before.active and not b.after.active
    assert all(vars(owner)[name] is b.new for owner, name, _ in b.bindings)
