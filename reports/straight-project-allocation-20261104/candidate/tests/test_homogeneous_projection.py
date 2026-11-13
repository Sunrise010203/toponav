"""Bit-exact projection allocation; candidate always explicitly archived."""
import importlib.util
from pathlib import Path
import types

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT/'reports/straight-project-allocation-20261104'


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


old = load(HERE/'before/roadvision/corridor_rgb_probe.py', '_project_old')
candidate = load(HERE/'candidate/roadvision/corridor_rgb_probe.py', '_project_candidate')


def assert_exact(a, b):
    if a is None:
        assert b is None
    else:
        assert b is not None
        assert a.dtype == b.dtype and a.shape == b.shape and a.strides == b.strides
        assert a.tobytes() == b.tobytes()


@pytest.mark.parametrize('n', [0, 1, 2, 8, 121, 1000])
@pytest.mark.parametrize('kind', ['identity', 'random', 'perspective', 'denominator_boundary'])
def test_exact_output_shapes_products_and_validity(n, kind):
    rng = np.random.default_rng(1441104)
    points = rng.normal(64, 30, (n, 2))
    matrix = np.eye(3)
    if kind == 'random':
        matrix = rng.normal(size=(3, 3))
    if kind == 'perspective':
        matrix = np.array([[1.2, -.03, 4.], [.01, .9, -2.], [.001, -.002, 1.]])
    if kind == 'denominator_boundary':
        matrix[2] = [0., 0., np.nextafter(1e-9, 0.)]
    original_points, original_matrix = points.tobytes(), matrix.tobytes()
    assert_exact(old._project(matrix, points), candidate._project(matrix, points))
    assert points.tobytes() == original_points and matrix.tobytes() == original_matrix


@pytest.mark.parametrize('layout', ['C', 'F', 'reverse', 'strided', 'f32', 'integer', 'list', 'subclass'])
def test_operand_bytes_layout_and_dispatch(layout, monkeypatch):
    p = np.array([[0., -0.], [np.nan, np.inf], [-np.inf, np.nextafter(0., 1.)]])
    if layout == 'F': p = np.asfortranarray(p)
    if layout == 'reverse': p = p[::-1]
    if layout == 'strided':
        data = np.zeros((6, 4)); data[::2, ::2] = p; p = data[::2, ::2]
    if layout == 'f32': p = p.astype(np.float32)
    if layout == 'integer': p = np.array([[1, 2], [3, 4]], np.int32)
    if layout == 'list': p = [[1., -0.], [2., 3.]]
    if layout == 'subclass':
        class Sub(np.ndarray): pass
        p = p.view(Sub)
    operands = []
    class Transposed:
        __array_priority__ = 10000
        def __rmatmul__(self, a):
            operands.append((a.dtype.str, a.shape, a.strides, a.tobytes()))
            return a.copy()
    matrix = types.SimpleNamespace(T=Transposed())
    concat = np.column_stack
    calls = []
    def observed(*a, **kw): calls.append(1); return concat(*a, **kw)
    monkeypatch.setattr(np, 'column_stack', observed)
    with np.errstate(invalid='ignore'):
        expected = old._project(matrix, p)
        got = candidate._project(matrix, p)
    assert operands[0] == operands[1]
    assert_exact(expected, got)
    assert len(calls) == (1 if layout == 'C' else 2)


@pytest.mark.parametrize('exponent', [-300, -100, -10, 0, 10, 100, 300])
def test_extreme_finite_and_nonfinite_results(exponent):
    rng = np.random.default_rng(441)
    for n in (1, 2, 17):
        points = rng.normal(size=(n, 2))*10.**exponent
        matrix = rng.normal(size=(3, 3))*10.**exponent
        with np.errstate(all='ignore'):
            assert_exact(old._project(matrix, points), candidate._project(matrix, points))


@pytest.mark.parametrize('p', [np.zeros((3,)), np.zeros((2, 1)), np.zeros((2, 4)),
                              np.zeros((1, 2, 2)), np.array([[1+2j, 3+0j]])])
def test_unsupported_shape_or_dtype_keeps_result_or_exception(p):
    try:
        expected = old._project(np.eye(3), p)
    except Exception as exc:
        with pytest.raises(type(exc)) as actual:
            candidate._project(np.eye(3), p)
        assert str(actual.value) == str(exc)
    else:
        assert_exact(expected, candidate._project(np.eye(3), p))


def test_nan_payload_and_signed_zero_operand_bits():
    bits = np.array([0x8000000000000000, 0x7ff8000000000013,
                     0x7ff0000000000000, 0xfff0000000000000], np.uint64).reshape(2, 2)
    p = bits.view(np.float64)
    seen = []
    class Product:
        __array_priority__ = 10000
        def __rmatmul__(self, operand): seen.append(operand.tobytes()); return operand.copy()
    with np.errstate(invalid='ignore'):
        old._project(types.SimpleNamespace(T=Product()), p)
        candidate._project(types.SimpleNamespace(T=Product()), p)
    assert seen[0] == seen[1]


def test_old_projection_context_covers_nonpair_aliases_and_restores_on_error():
    baseline = load(HERE/'baseline.py', '_allocation_baseline')
    bound = baseline.ProjectionBaseline(HERE/'before/roadvision/corridor_rgb_probe.py')
    from roadvision import corridor_rgb_probe as p, rgb_motion_checks as h
    original = p._project
    with pytest.raises(RuntimeError, match='original exception'):
        with bound.replace_aliases([], bound.local_before):
            assert p._project is h._project is bound.old
            raise RuntimeError('original exception')
    assert p._project is h._project is original
    assert all(vars(owner)[name] is bound.new for owner, name, _ in bound.bindings)
