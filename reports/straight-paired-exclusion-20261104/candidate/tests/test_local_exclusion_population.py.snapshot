"""The entire original exclusion bitmap must be equal, not just a sample."""
import numpy as np
import pytest

from roadvision.local_exclusion_population import LocalExclusion


def grids(shape=(118, 118), origin=(5, 5)):
    y, x = np.indices(shape)
    return x+origin[0], y+origin[1]


def original(x, y, point, radius=121.):
    return (x-point[0])**2+(y-point[1])**2 > radius


@pytest.mark.parametrize('shape,origin', [((118, 118), (5, 5)), ((31, 79), (-25, 11)),
    ((1, 1), (0, 0)), ((2, 17), (1000, -100)), ((17, 2), (-100, 1000))])
def test_whole_map_exact_random_and_fractional_boundaries(shape, origin):
    x, y = grids(shape, origin)
    before_x, before_y = x.copy(), y.copy()
    geometry = LocalExclusion(x, y)
    assert geometry.regular
    rng = np.random.default_rng(110404)
    points = list(rng.uniform(-128, 256, (400, 2)))
    for centre in ([5., 5.], [64., 64.], [122., 122.], [-128., 256.], [0., -0.]):
        for offset in (-11., 0., 11.):
            for toward in (-np.inf, np.inf):
                p = np.array(centre)
                p[0] = np.nextafter(p[0]+offset, toward)
                points.append(p)
    for point in points:
        np.testing.assert_array_equal(geometry.outside(point, 121.), original(x, y, point))
    np.testing.assert_array_equal(x, before_x)
    np.testing.assert_array_equal(y, before_y)


def test_exact_disk_boundary_and_conservative_window_still_evaluated():
    x, y = grids()
    geometry = LocalExclusion(x, y)
    point = np.array([64., 64.])
    outside = geometry.outside(point, 121.)
    assert not outside[64-5, 75-5]  # exactly 11px is NOT outside
    assert outside[64-5, 76-5]
    window = geometry._window(*point)
    assert x[window].shape == (25, 25)
    assert x[window].size == 625 < x.size
    changed = geometry.outside(np.nextafter(point, np.inf), 121.)
    np.testing.assert_array_equal(changed, original(x, y, np.nextafter(point, np.inf)))


@pytest.mark.parametrize('point', [
    [np.nan, 64.], [64., np.inf], [-np.inf, 64.], [257., 64.], [-129., 64.],
    np.array([64., 64.], np.float32), np.array([64., 64.], np.float16),
    np.array([64., 64.], np.longdouble), [64, 64], np.array([64., 64.], dtype=object),
    np.array([64., 64.], dtype='>f8'),
])
def test_unsupported_endpoints_use_original_full_expression(point, monkeypatch):
    x, y = grids()
    geometry = LocalExclusion(x, y)
    def forbidden(*args): raise AssertionError('not eligible for window')
    monkeypatch.setattr(geometry, '_window', forbidden)
    np.testing.assert_array_equal(geometry.outside(point, 121.), original(x, y, point))


@pytest.mark.parametrize('radius', [0., -1., 100., 121.01, np.inf, np.nan,
                                   np.float32(121.), np.array([121.])])
def test_other_radius_or_dtype_is_original(radius, monkeypatch):
    x, y = grids()
    geometry = LocalExclusion(x, y)
    monkeypatch.setattr(geometry, '_window', lambda *a: pytest.fail('fast radius'))
    point = np.array([64., 64.])
    np.testing.assert_array_equal(geometry.outside(point, radius), original(x, y, point, radius))


@pytest.mark.parametrize('kind', ['shift_cell', 'reverse', 'float_grid', 'large_origin', 'empty', 'broadcast'])
def test_unusual_grids_fall_back(kind, monkeypatch):
    x, y = grids()
    if kind == 'shift_cell': x[0, 0] += 1
    elif kind == 'reverse': x = x[:, ::-1]
    elif kind == 'float_grid': x = x.astype(float); y = y.astype(float)
    elif kind == 'large_origin': x += 100000
    elif kind == 'empty': x, y = grids((0, 0))
    elif kind == 'broadcast': x, y = x[:1], y[:, :1]
    geometry = LocalExclusion(x, y)
    assert not geometry.regular
    monkeypatch.setattr(geometry, '_window', lambda *a: pytest.fail('nonregular fast path'))
    p = np.array([64., 64.])
    np.testing.assert_array_equal(geometry.outside(p, 121.), original(x, y, p))


def test_maps_are_independent_and_have_no_cross_endpoint_stale_bits():
    x, y = grids()
    geometry = LocalExclusion(x, y)
    a = geometry.outside([64., 64.], 121.)
    a[:] = False
    for p in ([5., 5.], [120., 120.], [64., 64.], [256., 256.]):
        np.testing.assert_array_equal(geometry.outside(p, 121.), original(x, y, p))


@pytest.mark.parametrize('point', [[64.], [], ['bad', 'point']])
def test_malformed_endpoint_exception_type_is_not_repaired(point):
    x, y = grids()
    with pytest.raises(Exception) as expected:
        original(x, y, point)
    with pytest.raises(type(expected.value)):
        LocalExclusion(x, y).outside(point, 121.)
