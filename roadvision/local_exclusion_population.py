"""Exact exclusion geometry, never a restriction of descriptor competition.

The full bitmap is retained. Only distance arithmetic proven unnecessary far
from a finite, bounded endpoint is omitted. Scores, support, finite filtering,
count/max, roles and acceptance remain the caller's original operations.
Static prepared grids must stay unchanged for this call-local object's life.
"""
import math

import numpy as np


class LocalExclusion:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.regular = False
        if (x.ndim != 2 or x.shape != y.shape or not x.size or
                x.dtype.kind not in 'iu' or y.dtype.kind not in 'iu'):
            return
        height, width = x.shape
        self.origin_x, self.origin_y = int(x[0, 0]), int(y[0, 0])
        # Bounds make validation and the fast-path proof free of overflow.
        if not (-1024 <= self.origin_x <= self.origin_x+width-1 <= 1024 and
                -1024 <= self.origin_y <= self.origin_y+height-1 <= 1024):
            return
        self.regular = bool(
            np.all(x == (np.arange(width)+self.origin_x)[None, :]) and
            np.all(y == (np.arange(height)+self.origin_y)[:, None]))

    def _window(self, px, py):
        height, width = self.x.shape
        # +/-12 conservatively surrounds the radius-11 disk, including any
        # fractional endpoint and exact/nextafter boundary. Extra cells use
        # the original arithmetic too; they are not accepted geometrically.
        cx, cy = math.floor(px)-self.origin_x, math.floor(py)-self.origin_y
        return (slice(min(height, max(0, cy-12)), min(height, max(0, cy+13))),
                slice(min(width, max(0, cx-12)), min(width, max(0, cx+13))))

    def outside(self, expected, radius_squared):
        point = np.asarray(expected)
        if (not self.regular or point.shape != (2,) or point.dtype != np.dtype('float64') or
                type(radius_squared) not in (float, int, np.float64, np.int64) or
                radius_squared != 121. or not -128. <= point[0] <= 256. or
                not -128. <= point[1] <= 256.):
            # Includes nonfinite, wide/unusual dtypes, nonregular grids and
            # changed radii; preserve the original operation and error path.
            return ((self.x-expected[0])**2+(self.y-expected[1])**2 > radius_squared)
        window = self._window(point[0], point[1])
        outside = np.ones(self.x.shape, dtype=bool)
        # Same operands, dtype and subtract/square/add/compare ordering as
        # the full expression. Every other grid center is certainly outside.
        outside[window] = ((self.x[window]-expected[0])**2+
                           (self.y[window]-expected[1])**2 > radius_squared)
        return outside
