"""Track-local raw appearance preparation, never a shared acceptance decision.

Only pure patch/NCC computations are reused, with exact canvas/function/point
identity checks. Canvases and returned patches are read-only to their callers.
I's supported-patch mask sampling is deliberately NOT part of this context.
"""
import numpy as np


class TrackAppearance:
    """Region-owned static canvases; bounded caches cleared after EVERY track."""
    def __init__(self, canvases):
        unique = []
        self.canvases = {}
        for gray in canvases:
            canonical = next((index for index, other in enumerate(unique)
                              if gray.dtype == other.dtype and gray.shape == other.shape
                              and np.array_equal(gray, other)), None)
            if canonical is None:
                canonical = len(unique)
                unique.append(gray)
            self.canvases[id(gray)] = (gray, canonical)
        self.patches = {}
        self.correlations = []

    def patch(self, fn, gray, point):
        canvas = self.canvases.get(id(gray))
        if canvas is None or canvas[0] is not gray:
            return fn(gray, point)
        p = np.asarray(point)
        # Malformed/non-numeric points retain the original function's errors.
        if p.shape != (2,) or p.dtype.kind not in 'fiu':
            return fn(gray, point)
        key = (id(fn), canvas[1], p.dtype.str, p.shape, p.tobytes())
        entry = self.patches.get(key)
        if entry is not None and entry[0] is fn:
            return entry[1]
        patch = fn(gray, point)
        if len(self.patches) == 4:
            del self.patches[next(iter(self.patches))]
        self.patches[key] = (fn, patch)  # None is also an exact raw-patch result
        return patch

    def ncc(self, fn, first, second):
        # Ordered identities are sufficient: these are our immutable raw patches.
        for function, a, b, result in self.correlations:
            if function is fn and a is first and b is second:
                return result
        result = fn(first, second)
        if len(self.correlations) == 2:
            del self.correlations[0]
        self.correlations.append((fn, first, second, result))
        return result

    def clear(self):
        self.patches.clear()
        self.correlations.clear()
