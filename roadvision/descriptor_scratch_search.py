"""Call-local descriptor search preparation, not a new appearance policy.

Static canvas preparation and scratch storage are shared. Every template
still runs the original full-canvas matchTemplate, overwriting its output.
The object must be created from local warped canvases for ONE filter call;
it is never stored on a controller, returned as evidence or reused by frames.
"""
import cv2
import numpy as np


class PreparedAlternative:
    def __init__(self, target, *, patch_px, half, exclude_radius, mask=None):
        self.target = target.astype(np.float32)
        shape = (target.shape[0] - patch_px + 1, target.shape[1] - patch_px + 1)
        # Separability changes storage/work, not per-pixel arithmetic order.
        self.x = np.arange(shape[1])[None, :] + half
        self.y = np.arange(shape[0])[:, None] + half
        # Synchronous call-local scratch, NOT a cached score or validity map.
        self._scores = np.empty(shape, np.float32)
        self._valid = np.empty(shape, bool)
        self.radius_squared = exclude_radius ** 2
        self.support = None
        if mask is not None:
            eroded = cv2.erode(mask, np.ones((patch_px, patch_px), np.uint8),
                               borderType=cv2.BORDER_CONSTANT, borderValue=0)
            self.support = eroded[half:-half, half:-half] > 0

    def at(self, expected):
        """Share the same exclusion footprint for two logical searches.

        Do not change arithmetic order, round the endpoint or fit another
        endpoint from the target. Returned scores/counts use the same finite
        filtering and complete valid population as the original functions.
        """
        outside = ((self.x - expected[0]) ** 2 +
                   (self.y - expected[1]) ** 2 > self.radius_squared)
        if self.support is not None:
            outside = outside & self.support

        def search(template):
            scores = cv2.matchTemplate(self.target, template, cv2.TM_CCOEFF_NORMED,
                                       self._scores)
            np.isfinite(scores, out=self._valid)
            np.logical_and(self._valid, outside, out=self._valid)
            valid = self._valid
            if self.support is not None:
                count = int(np.count_nonzero(valid))
                return (float(scores[valid].max()) if count else None, count)
            return float(np.max(scores[valid])) if valid.any() else None

        return search
