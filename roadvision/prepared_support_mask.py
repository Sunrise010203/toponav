"""Lazy support conversion for owned masks in ONE native paired region.

Not patch evidence or a support decision: every requested footprint is still
sampled by the original OpenCV operation. The caller owns the source masks and
does not mutate them during this region, just as with prepared target canvases.
No global, cross-region, cross-track evidence or controller state is retained.
"""
import numpy as np


class PreparedSupportMask:
    def __init__(self, source):
        self.source = source
        self._canvas = None

    def sampling_canvas(self):
        if self._canvas is None:
            self._canvas = self.source.astype(np.float32)
        return self._canvas


def prepare_support_masks(masks):
    return tuple(PreparedSupportMask(mask)
                 if type(mask) is np.ndarray and mask.dtype == np.uint8 and mask.shape == (128, 128)
                 else mask for mask in masks)


def original_support_mask(mask):
    return mask.source if type(mask) is PreparedSupportMask else mask
