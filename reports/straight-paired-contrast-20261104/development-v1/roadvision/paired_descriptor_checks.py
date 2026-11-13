"""Native H/I work sharing, NOT a shared descriptor acceptance decision.

Only identical raw score maps for the same logical role can be shared between
channels in ONE track. Identical endpoint/support populations may also reuse
their exact reductions. Each original evidence function still applies its own
patch support and gates. Models are fitted separately. Rectified inputs keep
the old independent path.
"""
import cv2
import numpy as np

from roadvision import rgb_motion_checks as h, rgb_resampling_checks as i
from roadvision.prepared_descriptor_checks import filter_h, filter_i
from roadvision.prepared_descriptor_search import PreparedAlternative
from roadvision.local_exclusion_population import LocalExclusion
from roadvision.native_patch_contrast import h_evidence, i_evidence


class _TrackScores:
    """Two role maps and two populations per canvas, cleared after each track."""
    def __init__(self, target):
        self.target = target.astype(np.float32)
        self.entries = {}
        self.populations = {}
        # Static support interning is region-local; not descriptor evidence.
        self.supports = []

    def intern_support(self, support):
        # An all-true eroded support selects precisely H's unmasked population.
        if support is None or support.all():
            return None, None
        for index, previous in enumerate(self.supports):
            if np.array_equal(previous, support):
                return index, previous
        self.supports.append(support)
        return len(self.supports) - 1, support

    def population(self, expected, search):
        point = np.asarray(expected)
        key = (point.dtype.str, point.shape, point.tobytes(), search.x.shape,
               search.radius_squared, search.support_key)
        if key not in self.populations:
            outside = search.exclusion_geometry.outside(expected, search.radius_squared)
            if search.population_support is not None:
                outside = outside & search.population_support
            # Bounded even for interleaved private searches with many endpoints.
            if len(self.populations) == 2:
                del self.populations[next(iter(self.populations))]
            self.populations[key] = _Population(outside)
        return self.populations[key]

    def match(self, role, template):
        key = (template.dtype.str, template.shape, template.tobytes())
        if role in self.entries and self.entries[role][0] == key:
            return self.entries[role][1]
        scores = cv2.matchTemplate(self.target, template, cv2.TM_CCOEFF_NORMED)
        if role < 2:
            # A unique generation also guards reused/mutated OpenCV buffers.
            self.entries[role] = (key, scores, object())
        return scores

    def clear(self):
        self.entries.clear()
        self.populations.clear()


class _Population:
    """Original finite max/count over ONE exact endpoint/support footprint."""
    def __init__(self, outside):
        self.outside = outside
        self.reductions = {}

    def reduce(self, role, scores, generation):
        entry = self.reductions.get(role)
        if generation is not None and entry is not None and entry[0] is generation:
            return entry[1]
        valid = self.outside & np.isfinite(scores)
        count = int(np.count_nonzero(valid))
        result = (float(scores[valid].max()) if count else None, count)
        if role < 2 and generation is not None:
            # Keep only a generation token, not a reference to a replaced map.
            self.reductions[role] = (generation, result)
        return result


class _ChannelSearch(PreparedAlternative):
    def __init__(self, target, provider, *, mask=None):
        super().__init__(target, patch_px=11, half=5, exclude_radius=11., mask=mask)
        self.provider = provider
        self.support_key, self.population_support = provider.intern_support(self.support)
        self.exclusion_geometry = LocalExclusion(self.x, self.y)

    def at(self, expected):
        population = self.provider.population(expected, self)
        role = 0

        def search(template):
            nonlocal role
            scores = self.provider.match(role, template)
            entry = self.provider.entries.get(role)
            generation = entry[2] if entry is not None and entry[1] is scores else None
            result = population.reduce(role, scores, generation)
            role += 1
            if self.support is not None:
                return result
            return result[0]
        return search


def _prepare_h(images, region):
    # Native-only specialization of H's existing preparation, unchanged order.
    warp = np.eye(3)
    gray = [cv2.cvtColor(np.ascontiguousarray(im), cv2.COLOR_RGB2GRAY)
            for im in (images[0], images[2])]
    inverse = np.array(region.get('coverage_map', np.linalg.inv(warp)), float)
    tracking_inverse = np.linalg.inv(warp)
    tracks = region['tracks']
    train, _ = h.split_tracks(tracks)
    aligned = np.eye(3)
    if len(train) >= 8:
        first, last = [np.array([tracks[j][key] for j in train], float)
                       for key in ('first', 'last')]
        homography, mask = h._homography(first, last)
        if homography is not None and mask.mean() >= h.POLICY['min_inlier_ratio']:
            aligned = tracking_inverse @ homography @ warp
            gray[1] = cv2.warpPerspective(gray[1], aligned, (128,128),
                flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
    alignment = dict(homography=aligned.tolist(), fit_track_indices=train,
                     scope='training-coordinate prealignment only')
    return gray, inverse, tracking_inverse, aligned, alignment, train


def _prepare_i(images, region):
    warp = np.eye(3)
    tracking_inverse = np.linalg.inv(warp)
    coverage = np.array(region.get('coverage_map', tracking_inverse), float)
    tracks = region['tracks']
    train, _ = i.split_tracks(tracks)
    aligned = np.eye(3)
    if len(train) >= 8:
        first, last = [np.array([tracks[j][key] for j in train], float)
                       for key in ('first', 'last')]
        homography, mask = i._homography(first, last)
        if homography is not None and mask.mean() >= i.POLICY['min_inlier_ratio']:
            aligned = tracking_inverse @ homography @ warp
    gray, masks = i.prepare_canvases(images, warp, aligned)
    native_h = warp @ aligned @ tracking_inverse
    inv_h = np.linalg.inv(native_h)
    alignment = dict(homography=aligned.tolist(), native_homography=native_h.tolist(),
        fit_track_indices=train,
        scope='training-coordinate prealignment only; one native target resampling')
    return gray, masks, coverage, inv_h, alignment, set(train)


def _finish_h(region, kept, evidence, alignment, inverse):
    # Keep H's model functions, not I's or the raw region's acceptance/model.
    out = dict(view=region['view'], rectified_ground_assumption=region['rectified_ground_assumption'],
        status='unknown', coverage_map=inverse.tolist(), reasons=[], tracks=kept,
        models={}, descriptor_evidence=evidence, descriptor_alignment=alignment,
        stage_counts=dict(input_tracks=len(region['tracks']), descriptor_accepted=len(kept), retained=len(kept)))
    if len(kept) < h.POLICY['min_tracks']:
        out['reasons'] = ['insufficient_unambiguous_tracks']; return out
    a,b,c = [np.array([t[key] for t in kept], float) for key in ('first','middle','last')]
    if (min(h._grid_count(h._project(inverse,p)) for p in (a,c)) < h.POLICY['min_grid_cells'] or
            min(h._spread_ratio(p) for p in (a,c)) < h.POLICY['min_spread_ratio']):
        out['reasons'] = ['poor_spatial_support']; return out
    out['models'], out['reasons'] = h._region_models(a,b,c,inverse)
    if not out['reasons']: out['status'] = 'planar_flow_candidate'
    return out


def filter_pair(images, region, *, filters=(filter_h, filter_i)):
    """Same {'H': ..., 'I': ...} as independent filters, no OR or retries.

    Alternate supplied filters retain their original semantics. Geometry and
    policy eligibility are decided before descriptors, not after rejection.
    Score maps are private, bounded and synchronous; no array is evidence.
    """
    if (filters != (filter_h, filter_i) or region['rectified_ground_assumption'] or
            h.CHECK_POLICY['patch_px'] != 11 or h.CHECK_POLICY['exclude_radius_px'] != 11.):
        return dict(H=filters[0](images, region), I=filters[1](images, region))
    gray_h, inverse, tracking_inverse, aligned, align_h, train_h = _prepare_h(images, region)
    gray_i, masks, coverage, inv_h, align_i, train_i = _prepare_i(images, region)
    providers_h = [_TrackScores(im) for im in gray_h]
    providers_i = [provider if a.dtype == b.dtype and a.shape == b.shape and np.array_equal(a,b)
                   else _TrackScores(b)
                   for a,b,provider in zip(gray_h,gray_i,providers_h)]
    searches_h = tuple(_ChannelSearch(im, provider) for im,provider in zip(gray_h,providers_h))
    searches_i = tuple(_ChannelSearch(im, provider, mask=mask)
                      for im,provider,mask in zip(gray_i,providers_i,masks))
    providers = providers_h + providers_i
    kept_h, kept_i, evidence_h, evidence_i = [], [], [], []
    for index, track in enumerate(region['tracks']):
        try:
            p,q = h._project(tracking_inverse, np.array([track['first'],track['last']],float))
            mapped_h = h._project(np.linalg.inv(aligned), q.reshape(1,2))
            eh = (dict(accepted=False, reason='invalid_prealignment_projection') if mapped_h is None
                  else h_evidence(*gray_h,p,mapped_h[0],_searches=searches_h))
            point = np.array(track['first'],float)
            mapped_i = i._project(inv_h,np.array([track['last']],float))
            ei = (dict(accepted=False, reason='invalid_prealignment_projection') if mapped_i is None
                  else i_evidence(*gray_i,point,mapped_i[0],*masks,_searches=searches_i))
            eh['track_index'] = ei['track_index'] = index
            evidence_h.append(eh); evidence_i.append(ei)
            if eh['accepted']:
                kept_h.append({**track,'feature_split':'train' if index in train_h else 'validation'})
            if ei['accepted']:
                kept_i.append({**track,'feature_split':'train' if index in train_i else 'validation'})
        finally:
            for provider in providers: provider.clear()
    return dict(H=_finish_h(region,kept_h,evidence_h,align_h,inverse),
                I=i._finish(region,kept_i,evidence_i,align_i,coverage))
