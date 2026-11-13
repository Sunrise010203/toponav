"""Native H/I work sharing, NOT a shared descriptor acceptance decision.

Only identical raw score maps for the same logical role can be shared between
channels in ONE track. Each original evidence function still applies its own
patch support, exclusion, finite population, scores, counts and gates. Models
are fitted separately. Rectified inputs keep the old independent path.
"""
import cv2
import numpy as np

from roadvision import rgb_motion_checks as h, rgb_resampling_checks as i
from roadvision.prepared_descriptor_checks import filter_h, filter_i
from roadvision.prepared_descriptor_search import PreparedAlternative


class _TrackScores:
    """At most two role maps per canvas; cleared at every track boundary."""
    def __init__(self, target):
        self.target = target.astype(np.float32)
        self.entries = {}

    def match(self, role, template):
        key = (template.dtype.str, template.shape, template.tobytes())
        if role in self.entries and self.entries[role][0] == key:
            return self.entries[role][1]
        scores = cv2.matchTemplate(self.target, template, cv2.TM_CCOEFF_NORMED)
        if role < 2:
            self.entries[role] = (key, scores)
        return scores

    def clear(self):
        self.entries.clear()


class _ChannelSearch(PreparedAlternative):
    def __init__(self, target, provider, *, mask=None):
        super().__init__(target, patch_px=11, half=5, exclude_radius=11., mask=mask)
        self.provider = provider

    def at(self, expected):
        outside = ((self.x - expected[0]) ** 2 +
                   (self.y - expected[1]) ** 2 > self.radius_squared)
        if self.support is not None:
            outside = outside & self.support
        role = 0

        def search(template):
            nonlocal role
            scores = self.provider.match(role, template)
            role += 1
            valid = outside & np.isfinite(scores)
            if self.support is not None:
                return (float(scores[valid].max()) if valid.any() else None,
                        int(valid.sum()))
            return float(np.max(scores[valid])) if valid.any() else None
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
                  else h.patch_evidence(*gray_h,p,mapped_h[0],_searches=searches_h))
            point = np.array(track['first'],float)
            mapped_i = i._project(inv_h,np.array([track['last']],float))
            ei = (dict(accepted=False, reason='invalid_prealignment_projection') if mapped_i is None
                  else i.appearance_evidence(*gray_i,point,mapped_i[0],*masks,_searches=searches_i))
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
