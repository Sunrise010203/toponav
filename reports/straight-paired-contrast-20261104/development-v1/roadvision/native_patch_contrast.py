"""Experimental native patch contrast specialization, not patch/NCC sharing.

Only a boolean std>=2 gate is accelerated for 121 uint8 pixels. Extraction,
float32 patch values, NCC, search population and evidence decisions stay
independent for H and I. No patches/results are cached across calls or tracks.
"""
import cv2
import numpy as np

from roadvision import rgb_motion_checks as h, rgb_resampling_checks as i

_ORIGINAL_PATCH = h._patch
_ORIGINAL_SUPPORTED_PATCH = i._supported_patch
_ORIGINAL_H_EVIDENCE = h.patch_evidence
_ORIGINAL_I_EVIDENCE = i.appearance_evidence


def _contrast_gate(raw, patch):
    # Internal contract: patch is exactly raw.astype(np.float32), as in _patch.
    if type(raw) is np.ndarray and raw.dtype == np.uint8 and raw.shape == (11, 11):
        total = int(cv2.sumElems(raw)[0])
        squares = int(cv2.norm(raw, cv2.NORM_L2SQR))
        # All sums are exact integers below 2**24. Python arithmetic is exact.
        numerator = 121*squares-total*total
        # Original float32 variance differs from exact variance by <0.27:
        # mean/subtract/square <0.00974 per value; 120 sum roundings <=30;
        # divide rounding <=2**-9. Sqrt rounding cannot span either margin.
        # These are sufficient conditions, NOT new descriptor thresholds.
        if numerator < 121**2:
            return False
        if numerator > 9*121**2:
            return True
    return float(patch.std()) >= 2.


def prepared_patch(function, gray, point):
    # Honor altered callbacks, non-native canvases and changed patch policy.
    if (function is not _ORIGINAL_PATCH or type(gray) is not np.ndarray or
            gray.dtype != np.uint8 or gray.shape != (128, 128) or
            h.CHECK_POLICY['patch_px'] != 11):
        return function(gray, point)
    size=h.CHECK_POLICY['patch_px'];half=size//2
    p=np.asarray(point,float)
    if p.shape!=(2,) or not np.isfinite(p).all() or np.any(p<half+1) or np.any(p>=128-half-1): return None
    raw=cv2.getRectSubPix(gray,(size,size),tuple(map(float,p)))
    patch=raw.astype(np.float32)
    return patch if _contrast_gate(raw, patch) else None


def supported_patch(gray, point, mask):
    if i._supported_patch is not _ORIGINAL_SUPPORTED_PATCH:
        return i._supported_patch(gray, point, mask)
    patch=prepared_patch(i._patch,gray,point)
    if patch is None:return None
    valid=cv2.getRectSubPix(mask.astype(np.float32),(11,11),tuple(map(float,point)))
    return patch if np.all(valid>=1.-1e-6) else None


def _native_h_evidence(first,second,source,target,*,_searches=None):
    """Global alternative search, not merely a forward/backward local match.

    Endpoints use appearance-only normalized grayscale patches. Large local
    scale/rotation/lighting changes may reject valid matches; count that loss.
    """
    a,b=prepared_patch(h._patch,first,source),prepared_patch(h._patch,second,target)
    result={'accepted':False,'observed_ncc':None,'alternative_ncc':None,
            'reverse_alternative_ncc':None,'self_alternative_ncc':None,'margin':None}
    if a is None or b is None:
        result['reason']='patch_boundary_or_flat';return result
    observed=h._ncc(a,b)
    if _searches is None:
        alt=h._alternative(a,second,target);reverse=h._alternative(b,first,source)
        self_a,self_b=h._alternative(a,first,source),h._alternative(b,second,target)
    else:
        first_search=_searches[0].at(source);second_search=_searches[1].at(target)
        alt=second_search(a);reverse=first_search(b)
        self_a,self_b=first_search(a),second_search(b)
    if any(v is None for v in (observed,alt,reverse,self_a,self_b)):
        result['reason']='invalid_descriptor_search';return result
    self_score=max(self_a,self_b)
    margin=observed-max(alt,reverse)
    result.update(observed_ncc=observed,alternative_ncc=alt,reverse_alternative_ncc=reverse,
                  self_alternative_ncc=self_score,margin=margin)
    failures=[]
    if observed<h.CHECK_POLICY['minimum_ncc']:failures.append('appearance_changed')
    if margin<h.CHECK_POLICY['minimum_margin']:failures.append('competing_match')
    if self_score>=h.CHECK_POLICY['self_similarity_max']:failures.append('self_repetition')
    result.update(accepted=not failures,reason='unique_patch_support' if not failures else '+'.join(failures))
    return result


def _native_i_evidence(first, second, source, target, first_mask, second_mask, *, _searches=None):
    """Same numerical H gates; bidirectional full-canvas valid competition."""
    a=supported_patch(first,source,first_mask);b=supported_patch(second,target,second_mask)
    result=dict(accepted=False,observed_ncc=None,alternative_ncc=None,
                reverse_alternative_ncc=None,self_alternative_ncc=None,margin=None)
    if a is None or b is None:
        result['reason']='patch_boundary_flat_or_padding';return result
    observed=i._ncc(a,b)
    if _searches is None:
        alt,n1=i._masked_alternative(a,second,target,second_mask)
        reverse,n2=i._masked_alternative(b,first,source,first_mask)
        self_a,n3=i._masked_alternative(a,first,source,first_mask)
        self_b,n4=i._masked_alternative(b,second,target,second_mask)
    else:
        first_search=_searches[0].at(source);second_search=_searches[1].at(target)
        alt,n1=second_search(a);reverse,n2=first_search(b)
        self_a,n3=first_search(a);self_b,n4=second_search(b)
    result['alternative_search_counts']=[n1,n2,n3,n4]
    if any(v is None for v in (observed,alt,reverse,self_a,self_b)):
        result['reason']='invalid_descriptor_search';return result
    self_score=max(self_a,self_b);margin=observed-max(alt,reverse)
    result.update(observed_ncc=observed,alternative_ncc=alt,reverse_alternative_ncc=reverse,
                  self_alternative_ncc=self_score,margin=margin)
    failures=[]
    if observed<i.CHECK_POLICY['minimum_ncc']:failures.append('appearance_changed')
    if margin<i.CHECK_POLICY['minimum_margin']:failures.append('competing_match')
    if self_score>=i.CHECK_POLICY['self_similarity_max']:failures.append('self_repetition')
    result.update(accepted=not failures,reason='+'.join(failures) if failures else 'unique_patch_support')
    return result


def h_evidence(*args, **kwargs):
    if h.patch_evidence is not _ORIGINAL_H_EVIDENCE:
        return h.patch_evidence(*args, **kwargs)
    return _native_h_evidence(*args, **kwargs)


def i_evidence(*args, **kwargs):
    if i.appearance_evidence is not _ORIGINAL_I_EVIDENCE:
        return i.appearance_evidence(*args, **kwargs)
    return _native_i_evidence(*args, **kwargs)
