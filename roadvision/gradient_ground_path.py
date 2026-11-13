"""Opt-in gradient uniqueness at the SAME nine fixed ground-transfer cells.

Not an OR with baseline acceptance: every cell must satisfy gradient NCC,
margin and self-uniqueness plus native intensity/flow vetoes. Raw intensity
margin is REPLACED, not relaxed numerically; the accepted set changes and is
not promoted. No retry, scene labels, new depth or complete swept coverage.
"""
import cv2
import numpy as np

from roadvision.free_ground_path import assess_free_ground
from roadvision.rgb_resampling_checks import _supported_patch, _masked_alternative
from roadvision.rgb_motion_checks import CHECK_POLICY


def gradient_canvas(gray,mask):
    """Fixed 3x3 Sobel x/y, zero third channel; require derivative support."""
    gx=cv2.Sobel(gray,cv2.CV_32F,1,0,ksize=3,borderType=cv2.BORDER_CONSTANT)
    gy=cv2.Sobel(gray,cv2.CV_32F,0,1,ksize=3,borderType=cv2.BORDER_CONSTANT)
    canvas=np.stack((gx,gy,np.zeros_like(gx)),axis=2)
    support=cv2.erode(mask,np.ones((3,3),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0)
    return canvas,support


def gradient_ncc(a,b):
    # matchTemplate CCOEFF subtracts the mean PER CHANNEL, not one shared
    # RGB/vector mean. Match that convention for the observed descriptor.
    a=a.astype(np.float64).reshape(-1,3);b=b.astype(np.float64).reshape(-1,3)
    a-=a.mean(axis=0);b-=b.mean(axis=0)
    denominator=float(np.linalg.norm(a)*np.linalg.norm(b))
    return None if denominator<1e-9 else float(np.clip(np.sum(a*b)/denominator,-1.,1.))


def gradient_appearance(first,second,point,first_mask,second_mask):
    result=dict(accepted=False,observed_ncc=None,alternative_ncc=None,
        reverse_alternative_ncc=None,self_alternative_ncc=None,margin=None)
    a=_supported_patch(first,point,first_mask);b=_supported_patch(second,point,second_mask)
    if a is None or b is None:return dict(result,reason='gradient_patch_flat_boundary_or_padding')
    observed=gradient_ncc(a,b)
    alt,n1=_masked_alternative(a,second,point,second_mask)
    reverse,n2=_masked_alternative(b,first,point,first_mask)
    sa,n3=_masked_alternative(a,first,point,first_mask)
    sb,n4=_masked_alternative(b,second,point,second_mask)
    result['alternative_search_counts']=[n1,n2,n3,n4]
    if any(v is None or not np.isfinite(v) for v in (observed,alt,reverse,sa,sb)):
        return dict(result,reason='invalid_gradient_search')
    score=max(sa,sb);margin=observed-max(alt,reverse)
    result.update(observed_ncc=observed,alternative_ncc=alt,reverse_alternative_ncc=reverse,
        self_alternative_ncc=score,margin=margin)
    failures=[]
    if observed<CHECK_POLICY['minimum_ncc']:failures.append('gradient_appearance_changed')
    if margin<CHECK_POLICY['minimum_margin']:failures.append('gradient_competing_match')
    if score>=CHECK_POLICY['self_similarity_max']:failures.append('gradient_self_repetition')
    return dict(result,accepted=not failures,reason='+'.join(failures) if failures else 'unique_gradient_support')


def assess_gradient_free_ground(frames,motion,*,now_s,calibration_id,calibration=None):
    # Always run exactly one unchanged baseline sampler/warp/flow evaluation.
    # Its original reference selection is retained even when a target fails.
    out=assess_free_ground(frames,motion,now_s=now_s,calibration_id=calibration_id,calibration=calibration)
    out['descriptor_policy']=dict(name='sobel_xy_uniqueness_v1',promoted=False,
        intensity_margin_replaced=True,native_intensity_and_flow_vetoes=True,
        reference_selection='unchanged native reference only',all_nine_required=True,
        kernel='fixed_3x3_sobel_xy',derivative_support_radius_px=1,
        thresholds={k:CHECK_POLICY[k] for k in ('patch_px','minimum_ncc','minimum_margin','self_similarity_max','exclude_radius_px')})
    if not out['cells']:return out
    h=np.array(out['ground_homography_first_to_latest'])
    a,b=[cv2.cvtColor(np.ascontiguousarray(f['rgb'][:128,:128]),cv2.COLOR_RGB2GRAY) for f in (frames[0],frames[-1])]
    a=cv2.warpPerspective(a,h,(128,128),flags=cv2.INTER_LINEAR)
    support=cv2.warpPerspective(np.ones((128,128),np.float32),h,(128,128),flags=cv2.INTER_LINEAR)
    first,first_mask=gradient_canvas(a,(support>=1-1e-6).astype(np.uint8))
    second,second_mask=gradient_canvas(b,np.ones((128,128),np.uint8))
    cells=[]
    for cell in out['cells']:
        gradient=gradient_appearance(first,second,cell['pixel_latest'],first_mask,second_mask)
        raw_ok=(cell['reference_accepted'] and cell.get('observed_ncc') is not None and
            cell['observed_ncc']>=CHECK_POLICY['minimum_ncc'] and
            cell.get('self_alternative_ncc') is not None and cell['self_alternative_ncc']<CHECK_POLICY['self_similarity_max'])
        accepted=bool(raw_ok and cell.get('ground_flow_supported') and gradient['accepted'])
        reasons=[]
        if not raw_ok:reasons.append('native_intensity_veto')
        if not cell.get('ground_flow_supported'):reasons.append('ground_transfer_flow_not_supported')
        if not gradient['accepted']:reasons.append(gradient['reason'])
        cells.append(dict(cell,intensity_evidence=dict(cell),gradient_evidence=gradient,
            accepted=accepted,reason='+'.join(reasons) if reasons else 'native_and_gradient_ground_transfer_support'))
    out['cells']=cells
    # Empty/incomplete samples never acquire clearance by all([]).
    if len(cells)==9 and all(c['accepted'] for c in cells):
        out.update(status='conditional_free_ground_path',reason='sampled_gradient_ground_transfer_supports_restricted_short_stopping_clearance',conditional_stopping_clearance_m=.6)
    else:
        out.update(status='unknown',reason='one_or_more_gradient_lookahead_cells_unverified',conditional_stopping_clearance_m=None)
    return out
