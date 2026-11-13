"""Reference-only startup observability veto, NOT a free-space/depth sensor.

The clearance prior and explicit task authorization are still required. An
accepted texture test cannot establish absence of obstacles or certify motion.
Uses the existing nine lookahead cells/descriptor thresholds, without targets,
scene labels, training, or selecting an alternative geometric model.
"""
import cv2
import numpy as np
from roadvision.free_ground_path import select_reference_cells
from roadvision.rgb_ground_rig import _rectification
from roadvision.rgb_motion_checks import CHECK_POLICY
from roadvision.rgb_resampling_checks import _supported_patch, _masked_alternative


def assess_startup_texture(rgb,calibration):
    gray=cv2.cvtColor(np.ascontiguousarray(rgb[:128,:128]),cv2.COLOR_RGB2GRAY)
    cells=select_reference_cells(gray,np.ones((128,128),np.uint8),calibration)
    warp=_rectification()
    canvas=cv2.warpPerspective(gray,warp,(128,128),flags=cv2.INTER_LINEAR|cv2.WARP_INVERSE_MAP)
    support=cv2.warpPerspective(np.ones((128,128),np.float32),warp,(128,128),flags=cv2.INTER_LINEAR|cv2.WARP_INVERSE_MAP)
    mask=(support>=1-1e-6).astype(np.uint8);inverse=np.linalg.inv(warp)
    for cell in cells:
        point=inverse@np.r_[cell['pixel_latest'],1.];point=point[:2]/point[2]
        patch=_supported_patch(canvas,point,mask)
        score=None if patch is None else _masked_alternative(patch,canvas,point,mask)[0]
        cell.update(rectified_self_alternative_ncc=score,
                    rectified_accepted=score is not None and score<CHECK_POLICY['self_similarity_max'])
    good=[c for c in cells if c['rectified_accepted']]
    # At least six supported cells, spanning all three forward/lateral bins.
    # The canvas is only a calibrated level-ground hypothesis; missing/flat
    # samples or detected repetition never become positive clearance evidence.
    reference_count=sum(c['reference_accepted'] for c in cells)
    checks=dict(native_reference_cells_unique=all(c['reference_accepted'] for c in cells),
                rectified_supported_cell_count=len(good)>=6,
                forward_bin_coverage=len({c['nominal_point_latest_camera_xy_m'][0] for c in good})==3,
                lateral_bin_coverage=len({c['nominal_point_latest_camera_xy_m'][1] for c in good})==3)
    accepted=all(checks.values())
    return dict(accepted=bool(accepted),reason='reference_texture_supported' if accepted else 'reference_texture_ambiguous_or_unsupported',
                cells=cells,reference_supported_cells=reference_count,rectified_supported_cells=len(good),
                checks=checks,failed_checks=[key for key,value in checks.items() if not value],
                motion_authorized=False,certified_clearance=False,
                scope='static calibrated ground hypothesis; necessary observability veto only, not universal repetition detection')
