"""Opt-in joint-reference startup selection; NOT a free-space/depth sensor.

The clearance prior and explicit task authorization are still required. An
accepted texture test cannot establish absence of obstacles or certify motion.
Uses the existing nine lookahead cells/descriptor thresholds, without targets,
scene labels, training, or selecting an alternative geometric model. Equal
numerical thresholds do NOT make this a policy-equivalent implementation:
joint selection changes which references can pass. Not promoted to baseline.
"""
import cv2
import numpy as np
from roadvision.free_ground_path import reference_cell_candidates
from roadvision.rgb_ground_rig import _rectification
from roadvision.rgb_motion_checks import CHECK_POLICY
from roadvision.rgb_resampling_checks import _supported_patch, _masked_alternative
from roadvision.continuous_straight import StraightController


def assess_joint_startup_texture(rgb,calibration):
    gray=cv2.cvtColor(np.ascontiguousarray(rgb[:128,:128]),cv2.COLOR_RGB2GRAY)
    warp=_rectification()
    canvas=cv2.warpPerspective(gray,warp,(128,128),flags=cv2.INTER_LINEAR|cv2.WARP_INVERSE_MAP)
    support=cv2.warpPerspective(np.ones((128,128),np.float32),warp,(128,128),flags=cv2.INTER_LINEAR|cv2.WARP_INVERSE_MAP)
    mask=(support>=1-1e-6).astype(np.uint8);inverse=np.linalg.inv(warp)
    cells=[]
    for group in reference_cell_candidates(gray,np.ones((128,128),np.uint8),calibration):
        candidates=group['candidates']
        for cell in candidates:
            point=inverse@np.r_[cell['pixel_latest'],1.];point=point[:2]/point[2]
            patch=_supported_patch(canvas,point,mask)
            score=None if patch is None else _masked_alternative(patch,canvas,point,mask)[0]
            cell.update(rectified_self_alternative_ncc=score,
                        rectified_accepted=score is not None and score<CHECK_POLICY['self_similarity_max'])
        # Select once using ONLY the same stationary reference image in both
        # representations. Native validity has priority: rectification must
        # never rescue a flat/ambiguous native patch. Stable native-score
        # tie-break preserves the old choice whenever it already passes.
        chosen=max(candidates,key=lambda c:(c['reference_accepted'],c['rectified_accepted'],
            -c['reference_self_alternative_ncc'] if c['reference_self_alternative_ncc'] is not None else -2.))
        cells.append({**chosen,'nominal_point_latest_camera_xy_m':group['nominal_point_latest_camera_xy_m'],
            'selection_scope':'same reference RGB, native and rectified; no target or motion fit',
            'reference_candidate_count':len(candidates)})
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
                selection_policy='joint_reference_native_rectified_v1',
                scope='static calibrated ground hypothesis; necessary observability veto only, not universal repetition detection')


class ReferenceStraightController(StraightController):
    """Change startup selection only; inherited runtime/stop gates stay intact."""

    def _assess_startup_texture(self,rgb):
        return assess_joint_startup_texture(rgb,self.calibration)

    def _output(self,v=0.,omega=0.):
        out=super()._output(v,omega)
        out['experiment']=dict(variant='reference',promoted=False,
            startup_selection='joint_reference_native_rectified_v1')
        return out

    def result(self):
        return dict(super().result(),experiment=dict(variant='reference',promoted=False,
            startup_selection='joint_reference_native_rectified_v1'))
