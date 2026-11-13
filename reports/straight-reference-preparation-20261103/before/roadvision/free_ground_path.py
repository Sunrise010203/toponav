"""Conditional RGB ground-transfer lookahead, not semantic segmentation.

Uses explicit static/horizontal-ground and calibration priors. Patches lie
outside the y104..119 motion-fitting detection band. Flat, ambiguous, padded,
occluded or geometrically inconsistent patches are UNKNOWN, never free.
"""
import cv2
import numpy as np
from roadvision.calibrated_ground_motion import GroundCalibration,ground_pixels,_camera_motion
from roadvision.rgb_ground_rig import rig_basis
from roadvision.rgb_resampling_checks import appearance_evidence
from roadvision.reference_patch_evidence import reference_appearance_evidence
from roadvision.ground_history import validate_history
from roadvision.corridor_rgb_probe import _lk

FREE_PATH_POLICY=dict(forward_samples_m=[.85,1.,1.2],lateral_samples_m=[-.22,0.,.22],
                      minimum_baseline_m=.025,maximum_baseline_m=.4,
                      maximum_flow_residual_px=.5,conditional_stopping_clearance_m=.6,
                      reference_only_cell_offset_m=.01,
                      assumed_minimum_obstacle_height_m=.6,assumed_minimum_obstacle_width_m=.25)


def reference_cell_candidates(aligned,mask,calibration):
    """Nine fixed placements per cell, evaluated on the reference RGB only."""
    offsets=[(0.,0.),(-.01,-.01),(-.01,0.),(-.01,.01),(0.,-.01),(0.,.01),(.01,-.01),(.01,0.),(.01,.01)]
    cells=[]
    for x in FREE_PATH_POLICY['forward_samples_m']:
        for y in FREE_PATH_POLICY['lateral_samples_m']:
            candidates=[]
            for dx,dy in offsets:
                xy=[x+dx,y+dy];pixel=ground_pixels(np.array([xy]),'front',calibration)[0]
                e=reference_appearance_evidence(aligned,pixel,mask)
                score=e.get('self_alternative_ncc')
                candidates.append(dict(point_latest_camera_xy_m=xy,pixel_latest=pixel.tolist(),
                    reference_accepted=e['accepted'],reference_self_alternative_ncc=score))
            cells.append(dict(nominal_point_latest_camera_xy_m=[x,y],candidates=candidates))
    return cells


def select_reference_cells(aligned,mask,calibration):
    """Select ONE reference-only location; target rejection never retries.

    At-most-one-centimeter placements keep lateral neighboring centers at
    most .24m apart. All nine cells still need their heldout/flow checks.
    """
    cells=[]
    for cell in reference_cell_candidates(aligned,mask,calibration):
        chosen=max(cell['candidates'],key=lambda c:(c['reference_accepted'],-c['reference_self_alternative_ncc'] if c['reference_self_alternative_ncc'] is not None else -2.))
        cells.append({**chosen,'nominal_point_latest_camera_xy_m':cell['nominal_point_latest_camera_xy_m'],
            'selection_scope':'first warped RGB only; target not consulted','reference_candidate_count':9})
    return cells


def assess_free_ground(frames,motion,*,now_s,calibration_id,calibration=None):
    refs=validate_history(frames,now_s=now_s,calibration_id=calibration_id);calibration=calibration or GroundCalibration()
    out=dict(status='unknown',reason='motion_or_reference_unavailable',sample_horizon_m=1.2,conditional_stopping_clearance_m=None,cell_count=9,cells=[],
             geometry_reference_frame_id=refs[-1]['frame_id'],geometry_time_s=refs[-1]['captured_sim_s'],
             motion_authorized=False,arrival_status='unverified',certified_clearance=False,
             declared_assumptions='static level calibrated textured ground; bounded-size vertical blockers')
    if (motion.get('status')!='conditional_ground_motion' or
        motion.get('translation_reference_frame_id')!=refs[0]['frame_id'] or
        motion.get('from_time_s')!=refs[0]['captured_sim_s'] or motion.get('to_time_s')!=refs[-1]['captured_sim_s']):return out
    baseline=float(np.linalg.norm(motion['translation_m']))
    if not .025<=baseline<=.4:return {**out,'reason':'motion_baseline_outside_domain'}
    rot,t=_camera_motion('front',motion)
    k=np.array([[calibration.focal_px,0,calibration.principal_px],[0,calibration.focal_px,calibration.principal_px],[0,0,1.]])
    q=rig_basis('front').T@np.array([0.,0.,-1.])/calibration.height_m
    h=k@(rot+np.outer(t,q))@np.linalg.inv(k)
    a,b=[cv2.cvtColor(np.ascontiguousarray(f['rgb'][:128,:128]),cv2.COLOR_RGB2GRAY) for f in (frames[0],frames[-1])]
    aligned=cv2.warpPerspective(a,h,(128,128),flags=cv2.INTER_LINEAR)
    support=cv2.warpPerspective(np.ones((128,128),np.float32),h,(128,128),flags=cv2.INTER_LINEAR)
    first_mask=(support>=1-1e-6).astype(np.uint8);last_mask=np.ones((128,128),np.uint8)
    cells=[]
    selected=select_reference_cells(aligned,first_mask,calibration)
    pixels=np.array([c['pixel_latest'] for c in selected])
    observed,valid=_lk(aligned,b,pixels.astype(np.float32));fb=np.full(len(pixels),np.inf)
    if valid.any():
        back,back_valid=_lk(b,aligned,observed[valid].astype(np.float32))
        indices=np.flatnonzero(valid);fb[indices[back_valid]]=np.linalg.norm(back[back_valid]-pixels[indices[back_valid]],axis=1)
    residual=np.linalg.norm(observed-pixels,axis=1)
    for index,cell in enumerate(selected):
        p=pixels[index]
        evidence=appearance_evidence(aligned,b,p,p,first_mask,last_mask)
        # NCC tolerates subpixel wrong geometry. Measure remaining flow
        # independently after the fixed ground warp, do not let a high
        # photometric score certify a vertical blocker as ground.
        # .5px is twice the declared .25px image noise floor. It cannot
        # certify every sampled far patch as unobstructed out to1.2m.
        # Only the substantially shorter .6m stopping-clearance working
        # domain is claimed, and even that is conditional, not certified.
        flow_ok=bool(valid[index] and residual[index]<=.5 and fb[index]<=.25)
        evidence.update(ground_flow_residual_px=float(residual[index]) if valid[index] else None,
                        flow_fb_px=float(fb[index]) if np.isfinite(fb[index]) else None,ground_flow_supported=flow_ok)
        if not flow_ok:evidence.update(accepted=False,reason='ground_transfer_flow_not_supported')
        if not cell['reference_accepted']:evidence.update(accepted=False,reason='reference_cell_not_unique')
        cells.append({**cell,**evidence})
    out.update(cells=cells,ground_homography_first_to_latest=h.tolist(),policy=FREE_PATH_POLICY)
    if all(c['accepted'] for c in cells):out.update(status='conditional_free_ground_path',reason='sampled_ground_transfer_supports_restricted_short_stopping_clearance',conditional_stopping_clearance_m=.6)
    else:out['reason']='one_or_more_lookahead_cells_unverified'
    return out
