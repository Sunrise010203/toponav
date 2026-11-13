"""Opt-in rear RGB consistency and non-actuating active-observation review.

Never changes an accepted front/side estimate, its scale, or its covariance.
Rear is an additional held-out prediction check, not a stereo baseline or
independent proof of a static scene. Unknown/contradictory rear cannot vote
other evidence into acceptance. Active proposals have NO motor authority.
"""
import math
import numpy as np

from roadvision.calibrated_ground_motion import (
    GroundCalibration, GROUND_POLICY, select_descriptor, pixel_predictions,
)
from roadvision.corridor_rgb_probe import _grid_count, _spread_ratio, _project
from roadvision.ground_band import track_ground_band
from roadvision.ground_history import validate_history
from roadvision.rgb_motion_checks import filter_region as filter_h, split_tracks
from roadvision.rgb_resampling_checks import filter_region as filter_i


def task_contract(active=False):
    return dict(version='rear_observation_review_v1',promoted=False,
        rear_scope='additional fixed-motion heldout check and native back patch footprints; no motion refit or averaging',
        startup_authority='unchanged explicit bounded-probe initial prior',
        active_scope='proposal_only_no_actuator_connection' if active else 'disabled',
        proposal=dict(v_m_s=.08,omega_rad_s=0.,duration_s=.2,
                      actuator_distance_bound_m=.036,maximum_attempts=2),
        recovery_clock='original unrefreshable waiting-evidence deadline',
        unknown_is_free=False,pure_rotation_is_depth=False,
        map_is_clearance=False,certified_clearance=False,motion_authorized=False)


def assess_rear_consistency(frames,packet,*,now_s,calibration_id,calibration=None):
    """One training-selected descriptor; SAME fixed motion, no fit retries.

    Spatial, descriptor and temporal track gates are inherited. Both train
    and assigned held-out tracks must support the original motion. No new
    rear fit, pose truth, per-view time override or accepted-set union.
    """
    refs=validate_history(frames,now_s=now_s,calibration_id=calibration_id)
    calibration=calibration or GroundCalibration()
    out=dict(status='unknown',reason='unbound_or_unavailable_motion',
             frames=refs,geometry_reference_frame_id=refs[-1]['frame_id'],
             geometry_time_s=refs[-1]['captured_sim_s'],motion_authorized=False,
             certified_clearance=False,arrival_status='unverified')
    motion=packet.get('ground_motion',{})
    if (packet.get('geometry_reference_frame_id')!=refs[-1]['frame_id'] or
        packet.get('geometry_time_s')!=refs[-1]['captured_sim_s'] or
        motion.get('status')!='conditional_ground_motion' or
        motion.get('translation_reference_frame_id')!=refs[0]['frame_id'] or
        motion.get('from_time_s')!=refs[0]['captured_sim_s'] or
        motion.get('to_time_s')!=refs[-1]['captured_sim_s']):return out
    t=np.asarray(motion.get('translation_m'),float);yaw=motion.get('yaw_delta_rad')
    if (t.shape!=(2,) or not np.isfinite(t).all() or type(yaw) not in (int,float)
        or not math.isfinite(yaw) or not .025<=np.linalg.norm(t)<=.4 or abs(yaw)>.25):
        return {**out,'reason':'translation_or_rotation_outside_observation_domain'}
    images=[np.ascontiguousarray(f['rgb'][128:,128:]) for f in frames]
    raw=track_ground_band(images,view='back')
    channels=dict(H=filter_h(images,raw),I=filter_i(images,raw))
    selection=select_descriptor(raw['tracks'],channels,np.array(raw['coverage_map']))
    region=channels[selection['channel']]
    out.update(selection=selection,stage_counts=raw['stage_counts'],
               tracks=region['tracks'],source_reasons=region.get('reasons',[]))
    if region.get('reasons'):
        return {**out,'reason':'rear_spatial_temporal_or_descriptor_quality_unverified'}
    train,heldout=split_tracks(region['tracks'])
    if (len(region['tracks'])<GROUND_POLICY['minimum_tracks_per_view'] or
        len(train)<GROUND_POLICY['minimum_train_per_view'] or
        len(heldout)<GROUND_POLICY['minimum_validation_per_view']):
        return {**out,'reason':'insufficient_rear_tracks'}
    first,last=[np.array([t[k] for t in region['tracks']],float) for k in ('first','last')]
    coverage=np.array(region['coverage_map'])
    if (min(_grid_count(_project(coverage,p)) for p in (first,last))<GROUND_POLICY['minimum_grid_cells'] or
        min(_spread_ratio(p) for p in (first,last))<GROUND_POLICY['minimum_spread']):
        return {**out,'reason':'rear_spatial_support_unverified'}
    prediction=pixel_predictions(first,'back',np.r_[t,yaw],calibration)
    if not np.isfinite(prediction).all():return {**out,'reason':'rear_projection_invalid'}
    residual=np.linalg.norm(prediction-last,axis=1)
    metrics={}
    for name,indices in [('training',train),('heldout',heldout)]:
        e=residual[indices]
        metrics[name]=dict(count=len(indices),indices=indices,median_px=float(np.median(e)),
            p90_px=float(np.quantile(e,.9)),inlier_fraction=float(np.mean(e<=1.)))
    a,b=metrics['training'],metrics['heldout']
    supported=(a['inlier_fraction']>=GROUND_POLICY['minimum_train_fraction'] and
        b['median_px']<=GROUND_POLICY['validation_median_px'] and
        b['p90_px']<=GROUND_POLICY['validation_p90_px'] and
        b['inlier_fraction']>=GROUND_POLICY['validation_inlier_fraction'])
    return {**out,'status':'conditional_rear_consistent' if supported else 'conflict',
            'reason':'rear_supports_unchanged_motion' if supported else 'rear_contradicts_unchanged_motion',
            'pixel_checks':metrics,'motion_reestimated':False,'motion_averaged':False}


def review_active_proposal(*,now_s,frame_id,captured_s,state,permission,stationary_frames,
                           recovery_count,attempts,wait_started,probe_distance_bound_m,
                           policy,sweep,rear):
    """Read-only proposal audit. Even eligible proposals NEVER authorize motion.

    All budgets derive from the existing controller. No reset, extra initial
    prior, reverse motion, yaw-only maneuver, or topology input is available.
    The reserved .036m uses actuator bound, not desired-speed integration.
    """
    checks={}
    finite=lambda x:type(x) in (int,float) and math.isfinite(x)
    valid_clock=(finite(now_s) and finite(captured_s) and finite(wait_started) and
                 0<=now_s-captured_s<=policy.max_frame_age_s and wait_started<=now_s)
    remaining=policy.unverified_timeout_s-(now_s-wait_started) if valid_clock else None
    distance=policy.actuator_speed_bound_m_s*.2
    checks['current_wait_and_fresh_capture']=valid_clock and state=='waiting_evidence'
    checks['original_motion_permission']=permission=='bounded_probe'
    checks['confirmed_stationary']=type(stationary_frames) is int and stationary_frames>=policy.stationary_confirmations
    checks['original_recovery_limit']=type(recovery_count) is int and 0<=recovery_count<policy.max_recoveries
    checks['proposal_attempt_limit']=type(attempts) is int and 0<=attempts<2
    # Must leave the original hold+braking interval INSIDE the old deadline.
    brake_s=policy.actuator_speed_bound_m_s/policy.minimum_brake_deceleration_m_s2
    checks['original_time_budget']=remaining is not None and remaining>=.2+policy.command_horizon_s+brake_s
    checks['original_distance_budget']=(finite(probe_distance_bound_m) and probe_distance_bound_m>=0 and
        probe_distance_bound_m+distance<=policy.probe_distance_limit_m+1e-9)
    checks['current_rear_consistent']=(isinstance(rear,dict) and rear.get('status')=='conditional_rear_consistent' and
        (rear.get('geometry_reference_frame_id'),rear.get('geometry_time_s'))==(frame_id,captured_s))
    checks['current_requested_swept_coverage']=(isinstance(sweep,dict) and
        sweep.get('status')=='conditional_patch_supported_sweep' and
        (sweep.get('geometry_reference_frame_id'),sweep.get('geometry_time_s'))==(frame_id,captured_s) and
        (sweep.get('requested_v_m_s'),sweep.get('requested_omega_rad_s'))==(.08,0.))
    failures=[name for name,passed in checks.items() if not passed]
    return dict(status='ineligible' if failures else 'eligible_proposal_only',checks=checks,
        failed_checks=failures,geometry_reference_frame_id=frame_id,geometry_time_s=captured_s,
        requested_v_m_s=.08,requested_omega_rad_s=0.,duration_s=.2,
        actuator_distance_bound_m=distance,remaining_original_wait_s=remaining,
        stop_reserve_time_s=policy.command_horizon_s+brake_s,
        evaluated_not_executed=True,attempts_executed=attempts,
        motion_authorized=False,actuator_command=dict(v_m_s=0.,omega_rad_s=0.),
        scope='no executor; eligibility alone is not motion permission; no clock or distance budget reset')
