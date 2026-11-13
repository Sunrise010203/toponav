"""S2 fixed paired-corridor producer over frozen S1 RGB motion/descriptors.

Always selects the paired-parallel model; never ORs independent S1 fits.
No simulator or task node coordinates. Original producer remains frozen.
"""
import math
import numpy as np
from roadvision.calibrated_ground_motion import GroundCalibration
from roadvision.ground_history import transform_plane_to_latest,_packet
from roadvision.straight_history_support import prepare_straight_history
from roadvision.paired_corridor import estimate_paired_corridor
from roadvision.joint_straight_model import estimate_joint_corridor
from roadvision.front_support import estimate_perpendicular_front


def probe_straight_history(frames,*,now_s,calibration_id,calibration=None,side_roi='central'):
    calibration=calibration or GroundCalibration()
    original=prepare_straight_history(frames,now_s=now_s,calibration_id=calibration_id,calibration=calibration,side_roi=side_roi)
    motion=original['ground_motion'];diags=original['diagnostics']
    sides={v:diags[v]['plane_region'] for v in ('left','right')}
    # Choose by S1 TRAINING-only history selection, not acceptance results.
    # In direct mode, refit all raw ground + wall training evidence together.
    # In chain mode, keep its conditioned pose fixed. No failure fallback.
    if original['history_model_selection']['model']=='direct_long_ground':
        paired=estimate_joint_corridor(sides,original['regions'],motion,calibration)
    else:
        paired=estimate_paired_corridor(sides,motion,calibration)
    if paired['status']=='conditional_paired_corridor' and 'refined_motion' in paired:
        motion=paired['refined_motion']
    first={};planes={}
    for i,v in enumerate(('left','right')):
        p=dict(status='unknown',reason='paired_side_model_unavailable',normal_rig=None,distance_m=None)
        if paired['status']=='conditional_paired_corridor':
            theta=paired['heading_first_rad'];sign=1 if v=='left' else -1;interval=paired['conditional_interval']
            p.update(status='conditional_vertical_plane',reason='joint_parallel_sides_prior',normal_rig=(np.array([math.sin(theta),math.cos(theta),0.])*sign).tolist(),
                distance_m=paired['distances_first_m'][i],conditional_interval=dict(distance_half_width_m=interval['distance_half_widths_m'][i],normal_angle_half_width_rad=interval['heading_half_width_rad'],certified_error_bound=False),reference_frame='first_capture')
        first[v]=p;planes[v]=transform_plane_to_latest(p,motion)
    front=estimate_perpendicular_front(diags['front']['plane_region'],motion,calibration,
        heading=paired.get('heading_first_rad',0.),side_planes=first if paired['status']=='conditional_paired_corridor' else {})
    planes['front']=transform_plane_to_latest(front,motion)
    result=_packet(original['frames'],motion,original['regions'],planes,diags)
    # S1 interval arithmetic can leave NumPy scalar objects in a list. The
    # strict controller deliberately rejects non-native scalar objects.
    # Canonicalize only losslessly representable floating types at this S2
    # producer boundary, not arbitrary values at the safety gate. NaN/Inf
    # remain invalid; bool/string/complex/long-double inputs are not coerced.
    interval=result['front_surface'].get('range_interval_m')
    if isinstance(interval,list) and len(interval)==2:
        result['front_surface']['range_interval_m']=[
            float(v) if type(v) in (np.float16,np.float32,np.float64) else v for v in interval]
    result.update(version='rgb_straight_joint_history_v1',paired_corridor=paired,
        s1_initial_ground_motion=original['ground_motion'],
        history_model_selection=original['history_model_selection'],
        declared_assumptions=original.get('declared_assumptions',[])+['paired corridor sides assumed parallel'])
    return result
