"""S2 orchestration of FROZEN S1 ground/descriptor primitives, no plane fits.

Identical training-only direct-vs-chain selection, true timestamps and
per-increment requirement; avoids fitting unused independent S1 wall/front
planes before the S2 joint fit. Never uses an old S1 acceptance as fallback.
"""
import numpy as np
from roadvision.calibrated_ground_motion import GroundCalibration,select_descriptor
from roadvision.ground_history import validate_history,_ground_evidence,choose_history_model,compose_ground_steps
from roadvision.ground_band import track_fixed_roi
from roadvision.prepared_descriptor_checks import filter_h, filter_i
from roadvision.paired_descriptor_checks import filter_pair

SIDE_ROIS = {'central': (32,96,8,64), 'wide': (12,116,8,64)}


def prepare_straight_history(frames,*,now_s,calibration_id,calibration=None,side_roi='central'):
    # Explicit fixed hypotheses, selected BEFORE image work. Never retry the
    # central model after a wider region fails at an end-wall corner.
    if type(side_roi) is not str or side_roi not in SIDE_ROIS:raise ValueError('Unknown side ROI')
    if not isinstance(frames,list) or len(frames) not in (5,9,13,17):raise ValueError('Expected5/9/13/17 real chronological captures')
    keyframes=[frames[0],frames[len(frames)//2],frames[-1]]
    refs=validate_history(keyframes,now_s=now_s,calibration_id=calibration_id)
    for i in range(len(frames)-2):validate_history(frames[i:i+3],now_s=frames[i+2]['captured_sim_s'],calibration_id=calibration_id)
    if len({f['frame_id'] for f in frames})!=len(frames):raise ValueError('Duplicate history IDs')
    calibration=calibration or GroundCalibration()
    motion,regions,diags=_ground_evidence(keyframes,calibration);choice=choose_history_model(regions);local=[]
    if choice['model']=='short_increment_chain':
        steps=[]
        for i in range(0,len(frames)-2,2):
            batch=frames[i:i+3]
            if batch[-1]['captured_sim_s']-batch[0]['captured_sim_s']>.250001:raise ValueError('Local odometry span exceeds.25s')
            m,_,_=_ground_evidence(batch,calibration);steps.append(m)
            local.append(dict(frame_ids=[f['frame_id'] for f in batch],motion=m))
        motion=compose_ground_steps(steps);regions={};diags={}
    motion={**motion,'translation_reference_frame_id':refs[0]['frame_id'],'from_time_s':refs[0]['captured_sim_s'],'to_time_s':refs[-1]['captured_sim_s']}
    for view,(y,x) in {'left':(0,128),'right':(128,0),'front':(0,0)}.items():
        if motion['status']!='conditional_ground_motion':
            diags.setdefault(view,{})['plane_region']=dict(view=view,tracks=[],reasons=['ground_motion_unavailable'])
            continue
        images=[f['rgb'][y:y+128,x:x+128] for f in keyframes]
        # Central is the unchanged default; wide is an explicit experiment.
        # Neither fixed hypothesis is a wall mask. Wider sampling can include
        # end-wall corners; all train/heldout/conditioning gates remain.
        bounds=(32,96,16,64) if view=='front' else SIDE_ROIS[side_roi]
        raw=track_fixed_roi(images,view=view,bounds=bounds)
        channels=filter_pair(images,raw,filters=(filter_h,filter_i))
        selected=select_descriptor(raw['tracks'],channels,np.array(raw['coverage_map']))
        diags.setdefault(view,{}).update(plane_selection=selected,plane_region=channels[selected['channel']])
    return dict(frames=refs,ground_motion=motion,regions=regions,diagnostics=diags,
        history_model_selection=choice,local_odometry=local,
        declared_assumptions=['level calibrated camera','known horizontal static ground and camera height','rigid static scene','front surface perpendicular to paired corridor sides'])
