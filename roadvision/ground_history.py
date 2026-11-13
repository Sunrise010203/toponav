"""Development RGB history protocol, distinct from frozen .25s G/H/I.

Old frames keep their true times. Only the latest capture is live evidence;
the first is a bounded keyframe. All geometry is explicitly transformed to
the latest camera. Static ground and declared camera calibration are priors.
No simulator imports, scene labels, motor commands or truth enter this module.
"""
import hashlib
import math
import numpy as np

from roadvision.corridor_rgb_probe import CALIBRATION_ID, _project, _grid_count, _spread_ratio
from roadvision.rgb_ground_rig import track_region, _rectification, _region_models
from roadvision.rgb_motion_checks import split_tracks
from roadvision.prepared_descriptor_checks import filter_h, filter_i
from roadvision.paired_descriptor_checks import filter_pair
from roadvision.ground_band import track_ground_band, track_fixed_roi
from roadvision.vertical_support import estimate_supported_vertical
from roadvision.front_support import estimate_perpendicular_front
from roadvision.calibrated_ground_motion import (
    GroundCalibration, estimate_ground_motion, estimate_vertical_plane,
    select_descriptor, corridor_from_planes, _rotation,
)

HISTORY_POLICY=dict(latest_max_age_s=.25, maximum_keyframe_horizon_s=1.61,
                    maximum_gap_s=.81, protocol='rgb_ground_history_v1_development')


def validate_history(frames, *, now_s, calibration_id):
    if calibration_id!=CALIBRATION_ID:raise ValueError('Unsupported calibration')
    if type(now_s) not in (int,float) or not math.isfinite(now_s):raise ValueError('Invalid clock')
    if not isinstance(frames,list) or len(frames)!=3:raise ValueError('Expected three historical captures')
    refs=[]
    for f in frames:
        if not isinstance(f,dict) or set(f)!={'frame_id','captured_sim_s','rgb'}:raise ValueError('Only public RGB envelope allowed')
        if not isinstance(f['frame_id'],str) or not f['frame_id']:raise ValueError('Invalid ID')
        t=f['captured_sim_s'];rgb=f['rgb']
        if type(t) not in (int,float) or not math.isfinite(t) or t<0:raise ValueError('Invalid capture time')
        if not isinstance(rgb,np.ndarray) or rgb.dtype!=np.uint8 or rgb.shape!=(256,256,3):raise ValueError('Invalid RGB')
        refs.append(dict(frame_id=f['frame_id'],captured_sim_s=t,rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest()))
    times=[f['captured_sim_s'] for f in refs]
    if len({f['frame_id'] for f in refs})!=3:raise ValueError('Duplicate frame IDs')
    if any(b<=a or b-a>HISTORY_POLICY['maximum_gap_s'] for a,b in zip(times,times[1:])):raise ValueError('Unordered or gapped history')
    if times[-1]-times[0]>HISTORY_POLICY['maximum_keyframe_horizon_s']:raise ValueError('History too old')
    if not 0<=now_s-times[-1]<=HISTORY_POLICY['latest_max_age_s']:raise ValueError('Stale or future live frame')
    return refs


def transform_plane_to_latest(plane,motion):
    out=dict(plane,reference_frame='latest_capture')
    if plane['status']!='conditional_vertical_plane':return out
    r=np.eye(3);r[:2,:2]=_rotation(-motion['yaw_delta_rad'])
    t=-r@np.r_[motion['translation_m'],0.]
    n=r@np.array(plane['normal_rig']);d=plane['distance_m']+float(n@t)
    if d<=.1:return {**out,'status':'unknown','reason':'plane_crossed_or_behind','distance_m':None,'normal_rig':None}
    if 'conditional_interval' in plane:
        interval=dict(plane['conditional_interval']);cov=np.array(motion['conditional_uncertainty']['covariance_se2'])
        interval['distance_half_width_m']+=2*math.sqrt(float(np.linalg.eigvalsh(cov[:2,:2]).max()))
        interval['normal_angle_half_width_rad']+=2*math.sqrt(float(cov[2,2]))
        out['conditional_interval']=interval
        out['first_capture_interval']=plane['conditional_interval']
    return {**out,'normal_rig':n.tolist(),'distance_m':d}


def _upper_region(source):
    """Recompute spatial/temporal gates on the actual fixed upper ROI.

    The previous development code inherited a whole-image model (floor +
    walls) before selecting upper points. This is a new explicit ROI model,
    not ignoring its rejection. Descriptor filters rerun these gates too.
    """
    out={**source,'tracks':[t for t in source['tracks'] if all(8<=t[k][1]<64 for k in ('first','middle','last'))],
         'coverage_map':[[1.,0.,0.],[0.,128/56,-8*128/56],[0.,0.,1.]],'reasons':[],'status':'unknown','models':{}}
    if len(out['tracks'])<12:out['reasons']=['insufficient_upper_tracks'];return out
    a,b,c=[np.array([t[k] for t in out['tracks']]) for k in ('first','middle','last')]
    coverage=np.array(out['coverage_map'])
    if min(_grid_count(_project(coverage,p)) for p in (a,c))<6 or min(_spread_ratio(p) for p in (a,c))<.1:
        out['reasons']=['poor_spatial_support'];return out
    out['models'],out['reasons']=_region_models(a,b,c,coverage)
    if not out['reasons']:out['status']='planar_flow_candidate'
    return out


def _ground_evidence(frames,calibration):
    ground_regions={};diagnostics={}
    for view,(y,x) in {'front':(0,0),'left':(0,128),'right':(128,0)}.items():
        images=[f['rgb'][y:y+128,x:x+128] for f in frames]
        raw=track_ground_band(images,view=view)
        channels=filter_pair(images,raw,filters=(filter_h,filter_i))
        choice=select_descriptor(raw['tracks'],channels,np.array(raw['coverage_map']))
        ground_regions[view]=channels[choice['channel']]
        diagnostics[view]=dict(ground_selection=choice)
    return estimate_ground_motion(ground_regions,calibration),ground_regions,diagnostics


def _plane_evidence(frames,motion,calibration,diagnostics):
    planes={};first_planes={}
    for view,(y,x) in {'left':(0,128),'right':(128,0),'front':(0,0)}.items():
        images=[f['rgb'][y:y+128,x:x+128] for f in frames]
        bounds=(32,96,16,64) if view=='front' else (12,116,8,64)
        upper=track_fixed_roi(images,view=view,bounds=bounds)
        channels=filter_pair(images,upper,filters=(filter_h,filter_i))
        choice=select_descriptor(upper['tracks'],channels,np.array(upper['coverage_map']))
        selected=channels[choice['channel']]
        if view=='front':
            corridor=corridor_from_planes(first_planes)
            first=estimate_perpendicular_front(selected,motion,calibration,
                heading=corridor.get('heading_error_rad') or 0.,
                side_planes=first_planes if corridor['status']=='conditional_corridor_geometry' else {})
        else:first=estimate_supported_vertical(selected,motion,calibration)
        first_planes[view]=first
        planes[view]=transform_plane_to_latest(first,motion)
        diagnostics.setdefault(view,{}).update(plane_selection=choice,plane_region=selected,first_frame_plane=first)
    return planes


def _packet(refs,motion,ground_regions,planes,diagnostics):
    corridor=corridor_from_planes(planes)
    if corridor['status']=='conditional_corridor_geometry':
        intervals=[planes[v]['conditional_interval'] for v in ('left','right')]
        width_half=sum(i['distance_half_width_m'] for i in intervals)
        corridor['conditional_interval']=dict(width_half_width_m=width_half,offset_half_width_m=width_half/2,
            heading_half_width_rad=max(i['normal_angle_half_width_rad'] for i in intervals)+math.radians(corridor['parallel_gap_deg'])/2,
            certified_error_bound=False)
    front=dict(status='unknown',range_m=None,arrival_status='unverified',motion_authorized=False)
    if planes['front']['status']=='conditional_vertical_plane':
        n=np.array(planes['front']['normal_rig']);d=planes['front']['distance_m']
        if n[0]>math.cos(math.radians(15)):
            front.update(status='conditional_front_surface',range_m=float(d/n[0]),reason='surface_not_endpoint_identity')
            interval=planes['front']['conditional_interval'];angle=math.atan2(n[1],n[0]);da=interval['normal_angle_half_width_rad'];dd=interval['distance_half_width_m']
            denominators=[math.cos(angle-da),math.cos(angle+da),math.cos(angle)]
            front['range_interval_m']=[max(0.,d-dd)/max(denominators),(d+dd)/min(denominators)]
            front['certified_error_bound']=False
    return dict(version=HISTORY_POLICY['protocol'],frames=refs,policy=HISTORY_POLICY,
                ground_motion={**motion,'translation_reference_frame_id':refs[0]['frame_id'],
                               'from_time_s':refs[0]['captured_sim_s'],'to_time_s':refs[-1]['captured_sim_s']},
                vertical_planes=planes,corridor_geometry=corridor,front_surface=front,
                geometry_reference_frame_id=refs[-1]['frame_id'],geometry_time_s=refs[-1]['captured_sim_s'],
                regions=ground_regions,diagnostics=diagnostics,
                motion_authorized=False,arrival_status='unverified',atomic_edge_compatible=False,
                calibration_verified=False,static_scene_verified=False,
                declared_assumptions=['level calibrated camera','known horizontal static ground and camera height',
                                      'rigid static scene','front surface perpendicular to paired corridor sides'],
                calibration_assumption=motion.get('calibration_assumption'))


def probe_history(frames, *, now_s, calibration_id, calibration=None):
    refs=validate_history(frames,now_s=now_s,calibration_id=calibration_id)
    calibration=calibration or GroundCalibration()
    motion,regions,diagnostics=_ground_evidence(frames,calibration)
    planes=_plane_evidence(frames,motion,calibration,diagnostics)
    return _packet(refs,motion,regions,planes,diagnostics)


def compose_ground_steps(steps):
    """Chronological nonoverlapping SE2 increments; never bridge an unknown."""
    if not steps or any(s['status']!='conditional_ground_motion' for s in steps):
        return dict(status='unknown',reason='incomplete_local_odometry_chain',translation_m=None,yaw_delta_rad=None)
    translation=np.zeros(2);yaw=0.;deviations=np.zeros(3);uncertainty_available=True
    for s in steps:
        translation+=_rotation(yaw)@np.array(s['translation_m'])
        yaw+=s['yaw_delta_rad']
        if 'conditional_uncertainty' not in s:uncertainty_available=False;continue
        cov=np.array(s['conditional_uncertainty']['covariance_se2'])
        # Conservative sum of local standard deviations (not independence).
        deviations[:2]+=math.sqrt(float(np.linalg.eigvalsh(cov[:2,:2]).max()))+np.linalg.norm(s['translation_m'])*deviations[2]
        deviations[2]+=math.sqrt(float(cov[2,2]))
    out=dict(status='conditional_ground_motion',reason='composed_local_ground_rgb_increments',
                translation_m=translation.tolist(),yaw_delta_rad=float(yaw),
                motion_authorized=False,arrival_status='unverified',static_scene_verified=False,
                calibration_verified=False,local_step_count=len(steps))
    if uncertainty_available:
        out['conditional_uncertainty']=dict(covariance_se2=np.diag(deviations**2).tolist(),certified_error_bound=False,
            scope='sum of local noise standard deviations, including heading propagation; excludes prior/model error')
    if steps[0].get('calibration_assumption') is not None and all(s.get('calibration_assumption')==steps[0]['calibration_assumption'] for s in steps):
        out['calibration_assumption']=steps[0]['calibration_assumption']
    return out


def probe_chained_history(frames, *, now_s, calibration_id, calibration=None):
    """Short-baseline ground odometry and longer-baseline surface fits.

    All captures remain explicit, validated and hashed. Nonoverlapping local
    increments span at most .25s, and must ALL be supported. Plane tracks use
    the true first/middle/latest captures, not shifted timestamps.
    """
    if not isinstance(frames,list) or len(frames)<5 or len(frames)>17 or len(frames)%4!=1:
        raise ValueError('Expected 5,9,13,17 chronological RGB captures')
    keyframes=[frames[0],frames[len(frames)//2],frames[-1]]
    refs=validate_history(keyframes,now_s=now_s,calibration_id=calibration_id)
    calibration=calibration or GroundCalibration();steps=[];local=[];batches=[]
    for i in range(0,len(frames)-2,2):
        batch=frames[i:i+3]
        rr=validate_history(batch,now_s=now_s,calibration_id=calibration_id) if i==len(frames)-3 else validate_history(batch,now_s=batch[-1]['captured_sim_s'],calibration_id=calibration_id)
        if rr[-1]['captured_sim_s']-rr[0]['captured_sim_s']>.250001:raise ValueError('Local odometry span exceeds .25s')
        batches.append((batch,rr))
    if len({f['frame_id'] for f in frames})!=len(frames):raise ValueError('Duplicate history IDs')
    for batch,rr in batches:
        m,regions,diags=_ground_evidence(batch,calibration);steps.append(m)
        local.append(dict(frames=rr,motion=m,regions=regions,diagnostics=diags))
    motion=compose_ground_steps(steps);diagnostics={}
    planes=_plane_evidence(keyframes,motion,calibration,diagnostics)
    result=_packet(refs,motion,{},planes,diagnostics)
    result.update(version='rgb_chained_ground_history_v1_development',local_odometry=local)
    return result


def choose_history_model(regions):
    """Choose ONE complete model before consulting any held-out fit result."""
    scores={}
    for view,r in regions.items():
        train,_=split_tracks(r['tracks']);coverage=np.array(r.get('coverage_map',np.eye(3)))
        if train:
            a,c=[np.array([r['tracks'][i][k] for i in train]) for k in ('first','last')]
            cells=min(_grid_count(_project(coverage,p)) for p in (a,c))
        else:cells=0
        scores[view]=dict(training_track_count=len(train),training_grid_cells=cells,
                          eligible=len(train)>=6 and cells>=3)
    model='direct_long_ground' if sum(s['eligible'] for s in scores.values())>=2 else 'short_increment_chain'
    return dict(model=model,training_scores=scores,scope='training descriptor/coverage only; no heldout model result or OR of accepted models')


def probe_adaptive_history(frames,*,now_s,calibration_id,calibration=None):
    if not isinstance(frames,list) or len(frames)<5 or len(frames)>17 or len(frames)%4!=1:raise ValueError('Adaptive history requires5,9,13,17 actual captures')
    calibration=calibration or GroundCalibration();keyframes=[frames[0],frames[len(frames)//2],frames[-1]]
    refs=validate_history(keyframes,now_s=now_s,calibration_id=calibration_id)
    # Validate every non-keyframe BEFORE invoking image work, even if the
    # direct model will be chosen. Never hide malformed/replayed middle data.
    for i in range(len(frames)-2):
        validate_history(frames[i:i+3],now_s=frames[i+2]['captured_sim_s'],calibration_id=calibration_id)
    if len({f['frame_id'] for f in frames})!=len(frames):raise ValueError('Duplicate history IDs')
    motion,regions,diagnostics=_ground_evidence(keyframes,calibration)
    choice=choose_history_model(regions)
    if choice['model']=='direct_long_ground':
        planes=_plane_evidence(keyframes,motion,calibration,diagnostics)
        result=_packet(refs,motion,regions,planes,diagnostics)
    else:result=probe_chained_history(frames,now_s=now_s,calibration_id=calibration_id,calibration=calibration)
    result.update(version='rgb_adaptive_ground_history_v1_development',history_model_selection=choice)
    return result
