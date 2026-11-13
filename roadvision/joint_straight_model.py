"""S2 joint ground/paired-wall TRAINING fit, per-view heldout checks.

A fresh supported S1 direct ground model is mandatory. This estimator does
NOT rescue rejected motion or retry alternate surface models after failure.
All six unknowns (camera SE2, shared wall angle, two wall distances) use raw
training correspondences once, avoiding a double-counted S1 Gaussian prior.
Parallel/vertical/static/calibration assumptions remain unverified priors.
"""
import math
import numpy as np
from scipy.optimize import least_squares
from roadvision.calibrated_ground_motion import _camera_motion,pixel_predictions
from roadvision.corridor_rgb_probe import _project
from roadvision.rgb_ground_rig import rig_basis
from roadvision.rgb_motion_checks import split_tracks


def estimate_joint_corridor(regions,ground_regions,motion,calibration):
    out=dict(status='unknown',reason='supported_direct_ground_required',parallelism_assumed=True,
        certified_error_bound=False,motion_authorized=False,arrival_status='unverified',reference_frame='first_capture')
    if motion.get('status')!='conditional_ground_motion' or 'conditional_uncertainty' not in motion:return out
    if not .025<=np.linalg.norm(motion['translation_m'])<=.4:return out
    data={};k=np.array([[calibration.focal_px,0,calibration.principal_px],[0,calibration.focal_px,calibration.principal_px],[0,0,1.]])
    ki=np.linalg.inv(k)
    for kind,sources in (('ground',ground_regions),('wall',regions)):
        for view,r in sources.items():
            if view not in ('front','left','right') or r.get('view')!=view:return {**out,'reason':'invalid_region_view'}
            if kind=='wall' and (view not in ('left','right') or any(v not in {'rotation_or_low_parallax','insufficient_image_displacement'} for v in r.get('reasons',[]))):return {**out,'reason':'source_spatial_or_temporal_rejection'}
            train,heldout=split_tracks(r['tracks']);minimum=8 if kind=='wall' else 6
            if kind=='ground' and 'region_checks' in motion:
                prepared={r['region'] for r in motion['region_checks'] if r['accepted']}
                if view not in prepared:continue
            if kind=='ground' and 'fit_track_ids' in motion:
                retained=set(motion['fit_track_ids'])
                train=[i for i in train if f'{view}:{i}' in retained]
                minimum=4
            if len(train)<minimum or len(heldout)<4:
                if kind=='wall':return {**out,'reason':'insufficient_side_features'}
                continue
            a,b=[np.array([t[q] for t in r['tracks']],float) for q in ('first','last')]
            if not np.isfinite(a).all() or not np.isfinite(b).all():return {**out,'reason':'nonfinite_pixels'}
            data[kind,view]=dict(a=a,b=b,train=train,heldout=heldout)
    if any(('wall',v) not in data for v in ('left','right')) or sum(kind=='ground' for kind,v in data)<2:return out
    def prediction(q,key,ids):
        kind,view=key;d=data[key]
        if kind=='ground':return pixel_predictions(d['a'][ids],view,q[:3],calibration)
        pose={**motion,'translation_m':q[:2].tolist(),'yaw_delta_rad':float(q[2])};rot,t=_camera_motion(view,pose)
        normal=np.array([math.sin(q[3]),math.cos(q[3]),0.])*(1 if view=='left' else -1)
        distance=q[4 if view=='left' else 5]
        return _project(k@(rot+np.outer(t,rig_basis(view).T@normal/distance))@ki,d['a'][ids])
    def residual(q):
        vals=[]
        for key,d in data.items():
            p=prediction(q,key,d['train'])
            error=np.full((len(d['train']),2),1000.) if p is None else p-d['b'][d['train']]
            vals.extend(np.nan_to_num(error,nan=1000.,posinf=1000.,neginf=-1000.).ravel())
        return np.array(vals)
    initial=np.r_[motion['translation_m'],motion['yaw_delta_rad'],0.,1.,1.]
    fit=least_squares(residual,initial,bounds=([-.5,-.5,-.5,-.6,.25,.25],[.5,.5,.5,.6,3.5,3.5]),loss='soft_l1',f_scale=.25,max_nfev=100)
    if not fit.success or not np.isfinite(fit.x).all():return {**out,'reason':'joint_optimizer_failed'}
    q=fit.x;metrics={}
    for key,d in data.items():
        kind,view=key;metrics[f'{kind}_{view}']={}
        if kind=='wall':
            normal=np.array([math.sin(q[3]),math.cos(q[3]),0.])*(1 if view=='left' else -1)
            rays=np.column_stack((d['a'],np.ones(len(d['a']))))@ki.T
            if np.any(rays@(rig_basis(view).T@normal)<=1e-4):return {**out,'reason':'side_cheirality_failed'}
        for split in ('train','heldout'):
            ids=d[split];p=prediction(q,key,ids)
            if p is None or not np.isfinite(p).all():return {**out,'reason':'invalid_projection'}
            errors=np.linalg.norm(p-d['b'][ids],axis=1)
            metrics[f'{kind}_{view}'][split]=dict(median_px=float(np.median(errors)),p90_px=float(np.quantile(errors,.9)),inlier_fraction=float(np.mean(errors<=(1. if kind=='ground' else .75))))
    out['metrics']=metrics
    for key,d in metrics.items():
        med,p90,fraction=(.75,1.5,.8) if key.startswith('ground_') else (.35,.65,.9)
        if any(v['median_px']>med or v['p90_px']>p90 or v['inlier_fraction']<fraction for v in d.values()):return {**out,'reason':'joint_per_view_prediction_failed'}
    information=fit.jac.T@fit.jac
    if np.linalg.matrix_rank(information)<6:return {**out,'reason':'joint_rank_deficient'}
    cov=np.linalg.inv(information)*max(.25**2,float(np.mean(fit.fun**2)));half=2*np.sqrt(np.diag(cov))
    interval=dict(heading_half_width_rad=float(half[3]),distance_half_widths_m=half[4:].tolist(),certified_error_bound=False,
        scope='joint linearized pixel noise; excludes calibration/static/parallel/surface-model error')
    out['conditional_interval']=interval
    if half[3]>math.radians(4) or max(half[4:])>.12 or max(half[:2])>.03 or half[2]>math.radians(2):return {**out,'reason':'joint_interval_too_wide'}
    if not .025<=np.linalg.norm(q[:2])<=.4 or not 1.2<=sum(q[4:])<=4.:return {**out,'reason':'joint_outside_domain'}
    refined={key:motion[key] for key in ('calibration_assumption','translation_reference_frame_id','from_time_s','to_time_s') if key in motion}
    refined.update(status='conditional_ground_motion',translation_m=q[:2].tolist(),yaw_delta_rad=float(q[2]),
        motion_authorized=False,arrival_status='unverified',calibration_verified=False,static_scene_verified=False,
        atomic_edge_compatible=False,
        reason='joint_ground_paired_wall_training_with_per_view_heldout',
        conditional_uncertainty=dict(covariance_se2=cov[:3,:3].tolist(),pixel_noise_floor_px=.25,certified_error_bound=False,
            scope='joint linearized pixel noise; excludes prior/model errors'))
    return {**out,'status':'conditional_paired_corridor','reason':'joint_ground_parallel_wall_model',
        'heading_first_rad':float(q[3]),'distances_first_m':q[4:].tolist(),'refined_motion':refined}
