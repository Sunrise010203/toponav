"""Separate S2 paired-plane model, not a relaxation of frozen S1.

Parallel vertical sides are an EXPLICIT model prior, not a test of whether
arbitrary surfaces are parallel. Fit three parameters jointly on TRAINING
features; every view's heldout prediction and local conditioning must pass.
Noise intervals do not cover wrong calibration, dynamics or model identity.
"""
import math
import numpy as np
from scipy.optimize import least_squares
from roadvision.calibrated_ground_motion import _camera_motion
from roadvision.corridor_rgb_probe import _project
from roadvision.rgb_ground_rig import rig_basis
from roadvision.rgb_motion_checks import split_tracks

PAIRED_POLICY=dict(minimum_translation_m=.025,noise_floor_px=.25,
    median_px=.35,p90_px=.65,inlier_fraction=.9,maximum_distance_half_width_m=.12,
    maximum_heading_half_width_rad=math.radians(4),minimum_train=8,minimum_heldout=4)


def estimate_paired_corridor(regions,motion,calibration):
    out=dict(status='unknown',reason='metric_motion_unavailable',parallelism_assumed=True,
        certified_error_bound=False,motion_authorized=False,arrival_status='unverified',
        reference_frame='first_capture',policy=PAIRED_POLICY)
    if motion.get('status')!='conditional_ground_motion' or 'conditional_uncertainty' not in motion:return out
    covariance=np.array(motion['conditional_uncertainty']['covariance_se2'],float)
    if covariance.shape!=(3,3) or not np.isfinite(covariance).all() or np.linalg.eigvalsh(covariance).min()<-1e-10:return out
    if not .025<=np.linalg.norm(motion['translation_m'])<=.4:return out
    k=np.array([[calibration.focal_px,0,calibration.principal_px],[0,calibration.focal_px,calibration.principal_px],[0,0,1.]])
    ki=np.linalg.inv(k);data={}
    for view in ('left','right'):
        r=regions.get(view)
        if r is None:return {**out,'reason':'missing_side'}
        if r.get('view')!=view or any(v not in {'rotation_or_low_parallax','insufficient_image_displacement'} for v in r.get('reasons',[])):return {**out,'reason':'source_spatial_or_temporal_rejection'}
        train,heldout=split_tracks(r['tracks'])
        if len(train)<8 or len(heldout)<4:return {**out,'reason':'insufficient_per_view_features'}
        a,b=[np.array([t[q] for t in r['tracks']],float) for q in ('first','last')]
        data[view]=dict(a=a,b=b,train=train,heldout=heldout)
    def fit(m,initial):
        cameras={v:_camera_motion(v,m) for v in data}
        def predict(q,view,ids):
            normal=np.array([math.sin(q[0]),math.cos(q[0]),0.])*(1 if view=='left' else -1)
            distance=q[1 if view=='left' else 2];rot,t=cameras[view]
            return _project(k@(rot+np.outer(t,rig_basis(view).T@normal/distance))@ki,data[view]['a'][ids])
        def residual(q):
            values=[]
            for v,d in data.items():
                pred=predict(q,v,d['train'])
                values.extend((np.full((len(d['train']),2),1000.) if pred is None else pred-d['b'][d['train']]).ravel())
            return np.array(values)
        result=least_squares(residual,initial,bounds=([-.6,.25,.25],[.6,3.5,3.5]),loss='soft_l1',f_scale=.25,max_nfev=80)
        return result,predict
    f,predict=fit(motion,np.array([0.,1.,1.]))
    if not f.success or not np.isfinite(f.x).all():return {**out,'reason':'optimizer_failed'}
    theta,dl,dr=f.x
    metrics={}
    for v,d in data.items():
        normal=np.array([math.sin(theta),math.cos(theta),0.])*(1 if v=='left' else -1)
        rays=np.column_stack((d['a'],np.ones(len(d['a']))))@ki.T
        if np.any(rays@(rig_basis(v).T@normal)<=1e-4):return {**out,'reason':'side_cheirality_failed'}
        metrics[v]={}
        for split in ('train','heldout'):
            ids=d[split];prediction=predict(f.x,v,ids)
            if prediction is None:return {**out,'reason':'invalid_projection'}
            errors=np.linalg.norm(prediction-d['b'][ids],axis=1)
            metrics[v][split]=dict(median_px=float(np.median(errors)),p90_px=float(np.quantile(errors,.9)),inlier_fraction=float(np.mean(errors<=.75)))
    out['metrics']=metrics
    if any(s['median_px']>.35 or s['p90_px']>.65 or s['inlier_fraction']<.9 for d in metrics.values() for s in d.values()):return {**out,'reason':'paired_prediction_failed'}
    information=f.jac.T@f.jac
    if np.linalg.matrix_rank(information)<3:return {**out,'reason':'rank_deficient'}
    cov=np.linalg.inv(information)*max(.25**2,float(np.mean(f.fun**2)))
    half=2*np.sqrt(np.diag(cov));eigenvalues,eigenvectors=np.linalg.eigh(covariance)
    for j in range(3):
        delta=2*math.sqrt(max(0,eigenvalues[j]))*eigenvectors[:,j];deviations=[]
        for sign in (-1,1):
            pose=np.r_[motion['translation_m'],motion['yaw_delta_rad']]+sign*delta
            perturbed={**motion,'translation_m':pose[:2].tolist(),'yaw_delta_rad':float(pose[2])}
            fp,_=fit(perturbed,f.x)
            if not fp.success:return {**out,'reason':'pose_perturbation_failed'}
            deviations.append(np.abs(fp.x-f.x))
        half+=np.max(deviations,axis=0)
    interval=dict(heading_half_width_rad=float(half[0]),distance_half_widths_m=half[1:].tolist(),certified_error_bound=False,
        scope='local pixel noise and ground-pose perturbations; excludes prior/model errors')
    out['conditional_interval']=interval
    if half[0]>math.radians(4) or max(half[1:])>.12:return {**out,'reason':'paired_interval_too_wide'}
    if not 1.2<=dl+dr<=4.:return {**out,'reason':'width_outside_domain'}
    return {**out,'status':'conditional_paired_corridor','reason':'joint_parallel_vertical_model_with_per_view_heldout_support',
        'heading_first_rad':float(theta),'distances_first_m':[float(dl),float(dr)]}
