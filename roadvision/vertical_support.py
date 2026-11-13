"""Conditional *vertical-constrained* metric plane with error conditioning.

Alternative development estimator, NOT a relaxed version of the v5 residual
ratio gate. The old estimator/tests remain. Here measured ground SE2 locks
rotation, the vertical assumption removes a nuisance dimension, and pixel
noise plus ground-pose perturbations bound the admitted parameter interval.
These local bounds do not certify calibration/static scene/model identity.
"""
import math
import numpy as np
from scipy.optimize import least_squares
from roadvision.calibrated_ground_motion import _camera_motion
from roadvision.corridor_rgb_probe import _project
from roadvision.rgb_ground_rig import rig_basis
from roadvision.rgb_motion_checks import split_tracks

VERTICAL_POLICY=dict(minimum_translation_m=.025,pixel_noise_floor_px=.25,
                     median_px=.35,p90_px=.65,inlier_fraction=.9,
                     maximum_distance_half_width_m=.12,maximum_angle_half_width_rad=math.radians(4),
                     minimum_distance_m=.25,maximum_distance_m=5.)


def estimate_supported_vertical(region,motion,calibration):
    unknown=dict(status='unknown',reason='ground_motion_uncertain',distance_m=None,normal_rig=None,
                 motion_authorized=False,arrival_status='unverified',reference_frame='first_capture')
    if motion['status']!='conditional_ground_motion' or 'conditional_uncertainty' not in motion:return unknown
    cov=np.array(motion['conditional_uncertainty']['covariance_se2'],float)
    if cov.shape!=(3,3) or not np.isfinite(cov).all() or np.linalg.eigvalsh(cov).min()<-1e-10:return unknown
    if np.linalg.norm(motion['translation_m'])<VERTICAL_POLICY['minimum_translation_m']:return unknown
    if any(r not in {'rotation_or_low_parallax','insufficient_image_displacement'} for r in region.get('reasons',[])):
        return {**unknown,'reason':'source_spatial_or_temporal_model_rejected'}
    tracks=region['tracks'];train,validation=split_tracks(tracks)
    if len(train)<8 or len(validation)<4:return {**unknown,'reason':'insufficient_plane_tracks'}
    a,c=[np.array([t[k] for t in tracks],float) for k in ('first','last')]
    k=np.array([[calibration.focal_px,0,calibration.principal_px],[0,calibration.focal_px,calibration.principal_px],[0,0,1.]])
    ki=np.linalg.inv(k);basis=rig_basis(region['view']);rays=np.column_stack((a,np.ones(len(a))))@ki.T
    def fit(m,indices,initial=None):
        rot,t=_camera_motion(region['view'],m)
        def predict(q,ids):return _project(k@(rot+np.outer(t,basis.T@np.r_[q,0.]))@ki,a[ids])
        if initial is None:
            rotated=rays@rot.T;normalized=np.column_stack((c,np.ones(len(c))))@ki.T
            u,v=normalized[:,0],normalized[:,1]
            design=np.concatenate(((t[0]-u[indices]*t[2])[:,None]*rays[indices],(t[1]-v[indices]*t[2])[:,None]*rays[indices]))@basis.T[:,:2]
            rhs=np.r_[u[indices]*rotated[indices,2]-rotated[indices,0],v[indices]*rotated[indices,2]-rotated[indices,1]]
            if np.linalg.matrix_rank(design)<2:return None,None
            initial=np.linalg.lstsq(design,rhs,rcond=None)[0]
        def residual(q):
            prediction=predict(q,indices)
            return (np.full((len(indices),2),1000.) if prediction is None else prediction-c[indices]).ravel()
        return least_squares(residual,initial,loss='soft_l1',f_scale=.25,max_nfev=60),predict
    fitted,predict=fit(motion,train)
    if fitted is None or not fitted.success:return {**unknown,'reason':'vertical_fit_failed'}
    q=fitted.x;norm=np.linalg.norm(q)
    if not np.isfinite(q).all() or norm<1e-5:return {**unknown,'reason':'invalid_vertical_plane'}
    distance=1/norm;normal=np.r_[q/norm,0.]
    if not .25<=distance<=5. or np.any(rays@(basis.T@np.r_[q,0.])<=1e-4):
        return {**unknown,'reason':'plane_cheirality_or_distance_failed'}
    metrics=[]
    for ids in (train,validation):
        prediction=predict(q,ids)
        if prediction is None:return {**unknown,'reason':'invalid_plane_projection'}
        errors=np.linalg.norm(prediction-c[ids],axis=1)
        metrics.append(dict(median_px=float(np.median(errors)),p90_px=float(np.quantile(errors,.9)),inlier_fraction=float(np.mean(errors<=.75))))
    diag=dict(training_metrics=metrics[0],heldout_metrics=metrics[1],fit_track_indices=train,validation_track_indices=validation)
    if any(m['median_px']>.35 or m['p90_px']>.65 or m['inlier_fraction']<.9 for m in metrics):
        return {**unknown,**diag,'reason':'constrained_plane_prediction_failed'}
    information=fitted.jac.T@fitted.jac
    if np.linalg.matrix_rank(information)<2:return {**unknown,**diag,'reason':'plane_rank_deficient'}
    qcov=np.linalg.inv(information)*max(.25**2,float(np.mean(fitted.fun**2)))
    # Noise + perturbations of the inferred ground pose. Retain systematic
    # model limitation; this is not a statistical coverage certification.
    gradient_d=-q/norm**3;gradient_angle=np.array([-q[1],q[0]])/norm**2
    half_d=2*float(np.sqrt(gradient_d@qcov@gradient_d));half_a=2*float(np.sqrt(gradient_angle@qcov@gradient_angle))
    eigenvalues,eigenvectors=np.linalg.eigh(cov)
    for j in range(3):
        delta=2*math.sqrt(max(0,eigenvalues[j]))*eigenvectors[:,j]
        deviations_d=[];deviations_a=[]
        for sign in (-1,1):
            pose=np.r_[motion['translation_m'],motion['yaw_delta_rad']]+sign*delta
            perturbed={**motion,'translation_m':pose[:2].tolist(),'yaw_delta_rad':float(pose[2])}
            f,_=fit(perturbed,train,q)
            if f is None or not f.success or np.linalg.norm(f.x)<1e-5:return {**unknown,**diag,'reason':'pose_perturbation_failed'}
            deviations_d.append(abs(1/np.linalg.norm(f.x)-distance))
            deviations_a.append(abs(math.atan2(np.cross(q,f.x),q@f.x)))
        half_d+=max(deviations_d);half_a+=max(deviations_a)
    interval=dict(distance_half_width_m=half_d,normal_angle_half_width_rad=half_a,
                  pixel_noise_floor_px=.25,certified_error_bound=False,
                  scope='local pixel and ground-pose perturbations; excludes prior/model mismatch')
    diag['conditional_interval']=interval
    if half_d>.12 or half_a>math.radians(4):return {**unknown,**diag,'reason':'metric_plane_interval_too_wide'}
    return {**unknown,**diag,'status':'conditional_vertical_plane','reason':'vertical_prior_and_conditioned_metric_interval',
            'normal_rig':normal.tolist(),'distance_m':float(distance),'static_scene_verified':False,'calibration_verified':False}
