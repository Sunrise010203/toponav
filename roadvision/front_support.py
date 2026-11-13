"""Manhattan front-surface hypothesis conditioned on TWO side directions.

This adds a declared perpendicular-front prior; it is not general obstacle
segmentation, a verified Manhattan world or endpoint identity. Distance is
estimated from RGB transfer; no map distances or scene IDs are read.
"""
import math
import numpy as np
from scipy.optimize import least_squares
from roadvision.calibrated_ground_motion import _camera_motion
from roadvision.corridor_rgb_probe import _project
from roadvision.rgb_ground_rig import rig_basis
from roadvision.rgb_motion_checks import split_tracks


def estimate_perpendicular_front(region,motion,calibration,*,heading,side_planes):
    out=dict(status='unknown',reason='two_side_directions_or_motion_unavailable',normal_rig=None,distance_m=None,
             reference_frame='first_capture',motion_authorized=False,arrival_status='unverified')
    if (motion['status']!='conditional_ground_motion' or 'conditional_uncertainty' not in motion or
        any(side_planes.get(v,{}).get('status')!='conditional_vertical_plane' for v in ('left','right'))):return out
    if np.linalg.norm(motion['translation_m'])<.025:return out
    if any(r not in {'rotation_or_low_parallax','insufficient_image_displacement'} for r in region.get('reasons',[])):
        return {**out,'reason':'source_spatial_or_temporal_model_rejected'}
    tracks=region['tracks'];train,validation=split_tracks(tracks)
    if len(train)<8 or len(validation)<4:return {**out,'reason':'insufficient_plane_tracks'}
    a,c=[np.array([t[k] for t in tracks],float) for k in ('first','last')]
    k=np.array([[calibration.focal_px,0,calibration.principal_px],[0,calibration.focal_px,calibration.principal_px],[0,0,1.]])
    ki=np.linalg.inv(k);basis=rig_basis(region['view'])
    def fit(m,theta):
        normal=np.array([math.cos(theta),-math.sin(theta),0.]);rot,t=_camera_motion(region['view'],m)
        def predict(inv_d,ids):return _project(k@(rot+np.outer(t,basis.T@normal*inv_d[0]))@ki,a[ids])
        def residual(inv_d):
            p=predict(inv_d,train)
            return (np.full((len(train),2),1000.) if p is None else p-c[train]).ravel()
        f=least_squares(residual,[.8],bounds=(.125,4.),loss='soft_l1',f_scale=.25,max_nfev=60)
        return f,predict,normal
    f,predict,n=fit(motion,heading)
    if not f.success or f.x[0]<=.12501 or f.x[0]>=3.99999:return {**out,'reason':'front_fit_failed_or_out_of_domain'}
    distance=1/f.x[0];metrics=[]
    for ids in (train,validation):
        prediction=predict(f.x,ids)
        if prediction is None:return {**out,'reason':'invalid_front_projection'}
        e=np.linalg.norm(prediction-c[ids],axis=1)
        metrics.append(dict(median_px=float(np.median(e)),p90_px=float(np.quantile(e,.9)),inlier_fraction=float(np.mean(e<=.75))))
    diag=dict(training_metrics=metrics[0],heldout_metrics=metrics[1],fit_track_indices=train,validation_track_indices=validation,
              declared_additional_prior='front surface perpendicular to corridor side planes')
    if any(m['median_px']>.35 or m['p90_px']>.65 or m['inlier_fraction']<.9 for m in metrics):return {**out,**diag,'reason':'front_prediction_failed'}
    information=float((f.jac.T@f.jac)[0,0])
    if information<1e-8:return {**out,**diag,'reason':'front_range_rank_deficient'}
    half=2*math.sqrt(max(.25**2,float(np.mean(f.fun**2)))/information)/f.x[0]**2
    cov=np.array(motion['conditional_uncertainty']['covariance_se2']);values,vectors=np.linalg.eigh(cov)
    for j in range(3):
        delta=2*math.sqrt(max(0,values[j]))*vectors[:,j];errors=[]
        for sign in (-1,1):
            p=np.r_[motion['translation_m'],motion['yaw_delta_rad']]+sign*delta
            ff,_,_=fit({**motion,'translation_m':p[:2].tolist(),'yaw_delta_rad':float(p[2])},heading)
            if not ff.success:return {**out,**diag,'reason':'front_perturbation_failed'}
            errors.append(abs(1/ff.x[0]-distance))
        half+=max(errors)
    angle=max(side_planes[v]['conditional_interval']['normal_angle_half_width_rad'] for v in ('left','right'))
    errors=[]
    for sign in (-1,1):
        ff,_,_=fit(motion,heading+sign*angle)
        errors.append(abs(1/ff.x[0]-distance))
    half+=max(errors)
    interval=dict(distance_half_width_m=half,normal_angle_half_width_rad=angle,
                  pixel_noise_floor_px=.25,certified_error_bound=False,
                  scope='conditional on perpendicular-front prior; excludes prior/model mismatch')
    diag['conditional_interval']=interval
    if half>.12:return {**out,**diag,'reason':'front_range_interval_too_wide'}
    return {**out,**diag,'status':'conditional_vertical_plane','reason':'perpendicular_front_with_local_range_interval',
            'normal_rig':n.tolist(),'distance_m':float(distance),'static_scene_verified':False,'calibration_verified':False}
