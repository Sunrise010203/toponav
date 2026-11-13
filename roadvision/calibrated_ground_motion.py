"""Conditional metric SE(2) from ground RGB rays and declared calibration.

Height, level ground, rigid static scene and camera intrinsics are ASSUMPTIONS,
not certified by these images. No control, simulator, poses or scene labels.
Descriptor policy is selected using training tracks only, never an OR of the
H/I accepted sets. The internal feature check is not a pipeline-independent
dataset. All motion authorization and arrival outputs remain conservative.
"""
from dataclasses import asdict, dataclass
import itertools
import math

import numpy as np
from scipy.optimize import least_squares

from roadvision.corridor_rgb_probe import _grid_count, _spread_ratio, _project, _rotation_residual
from roadvision.rgb_ground_rig import probe_ground_rig, rig_basis, _rectification
from roadvision.rgb_motion_checks import filter_region as filter_h, split_tracks
from roadvision.rgb_resampling_checks import filter_region as filter_i


GROUND_POLICY = dict(minimum_train_per_view=6, minimum_validation_per_view=4,
                     minimum_views=2, minimum_tracks_per_view=12,
                     minimum_grid_cells=6, minimum_spread=.1,
                     ransac_trials=192, ransac_seed=1721, ransac_px=1.,
                     minimum_train_fraction=.8, validation_median_px=.75,
                     validation_p90_px=1.5, validation_inlier_fraction=.8,
                     max_evaluations=80)


@dataclass(frozen=True)
class GroundCalibration:
    height_m: float = .45
    focal_px: float = 64.
    principal_px: float = 64.

    def __post_init__(self):
        for key,value in asdict(self).items():
            if type(value) not in (float,int) or not math.isfinite(value) or value<=0:
                raise ValueError(f'Invalid declared calibration: {key}')


def ground_points(pixels, view, calibration):
    pixels=np.asarray(pixels,float)
    if pixels.ndim!=2 or pixels.shape[1]!=2 or not np.isfinite(pixels).all():
        raise ValueError('Expected finite Nx2 image points')
    normalized=(pixels-calibration.principal_px)/calibration.focal_px
    rays=np.column_stack((normalized,np.ones(len(pixels))))@rig_basis(view).T
    if np.any(rays[:,2]>=-1e-6):raise ValueError('Ground rays must lie below the declared horizon')
    return rays[:,:2]*(-calibration.height_m/rays[:,2,None])


def ground_pixels(points, view, calibration):
    points=np.asarray(points,float)
    xyz=np.column_stack((points,np.full(len(points),-calibration.height_m)))@rig_basis(view)
    result=np.full((len(points),2),np.nan)
    good=xyz[:,2]>1e-6
    result[good]=calibration.focal_px*xyz[good,:2]/xyz[good,2,None]+calibration.principal_px
    return result


def _rotation(yaw):
    return np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])


def fit_se2(first, last):
    """Returns camera translation in FIRST rig XY, then camera yaw change."""
    a,b=np.asarray(first,float),np.asarray(last,float)
    if len(a)<2 or a.shape!=b.shape or not np.isfinite(a).all() or not np.isfinite(b).all():return None
    ac,bc=a-a.mean(axis=0),b-b.mean(axis=0)
    if min(np.linalg.norm(ac),np.linalg.norm(bc))<1e-8:return None
    u,_,vt=np.linalg.svd(ac.T@bc)
    q=u@np.diag([1.,np.linalg.det(u@vt)])@vt
    translation=a.mean(axis=0)-b.mean(axis=0)@q.T
    return np.r_[translation,math.atan2(q[1,0],q[0,0])]


def pixel_predictions(first, view, pose, calibration):
    xy=ground_points(first,view,calibration)
    return ground_pixels((xy-np.array(pose[:2]))@_rotation(pose[2]),view,calibration)


def select_descriptor(tracks, channels, coverage):
    """Select exactly one complete descriptor policy using FIT evidence only."""
    train,_=split_tracks(tracks);scores={}
    for name,channel in channels.items():
        accepted={e['track_index'] for e in channel['descriptor_evidence'] if e['accepted']}
        indices=[i for i in train if i in accepted]
        if indices:
            cells=min(_grid_count(_project(coverage,np.array([tracks[i][k] for i in indices],float))) for k in ('first','last'))
        else:cells=0
        scores[name]=dict(train_accepted=len(indices),train_grid_cells=cells)
    # Stable H tie-break; validation feature acceptance/scores not consulted.
    chosen=max(channels,key=lambda n:(scores[n]['train_grid_cells'],scores[n]['train_accepted'],n=='H'))
    return dict(channel=chosen,training_scores=scores,fit_track_indices=train,
                scope='one policy per view selected using preassigned training indices only')


def _base(reason):
    return dict(status='unknown',reason=reason,translation_m=None,yaw_delta_rad=None,
                scale_source='declared_camera_height_m',static_scene_verified=False,
                calibration_verified=False,motion_authorized=False,arrival_status='unverified',
                atomic_edge_compatible=False)


def estimate_ground_motion(regions, calibration):
    result=_base('insufficient_cross_view_support');eligible=[];checks=[]
    for key,region in regions.items():
        tracks=region['tracks'];train,validation=split_tracks(tracks)
        check=dict(region=key,accepted=False,track_count=len(tracks),train_count=len(train),validation_count=len(validation))
        checks.append(check)
        if (len(tracks)<GROUND_POLICY['minimum_tracks_per_view'] or
                len(train)<GROUND_POLICY['minimum_train_per_view'] or len(validation)<GROUND_POLICY['minimum_validation_per_view']):continue
        a,c=[np.array([t[k] for t in tracks],float) for k in ('first','last')]
        coverage=np.array(region.get('coverage_map',np.eye(3)),float)
        if (min(_grid_count(_project(coverage,p)) for p in (a,c))<GROUND_POLICY['minimum_grid_cells'] or
                min(_spread_ratio(p) for p in (a,c))<GROUND_POLICY['minimum_spread']):
            check['reason']='poor_spatial_support';continue
        try:xy_a,xy_c=ground_points(a,region['view'],calibration),ground_points(c,region['view'],calibration)
        except ValueError:check['reason']='invalid_ground_rays';continue
        check.update(accepted=True,reason='prepared')
        eligible.append(dict(key=key,view=region['view'],a=a,c=c,xy_a=xy_a,xy_c=xy_c,train=train,validation=validation))
    result['region_checks']=checks;result['calibration_assumption']=asdict(calibration)
    if len({r['view'] for r in eligible})<GROUND_POLICY['minimum_views']:return result
    source=[];target=[];xy_a=[];xy_c=[];views=[];ids=[];train=[];validation=[]
    for r in eligible:
        offset=len(source);source.extend(r['a']);target.extend(r['c']);xy_a.extend(r['xy_a']);xy_c.extend(r['xy_c'])
        views.extend([r['view']]*len(r['a']));ids.extend(f'{r["key"]}:{i}' for i in range(len(r['a'])))
        train.extend(offset+i for i in r['train']);validation.extend(offset+i for i in r['validation'])
    source,target,xy_a,xy_c=map(np.array,(source,target,xy_a,xy_c))
    views=np.array(views);train=np.array(train);validation=np.array(validation)
    def predict(pose, indices):
        xy=(xy_a[indices]-pose[:2])@_rotation(pose[2]);out=np.empty((len(indices),2))
        vv=views[indices]
        for view in sorted(set(vv)):
            mask=vv==view;out[mask]=ground_pixels(xy[mask],view,calibration)
        return out
    def errors(pose, indices):
        p=predict(pose,indices);bad=~np.isfinite(p).all(axis=1)
        e=np.linalg.norm(np.nan_to_num(p,nan=1e6)-target[indices],axis=1);e[bad]=1e6
        return e
    pairs=list(itertools.combinations(range(len(train)),2))
    rng=np.random.default_rng(GROUND_POLICY['ransac_seed'])
    if len(pairs)>GROUND_POLICY['ransac_trials']:
        pairs=[pairs[i] for i in rng.choice(len(pairs),GROUND_POLICY['ransac_trials'],replace=False)]
    candidates=[fit_se2(xy_a[train],xy_c[train])]
    candidates.extend(fit_se2(xy_a[train[list(pair)]],xy_c[train[list(pair)]]) for pair in pairs)
    best=None;best_score=(-1,-math.inf)
    for pose in candidates:
        if pose is None:continue
        e=errors(pose,train);inliers=e<=GROUND_POLICY['ransac_px']
        score=(int(inliers.sum()),-float(np.median(e)))
        if score>best_score:best,best_score=(pose,inliers),score
    if best is None:return {**result,'reason':'no_finite_ground_model'}
    pose,mask=best
    result['training_inlier_fraction']=float(mask.mean())
    if mask.mean()<GROUND_POLICY['minimum_train_fraction']:
        return {**result,'reason':'training_model_unsupported'}
    fit=train[mask]
    if len({v for v in views[fit]})<2:return {**result,'reason':'training_cross_view_support_lost'}
    per_view=[]
    for view in sorted(set(views[train])):
        population=train[views[train]==view];kept=fit[views[fit]==view]
        per_view.append(dict(view=view,training_count=len(population),inlier_count=len(kept),
                             inlier_fraction=len(kept)/len(population),
                             accepted=len(kept)>=4 and len(kept)/len(population)>=.7))
    result['per_view_training_support']=per_view
    if not all(v['accepted'] for v in per_view):return {**result,'reason':'per_view_training_support_lost'}
    def residual(p):
        prediction=predict(p,fit)
        return np.nan_to_num(prediction-target[fit],nan=1000.,posinf=1000.,neginf=-1000.).ravel()
    optimized=least_squares(residual,pose,loss='soft_l1',f_scale=.5,max_nfev=GROUND_POLICY['max_evaluations'])
    result.update(optimizer_success=bool(optimized.success),fit_track_ids=[ids[i] for i in fit],
                  validation_track_ids=[ids[i] for i in validation])
    if not optimized.success:return {**result,'reason':'optimizer_not_converged'}
    information=optimized.jac.T@optimized.jac
    if np.linalg.matrix_rank(information)<3:return {**result,'reason':'ground_motion_rank_deficient'}
    # This is local linearized noise conditioning, not an empirical/certified
    # bound on height/pitch, incorrect surface identity or dynamic-scene bias.
    covariance=np.linalg.inv(information)*max(.25**2,float(np.mean(optimized.fun**2)))
    translation_std=float(np.sqrt(np.linalg.eigvalsh(covariance[:2,:2]).max()))
    yaw_std=float(np.sqrt(covariance[2,2]))
    result['conditional_uncertainty']=dict(pixel_noise_floor_px=.25,
        covariance_se2=covariance.tolist(),translation_std_m=translation_std,yaw_std_rad=yaw_std,
        certified_error_bound=False,scope='linearized pixel noise only; excludes prior and model errors')
    if translation_std>.015 or yaw_std>math.radians(1):
        return {**result,'reason':'ground_motion_ill_conditioned'}
    metrics=[]
    for region in eligible:
        checked=validation[views[validation]==region['view']]
        e=errors(optimized.x,checked)
        median,p90,fraction=float(np.median(e)),float(np.quantile(e,.9)),float(np.mean(e<=1.))
        metrics.append(dict(view=region['view'],count=len(checked),median_px=median,p90_px=p90,inlier_fraction=fraction,
                            accepted=median<=GROUND_POLICY['validation_median_px'] and p90<=GROUND_POLICY['validation_p90_px'] and fraction>=GROUND_POLICY['validation_inlier_fraction']))
    result['heldout_views']=metrics
    if not all(m['accepted'] for m in metrics):return {**result,'reason':'heldout_prediction_failed'}
    result.update(status='conditional_ground_motion',reason='shared_metric_ground_model_with_internal_heldout_support',
                  translation_m=optimized.x[:2].tolist(),yaw_delta_rad=float(optimized.x[2]),
                  assumed_domain='rigid static scene, horizontal ground and level calibrated camera',
                  supporting_views=sorted({r['view'] for r in eligible}))
    return result


def _camera_motion(view, motion):
    b=rig_basis(view);yaw=motion['yaw_delta_rad']
    r=np.eye(3);r[:2,:2]=_rotation(-yaw)
    t=-r@np.r_[motion['translation_m'],0.]
    return b.T@r@b,b.T@t


def estimate_vertical_plane(region, motion, calibration):
    """Conditioned plane q=n/d from measured ground-motion scale, not wall ID.

    Static-scene/ground-pose assumptions are inherited. A front surface never
    implies endpoint identity. Validation points are not plane-fit inliers.
    """
    result=dict(status='unknown',reason='ground_motion_or_parallax_unavailable',distance_m=None,
                normal_rig=None,motion_authorized=False,arrival_status='unverified')
    if motion['status']!='conditional_ground_motion' or np.linalg.norm(motion['translation_m'])<.015:return result
    if any(reason not in {'rotation_or_low_parallax','insufficient_image_displacement'} for reason in region.get('reasons',[])):
        return {**result,'reason':'source_spatial_or_temporal_model_rejected'}
    tracks=region['tracks'];train,validation=split_tracks(tracks)
    if len(train)<8 or len(validation)<4:return {**result,'reason':'insufficient_plane_tracks'}
    a,c=[np.array([t[k] for t in tracks],float) for k in ('first','last')]
    k=np.array([[calibration.focal_px,0,calibration.principal_px],[0,calibration.focal_px,calibration.principal_px],[0,0,1.]])
    ki=np.linalg.inv(k);r,t=_camera_motion(region['view'],motion)
    rays=np.column_stack((a,np.ones(len(a))))@ki.T
    normalized=np.column_stack((c,np.ones(len(c))))@ki.T
    rotated=rays@r.T;u,v=normalized[:,0],normalized[:,1]
    design=np.concatenate(((t[0]-u[train]*t[2])[:,None]*rays[train],
                           (t[1]-v[train]*t[2])[:,None]*rays[train]))
    rhs=np.r_[u[train]*rotated[train,2]-rotated[train,0],v[train]*rotated[train,2]-rotated[train,1]]
    if np.linalg.matrix_rank(design)<3:return {**result,'reason':'rank_deficient_plane'}
    q=np.linalg.lstsq(design,rhs,rcond=None)[0]
    def predict(q,indices):return _project(k@(r+np.outer(t,q))@ki,a[indices])
    def residual(q):
        p=predict(q,train)
        return (np.full((len(train),2),1000.) if p is None else p-c[train]).ravel()
    optimized=least_squares(residual,q,loss='soft_l1',f_scale=.5,max_nfev=80)
    result.update(fit_track_indices=train,validation_track_indices=validation,optimizer_success=bool(optimized.success))
    if not optimized.success:return {**result,'reason':'plane_optimizer_not_converged'}
    q=optimized.x
    if not np.isfinite(q).all() or np.linalg.norm(q)<1e-6:return {**result,'reason':'invalid_plane_vector'}
    predicted=predict(q,validation)
    if predicted is None:return {**result,'reason':'invalid_plane_prediction'}
    error=np.linalg.norm(predicted-c[validation],axis=1)
    metrics=dict(median_px=float(np.median(error)),p90_px=float(np.quantile(error,.9)),inlier_fraction=float(np.mean(error<=1.)))
    result['heldout_metrics']=metrics
    if metrics['median_px']>.75 or metrics['p90_px']>1.5 or metrics['inlier_fraction']<.8:
        return {**result,'reason':'heldout_plane_failed'}
    # q^T ray is inverse positive depth. Do not keep a sign-flipped solution.
    if np.any(rays@q<=1e-4):return {**result,'reason':'plane_cheirality_failed'}
    normal=rig_basis(region['view'])@q/np.linalg.norm(q);distance=1./np.linalg.norm(q)
    if abs(normal[2])>math.sin(math.radians(15)):
        return {**result,'reason':'not_vertical_plane'}
    if not .25<=distance<=8.:return {**result,'reason':'plane_distance_out_of_domain'}
    # Small plane reprojection error alone does not constrain metric depth.
    # Fit a competing rotation on training points, then compare all check
    # points without RANSAC selection. This is the existing .20px support
    # gain, not a claim that inherited motion/plane uncertainty is solved.
    _,rotation_only=_rotation_residual(a[train],c[train])
    rotation_prediction=_project(k@rotation_only@ki,a[validation])
    if rotation_prediction is None:return {**result,'reason':'invalid_rotation_competitor'}
    rotation_error=np.linalg.norm(rotation_prediction-c[validation],axis=1)
    gain=float(np.median(rotation_error))-metrics['median_px']
    result['rotation_competition']=dict(rotation_median_px=float(np.median(rotation_error)),gain_px=gain)
    if gain<.20:return {**result,'reason':'plane_rotation_competition_unresolved'}
    # Two side walls can approximate a fictitious near frontal plane to much
    # better than .75px when motion is tiny. Require a strong *relative* fit,
    # not just a small absolute residual in a subpixel-signal regime.
    relative=metrics['median_px']/max(float(np.median(rotation_error)),1e-9)
    result['rotation_competition']['relative_median_residual']=relative
    if relative>.10:return {**result,'reason':'plane_signal_to_residual_unresolved'}
    result.update(status='conditional_vertical_plane',reason='metric_ground_motion_conditioned_plane',
                  normal_rig=normal.tolist(),distance_m=float(distance),
                  static_scene_verified=False,calibration_verified=False)
    return result


def corridor_from_planes(planes):
    result=dict(status='unknown',reason='two_sided_planes_unavailable',offset_m=None,heading_error_rad=None,width_m=None,
                motion_authorized=False,arrival_status='unverified')
    if any(planes.get(v,{}).get('status')!='conditional_vertical_plane' for v in ('left','right')):return result
    l,r=planes['left'],planes['right'];normals=[]
    for plane in (l,r):
        normal=np.array(plane['normal_rig'][:2]);normal/=np.linalg.norm(normal)
        if normal[1]<0:normal=-normal
        normals.append(normal)
    separation=math.acos(float(np.clip(normals[0]@normals[1],-1,1)))
    if separation>math.radians(5):return {**result,'reason':'side_planes_not_parallel'}
    axis=(normals[0]+normals[1])/2;heading=math.atan2(axis[0],axis[1])
    width=l['distance_m']+r['distance_m']
    if abs(heading)>.5 or not 1.2<=width<=4.:return {**result,'reason':'corridor_out_of_declared_domain'}
    return {**result,'status':'conditional_corridor_geometry','reason':'two_parallel_vertical_plane_candidates',
            'offset_m':float((r['distance_m']-l['distance_m'])/2),'heading_error_rad':float(heading),
            'width_m':float(width),'parallel_gap_deg':math.degrees(separation)}


def probe_calibrated_ground(frames, *, now_s, calibration_id, calibration=None):
    baseline=probe_ground_rig(frames,now_s=now_s,calibration_id=calibration_id)
    calibration=calibration or GroundCalibration();regions={};selection={};channels={}
    for key,source in baseline['regions'].items():
        if not key.endswith('_ground'):continue
        y,x={'front':(0,0),'left':(0,128),'right':(128,0)}[source['view']]
        images=[f['rgb'][y:y+128,x:x+128] for f in frames]
        options=dict(H=filter_h(images,source),I=filter_i(images,source))
        chosen=select_descriptor(source['tracks'],options,np.linalg.inv(_rectification()))
        regions[key]=options[chosen['channel']];selection[key]=chosen;channels[key]=options
    ground=estimate_ground_motion(regions,calibration);planes={};plane_diagnostics={}
    for key,source in baseline['regions'].items():
        if not key.endswith('_raw'):continue
        # Fixed image hypothesis; it is neither a wall label nor a true mask.
        upper={**source,'tracks':[t for t in source['tracks'] if all(8<=t[k][1]<72 for k in ('first','middle','last'))],
               'coverage_map':[[1.,0.,0.],[0.,2.,-16.],[0.,0.,1.]]}
        y,x={'front':(0,0),'left':(0,128),'right':(128,0)}[source['view']]
        images=[f['rgb'][y:y+128,x:x+128] for f in frames]
        options=dict(H=filter_h(images,upper),I=filter_i(images,upper))
        choice=select_descriptor(upper['tracks'],options,np.array(upper['coverage_map']))
        selected=options[choice['channel']]
        plane_diagnostics[source['view']]=dict(selection=choice,region=selected,channels=options)
        planes[source['view']]=estimate_vertical_plane(selected,ground,calibration)
    corridor=corridor_from_planes(planes)
    front=dict(status='unknown',range_m=None,arrival_status='unverified',motion_authorized=False)
    if planes.get('front',{}).get('status')=='conditional_vertical_plane':
        normal=np.array(planes['front']['normal_rig']);d=planes['front']['distance_m']
        if normal[0]>math.cos(math.radians(15)):
            front.update(status='conditional_front_surface',range_m=float(d/normal[0]),
                         reason='surface_is_not_endpoint_identity')
    return dict(version='calibrated_ground_motion_v1_development',baseline=baseline,
                descriptor_selection=selection,descriptor_channels=channels,regions=regions,
                ground_motion=ground,vertical_planes=planes,plane_diagnostics=plane_diagnostics,
                corridor_geometry=corridor,front_surface=front,
                evidence_packet=baseline['evidence_packet'],review=baseline['review'],
                motion_authorized=False,arrival_status='unverified',atomic_edge_compatible=False,
                limitations=['stronger_declared_ground_and_height_prior_than_G_H_I',
                             'calibration_and_static_scene_not_certified','internal_holdout_not_independent_dataset',
                             'single_moving_plane_not_ego_motion_proof','no_control_integration_yet'])
