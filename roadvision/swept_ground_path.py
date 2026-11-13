"""Task4 conditional full-footprint envelope and RGB patch coverage.

A sparse far-lookahead acceptance is NOT an area certificate. Blind cells,
flat/ambiguous patches and unsupported planes remain UNKNOWN. No simulator,
truth, semantic colour rule or active recovery enters here. Rear patches are
opt-in for task5; the default three-view contract is unchanged.
"""
from dataclasses import dataclass,asdict
import math
import cv2
import numpy as np
from roadvision.continuous_straight import StraightPolicy
from roadvision.calibrated_ground_motion import GroundCalibration,ground_points,ground_pixels,_camera_motion
from roadvision.ground_history import validate_history
from roadvision.rgb_ground_rig import rig_basis
from roadvision.rgb_resampling_checks import appearance_evidence
from roadvision.corridor_rgb_probe import _lk


@dataclass(frozen=True)
class SweepInstallation:
    half_length_m: float=.16
    half_width_m: float=.17  # wheels, not just the .11m chassis half-width
    camera_forward_m: float=.20
    margin_m: float=.05
    cell_m: float=.02
    yaw_speed_bound_rad_s: float=.35
    minimum_yaw_deceleration_rad_s2: float=.5

    def __post_init__(self):
        for value in asdict(self).values():
            if type(value) not in (int,float) or not math.isfinite(value) or value<=0:raise ValueError('Invalid installation')
        if not (.16<=self.half_length_m<=.5 and .17<=self.half_width_m<=.5 and self.camera_forward_m==.2 and
                .05<=self.margin_m<=.2 and .01<=self.cell_m<=.02 and .35<=self.yaw_speed_bound_rad_s<=.5 and
                .1<=self.minimum_yaw_deceleration_rad_s2<=.5):raise ValueError('Outside declared conservative installation')


def task_contract(installation=None):
    installation=installation or SweepInstallation()
    if not isinstance(installation,SweepInstallation):raise ValueError('Typed installation required')
    return dict(version='swept_ground_guard_v1',installation=asdict(installation),
        scope='following_only; bounded startup probe retains existing explicit initial swept-path prior',
        assumptions=['static level calibrated ground','rigid declared camera/body/wheels installation',
            'forward actuator speed <= declared policy bound, yaw speed <= installation bound',
            'zero requested after bounded command horizon; minimum linear/angular braking are ASSUMPTIONS, not measured guarantees',
            'accepted RGB patches conditionally follow one ground plane; prior/model errors are not certified'],
        unknown_is_free=False,sparse_lookahead_is_area_proof=False,certified_clearance=False)


def _finite(value):return type(value) in (int,float) and math.isfinite(value)


def _rotated_bounds(points,angle):
    # Analytic extrema of x*cos(t)-y*sin(t), x*sin(t)+y*cos(t)
    # over ALL angles, not only sampled poses or endpoint rectangles.
    xs=[];ys=[]
    for x,y in points:
        angles=[-angle,angle,0.]
        for root in (math.atan2(-y,x),math.atan2(x,y)):
            angles.extend(root+k*math.pi for k in range(-2,3) if -angle<=root+k*math.pi<=angle)
        for a in angles:
            xs.append(x*math.cos(a)-y*math.sin(a));ys.append(x*math.sin(a)+y*math.cos(a))
    return min(xs),min(ys),max(xs),max(ys)


def predict_sweep(v,omega,*,age_s,policy,installation=None,translation_uncertainty_m=0.,yaw_uncertainty_rad=0.):
    p=installation or SweepInstallation()
    if not isinstance(p,SweepInstallation) or not isinstance(policy,StraightPolicy):raise ValueError('Typed installation/policy required')
    if not all(_finite(x) for x in (v,omega,age_s,translation_uncertainty_m,yaw_uncertainty_rad)):raise ValueError('Nonfinite command/time/uncertainty')
    if not (0<=v<=policy.max_speed_m_s and abs(omega)<=policy.max_omega_rad_s and 0<=age_s<=policy.max_frame_age_s and
            0<=translation_uncertainty_m<=.25 and 0<=yaw_uncertainty_rad<=.25):raise ValueError('Outside bounded command/time/uncertainty domain')
    reaction=policy.command_horizon_s+age_s
    speed=policy.actuator_speed_bound_m_s;yaw_speed=p.yaw_speed_bound_rad_s
    distance=speed*reaction+speed**2/(2*policy.minimum_brake_deceleration_m_s2)
    angle=yaw_speed*reaction+yaw_speed**2/(2*p.minimum_yaw_deceleration_rad_s2)+yaw_uncertainty_rad
    if angle>=math.pi/2:raise ValueError('Reachable heading exceeds forward-motion envelope domain')
    corners=[(x,y) for x in (-p.half_length_m,p.half_length_m) for y in (-p.half_width_m,p.half_width_m)]
    x0,y0,x1,y1=_rotated_bounds(corners,angle)
    pad=p.margin_m+translation_uncertainty_m+1e-9
    lateral=distance*math.sin(angle)
    bounds=[x0-p.camera_forward_m-pad,y0-lateral-pad,x1-p.camera_forward_m+distance+pad,y1+lateral+pad]
    polygon=[[bounds[0],bounds[1]],[bounds[2],bounds[1]],[bounds[2],bounds[3]],[bounds[0],bounds[3]]]
    # Nominal requested arc is diagnostic ONLY. The gate uses the enclosing
    # tube based on actuator bounds, BOTH yaw signs, and independent braking.
    t=0.;xy=np.zeros(2);yaw=0.;nominal=[];dt=.025
    horizon=reaction+max(v/policy.minimum_brake_deceleration_m_s2,abs(omega)/p.minimum_yaw_deceleration_rad_s2)
    while t<horizon+1e-9:
        nominal.append(dict(t_s=t,body_center_latest_camera_xy_m=(xy-np.array([p.camera_forward_m,0])).tolist(),yaw_rad=yaw))
        h=min(dt,max(0.,horizon-t))
        if h<1e-10:break
        elapsed=max(0.,t+h/2-reaction)
        vv=max(0.,v-policy.minimum_brake_deceleration_m_s2*elapsed)
        ww=math.copysign(max(0.,abs(omega)-p.minimum_yaw_deceleration_rad_s2*elapsed),omega)
        xy+=h*vv*np.array([math.cos(yaw+h*ww/2),math.sin(yaw+h*ww/2)]);yaw+=h*ww;t+=h
    return dict(status='prediction_only',bounds_m=bounds,polygon=polygon,nominal=nominal,
        requested_v_m_s=v,requested_omega_rad_s=omega,reaction_s=reaction,maximum_translation_m=distance,
        maximum_abs_yaw_rad=angle,translation_uncertainty_m=translation_uncertainty_m,yaw_uncertainty_rad=yaw_uncertainty_rad,
        installation=asdict(p),policy=asdict(policy),certified_clearance=False,motion_authorized=False,
        tube_scope='full wheels/body; all intermediate orientations; conservative independent actuator-bound translation and yaw braking')


def _polygon(value):
    a=np.asarray(value,float)
    if a.ndim!=2 or a.shape[1]!=2 or not 3<=len(a)<=32 or not np.isfinite(a).all() or np.abs(a).max()>20:raise ValueError('Invalid bounded polygon')
    if not cv2.isContourConvex(a.astype(np.float32)) or abs(cv2.contourArea(a.astype(np.float32)))<1e-10:raise ValueError('Nonconvex/degenerate polygon')
    return a


def required_cells(polygon,cell_m):
    poly=_polygon(polygon)
    if not _finite(cell_m) or not .01<=cell_m<=.02:raise ValueError('Unsupported metric coverage resolution')
    lo=np.floor(poly.min(axis=0)/cell_m).astype(int);hi=np.ceil(poly.max(axis=0)/cell_m).astype(int)
    if np.prod(hi-lo)>4096:raise ValueError('Coverage domain exceeds bounded working area')
    cells=[]
    for x in range(lo[0],hi[0]):
        for y in range(lo[1],hi[1]):
            x0,y0=x*cell_m,y*cell_m
            q=np.array([[x0,y0],[x0+cell_m,y0],[x0+cell_m,y0+cell_m],[x0,y0+cell_m]])
            area,_=cv2.intersectConvexConvex(poly.astype(np.float32),q.astype(np.float32))
            if area>1e-10:cells.append(q)
    return np.asarray(cells)


def coverage_audit(polygon,patches,*,cell_m):
    cells=required_cells(polygon,cell_m);covered=np.zeros(len(cells),bool)
    for patch in patches:
        poly=_polygon(patch['polygon'])
        if patch.get('accepted') is not True:continue
        edges=np.roll(poly,-1,axis=0)-poly
        q=cells[:,:,None,:]-poly[None,None,:,:]
        cross=edges[None,None,:,0]*q[:,:,:,1]-edges[None,None,:,1]*q[:,:,:,0]
        signed=np.sum(poly[:,0]*np.roll(poly[:,1],-1)-poly[:,1]*np.roll(poly[:,0],-1))
        inside=np.all(cross>=-1e-9,axis=2) if signed>0 else np.all(cross<=1e-9,axis=2)
        covered|=inside.all(axis=1)
    return dict(covered=bool(covered.all()),required_cells=len(cells),supported_cells=int(covered.sum()),unknown_cells=int((~covered).sum()),
        unknown_examples=[dict(center_xy_m=c.mean(axis=0).tolist(),polygon=c.tolist()) for c in cells[~covered][:8]],
        cell_m=cell_m,rule='entire conservative cell must lie inside ONE accepted RGB patch footprint; no dilation, gap fill, point-to-area or prior extrapolation')


def assess_plane_clearance(polygon,planes):
    points=_polygon(polygon);checks={};radius=float(np.linalg.norm(points,axis=1).max())
    for name,p in planes.items():
        item=dict(status='unknown');checks[name]=item
        if p.get('status')!='conditional_vertical_plane':continue
        n=np.asarray(p.get('normal_rig'),float);interval=p.get('conditional_interval',{})
        d=p.get('distance_m');dd=interval.get('distance_half_width_m');da=interval.get('normal_angle_half_width_rad')
        if n.shape!=(3,) or not np.isfinite(n).all() or abs(np.linalg.norm(n)-1)>.001 or abs(n[2])>.001 or not all(_finite(v) for v in (d,dd,da)) or d<=0 or dd<0 or not 0<=da<math.pi/2:continue
        clearance=d-dd-float((points@n[:2]).max())-radius*2*math.sin(da/2)
        item.update(status='supported' if clearance>0 else 'intersection_or_uncertain',lower_clearance_m=clearance)
    return dict(supported=bool(checks) and all(c['status']=='supported' for c in checks.values()),checks=checks,
                certified_clearance=False,scope='conditional latest-camera plane intervals over entire conservative footprint tube')


def ground_patch_support(frames,motion,points,*,calibration,include_back=False):
    """Fixed physical queries, one geometry-selected view each, no target OR.

    An accepted native 11px patch has its OWN projected footprint only. It
    cannot fill an unseen near-field region just because far samples pass.
    """
    if type(include_back) is not bool:raise ValueError('Explicit include_back bool required')
    panels={};out=[];unseen=0
    views=[('front',0,0),('left',0,128),('right',128,0)]
    if include_back:views.append(('back',128,128))
    for view,y,x in views:
        rot,t=_camera_motion(view,motion)
        k=np.array([[calibration.focal_px,0,calibration.principal_px],[0,calibration.focal_px,calibration.principal_px],[0,0,1.]])
        q=rig_basis(view).T@np.array([0.,0.,-1.])/calibration.height_m
        h=k@(rot+np.outer(t,q))@np.linalg.inv(k)
        first,last=[cv2.cvtColor(np.ascontiguousarray(f['rgb'][y:y+128,x:x+128]),cv2.COLOR_RGB2GRAY) for f in (frames[0],frames[-1])]
        aligned=cv2.warpPerspective(first,h,(128,128),flags=cv2.INTER_LINEAR)
        mask=(cv2.warpPerspective(np.ones((128,128),np.float32),h,(128,128),flags=cv2.INTER_LINEAR)>=1-1e-6).astype(np.uint8)
        panels[view]=(aligned,last,mask,np.ones((128,128),np.uint8))
    for point in points:
        candidates=[]
        for view in panels:
            pixel=ground_pixels(np.array([point]),view,calibration)[0]
            if np.isfinite(pixel).all() and (pixel>=6).all() and (pixel<=121).all() and pixel[1]>=calibration.principal_px+6:
                candidates.append((float(np.minimum(pixel-6,121-pixel).min()),view,pixel))
        if not candidates:unseen+=1;continue
        _,view,pixel=max(candidates,key=lambda item:(item[0],item[1]))
        a,b,mask,ones=panels[view]
        e=appearance_evidence(a,b,pixel,pixel,mask,ones)
        observed,valid=_lk(a,b,np.array([pixel],np.float32));flow=None;fb=None
        if valid[0]:
            flow=float(np.linalg.norm(observed[0]-pixel));back,bv=_lk(b,a,observed.astype(np.float32))
            if bv[0]:fb=float(np.linalg.norm(back[0]-pixel))
        flow_ok=flow is not None and fb is not None and flow<=.5 and fb<=.25
        if not flow_ok:e.update(accepted=False,reason='ground_transfer_flow_not_supported')
        corners=np.array([pixel+[-5,-5],pixel+[5,-5],pixel+[5,5],pixel+[-5,5]])
        polygon=ground_points(corners,view,calibration)
        out.append(dict(point_xy_m=list(map(float,point)),view=view,pixel=pixel.tolist(),polygon=polygon.tolist(),
                        ground_flow_residual_px=flow,flow_fb_px=fb,**e))
    return dict(patches=out,queries=len(points),outside_all_supported_views=unseen,
                scope=('fixed sweep-cell centers; front/left/right/back; ground-plane conditional native-patch footprints' if include_back else
                       'fixed sweep-cell centers; front/left/right only; ground-plane conditional native-patch footprints'))


def assess_swept_ground(frames,packet,v,omega,*,now_s,calibration_id,policy=None,installation=None,calibration=None,include_back=False):
    if type(include_back) is not bool:raise ValueError('Explicit include_back bool required')
    refs=validate_history(frames,now_s=now_s,calibration_id=calibration_id)
    p=installation or SweepInstallation();calibration=calibration or GroundCalibration();policy=policy or StraightPolicy()
    out=dict(status='unknown',reason='geometry_or_uncertainty_unavailable',geometry_reference_frame_id=refs[-1]['frame_id'],
        geometry_time_s=refs[-1]['captured_sim_s'],requested_v_m_s=v,requested_omega_rad_s=omega,
        certified_clearance=False,motion_authorized=False,arrival_status='unverified',contract=task_contract(p))
    motion=packet.get('ground_motion',{})
    if (packet.get('geometry_reference_frame_id')!=refs[-1]['frame_id'] or packet.get('geometry_time_s')!=refs[-1]['captured_sim_s'] or
        motion.get('status')!='conditional_ground_motion' or motion.get('translation_reference_frame_id')!=refs[0]['frame_id'] or
        motion.get('from_time_s')!=refs[0]['captured_sim_s'] or motion.get('to_time_s')!=refs[-1]['captured_sim_s']):return out
    cov=np.asarray(motion.get('conditional_uncertainty',{}).get('covariance_se2'),float)
    if cov.shape!=(3,3) or not np.isfinite(cov).all() or not np.allclose(cov,cov.T,atol=1e-10) or np.linalg.eigvalsh(cov).min()<-1e-12:return out
    baseline=float(np.linalg.norm(motion['translation_m']))
    if not .025<=baseline<=.4:return {**out,'reason':'motion_baseline_outside_domain'}
    sweep=predict_sweep(v,omega,age_s=now_s-refs[-1]['captured_sim_s'],policy=policy,installation=p,
        translation_uncertainty_m=2*math.sqrt(max(0.,float(np.linalg.eigvalsh(cov[:2,:2]).max()))),
        yaw_uncertainty_rad=2*math.sqrt(max(0.,float(cov[2,2]))))
    planes=packet.get('vertical_planes',{})
    relevant={name:planes.get(name,{}) for name in ('left','right')}
    if planes.get('front',{}).get('status')=='conditional_vertical_plane':relevant['front']=planes['front']
    walls=assess_plane_clearance(sweep['polygon'],relevant)
    cells=required_cells(sweep['polygon'],p.cell_m)
    options={'include_back':True} if include_back else {}
    rgb=ground_patch_support(frames,motion,[c.mean(axis=0) for c in cells],calibration=calibration,**options)
    coverage=coverage_audit(sweep['polygon'],rgb['patches'],cell_m=p.cell_m)
    out.update(sweep=sweep,planes=walls,coverage=coverage,rgb_support=rgb)
    if not walls['supported']:out['reason']='swept_plane_intersection_or_unknown'
    elif not coverage['covered']:out['reason']='swept_area_contains_unobserved_or_unverified_ground'
    else:out.update(status='conditional_patch_supported_sweep',reason='whole_conservative_tube_conditionally_supported_by_planes_and_rgb_patches')
    return out
