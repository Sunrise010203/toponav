"""Opt-in task3 ablations, NOT promoted S2 acceptance or a new safety gate.

Only RGB, real capture times and declared calibration enter the selector.
Training tracks choose a window BEFORE exactly one unchanged final fit. Its
held-out/geometry/free-space failures cannot trigger a search for acceptance.
Startup, stationary proof, permission and recovery budgets are inherited.
"""
from collections import deque
import math
import numpy as np
from roadvision.continuous_straight import StraightController
from roadvision.ground_band import track_ground_band
from roadvision.rgb_motion_checks import split_tracks
from roadvision.corridor_rgb_probe import _grid_count, _project
from roadvision.calibrated_ground_motion import ground_points, fit_se2, pixel_predictions

HISTORY_SELECTION_POLICY=dict(sizes=[5,9,13,17],horizon_s=1.61,max_capture_gap_s=.250001,
    max_keyframe_gap_s=.81,min_training_tracks=6,min_training_grid_cells=3,min_views=2,
    min_translation_m=.025,max_translation_m=.30,preferred_translation_m=.08,
    max_abs_yaw_rad=.12,max_training_median_px=.75)
CONTROL_SCHEDULE_POLICY=dict(center_band_m=.04,max_center_band_m=.08,lookahead_m=1.2,
    heading_priority_rad=.10,heading_gain=.8,omega_cap_rad_s=.12,
    omega_slew_rad_s2=.15,filter_tau_s=.30)


def training_view_score(region,view,calibration):
    """Never read validation coordinates, final status, models or residuals."""
    p=HISTORY_SELECTION_POLICY;tracks=region['tracks'];train,_=split_tracks(tracks)
    out=dict(eligible=False,scope='training_only',training_tracks=len(train),translation_m=0.)
    if len(train)<p['min_training_tracks']:return out
    first,last=[np.array([tracks[i][key] for i in train],float) for key in ('first','last')]
    coverage=np.array(region.get('coverage_map',np.eye(3)),float)
    cells=min(_grid_count(_project(coverage,x)) for x in (first,last))
    out['training_grid_cells']=cells
    try:pose=fit_se2(ground_points(first,view,calibration),ground_points(last,view,calibration))
    except ValueError:return out
    if pose is None:return out
    translation=float(np.linalg.norm(pose[:2]));yaw=float(abs(pose[2]))
    errors=np.linalg.norm(pixel_predictions(first,view,pose,calibration)-last,axis=1)
    residual=float(np.median(errors))
    # Nonfinite residuals must not leak NaNs into a public receipt.
    out.update(translation_m=translation,abs_yaw_rad=yaw,training_median_px=residual if math.isfinite(residual) else None)
    eligible=(cells>=p['min_training_grid_cells'] and p['min_translation_m']<=translation<=p['max_translation_m'] and
              yaw<=p['max_abs_yaw_rad'] and math.isfinite(residual) and residual<=p['max_training_median_px'])
    out['eligible']=bool(eligible)
    if eligible:out['score']=abs(math.log(translation/p['preferred_translation_m']))+yaw/.06+residual/.75
    return out


def history_training_score(frames,calibration):
    keys=[frames[0],frames[len(frames)//2],frames[-1]];views={}
    for view,y,x in [('front',0,0),('left',0,128),('right',128,0)]:
        region=track_ground_band([f['rgb'][y:y+128,x:x+128] for f in keys],view=view)
        views[view]=training_view_score(region,view,calibration)
    good=[v for v in views.values() if v['eligible']]
    return dict(eligible=len(good)>=HISTORY_SELECTION_POLICY['min_views'],eligible_views=len(good),
                score=float(np.mean([v['score'] for v in good])) if good else None,scope='training_only',views=views)


def select_navigation_history(buffer,*,now_s,calibration):
    p=HISTORY_SELECTION_POLICY;frames=list(buffer)
    if not frames or type(now_s) not in (int,float) or not math.isfinite(now_s) or now_s<0:raise ValueError('Invalid history clock/buffer')
    ids=set();times=[]
    for f in frames:
        if not isinstance(f,dict) or set(f)!={'frame_id','captured_sim_s','rgb'}:raise ValueError('Only public RGB envelope allowed')
        identity=f['frame_id'];t=f['captured_sim_s'];rgb=f['rgb']
        if not isinstance(identity,str) or not identity or len(identity)>256 or identity in ids:raise ValueError('Invalid/duplicate frame ID')
        if type(t) not in (int,float) or not math.isfinite(t) or t<0:raise ValueError('Invalid capture time')
        if not isinstance(rgb,np.ndarray) or rgb.dtype!=np.uint8 or rgb.shape!=(256,256,3):raise ValueError('Invalid RGB')
        ids.add(identity);times.append(t)
    if any(not 0<b-a<=p['max_capture_gap_s'] for a,b in zip(times,times[1:])):raise ValueError('Unordered/gapped capture buffer')
    if not 0<=now_s-times[-1]<=.25:raise ValueError('Stale/future live frame')
    candidates=[];eligible=[]
    for n in p['sizes']:
        if n>len(frames):continue
        f=frames[-n:];t=[x['captured_sim_s'] for x in (f[0],f[n//2],f[-1])]
        if t[-1]-t[0]>p['horizon_s'] or max(np.diff(t))>p['max_keyframe_gap_s']:continue
        score=history_training_score(f,calibration)
        candidates.append(dict(size=n,span_s=t[-1]-t[0],**score))
        if score['eligible']:eligible.append((score['score'],-score['eligible_views'],abs(n-9),n))
    selected=frames[-min(eligible)[-1]:] if eligible else None
    return selected,dict(scope='training_only_before_one_final_fit',candidates=candidates,
        selected_frame_ids=[f['frame_id'] for f in selected] if selected else [],
        selected_size=len(selected) if selected else None,
        final_geometry_attempts=1 if selected else 0,selection_is_motion_permission=False)


class AblationStraightController(StraightController):
    def __init__(self,*,variant,**kwargs):
        if variant not in ('history','control','combined'):raise ValueError('Unknown experimental straight variant')
        super().__init__(**kwargs)
        self.variant=variant;self.selection_receipt=None;self.control_receipt=None;self.filtered_omega=0.
        if variant in ('history','combined'):self.frames=deque(maxlen=17)

    def _navigation_history(self,now_s):
        if self.variant=='control':return super()._navigation_history(now_s)
        selected,self.selection_receipt=select_navigation_history(self.frames,now_s=now_s,calibration=self.calibration)
        return selected

    def _following_command(self,c,front):
        if self.variant=='history':return super()._following_command(c,front)
        # Preserve the baseline linear-speed/front-approach calculation exactly.
        v,_=super()._following_command(c,front)
        p=CONTROL_SCHEDULE_POLICY;heading=c['heading_error_rad'];offset=c['offset_m']
        uncertainty=c.get('conditional_interval',{}).get('offset_half_width_m',0.)
        if type(uncertainty) not in (int,float) or not math.isfinite(uncertainty) or uncertainty<0:uncertainty=0.
        band=min(p['max_center_band_m'],max(p['center_band_m'],uncertainty))
        effective=math.copysign(max(0.,abs(offset)-band),offset)
        weight=float(np.clip(1-abs(heading)/p['heading_priority_rad'],0.,1.))
        target=float(np.clip(-p['heading_gain']*(heading+weight*math.atan(effective/p['lookahead_m'])),
                             -min(p['omega_cap_rad_s'],self.policy.max_omega_rad_s),min(p['omega_cap_rad_s'],self.policy.max_omega_rad_s)))
        dt=max(0.,min(self.capture_dt,.1));alpha=dt/(p['filter_tau_s']+dt)
        filtered=self.filtered_omega+alpha*(target-self.filtered_omega)
        slew=p['omega_slew_rad_s2']*dt
        omega=float(np.clip(filtered,self.last_command_omega-slew,self.last_command_omega+slew))
        self.filtered_omega=omega
        self.control_receipt=dict(center_band_m=band,effective_offset_m=effective,lateral_weight=weight,
            target_omega_rad_s=target,dt_s=dt,scope='experimental_control_only_not_motion_permission')
        return v,omega

    def _output(self,v=0.,omega=0.):
        if not v and not omega:self.filtered_omega=0.;self.control_receipt=None
        out=super()._output(v,omega)
        out['experiment']=dict(variant=self.variant,promoted=False,history_selection=self.selection_receipt,control=self.control_receipt)
        return out

    def result(self):
        return dict(super().result(),experiment=dict(variant=self.variant,promoted=False,
            history_policy=HISTORY_SELECTION_POLICY if self.variant!='control' else None,
            control_policy=CONTROL_SCHEDULE_POLICY if self.variant!='history' else None))
