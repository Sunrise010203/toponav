"""Separate calibrated RGB straight primitive; no simulator, truth or node IDs.

Startup requires EXPLICIT bounded_probe authorization and a short swept-path clearance prior,
not a fabricated map distance. Never adapts partial motion into atomic failed.
S1 outputs remain read-only; this module owns restricted actuator decisions.
"""
from collections import deque
from dataclasses import dataclass,asdict
import hashlib
import math
import numpy as np
from roadvision.corridor_rgb_probe import CALIBRATION_ID
from roadvision.calibrated_ground_motion import GroundCalibration
from roadvision.ground_history import validate_history,_ground_evidence
from roadvision.straight_evidence import probe_straight_history
from roadvision.free_ground_path import assess_free_ground
from roadvision.straight_permission import assess_startup_texture


@dataclass(frozen=True)
class InitialClearancePrior:
    confirmed: bool=False
    swept_forward_clearance_m: float=.30
    source: str='task_operator_initial_swept_path_confirmation'

    def __post_init__(self):
        if type(self.confirmed) is not bool or self.source!='task_operator_initial_swept_path_confirmation':raise ValueError('Invalid initial operational prior')
        if type(self.swept_forward_clearance_m) not in (int,float) or not math.isfinite(self.swept_forward_clearance_m) or self.swept_forward_clearance_m<.30:raise ValueError('Startup prior must cover at least .30m swept forward path')


@dataclass(frozen=True)
class StraightPolicy:
    max_speed_m_s: float=.15
    max_omega_rad_s: float=.20
    max_duration_s: float=60.
    max_frame_age_s: float=.25
    bootstrap_duration_s: float=1.2
    unverified_timeout_s: float=1.
    brake_timeout_s: float=2.
    front_stop_lower_range_m: float=1.30
    front_slowdown_band_m: float=.60
    front_approach_minimum_fraction: float=.50
    frozen_rgb_motion_timeout_s: float=.30
    stationary_confirmations: int=3
    rearm_confirmations: int=3
    max_recoveries: int=3
    probe_distance_limit_m: float=.14
    actuator_speed_bound_m_s: float=.18
    command_horizon_s: float=.25
    minimum_brake_deceleration_m_s2: float=.25
    stopping_margin_m: float=.05

    def __post_init__(self):
        for name,value in vars(self).items():
            if name=='max_recoveries':
                if type(value) is not int or not 0<=value<=3:raise ValueError('Recovery count must be an integer in 0..3')
            elif name in ('stationary_confirmations','rearm_confirmations'):
                if type(value) is not int or value<3:raise ValueError('At least3 fresh stationary confirmations required')
            elif type(value) not in (int,float) or not math.isfinite(value) or value<=0:raise ValueError('Invalid positive finite policy field')
        if self.max_speed_m_s>.15 or self.max_omega_rad_s>.2 or self.max_frame_age_s>.25 or self.bootstrap_duration_s>1.2:raise ValueError('Outside validated command/freshness/bootstrap limits')
        if self.probe_distance_limit_m>.14 or self.actuator_speed_bound_m_s<.18 or self.command_horizon_s<.25 or self.minimum_brake_deceleration_m_s2>.25 or self.stopping_margin_m<.05:raise ValueError('Outside conservative probe/braking envelope')
        if self.front_approach_minimum_fraction>1 or self.frozen_rgb_motion_timeout_s>.30:raise ValueError('Outside approach fraction/frozen RGB response limits')


def _finite_vector(value,size):
    """Reject coercible strings/bools and malformed supported evidence."""
    if not isinstance(value,(list,tuple,np.ndarray)):return False
    try:
        a=np.asarray(value,dtype=object)
        return a.shape==(size,) and all(type(v) in (int,float) and math.isfinite(v) for v in a)
    except (TypeError,ValueError,OverflowError):return False


def _valid_ground_motion(motion):
    if not isinstance(motion,dict) or not _finite_vector(motion.get('translation_m'),2):return False
    yaw=motion.get('yaw_delta_rad')
    try:return type(yaw) in (int,float) and math.isfinite(yaw)
    except OverflowError:return False


class StraightController:
    def __init__(self,*,initial_clearance=None,calibration=None,policy=None,motion_permission='no_motion'):
        self.initial_clearance=initial_clearance or InitialClearancePrior();self.calibration=calibration or GroundCalibration();self.policy=policy or StraightPolicy()
        if not isinstance(self.initial_clearance,InitialClearancePrior) or not isinstance(self.calibration,GroundCalibration) or not isinstance(self.policy,StraightPolicy):raise ValueError('Expected explicit typed task/calibration/policy')
        if self.calibration!=GroundCalibration():raise ValueError('Controller scope requires the declared S1 .45m/f64/c64 installation')
        if motion_permission not in ('no_motion','bounded_probe'):raise ValueError('Explicit no_motion or bounded_probe permission required')
        self.motion_permission=motion_permission;self.startup_texture=None;self.rearm_frames=0;self.recovery_count=0;self.probe_distance_bound=0.
        self.stationary_samples=deque(maxlen=3)
        self.frames=deque(maxlen=9);self.seen=set();self.last_capture=-1.;self.last_now=-1.;self.started=None;self.last_frame_id=None
        self.state='observing';self.reason='awaiting_rgb';self.probe_started=None;self.unverified_started=None;self.brake_started=None
        self.motion_commands=0;self.stationary_frames=0;self.last_perception=None;self.last_free=None;self.last_rgb=None;self.last_image_change=None
        self.last_command_omega=0.;self.capture_dt=0.
        self.last_command_v=0.;self.frozen_rgb_motion_s=0.

    def _output(self,v=0.,omega=0.):
        self.last_command_omega=float(omega);self.last_command_v=float(v)
        return dict(state=self.state,reason=self.reason,v_m_s=float(v),omega_rad_s=float(omega),
                    arrival_status='unverified',atomic_edge_compatible=False,stationary_frames=self.stationary_frames,
                    motion_commands=self.motion_commands,image_change=self.last_image_change,
                    control_frame_id=self.last_frame_id,captured_sim_s=self.last_capture,decision_sim_s=self.last_now,
                    perception=self.last_perception,free_path=self.last_free,
                    permission=dict(mode=self.motion_permission,authorization=('bounded_probe' if self.state=='bootstrapping' and v else 'supported_following' if v else 'zero'),
                        probe_distance_bound_m=self.probe_distance_bound,stopping_reserve_m=self._stopping_reserve(),
                        rearm_confirmations=self.rearm_frames,recovery_count=self.recovery_count,startup_texture=self.startup_texture))

    def _abort(self,reason):self.state='aborted';self.reason=reason;return self._output()

    def _clock(self,now_s):
        if type(now_s) not in (int,float) or not math.isfinite(now_s) or now_s<0:return 'invalid_sim_time'
        if now_s<self.last_now:return 'sim_time_regression'
        self.last_now=now_s
        if self.started is None:self.started=now_s
        if now_s-self.started>=self.policy.max_duration_s:return 'motion_timeout'
        return None

    def missing(self,now_s):
        if self.state in ('visual_stop','aborted'):return self._output()
        return self._abort(self._clock(now_s) or 'missing_frame')

    def cancel(self,now_s):
        """External finite execution budget/cancellation, never node failure."""
        if self.state in ('visual_stop','aborted'):return self._output()
        return self._abort(self._clock(now_s) or 'execution_cancelled')

    def _stopping_reserve(self):
        # Conditional actuator assumption, NOT measured vehicle deceleration.
        p=self.policy;v=p.actuator_speed_bound_m_s
        return v*p.command_horizon_s+v*v/(2*p.minimum_brake_deceleration_m_s2)+p.stopping_margin_m

    def _wait(self,reason,now_s):
        if self.state!='waiting_evidence':
            self.brake_started=now_s;self.stationary_frames=0;self.rearm_frames=0;self.stationary_samples.clear()
        self.reason=reason;self.state='waiting_evidence'
        if self.unverified_started is None:self.unverified_started=now_s
        if now_s-self.unverified_started>=self.policy.unverified_timeout_s-1e-8:return self._abort('evidence_recovery_timeout_'+reason)
        return self._output()

    def _confirm_stationary(self,now_s):
        # All three capture endpoints must FOLLOW the zero request. The
        # capture which triggered braking was taken before zero was applied.
        samples=list(self.stationary_samples)
        # Fixed >=.1s sampling is part of the stop contract, not adaptive
        # navigation history. Fast packet delivery must not compress proof.
        if not samples or samples[-1]['captured_sim_s']!=self.last_capture:return
        if len(samples)<3 or samples[0]['captured_sim_s']<=self.brake_started:
            self.stationary_frames=0;return
        m=self._local_stationary_motion(now_s,samples)
        stable=(m is not None and m['status']=='conditional_ground_motion' and _valid_ground_motion(m) and
                np.linalg.norm(m['translation_m'])<.003 and abs(m['yaw_delta_rad'])<.005 and
                self.last_image_change is not None and self.last_image_change<.8)
        self.stationary_frames=self.stationary_frames+1 if stable else 0

    def _current_evidence(self,packet):
        return (packet.get('geometry_reference_frame_id')==self.last_frame_id and
                packet.get('geometry_time_s')==self.last_capture)

    def _local_stationary_motion(self,now_s,frames=None):
        frames=list(self.frames)[-3:] if frames is None else frames
        if len(frames)<3:return None
        validate_history(frames,now_s=now_s,calibration_id=CALIBRATION_ID)
        return _ground_evidence(frames,self.calibration)[0]

    def update(self,frame,now_s):
        try:return self._update(frame,now_s)
        except Exception:
            self._abort('input_or_perception_exception')
            raise

    def _update(self,frame,now_s):
        # Terminal state/result is latched even if a later transport packet
        # is malformed. No validation error may rewrite a completed stop.
        if self.state in ('visual_stop','aborted'):return self._output()
        if not isinstance(frame,dict) or set(frame)!={'frame_id','captured_sim_s','rgb'}:
            self._abort('invalid_frame_envelope');raise ValueError('Only public RGB envelope is allowed')
        error=self._clock(now_s)
        if error:return self._abort(error)
        identity=frame['frame_id'];captured=frame['captured_sim_s'];rgb=frame['rgb']
        if not isinstance(identity,str) or not identity or len(identity)>256:return self._abort('invalid_frame_id')
        if identity in self.seen:return self._abort('replayed_frame')
        if type(captured) not in (int,float) or not math.isfinite(captured) or captured<0:return self._abort('invalid_capture_time')
        if not 0<=now_s-captured<=self.policy.max_frame_age_s:return self._abort('stale_or_future_frame')
        if captured<=self.last_capture:return self._abort('out_of_order_frame')
        if self.last_capture>=0 and captured-self.last_capture>.250001:return self._abort('capture_stream_gap')
        if not isinstance(rgb,np.ndarray) or rgb.shape!=(256,256,3) or rgb.dtype!=np.uint8:return self._abort('invalid_rgb')
        if any(rgb[y:y+128,x:x+128].std()<2 for y,x in ((0,0),(0,128),(128,0))):return self._abort('missing_or_flat_view')
        self.capture_dt=captured-self.last_capture if self.last_capture>=0 else 0.
        self.seen.add(identity);self.last_capture=captured;self.last_frame_id=identity
        self.last_image_change=float(np.abs(rgb.astype(np.float32)-self.last_rgb).mean()) if self.last_rgb is not None else None
        self.last_rgb=rgb.astype(np.float32).copy();self.frames.append({**frame,'rgb':rgb.copy()})
        if self.state in ('observing','waiting_evidence','braking') and (not self.stationary_samples or captured-self.stationary_samples[-1]['captured_sim_s']>=.1-1e-8):
            self.stationary_samples.append(self.frames[-1])
        if not self.initial_clearance.confirmed:return self._abort('initial_clearance_prior_not_confirmed')
        if self.motion_permission=='no_motion':return self._abort('motion_not_authorized')
        # A camera can repeat pixels while supplying NEW IDs and timestamps.
        # Detect that contradiction only AFTER commanding appreciable motion;
        # independent of the reference-texture startup veto. At rest, exact
        # equal pixels remain valid stationary evidence. No pose/speed truth.
        appreciable_command=self.last_command_v>=.05 or abs(self.last_command_omega)>=.05
        self.frozen_rgb_motion_s=(self.frozen_rgb_motion_s+self.capture_dt
                                  if appreciable_command and self.last_image_change==0. else 0.)
        if self.frozen_rgb_motion_s>=self.policy.frozen_rgb_motion_timeout_s-1e-8:
            return self._abort('commanded_motion_without_rgb_response')
        if self.state=='braking':
            if now_s-self.brake_started>=self.policy.brake_timeout_s:return self._abort('brake_confirmation_timeout')
            self._confirm_stationary(now_s)
            if self.stationary_frames>=self.policy.stationary_confirmations:self.state='visual_stop';self.reason='fresh_rgb_supports_stationary_surface_stop'
            return self._output()
        if self.state=='waiting_evidence':
            self._confirm_stationary(now_s)
            if now_s-self.brake_started>=self.policy.brake_timeout_s:return self._abort('brake_confirmation_timeout')
            if now_s-self.unverified_started>=self.policy.unverified_timeout_s-1e-8:return self._abort('evidence_recovery_timeout_'+self.reason)
        if self.probe_started is None:
            if len(self.stationary_samples)<3:return self._output()
            m=self._local_stationary_motion(now_s,list(self.stationary_samples))
            if m is None or m['status']!='conditional_ground_motion':return self._abort('initial_rgb_ground_unsupported')
            # NaN comparisons are false, so threshold checks alone could let
            # an invalid "supported" estimate authorize the startup probe.
            if not _valid_ground_motion(m):return self._abort('invalid_initial_ground_motion')
            if np.linalg.norm(m['translation_m'])>.01 or abs(m['yaw_delta_rad'])>.02:return self._abort('initial_vehicle_not_visually_stationary')
            # Require persistent reference-only observability across all three
            # fresh stationary captures, not just one convenient patch/frame.
            startup_frames=list(self.stationary_samples)
            checks=[self._assess_startup_texture(f['rgb']) for f in startup_frames]
            self.startup_texture=dict(accepted=all(c['accepted'] for c in checks),evaluated_sim_s=now_s,frames=[dict(
                frame_id=f['frame_id'],captured_sim_s=f['captured_sim_s'],
                rgb_sha256=hashlib.sha256(f['rgb'].tobytes()).hexdigest(),
                accepted=c['accepted'],reason=c['reason'],checks=c.get('checks',{}),cells=c.get('cells',[]),
                failed_checks=c.get('failed_checks',[]),reference_supported_cells=c.get('reference_supported_cells'),
                rectified_supported_cells=c.get('rectified_supported_cells')) for f,c in zip(startup_frames,checks)],latest=checks[-1])
            if not self.startup_texture['accepted']:return self._abort('startup_texture_unverified')
            if self.policy.probe_distance_limit_m+self._stopping_reserve()>self.initial_clearance.swept_forward_clearance_m:return self._abort('initial_clearance_insufficient_for_probe_and_braking')
            self.probe_started=now_s;self.state='bootstrapping';self.reason='bounded_probe_inside_explicit_task_clearance'
        if len(self.frames)>=9:
            # The hook chooses BEFORE the one final geometry attempt. Unknown
            # selection is a stop, never permission to retry another fit.
            frames=self._navigation_history(now_s)
            if frames is None:
                self.rearm_frames=0;return self._wait('history_selection_unverified',now_s)
            r=self._probe_navigation_history(frames,now_s)
            self.last_perception=r
            if not self._current_evidence(r):
                self.rearm_frames=0;return self._wait('stale_geometry_binding',now_s)
            c=r['corridor_geometry'];front=r['front_surface']
            if c['status']=='conditional_corridor_geometry' and not all(type(c.get(k)) in (int,float) and math.isfinite(c[k]) for k in ('heading_error_rad','offset_m')):return self._abort('invalid_geometry_values')
            if front['status']=='conditional_front_surface':
                interval=front.get('range_interval_m')
                if not _finite_vector(interval,2) or not 0<=interval[0]<=interval[1]:
                    return self._abort('invalid_front_surface_interval')
            if front['status']=='conditional_front_surface' and front['range_interval_m'][0]<=self.policy.front_stop_lower_range_m:
                if c['status']!='conditional_corridor_geometry' or abs(c['heading_error_rad'])>.15 or abs(c['offset_m'])>.2:return self._abort('front_stop_alignment_uncertain')
                self.state='braking';self.reason='conditional_front_surface_near_not_node_arrival';self.brake_started=now_s;self.stationary_frames=0;self.stationary_samples.clear();return self._output()
            keyframes=[frames[0],frames[len(frames)//2],frames[-1]]
            self.last_free=self._assess_navigation_free_ground(keyframes,r['ground_motion'],now_s)
            clearance=self.last_free.get('conditional_stopping_clearance_m')
            stop_supported=(type(clearance) in (int,float) and math.isfinite(clearance) and clearance>=self._stopping_reserve())
            if c['status']=='conditional_corridor_geometry' and self.last_free['status']=='conditional_free_ground_path' and self._current_evidence(self.last_free) and stop_supported:
                if not self._following_guard(c,front,keyframes,r,now_s):
                    self.rearm_frames=0;return self._wait('swept_path_unverified',now_s)
                if self.state=='waiting_evidence':
                    if self.recovery_count>=self.policy.max_recoveries:return self._abort('recovery_count_exhausted')
                    self.rearm_frames+=1
                    if self.stationary_frames<self.policy.stationary_confirmations or self.rearm_frames<self.policy.rearm_confirmations:
                        return self._wait('awaiting_stationary_and_fresh_rearm',now_s)
                    self.recovery_count+=1
                self.state='following';self.reason='fresh_two_side_geometry_and_ground_transfer';self.unverified_started=None
                v,omega=self._following_command(c,front)
                self.motion_commands+=1;return self._output(v,omega)
            self.rearm_frames=0
            return self._wait('geometry_or_free_path_unverified',now_s)
        if self.state=='bootstrapping' and now_s-self.probe_started<self.policy.bootstrap_duration_s:
            # Command-speed envelope reserves the NEXT maximum capture gap.
            # Not odometry: valid only under the declared actuator speed bound.
            bound=self.policy.actuator_speed_bound_m_s*(now_s-self.probe_started+self.policy.command_horizon_s)
            if bound>self.policy.probe_distance_limit_m+1e-8:return self._wait('probe_distance_budget_exhausted',now_s)
            self.probe_distance_bound=bound
            self.motion_commands+=1;return self._output(self.policy.max_speed_m_s,0.)
        return self._wait('bootstrap_ended_without_usable_geometry',now_s)

    def _assess_startup_texture(self,rgb):
        """Baseline policy; startup alternatives must explicitly opt in."""
        return assess_startup_texture(rgb,self.calibration)

    def _probe_navigation_history(self,frames,now_s):
        """One baseline fit. Experimental samplers must explicitly opt in."""
        return probe_straight_history(frames,now_s=now_s,calibration_id=CALIBRATION_ID,calibration=self.calibration)

    def _assess_navigation_free_ground(self,frames,motion,now_s):
        """Baseline transfer checks; experimental descriptors must opt in."""
        return assess_free_ground(frames,motion,now_s=now_s,calibration_id=CALIBRATION_ID,calibration=self.calibration)

    def _following_guard(self,c,front,frames,packet,now_s):
        """Baseline/earlier ablations retain their original acceptance chain."""
        return True

    def _navigation_history(self,now_s):
        """Baseline: exactly nine consecutive captures, no alternative fit."""
        return list(self.frames)

    def _following_command(self,c,front):
        heading=c['heading_error_rad'];offset=c['offset_m']
        target=float(np.clip(-heading-.4*offset,-self.policy.max_omega_rad_s,self.policy.max_omega_rad_s))
        # Steering onset must not consume the low-resolution tracker
        # baseline in a single correction. Stopping bypasses slew.
        slew=.3*min(self.capture_dt,.1)
        omega=float(np.clip(target,self.last_command_omega-slew,self.last_command_omega+slew))
        v=self.policy.max_speed_m_s*max(.5,1-abs(heading)/.4)
        # Correct alignment before sacrificing parallax: slowing the
        # offset/yaw recovery phase made a real offset run lose its
        # usable nine-frame baseline. These are live RGB estimates,
        # not a pose oracle or a retry of rejected perception.
        if (front['status']=='conditional_front_surface' and
            abs(offset)<=.05 and abs(heading)<=.05):
            remaining=front['range_interval_m'][0]-self.policy.front_stop_lower_range_m
            fraction=float(np.clip(remaining/self.policy.front_slowdown_band_m,0.,1.))
            minimum=self.policy.front_approach_minimum_fraction
            cap=self.policy.max_speed_m_s*(minimum+(1-minimum)*fraction)
            if cap<v:self.reason='fresh_geometry_with_front_approach_slowdown'
            v=min(v,cap)
        return v,omega

    def result(self):
        kind='completed_visual_stop' if self.state=='visual_stop' else ('aborted_motion_uncertain' if self.motion_commands else 'rejected_no_motion') if self.state=='aborted' else 'in_progress'
        return dict(result_type=kind,state=self.state,reason=self.reason,arrival_status='unverified',atomic_edge_compatible=False,
                    motion_commands=self.motion_commands,initial_clearance_assumption=asdict(self.initial_clearance),
                    motion_permission=self.motion_permission,permission_contract='straight_motion_permission_v1',stopping_assumptions=dict(actuator_speed_bound_m_s=self.policy.actuator_speed_bound_m_s,command_horizon_s=self.policy.command_horizon_s,minimum_brake_deceleration_m_s2=self.policy.minimum_brake_deceleration_m_s2,margin_m=self.policy.stopping_margin_m,measured_feedback=False),
                    declared_domain='installed calibrated level camera, rigid static scene, textured ground, perpendicular corridor fronts')


# S3 uses the EXISTING restricted blue-grey-floor/neutral-wall installation,
# not a retrained classifier or an assertion of S2 material generalisation.
# A new task implementation in source, not a runtime patch of StraightController.
def _corner_floor(rgb):
    pixels=rgb.astype(np.int16)
    return (pixels[...,2]-pixels[...,0]>8)&(pixels[...,2]-pixels[...,1]>3)


def _corner_footprint_line(pixels,view):
    """Fit the ground footprint in IMAGE pixels, before metric inversion.

    Far-boundary row quantisation is large in metres. Fitting stair-stepped
    ground points as a Euclidean line would falsely reject a genuine wall.
    This single model instead checks the actual pixel fit, then inverts the
    declared level-ground projection. It does not retry a rejected model.
    """
    pixels=np.asarray(pixels,float)
    if len(pixels)<8 or np.ptp(pixels[:,0])<24:return None
    u=pixels[:,0]-63.5;v=pixels[:,1]-63.5
    slope,intercept=np.linalg.lstsq(np.column_stack((u,np.ones(len(u)))),v,rcond=None)[0]
    errors=np.abs(v-(slope*u+intercept))
    if np.quantile(errors,.9)>.65 or np.median(errors)>.4:return None
    q=np.array([intercept/(.45*64),-slope/.45]);norm=np.linalg.norm(q)
    if norm<1e-9:return None
    angle={'front':0.,'left':math.pi/2,'right':-math.pi/2,'back':math.pi}[view]
    rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    normal=rotation@q/norm;distance=1/norm
    if not .25<distance<6.:return None
    return dict(normal=normal.tolist(),distance_m=float(distance),count=len(pixels),
                pixel_median=float(np.median(errors)),pixel_p90=float(np.quantile(errors,.9)))


def analyze_corner_rgb(rgb,*,junction=False):
    """RGB-only footprint geometry in a declared flat, unpainted colour domain.

    f64 and raster principal63.5, level rig at .45m. The floor rule can be wrong
    outside its explicit appearance domain; it is NOT simulator segmentation.
    Four actual views track the same outer wall across camera seams.
    """
    from roadvision.calibrated_ground_motion import ground_points,ground_pixels
    if not isinstance(rgb,np.ndarray) or rgb.shape!=(256,256,3) or rgb.dtype!=np.uint8:
        return dict(status='unknown',reason='invalid_rgb')
    calibration=GroundCalibration(principal_px=63.5)
    views={};lines=[]
    for view,(y,x) in {'front':(0,0),'left':(0,128),'right':(128,0),'back':(128,128)}.items():
        image=rgb[y:y+128,x:x+128]
        if image.std()<2:return dict(status='unknown',reason='missing_or_flat_view')
        mask=_corner_floor(image);pixels=[]
        for column in range(8,121,2):
            runs=np.flatnonzero(mask[65:125,column]&mask[66:126,column]&mask[67:127,column]&mask[68:128,column])
            if not len(runs):continue
            pixels.append((column,int(runs[0])+64.5))
        if len(pixels)<8:return dict(status='unknown',reason='ground_boundary_not_visible')
        pixels=np.array(pixels)
        points=ground_points(pixels,'front' if view=='back' else view,calibration)
        if view=='back':points=-points
        line=None if junction else _corner_footprint_line(pixels[(pixels[:,0]>=48)&(pixels[:,0]<=80)],view)
        views[view]=dict(line=line,points=points.tolist())
        # Canonical per-view pixel-line extraction, fixed seed and residuals.
        # Multiple lines represent different physical boundary segments, not
        # alternate models whose acceptance sets are unioned after rejection.
        remaining=pixels.copy();rng=np.random.default_rng(1887);view_lines=[]
        for _ in range(3):
            if len(remaining)<8:break
            best=None;count=0
            if junction:
                # Exhaustive deterministic pixel hypotheses. Random64 pairs
                # can miss the only >=24px incoming segment at a T seam.
                ia,ib=np.triu_indices(len(remaining),1)
                valid=np.abs(remaining[ib,0]-remaining[ia,0])>=24
                a,b=remaining[ia[valid]],remaining[ib[valid]]
                if len(a):
                    slopes=(b[:,1]-a[:,1])/(b[:,0]-a[:,0])
                    matches=np.abs(remaining[None,:,1]-(a[:,None,1]+slopes[:,None]*(remaining[None,:,0]-a[:,None,0])))<=.65
                    support=matches.sum(axis=1);index=int(np.argmax(support))
                    best=matches[index];count=int(support[index])
            else:
                for _ in range(64):
                    a,b=remaining[rng.choice(len(remaining),2,replace=False)]
                    if abs(b[0]-a[0])<24:continue
                    slope=(b[1]-a[1])/(b[0]-a[0])
                    inliers=np.abs(remaining[:,1]-(a[1]+slope*(remaining[:,0]-a[0])))<=.65
                    if inliers.sum()>count:best=inliers;count=int(inliers.sum())
            if best is None or count<8:break
            fitted=_corner_footprint_line(remaining[best],view)
            if fitted is not None:lines.append(fitted);view_lines.append(fitted)
            remaining=remaining[~best]
        if junction:
            # T uses the canonical whole-view segment producer, not a short
            # far-boundary central fit or a retry of a rejected central model.
            axis=np.array({'front':(1,0),'left':(0,1),'right':(0,-1),'back':(-1,0)}[view])
            aligned=[line for line in view_lines if np.dot(line['normal'],axis)>.98]
            views[view]['line']=max(aligned,key=lambda line:line['count']) if aligned else None
    floor=_corner_floor(rgb[:128,:128]);path_clear=True
    samples=np.array([(f,l) for f in (.50,.55,.60) for l in (-.22,0.,.22)])
    for u,v in ground_pixels(samples,'front',calibration):
        if not np.isfinite([u,v]).all() or not 1<=u<126 or not 1<=v<126:
            path_clear=False;break
        ix,iy=int(round(u)),int(round(v))
        if not floor[iy-1:iy+2,ix-1:ix+2].all():path_clear=False;break
    return dict(status='conditional_colour_footprint',views=views,lines=lines,
                front=views['front']['line'],path_clear=path_clear,calibration_verified=False,
                clearance_certified=False,appearance_domain='unpainted blue-grey floor, neutral walls; static level calibrated rig')


def _junction_branches(rgb,width_m):
    """Conditional branch reachability, in the SAME declared colour domain.

    Check every raster pixel inside the projected prospective body path.
    A quantised distant end-wall normal is NOT a branch-opening test.
    No scene dimensions or segmentation IDs.
    Closed/unknown is not an invitation to turn towards the other branch.
    """
    from roadvision.calibrated_ground_motion import ground_pixels
    import cv2
    calibration=GroundCalibration(principal_px=63.5)
    status={}
    for view,sign,y,x in [('left',1,0,128),('right',-1,128,0)]:
        samples=np.array([(sign*l,sign*d) for l,d in
                         [(-.24,.50),(.24,.50),(.24,width_m/2+.75),(-.24,width_m/2+.75)]])
        pixels=ground_pixels(samples,view,calibration)
        if not np.isfinite(pixels).all() or np.any(pixels<1) or np.any(pixels>=126):
            status[view]='unknown';continue
        region=np.zeros((128,128),np.uint8)
        cv2.fillConvexPoly(region,np.rint(pixels).astype(np.int32),1)
        mask=_corner_floor(rgb[y:y+128,x:x+128])
        status[view]='open' if mask[region.astype(bool)].all() else 'blocked'
    return status


class CornerController:
    """Enter, image-feedback quarter-turn, align and leave an L corner or T.

    Width comes from initial RGB boundaries, not topology/simulator coordinates.
    Orientation comes from the associated outer-wall normal, NOT motor duration.
    Legacy S2 remains unchanged; no topology arrival or atomic rollback claim.
    """
    def __init__(self,*,direction,initial_clearance=None,junction='l',pause_at_entry=False):
        if type(pause_at_entry) is not bool or (direction not in ('left','right') and not (direction is None and pause_at_entry and junction=='t')):raise ValueError('Expected a planned left/right action')
        if junction not in ('l','t'):raise ValueError('Expected L corner or stem-entry T junction')
        self.pause_at_entry=pause_at_entry;self.entry_ready=False
        self.junction=junction;self.branch_observations=0;self.last_branches=None
        self.direction=direction;self.sign=1 if direction=='left' else -1
        self.initial_clearance=InitialClearancePrior() if initial_clearance is None else initial_clearance
        if not isinstance(self.initial_clearance,InitialClearancePrior):raise ValueError('Expected explicit typed initial clearance')
        self.state='observing';self.phase='entry';self.reason='awaiting_three_consistent_rgb_frames'
        self.last_now=-1.;self.last_capture=-1.;self.last_frame_id=None;self.seen=set();self.started=None
        self.last_rgb=None;self.image_change=None;self.perception=None;self.width_samples=[];self.width_m=None
        self.wall_angle=None;self.wall_distance=None;self.entry_wall_angle=None;self.turn_angle_rad=0.
        self.stationary_frames=0;self.brake_started=None;self.exit_initial_range=None;self.exit_progress_m=0.
        self.motion_commands=0

    def resume_turn(self,direction):
        """Release a route hold; no motion until a NEW fresh RGB update."""
        if direction not in ('left','right') or not self.pause_at_entry or not self.entry_ready or self.state!='braking' or self.phase!='entry_stop':raise ValueError('No confirmed route-entry hold to release')
        self.direction=direction;self.sign=1 if direction=='left' else -1
        self.pause_at_entry=False;self.entry_ready=False;self.branch_observations=0;self.last_branches=None

    @staticmethod
    def _angle(a):return math.atan2(math.sin(a),math.cos(a))

    def _output(self,v=0.,omega=0.):
        if v or omega:self.motion_commands+=1
        return dict(state=self.state,phase=self.phase,reason=self.reason,v_m_s=float(v),omega_rad_s=float(omega),
                    control_frame_id=self.last_frame_id,captured_sim_s=self.last_capture,decision_sim_s=self.last_now,
                    measured_width_m=self.width_m,turn_angle_rad=self.turn_angle_rad,exit_progress_m=self.exit_progress_m,
                    motion_commands=self.motion_commands,stationary_frames=self.stationary_frames,perception=self.perception,
                    arrival_status='unverified',atomic_edge_compatible=False)

    def _abort(self,reason):self.state='aborted';self.reason=reason;return self._output()

    def _clock(self,now):
        if type(now) not in (int,float) or not math.isfinite(now) or now<0:return 'invalid_sim_time'
        if now<self.last_now:return 'sim_time_regression'
        self.last_now=now
        if self.started is None:self.started=now
        return 'motion_timeout' if now-self.started>=75. else None

    def missing(self,now_s):
        if self.state in ('visual_stop','aborted'):return self._output()
        return self._abort(self._clock(now_s) or 'missing_frame')

    def cancel(self,now_s):
        if self.state in ('visual_stop','aborted'):return self._output()
        return self._abort(self._clock(now_s) or 'execution_cancelled')

    def update(self,frame,now_s):
        try:return self._update(frame,now_s)
        except Exception:
            self._abort('input_or_perception_exception');raise

    def _brake(self,phase,now):
        self.state='braking';self.phase=phase;self.brake_started=now;self.stationary_frames=0
        self.reason='fresh_rgb_requests_'+phase;return self._output()

    def _update(self,frame,now_s):
        if self.state in ('visual_stop','aborted'):return self._output()
        if not isinstance(frame,dict) or set(frame)!={'frame_id','captured_sim_s','rgb'}:
            self._abort('invalid_frame_envelope');raise ValueError('Only public RGB envelope allowed')
        error=self._clock(now_s)
        if error:return self._abort(error)
        identity,t,rgb=frame['frame_id'],frame['captured_sim_s'],frame['rgb']
        if not isinstance(identity,str) or not identity or len(identity)>256:return self._abort('invalid_frame_id')
        if identity in self.seen:return self._abort('replayed_frame')
        if type(t) not in (int,float) or not math.isfinite(t) or t<0:return self._abort('invalid_capture_time')
        if not 0<=now_s-t<=.25:return self._abort('stale_or_future_frame')
        if t<=self.last_capture:return self._abort('out_of_order_frame')
        if self.last_capture>=0 and t-self.last_capture>.250001:return self._abort('capture_stream_gap')
        self.last_capture=t;self.last_frame_id=identity;self.seen.add(identity)
        r=analyze_corner_rgb(rgb,junction=True) if self.junction=='t' else analyze_corner_rgb(rgb);self.perception=r
        if r['status']!='conditional_colour_footprint':return self._abort(r['reason'])
        self.image_change=float(np.abs(rgb.astype(float)-self.last_rgb).mean()) if self.last_rgb is not None else None
        self.last_rgb=rgb.astype(float).copy()
        if not self.initial_clearance.confirmed:return self._abort('initial_clearance_prior_not_confirmed')
        if self.state=='braking':
            limit=5. if self.junction=='t' and self.phase=='entry_stop' else 2.
            if now_s-self.brake_started>=limit:return self._abort('branch_or_brake_confirmation_timeout') if self.junction=='t' else self._abort('brake_confirmation_timeout')
            self.stationary_frames=self.stationary_frames+1 if self.image_change is not None and self.image_change<.8 else 0
            if self.stationary_frames<3:
                self.entry_ready=False
                if self.junction=='t' and self.phase=='entry_stop':self.branch_observations=0;self.last_branches=None
                return self._output()
            if self.phase=='entry_stop':
                # A visible free disk around the optical centre bounds the
                # swept body only within the explicit colour/flat-floor domain.
                if any(not v['points'] or min(np.linalg.norm(v['points'],axis=1))<.45 for v in r['views'].values()):
                    return self._abort('pivot_swept_disk_not_visually_supported')
                if self.pause_at_entry:
                    self.entry_ready=True;self.reason='route_entry_ready_not_node_identity';return self._output()
                if self.junction=='t':
                    branches=_junction_branches(rgb,self.width_m);r['branches']=branches
                    if 'unknown' in branches.values():return self._abort('junction_branch_observation_unknown')
                    self.branch_observations=self.branch_observations+1 if branches==self.last_branches else 1
                    self.last_branches=branches.copy()
                    if branches[self.direction]!='open':
                        self.branch_observations=0;self.reason='waiting_for_planned_branch_clear';return self._output()
                    if self.branch_observations<3:
                        self.reason='confirming_both_junction_branches';return self._output()
                else:
                    branch=r['views'][self.direction]['line']
                    if branch is None or branch['distance_m']<.75*self.width_m or self.sign*branch['normal'][1]<.85:
                        return self._abort('planned_branch_not_visually_open')
                front=r['front']
                if front is None:return self._abort('entry_outer_wall_unknown')
                self.wall_angle=math.atan2(front['normal'][1],front['normal'][0]);self.entry_wall_angle=self.wall_angle
                self.wall_distance=front['distance_m'];self.phase='turn';self.state='moving'
            elif self.phase=='turn_stop':
                self.phase='exit';self.state='moving';self.exit_initial_range=None
            else:
                self.state='visual_stop';self.reason='fresh_rgb_supports_completed_corner_exit';return self._output()
        if self.phase=='turn':
            if self.junction=='t' and any(min(np.linalg.norm(v['points'],axis=1))<.45 for v in r['views'].values()):
                return self._abort('junction_turn_swept_disk_obstructed')
            candidates=[]
            for line in r['lines']:
                angle=math.atan2(line['normal'][1],line['normal'][0])
                delta=self._angle(angle-self.wall_angle)
                if abs(delta)<.22 and abs(line['distance_m']-self.wall_distance)<.18:
                    candidates.append((abs(delta)+abs(line['distance_m']-self.wall_distance),angle,line))
            if not candidates:return self._abort('turn_outer_wall_association_unknown')
            _,angle,line=min(candidates,key=lambda v:v[0]);step=-self.sign*self._angle(angle-self.wall_angle)
            if step<-.035:return self._abort('opposite_visual_rotation')
            self.turn_angle_rad+=step;self.wall_angle=angle;self.wall_distance=line['distance_m']
            if self.turn_angle_rad>math.pi/2+.10:return self._abort('visual_turn_overshoot')
            remaining=math.pi/2-self.turn_angle_rad
            if remaining<=.025:return self._brake('turn_stop',now_s)
            self.reason='image_outer_wall_normal_feedback'
            return self._output(0.,self.sign*min(.30,max(.06,1.5*remaining)))
        front=r['front']
        if self.junction=='t' and (self.phase=='exit' or (self.pause_at_entry and self.phase=='entry')):
            # Route approach and exit use actual central boundary rays.
            # Completion is near side-wall evidence. A far end-wall fit is
            # neither needed nor well-conditioned here. Actual central front
            # boundary rays supply only the obstruction/range diagnostic.
            central=[point[0] for point in r['views']['front']['points'] if point[0]>0 and abs(point[1])<.25]
            if not central:return self._abort('front_boundary_unknown')
            distance=min(central)
        else:
            if front is None or front['normal'][0]<.8:return self._abort('front_boundary_unknown')
            distance=front['distance_m']/front['normal'][0]
        left=r['views']['left']['line'];right=r['views']['right']['line']
        outer=right if self.sign==1 else left
        incoming_side=None
        if self.junction=='t' and self.phase=='entry' and self.width_m is not None:
            # BOTH sides open at a T. Track the observed incoming boundary
            # behind the camera, including actual back-view line segments.
            # Never extrapolate motion across a missing observation or use
            # an elapsed-wheel-time distance. Far exit end walls are excluded
            # by the previously measured incoming half-width.
            candidates=[line for line in r['lines'] if abs(line['normal'][1])>.98 and
                        abs(line['distance_m']-self.width_m/2)<.25]
            outer=max(candidates,key=lambda line:line['count']) if candidates else None
            if outer is not None:incoming_side=1 if outer['normal'][1]>0 else -1
        if outer is None:return self._abort(self.phase+'_outer_side_unknown')
        # A far front footprint may occupy only8 raster rows: its normal is
        # poorly conditioned even with subpixel residuals. Steering instead
        # uses the NEAR outer side, while the front supplies only axial range.
        heading=math.atan2(-self.sign*outer['normal'][0],-self.sign*outer['normal'][1])
        if incoming_side is not None:heading=math.atan2(incoming_side*outer['normal'][0],incoming_side*outer['normal'][1])
        if abs(heading)>.20:return self._abort('outer_boundary_alignment_unknown')
        if self.state=='observing':
            if left is None or right is None or left['normal'][1]<.95 or right['normal'][1]>-.95:
                return self._abort('initial_two_side_width_unknown')
            width=left['distance_m']+right['distance_m']
            if not 1.5<=width<=2.4:return self._abort('initial_width_outside_domain')
            self.width_samples.append(width)
            if len(self.width_samples)<3:return self._output()
            if np.ptp(self.width_samples)>.08:return self._abort('inconsistent_initial_width')
            self.width_m=float(np.median(self.width_samples));self.state='moving'
        if self.phase=='entry':
            remaining=distance-(self.width_m/2-.20)
            if remaining<=.025:return self._brake('entry_stop',now_s)
            # The side opposite the planned turn remains the outer boundary.
            offset=self.sign*(outer['distance_m']-self.width_m/2)
            if incoming_side is not None:offset=incoming_side*(self.width_m/2-outer['distance_m'])
        else:
            if self.exit_initial_range is None:self.exit_initial_range=distance
            self.exit_progress_m=self.exit_initial_range-distance
            # Direct OBSERVED footprint points, not fitted line extensions or
            # subtraction of two far quantised ranges. At least3 actual side
            # boundary samples must exist in BOTH behind/ahead bands, on each
            # side. Camera is .20m ahead: a true inner-wall footprint .50m
            # behind it puts the opening behind the body with margin.
            def supported(view,side,low,high):
                points=np.asarray(r['views'][view]['points'])
                return np.count_nonzero((points[:,0]>=low)&(points[:,0]<=high)&
                    (np.abs(points[:,1]-side*self.width_m/2)<.12))>=3
            exit_clear=all(supported(view,side,-.65,-.50) and supported(view,side,.40,.60)
                           for view,side in [('left',1),('right',-1)])
            if exit_clear:return self._brake('exit_stop',now_s)
            if distance<.65:return self._abort('exit_obstruction_or_insufficient_space')
            offset=self.sign*(outer['distance_m']-self.width_m/2)
            remaining=1.
        if not r['path_clear']:return self._abort('forward_colour_ground_path_unknown')
        if abs(offset)>.25:return self._abort('lateral_alignment_outside_domain')
        self.reason='rgb_'+self.phase+'_boundary_following'
        omega=float(np.clip(-1.5*heading-.6*offset,-.20,.20))
        return self._output(min(.15,max(.045,remaining*.8)),omega)

    def result(self):
        kind='completed_visual_corner' if self.state=='visual_stop' else ('aborted_motion_uncertain' if self.motion_commands else 'rejected_no_motion') if self.state=='aborted' else 'in_progress'
        if self.junction=='t' and kind=='completed_visual_corner':kind='completed_visual_junction'
        result=dict(result_type=kind,state=self.state,phase=self.phase,reason=self.reason,direction=self.direction,
                    turn_angle_rad=self.turn_angle_rad,exit_progress_m=self.exit_progress_m,motion_commands=self.motion_commands,
                    arrival_status='unverified',atomic_edge_compatible=False,
                    declared_domain='restricted unpainted blue-grey floor/neutral walls, static flat L corner, level .45m rig; not general material recognition')
        if self.junction=='t':result.update(junction='t',branch_observations=self.branch_observations,observed_branches=self.last_branches,
            declared_domain='restricted unpainted blue-grey floor/neutral walls, static flat stem-entry T junction, level .45m rig; no forward exit or node identity')
        return result


def _route_features(rgb):
    """Natural WALL appearance only; no encoded fiducials or network training."""
    import cv2
    if not isinstance(rgb,np.ndarray) or rgb.shape!=(256,256,3) or rgb.dtype!=np.uint8:raise ValueError('Expected public RGB mosaic')
    orb=cv2.ORB_create(nfeatures=450,edgeThreshold=8,patchSize=15,fastThreshold=7)
    result=[]
    for y,x in [(0,0),(0,128),(128,0),(128,128)]:
        view=rgb[y:y+128,x:x+128]
        mask=(~_corner_floor(view))&(view.min(axis=2)>20)&(view.max(axis=2)<225)
        mask[:12]=False;mask[105:]=False
        points,descriptors=orb.detectAndCompute(cv2.cvtColor(view,cv2.COLOR_RGB2GRAY),mask.astype(np.uint8)*255)
        result.append((np.array([p.pt for p in points],float).reshape(-1,2),descriptors))
    return result


def _route_place_matches(features,references):
    """Independently match ALL public node/heading references, not expected ID.

    Ratios/Hamming plus same-view, near-aligned pixel displacement and spatial
    support. Two camera directions must each support the unique place. A wall
    alone or a task label never authorises node identity. Images are required
    near the nominal station; arbitrary viewpoint relocalisation is NOT claimed.
    """
    import cv2
    matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
    accepted=[];scores=[]
    for reference in references:
        supports=[]
        for (points,desc),(ref_points,ref_desc) in zip(features,reference['features']):
            count=0
            if desc is not None and ref_desc is not None and len(ref_desc)>=2:
                pairs=matcher.knnMatch(desc,ref_desc,k=2)
                good=[pair[0] for pair in pairs if len(pair)==2 and pair[0].distance<=50 and pair[0].distance<.70*pair[1].distance]
                # A repeated reference descriptor may not cast many votes.
                unique={}
                for m in sorted(good,key=lambda m:m.distance):unique.setdefault(m.trainIdx,m)
                good=list(unique.values())
                if len(good)>=8:
                    a=points[[m.queryIdx for m in good]];b=ref_points[[m.trainIdx for m in good]]
                    displacement=np.linalg.norm(a-b,axis=1)
                    spatial=(np.ptp(a[:,0])>=20 and np.ptp(a[:,1])>=12)
                    if spatial and np.median(displacement)<=6 and np.quantile(displacement,.9)<=12:count=len(good)
            supports.append(count)
        candidate=dict(node_id=reference['node_id'],heading_deg=reference['heading_deg'],view_support=supports)
        scores.append(candidate)
        if sum(n>=8 for n in supports)>=2:accepted.append(candidate)
    return dict(matches=accepted,best=sorted(scores,key=lambda c:sum(c['view_support']),reverse=True)[:3])


def _route_openings(rgb,width_m):
    """Fresh local structure at a stopped station, not a simulator node sensor."""
    import cv2
    from roadvision.calibrated_ground_motion import ground_pixels
    result={};calibration=GroundCalibration(principal_px=63.5)
    for view,angle,y,x in [('front',0.,0,0),('left',math.pi/2,0,128),('right',-math.pi/2,128,0),('back',math.pi,128,128)]:
        rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        local=np.array([(.50,-.24),(.50,.24),(width_m/2+.75,.24),(width_m/2+.75,-.24)])
        points=local@rotation.T
        pixels=ground_pixels(-points if view=='back' else points,'front' if view=='back' else view,calibration)
        if not np.isfinite(pixels).all() or np.any(pixels<1) or np.any(pixels>=126):result[view]=None;continue
        region=np.zeros((128,128),np.uint8);cv2.fillConvexPoly(region,np.rint(pixels).astype(np.int32),1)
        result[view]=bool(_corner_floor(rgb[y:y+128,x:x+128])[region.astype(bool)].all())
    return result


class ContinuousRouteController:
    """Single-flight continuous executor using the PUBLIC v2 planner.

    Pure input boundary: topology/task/public reference RGB/current RGB/time.
    Planner callables are explicit dependencies, NOT simulator callbacks. No
    atomic-session failed/succeeded adaptation; a segment exit never advances
    a node. A NEW stopped RGB place+structure confirmation must do that.
    """
    def __init__(self,*,graph,references,plan_from_estimate,next_action,start=None,goal=None,heading=None,initial_clearance=None,session_id='continuous-route'):
        import copy
        self.graph=copy.deepcopy(graph)
        if self.graph.get('schema_version')!='2.0':raise ValueError('Continuous route requires v2 topology')
        if set(self.graph)-{'schema_version','scene_id','directed','exit_direction_frame','nodes','edges','task'}:raise ValueError('Only public topological graph fields allowed')
        for node in self.graph['nodes']:
            if set(node)-{'node_id','node_type','neighbors','exits','reference_images'}:raise ValueError('Metric/node truth fields forbidden')
            if any(set(e)!={'neighbor_id','channel_id','direction'} for e in node['exits']):raise ValueError('Only public cardinal exits allowed')
        if any(set(e)!={'channel_id','source','target','bidirectional','enabled'} for e in self.graph['edges']):raise ValueError('Metric/channel truth fields forbidden')
        if set(self.graph.get('task',{}))-{'start_node','goal_node','initial_heading_deg'}:raise ValueError('Only public task fields allowed')
        task=self.graph.get('task',{});self.start=task.get('start_node') if start is None else start
        self.goal=task.get('goal_node') if goal is None else goal
        self.heading=task.get('initial_heading_deg') if heading is None else heading
        if type(self.heading) not in (int,float) or self.heading not in (0,90,180,270):raise ValueError('Explicit cardinal heading prior required')
        ids={n['node_id'] for n in self.graph['nodes']}
        if self.start not in ids or self.goal not in ids or self.start==self.goal:raise ValueError('Invalid task nodes')
        if not isinstance(session_id,str) or not session_id or len(session_id)>120:raise ValueError('Invalid session ID')
        if not callable(plan_from_estimate) or not callable(next_action):raise ValueError('Explicit public planning dependencies required')
        self.plan=plan_from_estimate;self.next_action=next_action;self.session_id=session_id
        self.references=[];reference_keys=set()
        for ref in references:
            if not isinstance(ref,dict) or set(ref)!={'node_id','heading_deg','rgb'}:raise ValueError('Only public labelled reference RGB allowed')
            key=(ref['node_id'],ref['heading_deg'])
            if key[0] not in ids or type(key[1]) not in (int,float) or key[1] not in (0,90,180,270) or key in reference_keys:raise ValueError('Invalid/duplicate map reference')
            reference_keys.add(key);self.references.append(dict(node_id=key[0],heading_deg=key[1],features=_route_features(ref['rgb'])))
        if reference_keys!={(node,h) for node in ids for h in (0,90,180,270)}:raise ValueError('Complete node/heading public reference inventory required')
        self.initial_clearance=InitialClearancePrior() if initial_clearance is None else initial_clearance
        self.leaf=CornerController(direction=None,junction='t',pause_at_entry=True,initial_clearance=self.initial_clearance)
        self.state='observing';self.stage='seek_initial';self.reason='approach_then_independent_node_confirmation'
        self.last_now=-1.;self.last_capture=-1.;self.last_frame_id=None;self.seen=set();self.started=None
        self.last_leaf=None;self.place=None;self.structure=None;self.window=[]
        self.node_estimate=None;self.last_confirmed_node=None;self.pending=None;self.expected_event=None
        self.event_records={};self.exit_consumed=False;self.feedback_started=None
        self.command_count=0;self.motion_commands=0;self.transitions=0;self.duplicate_feedback=0
        self.node_confirmations=[];self.commands=[];self.route=None;self.last_exit_capture=-1.

    def _output(self,v=0.,omega=0.):
        import copy
        if v or omega:self.motion_commands+=1
        return copy.deepcopy(dict(state=self.state,phase=self.stage,reason=self.reason,v_m_s=float(v),omega_rad_s=float(omega),
            control_frame_id=self.last_frame_id,captured_sim_s=self.last_capture,decision_sim_s=self.last_now,
            arrival_status='visually_confirmed_map_node' if self.state=='visual_stop' else 'unverified',atomic_edge_compatible=False,
            node_estimate=self.node_estimate,last_confirmed_node=self.last_confirmed_node,
            pending_command=self.pending,pending_feedback=self.expected_event if not self.exit_consumed else None,
            transition_count=self.transitions,command_count=self.command_count,motion_commands=self.motion_commands,
            duplicate_feedback=self.duplicate_feedback,leaf=self.last_leaf,place=self.place,structure=self.structure,
            confirmation_window=len(self.window),route=self.route))

    def _abort(self,reason):
        self.state='aborted';self.reason=reason;self.node_estimate=None;self.window=[]
        return self._output()

    def _clock(self,now):
        if type(now) not in (int,float) or not math.isfinite(now) or now<0:return 'invalid_sim_time'
        if now<self.last_now:return 'sim_time_regression'
        self.last_now=now
        if self.started is None:self.started=now
        return 'route_timeout' if now-self.started>=240. else None

    def missing(self,now_s):
        if self.state in ('visual_stop','aborted'):return self._output()
        return self._abort(self._clock(now_s) or 'missing_frame')

    def cancel(self,now_s):
        if self.state in ('visual_stop','aborted'):return self._output()
        return self._abort(self._clock(now_s) or 'execution_cancelled')

    def _map_structure(self,node,heading):
        axes={'east':0,'north':90,'west':180,'south':270};relative={'front':0,'left':90,'right':-90,'back':180}
        exits=next(n['exits'] for n in self.graph['nodes'] if n['node_id']==node)
        absolute={axes[e['direction']] for e in exits}
        return {view:(heading+angle)%360 in absolute for view,angle in relative.items()}

    def _confirm_node(self,frame,now):
        self.place=_route_place_matches(_route_features(frame['rgb']),self.references)
        self.structure=_route_openings(frame['rgb'],self.leaf.width_m)
        matches=[m for m in self.place['matches'] if self._map_structure(m['node_id'],m['heading_deg'])==self.structure]
        expected=self.pending['target'] if self.pending else self.start
        if len(matches)!=1:return self._abort('node_reference_or_structure_not_unique')
        match=matches[0];key=(match['node_id'],match['heading_deg'])
        if key!=(expected,self.heading):return self._abort('node_or_heading_prior_conflict')
        if frame['captured_sim_s']<=self.last_exit_capture:return self._abort('pre_exit_capture_cannot_confirm_arrival')
        self.window.append(dict(frame_id=frame['frame_id'],captured_sim_s=frame['captured_sim_s'],node_id=key[0],heading_deg=key[1]))
        if len(self.window)<3:self.reason='confirming_independent_node_rgb';return self._output()
        proof=dict(node_id=key[0],heading_deg=key[1],captured_sim_s=frame['captured_sim_s'],decision_sim_s=now,
                   frame_ids=[r['frame_id'] for r in self.window],command_id=self.pending['command_id'] if self.pending else None)
        self.node_confirmations.append(proof);self.last_confirmed_node=key[0]
        self.node_estimate=dict(node_id=key[0],heading_deg=key[1],status='localized',heading_status='certain')
        if self.pending:
            if not self.exit_consumed:return self._abort('arrival_without_consumed_segment_feedback')
            self.transitions+=1
        self.pending=None;self.window=[];self.expected_event=None;self.exit_consumed=False
        self.route=self.plan(self.graph,self.node_estimate,self.goal)
        action=self.next_action(self.graph,self.node_estimate,self.route)
        if action['status']=='reached':
            self.state='visual_stop';self.stage='goal_stop';self.reason='goal_independently_confirmed_by_fresh_rgb';return self._output()
        if action['status']!='move' or action['action'] not in ('left','right'):return self._abort('route_action_outside_stem_entry_domain')
        if self.command_count>=12:return self._abort('route_command_limit')
        target=self.route['nodes'][1];channel=self.route['channels'][0]
        axes={'east':0,'north':90,'west':180,'south':270}
        source_exit=next(e for n in self.graph['nodes'] if n['node_id']==key[0] for e in n['exits'] if e['neighbor_id']==target)
        back_exit=next(e for n in self.graph['nodes'] if n['node_id']==target for e in n['exits'] if e['neighbor_id']==key[0])
        if (axes[source_exit['direction']]-axes[back_exit['direction']])%360!=180:return self._abort('curved_channel_not_supported')
        self.command_count+=1
        self.pending=dict(command_id=f'{self.session_id}:cmd-{self.command_count:04d}',source=key[0],target=target,channel_id=channel,
                          action=action['action'],issued_sim_s=now,source_frame_ids=proof['frame_ids'],source_heading_deg=key[1])
        self.commands.append(dict(self.pending));self.node_estimate=None
        self.leaf.resume_turn(action['action']);self.stage='turn_and_exit';self.state='moving';self.reason='public_planner_action_issued'
        return self._output()

    def update(self,frame,now_s):
        try:return self._update(frame,now_s)
        except Exception:
            self._abort('input_or_perception_exception');raise

    def _update(self,frame,now_s):
        if self.state in ('visual_stop','aborted'):return self._output()
        if not isinstance(frame,dict) or set(frame)!={'frame_id','captured_sim_s','rgb'}:
            self._abort('invalid_frame_envelope');raise ValueError('Only public RGB envelope allowed')
        error=self._clock(now_s)
        if error:return self._abort(error)
        identity,t,rgb=frame['frame_id'],frame['captured_sim_s'],frame['rgb']
        if not isinstance(identity,str) or not identity or len(identity)>256:return self._abort('invalid_frame_id')
        if identity in self.seen:return self._abort('replayed_frame')
        if type(t) not in (int,float) or not math.isfinite(t) or t<0:return self._abort('invalid_capture_time')
        if not 0<=now_s-t<=.25:return self._abort('stale_or_future_frame')
        if t<=self.last_capture:return self._abort('out_of_order_frame')
        if self.last_capture>=0 and t-self.last_capture>.250001:return self._abort('capture_stream_gap')
        self.last_frame_id=identity;self.last_capture=t;self.seen.add(identity)
        if self.stage=='waiting_feedback':
            if now_s-self.feedback_started>=1.:return self._abort('segment_feedback_timeout_pose_uncertain')
            self.reason='waiting_for_bound_segment_feedback';return self._output()
        leaf=self.leaf.update(frame,now_s);self.last_leaf=leaf
        if leaf['state']=='aborted':return self._abort('primitive_'+leaf['reason'])
        if self.stage in ('seek_initial','seek_next') and self.leaf.entry_ready:
            self.state='observing';return self._confirm_node(frame,now_s)
        if self.stage in ('seek_initial','seek_next'):
            self.window=[];self.state='moving' if leaf['v_m_s'] or leaf['omega_rad_s'] else 'observing'
        if self.stage=='turn_and_exit' and leaf['state']=='visual_stop':
            if not self.pending:return self._abort('unbound_visual_exit')
            self.heading=(self.heading+(90 if self.pending['action']=='left' else -90))%360
            self.expected_event=dict(event_id=self.pending['command_id']+':visual-exit',command_id=self.pending['command_id'],
                                     status='visual_segment_complete',frame_id=identity,captured_sim_s=t)
            self.feedback_started=now_s;self.state='waiting_feedback';self.stage='waiting_feedback';self.reason='visual_exit_is_not_node_arrival'
            return self._output()
        self.reason='rgb_'+self.stage+'_'+leaf['phase']
        return self._output(leaf['v_m_s'],leaf['omega_rad_s'])

    def feedback(self,event,now_s):
        """Consume only the exact RGB-bound leaf exit; never a node-arrival flag.

        The runner may deliver, duplicate or drop this output-only event. A
        fresh subsequent node confirmation is mandatory even after delivery.
        """
        import copy
        if self.state in ('visual_stop','aborted'):return self._output()
        error=self._clock(now_s)
        if error:return self._abort(error)
        fields={'event_id','command_id','status','frame_id','captured_sim_s'}
        if not isinstance(event,dict) or set(event)!=fields:return self._abort('invalid_segment_feedback')
        # bool is an int subclass: validate BEFORE duplicate equality so True
        # cannot impersonate a previously consumed captured_sim_s of 1.0.
        if any(not isinstance(event[k],str) or not event[k] or len(event[k])>256 for k in ('command_id','status','frame_id')):return self._abort('invalid_segment_feedback')
        if type(event['captured_sim_s']) not in (int,float) or not math.isfinite(event['captured_sim_s']):return self._abort('invalid_segment_feedback')
        identity=event['event_id']
        if not isinstance(identity,str) or not identity or len(identity)>256:return self._abort('invalid_feedback_id')
        if identity in self.event_records:
            if self.event_records[identity]!=event:return self._abort('conflicting_duplicate_feedback_pose_uncertain')
            self.duplicate_feedback+=1;return self._output()
        t=event['captured_sim_s']
        if type(t) not in (int,float) or not math.isfinite(t) or not 0<=now_s-t<=.25:return self._abort('stale_segment_feedback_pose_uncertain')
        if self.stage!='waiting_feedback' or self.expected_event!=event:return self._abort('unbound_segment_feedback_pose_uncertain')
        self.event_records[identity]=copy.deepcopy(event);self.exit_consumed=True;self.last_exit_capture=t
        self.leaf=CornerController(direction=None,junction='t',pause_at_entry=True,initial_clearance=self.initial_clearance)
        self.stage='seek_next';self.state='observing';self.reason='segment_consumed_next_node_unverified';self.window=[]
        return self._output()

    def result(self):
        import copy
        kind='completed_continuous_route' if self.state=='visual_stop' else ('aborted_motion_uncertain' if self.motion_commands else 'rejected_no_motion') if self.state=='aborted' else 'in_progress'
        return copy.deepcopy(dict(result_type=kind,state=self.state,stage=self.stage,reason=self.reason,
            start=self.start,goal=self.goal,heading_deg=self.heading,node_estimate=self.node_estimate,
            last_confirmed_node=self.last_confirmed_node,pending_command=self.pending,
            position_status='visually_confirmed_map_node' if self.state=='visual_stop' else 'uncertain',
            arrival_status='visually_confirmed_map_node' if self.state=='visual_stop' else 'unverified',atomic_edge_compatible=False,
            motion_commands=self.motion_commands,command_count=self.command_count,transition_count=self.transitions,
            duplicate_feedback=self.duplicate_feedback,node_confirmations=self.node_confirmations,commands=self.commands,
            consumed_segment_events=list(self.event_records.values()),
            declared_domain='mapped natural wall reference RGB, static unpainted calibrated flat orthogonal corridors; stem-entry L/T/dead-end stations only; no arbitrary viewpoint relocalisation or real-car guarantee'))
