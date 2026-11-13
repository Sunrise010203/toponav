"""Experimental task5 additional guards and read-only recovery proposals.

Baseline startup/rearm/stop remain intact. No active proposal is ever sent
as a velocity; waiting stays zero. This is deliberately not a production
recovery executor, since real near-body swept coverage remains unproved.
"""
from roadvision.continuous_straight import StraightController
from roadvision.corridor_rgb_probe import CALIBRATION_ID
from roadvision.swept_ground_path import assess_swept_ground,task_contract as swept_contract
from roadvision.rear_observation import assess_rear_consistency,review_active_proposal,task_contract


class RearStraightController(StraightController):
    def __init__(self,*,active_review=False,**kwargs):
        super().__init__(**kwargs)
        if type(active_review) is not bool:raise ValueError('Explicit bool required')
        self.active_review=active_review
        self.last_rear=None;self.last_swept_path=None
        self.last_active_review=None;self.last_active_sweep=None

    def _rear(self,frames,packet,now_s):
        if not self.last_rear or not self._current_evidence(self.last_rear):
            self.last_rear=assess_rear_consistency(frames,packet,now_s=now_s,
                calibration_id=CALIBRATION_ID,calibration=self.calibration)
        return self.last_rear

    def _sweep(self,frames,packet,v,w,now_s):
        return assess_swept_ground(frames,packet,v,w,now_s=now_s,
            calibration_id=CALIBRATION_ID,calibration=self.calibration,policy=self.policy,include_back=True)

    def _following_guard(self,c,front,frames,packet,now_s):
        rear=self._rear(frames,packet,now_s)
        reason=self.reason
        try:v,w=super()._following_command(c,front)
        finally:self.reason=reason
        # Evaluate coverage even if rear unknown: retain the geometric reason
        # for refusal; never search alternative frames/views for a passing fit.
        self.last_swept_path=self._sweep(frames,packet,v,w,now_s)
        g=self.last_swept_path
        return (rear['status']=='conditional_rear_consistent' and self._current_evidence(rear) and
            g['status']=='conditional_patch_supported_sweep' and self._current_evidence(g) and
            (g['requested_v_m_s'],g['requested_omega_rad_s'])==(v,w))

    def _output(self,v=0.,omega=0.):
        # Diagnostics run once per current real geometry packet. The proposal
        # receives no velocity connection and cannot change parent state.
        packet=self.last_perception
        if self.state not in ('aborted','visual_stop') and packet and self._current_evidence(packet) and len(self.frames)>=9:
            frames=list(self.frames);keyframes=[frames[0],frames[len(frames)//2],frames[-1]]
            rear=self._rear(keyframes,packet,self.last_now)
            if (self.active_review and self.state=='waiting_evidence' and
                (not self.last_active_review or not self._current_evidence(self.last_active_review))):
                self.last_active_sweep=self._sweep(keyframes,packet,.08,0.,self.last_now)
                self.last_active_review=review_active_proposal(now_s=self.last_now,
                    frame_id=self.last_frame_id,captured_s=self.last_capture,state=self.state,
                    permission=self.motion_permission,stationary_frames=self.stationary_frames,
                    recovery_count=self.recovery_count,attempts=0,wait_started=self.unverified_started,
                    probe_distance_bound_m=self.probe_distance_bound,policy=self.policy,
                    sweep=self.last_active_sweep,rear=rear)
        out=super()._output(v,omega)
        out['rear_observation']=dict(enabled=True,promoted=False,rear=self.last_rear,
            following_sweep=self.last_swept_path,active_review=self.last_active_review,
            active_sweep=self.last_active_sweep,active_executed=False)
        return out

    def result(self):
        return dict(super().result(),rear_observation=dict(promoted=False,
            contract=task_contract(self.active_review),trajectory_contract=swept_contract(),
            active_executed=False))
