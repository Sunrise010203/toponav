"""Opt-in task4 following guard; no new startup or recovery permission."""
from roadvision.continuous_straight import StraightController
from roadvision.swept_ground_path import assess_swept_ground,SweepInstallation,task_contract
from roadvision.corridor_rgb_probe import CALIBRATION_ID


class SweptStraightController(StraightController):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.sweep_installation=SweepInstallation();self.last_swept_path=None

    def _following_guard(self,c,front,frames,packet,now_s):
        # Compute the SAME baseline candidate, without resetting an existing
        # wait/rearm budget or making the diagnostic computation a command.
        reason=self.reason
        try:v,omega=super()._following_command(c,front)
        finally:self.reason=reason
        self.last_swept_path=assess_swept_ground(frames,packet,v,omega,now_s=now_s,
            calibration_id=CALIBRATION_ID,calibration=self.calibration,policy=self.policy,installation=self.sweep_installation)
        return (self.last_swept_path['status']=='conditional_patch_supported_sweep' and
                self._current_evidence(self.last_swept_path) and
                (self.last_swept_path['requested_v_m_s'],self.last_swept_path['requested_omega_rad_s'])==(v,omega))

    def _output(self,v=0.,omega=0.):
        out=super()._output(v,omega)
        out['trajectory_guard']=dict(enabled=True,promoted=False,scope='following_only',
            bootstrap_authority='unchanged_explicit_initial_swept_path_prior',evidence=self.last_swept_path)
        return out

    def result(self):
        return dict(super().result(),trajectory_guard=dict(enabled=True,promoted=False,contract=task_contract()))
