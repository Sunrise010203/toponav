"""Opt-in wider fixed side sampling, not promoted ordinary navigation.

Uses more actual upper-side RGB support at the SAME feature density and
unchanged descriptor, temporal, train/heldout, residual and interval gates.
This can include end-wall corners and must reject an inconsistent model;
there is no central-ROI fallback or search for an accepted fit. Ground,
front, history, controller, permission and braking policies are unchanged.
Does not fix near-field coverage or verify the parallel-wall assumption.
"""
from roadvision.reference_straight import ReferenceStraightController
from roadvision.straight_evidence import probe_straight_history
from roadvision.corridor_rgb_probe import CALIBRATION_ID


def experiment_contract():
    return dict(variant='wide_reference',promoted=False,
        startup_selection='joint_reference_native_rectified_v1',
        side_sampling='fixed_wide_upper_v1',side_roi_bounds=[12,116,8,64],
        alternative_fit_fallback=False)


class WideReferenceStraightController(ReferenceStraightController):
    def _probe_navigation_history(self,frames,now_s):
        result=probe_straight_history(frames,now_s=now_s,
            calibration_id=CALIBRATION_ID,calibration=self.calibration,side_roi='wide')
        result['side_sampling']=experiment_contract()
        return result

    def _output(self,v=0.,omega=0.):
        out=super()._output(v,omega)
        out['experiment']=experiment_contract()
        return out

    def result(self):
        return dict(super().result(),experiment=experiment_contract())
