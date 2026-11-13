"""Explicit Sobel-transfer experiment over the unpromoted wide-side variant.

History, geometry, flow, commands, authorization and recovery are inherited.
Only the free-transfer appearance uniqueness representation changes. The
intensity margin is replaced; equal numbers do not mean equal acceptance.
"""
from roadvision.wide_reference_straight import WideReferenceStraightController
from roadvision.wide_reference_straight import experiment_contract as wide_contract
from roadvision.gradient_ground_path import assess_gradient_free_ground
from roadvision.corridor_rgb_probe import CALIBRATION_ID


def experiment_contract():
    return dict(wide_contract(),variant='gradient_transfer',
        free_path_descriptor='sobel_xy_uniqueness_v1',
        intensity_margin_replaced=True,native_intensity_and_flow_vetoes=True)


class GradientTransferStraightController(WideReferenceStraightController):
    def _assess_navigation_free_ground(self,frames,motion,now_s):
        return assess_gradient_free_ground(frames,motion,now_s=now_s,
            calibration_id=CALIBRATION_ID,calibration=self.calibration)

    def _output(self,v=0.,omega=0.):
        out=super()._output(v,omega)
        out['experiment']=experiment_contract()
        return out

    def result(self):
        return dict(super().result(),experiment=experiment_contract())
