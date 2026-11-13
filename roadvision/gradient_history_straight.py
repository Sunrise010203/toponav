"""Opt-in composition of existing training-only history and gradient transfer.

Not promoted: changing the history changes the accepted set, not just compute.
Select before ONE final geometry fit; never retry a rejected fit/window or
reuse an old acceptance. Freshness, stationary confirmation, recovery budget,
commands and all descriptor/model gates remain inherited and unchanged.
"""
from collections import deque
from copy import deepcopy

from roadvision.gradient_straight import GradientTransferStraightController
from roadvision.gradient_straight import experiment_contract as gradient_contract
from roadvision.straight_ablation import HISTORY_SELECTION_POLICY, select_navigation_history


def experiment_contract():
    return dict(gradient_contract(), variant='gradient_history',
                history_selection_policy=deepcopy(HISTORY_SELECTION_POLICY),
                history_selection_scope='training_only_before_one_final_fit',
                alternative_fit_fallback=False)


class GradientHistoryStraightController(GradientTransferStraightController):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.frames = deque(maxlen=17)
        self.selection_receipt = None

    def _navigation_history(self, now_s):
        selected, receipt = select_navigation_history(
            self.frames, now_s=now_s, calibration=self.calibration)
        self.selection_receipt = dict(receipt, evaluated_sim_s=now_s,
                                      evaluated_frame_id=self.last_frame_id)
        return selected

    def _output(self, v=0., omega=0.):
        out = super()._output(v, omega)
        out['experiment'] = dict(experiment_contract(), history_selection=deepcopy(self.selection_receipt))
        return out

    def result(self):
        return dict(super().result(), experiment=experiment_contract())
