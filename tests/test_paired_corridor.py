"""Explicit paired-parallel prior; internal synthetic, not independent data."""
import copy
import math
import numpy as np
import pytest
from roadvision.calibrated_ground_motion import GroundCalibration
from roadvision.paired_corridor import estimate_paired_corridor
from tests.test_calibrated_ground_motion import plane_region
from tests.test_vertical_support import motion


def pair(heading=.08):
    n=(math.sin(heading),math.cos(heading),0.)
    return {'left':plane_region('left',n,.74),'right':plane_region('right',tuple(-x for x in n),1.06)}


def test_joint_parallel_fit_reports_assumption_scale_and_noise_interval():
    r=estimate_paired_corridor(pair(),motion(),GroundCalibration())
    assert r['status']=='conditional_paired_corridor'
    assert r['heading_first_rad']==pytest.approx(.08,abs=1e-5)
    assert r['distances_first_m']==pytest.approx([.74,1.06],abs=1e-5)
    assert r['conditional_interval']['heading_half_width_rad']>0
    assert r['parallelism_assumed'] and not r['certified_error_bound']
    assert not r['motion_authorized'] and r['arrival_status']=='unverified'


@pytest.mark.parametrize('fault',['missing','uncertainty','temporal','floor','nonparallel','heldout'])
def test_invalid_or_mixed_evidence_does_not_inherit_a_free_corridor(fault):
    p=pair();m=motion()
    if fault=='missing':del p['right']
    elif fault=='uncertainty':m.pop('conditional_uncertainty')
    elif fault=='temporal':p['right']['reasons']=['temporal_models_inconsistent']
    elif fault=='floor':p['right']=plane_region('right',(0.,0.,-1.),.45)
    elif fault=='nonparallel':p['right']=plane_region('right',(.5,-math.sqrt(.75),0.),1.06)
    elif fault=='heldout':
        for t in p['right']['tracks']:
            if t['feature_split']=='validation':t['last'][0]+=3.
    r=estimate_paired_corridor(p,m,GroundCalibration())
    assert r['status']=='unknown'


def test_zero_or_uncertain_motion_cannot_fit_new_metric_corridor():
    for m in (motion(0.),motion()|{'conditional_uncertainty':{'covariance_se2':(np.eye(3)*.1).tolist()}}):
        assert estimate_paired_corridor(pair(),m,GroundCalibration())['status']=='unknown'
