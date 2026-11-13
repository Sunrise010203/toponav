import copy
import numpy as np
import pytest
from roadvision.vertical_support import estimate_supported_vertical
from roadvision.calibrated_ground_motion import GroundCalibration
from tests.test_calibrated_ground_motion import plane_region


def motion(t=.12):
    return dict(status='conditional_ground_motion',translation_m=[t,0.],yaw_delta_rad=0.,
                conditional_uncertainty=dict(covariance_se2=np.diag([.001**2,.001**2,.0005**2]).tolist()))


def test_vertical_plane_has_reference_and_uncertainty():
    r=estimate_supported_vertical(plane_region(),motion(),GroundCalibration())
    assert r['status']=='conditional_vertical_plane'
    assert abs(r['distance_m']-1.2)<.001
    assert r['conditional_interval']['distance_half_width_m']>0
    assert not r['conditional_interval']['certified_error_bound']


def test_without_motion_uncertainty_no_plane_support():
    m=motion();del m['conditional_uncertainty']
    assert estimate_supported_vertical(plane_region(),m,GroundCalibration())['status']=='unknown'


def test_uncertain_motion_and_source_failure_are_not_overridden():
    m=motion();m['conditional_uncertainty']['covariance_se2']=(np.eye(3)*.1).tolist()
    assert estimate_supported_vertical(plane_region(),m,GroundCalibration())['status']=='unknown'
    r=plane_region();r['reasons']=['temporal_models_inconsistent']
    assert estimate_supported_vertical(r,motion(),GroundCalibration())['status']=='unknown'


@pytest.mark.parametrize('t',[.03,.06,.12,.24])
def test_two_side_walls_are_not_a_front_wall_under_weak_motion(t):
    a=np.array([[u,v] for v in (12.,28.,44.,60.) for u in (14.,22.,30.,38.,46.,82.,90.,98.,106.,114.)])
    rays=(a-64.)/64.;depth=.9/np.abs(rays[:,0]);b=64.+64.*rays*depth[:,None]/(depth[:,None]-t)
    r=dict(view='front',tracks=[dict(first=p.tolist(),last=q.tolist(),feature_split='train' if i%2==0 else 'validation') for i,(p,q) in enumerate(zip(a,b))])
    assert estimate_supported_vertical(r,motion(t),GroundCalibration())['status']=='unknown'


def test_floor_cannot_be_a_vertical_plane():
    assert estimate_supported_vertical(plane_region(normal=(0.,0.,-1.),distance=.45),motion(),GroundCalibration())['status']=='unknown'
