import numpy as np
import pytest
from roadvision.front_support import estimate_perpendicular_front
from roadvision.calibrated_ground_motion import GroundCalibration
from tests.test_calibrated_ground_motion import plane_region
from tests.test_vertical_support import motion


def side_planes():
    return {v:dict(status='conditional_vertical_plane',conditional_interval=dict(normal_angle_half_width_rad=.01)) for v in ('left','right')}


def test_front_needs_two_side_directions_and_is_never_endpoint_identity():
    region=plane_region(view='front',normal=(1.,0.,0.),distance=1.2)
    r=estimate_perpendicular_front(region,motion(),GroundCalibration(),heading=0.,side_planes=side_planes())
    assert r['status']=='conditional_vertical_plane'
    assert abs(r['distance_m']-1.2)<.001
    assert r['arrival_status']=='unverified' and r['motion_authorized'] is False
    assert estimate_perpendicular_front(region,motion(),GroundCalibration(),heading=0.,side_planes={})['status']=='unknown'


def test_paint_ground_cannot_be_a_front_surface():
    region=plane_region(view='front',normal=(0.,0.,-1.),distance=.45)
    assert estimate_perpendicular_front(region,motion(),GroundCalibration(),heading=0.,side_planes=side_planes())['status']=='unknown'


@pytest.mark.parametrize('t',[.03,.06,.12,.24])
def test_side_wall_mixture_is_not_a_perpendicular_front(t):
    a=np.array([[u,v] for v in (12.,28.,44.,60.) for u in (14.,22.,30.,38.,46.,82.,90.,98.,106.,114.)])
    rays=(a-64.)/64.;depth=.9/np.abs(rays[:,0]);b=64.+64.*rays*depth[:,None]/(depth[:,None]-t)
    region=dict(view='front',tracks=[dict(first=p.tolist(),last=q.tolist(),feature_split='train' if i%2==0 else 'validation') for i,(p,q) in enumerate(zip(a,b))])
    assert estimate_perpendicular_front(region,motion(t),GroundCalibration(),heading=0.,side_planes=side_planes())['status']=='unknown'
