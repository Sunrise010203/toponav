import copy
import math
import numpy as np
import pytest
from roadvision.joint_straight_model import estimate_joint_corridor
from roadvision.calibrated_ground_motion import ground_pixels,GroundCalibration,_rotation
from tests.test_paired_corridor import pair
from tests.test_vertical_support import motion


def ground():
    c=GroundCalibration();out={}
    for view in ('front','left','right'):
        a=np.array([[u,v] for u in (20.,36.,52.,68.,84.,100.) for v in (104.,108.,112.,116.)]);from roadvision.calibrated_ground_motion import ground_points
        xy=ground_points(a,view,c);b=ground_pixels(xy-[.12,0.],view,c)
        out[view]=dict(view=view,tracks=[dict(first=p.tolist(),last=q.tolist(),feature_split='train' if i%2==0 else 'validation') for i,(p,q) in enumerate(zip(a,b))])
    return out


def test_joint_ground_and_walls_can_remove_small_initial_pose_bias():
    m=motion();m['translation_m']=[.121,-.001];m['yaw_delta_rad']=.002
    r=estimate_joint_corridor(pair(),ground(),m,GroundCalibration())
    assert r['status']=='conditional_paired_corridor'
    assert r['heading_first_rad']==pytest.approx(.08,abs=1e-5)
    assert r['refined_motion']['translation_m']==pytest.approx([.12,0.],abs=1e-5)
    assert abs(r['refined_motion']['yaw_delta_rad'])<1e-5
    assert r['conditional_interval']['heading_half_width_rad']>0
    assert not r['certified_error_bound']


@pytest.mark.parametrize('fault',['original_unknown','ground_heldout','wall_heldout','ground_train_view_loss','temporal'])
def test_joint_fit_cannot_skip_original_support_or_per_view_heldout(fault):
    m=motion();g=ground();p=pair()
    if fault=='original_unknown':m['status']='unknown'
    elif fault=='ground_train_view_loss':g.pop('right');g.pop('left')
    elif fault=='temporal':p['right']['reasons']=['temporal_models_inconsistent']
    else:
        for t in (g['front'] if fault=='ground_heldout' else p['right'])['tracks']:
            if t['feature_split']=='validation':t['last'][0]+=3.
    assert estimate_joint_corridor(p,g,m,GroundCalibration())['status']=='unknown'
