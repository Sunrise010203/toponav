import cv2
import numpy as np
import pytest
from roadvision.free_ground_path import assess_free_ground
from roadvision.calibrated_ground_motion import GroundCalibration,_camera_motion
from roadvision.rgb_ground_rig import rig_basis
from roadvision.corridor_rgb_probe import CALIBRATION_ID


def synthetic_window(plane_normal=(0.,0.,-1.),distance=.45):
    rng=np.random.default_rng(791)
    first=cv2.GaussianBlur(rng.integers(20,240,(128,128),dtype=np.uint8),(3,3),.6)
    m=dict(status='conditional_ground_motion',translation_m=[.12,0.],yaw_delta_rad=0.,
           translation_reference_frame_id='f0',from_time_s=0.,to_time_s=.8)
    r,t=_camera_motion('front',m);k=np.array([[64.,0,64.],[0,64.,64.],[0,0,1.]])
    h=k@(r+np.outer(t,rig_basis('front').T@np.array(plane_normal)/distance))@np.linalg.inv(k)
    last=cv2.warpPerspective(first,h,(128,128))
    frames=[]
    for i,(time,a) in enumerate([(0.,first),(.4,first),(.8,last)]):
        panel=np.tile(a,(2,2));rgb=np.repeat(panel[:,:,None],3,axis=2)
        frames.append(dict(frame_id=f'f{i}',captured_sim_s=time,rgb=rgb))
    return frames,m


def test_textured_ground_correspondence_supports_only_conditional_free_path():
    frames,m=synthetic_window();r=assess_free_ground(frames,m,now_s=.8,calibration_id=CALIBRATION_ID,calibration=GroundCalibration())
    assert r['status']=='conditional_free_ground_path'
    assert r['sample_horizon_m']==1.2 and r['conditional_stopping_clearance_m']==.6 and r['cell_count']==9
    assert r['motion_authorized'] is False and r['arrival_status']=='unverified'


def test_a_textured_vertical_blocker_is_not_free_ground():
    frames,m=synthetic_window(plane_normal=(1.,0.,0.),distance=1.)
    r=assess_free_ground(frames,m,now_s=.8,calibration_id=CALIBRATION_ID,calibration=GroundCalibration())
    assert r['status']=='unknown'


def test_static_flat_and_wrong_reference_never_supply_clearance():
    frames,m=synthetic_window()
    for f in frames:f['rgb'][:]=128
    assert assess_free_ground(frames,m,now_s=.8,calibration_id=CALIBRATION_ID,calibration=GroundCalibration())['status']=='unknown'
    frames,m=synthetic_window();m['translation_reference_frame_id']='foreign'
    assert assess_free_ground(frames,m,now_s=.8,calibration_id=CALIBRATION_ID,calibration=GroundCalibration())['status']=='unknown'


@pytest.mark.parametrize('distance',[.4,.5,.6,.8])
@pytest.mark.parametrize('offset,width',[(0.,.5),(.20,.25),(-.20,.25)])
def test_physically_occluding_blocker_inside_stopping_domain_rejects(distance,offset,width):
    floor,m=synthetic_window();wall,_=synthetic_window(plane_normal=(1.,0.,0.),distance=distance)
    current_distance=distance-.12
    yy,xx=np.indices((128,128));left=64-64*(offset+width/2)/current_distance;right=64-64*(offset-width/2)/current_distance
    mask=(yy<=64+.45*64/current_distance)&(xx>=left)&(xx<=right)
    floor[-1]['rgb'][:128,:128][mask]=wall[-1]['rgb'][:128,:128][mask]
    r=assess_free_ground(floor,m,now_s=.8,calibration_id=CALIBRATION_ID,calibration=GroundCalibration())
    assert r['status']=='unknown'
