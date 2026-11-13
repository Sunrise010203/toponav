import numpy as np
import cv2
from roadvision.ground_band import track_ground_band


def test_empty_ground_band_has_no_tracks():
    r=track_ground_band([np.zeros((128,128,3),np.uint8)]*3,view='front')
    assert not r['tracks'] and r['status']=='unknown'


def test_texture_above_the_ground_band_does_not_supply_features():
    rng=np.random.default_rng(818)
    a=np.zeros((128,128,3),np.uint8);a[:84]=rng.integers(0,256,(84,128,3),dtype=np.uint8)
    r=track_ground_band([a]*3,view='front')
    assert not r['tracks']


def test_a_ground_feature_may_leave_the_detection_band_while_still_visible():
    rng=np.random.default_rng(441)
    a=cv2.GaussianBlur(rng.integers(0,256,(128,128,3),dtype=np.uint8),(3,3),.6)
    images=[cv2.warpAffine(a,np.float32([[1,0,0],[0,1,-d]]),(128,128),borderMode=cv2.BORDER_REFLECT_101) for d in (0,4,8)]
    r=track_ground_band(images,view='right')
    assert any(t['last'][1]<104 for t in r['tracks'])
    assert all(104<=t['first'][1]<119 for t in r['tracks'])
