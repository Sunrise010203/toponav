import numpy as np
import pytest
from roadvision import straight_history_support as module
from roadvision.corridor_rgb_probe import CALIBRATION_ID


def frames():
    rgb=np.random.default_rng(9).integers(0,255,(256,256,3),dtype=np.uint8)
    return [dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb=rgb) for i in range(9)]


@pytest.mark.parametrize('fault',['duplicate','interior_truth','interior_order','stale_latest','too_long'])
def test_entire_real_history_validated_before_any_pixel_work(monkeypatch,fault):
    fs=frames();now=.8
    if fault=='duplicate':fs[3]['frame_id']='f2'
    elif fault=='interior_truth':fs[3]['true_pose']=[0,0]
    elif fault=='interior_order':fs[3]['captured_sim_s']=.1
    elif fault=='stale_latest':now=1.2
    else:
        for i,f in enumerate(fs):f['captured_sim_s']=i*.25
        now=2.
    def forbidden(*a,**k):raise AssertionError('pixel work before validation')
    monkeypatch.setattr(module,'_ground_evidence',forbidden)
    with pytest.raises(ValueError):module.prepare_straight_history(fs,now_s=now,calibration_id=CALIBRATION_ID)


def test_side_detection_roi_is_fixed_central_not_truth_or_heldout_selected(monkeypatch):
    calls=[]
    def ground(*a):return ({'status':'conditional_ground_motion','translation_m':[.1,0.],'yaw_delta_rad':0.},{},{})
    monkeypatch.setattr(module,'_ground_evidence',ground)
    monkeypatch.setattr(module,'choose_history_model',lambda *_:{'model':'direct_long_ground'})
    def track(images,*,view,bounds):
        calls.append((view,bounds));return {'view':view,'tracks':[],'coverage_map':np.eye(3).tolist()}
    monkeypatch.setattr(module,'track_fixed_roi',track)
    monkeypatch.setattr(module,'filter_h',lambda ims,r:r)
    monkeypatch.setattr(module,'filter_i',lambda ims,r:r)
    monkeypatch.setattr(module,'select_descriptor',lambda *a:{'channel':'H'})
    module.prepare_straight_history(frames(),now_s=.8,calibration_id=CALIBRATION_ID)
    assert calls==[('left',(32,96,8,64)),('right',(32,96,8,64)),('front',(32,96,16,64))]
