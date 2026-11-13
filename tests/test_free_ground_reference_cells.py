import numpy as np
from roadvision import free_ground_path as module
from roadvision.calibrated_ground_motion import GroundCalibration
from roadvision.corridor_rgb_probe import CALIBRATION_ID
from tests.test_free_ground_path import synthetic_window


def test_reference_only_selection_stays_inside_each_one_centimeter_cell():
    frames,m=synthetic_window();gray=frames[0]['rgb'][:128,:128,0]
    cells=module.select_reference_cells(gray,np.ones_like(gray),GroundCalibration())
    assert len(cells)==9
    for c in cells:
        delta=np.array(c['point_latest_camera_xy_m'])-c['nominal_point_latest_camera_xy_m']
        assert np.abs(delta).max()<=.01000001
        assert c['reference_accepted']
    assert cells==module.select_reference_cells(gray,np.ones_like(gray),GroundCalibration())


def test_flat_reference_cannot_select_fake_unique_patches():
    gray=np.full((128,128),128,np.uint8)
    cells=module.select_reference_cells(gray,np.ones_like(gray),GroundCalibration())
    assert all(not c['reference_accepted'] for c in cells)


def test_one_target_check_per_preselected_cell_no_retry_after_rejection(monkeypatch):
    targets=[]
    def evidence(a,b,p,q,*masks):
        if a is not b:targets.append(list(p))
        return dict(accepted=a is b,self_alternative_ncc=.1,reason='source_unique' if a is b else 'forced_heldout_rejection')
    monkeypatch.setattr(module,'appearance_evidence',evidence)
    frames,m=synthetic_window();r=module.assess_free_ground(frames,m,now_s=.8,calibration_id=CALIBRATION_ID)
    assert len(targets)==9 and r['status']=='unknown'
    assert all(c['reason']=='forced_heldout_rejection' for c in r['cells'])
