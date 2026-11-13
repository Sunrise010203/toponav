import math
import numpy as np
from roadvision import straight_evidence as module
from roadvision.calibrated_ground_motion import GroundCalibration
from tests.test_paired_corridor import pair
from tests.test_vertical_support import motion


def packet():
    m=motion()|{'translation_reference_frame_id':'f0','from_time_s':0.,'to_time_s':.8}
    return dict(ground_motion=m,frames=[{'frame_id':'f0','captured_sim_s':0.},{'frame_id':'f4','captured_sim_s':.4},{'frame_id':'f8','captured_sim_s':.8}],
        regions={},history_model_selection={'model':'stub'},diagnostics={v:{'plane_region':r} for v,r in pair().items()}|{'front':{'plane_region':{'view':'front','tracks':[]}}},
        corridor_geometry={'status':'conditional_corridor_geometry'},front_surface={'status':'conditional_front_surface'})


def test_new_model_never_or_falls_back_to_s1_independent_side_success(monkeypatch):
    monkeypatch.setattr(module,'prepare_straight_history',lambda *a,**k:packet())
    monkeypatch.setattr(module,'estimate_paired_corridor',lambda *a,**k:{'status':'unknown','reason':'test_rejection'})
    r=module.probe_straight_history([],now_s=.8,calibration_id='ignored_stub',calibration=GroundCalibration())
    assert r['corridor_geometry']['status']=='unknown'
    assert r['front_surface']['status']=='unknown'
    assert not r['motion_authorized']


def test_joint_geometry_is_in_latest_not_first_coordinates(monkeypatch):
    p=packet();p['ground_motion']['yaw_delta_rad']=.04
    monkeypatch.setattr(module,'prepare_straight_history',lambda *a,**k:p)
    monkeypatch.setattr(module,'estimate_paired_corridor',lambda *a,**k:dict(status='conditional_paired_corridor',heading_first_rad=.08,distances_first_m=[.74,1.06],conditional_interval={'distance_half_widths_m':[.03,.03],'heading_half_width_rad':.01}))
    monkeypatch.setattr(module,'estimate_perpendicular_front',lambda *a,**k:{'status':'unknown','reason':'fixture'})
    r=module.probe_straight_history([],now_s=.8,calibration_id='ignored_stub',calibration=GroundCalibration())
    c=r['corridor_geometry']
    assert c['status']=='conditional_corridor_geometry'
    assert abs(c['heading_error_rad']-(.08+.04))<1e-8
    assert r['geometry_reference_frame_id']=='f8'


def test_history_train_selection_fixes_one_estimator_without_retry(monkeypatch):
    for model,expected in [('direct_long_ground','joint'),('short_increment_chain','fixed')]:
        p=packet();p['history_model_selection']['model']=model;calls=[]
        monkeypatch.setattr(module,'prepare_straight_history',lambda *a,**k:p)
        def joint(*a,**k):
            calls.append('joint');return {'status':'unknown','reason':'reject'}
        def fixed(*a,**k):
            calls.append('fixed');return {'status':'unknown','reason':'reject'}
        monkeypatch.setattr(module,'estimate_joint_corridor',joint)
        monkeypatch.setattr(module,'estimate_paired_corridor',fixed)
        r=module.probe_straight_history([],now_s=.8,calibration_id='stub')
        assert calls==[expected]
        assert r['corridor_geometry']['status']=='unknown'
