"""Honest historical keyframes: no relaxed frozen probe envelope."""
import copy
import numpy as np
import pytest

from roadvision.ground_history import validate_history, transform_plane_to_latest
from roadvision.corridor_rgb_probe import CALIBRATION_ID


def frames():
    return [dict(frame_id=f'f{i}', captured_sim_s=t, rgb=np.zeros((256,256,3),np.uint8))
            for i,t in enumerate((1.,1.8,2.6))]


def test_live_freshness_is_separate_from_bounded_history():
    refs=validate_history(frames(),now_s=2.7,calibration_id=CALIBRATION_ID)
    assert refs[0]['captured_sim_s']==1. and refs[-1]['captured_sim_s']==2.6
    with pytest.raises(ValueError):validate_history(frames(),now_s=2.86,calibration_id=CALIBRATION_ID)


@pytest.mark.parametrize('kind',['replay','order','future','long','truth','nan','bool'])
def test_history_rejects_malformed_and_unauthorized_fields(kind):
    f=frames();now=2.6
    if kind=='replay':f[2]['frame_id']=f[0]['frame_id']
    if kind=='order':f[1]['captured_sim_s']=.9
    if kind=='future':now=2.5
    if kind=='long':f[0]['captured_sim_s']=.1
    if kind=='truth':f[0]['pose']=[0,0]
    if kind=='nan':f[1]['captured_sim_s']=float('nan')
    if kind=='bool':f[1]['captured_sim_s']=True
    with pytest.raises(ValueError):validate_history(f,now_s=now,calibration_id=CALIBRATION_ID)


def test_plane_transform_uses_camera_translation_and_latest_coordinates():
    plane=dict(status='conditional_vertical_plane',normal_rig=[1.,0.,0.],distance_m=2.)
    motion=dict(translation_m=[.3,.1],yaw_delta_rad=np.pi/2)
    p=transform_plane_to_latest(plane,motion)
    np.testing.assert_allclose(p['normal_rig'],[0.,-1.,0.],atol=1e-10)
    assert p['distance_m']==pytest.approx(1.7)
    assert plane['distance_m']==2.


def test_local_composition_carries_rotated_translation_and_rejects_gaps():
    from roadvision.ground_history import compose_ground_steps
    s=dict(status='conditional_ground_motion',translation_m=[.1,0.],yaw_delta_rad=np.pi/2)
    m=compose_ground_steps([s,s]);np.testing.assert_allclose(m['translation_m'],[.1,.1],atol=1e-10)
    assert m['yaw_delta_rad']==pytest.approx(np.pi)
    assert compose_ground_steps([s,dict(status='unknown')])['status']=='unknown'


def test_history_model_selection_never_uses_validation_or_acceptance_scores():
    from roadvision.ground_history import choose_history_model
    regions={v:dict(tracks=[dict(first=[20+15*(i%4),100+7*(i//4)],last=[21+15*(i%4),101+7*(i//4)],feature_split='train') for i in range(8)]) for v in ('front','left')}
    assert choose_history_model(regions)['model']=='direct_long_ground'
    modified=copy.deepcopy(regions)
    for r in modified.values():
        r['status']='unknown';r['heldout_score']=float('inf')
        r['tracks'] += [dict(first=[99.,99.],last=[20.,30.],feature_split='validation')]*20
    assert choose_history_model(modified)==choose_history_model(regions)
    assert choose_history_model({'front':regions['front']})['model']=='short_increment_chain'


def test_adaptive_history_cannot_fallback_after_heldout_fit_failure(monkeypatch):
    import roadvision.ground_history as module
    f=[dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb=np.zeros((256,256,3),np.uint8)) for i in range(9)]
    regions={v:dict(tracks=[dict(first=[20+15*(i%4),100+7*(i//4)],last=[21+15*(i%4),101+7*(i//4)],feature_split='train') for i in range(8)]) for v in ('front','left')}
    unknown=dict(status='unknown',reason='heldout_prediction_failed',translation_m=None,yaw_delta_rad=None)
    monkeypatch.setattr(module,'_ground_evidence',lambda *a:(unknown,regions,{}))
    def forbidden(*a,**k):raise AssertionError('No acceptance-set fallback')
    monkeypatch.setattr(module,'probe_chained_history',forbidden)
    r=module.probe_adaptive_history(f,now_s=.8,calibration_id=CALIBRATION_ID)
    assert r['ground_motion']['status']=='unknown'
    assert r['history_model_selection']['model']=='direct_long_ground'


def test_adaptive_history_validates_non_keyframe_envelopes_before_images(monkeypatch):
    import roadvision.ground_history as module
    f=[dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb=np.zeros((256,256,3),np.uint8)) for i in range(9)]
    f[1]['truth']=[0,0]
    def forbidden(*a,**k):raise AssertionError('Must validate ALL envelopes first')
    monkeypatch.setattr(module,'_ground_evidence',forbidden)
    with pytest.raises(ValueError):module.probe_adaptive_history(f,now_s=.8,calibration_id=CALIBRATION_ID)
