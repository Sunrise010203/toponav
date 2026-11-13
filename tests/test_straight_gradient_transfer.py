"""Fixed gradient descriptor contracts, not physical navigation acceptance."""
import copy
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest
from PIL import Image

from roadvision import gradient_ground_path as gradient
from roadvision.gradient_straight import GradientTransferStraightController
from roadvision.wide_reference_straight import WideReferenceStraightController
from roadvision.continuous_straight import StraightController,StraightPolicy,InitialClearancePrior
from roadvision.free_ground_path import assess_free_ground
from roadvision.corridor_rgb_probe import CALIBRATION_ID

FIXTURE=Path(__file__).parent/'fixtures/straight_gradient'


def exposed():
    data=json.loads((FIXTURE/'manifest.json').read_text());frames=[]
    for e in data['frames']:
        rgb=np.array(Image.open(FIXTURE/e['rgb_path']))
        assert hashlib.sha256(rgb.tobytes()).hexdigest()==e['rgb_sha256']
        frames.append(dict(frame_id=e['frame_id'],captured_sim_s=e['captured_sim_s'],rgb=rgb))
    return frames,data['motion']


def assess(frames,motion):
    return gradient.assess_gradient_free_ground(frames,motion,now_s=frames[-1]['captured_sim_s'],calibration_id=CALIBRATION_ID)


def test_exposed_competing_intensity_uses_fixed_gradient_not_retry_or_relaxed_flow():
    frames,m=exposed();raw=assess_free_ground(frames,m,now_s=1.3,calibration_id=CALIBRATION_ID)
    out=assess(frames,m)
    assert raw['status']=='unknown' and raw['cells'][0]['reason']=='competing_match'
    assert out['status']=='conditional_free_ground_path'
    assert out['descriptor_policy']['intensity_margin_replaced']
    for old,new in zip(raw['cells'],out['cells']):
        assert new['intensity_evidence']==old
        assert old['pixel_latest']==new['pixel_latest']
        assert old['ground_flow_supported'] and old['ground_flow_residual_px']<=.5 and old['flow_fb_px']<=.25
        assert new['gradient_evidence']['observed_ncc']>=.7
        assert new['gradient_evidence']['margin']>=.06
        assert new['gradient_evidence']['self_alternative_ncc']<.94
    assert not out['motion_authorized'] and not out['certified_clearance'] and out['arrival_status']=='unverified'


def test_vector_ncc_agrees_with_opencv_per_channel_mean_convention():
    rng=np.random.default_rng(717)
    a=rng.normal(size=(11,11,3)).astype(np.float32)
    b=a*np.array([2.,1.,.5],np.float32)+np.array([300.,-200.,500.],np.float32)
    expected=float(cv2.matchTemplate(b,a,cv2.TM_CCOEFF_NORMED)[0,0])
    assert gradient.gradient_ncc(a,b)==pytest.approx(expected,abs=2e-3)
    assert gradient.gradient_ncc(np.ones_like(a),np.ones_like(b)) is None


def test_derivative_footprint_rejects_padding_outside_native_patch():
    gray=np.random.default_rng(17).integers(0,255,(128,128),dtype=np.uint8)
    mask=np.ones_like(gray);mask[:,58]=0
    canvas,support=gradient.gradient_canvas(gray,mask)
    # Native 11px patch is x59..69. Its Sobel footprint also needs x58.
    from roadvision.rgb_resampling_checks import _supported_patch
    assert _supported_patch(gray,[64.,64.],mask) is not None
    assert _supported_patch(canvas,[64.,64.],support) is None


def stub_packet():
    frames,m=exposed();p=assess_free_ground(frames,m,now_s=1.3,calibration_id=CALIBRATION_ID)
    for c in p['cells']:
        c.update(reference_accepted=True,observed_ncc=.95,self_alternative_ncc=.8,
            margin=.01,ground_flow_supported=True,accepted=False,reason='competing_match')
    return frames,m,p


@pytest.mark.parametrize('field,value',[
    ('reference_accepted',False),('observed_ncc',.69999),('observed_ncc',None),
    ('self_alternative_ncc',.94),('self_alternative_ncc',None),('ground_flow_supported',False),
])
def test_gradient_never_rescues_native_or_flow_veto(monkeypatch,field,value):
    frames,m,p=stub_packet();p['cells'][4][field]=value
    monkeypatch.setattr(gradient,'assess_free_ground',lambda *a,**k:copy.deepcopy(p))
    monkeypatch.setattr(gradient,'gradient_appearance',lambda *a,**k:dict(accepted=True,reason='synthetic_gradient_support'))
    out=assess(frames,m)
    assert out['status']=='unknown' and out['conditional_stopping_clearance_m'] is None


def test_gradient_is_mandatory_even_when_baseline_passes(monkeypatch):
    frames,m,p=stub_packet();p['status']='conditional_free_ground_path'
    for c in p['cells']:c.update(accepted=True,margin=.2,reason='unique_patch_support')
    monkeypatch.setattr(gradient,'assess_free_ground',lambda *a,**k:copy.deepcopy(p))
    calls=[]
    def evidence(*args):
        calls.append(1)
        return dict(accepted=len(calls)!=5,reason='synthetic_gradient_rejection')
    monkeypatch.setattr(gradient,'gradient_appearance',evidence)
    out=assess(frames,m)
    assert len(calls)==9 and out['status']=='unknown' and not out['cells'][4]['accepted']


@pytest.mark.parametrize('count',[0,8])
def test_empty_or_incomplete_cells_never_get_clearance(monkeypatch,count):
    frames,m,p=stub_packet();p['cells']=p['cells'][:count];p['status']='unknown'
    monkeypatch.setattr(gradient,'assess_free_ground',lambda *a,**k:copy.deepcopy(p))
    monkeypatch.setattr(gradient,'gradient_appearance',lambda *a,**k:dict(accepted=True,reason='test'))
    assert assess(frames,m)['status']=='unknown'


def synthetic_window(normal=(0.,0.,-1.),distance=.45):
    from roadvision.calibrated_ground_motion import _camera_motion
    from roadvision.rgb_ground_rig import rig_basis
    first=cv2.GaussianBlur(np.random.default_rng(791).integers(20,240,(128,128),dtype=np.uint8),(3,3),.6)
    m=dict(status='conditional_ground_motion',translation_m=[.12,0.],yaw_delta_rad=0.,translation_reference_frame_id='f0',from_time_s=0.,to_time_s=.8)
    r,t=_camera_motion('front',m);k=np.array([[64.,0,64.],[0,64.,64.],[0,0,1.]])
    h=k@(r+np.outer(t,rig_basis('front').T@np.array(normal)/distance))@np.linalg.inv(k)
    last=cv2.warpPerspective(first,h,(128,128));frames=[]
    for i,(time,img) in enumerate([(0.,first),(.4,first),(.8,last)]):
        rgb=np.repeat(np.tile(img,(2,2))[:,:,None],3,axis=2)
        frames.append(dict(frame_id=f'f{i}',captured_sim_s=time,rgb=rgb))
    return frames,m


def test_conditional_textured_ground_is_not_certified_clearance():
    frames,m=synthetic_window();r=assess(frames,m)
    assert r['status']=='conditional_free_ground_path' and r['conditional_stopping_clearance_m']==.6
    assert r['certified_clearance'] is r['motion_authorized'] is False


@pytest.mark.parametrize('distance',[.4,.5,.6,.8])
@pytest.mark.parametrize('offset,width',[(0.,.5),(.20,.25),(-.20,.25)])
def test_minimum_size_occluding_blockers_still_reject(distance,offset,width):
    floor,m=synthetic_window();wall,_=synthetic_window((1.,0.,0.),distance)
    current=distance-.12;yy,xx=np.indices((128,128))
    mask=(yy<=64+.45*64/current)&(xx>=64-64*(offset+width/2)/current)&(xx<=64-64*(offset-width/2)/current)
    floor[-1]['rgb'][:128,:128][mask]=wall[-1]['rgb'][:128,:128][mask]
    assert assess(floor,m)['status']=='unknown'


@pytest.mark.parametrize('kind',['flat','repeated','foreign','no_baseline','vertical'])
def test_invalid_ground_or_appearance_is_unknown(kind):
    frames,m=synthetic_window()
    if kind=='flat':
        for f in frames:f['rgb'][:]=128
    elif kind=='repeated':
        yy,xx=np.indices((256,256));tile=((xx//4+yy//4)%2*100+60).astype(np.uint8)
        for f in frames:f['rgb']=np.repeat(tile[:,:,None],3,axis=2)
    elif kind=='foreign':m['translation_reference_frame_id']='foreign'
    elif kind=='no_baseline':m['translation_m']=[0.,0.]
    else:frames,m=synthetic_window((1.,0.,0.),1.)
    assert assess(frames,m)['status']=='unknown'


def test_controls_geometry_and_startup_are_inherited_unchanged():
    c=GradientTransferStraightController()
    assert c.policy==StraightPolicy() and c.motion_permission=='no_motion'
    for name in ('_update','_wait','_confirm_stationary','_clock','_stopping_reserve',
                 '_following_guard','_following_command','_navigation_history','missing','cancel'):
        assert getattr(type(c),name) is getattr(StraightController,name)
    for name in ('_assess_startup_texture','_probe_navigation_history'):
        assert getattr(type(c),name) is getattr(WideReferenceStraightController,name)


@pytest.mark.parametrize('confirmed,mode,reason',[(False,'bounded_probe','initial_clearance_prior_not_confirmed'),(True,'no_motion','motion_not_authorized')])
def test_authorization_is_not_changed(confirmed,mode,reason):
    frames,_=exposed();c=GradientTransferStraightController(initial_clearance=InitialClearancePrior(confirmed=confirmed),motion_permission=mode)
    r=c.update(dict(frame_id='start',captured_sim_s=0.,rgb=frames[0]['rgb']),0.)
    assert r['reason']==reason and r['v_m_s']==r['omega_rad_s']==0.


def test_baseline_hook_still_resolves_original_function(monkeypatch):
    import roadvision.continuous_straight as base
    calls=[];sentinel=object()
    def original(*args,**kwargs):calls.append((args,kwargs));return sentinel
    monkeypatch.setattr(base,'assess_free_ground',original)
    assert StraightController()._assess_navigation_free_ground([],{},1.) is sentinel
    assert len(calls)==1


def test_gradient_public_replay_binds_variant_and_terminal_stop(tmp_path):
    from simulation.continuous_straight_run import run_case
    from simulation.continuous_straight_acceptance import replay_safety_public
    base=tmp_path/'run'
    result=run_case(base,straight_variant='gradient_transfer',operator_confirmed=True,motion_permission='bounded_probe',max_ticks=10,terminal_check_ticks=3)
    rows=[json.loads(x) for x in (base/'decisions.jsonl').read_text().splitlines()]
    assert any((r['decision'].get('free_path') or {}).get('descriptor_policy') for r in rows)
    assert result['audit']['safe_final_stop']
    assert replay_safety_public(base,result,rows)['terminal_frames']==3
    task_path=base/'public/task.json';task=json.loads(task_path.read_text());task['straight_variant']='wide_reference';task_path.write_text(json.dumps(task))
    with pytest.raises(ValueError,match='differs'):replay_safety_public(base,result,rows)


@pytest.mark.parametrize('direction',['left','right'])
def test_gradient_does_not_mix_corner_control(tmp_path,direction):
    from simulation.continuous_straight_run import run_case
    with pytest.raises(ValueError,match='Straight-only'):
        run_case(tmp_path/'invalid',straight_variant='gradient_transfer',corner_direction=direction)
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('module',['gradient_ground_path.py','gradient_straight.py'])
def test_gradient_implementation_has_no_truth_or_simulation_imports(module):
    import ast
    tree=ast.parse((Path(gradient.__file__).parent/module).read_text())
    for node in ast.walk(tree):
        if isinstance(node,ast.ImportFrom):
            assert not (node.module or '').startswith(('simulation','pybullet'))
        elif isinstance(node,ast.Import):
            assert not any(n.name.startswith(('simulation','pybullet')) for n in node.names)
