"""Wide side sampling contracts; exposed RGB regression is not acceptance."""
from pathlib import Path
import ast
import copy
import hashlib
import json
import math

import numpy as np
from PIL import Image
import pytest

from roadvision import straight_history_support as support
from roadvision import wide_reference_straight as wide
from roadvision.calibrated_ground_motion import GroundCalibration
from roadvision.continuous_straight import StraightController, StraightPolicy, InitialClearancePrior
from roadvision.reference_straight import ReferenceStraightController
from roadvision.corridor_rgb_probe import CALIBRATION_ID

FIXTURE=Path(__file__).parent/'fixtures/straight_wide_side'


def exposed_frames():
    manifest=json.loads((FIXTURE/'manifest.json').read_text())
    frames=[]
    for e in manifest['frames']:
        rgb=np.array(Image.open(FIXTURE/e['rgb_path']))
        assert hashlib.sha256(rgb.tobytes()).hexdigest()==e['rgb_sha256']
        frames.append(dict(frame_id=e['frame_id'],captured_sim_s=e['captured_sim_s'],rgb=rgb))
    return frames


def test_wide_exposed_interval_uses_more_real_support_not_a_wider_gate():
    from roadvision.straight_evidence import probe_straight_history
    frames=exposed_frames();kwargs=dict(now_s=frames[-1]['captured_sim_s'],calibration_id=CALIBRATION_ID)
    central=probe_straight_history(frames,**kwargs)
    wider=probe_straight_history(frames,side_roi='wide',**kwargs)
    assert central['paired_corridor']['reason']=='joint_interval_too_wide'
    assert central['paired_corridor']['conditional_interval']['heading_half_width_rad']>math.radians(4)
    assert wider['paired_corridor']['status']=='conditional_paired_corridor'
    assert wider['paired_corridor']['conditional_interval']['heading_half_width_rad']<math.radians(4)
    assert wider['s1_initial_ground_motion']==central['s1_initial_ground_motion']
    assert wider['history_model_selection']==central['history_model_selection']
    assert wider['diagnostics']['front']==central['diagnostics']['front']
    for v in ('left','right'):
        assert len(wider['diagnostics'][v]['plane_region']['tracks'])>len(central['diagnostics'][v]['plane_region']['tracks'])
        for metrics in wider['paired_corridor']['metrics'][f'wall_{v}'].values():
            assert metrics['median_px']<=.35 and metrics['p90_px']<=.65 and metrics['inlier_fraction']>=.9
    assert not wider['motion_authorized'] and wider['arrival_status']=='unverified'


@pytest.mark.parametrize('side_roi,expected',[('central',(32,96,8,64)),('wide',(12,116,8,64))])
def test_only_sides_receive_fixed_bounds_once(monkeypatch,side_roi,expected):
    calls=[];track=support.track_fixed_roi
    def record(images,*,view,bounds):
        calls.append((view,bounds))
        return track(images,view=view,bounds=bounds)
    monkeypatch.setattr(support,'track_fixed_roi',record)
    frames=exposed_frames()
    support.prepare_straight_history(frames,now_s=frames[-1]['captured_sim_s'],calibration_id=CALIBRATION_ID,side_roi=side_roi)
    assert calls==[('left',expected),('right',expected),('front',(32,96,16,64))]


@pytest.mark.parametrize('value',['invalid','WIDE','',None,True,[],{},32])
def test_invalid_roi_rejects_before_image_work(monkeypatch,value):
    monkeypatch.setattr(support,'_ground_evidence',lambda *a:pytest.fail('unexpected image work'))
    with pytest.raises(ValueError,match='Unknown side ROI'):
        support.prepare_straight_history([],now_s=0.,calibration_id=CALIBRATION_ID,side_roi=value)


@pytest.mark.parametrize('failure',['source_spatial_or_temporal_rejection','joint_per_view_prediction_failed','joint_interval_too_wide'])
def test_failed_wide_fit_is_not_retried_or_replaced(monkeypatch,failure):
    calls=[];packet=dict(status='unknown',reason=failure)
    def probe(frames,**kwargs):
        calls.append(kwargs)
        return dict(paired_corridor=copy.deepcopy(packet))
    monkeypatch.setattr(wide,'probe_straight_history',probe)
    c=wide.WideReferenceStraightController()
    result=c._probe_navigation_history([],1.)
    assert result['paired_corridor']==packet and len(calls)==1
    assert calls[0]['side_roi']=='wide'
    assert not result['side_sampling']['alternative_fit_fallback']


def test_all_control_safety_and_history_methods_are_inherited():
    c=wide.WideReferenceStraightController()
    assert c.policy==StraightPolicy() and c.motion_permission=='no_motion'
    for name in ('_update','_wait','_confirm_stationary','_clock','_stopping_reserve',
                 '_following_guard','_following_command','_navigation_history','missing','cancel'):
        assert getattr(type(c),name) is getattr(StraightController,name)
    assert type(c)._assess_startup_texture is ReferenceStraightController._assess_startup_texture
    assert c.result()['experiment']==wide.experiment_contract()


@pytest.mark.parametrize('view',['left','right'])
def test_wide_heldout_inconsistency_still_rejects(view):
    from roadvision.joint_straight_model import estimate_joint_corridor
    from roadvision.rgb_motion_checks import split_tracks
    frames=exposed_frames();cal=GroundCalibration()
    data=support.prepare_straight_history(frames,now_s=frames[-1]['captured_sim_s'],
        calibration_id=CALIBRATION_ID,calibration=cal,side_roi='wide')
    sides={v:copy.deepcopy(data['diagnostics'][v]['plane_region']) for v in ('left','right')}
    _,heldout=split_tracks(sides[view]['tracks'])
    for i in heldout:sides[view]['tracks'][i]['last'][0]+=10.
    result=estimate_joint_corridor(sides,data['regions'],data['ground_motion'],cal)
    assert result['status']=='unknown' and result['reason']=='joint_per_view_prediction_failed'
    assert not result['motion_authorized']


@pytest.mark.parametrize('confirmed,permission,reason',[
    (False,'bounded_probe','initial_clearance_prior_not_confirmed'),
    (True,'no_motion','motion_not_authorized'),
])
def test_no_new_movement_authorization(confirmed,permission,reason):
    c=wide.WideReferenceStraightController(initial_clearance=InitialClearancePrior(confirmed=confirmed),motion_permission=permission)
    d=c.update(dict(frame_id='x',captured_sim_s=0.,rgb=exposed_frames()[0]['rgb']),0.)
    assert d['reason']==reason and d['v_m_s']==d['omega_rad_s']==0.
    assert not d['experiment']['promoted']


def test_wide_public_replay_binds_variant_and_preserves_terminal_stop(tmp_path):
    from simulation.continuous_straight_run import run_case
    from simulation.continuous_straight_acceptance import replay_safety_public
    path=tmp_path/'run'
    result=run_case(path,straight_variant='wide_reference',operator_confirmed=True,
                    motion_permission='bounded_probe',max_ticks=10,terminal_check_ticks=3)
    rows=[json.loads(x) for x in (path/'decisions.jsonl').read_text().splitlines()]
    assert any(r['decision'].get('perception',{}).get('side_sampling') for r in rows if r['decision'].get('perception'))
    assert result['audit']['safe_final_stop']
    assert replay_safety_public(path,result,rows)['terminal_frames']==3
    task_path=path/'public/task.json';task=json.loads(task_path.read_text());task['straight_variant']='reference'
    task_path.write_text(json.dumps(task))
    with pytest.raises(ValueError,match='differs'):replay_safety_public(path,result,rows)


@pytest.mark.parametrize('direction',['left','right'])
def test_wide_cannot_change_corner_control(tmp_path,direction):
    from simulation.continuous_straight_run import run_case
    with pytest.raises(ValueError,match='Straight-only'):
        run_case(tmp_path/'invalid',straight_variant='wide_reference',corner_direction=direction)
    assert not list(tmp_path.iterdir())


def test_default_producer_hook_still_resolves_original_function(monkeypatch):
    import roadvision.continuous_straight as control
    calls=[];sentinel=object()
    def probe(frames,**kwargs):calls.append((frames,kwargs));return sentinel
    monkeypatch.setattr(control,'probe_straight_history',probe)
    c=StraightController()
    assert c._probe_navigation_history([],1.) is sentinel
    assert len(calls)==1 and 'side_roi' not in calls[0][1]


def test_wide_module_has_no_physics_or_scene_imports():
    tree=ast.parse(Path(wide.__file__).read_text())
    imports=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
    assert not any((n or '').startswith(('simulation','pybullet')) for n in imports)
