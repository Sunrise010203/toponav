"""Exposed regression RGB and synthetic selection contracts, not acceptance."""
from pathlib import Path
import hashlib
import json

import numpy as np
from PIL import Image
import pytest

from roadvision import reference_straight as permission
from roadvision.free_ground_path import reference_cell_candidates, select_reference_cells
from roadvision.calibrated_ground_motion import GroundCalibration


FIXTURE=Path(__file__).parent/'fixtures/straight_reference'


def exposed_rgb():
    rgb=np.array(Image.open(FIXTURE/'default_start.png'))
    manifest=json.loads((FIXTURE/'manifest.json').read_text())
    assert hashlib.sha256(rgb.tobytes()).hexdigest()==manifest['rgb_sha256']
    return rgb


def test_exposed_default_start_has_joint_support_without_relaxed_thresholds():
    out=permission.assess_joint_startup_texture(exposed_rgb(),GroundCalibration())
    assert out['accepted'] and out['reference_supported_cells']==9
    assert out['rectified_supported_cells']>=6
    assert all(out['checks'].values()) and not out['failed_checks']
    assert out['motion_authorized'] is out['certified_clearance'] is False
    assert out['selection_policy']=='joint_reference_native_rectified_v1'


def test_joint_reference_placements_are_fixed_bounded_and_repeatable():
    rgb=exposed_rgb();cal=GroundCalibration()
    a=permission.assess_joint_startup_texture(rgb,cal)
    assert a==permission.assess_joint_startup_texture(rgb.copy(),cal)
    for cell in a['cells']:
        delta=np.array(cell['point_latest_camera_xy_m'])-cell['nominal_point_latest_camera_xy_m']
        assert np.abs(delta).max()<=.010000001
        assert cell['reference_candidate_count']==9
        assert 'no target' in cell['selection_scope']


def test_free_path_selection_remains_native_only_and_exactly_legacy_ranked():
    gray=exposed_rgb()[:128,:128,0];mask=np.ones_like(gray);cal=GroundCalibration()
    groups=reference_cell_candidates(gray,mask,cal)
    selected=select_reference_cells(gray,mask,cal)
    for group,cell in zip(groups,selected):
        expected=max(group['candidates'],key=lambda c:(c['reference_accepted'],
            -c['reference_self_alternative_ncc'] if c['reference_self_alternative_ncc'] is not None else -2.))
        assert all(cell[k]==v for k,v in expected.items())
        assert cell['reference_candidate_count']==9
        assert cell['selection_scope']=='first warped RGB only; target not consulted'


@pytest.mark.parametrize('native,scores,failed',[
    ([True]*9,[.93]*9,[]),
    ([True]*9,[.93,.93,.94,.93,.94,.94,.94,.93,.93],['rectified_supported_cell_count']),
    ([False]+[True]*8,[.93]*9,['native_reference_cells_unique']),
    ([True]*9,[.93]*6+[.94]*3,['forward_bin_coverage']),
    ([True]*9,[.93,.93,.94]*3,['lateral_bin_coverage']),
    ([True]*9,[.94]*9,['rectified_supported_cell_count','forward_bin_coverage','lateral_bin_coverage']),
])
def test_all_original_native_count_and_spatial_thresholds_remain(monkeypatch,native,scores,failed):
    groups=[]
    for i,(x,y) in enumerate((x,y) for x in (.85,1.,1.2) for y in (-.22,0.,.22)):
        groups.append(dict(nominal_point_latest_camera_xy_m=[x,y],candidates=[dict(
            pixel_latest=[64.,100.],point_latest_camera_xy_m=[x,y],
            reference_accepted=native[i],reference_self_alternative_ncc=.2)]))
    monkeypatch.setattr(permission,'reference_cell_candidates',lambda *_:groups)
    monkeypatch.setattr(permission,'_supported_patch',lambda *_:np.ones((11,11),np.float32))
    values=iter(scores)
    monkeypatch.setattr(permission,'_masked_alternative',lambda *_:(next(values),1))
    out=permission.assess_joint_startup_texture(np.zeros((256,256,3),np.uint8),GroundCalibration())
    assert out['failed_checks']==failed
    assert out['accepted'] is (not failed)
    assert out['motion_authorized'] is out['certified_clearance'] is False


def test_native_validity_has_priority_over_rectified_success(monkeypatch):
    candidates=[dict(pixel_latest=[64.,100.],reference_accepted=True,reference_self_alternative_ncc=.3),
                dict(pixel_latest=[65.,100.],reference_accepted=False,reference_self_alternative_ncc=.99)]
    monkeypatch.setattr(permission,'reference_cell_candidates',lambda *_:[dict(
        nominal_point_latest_camera_xy_m=[.85,0.],candidates=candidates)])
    monkeypatch.setattr(permission,'_supported_patch',lambda *_:np.ones((11,11),np.float32))
    scores=iter([.95,.2])
    monkeypatch.setattr(permission,'_masked_alternative',lambda *_:(next(scores),1))
    out=permission.assess_joint_startup_texture(np.zeros((256,256,3),np.uint8),GroundCalibration())
    assert out['cells'][0]['pixel_latest']==[64.,100.]
    assert not out['accepted']


@pytest.mark.parametrize('name',['repeated','floor_paint'])
def test_known_ambiguous_ground_is_still_rejected(name):
    rgb=np.array(Image.open(FIXTURE.parent/'straight_startup'/f'{name}.png'))
    out=permission.assess_joint_startup_texture(rgb,GroundCalibration())
    assert not out['accepted'] and not out['motion_authorized']


def test_flat_image_is_not_rescued_by_candidate_search():
    out=permission.assess_joint_startup_texture(np.full((256,256,3),128,np.uint8),GroundCalibration())
    assert not out['accepted']
    assert out['reference_supported_cells']==out['rectified_supported_cells']==0


def test_old_baseline_still_rejects_exposed_default_without_joint_opt_in():
    from roadvision.straight_permission import assess_startup_texture
    result=assess_startup_texture(exposed_rgb(),GroundCalibration())
    assert not result['accepted'] and result['rectified_supported_cells']==5
    assert 'selection_policy' not in result


def test_only_startup_selection_is_overridden_not_safety_or_control():
    from roadvision.continuous_straight import StraightController,StraightPolicy
    c=permission.ReferenceStraightController()
    assert c.policy==StraightPolicy() and c.motion_permission=='no_motion'
    for name in ('_update','_wait','_confirm_stationary','_clock','_stopping_reserve',
                 '_following_guard','_following_command','_navigation_history','missing','cancel'):
        assert getattr(type(c),name) is getattr(StraightController,name)


@pytest.mark.parametrize('confirmed,mode,reason',[
    (False,'bounded_probe','initial_clearance_prior_not_confirmed'),
    (True,'no_motion','motion_not_authorized'),
])
def test_joint_reference_never_bypasses_operator_permission(confirmed,mode,reason):
    from roadvision.continuous_straight import InitialClearancePrior
    c=permission.ReferenceStraightController(initial_clearance=InitialClearancePrior(confirmed=confirmed),motion_permission=mode)
    d=c.update(dict(frame_id='reference-0',captured_sim_s=0.,rgb=exposed_rgb()),0.)
    assert d['reason']==reason and d['v_m_s']==d['omega_rad_s']==0.
    assert d['experiment']['promoted'] is False


def test_reference_variant_public_replay_and_tamper_rejection(tmp_path):
    from simulation.continuous_straight_run import run_case
    from simulation.continuous_straight_acceptance import replay_safety_public
    base=tmp_path/'run'
    result=run_case(base,straight_variant='reference',operator_confirmed=True,
                    motion_permission='bounded_probe',max_ticks=4,terminal_check_ticks=3)
    rows=[json.loads(line) for line in (base/'decisions.jsonl').read_text().splitlines()]
    assert result['controller_result']['motion_commands']>0
    assert result['audit']['safe_final_stop'] and result['safety_checks']['terminal_latched']
    assert replay_safety_public(base,result,rows)['terminal_frames']==3
    path=base/'public/task.json';task=json.loads(path.read_text())
    assert task['straight_variant']=='reference'
    task['straight_variant']='baseline';path.write_text(json.dumps(task))
    with pytest.raises(ValueError,match='differs'):replay_safety_public(base,result,rows)


@pytest.mark.parametrize('kwargs',[dict(corner_direction='left'),dict(straight_variant='invalid')])
def test_reference_variant_cannot_silently_change_corner_or_bad_mode(tmp_path,kwargs):
    from simulation.continuous_straight_run import run_case
    args=dict(straight_variant='reference');args.update(kwargs)
    with pytest.raises(ValueError):run_case(tmp_path/'invalid',**args)
    assert not list(tmp_path.iterdir())


def test_reference_producer_has_no_simulator_or_truth_dependency():
    import ast
    tree=ast.parse(Path(permission.__file__).read_text())
    for node in ast.walk(tree):
        if isinstance(node,ast.ImportFrom):
            assert not (node.module or '').startswith(('simulation','pybullet'))
        if isinstance(node,ast.Import):
            assert not any(name.name.startswith(('simulation','pybullet')) for name in node.names)
