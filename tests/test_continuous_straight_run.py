import pytest
import ast
from pathlib import Path
from simulation.continuous_straight_run import run_case
from roadvision import continuous_straight,free_ground_path,straight_permission
from roadvision import paired_corridor,joint_straight_model,straight_evidence,straight_history_support


def test_unconfirmed_launch_is_a_real_physical_no_move_rejection(tmp_path):
    r=run_case(tmp_path/'case',max_ticks=5,operator_confirmed=False)
    assert r['controller_result']['result_type']=='rejected_no_motion'
    assert r['audit']['wall_contact_physics_steps']==0
    assert r['audit']['displacement_m']<.001
    assert not r['audit']['normal_geometric_stop_pass']


def test_persistent_freeze_uses_new_metadata_held_pixels_and_real_terminal_frames(tmp_path):
    import json,hashlib
    import numpy as np
    from PIL import Image
    base=tmp_path/'freeze'
    from simulation.corridor_motion import CorridorConfig
    # Exposed positive fixture: the original seed313/6m now fails preflight
    # and is explicitly retained as a conservative false rejection below.
    r=run_case(base,scene=CorridorConfig(length_m=4.),seed=31013,max_ticks=20,operator_confirmed=True,motion_permission='bounded_probe',fault={'kind':'freeze','tick':3},terminal_check_ticks=3)
    rows=[json.loads(s) for s in (base/'decisions.jsonl').read_text().splitlines()]
    frozen=[row for row in rows if row['injected_fault']=='freeze']
    assert len(frozen)==3
    assert len({row['input']['frame_id'] for row in frozen})==3
    assert len({row['input']['captured_sim_s'] for row in frozen})==3
    assert len({row['input']['rgb_sha256'] for row in frozen})==1
    for row in frozen:
        rgb=np.array(Image.open(base/'public'/row['input']['rgb_path']))
        assert hashlib.sha256(rgb.tobytes()).hexdigest()==row['input']['rgb_sha256']
    assert r['controller_result']['reason']=='commanded_motion_without_rgb_response'
    assert r['audit']['safe_final_stop'] and r['audit']['wall_contact_physics_steps']==0
    assert r['audit']['displacement_m']<.06
    assert r['safety_checks']['terminal_latched'] and r['safety_checks']['terminal_good_frame_checks']==3


def test_freeze_requires_a_real_preceding_payload(tmp_path):
    with pytest.raises(ValueError):run_case(tmp_path/'bad',fault={'kind':'freeze','tick':0})
    assert not list(tmp_path.iterdir())


def test_decision_modules_import_no_simulator_or_truth_readers():
    for module in (continuous_straight,free_ground_path,straight_permission,paired_corridor,joint_straight_model,straight_evidence,straight_history_support):
        source=Path(module.__file__).read_text();tree=ast.parse(source)
        names=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
        assert not any(n and (n.startswith('simulation') or n.startswith('pybullet')) for n in names)
        assert 'audit_state(' not in source and 'getBasePosition' not in source


# S3 extends the source runner; no monkey-patching of old world/controller.
def test_corner_fixture_is_l_shaped_and_public_input_remains_rgb_only(tmp_path):
    import pybullet as p
    from simulation.continuous_audit_world import CornerAuditWorld
    for direction,sign in [('left',1),('right',-1)]:
        with CornerAuditWorld(assets=tmp_path/direction,direction=direction) as w:
            f=w.capture();assert set(f)=={'frame_id','captured_sim_s','rgb'}
            assert f['rgb'].shape==(256,256,3)
            assert len(w.walls)==6
            def hit(a,b):return p.rayTest(a,b,physicsClientId=w.client)[0][0]
            assert hit((2.5,0.,.8),(2.5,sign*2.,.8))==-1
            assert hit((2.5,0.,.8),(2.5,-sign*2.,.8)) in w.walls
            assert hit((2.5,0.,.8),(4.,0.,.8)) in w.walls
            before=w.audit_state();w.drive(0.,sign*.15)
            for _ in range(10):w.step_control()
            w.drive(0.,0.)
            for _ in range(10):w.step_control()
            after=w.audit_state()
            assert sign*(after['yaw_deg']-before['yaw_deg'])>3.
            assert w.wall_contact_physics_steps==0
            assert after['speed_m_s']<.01 and abs(after['yaw_rate_rad_s'])<.02


def test_corner_without_confirmation_is_real_no_motion(tmp_path):
    r=run_case(tmp_path/'corner',corner_direction='left',operator_confirmed=False,max_ticks=5)
    assert r['version']=='continuous_corner_run_v1'
    assert r['controller_result']['result_type']=='rejected_no_motion'
    assert r['audit']['displacement_m']<.001 and r['audit']['wall_contact_physics_steps']==0
    assert not r['audit']['corner_geometric_exit_pass']


def test_corner_rejects_mixed_truth_setup_and_invalid_layout_arguments(tmp_path):
    import pytest
    from simulation.corridor_motion import CorridorConfig
    with pytest.raises(ValueError):run_case(tmp_path/'a',corner_direction='left',scene=CorridorConfig())
    with pytest.raises(ValueError):run_case(tmp_path/'b',layout_direction='right')
    with pytest.raises(ValueError):run_case(tmp_path/'c',corner_direction='left',corner_geometry={'true_pose':[0,0]})
    assert not list(tmp_path.iterdir())


def test_t_fixture_has_both_branches_closed_front_and_only_rgb_input(tmp_path):
    import pybullet as p
    from simulation.continuous_audit_world import CornerAuditWorld
    with CornerAuditWorld(assets=tmp_path/'t',junction='t',blocked_branch='left') as w:
        assert set(w.capture())=={'frame_id','captured_sim_s','rgb'}
        def hit(a,b):return p.rayTest(a,b,physicsClientId=w.client)[0][0]
        assert hit((2.5,0.,.8),(2.5,-2.,.8))==-1
        assert hit((2.5,0.,.8),(2.5,2.,.8))==w.branch_blocker
        assert hit((2.5,0.,.8),(4.,0.,.8)) in w.walls
        w.clear_branch_blocker()
        assert hit((2.5,0.,.8),(2.5,2.,.8))==-1
        # Slabs are adjacent with no overlapping area (no texture z-fighting).
        floor=p.getAABB(w.floor,physicsClientId=w.client)
        branch=p.getAABB(w.exit_floor,physicsClientId=w.client)
        assert abs(floor[1][0]-branch[0][0])<1e-9


def test_t_without_prior_is_real_no_motion(tmp_path):
    r=run_case(tmp_path/'t',junction='t',corner_direction='right',max_ticks=5)
    assert r['controller_result']['result_type']=='rejected_no_motion'
    assert r['audit']['displacement_m']<.001 and r['audit']['wall_contact_physics_steps']==0


def test_t_runner_rejects_incoherent_fixture_events_before_creating_output(tmp_path):
    import pytest
    for kwargs in [dict(junction='t'),dict(blocked_branch='left'),
                   dict(junction='t',corner_direction='left',layout_direction='right'),
                   dict(junction='t',corner_direction='left',clear_blocker_tick=3),
                   dict(junction='t',corner_direction='left',blocked_branch='left',clear_blocker_tick=True)]:
        with pytest.raises(ValueError):run_case(tmp_path/'bad',**kwargs)
    assert not list(tmp_path.iterdir())


def test_route_world_references_are_public_photos_not_pose_and_robot_never_reset(tmp_path):
    import json,inspect,ast
    import pybullet as p
    from simulation.continuous_audit_world import RouteAuditWorld
    from simulation.corridor_motion import CorridorWorld
    root=Path(__file__).resolve().parents[1]
    import runpy
    scene=runpy.run_path(str(root/'reports/multi-node-20261016/development/layouts.py'))['layout']()[1]
    with RouteAuditWorld(physical_scene=scene,assets=tmp_path/'assets',spawn_node='T',spawn_heading_deg=0,seed=9) as w:
        before=w.audit_state();steps=w.steps;robot=w.robot
        refs=w.reference_atlas()
        assert len(refs)==len(scene['nodes'])*4 and all(set(r)=={'node_id','heading_deg','rgb'} for r in refs)
        assert w.steps==steps and w.robot==robot and w.audit_state()==before
        assert set(w.capture())=={'frame_id','captured_sim_s','rgb'}
        assert p.getNumJoints(w.robot,physicsClientId=w.client)==4
        assert p.getDynamicsInfo(w.robot,-1,physicsClientId=w.client)[0]==1.
        w.drive(.10,0.)
        for _ in range(5):w.step_control()
        assert w.audit_state()['position_m'][0]>before['position_m'][0]+.03
        assert w.wall_contact_physics_steps==0
    # Every robot-construction statement but the initial transform is original.
    import textwrap
    a=ast.parse(textwrap.dedent(inspect.getsource(CorridorWorld._robot))).body[0]
    b=ast.parse(textwrap.dedent(inspect.getsource(RouteAuditWorld._robot))).body[0]
    b.body.pop(0) # pre-spawn scene construction, not a robot modification
    for f in (a,b):
        for n in ast.walk(f):
            if isinstance(n,ast.Call):n.keywords=[k for k in n.keywords if k.arg not in ('basePosition','baseOrientation')]
    assert ast.dump(a)==ast.dump(b)
    source=inspect.getsource(RouteAuditWorld)
    assert 'resetBase' not in source


def test_route_runner_rejects_mixed_scene_and_invalid_transport_before_output(tmp_path):
    import json,pytest
    root=Path(__file__).resolve().parents[1]
    c=json.loads((root/'reports/multi-node-20261016/suite.json').read_text())['cases'][0]
    for kwargs in [dict(corner_direction='left'),dict(transport_fault={'kind':'drop','segment':True}),dict(heading_deg=45)]:
        with pytest.raises(ValueError):run_case(tmp_path/'bad',route_graph=c['graph'],route_scene=c['physical_scene'],**kwargs)
    assert not list(tmp_path.iterdir())


def test_s6_future_and_reordered_packets_are_stopped_and_latched_physically(tmp_path):
    for kind in ('future','out_of_order'):
        r=run_case(tmp_path/kind,max_ticks=5,operator_confirmed=True,motion_permission='bounded_probe',
                   fault=dict(kind=kind,tick=1),terminal_check_ticks=3)
        assert r['controller_result']['state']=='aborted'
        assert r['audit']['safe_final_stop']
        assert r['safety_checks']['terminal_good_frame_checks']==3
        assert r['safety_checks']['terminal_latched']


def test_s6_bad_envelope_has_durable_zero_abort_not_a_missing_log(tmp_path):
    import json
    p=tmp_path/'malformed'
    r=run_case(p,operator_confirmed=True,motion_permission='bounded_probe',max_ticks=5,fault=dict(kind='bad_envelope',tick=1),terminal_check_ticks=3)
    rows=[json.loads(l) for l in (p/'decisions.jsonl').read_text().splitlines()]
    assert r['exception'] is None
    assert rows[-1]['injected_fault']=='bad_envelope' and rows[-1]['input_error']['type']=='ValueError'
    assert rows[-1]['decision']['state']=='aborted'
    assert rows[-1]['decision']['v_m_s']==rows[-1]['decision']['omega_rad_s']==0.
    assert r['safety_checks']['terminal_latched']
    assert r['safety_checks']['caught_input_errors'][0]['tick']==1


def test_s6_camera_silence_does_not_create_a_fake_capture(tmp_path,monkeypatch):
    from simulation.continuous_audit_world import ContinuousAuditWorld
    calls=[];original=ContinuousAuditWorld.capture
    def capture(self):calls.append(self.steps);return original(self)
    monkeypatch.setattr(ContinuousAuditWorld,'capture',capture)
    r=run_case(tmp_path/'silent',operator_confirmed=True,motion_permission='bounded_probe',max_ticks=5,fault=dict(kind='silence',tick=1),terminal_check_ticks=3)
    # tick0 + three terminal probes; no camera call during silence at tick1.
    assert len(calls)==4 and r['frame_count']==1
    assert r['controller_result']['reason']=='missing_frame' and r['audit']['safe_final_stop']


@pytest.mark.parametrize('value',[True,-1,11,1.2])
def test_s6_terminal_check_budget_rejected_before_output(tmp_path,value):
    import pytest
    with pytest.raises(ValueError):run_case(tmp_path/'bad',terminal_check_ticks=value)
    assert not list(tmp_path.iterdir())


def test_s6_public_replay_binds_good_terminal_frames_and_rejects_tampering(tmp_path):
    import json
    from simulation.continuous_straight_acceptance import replay_safety_public,verify_public_records,audit_safety_physics
    p=tmp_path/'case';result=run_case(p,max_ticks=2,terminal_check_ticks=3)
    rows=[json.loads(l) for l in (p/'decisions.jsonl').read_text().splitlines()]
    verify_public_records(p,rows)
    replay=replay_safety_public(p,result,rows)
    assert replay['terminal_frames']==3
    trace=json.loads((p/'audit/physics-trace.json').read_text())
    assert not audit_safety_physics(result,rows,trace)['errors']
    path=p/'public/terminal-checks.json';saved=json.loads(path.read_text());saved[0]['decision']['v_m_s']=.1;path.write_text(json.dumps(saved))
    with pytest.raises(ValueError,match='Terminal latching'):replay_safety_public(p,result,rows)


def test_s6_public_replay_handles_declared_malformed_envelope_exactly(tmp_path):
    import json
    from simulation.continuous_straight_acceptance import replay_safety_public
    p=tmp_path/'case';r=run_case(p,max_ticks=2,operator_confirmed=True,motion_permission='bounded_probe',fault=dict(kind='bad_envelope',tick=1),terminal_check_ticks=3)
    rows=[json.loads(l) for l in (p/'decisions.jsonl').read_text().splitlines()]
    assert replay_safety_public(p,r,rows)['decisions']==2
    rows[-1]['input_error']['message']='tampered'
    with pytest.raises(ValueError,match='exception trail'):replay_safety_public(p,r,rows)


def test_original_default_texture_is_conservatively_rejected_before_probe(tmp_path):
    # Retain this lost operating condition, rather than silently widening the
    # observability thresholds to make a movement-only test pass.
    r=run_case(tmp_path/'default',operator_confirmed=True,motion_permission='bounded_probe',max_ticks=5)
    assert r['controller_result']['reason']=='startup_texture_unverified'
    assert r['controller_result']['motion_commands']==0 and r['audit']['safe_final_stop']
