"""Mocked geometry tests state boundaries, never real-RGB clearance acceptance."""
import json
import pytest
from roadvision import continuous_straight as base
from roadvision import rear_straight as guarded


def controller(monkeypatch,rear_status='conditional_rear_consistent',sweep_status='unknown',active=True):
    from test_straight_task2_permissions import setup
    c,_=setup(monkeypatch)
    r=guarded.RearStraightController(initial_clearance=base.InitialClearancePrior(confirmed=True),
        motion_permission='bounded_probe',active_review=active)
    r._local_stationary_motion=c._local_stationary_motion
    def rear(frames,packet,**kw):return dict(status=rear_status,
        geometry_reference_frame_id=frames[-1]['frame_id'],geometry_time_s=frames[-1]['captured_sim_s'])
    def sweep(frames,packet,v,w,**kw):return dict(status=sweep_status,requested_v_m_s=v,requested_omega_rad_s=w,
        geometry_reference_frame_id=frames[-1]['frame_id'],geometry_time_s=frames[-1]['captured_sim_s'])
    monkeypatch.setattr(guarded,'assess_rear_consistency',rear)
    monkeypatch.setattr(guarded,'assess_swept_ground',sweep)
    return r


@pytest.mark.parametrize('status',['unknown','conflict'])
def test_rear_rejection_cannot_be_voted_away_or_reset_wait(monkeypatch,status):
    from test_straight_task2_permissions import frame
    c=controller(monkeypatch,rear_status=status,sweep_status='conditional_patch_supported_sweep')
    for i in range(9):out=c.update(frame(i),i*.1)
    assert out['state']=='waiting_evidence' and out['v_m_s']==0
    start=c.unverified_started
    for i in range(9,20):
        out=c.update(frame(i),i*.1)
        assert out['v_m_s']==out['omega_rad_s']==0 and c.unverified_started==start
    assert out['state']=='aborted'
    assert out['rear_observation']['active_review']['status']=='ineligible'
    assert not out['rear_observation']['active_executed']
    assert c.update({},float('nan'))==out


def test_positive_rear_without_full_sweep_remains_zero(monkeypatch):
    from test_straight_task2_permissions import frame
    c=controller(monkeypatch)
    for i in range(9):out=c.update(frame(i),i*.1)
    assert out['state']=='waiting_evidence' and out['v_m_s']==0
    assert 'current_requested_swept_coverage' in out['rear_observation']['active_review']['failed_checks']


def test_all_extra_guards_pass_preserves_original_command_not_active_proposal(monkeypatch):
    from test_straight_task2_permissions import frame
    c=controller(monkeypatch,sweep_status='conditional_patch_supported_sweep')
    for i in range(9):out=c.update(frame(i),i*.1)
    assert out['state']=='following' and out['v_m_s']==.15
    assert not out['rear_observation']['active_executed']
    assert out['rear_observation']['active_review'] is None
    assert out['arrival_status']=='unverified'


@pytest.mark.parametrize('which,field,value',[
    ('rear','geometry_reference_frame_id','old'),('rear','geometry_time_s',0.),
    ('sweep','geometry_reference_frame_id','old'),('sweep','geometry_time_s',0.),
    ('sweep','requested_v_m_s',.08),('sweep','requested_omega_rad_s',.2)])
def test_wrong_binding_rejects(monkeypatch,which,field,value):
    from test_straight_task2_permissions import frame
    c=controller(monkeypatch,sweep_status='conditional_patch_supported_sweep')
    name='assess_rear_consistency' if which=='rear' else 'assess_swept_ground'
    fn=getattr(guarded,name)
    def wrong(*args,**kw):
        out=fn(*args,**kw);out[field]=value;return out
    monkeypatch.setattr(guarded,name,wrong)
    for i in range(9):out=c.update(frame(i),i*.1)
    assert out['state']=='waiting_evidence' and out['v_m_s']==0


@pytest.mark.parametrize('confirmed,permission',[(True,'no_motion'),(False,'bounded_probe')])
def test_never_bypass_explicit_startup(monkeypatch,confirmed,permission):
    from test_straight_task2_permissions import frame
    c=controller(monkeypatch,sweep_status='conditional_patch_supported_sweep')
    c.initial_clearance=base.InitialClearancePrior(confirmed=confirmed);c.motion_permission=permission
    out=c.update(frame(0),0.)
    assert out['state']=='aborted' and out['motion_commands']==0
    assert out['rear_observation']['rear'] is None


@pytest.mark.parametrize('variant',['rear','rear_active'])
def test_public_contract_exact_replay_and_mutation_rejection(tmp_path,variant):
    from simulation.continuous_straight_run import run_case
    from simulation.continuous_straight_acceptance import replay_safety_public
    p=tmp_path/variant;r=run_case(p,straight_variant=variant,operator_confirmed=True,max_ticks=3,terminal_check_ticks=3)
    rows=[json.loads(l) for l in (p/'decisions.jsonl').read_text().splitlines()]
    assert replay_safety_public(p,r,rows)['terminal_frames']==3
    path=p/'public/task.json';task=json.loads(path.read_text());task['rear_contract']['active_scope']='actuate';path.write_text(json.dumps(task))
    with pytest.raises(ValueError,match='differs'):replay_safety_public(p,r,rows)


@pytest.mark.parametrize('variant',['rear','rear_active'])
def test_variants_forbidden_in_corner_or_routes(tmp_path,variant):
    from simulation.continuous_straight_run import run_case
    with pytest.raises(ValueError,match='route/corner'):run_case(tmp_path/variant,corner_direction='left',straight_variant=variant)


@pytest.mark.parametrize('name',[
    'test_ambiguous_preflight_cannot_use_operator_clearance_to_probe',
    'test_complete_geometry_rejection_ends_probe_immediately',
    'test_return_of_geometry_does_not_interrupt_braking',
    'test_rearm_requires_post_zero_stationarity_and_consecutive_good_evidence',
    'test_intermittent_geometry_cannot_extend_wait_budget',
    'test_budget_reserves_next_command_interval_and_braking',
    'test_recovery_count_is_not_reset_by_transient_success',
    'test_a_larger_latency_assumption_requires_more_prior_clearance',
    'test_stale_fit_breaks_the_consecutive_rearm_sequence',
    'test_fast_packets_cannot_shorten_stationary_confirmation_time'])
def test_existing_safety_contracts_with_extra_guards_passing(monkeypatch,name):
    import test_straight_task2_permissions as tests
    monkeypatch.setattr(guarded.RearStraightController,'_following_guard',lambda *args:True)
    # These are state-machine tests; expensive RGB diagnostics are irrelevant.
    monkeypatch.setattr(guarded.RearStraightController,'_rear',lambda *args:dict(status='unknown'))
    monkeypatch.setattr(base,'StraightController',lambda **kw:guarded.RearStraightController(**kw))
    getattr(tests,name)(monkeypatch)


def test_back_only_freeze_is_declared_rgb_fault_not_hidden_truth(tmp_path):
    import numpy as np
    from PIL import Image
    from simulation.continuous_straight_run import run_case
    from simulation.continuous_straight_acceptance import replay_safety_public
    from simulation.corridor_motion import CorridorConfig
    p=tmp_path/'freeze';r=run_case(p,scene=CorridorConfig(length_m=4.),seed=31013,operator_confirmed=True,motion_permission='bounded_probe',
        max_ticks=9,terminal_check_ticks=3,fault=dict(kind='back_freeze',tick=5))
    rows=[json.loads(l) for l in (p/'decisions.jsonl').read_text().splitlines()]
    faults=[row for row in rows if row.get('injected_fault')=='back_freeze']
    assert faults
    a=np.array(Image.open(p/'public'/faults[0]['input']['rgb_path']))
    b=np.array(Image.open(p/'public'/faults[-1]['input']['rgb_path']))
    assert np.array_equal(a[128:,128:],b[128:,128:])
    assert replay_safety_public(p,r,rows)['terminal_frames']==3
