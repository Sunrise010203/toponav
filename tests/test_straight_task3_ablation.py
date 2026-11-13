"""Task3 ablation contracts; synthetic fits do not certify RGB geometry."""
import numpy as np
import pytest
from roadvision import straight_ablation as a
from roadvision import continuous_straight as base


def frames(n=17,dt=.1):
    rgb=np.random.default_rng(14).integers(0,255,(256,256,3),dtype=np.uint8)
    return [dict(frame_id=f'f{i}',captured_sim_s=i*dt,rgb=rgb.copy()) for i in range(n)]


def metric(n,eligible=True):
    return dict(eligible=eligible,eligible_views=2 if eligible else 0,score=abs(n-13),scope='training_only')


def test_selects_one_window_without_consulting_final_fits(monkeypatch):
    calls=[]
    monkeypatch.setattr(a,'history_training_score',lambda f,*args:(calls.append(len(f)) or metric(len(f))))
    f=frames();selected,receipt=a.select_navigation_history(f,now_s=1.6,calibration=base.GroundCalibration())
    assert [len(selected),calls]==[13,[5,9,13,17]]
    assert receipt['selected_frame_ids']==[x['frame_id'] for x in f[-13:]]
    assert receipt['final_geometry_attempts']==1


def test_no_eligible_history_is_unknown_not_an_acceptance_fallback(monkeypatch):
    monkeypatch.setattr(a,'history_training_score',lambda f,*args:metric(len(f),False))
    selected,receipt=a.select_navigation_history(frames(),now_s=1.6,calibration=base.GroundCalibration())
    assert selected is None and receipt['selected_frame_ids']==[]


@pytest.mark.parametrize('bad',['truth','repeat','gap','stale'])
def test_every_buffer_packet_is_checked_before_selection(monkeypatch,bad):
    f=frames();calls=[]
    if bad=='truth':f[3]['pose']=[0,0]
    if bad=='repeat':f[3]['frame_id']=f[2]['frame_id']
    if bad=='gap':f[3]['captured_sim_s']=f[2]['captured_sim_s']
    now=2. if bad=='stale' else 1.6
    monkeypatch.setattr(a,'history_training_score',lambda *args:calls.append(1))
    with pytest.raises(ValueError):a.select_navigation_history(f,now_s=now,calibration=base.GroundCalibration())
    assert not calls


def test_over_horizon_windows_are_not_scored(monkeypatch):
    calls=[]
    monkeypatch.setattr(a,'history_training_score',lambda f,*args:(calls.append(len(f)) or metric(len(f))))
    f=frames(dt=.2)
    a.select_navigation_history(f,now_s=3.2,calibration=base.GroundCalibration())
    assert calls==[5,9]  # no fake compression of timestamps into1.6s


def test_rotation_does_not_count_as_translation_parallax():
    # Training points are physically consistent with a pure camera yaw.
    from roadvision.calibrated_ground_motion import ground_pixels
    c=base.GroundCalibration()
    points=np.array([[x,y] for x in (.55,.65,.75) for y in (-.2,-.1,0,.1)])
    first=ground_pixels(points,'front',c)
    from roadvision.calibrated_ground_motion import pixel_predictions
    last=pixel_predictions(first,'front',[0.,0.,.08],c)
    tracks=[dict(first=p.tolist(),middle=p.tolist(),last=q.tolist(),feature_split='train') for p,q in zip(first,last)]
    region=dict(tracks=tracks,coverage_map=np.eye(3).tolist(),status='accepted',models={'accepted':True})
    score=a.training_view_score(region,'front',c)
    assert score['translation_m']<1e-8 and not score['eligible']


@pytest.mark.parametrize('variant',['history','control','combined'])
def test_no_motion_permission_and_terminal_latch_are_not_bypassed(variant):
    c=a.AblationStraightController(variant=variant,initial_clearance=base.InitialClearancePrior(confirmed=True))
    f=frames(1)[0];out=c.update(f,0.)
    assert out['state']=='aborted' and out['reason']=='motion_not_authorized' and out['v_m_s']==0
    assert c.update({},float('nan'))['v_m_s']==0


@pytest.mark.parametrize('variant',['baseline','other','',None,True])
def test_only_explicit_experimental_variants_are_accepted(variant):
    with pytest.raises(ValueError):a.AblationStraightController(variant=variant)


def controller(variant='control'):
    c=a.AblationStraightController(variant=variant,initial_clearance=base.InitialClearancePrior(confirmed=True),motion_permission='bounded_probe')
    c.capture_dt=.1
    return c


def geometry(offset=0.,heading=0.,uncertainty=0.):
    return dict(heading_error_rad=heading,offset_m=offset,conditional_interval=dict(offset_half_width_m=uncertainty))


def test_center_band_does_not_chase_small_offset():
    c=controller();v,w=c._following_command(geometry(.03),dict(status='unknown'))
    assert v==.15 and w==0


def test_heading_priority_suppresses_lateral_turn_when_heading_is_large():
    left=controller();right=controller()
    _,a1=left._following_command(geometry(.16,.12),dict(status='unknown'))
    _,a2=right._following_command(geometry(-.16,.12),dict(status='unknown'))
    assert a1==a2 and a1<0


@pytest.mark.parametrize('offset',[-.2,.2])
def test_slew_and_omega_caps_never_exceed_task2_bounds(offset):
    c=controller();old=0.
    for _ in range(50):
        v,w=c._following_command(geometry(offset),dict(status='unknown'))
        assert abs(w-old)<=.0150001 and abs(w)<=.12 and 0<v<=.15
        c.last_command_omega=w;old=w


def test_immediate_stop_bypasses_smoothing_and_clears_filter():
    c=controller();c._following_command(geometry(.2),dict(status='unknown'))
    c.last_command_v=.15;c.last_command_omega=.08
    out=c.cancel(.1)
    assert out['v_m_s']==out['omega_rad_s']==0 and c.filtered_omega==0


def test_history_only_uses_identical_task2_control_formula():
    c=controller('history');b=base.StraightController()
    b.capture_dt=c.capture_dt=.1
    g=geometry(.16,.04);front=dict(status='unknown')
    assert c._following_command(g,front)==b._following_command(g,front)


def test_control_only_keeps_exact_nine_capture_history():
    c=controller('control');c.frames.extend(frames(9))
    selected=c._navigation_history(.8)
    assert len(selected)==9 and c.frames.maxlen==9


def test_selector_ignores_heldout_coordinates_and_acceptance_fields():
    from roadvision.calibrated_ground_motion import ground_pixels,pixel_predictions
    c=base.GroundCalibration()
    points=np.array([[x,y] for x in (.55,.65,.75) for y in (-.2,-.1,0,.1)])
    first=ground_pixels(points,'front',c);last=pixel_predictions(first,'front',[.08,0,0],c)
    train=[dict(first=p.tolist(),last=q.tolist(),feature_split='train') for p,q in zip(first,last)]
    validation=[dict(first=[40.,110.],last=[40.,110.],feature_split='validation')]
    region=dict(tracks=train+validation,coverage_map=[[1,0,0],[0,4,-384],[0,0,1]],status='accepted',models={'validation':'pass'})
    original=a.training_view_score(region,'front',c)
    region['tracks'][-1]['last']=[float('nan'),float('inf')]
    region['status']='unknown';region['models']={'validation':'fail'}
    assert a.training_view_score(region,'front',c)==original
    assert original['translation_m']==pytest.approx(.08)


@pytest.mark.parametrize('variant',['history','control','combined'])
def test_one_final_fit_and_no_retry_after_rejection(monkeypatch,variant):
    from test_straight_task2_permissions import setup,frame
    c,fits=setup(monkeypatch)
    exp=controller(variant)
    exp._local_stationary_motion=c._local_stationary_motion
    monkeypatch.setattr(a,'history_training_score',lambda f,*args:dict(metric(len(f)),score=abs(len(f)-9)))
    calls=[];original=base.probe_straight_history
    def probe(f,**kw):calls.append([x['frame_id'] for x in f]);return original(f,**kw)
    monkeypatch.setattr(base,'probe_straight_history',probe)
    fits['good']=False
    for i in range(9):out=exp.update(frame(i),i*.1)
    assert len(calls)==1 and out['state']=='waiting_evidence' and out['v_m_s']==0


SAFETY_TESTS=[
    'test_ambiguous_preflight_cannot_use_operator_clearance_to_probe',
    'test_complete_geometry_rejection_ends_probe_immediately',
    'test_return_of_geometry_does_not_interrupt_braking',
    'test_rearm_requires_post_zero_stationarity_and_consecutive_good_evidence',
    'test_intermittent_geometry_cannot_extend_wait_budget',
    'test_budget_reserves_next_command_interval_and_braking',
    'test_recovery_count_is_not_reset_by_transient_success',
    'test_a_larger_latency_assumption_requires_more_prior_clearance',
    'test_stale_fit_breaks_the_consecutive_rearm_sequence',
    'test_fast_packets_cannot_shorten_stationary_confirmation_time',
]


@pytest.mark.parametrize('variant',['history','control','combined'])
@pytest.mark.parametrize('name',SAFETY_TESTS)
def test_inherited_task2_safety_contracts(monkeypatch,variant,name):
    import test_straight_task2_permissions as contracts
    # Synthetic selector scores isolate safety-state-machine tests, NOT a
    # claim that RGB geometry passed. Actual-RGB comparisons live in reports.
    monkeypatch.setattr(a,'history_training_score',lambda f,*args:dict(metric(len(f)),score=abs(len(f)-9)))
    monkeypatch.setattr(base,'StraightController',lambda **kw:a.AblationStraightController(variant=variant,**kw))
    getattr(contracts,name)(monkeypatch)


@pytest.mark.parametrize('variant',['history','control','combined'])
def test_baseline_safety_policy_and_state_machine_are_inherited(variant):
    c=controller(variant)
    assert c.policy==base.StraightPolicy()
    for name in ('_update','_wait','_confirm_stationary','_clock','_stopping_reserve','missing','cancel'):
        assert getattr(type(c),name) is getattr(base.StraightController,name)


def test_experimental_public_task_binding_and_tamper(tmp_path):
    import json
    from simulation.continuous_straight_run import run_case
    from simulation.continuous_straight_acceptance import replay_safety_public
    r=run_case(tmp_path/'run',straight_variant='combined',operator_confirmed=True,terminal_check_ticks=3,max_ticks=3)
    rows=[json.loads(l) for l in (tmp_path/'run/decisions.jsonl').read_text().splitlines()]
    assert replay_safety_public(tmp_path/'run',r,rows)['terminal_frames']==3
    task=tmp_path/'run/public/task.json';t=json.loads(task.read_text());t['straight_variant']='control';task.write_text(json.dumps(t))
    with pytest.raises(ValueError,match='differs'):replay_safety_public(tmp_path/'run',r,rows)


@pytest.mark.parametrize('variant',['bad',None,True])
def test_runner_rejects_unknown_variant(tmp_path,variant):
    from simulation.continuous_straight_run import run_case
    with pytest.raises(ValueError):run_case(tmp_path/'invalid',straight_variant=variant)


def test_frozen_candidate_verifier_rejects_source_drift(tmp_path,monkeypatch):
    import hashlib
    from tools import run_straight_task3 as runner
    p=tmp_path/'source.py';p.write_text('frozen')
    monkeypatch.setattr(runner,'ROOT',tmp_path)
    suite=dict(candidate_files={'source.py':hashlib.sha256(p.read_bytes()).hexdigest()})
    runner.verify_candidate(suite);p.write_text('changed')
    with pytest.raises(ValueError,match='changed'):runner.verify_candidate(suite)


@pytest.mark.parametrize('uncertainty',[float('nan'),True,-.2,None])
def test_invalid_interval_cannot_expand_center_band(uncertainty):
    c=controller();c._following_command(geometry(.1,uncertainty=uncertainty),dict(status='unknown'))
    assert c.control_receipt['center_band_m']==.04
