"""Task-1 diagnostics; permissive characterizations tightened by task 2.

The exact task-1 tests are preserved in the task-2 before snapshot.
"""
import copy
import json

import numpy as np
import pytest

from tools.diagnose_straight_task1 import compare_history, evidence, summarize
from roadvision import continuous_straight as control
from ros2_ws.src.toponav_bridge.toponav_bridge.protocol import MotorGate, command_from_decision


def row(stamp=1., state='waiting_evidence', perception=None, free=None):
    return dict(tick=10, sim_s=stamp, decision=dict(
        state=state, reason='geometry_or_free_path_unverified',
        captured_sim_s=stamp, control_frame_id='frame', v_m_s=0., omega_rad_s=0.,
        perception=perception, free_path=free))


def packet(stamp=1., ground='conditional_ground_motion', corridor='unknown'):
    return dict(geometry_time_s=stamp, ground_motion=dict(status=ground, reason='ground_reason'),
                corridor_geometry=dict(status=corridor, reason='corridor_reason'),
                paired_corridor=dict(status='unknown', reason='paired_reason'))


def test_multiple_simultaneous_blockers_are_not_misreported_as_one_root_cause():
    r=row(perception=packet(ground='unknown'), free=dict(geometry_time_s=1.,status='unknown',reason='cell_failed'))
    assert evidence(r)['blockers']==['ground_motion_unavailable','corridor_geometry_unavailable','free_ground_unavailable']


def test_carried_diagnostics_are_not_counted_as_new_fits():
    r=row(stamp=1.1,perception=packet(),free=dict(geometry_time_s=1.,status='unknown'))
    out=evidence(r)
    assert not out['perception_current'] and not out['free_check_current']
    assert out['blockers']==[]
    summary,_=summarize([r])
    assert summary['fresh_perception_count']==0 and summary['blocker_counts']=={}


@pytest.mark.parametrize('fault',['missing','replay','stale','occlusion'])
def test_early_fault_retains_old_capture_stamp_but_is_not_a_new_fit(fault):
    r=row(perception=packet(),free=dict(geometry_time_s=1.,status='unknown'))
    r.update(sim_s=1.1,injected_fault=fault)
    # Both decision metadata and cached evidence refer to time 1.0; the
    # fault happened at tick time 1.1, without a newly computed geometry fit.
    r['decision']['state']='aborted'
    result=evidence(r)
    assert not result['perception_current'] and not result['free_check_current']
    assert result['blockers']==[]


def test_current_corridor_can_fail_independently_of_current_free_path():
    out=evidence(row(perception=packet(),free=dict(geometry_time_s=1.,status='conditional_free_ground_path',cells=[])))
    assert out['blockers']==['corridor_geometry_unavailable']


def test_current_free_path_can_fail_independently_of_current_corridor():
    out=evidence(row(perception=packet(corridor='conditional_corridor_geometry'),
                     free=dict(geometry_time_s=1.,status='unknown',reason='unverified',
                               cells=[dict(accepted=False,reason='ambiguous',reference_accepted=False)])))
    assert out['blockers']==['free_ground_unavailable']
    assert out['rejected_cells'][0]['reason']=='ambiguous'


def test_startup_motion_is_not_labelled_geometry_supported_following():
    r=row(state='bootstrapping');r['decision']['v_m_s']=.15
    assert evidence(r)['authorization']=='startup_probe'


def test_first_motion_and_first_wait_preserve_original_event():
    first=row(state='bootstrapping');first['tick']=2;first['decision']['v_m_s']=.15
    second=row(perception=packet())
    summary,records=summarize([first,second])
    assert summary['first_motion']['tick']==2
    assert summary['first_wait']==records[1]
    assert summary['authorization_counts']=={'startup_probe':1,'zero_command':1}


def comparison_fixture(tmp_path):
    old,new=tmp_path/'old',tmp_path/'new'
    for base in (old,new):
        (base/'public').mkdir(parents=True)
        (base/'public/sequence.json').write_text(json.dumps(dict(frames=[dict(frame_id='x',captured_sim_s=.5,rgb_sha256='abc')])) )
        (base/'result.json').write_text(json.dumps(dict(frame_count=1,decision_count=1,
            controller_result=dict(state='aborted'),audit=dict(displacement_m=.1),exception=None)))
    return old,new


def test_comparison_never_claims_deleted_intermediate_decisions_match(tmp_path):
    old,new=comparison_fixture(tmp_path)
    result=compare_history(old,new)
    assert result['passed'] and not result['historical_decisions_available']
    assert not result['historical_per_decision_equality_claimed']


@pytest.mark.parametrize('field,value',[('frame_id','other'),('captured_sim_s',.6),('rgb_sha256','different')])
def test_reproduction_must_match_all_retained_capture_fields(tmp_path,field,value):
    old,new=comparison_fixture(tmp_path)
    path=new/'public/sequence.json';data=json.loads(path.read_text())
    data['frames'][0][field]=value;path.write_text(json.dumps(data))
    assert not compare_history(old,new)['passed']


@pytest.mark.parametrize('field,value',[('controller_result',{'state':'visual_stop'}),('audit',{'displacement_m':.2}),('exception','bad')])
def test_reproduction_must_match_results_and_physics_without_exception(tmp_path,field,value):
    old,new=comparison_fixture(tmp_path)
    path=new/'result.json';data=json.loads(path.read_text());data[field]=value
    path.write_text(json.dumps(data))
    assert not compare_history(old,new)['passed']


def image_frame(i):
    pixels=np.random.default_rng(i).integers(0,255,(256,256,3),dtype=np.uint8)
    return dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb=pixels)


def stub_controller(monkeypatch,confirmed=True):
    monkeypatch.setattr(control.StraightController,'_local_stationary_motion',
                        lambda *_:dict(status='conditional_ground_motion',translation_m=[0.,0.],yaw_delta_rad=0.))
    perception=dict(corridor_geometry=dict(status='unknown'),front_surface=dict(status='unknown'),
                    ground_motion=dict(status='unknown'))
    free=dict(status='unknown')
    monkeypatch.setattr(control,'probe_straight_history',lambda *args,**kwargs:bind_evidence(args[0], copy.deepcopy(perception)))
    monkeypatch.setattr(control,'assess_free_ground',lambda *args,**kwargs:bind_evidence(args[0], copy.deepcopy(free)))
    c=control.StraightController(initial_clearance=control.InitialClearancePrior(confirmed=confirmed),motion_permission='bounded_probe')
    return c,perception,free


def test_task2_stops_probe_at_first_complete_geometry_rejection(monkeypatch):
    c,_,_=stub_controller(monkeypatch)
    for i in range(9):out=c.update(image_frame(i),i*.1)
    assert out['state']=='waiting_evidence' and out['v_m_s']==0. and c.motion_commands==6
    assert out['perception']['corridor_geometry']['status']=='unknown'
    assert out['free_path']['status']=='unknown'
    # Task 1 documented six further probe commands here; task 2 forbids them.


def test_missing_operator_prior_blocks_that_startup_path(monkeypatch):
    c,_,_=stub_controller(monkeypatch,confirmed=False)
    out=c.update(image_frame(0),0.)
    assert out['state']=='aborted' and out['reason']=='initial_clearance_prior_not_confirmed'
    assert out['v_m_s']==out['omega_rad_s']==0.


def test_task2_wait_cannot_resume_without_stop_confirmation(monkeypatch):
    c,perception,free=stub_controller(monkeypatch)
    perception['corridor_geometry']=dict(status='conditional_corridor_geometry',heading_error_rad=0.,offset_m=.1)
    free['status']='conditional_free_ground_path';free['conditional_stopping_clearance_m']=.6
    for i in range(9):out=c.update(image_frame(i),i*.1)
    assert out['state']=='following'
    free['status']='unknown'
    assert c.update(image_frame(9),.9)['state']=='waiting_evidence'
    free['status']='conditional_free_ground_path';free['conditional_stopping_clearance_m']=.6
    out=c.update(image_frame(10),1.)
    assert out['state']=='waiting_evidence' and out['v_m_s']==0. and c.stationary_frames==0


def test_terminal_abort_never_rearms_when_good_evidence_returns(monkeypatch):
    c,_,_=stub_controller(monkeypatch)
    c.cancel(0.)
    out=c.update(image_frame(1),.1)
    assert out['state']=='aborted' and out['v_m_s']==out['omega_rad_s']==0.


def wire_command():
    return command_from_decision('task1',1,dict(control_frame_id='f1',captured_sim_s=1.,
        decision_sim_s=1.,v_m_s=.15,omega_rad_s=0.,state='moving'))


def test_motor_gate_validates_protocol_not_rgb_or_free_space():
    gate=MotorGate('task1');gate.arm('f1',1.,10.)
    command=wire_command()
    assert 'perception' not in command and 'free_path' not in command
    assert gate.accept(command,1.,10.1)==(.15,0.)


def test_task2_motor_gate_also_expires_accepted_command_hold():
    gate=MotorGate('task1');gate.arm('f1',1.,10.)
    gate.accept(wire_command(),1.,10.1)
    assert gate.deadline is None and gate.hold_deadline==10.9
    assert gate.poll(20.)==(0.,0.) and gate.latched
    # Still requires the owner's timer to run; not hardware hang protection.


def test_motor_gate_frame_wait_expiry_is_latched():
    gate=MotorGate('task1');gate.arm('f1',1.,10.)
    assert gate.poll(11.)==(0.,0.) and gate.latched
    assert gate.accept(wire_command(),1.,11.1)==(0.,0.)


def bind_evidence(frames,packet):
    return {**packet,'geometry_reference_frame_id':frames[-1]['frame_id'],'geometry_time_s':frames[-1]['captured_sim_s']}


@pytest.fixture(autouse=True)
def synthetic_preflight(monkeypatch):
    # These state-machine tests stub geometry; actual RGB veto is tested separately.
    monkeypatch.setattr(control,'assess_startup_texture',lambda *_:dict(accepted=True,reason='unit_stub'))
