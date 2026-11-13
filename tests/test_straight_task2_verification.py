"""Post-run checker must reject all-refusal and braking-contract violations."""
import copy
import pytest
from tools.verify_straight_task2 import assess
from ros2_ws.src.toponav_bridge.toponav_bridge.protocol import MotorGate,command_from_decision


def fixture():
    result=dict(exception=None,controller_result=dict(state='aborted',reason='done'),
        safety_checks=dict(caught_input_errors=[],terminal_latched=True),
        audit=dict(safe_final_stop=True,wall_contact_physics_steps=0,maximum_audited_speed_m_s=.15,
            braking_episodes=[],bootstrap_max_displacement_m=.08,displacement_m=.1,normal_geometric_stop_pass=False))
    rows=[dict(decision=dict(state='following',v_m_s=.15,omega_rad_s=0.,permission=dict(stopping_reserve_m=.1598)))]
    return dict(id='case',expectation='progress_and_safe_stop'),result,rows


def test_safe_abort_is_not_full_navigation_success():
    case,result,rows=fixture();out=assess(case,result,rows,[])
    assert out['passed'] and not out['completed_geometric_task']


def test_all_refusal_cannot_pass_positive_condition():
    case,result,rows=fixture();rows[0]['decision'].update(state='aborted',v_m_s=0.)
    assert 'positive_case_did_not_follow' in assess(case,result,rows,[])['errors']


def test_negative_case_cannot_be_relabelled_as_an_authorized_probe():
    case,result,rows=fixture();case['expectation']='no_motion'
    d=rows[0]['decision'];d['state']='bootstrapping'
    d['permission'].update(mode='bounded_probe',probe_distance_bound_m=.135)
    assert 'negative_case_moved' in assess(case,result,rows,[])['errors']


@pytest.mark.parametrize('change,expected',[
    ({'resumed_before_stationary':True},'resumed_before_physical_stop'),
    ({'distance_m':.2},'brake_distance_outside_reserve'),
    ({'distance_m':None},'brake_distance_outside_reserve')])
def test_braking_failures_cannot_be_hidden_by_final_settling(change,expected):
    case,result,rows=fixture();episode=dict(resumed_before_stationary=False,distance_m=.01);episode.update(change)
    result['audit']['braking_episodes']=[episode]
    assert expected in assess(case,result,rows,[])['errors']


def test_observing_command_cannot_move_through_motor_gate():
    gate=MotorGate('test');gate.arm('f1',1.,10.)
    decision=dict(control_frame_id='f1',captured_sim_s=1.,decision_sim_s=1.,state='observing',v_m_s=.15,omega_rad_s=0.)
    assert gate.accept(command_from_decision('test',1,decision),1.,10.1)==(0.,0.) and gate.latched


def test_visual_stop_is_checked_at_decision_time_not_only_after_final_settling():
    case,result,rows=fixture();case['expectation']='front_visual_stop';result['controller_result']['state']='visual_stop'
    rows.append(dict(sim_s=1.,decision=dict(state='visual_stop',v_m_s=0.,omega_rad_s=0.,permission=dict(stopping_reserve_m=.1598))))
    trace=[dict(sim_s=1.,speed_m_s=.03,yaw_rate_rad_s=0.)]
    assert 'visual_stop_before_physical_stationary' in assess(case,result,rows,trace)['errors']
    trace[0]['speed_m_s']=0.
    assert assess(case,result,rows,trace)['passed']
