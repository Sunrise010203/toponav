"""Pure wire/safety tests run without ROS, DDS or physics dependencies."""
import copy
import json
from pathlib import Path
import sys

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/toponav_bridge'))
from toponav_bridge.protocol import MotorGate, command_from_decision, decode, validate_command


def command(seq=1,**changes):
    d=dict(v_m_s=.15,omega_rad_s=.0,state='moving',control_frame_id='f1',
           captured_sim_s=1.,decision_sim_s=1.,pending_feedback=None)
    c=command_from_decision('test-session',seq,d);c.update(changes);return c


def armed():
    gate=MotorGate('test-session',lease_s=.8)
    gate.arm('f1',1.,10.)
    return gate


def test_public_command_does_not_contain_world_or_controller_internals():
    c=command();validate_command(c)
    assert set(c)=={'version','session_id','seq','frame_id','captured_sim_s','decision_sim_s',
                    'v_m_s','omega_rad_s','state','segment_event'}
    assert decode(json.dumps(c))==c


def test_abort_does_not_retransmit_an_unconsumed_visual_segment_event():
    event=dict(event_id='test-session:cmd-0001:visual-exit',command_id='test-session:cmd-0001',
               status='visual_segment_complete',frame_id='f1',captured_sim_s=1.)
    decision=dict(v_m_s=0.,omega_rad_s=0.,state='aborted',control_frame_id='f1',
                  captured_sim_s=1.,decision_sim_s=1.,pending_feedback=event)
    c=command_from_decision('test-session',1,decision)
    assert c['segment_event'] is None and decision['pending_feedback']==event
    validate_command(c)
    g=armed();assert g.accept(c,1.,10.1)==(0.,0.) and g.reason=='remote_abort'


@pytest.mark.parametrize('value',['{"x":1,"x":2}','{"x":NaN}','[1,2]','not-json','{"x":1e999}'])
def test_wire_rejects_duplicate_keys_nonfinite_and_nonobjects(value):
    with pytest.raises(ValueError):decode(value)


@pytest.mark.parametrize('change',[
    {'version':True},{'version':2},{'seq':True},{'seq':0},{'captured_sim_s':True},
    {'captured_sim_s':float('nan')},{'decision_sim_s':float('inf')},
    {'v_m_s':.151},{'v_m_s':-.1},{'omega_rad_s':.31},{'frame_id':''},
    {'state':'arrived'},{'state':'visual_stop'},{'segment_event':{'true_node':'G'}},
    {'pose':[0,0]},{'session_id':None},
])
def test_invalid_command_cannot_reach_motors(change):
    g=armed();out=g.accept(command(**change),1.,10.1)
    assert g.latched and out==(0.,0.)


def test_valid_fresh_rgb_bound_command_and_deadline():
    g=armed();assert g.accept(command(),1.,10.1)==(.15,0.)
    g.arm('f2',1.1,10.2)
    assert g.poll(10.89)==(.15,0.)
    assert g.poll(10.91)==(0.,0.) and g.reason=='command_lease_expired'


@pytest.mark.parametrize('change',[
    {'session_id':'different'},{'seq':2},{'frame_id':'old'},
    {'captured_sim_s':.9},{'decision_sim_s':1.1},
])
def test_session_sequence_frame_and_clock_bindings_are_enforced(change):
    g=armed();assert g.accept(command(**change),1.,10.1)==(0.,0.)
    assert g.latched


def test_replayed_valid_command_latches_zero_and_later_good_input_cannot_restart():
    g=armed();c=command();g.accept(c,1.,10.1)
    assert g.accept(c,1.,10.2)==(0.,0.) and g.latched
    reason=g.reason;g.arm('f2',1.1,10.3)
    assert g.accept(command(seq=2,frame_id='f2',captured_sim_s=1.1,decision_sim_s=1.1),1.1,10.4)==(0.,0.)
    assert g.reason==reason


def test_remote_zero_abort_can_stop_without_a_new_rgb_but_cannot_authorise_motion():
    g=armed();g.accept(command(),1.,10.1);g.arm('f2',1.1,10.2)
    c=command(seq=2,state='aborted',v_m_s=0.)
    assert g.accept(c,1.1,10.3)==(0.,0.) and g.reason=='remote_abort'


def test_completion_requires_current_rgb_and_zero_command():
    g=armed();assert g.accept(command(state='visual_stop',v_m_s=0.),1.,10.1)==(0.,0.)
    assert g.latched and g.reason=='remote_visual_stop'


@pytest.mark.parametrize('now,wall',[(.9,10.1),(1.4,10.1),(1.,9.9)])
def test_clock_regression_stale_frame_and_steady_clock_regression_fail_closed(now,wall):
    g=armed();assert g.accept(command(),now,wall)==(0.,0.) and g.latched


def test_disarmed_gate_never_moves_and_invalid_leases_rejected():
    g=MotorGate('test-session');assert g.poll(1.)==(0.,0.)
    assert g.accept(command(),1.,2.)==(0.,0.) and g.latched
    for lease in (True,0.,float('nan'),5.):
        with pytest.raises(ValueError):MotorGate('test-session',lease_s=lease)


def test_s7_suite_validation_is_dry(tmp_path):
    from toponav_bridge.runner import load_suite
    s=json.loads((ROOT/'reports/ros2-integration-20261018/suite.json').read_text())
    p=tmp_path/'suite.json';p.write_text(json.dumps(s))
    assert len(load_suite(p)['cases'])==8 and list(tmp_path.iterdir())==[p]


@pytest.mark.parametrize('mutation',['path','seed','count','fault_coverage','normal_refusal','budget','tick','graph_truth','one_edge','same_layout','task_conflict','bool_heading'])
def test_s7_suite_cannot_silently_weaken_setup_or_denominators(tmp_path,mutation):
    from toponav_bridge.runner import load_suite
    s=json.loads((ROOT/'reports/ros2-integration-20261018/suite.json').read_text());c=s['cases'][0]
    if mutation=='path':c['id']='../escape'
    elif mutation=='seed':c['seed']=s['cases'][1]['seed']
    elif mutation=='count':s['cases'].pop()
    elif mutation=='fault_coverage':s['cases'][-1]['fault']='camera_silence'
    elif mutation=='normal_refusal':c['fault']='feedback_silence'
    elif mutation=='budget':c['max_ticks']=10
    elif mutation=='tick':s['cases'][2]['fault_tick']=0
    elif mutation=='graph_truth':c['graph']['geometry']={'position':[0,0]}
    elif mutation=='one_edge':c['goal_node']='B'
    elif mutation=='same_layout':s['cases'][1]['physical_scene']=copy.deepcopy(c['physical_scene']);s['cases'][1]['graph']=copy.deepcopy(c['graph'])
    elif mutation=='task_conflict':c['graph']['task']['start_node']='T'
    else:c['heading_deg']=True
    p=tmp_path/'suite.json';p.write_text(json.dumps(s))
    with pytest.raises(ValueError):load_suite(p)


def public_fixture(tmp_path):
    import hashlib
    import numpy as np
    from PIL import Image
    from roadvision.corridor_rgb_probe import CALIBRATION_ID
    rgb=np.zeros((256,256,3),np.uint8);Image.fromarray(rgb).save(tmp_path/'ref.png')
    task=dict(version=1,session_id='test-session',start_node='A',goal_node='G',heading_deg=90,operator_confirmed=True,calibration_id=CALIBRATION_ID)
    ref=dict(node_id='A',heading_deg=90,rgb_path='ref.png',rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest())
    for name,data in [('task',task),('topology',{'schema_version':'2.0'}),('references',[ref])]:
        (tmp_path/(name+'.json')).write_text(json.dumps(data))
    return task,ref


@pytest.mark.parametrize('hazard',['traversal','absolute','hash','hidden_field','task_truth','calibration','bool_version','bool_prior'])
def test_public_priors_cannot_escape_pixel_or_metadata_boundary(tmp_path,hazard):
    from toponav_bridge.protocol import load_public
    task,ref=public_fixture(tmp_path)
    if hazard=='traversal':ref['rgb_path']='../ref.png'
    elif hazard=='absolute':ref['rgb_path']=str(tmp_path/'ref.png')
    elif hazard=='hash':ref['rgb_sha256']='0'*64
    elif hazard=='hidden_field':ref['position_m']=[0,0]
    elif hazard=='task_truth':task['true_node']='A'
    elif hazard=='calibration':task['calibration_id']='different'
    elif hazard=='bool_version':task['version']=True
    else:task['operator_confirmed']=1
    (tmp_path/'task.json').write_text(json.dumps(task));(tmp_path/'references.json').write_text(json.dumps([ref]))
    with pytest.raises(ValueError):load_public(tmp_path)


def test_launch_keeps_physical_scenario_out_of_navigation_arguments():
    import ast
    tree=ast.parse((ROOT/'ros2_ws/src/toponav_bridge/launch/continuous_route.launch.py').read_text())
    assignments={n.targets[0].id:ast.unparse(n.value) for n in ast.walk(tree) if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name)}
    assert '--scenario' in assignments['world'] and '--scenario' not in assignments['nav']
    assert 'use_sim_time' in assignments['nav']
    module=ast.parse((ROOT/'ros2_ws/src/toponav_bridge/toponav_bridge/nodes.py').read_text())
    nav=next(n for n in module.body if isinstance(n,ast.ClassDef) and n.name=='Navigation')
    text=ast.unparse(nav)
    for forbidden in ('RouteAuditWorld','physical_scene','audit_state','getBasePosition','physics_trace'):
        assert forbidden not in text
    assert 'ClockType.STEADY_TIME' in text


def replay_fixture(tmp_path):
    """Real pure controller on flat RGB; NO physics, DDS or acceptance claim."""
    import hashlib
    import numpy as np
    from PIL import Image
    from roadvision.continuous_straight import ContinuousRouteController,InitialClearancePrior
    from simulation.path_planning import plan_from_estimate,next_action
    public=tmp_path/'public';public.mkdir();task,ref=public_fixture(public)
    graph=json.loads((ROOT/'reports/ros2-integration-20261018/suite.json').read_text())['cases'][0]['graph']
    (public/'topology.json').write_text(json.dumps(graph))
    inventory=[dict(ref,node_id=n['node_id'],heading_deg=h) for n in graph['nodes'] for h in (0,90,180,270)]
    (public/'references.json').write_text(json.dumps(inventory))
    rgb=np.zeros((256,256,3),np.uint8)
    c=ContinuousRouteController(graph=graph,references=[dict(node_id=r['node_id'],heading_deg=r['heading_deg'],rgb=rgb) for r in inventory],
        plan_from_estimate=plan_from_estimate,next_action=next_action,start='A',goal='G',heading=90,session_id='test-session',initial_clearance=InitialClearancePrior(confirmed=True))
    nav=tmp_path/'navigation';(nav/'frames').mkdir(parents=True);events=[]
    for i in range(4):
        t=1.+i*.1;f=dict(frame_id=f'f{i}',captured_sim_s=t,rgb=rgb)
        d=c.update(f,t);Image.fromarray(rgb).save(nav/f'frames/{i}.png')
        events.append(dict(kind='image',now_sim_s=t,terminal=i>0,result_unchanged=True if i else None,
            input=dict(frame_id=f['frame_id'],captured_sim_s=t,rgb_path=f'frames/{i}.png',rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest()),
            decision=d,command=command_from_decision('test-session',i+1,d)))
    (nav/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
    (nav/'result.json').write_text(json.dumps(dict(controller_result=c.result(),bridge_reason=None,commands=4,images=4)))
    return events


def test_ros_public_replay_runs_with_no_world_fixture_or_audit_directory(tmp_path):
    from toponav_bridge.runner import replay_navigation
    replay_fixture(tmp_path);summary,rows=replay_navigation(tmp_path)
    assert summary['terminal_frames']==3 and len(rows)==4
    assert set(p.name for p in tmp_path.iterdir())=={'public','navigation'}


@pytest.mark.parametrize('hazard',['pixels','command','seq','terminal','missing_terminal','path','result','false_feedback_session','bool_feedback_version','bool_feedback_seq'])
def test_ros_exact_public_replay_rejects_tampered_evidence(tmp_path,hazard):
    from toponav_bridge.runner import replay_navigation
    events=replay_fixture(tmp_path);nav=tmp_path/'navigation'
    if hazard=='pixels':
        from PIL import Image
        import numpy as np
        Image.fromarray(np.full((256,256,3),255,np.uint8)).save(nav/'frames/0.png')
    elif hazard=='command':events[0]['command']['v_m_s']=.1
    elif hazard=='seq':events[1]['command']['seq']=1
    elif hazard=='terminal':events[1]['result_unchanged']=False
    elif hazard=='missing_terminal':events.pop()
    elif hazard=='path':events[0]['input']['rgb_path']='../../outside.png'
    elif hazard=='result':
        r=json.loads((nav/'result.json').read_text());r['controller_result']['arrival_status']='arrived';(nav/'result.json').write_text(json.dumps(r))
    else:
        f=dict(version=True if hazard=='bool_feedback_version' else 1,session_id='foreign' if hazard=='false_feedback_session' else 'test-session',
               applied_seq=True if hazard=='bool_feedback_seq' else 1,applied_sim_s=1.,event={})
        events.insert(1,dict(kind='feedback',now_sim_s=1.,feedback=f,decision=events[0]['decision']))
    (nav/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
    with pytest.raises(ValueError):replay_navigation(tmp_path)
