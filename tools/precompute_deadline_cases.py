"""Read-only lexical source audit and fictional integer-ns schedules ONLY.

Never imports the audited modules, executes native code or reads public images.
Source-order observations are not a control-flow or runtime scheduling proof.
"""
import argparse
import ast
import copy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

from tools.offline_scope_index import read_index
from tools.offline_scope_review import BundleError, parse
from tools.precompute_deadline_contract import ContractError, FORMAT, LEASE_NS, MAX_NS, compare_schedule, check

SOURCE_FILES=('simulation/gazebo_cross_validation.py','ros2_ws/src/toponav_bridge/toponav_bridge/nodes.py',
    'ros2_ws/src/toponav_bridge/toponav_bridge/protocol.py','simulation/gazebo_actuator.py',
    'tools/frame_deadline_proposal.py','simulation/native/receiver_lease.hh')
CASES=('quick','first_slow_compute','exact_boundary','one_ns_before','one_ns_after','slow_serialization',
    'slow_capture','held_expires_first','held_exact_return','held_timely_replacement','held_after_pending',
    'held_already_expired','never_returns','owner_stopped','wrong_frame','wrong_sim_identity','wrong_view_digest',
    'stationary_sim_slow_host','six_views','completion_rebase_counterexample')
INVALID=('bool_host_stamp','float_host_stamp','negative_host_stamp','deadline_overflow','stages_reversed',
    'return_before_compute','review_before_return','extra_field','wrong_clock_domain','missing_view',
    'duplicate_view','result_without_return','missing_result_binding','bool_sim_identity')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def write(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')


def _method(tree,cls,name):
    body=tree.body
    if cls is not None:
        nodes=[n for n in body if isinstance(n,ast.ClassDef) and n.name==cls];check(len(nodes)==1,'source_class_anchor:'+cls);body=nodes[0].body
    nodes=[n for n in body if isinstance(n,ast.FunctionDef) and n.name==name]
    check(len(nodes)==1,'source_function_anchor:'+name);return nodes[0]


def _call(method,func):
    nodes=[n for n in ast.walk(method) if isinstance(n,ast.Call) and ast.unparse(n.func)==func]
    check(len(nodes)==1,'source_call_not_unique:'+func);return nodes[0].lineno


def _png(method,arg):
    nodes=[n for n in ast.walk(method) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
        and n.func.attr=='save' and isinstance(n.func.value,ast.Call) and ast.unparse(n.func.value.func)=='PNG.fromarray'
        and len(n.func.value.args)==1 and ast.unparse(n.func.value.args[0])==arg]
    check(len(nodes)==1,'source_png_anchor');return nodes[0].lineno


def audit_sources(root,pins):
    check(type(pins) is dict and set(pins)==set(SOURCE_FILES) and all(type(h) is str and len(h)==64
        and all(c in '0123456789abcdef' for c in h) for h in pins.values()),'source_six_pins_required')
    raw={n:read_index(Path(root)/n) for n in SOURCE_FILES}
    check(all(hashlib.sha256(raw[n]).hexdigest()==pins[n] for n in SOURCE_FILES),'source_pin_mismatch')
    trees={n:ast.parse(raw[n]) for n in SOURCE_FILES if n.endswith('.py')};claims=[]
    def order(code,path,anchors):
        lines=[v for _,v in anchors];check(lines==sorted(lines) and len(set(lines))==len(lines),'source_order_changed:'+code)
        claims.append(dict(claim=code,path=path,anchors=[dict(anchor=n,line=v) for n,v in anchors],scope='lexical AST order only, not runtime/control-flow proof'))
    g=_method(trees[SOURCE_FILES[0]],None,'run')
    loops=[n for n in ast.walk(g) if isinstance(n,ast.For) and isinstance(n.target,ast.Name) and n.target.id=='tick'
           and any(isinstance(c,ast.Call) and ast.unparse(c.func)=='controller.update' for c in ast.walk(n))]
    check(len(loops)==1,'source_tick_loop_anchor');g=loops[0]
    order('gazebo_work_before_pending',SOURCE_FILES[0],[('frame_return',_call(g,'io.next_frame')),('public_png',_png(g,'rgb')),
        ('controller_update',_call(g,'controller.update')),('local_arm',_call(g,'gate.arm')),('local_accept',_call(g,'gate.accept')),('submission',_call(g,'io.command'))])
    world=_method(trees[SOURCE_FILES[1]],'World','frame')
    order('ros_pending_before_png_and_dds',SOURCE_FILES[1],[('capture_return',_call(world,'self.world.capture')),
        ('pending_arm',_call(world,'self.gate.arm')),('public_png',_png(world,"f['rgb']")),('camera_publish',_call(world,'self.camera.publish'))])
    nav=_method(trees[SOURCE_FILES[1]],'Navigation','drain')
    claims.append(dict(claim='ros_navigation_update_location',path=SOURCE_FILES[1],anchors=[dict(anchor='Navigation.drain controller_update',line=_call(nav,'self.controller.update'))],scope='declared separate role/method only, not a new process observation'))
    protocol=trees[SOURCE_FILES[2]];arm=_method(protocol,'MotorGate','arm');poll=_method(protocol,'MotorGate','poll');accept=_method(protocol,'MotorGate','accept')
    def assignment(method,target,expression):
        nodes=[n for n in ast.walk(method) if isinstance(n,ast.Assign) and any(ast.unparse(t)==target for t in n.targets)
               and ast.unparse(n.value)==expression];check(len(nodes)==1,'source_assignment_anchor:'+target);return nodes[0].lineno
    order('existing_pending_and_hold_rules',SOURCE_FILES[2],[('arm_poll_prior_hold',_call(arm,'self.poll')),
        ('pending_relative_to_arm',assignment(arm,'self.deadline','wall + self.lease_s'))])
    check('wall >= d' in ast.unparse(poll) and '(self.deadline, self.hold_deadline)' in ast.unparse(poll),'source_effective_deadline_rule_changed')
    order('ros_timely_result_starts_hold',SOURCE_FILES[2],[('accept_poll',_call(accept,'self.poll')),
        ('accepted_hold',assignment(accept,'self.hold_deadline','wall + self.lease_s'))])
    act=trees[SOURCE_FILES[3]]
    values={n.targets[0].id:n.value.value for n in act.body if isinstance(n,ast.Assign) and len(n.targets)==1
            and isinstance(n.targets[0],ast.Name) and isinstance(n.value,ast.Constant)}
    check(values.get('LEASE_S')==.8 and values.get('AUDIT_TIMEOUT_S')==.4,'source_budget_changed')
    check(_method(protocol,'MotorGate','__init__').args.defaults[-1].value==.8,'source_ros_default_lease_changed')
    proposal=_method(trees[SOURCE_FILES[4]],'DeadlineProposal','admit');text=ast.unparse(proposal)
    check("view.source != 'bounded_exposure_mapping'" in text and 'deadline = min(lower_bounds) + LEASE_S' in text,'source_prior_exposure_proposal_changed')
    claims.append(dict(claim='prior_shadow_requires_assumed_exposure_mapping',path=SOURCE_FILES[4],anchors=[dict(anchor='DeadlineProposal.admit',line=proposal.lineno)],scope='existing shadow design already present; NOT a new implementation or trusted provider'))
    check(all(read_index(Path(root)/n)==raw[n] for n in SOURCE_FILES),'source_changed_during_audit')
    return dict(file_sha256=pins,claims=claims,original_lease_s=.8,original_ack_s=.4,source_only=True,
        native_header_scope='frozen bytes only; no C++ behavior or native runtime executed',production_changes=False,native_transport=False,movement=False)


def schedule(name):
    check(name in CASES or name in INVALID,'unknown_case');start=10_000_000_000
    b=dict(frame_id='fictional-frame-1',captured_sim_ns=100_000_000,
        view_digests=[[n,str(i+1)*64] for i,n in enumerate(('front','left','right','back'))])
    doc=dict(format=FORMAT,clock_domain='synthetic_monotonic_ns',capture_started_ns=start-20_000_000,
        frame_available_ns=start,serialized_ns=start+20_000_000,compute_started_ns=start+20_000_000,
        compute_returned_ns=start+100_000_000,review_ns=start+1_500_000_000,held_deadline_ns=None,
        input_binding=b,result_binding=copy.deepcopy(b))
    duration={'first_slow_compute':1_100_000_000,'exact_boundary':LEASE_NS,'one_ns_before':LEASE_NS-1,
        'one_ns_after':LEASE_NS+1,'completion_rebase_counterexample':700_000_000,'stationary_sim_slow_host':1_100_000_000}.get(name)
    if duration is not None:doc['compute_returned_ns']=start+duration
    if name=='slow_serialization':doc.update(serialized_ns=start+900_000_000,compute_started_ns=start+900_000_000,compute_returned_ns=start+1_000_000_000)
    if name=='slow_capture':doc['capture_started_ns']=start-1_100_000_000
    if name.startswith('held_'):
        delta={'held_expires_first':50_000_000,'held_exact_return':100_000_000,'held_timely_replacement':300_000_000,
               'held_after_pending':2_000_000_000,'held_already_expired':0}[name];doc['held_deadline_ns']=start+delta
    if name in ('never_returns','owner_stopped'):doc.update(compute_returned_ns=None,result_binding=None)
    if name=='wrong_frame':doc['result_binding']['frame_id']='different-frame'
    if name=='wrong_sim_identity':doc['result_binding']['captured_sim_ns']+=1
    if name=='wrong_view_digest':doc['result_binding']['view_digests'][0][1]='0'*64
    if name=='six_views':
        for key in ('input_binding','result_binding'):doc[key]['view_digests'].extend([['up','5'*64],['down','6'*64]])
    if name=='bool_host_stamp':doc['frame_available_ns']=True
    if name=='float_host_stamp':doc['frame_available_ns']=float(start)
    if name=='negative_host_stamp':doc['capture_started_ns']=-1
    if name=='deadline_overflow':
        for key in ('capture_started_ns','frame_available_ns','serialized_ns','compute_started_ns','compute_returned_ns','review_ns'):doc[key]=MAX_NS
    if name=='stages_reversed':doc['serialized_ns']=start-1
    if name=='return_before_compute':doc['compute_returned_ns']=start
    if name=='review_before_return':doc['review_ns']=start+50_000_000
    if name=='extra_field':doc['motion_authority']=True
    if name=='wrong_clock_domain':doc['clock_domain']='simulator_time'
    if name=='missing_view':doc['input_binding']['view_digests'].pop()
    if name=='duplicate_view':doc['input_binding']['view_digests'][1]=doc['input_binding']['view_digests'][0]
    if name=='result_without_return':doc['compute_returned_ns']=None
    if name=='missing_result_binding':doc['result_binding']=None
    if name=='bool_sim_identity':doc['input_binding']['captured_sim_ns']=True
    return doc


def exercise(name,output):
    output=Path(output);output.mkdir(exist_ok=False);input_path=output/'schedule.json';write(input_path,schedule(name));pin=sha(input_path)
    try:
        result=asdict(compare_schedule(parse(read_index(input_path))));row=dict(case=name,valid=True,result=result,error=None)
    except (ContractError,BundleError) as exc:row=dict(case=name,valid=False,result=None,error=str(exc))
    check(row['valid']==(name in CASES),'unexpected_case_outcome:'+name)
    check(sha(input_path)==pin,'fixture_input_changed');row.update(input_sha256=pin,synthetic_only=True)
    write(output/'result.json',row);return row


def run(root,output,source_pins,source_pins_sha256):
    output=Path(output);output.mkdir(exist_ok=False)
    raw=read_index(source_pins);check(hashlib.sha256(raw).hexdigest()==source_pins_sha256,'source_pin_file_mismatch')
    audit=audit_sources(root,parse(raw));write(output/'source-audit.json',audit)
    cases=[exercise(n,output/n) for n in CASES+INVALID]
    check(read_index(source_pins)==raw,'source_pin_file_changed')
    summary=dict(source_audit=audit,cases=cases,valid_cases=len(CASES),invalid_refused=len(INVALID),
        synthetic_only=True,actual_controller_or_capture_timing_measured=False,production_changes=False,native_transport=False,movement=False)
    write(output/'summary.json',summary);return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--source-pins',type=Path,required=True);p.add_argument('--source-pins-sha256',required=True);args=p.parse_args()
    print(json.dumps(run(Path(__file__).resolve().parents[1],args.output,args.source_pins,args.source_pins_sha256)))
