"""Opt-in zero-only fake-clock LeaseGuard / standalone ReceiverLease probe.

NOT a runtime issuer, owner, transport, exposure provider or production gate.
Calls the ORIGINAL pure guards with deliberately fictional absolute deadlines.
Raw rejection/counterexample packets bypass the abstract candidate deliberately.
No Gazebo native constructor, socket, publisher, sleep, clock read or device.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from simulation.gazebo_actuator import LeaseGuard, LEASE_S, AUDIT_TIMEOUT_S
from ros2_ws.src.toponav_bridge.toponav_bridge.protocol import command_from_decision
from tools.offline_scope_index import read_index
from tools.offline_scope_review import parse
from tools.precompute_deadline_cases import schedule
from tools.precompute_deadline_contract import compare_schedule, check, shape

ROOT=Path(__file__).resolve().parents[1]
SESSION='synthetic-zero'
MAX_NS=1_000_000_000_000
SOURCE_FILES=('simulation/gazebo_actuator.py','simulation/native/receiver_lease.hh',
    'ros2_ws/src/toponav_bridge/toponav_bridge/protocol.py','tools/precompute_deadline_contract.py',
    'tools/precompute_deadline_cases.py','tools/work_budget_receiver_probe.py','tools/work_budget_receiver_zero.cc')
CASES=('quick','delayed_reception','first_slow_result','exact_boundary','one_ns_before','one_ns_after',
    'old_hold_first','old_hold_boundary','timely_replacement','expires_only_extension_refused',
    'joint_resign_counterexample','late_joint_resign_counterexample','post_expiry_fresh_cannot_rearm',
    'owner_stall_before_send','owner_stall_after_send','owner_stall_no_poll','same_frame_cannot_renew',
    'same_packet_replay','slow_capture_gap','delayed_after_deadline')
FALSE_FIELDS=('exposure_age_bounded','provenance_authenticated','shadow_admission_authorized',
    'native_live_observation_authorized','motion_authority','independent_enforcement_observed',
    'physical_stop_proved','production_guard_fixed','availability_budget_authenticated_by_backend')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def write(path,value):
    with Path(path).open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')


def send(now,issued=10_100_000_000,expires=10_800_000_000,seq=1,frame='f1',captured=100_000_000):
    return dict(op='send',now_ns=now,seq=seq,frame=frame,captured_sim_ns=captured,issued_ns=issued,expires_ns=expires)


def poll(now):return dict(op='poll',now_ns=now)


def validate_actions(actions):
    check(type(actions) is list and 1<=len(actions)<=64,'action_count')
    for a in actions:
        check(type(a) is dict and a.get('op') in ('send','poll'),'action_kind')
        shape(a,'op now_ns' if a['op']=='poll' else 'op now_ns seq frame captured_sim_ns issued_ns expires_ns','action_schema')
        fields=('now_ns',) if a['op']=='poll' else ('now_ns','captured_sim_ns','issued_ns','expires_ns')
        check(all(type(a[k]) is int and 0<=a[k]<=MAX_NS for k in fields),'fictional_ns_bound')
        if a['op']=='send':
            check(type(a['seq']) is int and 1<=a['seq']<=64,'sequence_bound')
            check(type(a['frame']) is str and re.fullmatch('[A-Za-z0-9_-]{1,64}',a['frame']) is not None,'frame_schema')
    return actions


def native_input(actions):
    validate_actions(actions);lines=[]
    for a in actions:
        keys=('now_ns',) if a['op']=='poll' else ('now_ns','seq','frame','captured_sim_ns','issued_ns','expires_ns')
        lines.append(a['op']+' '+' '.join(str(a[k]) for k in keys))
    return '\n'.join(lines)+'\n'


def zero_packet(a,request_id):
    # Do not use moving defaults in old tests. No velocity is accepted as input.
    validate_actions([a]);check(a['op']=='send','send_required')
    captured=a['captured_sim_ns']/1e9
    command=command_from_decision(SESSION,a['seq'],dict(state='observing',v_m_s=0.,omega_rad_s=0.,
        control_frame_id=a['frame'],captured_sim_s=captured,decision_sim_s=captured))
    return dict(kind='command',session_id=SESSION,request_id=request_id,command=command,sim_s=captured,
        issued_steady_s=a['issued_ns']/1e9,expires_steady_s=a['expires_ns']/1e9)


def python_trace(actions):
    validate_actions(actions)  # Whole batch validated BEFORE constructing the pure guard.
    guard=LeaseGuard(SESSION);rows=[];request_id=0
    for a in actions:
        accepted=None
        if a['op']=='send':
            request_id+=1;accepted=guard.handle(zero_packet(a,request_id),a['now_ns']/1e9)
        else:guard.poll(a['now_ns']/1e9)
        status=guard.status()
        rows.append(dict(accepted=accepted,latched=status['latched'],reason=status['reason'],seq=status['command_seq'],
            deadline_steady_s=status['deadline_steady_s'],wheels=status['wheels'],frame=guard.frame_id,
            issued_steady_s=guard.issued,captured_sim_s=guard.captured))
    return rows


def compile_helper(root,output):
    """Standalone stdlib/header binary, NEVER the Gazebo plugin or native IO."""
    root=Path(root);output=Path(output)
    check(not output.exists(),'helper_output_exists')
    command=['g++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(root),
             str(root/'tools/work_budget_receiver_zero.cc'),'-o',str(output)]
    result=subprocess.run(command,capture_output=True,text=True,check=False)
    return dict(command=command,returncode=result.returncode,stdout=result.stdout,stderr=result.stderr,
        binary_sha256=sha(output) if result.returncode==0 else None)


def native_trace(binary,actions):
    result=subprocess.run([str(binary)],input=native_input(actions),capture_output=True,text=True,check=False)
    check(result.returncode==0,'standalone_helper_failed')
    rows=[json.loads(line) for line in result.stdout.splitlines()]
    check(len(rows)==len(actions),'standalone_row_count')
    return rows,dict(returncode=result.returncode,stdout=result.stdout,stderr=result.stderr)


def expected(accepted,latched=False,seq=1,deadline=10_800_000_000,frame='f1',issued=10_100_000_000,captured=100_000_000):
    return dict(accepted=accepted,latched=latched,seq=seq,deadline_steady_s=None if deadline is None else deadline/1e9,
        wheels=[0.,0.],frame=frame,issued_steady_s=issued/1e9,captured_sim_s=captured/1e9)


def fixture(name):
    check(name in CASES,'unknown_case');doc=schedule('quick')
    # Use a simple identity supported by both original guards. Digests remain fictional.
    for k in ('input_binding','result_binding'):doc[k]['frame_id']='f1'
    good=expected(True);due=expected(None,True);rejected=expected(False,True,seq=0,deadline=None,frame='',issued=-1_000_000_000,captured=-1_000_000_000)
    actions=[send(10_100_000_000),poll(10_799_999_999),poll(10_800_000_000)];exp=[good,expected(None),due]
    if name=='delayed_reception':actions[0]['now_ns']=10_700_000_000
    if name in ('first_slow_result','exact_boundary','one_ns_before','one_ns_after'):
        now={'first_slow_result':11_100_000_000,'exact_boundary':10_800_000_000,
             'one_ns_before':10_799_999_999,'one_ns_after':10_800_000_001}[name]
        doc['compute_returned_ns']=now;actions=[send(now,issued=now)]
        exp=[expected(True,issued=now)] if name=='one_ns_before' else [rejected]
    if name in ('old_hold_first','old_hold_boundary','timely_replacement'):
        old=send(9_600_000_000,issued=9_600_000_000,expires=10_200_000_000,frame='f0',captured=50_000_000)
        now={'old_hold_first':10_300_000_000,'old_hold_boundary':10_200_000_000,'timely_replacement':10_100_000_000}[name]
        doc.update(held_deadline_ns=10_200_000_000,compute_returned_ns=now)
        actions=[old,send(now,issued=now,seq=2),poll(10_800_000_000)]
        eold=expected(True,deadline=10_200_000_000,frame='f0',issued=9_600_000_000,captured=50_000_000)
        if name=='timely_replacement':exp=[eold,expected(True,seq=2),expected(None,True,seq=2)]
        else:
            bad=dict(eold,accepted=False,latched=True);exp=[eold,bad,dict(bad,accepted=None)]
    if name=='expires_only_extension_refused':
        actions=[send(10_700_000_000,expires=11_500_000_000)];exp=[rejected]
    if name in ('joint_resign_counterexample','late_joint_resign_counterexample'):
        now=10_700_000_000 if name=='joint_resign_counterexample' else 11_100_000_000
        doc['compute_returned_ns']=now
        actions=[send(now,issued=now,expires=now+800_000_000),poll(10_800_000_000 if now<10_800_000_000 else now+1),poll(now+800_000_000)]
        first=expected(True,deadline=now+800_000_000,issued=now)
        exp=[first,dict(first,accepted=None),dict(first,accepted=None,latched=True)]
    if name=='post_expiry_fresh_cannot_rearm':
        actions=[send(10_100_000_000),poll(10_800_000_000),send(10_900_000_000,issued=10_900_000_000,expires=11_700_000_000,seq=2,frame='f2',captured=200_000_000)]
        exp=[good,due,expected(False,True)]
    if name.startswith('owner_stall_'):
        doc['review_ns']=100_000_000_000
        if name=='owner_stall_before_send':
            doc.update(compute_returned_ns=None,result_binding=None)
            actions=[poll(10_000_000_000),poll(100_000_000_000)]
            empty=expected(None,seq=0,deadline=None,frame='',issued=-1_000_000_000,captured=-1_000_000_000);exp=[empty,dict(empty)]
        elif name=='owner_stall_after_send':actions=[send(10_100_000_000),poll(100_000_000_000)];exp=[good,due]
        else:actions=[send(10_100_000_000)];exp=[good]  # NO poll at the much later declared review.
    if name in ('same_frame_cannot_renew','same_packet_replay'):
        a=send(10_200_000_000,issued=10_200_000_000,expires=11_000_000_000,seq=2,captured=200_000_000)
        if name=='same_packet_replay':a=send(10_200_000_000)
        actions=[send(10_100_000_000),a];exp=[good,expected(False,True)]
    if name=='slow_capture_gap':doc['capture_started_ns']=8_000_000_000
    if name=='delayed_after_deadline':actions=[send(10_900_000_000)];exp=[rejected]
    comparison=asdict(compare_schedule(doc))
    return dict(case=name,schedule=doc,comparison=comparison,actions=validate_actions(actions),expected=exp)


def inspect_rows(rows,expected_rows):
    errors=[]
    if len(rows)!=len(expected_rows):errors.append('row_count')
    for i,(actual,wanted) in enumerate(zip(rows,expected_rows)):
        for key,value in wanted.items():
            if actual.get(key)!=value:errors.append(f'{i}:{key}')
        if type(actual.get('reason')) is not str:errors.append(f'{i}:reason_schema')
    return errors


def run(root,output,pins_file,pins_sha256):
    root=Path(root);output=Path(output)
    raw=read_index(pins_file);check(hashlib.sha256(raw).hexdigest()==pins_sha256,'pins_file_mismatch')
    pins=parse(raw);check(type(pins) is dict and set(pins)==set(SOURCE_FILES),'pins_schema')
    check(all(hashlib.sha256(read_index(root/n)).hexdigest()==h for n,h in pins.items()),'source_pin_mismatch')
    check(LEASE_S==.8 and AUDIT_TIMEOUT_S==.4,'original_limits_changed')
    output.mkdir(exist_ok=False)
    compiler=subprocess.run(['g++','--version'],text=True,capture_output=True,check=False)
    write(output/'compiler-version.json',dict(returncode=compiler.returncode,stdout=compiler.stdout,stderr=compiler.stderr,
        hermetic_toolchain=False,system_headers_pinned=False))
    binary=output/'work-budget-receiver-zero';build=compile_helper(root,binary);write(output/'build.json',build)
    check(build['returncode']==0,'standalone_build_failed')
    rows=[]
    for name in CASES:
        folder=output/name;folder.mkdir();f=fixture(name);write(folder/'input.json',f);pin=sha(folder/'input.json')
        py=python_trace(f['actions']);native,process=native_trace(binary,f['actions'])
        with (folder/'native-input.txt').open('x') as stream:stream.write(native_input(f['actions']))
        write(folder/'native-process.json',process)
        errors=dict(python=inspect_rows(py,f['expected']),receiver=inspect_rows(native,f['expected']))
        row=dict(case=name,input_sha256=pin,python=py,receiver=native,errors=errors,
            expectations_match=not any(errors.values()),classification='synthetic_zero_only_software_arithmetic',
            actual_exposure_wall_s=None,**{k:False for k in FALSE_FIELDS})
        check(sha(folder/'input.json')==pin,'fixture_changed');write(folder/'result.json',row);rows.append(row)
    check(read_index(pins_file)==raw,'pins_file_changed')
    check(all(hashlib.sha256(read_index(root/n)).hexdigest()==h for n,h in pins.items()),'source_changed')
    check(sha(binary)==build['binary_sha256'],'helper_binary_changed')
    result=dict(cases=rows,case_count=len(CASES),backend_count=2,source_sha256=pins,
        expectations_match=all(r['expectations_match'] for r in rows),original_lease_s=.8,original_ack_s=.4,
        formal_retries=0,synthetic_only=True,actual_timing_measured=False,native_transport=False,movement=False,
        production_changes=False,actual_exposure_wall_s=None,**{k:False for k in FALSE_FIELDS})
    write(output/'summary.json',result);return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--source-pins',type=Path,required=True)
    parser.add_argument('--source-pins-sha256',required=True);args=parser.parse_args()
    result=run(ROOT,args.output,args.source_pins,args.source_pins_sha256)
    print(json.dumps(dict(case_count=result['case_count'],expectations_match=result['expectations_match'])))
    sys.exit(0 if result['expectations_match'] else 1)
