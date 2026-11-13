"""Fake times and zero commands only; no native Gazebo or actuator runtime."""
import ast
import copy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess

import pytest

from tools import work_budget_receiver_probe as probe
from tools.precompute_deadline_contract import ContractError, compare_schedule


@pytest.fixture(scope='module')
def binary(tmp_path_factory):
    output=tmp_path_factory.mktemp('work-budget-zero')/'helper'
    result=probe.compile_helper(probe.ROOT,output)
    assert result['returncode']==0,result
    assert result['binary_sha256']==hashlib.sha256(output.read_bytes()).hexdigest()
    return output


@pytest.mark.parametrize('name',probe.CASES)
def test_original_python_guard_matches_declared_zero_trace(name):
    f=probe.fixture(name)
    assert not probe.inspect_rows(probe.python_trace(f['actions']),f['expected'])


@pytest.mark.parametrize('name',probe.CASES)
def test_original_header_matches_declared_zero_trace(binary,name):
    f=probe.fixture(name);rows,process=probe.native_trace(binary,f['actions'])
    assert not probe.inspect_rows(rows,f['expected'])
    assert process['returncode']==0 and not process['stderr']


@pytest.mark.parametrize('name',probe.CASES)
def test_schedules_keep_2k_authority_and_exposure_limits(name):
    f=probe.fixture(name)
    assert f['comparison']==asdict(compare_schedule(f['schedule']))
    assert f['comparison']['actual_exposure_wall_s'] is None
    assert all(f['comparison'][k] is False for k in probe.FALSE_FIELDS if k in f['comparison'])
    assert f['comparison']['new_frame_budget_covers_capture_wait'] is False
    for a in f['actions']:
        if a['op']=='send':
            p=probe.zero_packet(a,1)
            assert p['command']['v_m_s']==p['command']['omega_rad_s']==0
            assert p['command']['state']=='observing'
            assert p['sim_s']==p['command']['captured_sim_s']==p['command']['decision_sim_s']


@pytest.mark.parametrize('name',['joint_resign_counterexample','late_joint_resign_counterexample'])
def test_resigning_is_an_explicit_unfixed_backend_binding_gap(binary,name):
    f=probe.fixture(name);original=f['schedule']['frame_available_ns']/1e9+.8
    py=probe.python_trace(f['actions']);native,_=probe.native_trace(binary,f['actions'])
    for rows in (py,native):
        assert rows[0]['accepted'] and rows[0]['deadline_steady_s']>original
        assert not rows[1]['latched'] and rows[2]['latched']
    candidate=f['comparison']['windows'][2]
    assert candidate['reported_result_within_window']==(name=='joint_resign_counterexample')
    if name=='late_joint_resign_counterexample':
        assert candidate['hypothetical_hold_after_timely_result_ns'] is None


def test_no_poll_is_not_a_stop_observation():
    f=probe.fixture('owner_stall_no_poll');rows=probe.python_trace(f['actions'])
    assert f['schedule']['review_ns']/1e9>rows[-1]['deadline_steady_s']
    assert len(rows)==1 and not rows[-1]['latched']
    assert f['actions'][-1]['now_ns']<f['schedule']['review_ns']


def test_pre_send_stall_does_not_invent_receiver_pending(binary):
    f=probe.fixture('owner_stall_before_send')
    assert f['comparison']['windows'][2]['work_deadline_due_at_review']
    for rows in (probe.python_trace(f['actions']),probe.native_trace(binary,f['actions'])[0]):
        assert rows[-1]['seq']==0 and rows[-1]['deadline_steady_s'] is None and not rows[-1]['latched']


def test_timely_replacement_is_not_forever_clipped_to_first_hold(binary):
    f=probe.fixture('timely_replacement')
    for rows in (probe.python_trace(f['actions']),probe.native_trace(binary,f['actions'])[0]):
        assert rows[0]['deadline_steady_s']==10.2
        assert rows[1]['accepted'] and rows[1]['deadline_steady_s']==10.8 and rows[1]['seq']==2


def test_capture_gap_is_not_exposure_evidence():
    f=probe.fixture('slow_capture_gap')
    assert f['comparison']['capture_wait_ns']==2_000_000_000
    assert f['comparison']['windows'][2]['reported_result_within_window']
    assert not f['comparison']['exposure_age_bounded']


INVALID=[[],[{}],[dict(op='stop',now_ns=1)],[dict(op='poll',now_ns=True)],
    [dict(op='poll',now_ns=1.)],[dict(op='poll',now_ns=-1)],
    [dict(op='poll',now_ns=probe.MAX_NS+1)],[dict(op='poll',now_ns=1,v=0)],
    [dict(probe.send(1),seq=True)],[dict(probe.send(1),seq=65)],
    [dict(probe.send(1),frame='x\ny')],[dict(probe.send(1),frame='中')],
    [dict(probe.send(1),frame='')],[dict(probe.send(1),frame='f'*65)],
    [dict(probe.send(1),v=.1)],[dict(probe.send(1),expires_ns=float('nan'))],
    [dict(op='poll',now_ns=1)]*65,tuple([probe.poll(1)])]


@pytest.mark.parametrize('actions',INVALID)
def test_invalid_whole_batch_refused_before_pure_guard_or_subprocess(monkeypatch,actions):
    def forbidden(*a,**kw):raise AssertionError('side effect before validation')
    monkeypatch.setattr(probe,'LeaseGuard',forbidden);monkeypatch.setattr(probe.subprocess,'run',forbidden)
    for fn in (probe.python_trace,probe.native_input):
        with pytest.raises(ContractError):fn(actions)
    with pytest.raises(ContractError):probe.native_trace('/not-used',actions)


@pytest.mark.parametrize('text',['','poll -1\n','poll true\n','poll 1.5\n','poll 1000000000001\n',
    'poll 1 extra\n','send 10 1 f1 0 0 100 0.1\n','send 10 0 f1 0 0 100\n',
    'send 10 1 bad.frame 0 0 100\n','send 10 1 f1 nan 0 100\n','stop 10\n',
    'poll 1\n'*65])
def test_standalone_closed_grammar_rejects_unsafe_input(binary,text):
    p=subprocess.run([str(binary)],input=text,text=True,capture_output=True)
    assert p.returncode==2 and 'invalid synthetic zero-only operation' in p.stderr
    for line in p.stdout.splitlines():assert json.loads(line)['wheels']==[0,0]


def test_whole_batch_validation_even_if_first_action_valid(monkeypatch):
    monkeypatch.setattr(probe,'LeaseGuard',lambda *a:pytest.fail('must validate entire batch first'))
    with pytest.raises(ContractError):probe.python_trace([probe.poll(1),dict(probe.send(2),v=.1)])


def test_fixture_is_fresh_not_shared_mutable_state():
    a=probe.fixture('quick');b=probe.fixture('quick');a['actions'][0]['now_ns']=1
    a['schedule']['input_binding']['view_digests'][0][1]='0'*64
    assert b==probe.fixture('quick')


def test_all_fixture_names_unique_and_unknown_refused():
    assert len(probe.CASES)==len(set(probe.CASES))==20
    with pytest.raises(ContractError):probe.fixture('not-a-case')


def test_probe_contains_no_runtime_entrypoints_and_header_helper_zero_only():
    tree=ast.parse((probe.ROOT/'tools/work_budget_receiver_probe.py').read_text())
    calls={ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n,ast.Call)}
    assert not calls.intersection({'time.monotonic','time.sleep','socket.socket','ActuatorClient','GazeboIO','serve','controller.update'})
    helper=(probe.ROOT/'tools/work_budget_receiver_zero.cc').read_text()
    includes=[line for line in helper.splitlines() if line.startswith('#include "')]
    assert includes==['#include "simulation/native/receiver_lease.hh"']
    assert 'p.v = 0.; p.w = 0.;' in helper
    assert 'gz/' not in helper


def test_inspector_retains_each_mismatch_not_only_successes():
    e=probe.fixture('quick')['expected'];rows=probe.python_trace(probe.fixture('quick')['actions'])
    rows[0]['wheels']=[.1,0];rows[1]['deadline_steady_s']=100;rows[2]['latched']=False
    assert probe.inspect_rows(rows,e)==['0:wheels','1:deadline_steady_s','2:latched']


def test_compile_refuses_overwrite_before_subprocess(tmp_path,monkeypatch):
    output=tmp_path/'exists';output.write_bytes(b'preserve')
    monkeypatch.setattr(probe.subprocess,'run',lambda *a,**k:pytest.fail('must not compile'))
    with pytest.raises(ContractError):probe.compile_helper(probe.ROOT,output)
    assert output.read_bytes()==b'preserve'


def test_formal_pin_mismatch_refused_before_output_or_compile(tmp_path,monkeypatch):
    pins=tmp_path/'pins.json';pins.write_text('{}')
    output=tmp_path/'out'
    monkeypatch.setattr(probe.subprocess,'run',lambda *a,**k:pytest.fail('must not compile'))
    for digest in ('0'*64,hashlib.sha256(pins.read_bytes()).hexdigest()):
        with pytest.raises(ContractError):probe.run(probe.ROOT,output,pins,digest)
    assert not output.exists()
