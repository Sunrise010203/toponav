"""Native C++ receiver policy + XML/audit contracts; no braking claims."""
import copy
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest

from simulation.gazebo_actuator import LeaseGuard
from simulation.gazebo_receiver import ReceiverEvidence, install_interlock
from simulation.gazebo_scenes import SCENARIOS, build_scenario_world
from tests.test_review_gazebo_watchdog import packet, SESSION

ROOT=Path(__file__).resolve().parents[1]
HARNESS=r'''
#include "simulation/native/receiver_lease.hh"
#include <iostream>
#include <iomanip>
#include <sstream>
int main() {
 toponav::ReceiverLease g("test");std::string line;
 while(std::getline(std::cin,line)) {
  std::istringstream in(line);std::string op,s;in>>op>>s;double now=std::stod(s);bool ok=false;
  if(op=="poll")g.Poll(now);
  else {toponav::Envelope p;std::string a,b,c,v,w;
   in>>p.kind>>p.session>>p.seq>>p.frame>>a>>b>>c>>v>>w;
   p.captured=std::stod(a);p.issued=std::stod(b);p.expires=std::stod(c);p.v=std::stod(v);p.w=std::stod(w);
   ok=g.Accept(p,now);
  }
  std::cout<<std::setprecision(17)<<"{\"ok\":"<<(ok?"true":"false")<<",\"latched\":"<<(g.latched?"true":"false")
   <<",\"reason\":\""<<g.reason<<"\",\"seq\":"<<g.seq<<",\"deadline\":"<<g.deadline<<",\"wheels\":["<<g.v<<","<<g.w<<"]}\n";
 }
}
'''

@pytest.fixture(scope='module')
def policy(tmp_path_factory):
    out=tmp_path_factory.mktemp('receiver-policy');src=out/'policy.cc';binary=out/'policy'
    src.write_text(HARNESS)
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(ROOT),str(src),'-o',str(binary)],check=True,capture_output=True)
    def run(*ops):
        result=subprocess.run([str(binary)],input='\n'.join(ops)+'\n',text=True,capture_output=True,check=True)
        return [json.loads(l) for l in result.stdout.splitlines()]
    return run


def command(now=10., **changes):
    p=dict(kind='command',session='test',seq=1,frame='f1',captured=.1,issued=10.,expires=10.8,v=.1,w=0.)
    p.update(changes)
    return 'accept '+str(now)+' '+' '.join(str(p[k]) for k in ('kind','session','seq','frame','captured','issued','expires','v','w'))


def test_native_expiry_and_late_fresh_command_cannot_rearm(policy):
    a,b,c,d=policy(command(),'poll 10.799','poll 10.8',command(10.81,seq=2,frame='f2',captured=.2,issued=10.81,expires=11.61))
    assert a['ok'] and b['wheels']==[.1,0.]
    assert c['latched'] and c['reason']=='command_lease_expired'
    assert not d['ok'] and d['wheels']==[0.,0.]


def test_transport_delay_does_not_reset_deadline(policy):
    a,b=policy(command(10.7),'poll 10.8')
    assert a['deadline']==10.8 and b['wheels']==[0.,0.]


@pytest.mark.parametrize('field,value', [('kind','heartbeat'),('session','wrong'),('seq',2),('captured','nan'),
    ('issued',11.),('issued',-1.),('issued','nan'),('expires',10.),('expires',11.),('expires','inf'),
    ('v',-.1),('v',.151),('v','nan'),('w',.301),('w','inf')])
def test_native_rejects_bad_authority(policy,field,value):
    result=policy(command(**{field:value}))[0]
    assert result['latched'] and not result['ok'] and result['wheels']==[0.,0.]


@pytest.mark.parametrize('changes',[dict(seq=1),dict(frame='f1'),dict(captured=.1)])
def test_native_reused_evidence_cannot_renew(policy,changes):
    values=dict(seq=2,frame='f2',captured=.2,issued=10.1,expires=10.9);values.update(changes)
    a,b=policy(command(),command(10.1,**values))
    assert a['ok'] and not b['ok'] and b['wheels']==[0.,0.]


def test_native_new_frame_can_renew_only_before_original_expiry(policy):
    a,b,c=policy(command(),command(10.5,seq=2,frame='f2',captured=.2,issued=10.5,expires=11.3),'poll 11.3')
    assert a['ok'] and b['ok'] and b['deadline']==11.3
    assert c['wheels']==[0.,0.]


def test_native_idle_creates_no_authority_and_cannot_replace_motion(policy):
    a,b,c=policy(command(9,kind='idle',seq=0,v=0),command(),command(10.1,kind='idle',seq=0,v=0))
    assert a['ok'] and a['deadline']==0 and b['ok']
    assert not c['ok'] and c['latched']


def test_native_stop_requires_no_new_frame_and_latches(policy):
    _,b,c=policy(command(),command(10.1,kind='stop',seq=0,v=0),command(10.2))
    assert b['ok'] and b['reason']=='remote_stop' and not c['ok']


def test_native_clock_regression_fails_closed(policy):
    _,b=policy(command(),'poll 9.9')
    assert b['reason']=='steady_clock_regression' and b['wheels']==[0.,0.]


@pytest.mark.parametrize('name',SCENARIOS)
def test_all_fixture_geometry_cameras_and_original_drive_are_unchanged(name):
    before=ET.fromstring(build_scenario_world(name))
    after=ET.fromstring(install_interlock(ET.tostring(before,encoding='unicode'),'test'))
    robot=after.find("world/model[@name='robot']")
    extra=robot.find("plugin[@name='toponav::ReceiverInterlock']")
    assert extra.findtext('session_id')=='test'
    robot.remove(extra)
    # Ignore pretty-print whitespace only, not physical/render/camera fields.
    def normalize(tree):
        for e in tree.iter():
            if e.text is not None:e.text=e.text.strip() or None
            e.tail=None
        return ET.tostring(tree)
    assert normalize(before)==normalize(after)


@pytest.mark.parametrize('field,value',[('wheel_radius','.09'),('wheel_separation','.31'),('topic','other')])
def test_unsupported_drive_cannot_silently_bypass_interlock(field,value):
    tree=ET.fromstring(build_scenario_world('left'))
    tree.find("world/model[@name='robot']/plugin[@name='gz::sim::systems::DiffDrive']/"+field).text=value
    with pytest.raises(ValueError):install_interlock(ET.tostring(tree,encoding='unicode'),'test')


def test_double_install_or_invalid_session_rejected():
    world=build_scenario_world('left')
    with pytest.raises(ValueError):install_interlock(install_interlock(world,'test'),'test')
    with pytest.raises(ValueError):install_interlock(world,'bad"session')


def status(**changes):
    result=dict(version=1,session_id='test',wall_s=10.,sim_s=.1,latched=False,reason='accepted',
                command_seq=1,deadline_steady_s=10.8,wheels=[.1,0.]);result.update(changes);return result


def test_missing_status_prevents_motion_and_completion():
    evidence=ReceiverEvidence('test')
    with pytest.raises(RuntimeError):evidence.require_ready()
    result=evidence.summary(0.,.1)
    assert result['fault_latched'] and not result['final_zero'] and not result['final_step_fresh']


@pytest.mark.parametrize('changes',[dict(session_id='wrong'),dict(latched=1),dict(version=True),dict(wall_s=-1),
    dict(wheels=[.2,0]),dict(wheels=[0,float('nan')]),dict(reason='unknown'),dict(command_seq=True)])
def test_invalid_native_status_remains_faulted(changes):
    e=ReceiverEvidence('test');e.push(json.dumps(status(**changes)))
    assert e.error and e.summary(0.,.1)['fault_latched']
    with pytest.raises(RuntimeError):e.require_ready()


def test_native_fault_cannot_be_hidden_by_later_status():
    e=ReceiverEvidence('test');e.push(json.dumps(status(latched=True,reason='command_lease_expired',wheels=[0.,0.])))
    e.push(json.dumps(status(wall_s=11.,sim_s=.2,command_seq=2)))
    assert e.error and e.summary(0.,.2)['fault_latched']


def test_receiver_zero_is_not_physical_stop_and_requires_fresh_cleanup_step():
    e=ReceiverEvidence('test');e.push(json.dumps(status(latched=True,reason='remote_stop',wheels=[0.,0.])))
    assert e.summary(0.,.1)['final_step_fresh']
    result=e.summary(.1,.2)
    assert not result['final_step_fresh'] and result['final_zero'] and 'stationary' not in result


def test_worker_publication_preserves_original_issued_and_deadline():
    guard=LeaseGuard(SESSION);p=packet()
    assert guard.handle(p,10.7)
    meta=guard.publication()
    assert meta['kind']=='command' and float(meta['issued_steady_s'])==10. and float(meta['expires_steady_s'])==10.8
    guard.poll(10.8)
    assert guard.publication()['kind']=='stop'


def test_real_adapter_refuses_motion_without_native_receiver_evidence():
    import threading
    from unittest.mock import Mock
    from simulation.gazebo_cross_validation import GazeboIO
    io=GazeboIO.__new__(GazeboIO)
    io.condition=threading.Condition();io.actuator=Mock()
    io.receiver_evidence=ReceiverEvidence('test');io.command_context={'bound':'pending'}
    with pytest.raises(RuntimeError,match='Receiver interlock'):
        io.command(.1,0.)
    io.actuator.request.assert_not_called()
    assert io.command_context is None
    io.command(0.,0.)
    io.actuator.request.assert_called_once_with('stop')


def test_corrupt_receiver_binary_cannot_silently_fall_back(tmp_path,monkeypatch):
    from simulation import gazebo_receiver as module
    binary=tmp_path/'receiver.so';binary.write_bytes(b'not-the-built-plugin')
    build=tmp_path/'build.json';build.write_text(json.dumps(dict(binary_sha256='wrong',sources={})))
    monkeypatch.setattr(module,'BINARY',binary);monkeypatch.setattr(module,'BUILD',build)
    with pytest.raises(RuntimeError,match='Missing/stale'):
        module.verify_binary()
