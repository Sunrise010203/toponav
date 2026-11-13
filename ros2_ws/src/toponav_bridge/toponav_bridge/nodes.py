"""Real DDS processes: camera/actuator owner, public-RGB navigation, feedback.

The world is lock-step, accelerated simulation, NOT a wall-time robot model.
Only its process owns motors; its STEADY_TIME lease still runs when navigation
is SIGSTOP'ed. On expiry it resumes physics with ZERO, latches, and audits the
actual braking trace. No pose/contact/depth crosses a control topic.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image as PNG
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock as RclClock,ClockType
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .protocol import VERSION,MotorGate,command_from_decision,decode,encode,validate_command,validate_event,load_public,number

RELIABLE=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
LATCHED=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)


def save(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n');temporary.replace(path)


def append(path,data):
    with Path(path).open('a') as f:f.write(encode(data)+'\n');f.flush()


def pub(publisher,data):publisher.publish(String(data=encode(data)))


def seconds(stamp):return round(stamp.sec+stamp.nanosec/1e9,9)


def set_stamp(stamp,value):
    ns=round(value*1e9);stamp.sec=ns//10**9;stamp.nanosec=ns%10**9


class Navigation(Node):
    def __init__(self,args):
        super().__init__('navigation',namespace='toponav')
        from roadvision.continuous_straight import ContinuousRouteController,InitialClearancePrior
        from simulation.path_planning import plan_from_estimate,next_action
        self.out=Path(args.output)/'navigation';self.out.mkdir(parents=True,exist_ok=False)
        (self.out/'frames').mkdir();self.task,graph,references=load_public(args.public)
        self.session=self.task['session_id'];self.controller=ContinuousRouteController(
            graph=graph,references=references,plan_from_estimate=plan_from_estimate,next_action=next_action,
            start=self.task['start_node'],goal=self.task['goal_node'],heading=self.task['heading_deg'],
            initial_clearance=InitialClearancePrior(confirmed=self.task['operator_confirmed']),session_id=self.session)
        self.commands=self.create_publisher(String,'motor/command',RELIABLE)
        self.decisions=self.create_publisher(String,'navigation/decision',RELIABLE)
        self.processed=self.create_publisher(String,'camera/processed',RELIABLE)
        self.ready=self.create_publisher(String,'ready/navigation',LATCHED)
        self.create_subscription(Image,'camera/rgb',self.image,RELIABLE)
        self.create_subscription(Clock,'/clock',self.clock,LATCHED)
        self.create_subscription(String,'segment/feedback',self.feedback,RELIABLE)
        self.create_subscription(String,'motor/status',self.status,LATCHED)
        self.seq=0;self.image_count=0;self.now=None;self.queued=None;self.last_delivery=None
        self.waiting=None;self.wait_started=None;self.bridge_reason=None;self.abort_sent=False
        self.steady=RclClock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05,self.tick,clock=self.steady)
        save(self.out/'process.json',dict(role='navigation',pid=__import__('os').getpid(),inputs=['public task/topology/reference RGB','camera/rgb','/clock','segment/feedback','motor/status']))

    def clock(self,msg):
        now=seconds(msg.clock)
        if self.now is not None and now<self.now:
            self.abort('simulation_clock_regression');return
        self.now=now;self.drain()

    def emit(self,decision,kind,**record):
        self.seq+=1;command=command_from_decision(self.session,self.seq,decision)
        # A zero request must not wait for filesystem/diagnostic I/O.
        # Normal motion still requires its durable decision trail first.
        zero=command['v_m_s']==command['omega_rad_s']==0.
        if zero:
            pub(self.commands,command)
            if command['state']=='aborted':self.abort_sent=True
        entry=dict(kind=kind,now_sim_s=self.now,steady_s=time.monotonic(),decision=decision,command=command,**record)
        append(self.out/'events.jsonl',entry)
        save(self.out/'result.json',dict(controller_result=self.controller.result(),bridge_reason=self.bridge_reason,commands=self.seq,images=self.image_count))
        pub(self.decisions,dict(version=VERSION,session_id=self.session,seq=self.seq,decision=decision))
        if not zero:pub(self.commands,command)

    def abort(self,reason):
        # update() can latch aborted BEFORE raising. That is not proof that
        # the transport has already sent the corresponding motor zero.
        if self.controller.state=='visual_stop' or self.abort_sent:return
        self.bridge_reason=reason;self.waiting=None;self.queued=None;self.wait_started=None
        decision=self.controller.cancel(self.now if self.now is not None else 0.)
        self.emit(decision,'bridge_abort',transport_reason=reason)

    def image(self,msg):
        if self.queued is not None:
            self.abort('rgb_backlog_not_single_flight');return
        self.queued=msg;self.last_delivery=time.monotonic();self.drain()

    def drain(self):
        if self.queued is None or self.now is None or self.waiting is not None:return
        msg=self.queued
        # Different DDS topics have no ordering guarantee. Wait for the clock
        # from this batch; a genuinely future header is rejected by timeout or
        # the controller, never silently retimestamped.
        captured=seconds(msg.header.stamp)
        if captured>self.now+1e-8:return
        self.queued=None
        try:
            if msg.encoding!='rgb8' or msg.width!=256 or msg.height!=256 or msg.step!=768 or msg.is_bigendian or len(msg.data)!=196608 or not msg.header.frame_id.startswith(self.session+'/'):raise ValueError('Invalid RGB ROS envelope')
            rgb=np.frombuffer(bytes(msg.data),dtype=np.uint8).reshape(256,256,3).copy()
            frame=dict(frame_id=msg.header.frame_id,captured_sim_s=captured,rgb=rgb)
            path=f'frames/{self.image_count:04d}.png';PNG.fromarray(rgb).save(self.out/path)
            ref=dict(frame_id=frame['frame_id'],captured_sim_s=captured,rgb_path=path,rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest())
            before=self.controller.result();terminal=before['state'] in ('aborted','visual_stop')
            d=self.controller.update(frame,self.now);self.image_count+=1
            if d['pending_feedback'] is not None and not terminal:
                self.waiting=dict(d['pending_feedback']);self.wait_started=time.monotonic()
            self.emit(d,'image',input=ref,terminal=terminal,result_unchanged=self.controller.result()==before if terminal else None)
            pub(self.processed,dict(version=VERSION,session_id=self.session,frame_id=frame['frame_id']))
        except Exception as error:self.abort('image_or_perception_error:'+type(error).__name__)

    def feedback(self,msg):
        try:
            data=decode(msg.data)
            if set(data)!={'version','session_id','applied_seq','applied_sim_s','event'} or type(data['version']) is not int or data['version']!=VERSION or data['session_id']!=self.session or type(data['applied_seq']) is not int or not number(data['applied_sim_s']):raise ValueError('Invalid feedback envelope')
            validate_event(data['event'],self.session)
            if self.waiting is None or data['event']!=self.waiting or data['applied_seq']!=self.seq:raise ValueError('Unbound feedback')
            if self.now is None or not 0<=self.now-data['applied_sim_s']<=.250001:raise ValueError('Stale feedback transport')
            d=self.controller.feedback(data['event'],self.now)
            append(self.out/'events.jsonl',dict(kind='feedback',now_sim_s=self.now,steady_s=time.monotonic(),feedback=data,decision=d))
            self.waiting=None;self.wait_started=None
            save(self.out/'result.json',dict(controller_result=self.controller.result(),bridge_reason=self.bridge_reason,commands=self.seq,images=self.image_count))
            self.drain()
        except (ValueError,TypeError,KeyError):self.abort('invalid_segment_feedback_transport')

    def status(self,msg):
        try:
            data=decode(msg.data)
            if type(data['version']) is not int or data['version']!=VERSION or data['session_id']!=self.session:return
            if data['latched'] and data['reason']!='remote_visual_stop':self.abort('actuator_latched:'+data['reason'])
        except (ValueError,TypeError,KeyError):self.abort('invalid_actuator_status')

    def tick(self):
        pub(self.ready,dict(version=VERSION,session_id=self.session,role='navigation'))
        if self.controller.state in ('visual_stop','aborted'):return
        now=time.monotonic()
        if self.waiting is not None:
            if now-self.wait_started>.6:self.abort('feedback_wall_timeout')
        elif self.last_delivery is not None and now-self.last_delivery>.6:self.abort('camera_or_clock_wall_timeout')


class Feedback(Node):
    """Transport echo of an accepted zero-speed visual exit, NOT node arrival."""
    def __init__(self,args):
        super().__init__('feedback',namespace='toponav')
        self.session=decode((Path(args.public)/'task.json').read_text())['session_id']
        self.out=Path(args.output)/'feedback';self.out.mkdir(parents=True,exist_ok=False)
        self.publisher=self.create_publisher(String,'segment/feedback',RELIABLE)
        self.ready=self.create_publisher(String,'ready/feedback',LATCHED)
        self.create_subscription(String,'motor/applied',self.applied,RELIABLE)
        self.drop=args.drop_feedback or self.declare_parameter('drop_feedback',False).value;self.seen=set()
        self.create_timer(.1,lambda:pub(self.ready,dict(version=VERSION,session_id=self.session,role='feedback')),clock=RclClock(clock_type=ClockType.STEADY_TIME))
        save(self.out/'process.json',dict(role='feedback',pid=__import__('os').getpid()))

    def applied(self,msg):
        try:
            data=decode(msg.data)
            if set(data)!={'version','session_id','applied_seq','applied_sim_s','command'} or type(data['version']) is not int or data['version']!=VERSION or data['session_id']!=self.session or type(data['applied_seq']) is not int or not number(data['applied_sim_s']):return
            command=validate_command(data['command']);event=command['segment_event']
        except (ValueError,TypeError,KeyError):return
        if event is None:return
        if data['applied_seq']!=command['seq'] or command['v_m_s'] or command['omega_rad_s'] or event['event_id'] in self.seen:return
        self.seen.add(event['event_id'])
        response={k:data[k] for k in ('version','session_id','applied_seq','applied_sim_s')}|dict(event=event)
        append(self.out/'events.jsonl',dict(dropped=self.drop,response=response))
        if not self.drop:pub(self.publisher,response)


class World(Node):
    def __init__(self,args):
        super().__init__('world',namespace='toponav')
        from simulation.continuous_audit_world import RouteAuditWorld
        from simulation.topology import validate_pair
        self.base=Path(args.output);self.out=self.base/'world';self.out.mkdir(parents=True,exist_ok=False)
        self.audit=self.out/'audit';self.audit.mkdir();self.public=Path(args.public)
        (self.public/'frames').mkdir()
        self.case=decode(Path(args.scenario).read_text());self.task=decode((self.public/'task.json').read_text())
        self.session=self.task['session_id'];scene=self.case['physical_scene']
        validate_pair(decode((self.public/'topology.json').read_text()),scene)
        self.world=RouteAuditWorld(physical_scene=scene,assets=self.audit/'assets',seed=self.case['seed'],spawn_node=self.task['start_node'],spawn_heading_deg=self.task['heading_deg'])
        self.initial=self.world.audit_state();self.truth=[self.initial];self.rows=[];self.frames=[]
        self.gate=MotorGate(self.session);self.started=time.monotonic();self.tick_id=0
        self.ready_roles=set();self.phase='startup';self.done=False;self.settled=0
        self.probes=[];self.probe_wait=None;self.probe_deadline=None;self.rejected_after_stop=0
        self.clock_pub=self.create_publisher(Clock,'/clock',LATCHED)
        self.camera=self.create_publisher(Image,'camera/rgb',RELIABLE)
        self.applied=self.create_publisher(String,'motor/applied',RELIABLE)
        self.status_pub=self.create_publisher(String,'motor/status',LATCHED)
        self.create_subscription(String,'motor/command',self.command,RELIABLE)
        self.create_subscription(String,'camera/processed',self.processed,RELIABLE)
        for role in ('navigation','feedback'):
            self.create_subscription(String,'ready/'+role,self.ready, LATCHED)
        self.create_timer(.005,self.timer,clock=RclClock(clock_type=ClockType.STEADY_TIME))
        save(self.out/'process.json',dict(role='world',pid=__import__('os').getpid(),clock='lock-step sim + independent steady wall lease',lease_wall_s=self.gate.lease_s))
        self.publish_clock()

    @property
    def sim_s(self):return round(self.world.sim_s,9)

    def publish_clock(self):
        c=Clock();set_stamp(c.clock,self.sim_s);self.clock_pub.publish(c)

    def ready(self,msg):
        try:
            data=decode(msg.data)
            if type(data.get('version')) is int and data==dict(version=VERSION,session_id=self.session,role=data.get('role')) and data['role'] in ('navigation','feedback'):self.ready_roles.add(data['role'])
        except (ValueError,TypeError,KeyError):return

    def progress(self):
        save(self.out/'progress.json',dict(tick=self.tick_id,phase=self.phase,latched=self.gate.latched,reason=self.gate.reason,steady_elapsed_s=time.monotonic()-self.started))

    def frame(self,terminal=False):
        self.publish_clock();f=self.world.capture();f['frame_id']=self.session+'/'+f['frame_id'];f['captured_sim_s']=self.sim_s
        if not terminal:self.gate.arm(f['frame_id'],self.sim_s,time.monotonic())
        mode=self.case['fault'];fault_tick=self.case['fault_tick']
        suppressed=not terminal and mode=='camera_silence' and self.tick_id==fault_tick
        captured=f['captured_sim_s']
        if not terminal and mode=='future_camera' and self.tick_id==fault_tick:captured+=1.
        relative=f'frames/{self.tick_id:04d}.png';PNG.fromarray(f['rgb']).save(self.public/relative)
        ref=dict(frame_id=f['frame_id'],captured_sim_s=captured,rgb_path=relative,rgb_sha256=hashlib.sha256(f['rgb'].tobytes()).hexdigest(),terminal=terminal,suppressed=suppressed,tick=self.tick_id,
                 lease_deadline_steady_s=self.gate.deadline if not terminal else None)
        self.frames.append(ref)
        if terminal:
            self.probes.append(dict(frame_id=f['frame_id'],processed=False));self.probe_wait=f['frame_id'];self.probe_deadline=time.monotonic()+1.5
        if not suppressed:
            msg=Image();msg.height=msg.width=256;msg.encoding='rgb8';msg.is_bigendian=0;msg.step=768
            msg.header.frame_id=f['frame_id'];set_stamp(msg.header.stamp,captured);msg.data=f['rgb'].tobytes();self.camera.publish(msg)
        self.progress()

    def record_stop(self,reason):
        self.gate.trip(reason);self.world.drive(0.,0.)
        self.rows.append(dict(tick=self.tick_id,sim_s=self.sim_s,steady_s=time.monotonic(),decision=dict(v_m_s=0.,omega_rad_s=0.),gate_reason=self.gate.reason,source='gate_stop'))
        self.phase='settling';self.announce();self.progress()

    def announce(self):
        pub(self.status_pub,dict(version=VERSION,session_id=self.session,latched=self.gate.latched,reason=self.gate.reason,sim_s=self.sim_s))

    def command(self,msg):
        try:c=decode(msg.data)
        except ValueError:c={}
        previous_latch=self.gate.latched
        wheels=self.gate.accept(c,self.sim_s,time.monotonic());self.world.drive(*wheels)
        record=dict(tick=self.tick_id,sim_s=self.sim_s,steady_s=time.monotonic(),command=c,applied_v_m_s=wheels[0],applied_omega_rad_s=wheels[1],reason=self.gate.reason,already_latched=previous_latch)
        append(self.out/'commands.jsonl',record)
        if previous_latch:
            self.rejected_after_stop+=1;return
        self.rows.append(dict(tick=self.tick_id,sim_s=self.sim_s,decision=dict(v_m_s=wheels[0],omega_rad_s=wheels[1]),gate_reason=self.gate.reason,source='command'))
        if self.gate.latched:
            self.record_stop(self.gate.reason);return
        pub(self.applied,dict(version=VERSION,session_id=self.session,applied_seq=c['seq'],applied_sim_s=self.sim_s,command=c))
        self.world.step_control();self.truth.append(self.world.audit_state());self.tick_id+=1
        if self.tick_id>=self.case['max_ticks']:
            self.record_stop('execution_budget_exhausted');return
        self.frame()

    def processed(self,msg):
        try:
            data=decode(msg.data)
            if type(data.get('version')) is int and data==dict(version=VERSION,session_id=self.session,frame_id=self.probe_wait) and self.probe_wait is not None:
                self.probes[-1]['processed']=True;self.probe_wait=None
        except (ValueError,TypeError,KeyError):return

    def timer(self):
        if self.done:return
        wall=time.monotonic()
        if self.phase=='startup':
            self.publish_clock()
            if self.ready_roles=={'navigation','feedback'}:
                self.phase='running';self.frame()
            elif wall-self.started>20:self.record_stop('readiness_timeout')
        elif self.phase=='running':
            self.gate.poll(wall)
            if self.gate.latched:self.record_stop(self.gate.reason)
        elif self.phase=='settling':
            self.world.drive(0.,0.);self.world.step_control();self.truth.append(self.world.audit_state());self.publish_clock();self.settled+=1
            if self.settled>=10:self.phase='probing'
        elif self.phase=='probing':
            if self.probe_wait is not None and wall<=self.probe_deadline:return
            if self.probe_wait is not None:self.probe_wait=None
            if len(self.probes)==3:self.finish();return
            self.world.drive(0.,0.);self.world.step_control();self.truth.append(self.world.audit_state());self.tick_id+=1;self.frame(terminal=True)

    def finish(self):
        final=self.world.audit_state();trace=list(self.world.physics_trace)
        audit=dict(initial=self.initial,final=final,wall_contact_physics_steps=self.world.wall_contact_physics_steps,
                   maximum_audited_speed_m_s=max(t['speed_m_s'] for t in trace),
                   maximum_audited_yaw_rate_rad_s=max(abs(t['yaw_rate_rad_s']) for t in trace))
        save(self.audit/'truth.json',self.truth);save(self.audit/'physics-trace.json',trace)
        save(self.out/'applied-rows.json',self.rows);save(self.public/'frames.json',self.frames)
        save(self.out/'result.json',dict(session_id=self.session,gate_reason=self.gate.reason,gate_latched=self.gate.latched,
                                      terminal_probes=self.probes,rejected_after_stop=self.rejected_after_stop,physics_audit=audit,ticks=self.tick_id))
        self.world.drive(0.,0.);self.world.close();self.done=True


class Fault(Node):
    """Test-only DDS publisher. Never used by normal navigation."""
    def __init__(self,args):
        super().__init__('fault_injector',namespace='toponav')
        self.mode=args.fault_mode;self.tick=args.fault_tick;self.sent=False
        self.out=Path(args.output)/'fault';self.out.mkdir(parents=True,exist_ok=False)
        self.publisher=self.create_publisher(String,'motor/command',RELIABLE)
        self.create_subscription(String,'motor/command',self.command,RELIABLE)

    def command(self,msg):
        if self.sent:return
        data=decode(msg.data)
        if data['seq']!=self.tick+1:return
        self.sent=True
        if self.mode=='bad_version':data['version']=999
        pub(self.publisher,data);save(self.out/'injected.json',dict(mode=self.mode,command=data,steady_s=time.monotonic()))


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('role',choices=['world','navigation','feedback','fault']);p.add_argument('--public',required=True)
    p.add_argument('--output',required=True);p.add_argument('--scenario');p.add_argument('--drop-feedback',action='store_true')
    p.add_argument('--fault-mode',choices=['replay_command','bad_version']);p.add_argument('--fault-tick',type=int,default=180)
    args,ros_args=p.parse_known_args(argv);rclpy.init(args=ros_args)
    node=None
    try:
        node={'world':World,'navigation':Navigation,'feedback':Feedback,'fault':Fault}[args.role](args)
        while rclpy.ok() and not getattr(node,'done',False):rclpy.spin_once(node,timeout_sec=.02)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:
            if isinstance(node,World) and not node.done:node.world.drive(0.,0.);node.world.close()
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
