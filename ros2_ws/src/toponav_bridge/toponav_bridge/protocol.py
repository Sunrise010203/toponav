"""Versioned public wire contract and independent, latched actuator lease.

No ROS, physics, pose, scene or audit imports. Steady wall time and simulation
capture time are intentionally separate; neither is substituted for the other.
"""
import json
import math

VERSION=1
COMMAND_FIELDS={'version','session_id','seq','frame_id','captured_sim_s',
                'decision_sim_s','v_m_s','omega_rad_s','state','segment_event'}


def number(value):
    if type(value) not in (int,float):return False
    try:return math.isfinite(value)
    except OverflowError:return False


def identity(value):
    return isinstance(value,str) and 0<len(value)<=256


def decode(raw):
    def pairs(items):
        result={}
        for key,value in items:
            if key in result:raise ValueError('Duplicate JSON key')
            result[key]=value
        return result
    def invalid(value):raise ValueError('Nonfinite JSON number')
    if not isinstance(raw,str) or len(raw)>2_000_000:raise ValueError('Invalid wire size')
    def floating(value):
        result=float(value)
        if not math.isfinite(result):raise ValueError('Nonfinite JSON number')
        return result
    data=json.loads(raw,object_pairs_hook=pairs,parse_constant=invalid,parse_float=floating)
    if not isinstance(data,dict):raise ValueError('Wire object required')
    return data


def encode(data):
    return json.dumps(data,allow_nan=False,separators=(',',':'))


def load_public(root):
    """Load ONLY explicit public priors, never scene or physical audit data."""
    from pathlib import Path
    import hashlib
    import numpy as np
    from PIL import Image
    from roadvision.corridor_rgb_probe import CALIBRATION_ID
    root=Path(root).resolve();task=decode((root/'task.json').read_text())
    if set(task)!={'version','session_id','start_node','goal_node','heading_deg','operator_confirmed','calibration_id'} or type(task['version']) is not int or task['version']!=VERSION or type(task['operator_confirmed']) is not bool:raise ValueError('Invalid public task')
    if task['calibration_id']!=CALIBRATION_ID:raise ValueError('Calibration mismatch')
    graph=decode((root/'topology.json').read_text());references=[]
    inventory=json.loads((root/'references.json').read_text())
    if not isinstance(inventory,list):raise ValueError('Invalid reference inventory')
    for ref in inventory:
        if set(ref)!={'node_id','heading_deg','rgb_path','rgb_sha256'}:raise ValueError('Invalid public reference fields')
        path=(root/ref['rgb_path']).resolve()
        if Path(ref['rgb_path']).is_absolute() or not path.is_relative_to(root):raise ValueError('Reference escaped public root')
        rgb=np.array(Image.open(path))
        if rgb.shape!=(256,256,3) or rgb.dtype!=np.uint8 or hashlib.sha256(rgb.tobytes()).hexdigest()!=ref['rgb_sha256']:raise ValueError('Invalid public reference pixels')
        references.append(dict(node_id=ref['node_id'],heading_deg=ref['heading_deg'],rgb=rgb))
    return task,graph,references


def command_from_decision(session,seq,decision):
    return dict(version=VERSION,session_id=session,seq=seq,
                frame_id=decision['control_frame_id'],captured_sim_s=decision['captured_sim_s'],
                decision_sim_s=decision['decision_sim_s'],v_m_s=decision['v_m_s'],
                omega_rad_s=decision['omega_rad_s'],state=decision['state'],
                segment_event=decision.get('pending_feedback') if decision['state']=='waiting_feedback' else None)


def validate_event(event,session):
    fields={'event_id','command_id','status','frame_id','captured_sim_s'}
    if not isinstance(event,dict) or set(event)!=fields:raise ValueError('Invalid segment event fields')
    if any(not identity(event[k]) for k in ('event_id','command_id','frame_id')):raise ValueError('Invalid segment identity')
    if not event['command_id'].startswith(session+':cmd-') or event['event_id']!=event['command_id']+':visual-exit':raise ValueError('Unbound segment identity')
    if event['status']!='visual_segment_complete' or not number(event['captured_sim_s']) or event['captured_sim_s']<0:raise ValueError('Invalid segment status/time')


def validate_command(c):
    if not isinstance(c,dict) or set(c)!=COMMAND_FIELDS:raise ValueError('Invalid command fields')
    if type(c['version']) is not int or c['version']!=VERSION:raise ValueError('Unsupported command version')
    if not identity(c['session_id']) or not identity(c['frame_id']):raise ValueError('Invalid command identity')
    if type(c['seq']) is not int or c['seq']<1:raise ValueError('Invalid command sequence')
    if not all(number(c[k]) for k in ('captured_sim_s','decision_sim_s','v_m_s','omega_rad_s')):raise ValueError('Invalid numeric command')
    if c['captured_sim_s']<0 or c['decision_sim_s']<0:raise ValueError('Negative simulation time')
    if not 0<=c['v_m_s']<=.15 or abs(c['omega_rad_s'])>.30:raise ValueError('Command speed outside restricted domain')
    if c['state'] not in ('observing','moving','waiting_feedback','visual_stop','aborted'):raise ValueError('Invalid route state')
    if c['state']!='moving' and (c['v_m_s'] or c['omega_rad_s']):raise ValueError('Non-moving state command must be zero')
    if c['segment_event'] is not None:
        validate_event(c['segment_event'],c['session_id'])
        if c['state']!='waiting_feedback' or (c['segment_event']['frame_id'],c['segment_event']['captured_sim_s'])!=(c['frame_id'],c['captured_sim_s']):raise ValueError('Segment event not bound to zero RGB command')
    return c


class MotorGate:
    """One fresh frame -> one bounded command; ANY error latches physical zero.

    Pending-frame and accepted-command hold deadlines are separate. Arming
    another frame cannot extend a held velocity. The owner must still poll;
    this class cannot stop a hung process or hardware by itself.

    A zero abort is allowed without a new frame: stopping does not grant motion
    or arrival authority. No other message refreshes a lease. Re-arm requires a
    NEW process/session, never merely recovery of camera or DDS delivery.
    """
    def __init__(self,session_id,lease_s=.8):
        if not identity(session_id) or not number(lease_s) or not .05<=lease_s<=2.:raise ValueError('Invalid session/lease')
        self.session_id=session_id;self.lease_s=lease_s
        self.latched=False;self.reason='disarmed';self.seq=0;self.expected=None
        self.deadline=None;self.hold_deadline=None;self.last_wall=-1.;self.wheels=(0.,0.)

    def trip(self,reason):
        if not self.latched:self.reason=reason
        self.latched=True;self.wheels=(0.,0.)
        return self.wheels

    def _wall(self,wall):
        if not number(wall) or wall<self.last_wall:return False
        self.last_wall=wall;return True

    def arm(self,frame_id,captured_sim_s,wall):
        if self.latched:return
        if not self._wall(wall) or not identity(frame_id) or not number(captured_sim_s) or captured_sim_s<0:
            self.trip('invalid_frame_lease');return
        if self.expected is not None:
            self.trip('frame_lease_overwritten');return
        self.poll(wall)
        if self.latched:return
        self.expected=(frame_id,captured_sim_s);self.deadline=wall+self.lease_s;self.reason='awaiting_bound_command'

    def poll(self,wall):
        if not self._wall(wall):return self.trip('steady_clock_regression')
        if not self.latched and any(d is not None and wall>=d for d in (self.deadline,self.hold_deadline)):return self.trip('command_lease_expired')
        return self.wheels

    def accept(self,c,sim_s,wall):
        if self.latched:return (0.,0.)
        if self.poll(wall)==(0.,0.) and self.latched:return (0.,0.)
        try:validate_command(c)
        except (ValueError,TypeError,KeyError):return self.trip('invalid_command_envelope')
        if c['session_id']!=self.session_id:return self.trip('wrong_session')
        if c['seq']!=self.seq+1:return self.trip('replayed_or_out_of_order_command')
        if not number(sim_s) or not 0<=c['captured_sim_s']<=c['decision_sim_s']<=sim_s+1e-8:return self.trip('invalid_command_clock')
        if c['state']=='aborted':
            self.seq=c['seq'];return self.trip('remote_abort')
        if self.expected is None:return self.trip('command_without_frame_lease')
        if (c['frame_id'],c['captured_sim_s'])!=self.expected:return self.trip('command_not_bound_to_current_frame')
        if sim_s-c['captured_sim_s']>.250001:return self.trip('stale_command')
        self.seq=c['seq'];self.expected=None;self.deadline=None;self.hold_deadline=wall+self.lease_s
        self.wheels=(float(c['v_m_s']),float(c['omega_rad_s']));self.reason='accepted'
        if c['state']=='visual_stop':return self.trip('remote_visual_stop')
        return self.wheels
