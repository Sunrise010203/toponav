"""Frozen 17-class model -> original topology localization/planning -> RGB servo.

Native four-view RGB is the ONLY measured input. No marker, down camera,
reference XY, simulator pose, wheel odometry or scene object. Bounded calibrated
blue-floor geometry is a separate low-level controller, not model retraining.
"""
from __future__ import annotations
import copy
import math
from pathlib import Path
import time
import numpy as np
from roadvision.contract import CLASS_IDS, to_observation
from roadvision.visual_corridor_geometry import (measurements, incoming_geometry,
    station_distance, front_wall_range, split_views, VIEW_OFFSETS, enclosed_side_walls)
from simulation.structural_localization import update_candidates, decide, map_exits
from simulation.topology import arrival_heading
from simulation.dijkstra import plan

MODEL_DIR=Path(__file__).resolve().parents[1]/'models/road_structure_frozen_v1_20261014'
HEIGHT=.8/3
WIDTH=.8
CAMERA_OFFSET=.18


def wrap(a):return (a+math.pi)%(2*math.pi)-math.pi


class FrozenModelPerception:
    """Reuse exact frozen weights/preprocessing/threshold, with native SIM time.

    Predictor.predict uses time.time_ns and cannot accept Gazebo simulation
    stamps. This adapter does not invent wall-clock capture times or retime RGB.
    """
    def __init__(self, run=MODEL_DIR):
        import torch
        from roadvision.learn import Predictor
        self.predictor=Predictor(Path(run),'cpu')
        self.metadata=self.predictor.metadata
        self.torch=torch

    def predict(self, frame, now):
        from roadvision.learn import prepare
        start=time.perf_counter()
        inputs,valid=prepare(frame['rgb'])
        with self.torch.no_grad():
            probabilities=self.predictor.model(inputs.unsqueeze(0)).softmax(1)[0].tolist()
        age=now-frame['captured_sim_s']
        observation=to_observation(probabilities,self.metadata['threshold'],valid,age)
        index=max(range(17),key=probabilities.__getitem__)
        accepted=observation['confidence']>0
        return dict(class_id=CLASS_IDS[index],score=probabilities[index],probabilities=probabilities,
                    accepted=accepted,observation=observation,
                    rejection_reason=None if accepted else 'missing_view' if not valid else 'unobservable' if index==16 else 'low_confidence_or_frame_age',
                    model_sha256=self.metadata['model_sha256'],frame_id=frame['frame_id'],
                    captured_sim_s=frame['captured_sim_s'],decision_sim_s=now,
                    clock_domain='original_simulation_capture',inference_ms=(time.perf_counter()-start)*1000)


class ModelRouteController:
    def __init__(self, graph, perception=None):
        self.graph=copy.deepcopy(graph)
        self.perception=perception if perception is not None else FrozenModelPerception()
        self.goal=graph['task']['goal_node']
        self.estimate=dict(node_id=graph['task']['start_node'],heading_deg=graph['task']['initial_heading_deg'])
        self.route=plan(graph)
        self.state,self.phase,self.reason='observing','confirm_node','awaiting_model_start_confirmation'
        self.last_stamp=-1.;self.last_id=None;self.started=None;self.phase_start=None
        self.last_now=None;self.seen=set()
        self.stable=0;self.motion_commands=0;self.visited=[];self.events=[]
        self.prediction=None;self.localization=None;self.planner=None
        self.edge_prior=None;self.edge_action=None;self.target=None;self.expected=None
        self.departed=False;self.departure_frames=0;self.turn_error=0.
        self.departure_source=None
        self.geometry={};self.station_error=None
        self.evidence_hold_start=None

    def _phase(self,phase,now):
        self.phase,self.phase_start,self.stable=phase,now,0
        self.evidence_hold_start=None

    def _matches(self, estimate):
        if not self.prediction['accepted']:return False
        exits=map_exits(self.graph,estimate)
        return all(v==(k in exits) for k,v in self.prediction['observation']['relative_open'].items())

    def _event(self,event,frame,**data):
        self.events.append(dict(event=event,event_id=frame['frame_id']+':'+event,
            frame_id=frame['frame_id'],captured_sim_s=frame['captured_sim_s'],**data))

    def _authorize(self,frame,now):
        if self.edge_prior is None:
            loc=update_candidates(self.graph,[self.estimate],self.prediction['observation'])
        else:
            loc=update_candidates(self.graph,[self.edge_prior],self.prediction['observation'],
                                  previous_action=self.edge_action,feedback={'status':'succeeded'})
        self.localization=loc
        if loc['status']!='localized' or (self.target and loc['estimate']['node_id']!=self.target):
            raise ValueError('model_localization_not_unique_or_wrong_transition')
        self.estimate=loc['estimate']
        node=self.estimate['node_id']
        self.visited.append(node)
        self._event('model_node_confirmed',frame,node_id=node,heading_deg=self.estimate['heading_deg'],
                    class_id=self.prediction['class_id'],score=self.prediction['score'])
        self.planner=decide(self.graph,loc,self.goal)
        if self.planner['status']=='reached':
            self.state,self.reason='visual_stop','model_goal_confirmed_stopped'
            return
        if self.planner['status']!='move':raise ValueError('public_planner_refused_motion')
        self.edge_prior=copy.deepcopy(self.estimate)
        self.edge_action=self.planner['action']
        self.target=self.planner['route']['nodes'][1]
        self.expected=dict(node_id=self.target,heading_deg=arrival_heading(self.graph,node,self.target))
        self.turn_error=math.radians(dict(straight=0,left=90,right=-90,uturn=180)[self.edge_action])
        self._event('model_authorized_exit',frame,node_id=node,action=self.edge_action,target_node=self.target,
                    class_id=self.prediction['class_id'],observation=self.prediction['observation'])
        self._phase('align',now);self.reason='model_localized_public_planner_exit'

    def _turn_geometry(self):
        candidates=[]
        for view,m in self.geometry.items():
            if m is None or abs(m['offset'])>.18:continue
            for flip in (0,math.pi):
                angle=wrap(VIEW_OFFSETS[view]+m['angle']+flip)
                delta=abs(wrap(angle-self.turn_error))
                if delta<math.radians(18):
                    candidates.append((delta, -m['support'],angle))
        if not candidates:return None
        return min(candidates)[2]

    def update(self,frame,now):
        # Terminal decisions are immutable, including their last camera/time
        # binding. A later malformed packet must not escape or rewrite them.
        if self.state in ('aborted','visual_stop'):return self._output()
        v=omega=0.
        try:
            if not isinstance(frame,dict) or set(frame)!={'frame_id','captured_sim_s','rgb'}:
                raise ValueError('invalid_frame_envelope')
            stamp=frame['captured_sim_s'];identity=frame['frame_id']
            if type(now) not in (int,float) or not math.isfinite(now) or now<0:
                raise ValueError('invalid_sim_time')
            if self.last_now is not None and now<self.last_now:
                raise ValueError('sim_time_regression')
            if type(stamp) not in (int,float) or not math.isfinite(stamp) or stamp<0:
                raise ValueError('invalid_capture_time')
            if not isinstance(identity,str) or not identity or len(identity)>256:
                raise ValueError('invalid_frame_id')
            if not (0<=now-stamp<=.250001 and stamp>self.last_stamp and identity not in self.seen):
                raise ValueError('stale_or_invalid_original_camera_frame')
            self.last_stamp,self.last_id,self.last_now=stamp,identity,now
            self.seen.add(identity)
            if self.started is None:self.started=self.phase_start=now
            if self.route['status']!='reachable':
                raise ValueError('goal_unreachable_no_motion')
            else:
                if any(not pixels.any() for pixels in split_views(frame['rgb']).values()):
                    raise ValueError('missing_original_rgb_view')
                self.prediction=self.perception.predict(frame,now)
                self.geometry=measurements(frame['rgb'],HEIGHT,WIDTH,CAMERA_OFFSET)
                incoming=incoming_geometry(self.geometry)
                self.state='observing'
                limit=35 if self.phase in ('cruise','dock') else 22 if self.phase=='align' else 5
                if now-self.started>220 or now-self.phase_start>limit:
                    raise ValueError('model_or_visual_progress_timeout')
                if self.phase=='confirm_node':
                    candidate=self.estimate if self.expected is None else self.expected
                    if self._matches(candidate):
                        self.stable+=1;self.reason='fresh_zero_model_node_confirmation'
                        if self.stable>=3:self._authorize(frame,now)
                    else:
                        self.stable=0;self.reason='model_uncertain_or_topology_mismatch_hold'
                elif self.phase=='align':
                    angle=self._turn_geometry()
                    if angle is None:raise ValueError('visual_turn_axis_unobservable')
                    self.turn_error=angle
                    if abs(angle)<math.radians(3):
                        turned=dict(node_id=self.estimate['node_id'],heading_deg=self.expected['heading_deg'])
                        if self._matches(turned):
                            self.stable+=1;self.reason='model_and_rgb_turn_confirmation'
                            if self.stable>=3:
                                self._phase('cruise',now);self.departed=False;self.departure_frames=0;self.departure_source=None
                                self.reason='model_confirmed_planned_exit_aligned'
                        else:
                            self.stable=0;self.reason='model_turn_signature_mismatch_hold'
                    else:
                        self.stable=0;omega=float(np.clip(1.4*angle,-.30,.30));self.reason='rgb_axis_in_place_turn'
                elif self.phase in ('cruise','dock'):
                    if incoming is None:raise ValueError('incoming_corridor_geometry_missing')
                    omega=float(np.clip(1.7*incoming['angle']+1.7*incoming['offset'],-.30,.30))
                    expected_open={k:k in map_exits(self.graph,self.expected) for k in VIEW_OFFSETS}
                    distance,source=station_distance(self.geometry,expected_open,split_views(frame['rgb'])['front'],HEIGHT,WIDTH,CAMERA_OFFSET)
                    self.station_error=distance
                    straight=self.prediction['accepted'] and self.prediction['class_id']=='exits_1001'
                    wall_enclosed=enclosed_side_walls(frame['rgb'])
                    if straight or wall_enclosed:
                        self.departure_frames+=1
                        if self.departure_frames>=3 and not self.departed:
                            self.departed=True
                            self.departure_source='accepted_model_straight' if straight else 'rgb_both_side_walls_enclosed'
                            self._event('visual_departure_confirmed',frame,source=self.departure_source)
                    else:self.departure_frames=0
                    if self.phase=='cruise':
                        v=.12;self.reason='rgb_corridor_follow_model_transit'
                        if self.departed and self._matches(self.expected):
                            if distance is None:
                                v=omega=0.;self.reason='model_target_seen_geometry_unconfirmed_hold'
                            elif -.10<distance<.65:
                                self._phase('dock',now);v=omega=0.;self.reason='model_target_and_rgb_station_acquired'
                            elif distance<=-.10:
                                raise ValueError('visual_station_overshoot_before_model_confirmation')
                        if self.departed and distance is not None and -.10<distance<.17 and not self._matches(self.expected):
                            v=omega=0.;self.reason='junction_guard_without_confirmed_model_hold'
                        cap=front_wall_range(split_views(frame['rgb'])['front'],HEIGHT,CAMERA_OFFSET)
                        if cap is not None and cap<.52 and self.phase=='cruise':
                            v=omega=0.;self.reason='wall_guard_without_confirmed_model_station_hold'
                    else:
                        if not self._matches(self.expected):
                            self.stable=0;v=omega=0.;self.reason='model_target_lost_hold'
                        elif distance is None:
                            self.stable=0;v=omega=0.;self.reason='rgb_station_geometry_lost_hold'
                        elif distance<-.10:raise ValueError('visual_station_overshoot')
                        elif abs(distance)<.035 and abs(incoming['offset'])<.07 and abs(incoming['angle'])<math.radians(5):
                            v=omega=0.;self.stable+=1;self.reason='visual_station_zero_settling'
                            if self.stable>=3:
                                self._event('visual_edge_station_stopped',frame,target_node=self.target,station_source=source)
                                self._phase('confirm_node',now)
                        else:
                            self.stable=0;v=min(.09,max(0.,.8*(distance-.012)))
                            self.reason='rgb_station_approach_with_model_gate'
                if self.reason.endswith('_hold'):
                    if self.evidence_hold_start is None:self.evidence_hold_start=now
                    if now-self.evidence_hold_start>4.:
                        raise ValueError('model_evidence_hold_timeout')
                else:self.evidence_hold_start=None
                if v or omega:
                    self.state='moving';self.motion_commands+=1
        except (ValueError,KeyError,TypeError,RuntimeError,OverflowError) as error:
            self.state,self.reason='aborted',str(error);self.stable=0;v=omega=0.
        except Exception:
            self.state,self.reason='aborted','input_or_perception_exception';self.stable=0
            raise
        return self._output(v,omega)

    def _output(self,v=0.,omega=0.):
        return dict(state=self.state,phase=self.phase,reason=self.reason,v_m_s=v,omega_rad_s=omega,
                    control_frame_id=self.last_id,captured_sim_s=self.last_stamp,decision_sim_s=self.last_now,
                    visited_nodes=list(self.visited),expected_node=self.target,motion_commands=self.motion_commands,
                    model=copy.deepcopy(self.prediction),localization=copy.deepcopy(self.localization),
                    planner=copy.deepcopy(self.planner),visual_geometry=copy.deepcopy(self.geometry),
                    station_error_m=self.station_error,departure_visually_confirmed=self.departed,
                    departure_source=self.departure_source)

    def cancel(self,now):
        if self.state not in ('aborted','visual_stop'):self.state,self.reason='aborted','operator_or_tick_limit_stop'

    def result(self):
        return dict(result_type='frozen_model_route_complete' if self.state=='visual_stop' else 'frozen_model_route_incomplete',
                    state=self.state,reason=self.reason,motion_commands=self.motion_commands,visited_nodes=self.visited,
                    node_events=[e for e in self.events if e['event']=='model_node_confirmed'],events=self.events,
                    model_sha256=self.perception.metadata['model_sha256'],planned_nodes=self.route['nodes'],
                    scope='frozen 17-class model + original localization/planner + new calibrated floor-geometry servo; no tags/paint/pose; not general-vision or formal S2/S6 acceptance')
