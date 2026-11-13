"""Independent landmark/guide-line baseline, NOT the original natural RGB model.

Inputs: graph/cardinal exits, ID labels, tag size, fresh native up/down RGB.
No scene, world coordinates, odometry, pose, simulator or audit imports.
"""
from __future__ import annotations
import math
import cv2
import numpy as np
from simulation.dijkstra import plan


def wrap(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi


def landmarks(rgb, labels, size=.32):
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), parameters)
    corners, ids, _ = detector.detectMarkers(rgb)
    found = {}
    if ids is None:
        return found
    for marker, quad in zip(ids.flatten(), corners):
        key = str(int(marker))
        if key not in labels or labels[key] in found:
            raise ValueError('unknown_or_duplicate_visual_marker')
        q = quad[0].astype(float)
        side = float(np.mean(np.linalg.norm(np.roll(q,-1,axis=0)-q,axis=1)))
        if side < 20:
            continue
        centre = q.mean(axis=0)
        edge = (q[1]-q[0] + q[2]-q[3])/2
        found[labels[key]] = dict(x=float((centre[1]-127.5)*size/side),
            y=float(-(centre[0]-127.5)*size/side), yaw=float(-math.atan2(edge[1],edge[0])))
    return found


def guide(rgb):
    """Near-field yellow painted strip only; branch decisions belong to tags."""
    r,g,b = rgb.astype(float).transpose(2,0,1)
    mask = (r > 110) & (g > 100) & (b < .55*g) & (r > .9*g)
    samples = []
    for row in range(48,145,8):
        xs = np.flatnonzero(mask[row])
        if 2 <= len(xs) <= 25:
            samples.append((row, float(np.median(xs))))
    if len(samples) < 5:
        return None
    rows, cols = np.array(samples).T
    slope, intercept = np.polyfit(rows,cols,1)
    return float((127.5-(slope*120+intercept))/128), float(math.atan(slope))


class LandmarkRouteController:
    def __init__(self, graph, labels, marker_size=.32):
        self.graph, self.labels, self.marker_size = graph, labels, marker_size
        self.route = plan(graph)
        self.nodes = {n['node_id']:n for n in graph['nodes']}
        self.state, self.phase, self.reason = 'observing', 'acquire', 'awaiting_start_tag'
        self.index = 0
        self.visited = []
        self.events = []
        self.stable = 0
        self.last_stamp = -1.
        self.last_id = None
        self.last_now = None
        self.seen = set()
        self.phase_start = None
        self.start = None
        self.heading = math.radians(graph['task']['initial_heading_deg'])
        self.motion_commands = 0

    def _phase(self, phase, now):
        self.phase, self.phase_start, self.stable = phase, now, 0

    def update(self, frame, now):
        if self.state in ('aborted','visual_stop'):
            return self._output()
        v, omega = 0., 0.
        try:
            required = {'frame_id','captured_sim_s','up_rgb','down_rgb'}
            if not isinstance(frame,dict) or set(frame) not in (required,required|{'rgb'}):
                raise ValueError('invalid_frame_envelope')
            stamp, identity = frame['captured_sim_s'], frame['frame_id']
            if type(now) not in (int,float) or not math.isfinite(now) or now < 0:
                raise ValueError('invalid_sim_time')
            if self.last_now is not None and now < self.last_now:
                raise ValueError('sim_time_regression')
            if type(stamp) not in (int,float) or not math.isfinite(stamp) or stamp < 0:
                raise ValueError('invalid_capture_time')
            if not isinstance(identity,str) or not identity or len(identity)>256:
                raise ValueError('invalid_frame_id')
            if not (0 <= now-stamp <= .250001 and stamp > self.last_stamp and identity not in self.seen):
                raise ValueError('stale_or_invalid_camera_frame')
            self.last_stamp, self.last_id, self.last_now = stamp, identity, now
            self.seen.add(identity)
            if self.start is None:
                self.start = self.phase_start = now
            if self.route['status'] != 'reachable':
                self.state, self.reason = 'aborted', 'goal_unreachable_no_motion'
            else:
                for key in ('up_rgb','down_rgb'):
                    if not isinstance(frame[key],np.ndarray) or frame[key].shape != (256,256,3) or frame[key].dtype != np.uint8:
                        raise ValueError('invalid_auxiliary_camera')
                observed = landmarks(frame['up_rgb'], self.labels, self.marker_size)
                current = self.route['nodes'][self.index]
                target = self.route['nodes'][self.index+1] if self.index+1 < len(self.route['nodes']) else None
                allowed = {current, target}
                if set(observed)-allowed:
                    raise ValueError('unexpected_node_marker')
                if now-self.start > 220 or now-self.phase_start > (30 if self.phase == 'cruise' else 22):
                    raise ValueError('visual_progress_timeout')
                self.state = 'observing'
                if self.phase == 'cruise' and target in observed:
                    self.index += 1
                    current = target
                    self._phase('dock',now)
                if self.phase in ('acquire','dock'):
                    if current not in observed:
                        raise ValueError('required_node_marker_missing')
                    tag = observed[current]
                    x,y = tag['x'],tag['y']
                    heading_error = wrap(self.heading-tag['yaw'])
                    if x < -.07 or abs(y) > .20:
                        raise ValueError('node_approach_outside_servo_domain')
                    if math.hypot(x,y) < .04 and abs(heading_error) < math.radians(4):
                        self.stable += 1
                        self.reason = 'fresh_stationary_node_confirmation'
                        if self.stable >= 3:
                            self.visited.append(current)
                            self.events.append(dict(node_id=current, frame_id=frame['frame_id'], captured_sim_s=stamp, event='visual_node_confirmed'))
                            if current == self.graph['task']['goal_node']:
                                self.state, self.reason = 'visual_stop', 'goal_visually_confirmed_stopped'
                            else:
                                next_node = self.route['nodes'][self.index+1]
                                exit_ = next(e for e in self.nodes[current]['exits'] if e['neighbor_id']==next_node)
                                self.heading = dict(east=0.,north=math.pi/2,west=math.pi,south=-math.pi/2)[exit_['direction']]
                                self._phase('align',now)
                                self.reason = 'topology_exit_selected'
                    else:
                        self.stable = 0
                        v = min(.10,max(0.,.8*(x-.018)))
                        omega = float(np.clip(1.6*heading_error + 1.8*math.atan2(y,max(x,.12)), -.30,.30))
                        self.reason = 'camera_relative_node_approach'
                elif self.phase == 'align':
                    if current not in observed:
                        raise ValueError('turn_marker_missing')
                    error = wrap(self.heading-observed[current]['yaw'])
                    if abs(error) < math.radians(2):
                        self.stable += 1
                        self.reason = 'turn_settling'
                        if self.stable >= 3:
                            self._phase('cruise',now)
                            self.reason = 'planned_exit_aligned'
                    else:
                        self.stable = 0
                        omega = float(np.clip(1.7*error,-.30,.30))
                        self.reason = 'visual_in_place_turn'
                elif self.phase == 'cruise':
                    if current in observed:
                        tag = observed[current]
                        omega = float(np.clip(1.8*wrap(self.heading-tag['yaw'])+1.5*tag['y'],-.30,.30))
                    else:
                        line = guide(frame['down_rgb'])
                        if line is None:
                            raise ValueError('guide_line_missing')
                        lateral, angle = line
                        omega = float(np.clip(1.4*lateral+1.8*angle,-.30,.30))
                    v = .12
                    self.reason = 'rgb_guide_following'
                if v or omega:
                    self.state = 'moving'
                    self.motion_commands += 1
        except (ValueError,KeyError,TypeError,OverflowError,cv2.error) as error:
            self.state, self.reason = 'aborted', str(error)
            self.stable = 0
            v = omega = 0.
        except Exception:
            self.state, self.reason = 'aborted', 'input_or_perception_exception'
            self.stable = 0
            raise
        return self._output(v,omega)

    def _output(self,v=0.,omega=0.):
        return dict(state=self.state, phase=self.phase, reason=self.reason, v_m_s=v, omega_rad_s=omega,
                    control_frame_id=self.last_id,captured_sim_s=self.last_stamp,decision_sim_s=self.last_now,
                    visited_nodes=list(self.visited),expected_node=self.route['nodes'][self.index] if self.route['nodes'] else None,
                    motion_commands=self.motion_commands)

    def cancel(self, now):
        if self.state not in ('visual_stop','aborted'):
            self.state,self.reason='aborted','operator_or_tick_limit_stop'

    def result(self):
        return dict(result_type='landmark_route_complete' if self.state=='visual_stop' else 'landmark_route_incomplete',
                    state=self.state,reason=self.reason,motion_commands=self.motion_commands,
                    visited_nodes=self.visited,node_events=self.events,planned_nodes=self.route['nodes'],
                    scope='new ceiling-tag and floor-guide RGB baseline; not original natural RGB/model acceptance')
