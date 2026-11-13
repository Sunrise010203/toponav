"""Gazebo / DART cross-check: unchanged local primitives or new assisted route.

Native Gazebo transport, not a ROS-Humble/Gazebo-Harmonic compatibility claim.
Pose and contacts are audit-only. No PyBullet import, pose reset or truth steering.
"""
from __future__ import annotations

import argparse
from collections import defaultdict, deque
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import threading
import time
import traceback
import uuid
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image as PNG

ROOT = Path(__file__).resolve().parents[1]
VIEWS = ("front", "left", "right", "back")
STEP_HELPER_TIMEOUT_S = 30.
MAX_GAZEBO_POSES = 65536  # Native robot-pose prefix, never a rolling window.
MAX_POSE_NOT_RETAINED = 2**64-1  # Saturating UNVALIDATED robot-entry count.
MAX_GAZEBO_CONTACTS = 65536  # Observed wall-contact prefix, never a rolling window.
MAX_CONTACT_NOT_RETAINED = 2**64-1  # UNVALIDATED entries, NOT known wall contacts.
MAX_GAZEBO_IMAGE_WIDTH = 768  # Largest declared observer, not driver input.
MAX_GAZEBO_IMAGE_HEIGHT = 448


def tag(parent, element, value=None, **attrs):
    item = ET.SubElement(parent, element, attrs)
    if value is not None:
        item.text = str(value)
    return item


def geometry(parent, shape, size):
    g = tag(parent, "geometry")
    tag(tag(g, shape), "size" if shape == "box" else "radius", size)


def box(world, name, size, xyz, colour):
    model = tag(world, "model", name=name)
    tag(model, "static", "true")
    tag(model, "pose", " ".join(map(str, (*xyz, 0, 0, 0))))
    link = tag(model, "link", name="link")
    geometry(tag(link, "collision", name="collision"), "box", " ".join(map(str, size)))
    visual = tag(link, "visual", name="visual")
    geometry(visual, "box", " ".join(map(str, size)))
    material = tag(visual, "material")
    for field in ("ambient", "diffuse"):
        tag(material, field, " ".join(map(str, (*colour, 1))))
    tag(material, "specular", "0 0 0 1")


def camera(link, name, pose, topic, width=128, fov=math.pi / 2):
    sensor = tag(link, "sensor", name=name, type="camera")
    tag(sensor, "pose", pose)
    tag(sensor, "always_on", "true")
    tag(sensor, "update_rate", 10)
    tag(sensor, "topic", topic)
    cam = tag(sensor, "camera")
    tag(cam, "horizontal_fov", fov)
    image = tag(cam, "image")
    tag(image, "width", width)
    tag(image, "height", width)
    tag(image, "format", "R8G8B8")
    clip = tag(cam, "clip")
    tag(clip, "near", .05)
    tag(clip, "far", 30)


def build_world(direction="left"):
    if direction not in ("left", "right"):
        raise ValueError("Only declared left/right L-corner fixtures are supported")
    sign = 1 if direction == "left" else -1
    sdf = ET.Element("sdf", version="1.9")
    world = tag(sdf, "world", name="toponav")
    physics = tag(world, "physics", name="dart_120hz", type="ignored")
    tag(physics, "max_step_size", 1 / 120)
    tag(physics, "real_time_factor", 0)
    tag(world, "gravity", "0 0 -9.81")
    scene = tag(world, "scene")
    tag(scene, "ambient", "1 1 1 1")
    tag(scene, "background", "0.7 0.75 0.8 1")
    tag(scene, "shadows", "false")
    for filename, name in (("physics", "Physics"), ("user-commands", "UserCommands"),
                           ("scene-broadcaster", "SceneBroadcaster"), ("contact", "Contact")):
        plugin = tag(world, "plugin", filename=f"gz-sim-{filename}-system",
                     name=f"gz::sim::systems::{name}")
    sensors = tag(world, "plugin", filename="gz-sim-sensors-system",
                  name="gz::sim::systems::Sensors")
    tag(sensors, "render_engine", "ogre2")
    light = tag(world, "light", name="sun", type="directional")
    tag(light, "pose", "0 0 5 0 0 0")
    tag(light, "diffuse", "0.7 0.7 0.7 1")
    tag(light, "direction", "1 -1 -3")
    # Same declared L fixture dimensions as the existing PyBullet default.
    box(world, "floor_entry", (3.6, 1.8, .1), (1.6, 0, -.05), (.20, .29, .40))
    box(world, "floor_exit", (1.8, 2.6, .1), (2.5, sign * 2.2, -.05), (.20, .29, .40))
    walls = [((3.7, .1, 1.6), (1.6, -.95, .8)),
             ((1.8, .1, 1.6), (.7, .95, .8)),
             ((.1, 4.5, 1.6), (3.45, 1.3, .8)),
             ((.1, 2.6, 1.6), (1.55, 2.2, .8)),
             ((1.8, .1, 1.6), (2.5, 3.55, .8)),
             ((.1, 1.8, 1.6), (-.25, 0, .8))]
    for index, (size, (x, y, z)) in enumerate(walls):
        box(world, f"wall_{index}", size, (x, sign * y, z), (.65, .62, .58))
    robot = tag(world, "model", name="robot")
    tag(robot, "pose", ".5 0 .16 0 0 0")
    for name, xyz, shape, mass in (("base", (0, 0, 0), "box", 1.),
              ("left_wheel", (0, .15, -.08), "cylinder", .1),
              ("right_wheel", (0, -.15, -.08), "cylinder", .1),
              ("front_caster", (.13, 0, -.13), "sphere", .01),
              ("rear_caster", (-.13, 0, -.13), "sphere", .01)):
        link = tag(robot, "link", name=name)
        tag(link, "pose", " ".join(map(str, (*xyz, 0, 0, 0))))
        inertial = tag(link, "inertial")
        tag(inertial, "mass", mass)
        inertia = tag(inertial, "inertia")
        moments = ((.22**2+.12**2)/12, (.32**2+.12**2)/12, (.32**2+.22**2)/12) if name == "base" else \
                  (.1*(3*.08**2+.04**2)/12, .1*.08**2/2, .1*(3*.08**2+.04**2)/12) if shape == "cylinder" else (.0000036,)*3
        for field, moment in zip(("ixx", "iyy", "izz"), moments):
            tag(inertia, field, moment)
        for kind in ("collision", "visual"):
            item = tag(link, kind, name=name + "_" + kind)
            if shape == "cylinder":
                tag(item, "pose", f"0 0 0 {math.pi / 2} 0 0")
            g = tag(tag(item, "geometry"), shape)
            if shape == "box":
                tag(g, "size", ".32 .22 .12")
            else:
                tag(g, "radius", .08 if shape == "cylinder" else .03)
                if shape == "cylinder":
                    tag(g, "length", .04)
            if kind == "visual":
                mat = tag(item, "material")
                for field in ("ambient", "diffuse"):
                    tag(mat, field, "0.15 0.18 0.20 1")
            else:
                ode = tag(tag(tag(item, "surface"), "friction"), "ode")
                mu = .001 if shape == "sphere" else 1.5 if shape == "cylinder" else .5
                tag(ode, "mu", mu)
                tag(ode, "mu2", mu)
        contact = tag(link, "sensor", name=name + "_contact", type="contact")
        tag(contact, "always_on", "true")
        tag(contact, "update_rate", 120)
        tag(tag(contact, "contact"), "collision", name + "_collision")
        if name == "base":
            for view, yaw in zip(VIEWS, (0, math.pi / 2, -math.pi / 2, math.pi)):
                camera(link, view, f".2 0 .29 0 0 {yaw}", "/toponav/camera/" + view)
        else:
            joint = tag(robot, "joint", name=name + "_joint",
                        type="revolute" if shape == "cylinder" else "fixed")
            tag(joint, "parent", "base")
            tag(joint, "child", name)
            if shape == "cylinder":
                axis = tag(joint, "axis")
                tag(axis, "xyz", "0 1 0")
                limit = tag(axis, "limit")
                tag(limit, "effort", 1.5)
    drive = tag(robot, "plugin", filename="gz-sim-diff-drive-system",
                name="gz::sim::systems::DiffDrive")
    for field, value in (("left_joint", "left_wheel_joint"), ("right_joint", "right_wheel_joint"),
             ("wheel_separation", .30), ("wheel_radius", .08), ("topic", "/toponav/cmd_vel")):
        tag(drive, field, value)
    pose = tag(robot, "plugin", filename="gz-sim-pose-publisher-system",
               name="gz::sim::systems::PosePublisher")
    for field, value in (("publish_model_pose", "true"), ("publish_link_pose", "false"),
             ("use_pose_vector_msg", "true"), ("update_frequency", 120)):
        tag(pose, field, value)
    observer = tag(world, "model", name="observer")
    tag(observer, "static", "true")
    camera(tag(observer, "link", name="mount"), "overview",
           f"1.5 {sign * 1.2} 7 0 {math.pi / 2} {math.pi / 2}",
           "/toponav/overview", width=384, fov=1.1)
    ET.indent(sdf)
    return ET.tostring(sdf, encoding="unicode")


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nsec


class CameraBuffer:
    """All subscribed cameras must have identical original capture timestamps."""
    def __init__(self, extra_views=()):
        self.views = {view: {} for view in (*VIEWS, *extra_views)}

    def put(self, view, timestamp, pixels):
        self.views[view][timestamp] = pixels
        for old in sorted(self.views[view])[:-8]:
            del self.views[view][old]

    def latest(self, after=-1, before=None):
        common = set.intersection(*(set(frames) for frames in self.views.values()))
        common = [t for t in common if t > after and (before is None or t <= before)]
        if not common:
            return None
        timestamp = max(common)
        rgb = np.concatenate([np.concatenate([self.views[v][timestamp] for v in VIEWS[:2]], axis=1),
                              np.concatenate([self.views[v][timestamp] for v in VIEWS[2:]], axis=1)], axis=0)
        return timestamp, rgb


def native_imports():
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    from gz.msgs10.world_control_pb2 import WorldControl
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.twist_pb2 import Twist
    return Node, Image, Pose_V, Contacts, WorldStatistics, WorldControl, Boolean, Twist


class StepServiceClient:
    """Bounded pipe replies; never reuse an ACK after an uncertain request.

    The helper still performs the native synchronous service call away from
    camera callbacks. This is NOT an independent motor lease watchdog.
    """
    def __init__(self, process, timeout_s=None):
        self.process = process
        self.timeout_s = STEP_HELPER_TIMEOUT_S if timeout_s is None else timeout_s
        self.failed = False
        self.buffer = b''

    def receive(self, field):
        if self.failed:
            raise RuntimeError('Gazebo step helper unavailable after prior failure')
        deadline = time.monotonic() + self.timeout_s
        try:
            with selectors.DefaultSelector() as selector:
                selector.register(self.process.stdout, selectors.EVENT_READ)
                while b'\n' not in self.buffer:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0 or not selector.select(remaining):
                        raise TimeoutError('Gazebo step helper reply timed out: ' + field)
                    # Read bytes, not TextIO.readline(): fd readiness may
                    # mean only a partial line. A second readline would block
                    # beyond the deadline; a text buffer may hide ready bytes.
                    chunk = os.read(self.process.stdout.fileno(), 4096)
                    if not chunk:
                        raise RuntimeError('Gazebo step helper closed its output before reply: ' + field)
                    self.buffer += chunk
            line, self.buffer = self.buffer.split(b'\n', 1)
            try:
                reply = json.loads(line)
            except (ValueError, UnicodeError) as exc:
                raise RuntimeError('Gazebo step helper malformed JSON reply') from exc
            if type(reply) is not dict or reply.get(field) is not True:
                raise RuntimeError('Gazebo step helper rejected or malformed reply: ' + str(reply))
            return reply
        except BaseException:
            self.failed = True
            raise

    def request(self, iterations):
        if self.failed:
            raise RuntimeError('Gazebo step helper unavailable after prior failure')
        # Same finite budget as the helper. Only one small request can be
        # outstanding; failed exchanges cannot fill the pipe with retries.
        if type(iterations) is not int or not 1 <= iterations <= 120:
            raise ValueError('Invalid finite step budget')
        try:
            self.process.stdin.write(json.dumps({'iterations': iterations}) + '\n')
            self.process.stdin.flush()
            return self.receive('ok')
        except BaseException:
            self.failed = True
            # A failed flush can leave bytes buffered in TextIO. Closing the
            # same pipe later would flush again and mask the primary error
            # during run() cleanup, preventing result.json from being saved.
            try:
                self.process.stdin.close()
            except OSError:
                pass  # Preserve the original failed exchange, not its reflush.
            raise


def start_step_service():
    process = subprocess.Popen([sys.executable, '-u', '-m', 'simulation.gazebo_step_service'],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    client = StepServiceClient(process)
    try:
        client.receive('ready')
    except BaseException:
        # GazeboIO construction has not returned: run().io is still None and
        # cannot clean this child. Reap only the helper that we just created.
        process.kill()
        process.communicate(timeout=5)
        raise
    return client


class GazeboIO:
    def __init__(self, route=False, session_id=None, actuator_audit=None):
        from simulation.gazebo_receiver import ReceiverEvidence, verify_binary
        verify_binary()  # Never silently fall back to an unprotected drive.
        self.receiver_evidence = ReceiverEvidence(session_id, bind_submissions=True, require_frame_echo=True)
        Node, Image, Pose_V, Contacts, Stats, Control, Boolean, Twist = native_imports()
        self.Control, self.Boolean, self.Twist = Control, Boolean, Twist
        self.node = Node()
        self.condition = threading.Condition()
        self.cameras = CameraBuffer(("up", "down") if route else ())
        self.now_ns = 0
        self.poses = []
        self._pose_not_retained = 0
        self.contacts = []
        self._contact_not_retained = 0
        self.contact_messages = defaultdict(int)
        self.overview = None
        self.error = None
        self.clock_messages = 0
        self.callbacks = []
        from gz.msgs10.stringmsg_pb2 import StringMsg
        if not self.node.subscribe(StringMsg, '/toponav/receiver_status', self.receiver_status):
            raise RuntimeError('Could not subscribe to native receiver interlock')
        for view in self.cameras.views:
            cb = lambda msg, v=view: self.image(v, msg)
            self.callbacks.append(cb)
            if not self.node.subscribe(Image, "/toponav/camera/" + view, cb):
                raise RuntimeError("Could not subscribe to " + view)
        if not self.node.subscribe(Image, "/toponav/overview", self.overview_image):
            raise RuntimeError("Could not subscribe to /toponav/overview")
        if not self.node.subscribe(Stats, "/world/toponav/stats", self.stats):
            raise RuntimeError("Could not subscribe to /world/toponav/stats")
        from gz.msgs10.clock_pb2 import Clock
        if not self.node.subscribe(Clock, "/world/toponav/clock", self.clock):
            raise RuntimeError("Could not subscribe to /world/toponav/clock")
        if not self.node.subscribe(Pose_V, "/model/robot/pose", self.pose):
            raise RuntimeError("Could not subscribe to /model/robot/pose")
        for link in ("base", "left_wheel", "right_wheel", "front_caster", "rear_caster"):
            cb = lambda msg, name=link: self.contact(name, msg)
            self.callbacks.append(cb)
            topic = f"/world/toponav/model/robot/link/{link}/sensor/{link}_contact/contact"
            if not self.node.subscribe(Contacts, topic, cb):
                raise RuntimeError("Could not subscribe to " + topic)
        self.display_type = Image
        self.display_publisher = self.node.advertise("/toponav/topology_display", Image)
        from simulation.gazebo_actuator import ActuatorClient
        self.actuator = ActuatorClient(session_id, actuator_audit,
                                       submission_observer=self._record_actuator_submission)
        self.command_context = None
        try:
            self._step_service = start_step_service()
        except BaseException:
            self.actuator.close()  # No motion command exists during construction.
            raise
        self.stepper = self._step_service.process

    def receiver_status(self, message):
        with self.condition:
            self.receiver_evidence.push(message.data)
            self.condition.notify_all()

    def _record_actuator_submission(self, packet, sent_steady_s):
        with self.condition:
            self.receiver_evidence.record_submission(packet, sent_steady_s)

    @staticmethod
    def pixels(msg, *, expected_sizes=None):
        # Reject metadata BEFORE fetching payload or creating a NumPy view/copy.
        # Transport/protobuf already received the message; this is not a wire bound.
        width, height, step = msg.width, msg.height, msg.step
        if (type(width) is not int or type(height) is not int
                or not 0 < width <= MAX_GAZEBO_IMAGE_WIDTH
                or not 0 < height <= MAX_GAZEBO_IMAGE_HEIGHT
                or (expected_sizes is not None and (width, height) not in expected_sizes)):
            raise ValueError("Unexpected camera size")
        value = msg.DESCRIPTOR.fields_by_name["pixel_format_type"].enum_type.values_by_number.get(msg.pixel_format_type)
        if value is None or value.name != "RGB_INT8" or type(step) is not int or step != width * 3:
            raise ValueError("Gazebo camera must provide packed RGB_INT8")
        data = msg.data
        if len(data) != height * step:
            raise ValueError("Gazebo camera must provide packed RGB_INT8")
        return np.frombuffer(data, np.uint8).reshape(height, width, 3).copy()

    def image(self, view, msg):
        with self.condition:
            try:
                if view not in self.cameras.views:
                    raise ValueError("Unknown camera view")
                rgb = self.pixels(msg, expected_sizes=((256, 256),) if view in ("up", "down") else ((128, 128),))
                self.cameras.put(view, stamp_ns(msg.header), rgb)
            except Exception as error:
                if self.error is None:
                    self.error = str(error) or type(error).__name__
            self.condition.notify_all()

    def overview_image(self, msg):
        with self.condition:
            try:
                rgb = self.pixels(msg, expected_sizes=((384, 384), (768, 448)))
                self.overview = (stamp_ns(msg.header), rgb)
            except Exception as error:
                self.overview = None  # Never keep a stale observer image after failure.
                if self.error is None:
                    self.error = str(error) or type(error).__name__
            self.condition.notify_all()

    def stats(self, msg):
        with self.condition:
            self.now_ns = max(self.now_ns, msg.sim_time.sec * 1_000_000_000 + msg.sim_time.nsec)
            self.condition.notify_all()

    def clock(self, msg):
        with self.condition:
            self.now_ns = max(self.now_ns, msg.sim.sec * 1_000_000_000 + msg.sim.nsec)
            self.clock_messages += 1
            self.condition.notify_all()

    def pose(self, msg):
        with self.condition:
            for p in msg.pose:
                if p.name != "robot":
                    continue
                if len(self.poses) >= MAX_GAZEBO_POSES:
                    self._pose_not_retained = min(self._pose_not_retained+1, MAX_POSE_NOT_RETAINED)
                    if self.error is None:
                        self.error = 'Gazebo pose history capacity exceeded'
                    continue  # Not parsed/copied or usable as fresh physical evidence.
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self.poses.append(dict(sim_s=stamp_ns(p.header) / 1e9,
                    position_m=[p.position.x, p.position.y, p.position.z], yaw_deg=math.degrees(yaw)))
            self.condition.notify_all()

    def contact(self, link, msg):
        with self.condition:
            self.contact_messages[link] += 1
            for c in msg.contact:
                if len(self.contacts) >= MAX_GAZEBO_CONTACTS:
                    self._contact_not_retained = min(self._contact_not_retained+1, MAX_CONTACT_NOT_RETAINED)
                    if self.error is None:
                        self.error = 'Gazebo contact history capacity exceeded'
                    continue  # Do not inspect names/header or classify omitted entries.
                if "wall_" in c.collision1.name or "wall_" in c.collision2.name:
                    self.contacts.append(dict(link=link, sim_s=stamp_ns(msg.header) / 1e9,
                        collision1=c.collision1.name, collision2=c.collision2.name))
            self.condition.notify_all()

    def command(self, v, omega):
        if not (0 <= v <= .15 and abs(omega) <= .30):
            raise ValueError("Command outside frozen controller limits")
        context, self.command_context = self.command_context, None
        # Serialize send+record against status callbacks, including a status
        # that arrives before the IPC ACK. Never defer an unknown status and
        # later bless it merely because a matching command was sent afterward.
        with self.condition:
            if v or omega:
                if getattr(self, '_contact_not_retained', 0):
                    raise RuntimeError('Gazebo contact history capacity exceeded')
                if getattr(self, '_pose_not_retained', 0):
                    raise RuntimeError('Gazebo pose history capacity exceeded')
                if getattr(self, 'error', None) is not None:
                    raise RuntimeError('Gazebo I/O callback failure: '+str(self.error))
                self.receiver_evidence.require_fresh(time.monotonic())
            if context is None:
                if v or omega:
                    self.actuator.request('stop')
                    raise ValueError('Motion requires a bound actuator lease')
                self.actuator.request('stop')
            elif context == {'idle': True} and v == omega == 0:
                self.actuator.request('idle')
            else:
                command = context['command']
                if (v, omega) != (command['v_m_s'], command['omega_rad_s']):
                    self.actuator.request('stop')
                    raise ValueError('Actuator velocity differs from validated command')
                self.actuator.request('command', **context)

    def refresh_disarmed_receiver(self):
        """One zero-authority startup observation after the GUI start wait.

        Paused disarmed worlds do not emit periodic status. Only the native
        and independently ACKed never-commanded state may be stepped here;
        this is not an active-lease refresh, fault reset or re-arm path.
        """
        with self.condition:
            self.receiver_evidence.require_ready()
            native = self.receiver_evidence.events[-1]
            ack = self.actuator.status
            if any(item['latched'] or item['reason'] != 'disarmed'
                   or item['command_seq'] != 0 or item['wheels'] != [0., 0.]
                   for item in (native, ack)):
                raise RuntimeError('Receiver startup observation requires never-commanded disarmed authority')
        self.step(12)
        time.sleep(.1)
        with self.condition:
            self.receiver_evidence.require_fresh(time.monotonic())

    def publish_display(self, rgb):
        msg = self.display_type()
        msg.width, msg.height = rgb.shape[1], rgb.shape[0]
        msg.step = msg.width * 3
        msg.pixel_format_type = 3  # gz.msgs.PixelFormatType.RGB_INT8
        msg.data = rgb.tobytes()
        msg.header.stamp.sec, msg.header.stamp.nsec = divmod(self.now_ns, 1_000_000_000)
        self.display_publisher.publish(msg)

    def step(self, iterations=12):
        before = self.now_ns
        try:
            self._step_service.request(iterations)
            # Service success acknowledges a QUEUED request, not completed physics.
            # Never feed the empty creation-time render to the RGB controller.
            deadline = time.monotonic() + 30
            target = before + round(iterations * 1e9 / 120) - 1000
            with self.condition:
                while self.now_ns < target:
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f"Gazebo acknowledged step but clock did not advance: {before} -> {self.now_ns}, target={target}")
                    self.condition.wait(.05)
        except BaseException:
            # Do not queue more physics after an unknown completion outcome,
            # including cleanup: the zero submission is independent of this.
            self._step_service.failed = True
            raise

    def next_frame(self, previous, timeout=30):
        deadline = time.monotonic() + timeout
        with self.condition:
            while True:
                if self.error is not None:
                    raise RuntimeError(self.error)
                frame = self.cameras.latest(after=previous, before=self.now_ns)
                if frame is not None:
                    return frame[0], frame[1], self.now_ns / 1e9
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("No synchronized original-stamp Gazebo RGB camera set; "
                                       f"sim_ns={self.now_ns}, views=" + str({v:list(x) for v,x in self.cameras.views.items()}))
                self.condition.wait(min(remaining, .1))


def _observe_failed_actuator_stop(io):
    """Observe an already closed actuator, never repair/re-arm its channel.

    Fresh native zero authority is a prerequisite for the finite physics
    observations, not physical-stop proof. Keep this evidence separate from
    the failed zero request, original exception and task completion gate.
    """
    from simulation.gazebo_actuator import LEASE_S
    from ros2_ws.src.toponav_bridge.toponav_bridge.protocol import number
    record = dict(attempted=True, authority_confirmed=False, observation_complete=False,
        stationary=False, error=None, wait_limit_s=LEASE_S, started_steady_s=None,
        authority=None, steps_requested=10, steps_completed=0, pose_samples_before=None,
        last_step_started_sim_s=None, final_sim_s=None, post_stop_pose_pair_fresh=False,
        final_pose_pair=None, final_speed_m_s=None, final_yaw_rate_rad_s=None,
        receiver_status=None,
        scope='Independent failed-channel zero-authority/pose observation; NOT a cleanup ACK, task success or hardware certification')
    try:
        actuator = io.actuator
        if actuator.failed is not True or actuator.channel.fileno() != -1:
            raise RuntimeError('Stop observation requires an irreversibly closed actuator channel')
        minimum_seq = actuator.status['command_seq']
        if type(minimum_seq) is not int or minimum_seq < 0:
            raise RuntimeError('Stop observation has no validated prior command sequence')
        previous_wall = time.monotonic()
        if not number(previous_wall) or previous_wall < 0:
            raise RuntimeError('Invalid stop observation clock')
        record['started_steady_s'] = previous_wall
        end = previous_wall+LEASE_S

        def zero_status():
            nonlocal previous_wall
            if getattr(io, '_pose_not_retained', 0):
                raise RuntimeError('Stop observation pose history capacity exceeded')
            now = time.monotonic()
            if not number(now) or now < previous_wall:
                raise RuntimeError('Stop observation clock regressed')
            previous_wall = now
            # Never reset a helper's sticky failure or queue another request
            # after an unknown outcome, including during this cleanup path.
            if io._step_service.failed is not False or io._step_service.process.poll() is not None:
                raise RuntimeError('Stop observation step helper unavailable or outcome unknown')
            evidence = io.receiver_evidence
            if evidence.error:
                raise RuntimeError('Stop observation receiver audit failed: '+str(evidence.error))
            final = evidence.events[-1] if evidence.events else None
            if final and final['wall_s'] > now:
                raise RuntimeError('Stop observation receiver status is from the future')
            if (final and final['latched'] and final['wheels'] == [0., 0.]
                    and final['command_seq'] >= minimum_seq and 0 <= now-final['wall_s'] < LEASE_S):
                return final
            return None

        with io.condition:
            while True:
                final = zero_status()
                remaining = end-previous_wall
                if remaining <= 0:
                    raise TimeoutError('No fresh native zero authority within original lease budget')
                if final is not None:
                    record['authority'] = copy.deepcopy(final)
                    record['authority_confirmed'] = True
                    break
                io.condition.wait(min(.02, remaining))
            record['pose_samples_before'] = len(io.poses)
        for _ in range(record['steps_requested']):
            with io.condition:
                if zero_status() is None:
                    raise RuntimeError('Stop observation lost fresh native zero authority')
                if not number(io.now_ns) or io.now_ns < 0:
                    raise RuntimeError('Invalid stop observation simulation clock')
                record['last_step_started_sim_s'] = io.now_ns/1e9
                last_pose_count = len(io.poses)
            io.step(12)
            record['steps_completed'] += 1
            time.sleep(.03)
        with io.condition:
            if zero_status() is None:
                raise RuntimeError('Stop observation ended without fresh native zero authority')
            if not number(io.now_ns) or io.now_ns < 0:
                raise RuntimeError('Invalid final stop observation simulation clock')
            record['final_sim_s'] = io.now_ns/1e9
            record['receiver_status'] = copy.deepcopy(io.receiver_evidence.summary(
                record['last_step_started_sim_s'], record['final_sim_s']))
            if len(io.poses)-last_pose_count < 2:
                raise RuntimeError('Stop observation lacks two poses from its final step')
            pair = copy.deepcopy(io.poses[-2:])
        for pose in pair:
            if (not number(pose['sim_s']) or pose['sim_s'] < 0 or not number(pose['yaw_deg'])
                    or not isinstance(pose['position_m'], list) or len(pose['position_m']) != 3
                    or not all(number(v) for v in pose['position_m'])):
                raise ValueError('Invalid stop observation pose')
        before, final = pair
        fresh = (record['last_step_started_sim_s'] < before['sim_s']
                 < final['sim_s'] <= record['final_sim_s'])
        record['post_stop_pose_pair_fresh'] = fresh
        record['final_pose_pair'] = pair
        if not fresh:
            raise RuntimeError('Stop observation pose pair is stale, unordered or future')
        dt = final['sim_s']-before['sim_s']
        speed = math.dist(final['position_m'][:2], before['position_m'][:2])/dt
        yawrate = math.radians(abs((final['yaw_deg']-before['yaw_deg']+180)%360-180))/dt
        if not number(speed) or not number(yawrate):
            raise ValueError('Invalid stop observation physical rates')
        record.update(final_speed_m_s=speed, final_yaw_rate_rad_s=yawrate)
        receiver = record['receiver_status']
        if not receiver['final_step_fresh'] or not receiver['final_zero']:
            raise RuntimeError('Stop observation receiver evidence is not from its final step')
        record['observation_complete'] = True
        record['stationary'] = speed < .01 and yawrate < .02
    except Exception as exc:
        record['error'] = f'{type(exc).__name__}: {exc}'
    return record


def _failure_stop(output, controller, gate, io, pending, error, committed_count):
    """Preserve the failed tick separately; never rewrite the committed trail.

    A submitted zero is only a transport request, not measured braking proof.
    Stop BEFORE controller cancellation, serialization or cleanup stepping.
    """
    gate.trip('runner_exception')
    submitted = False; stop_error = None
    if io is not None:
        try:
            io.command(0., 0.)
            submitted = True
        except Exception as exc:
            stop_error = f'{type(exc).__name__}: {exc}'
    cancel_error = None
    try:
        controller.cancel(io.now_ns / 1e9 if io is not None else 0.)
    except Exception as exc:
        cancel_error = f'{type(exc).__name__}: {exc}'
    record = dict(version=1, exception=error, pending_tick=copy.deepcopy(pending),
                  committed_decision_count=committed_count,
                  zero_command_submitted=submitted, zero_command_error=stop_error,
                  cancellation_error=cancel_error,
                  controller_terminal=controller.state in ('aborted', 'visual_stop'),
                  actuator_gate=dict(latched=gate.latched, reason=gate.reason, wheels=list(gate.wheels)),
                  scope='Failure capture and zero submission only; not physical stop confirmation or a replayed successful decision')
    (output / 'audit/failure.json').write_text(json.dumps(record, indent=2, allow_nan=False)+'\n')
    return record


def run(output, direction="left", max_ticks=650, confirmed=False, render_mode="display", gui=False,
        scenario=None, disabled=(), goal=None, wait_for_start=False, hidden_markers=(),
        motion_permission='no_motion'):
    from simulation.gazebo_scenes import (scenario_pair, build_scenario_world, make_controller,
                                         gui_config, write_display)
    from simulation.dijkstra import plan
    from ros2_ws.src.toponav_bridge.toponav_bridge.protocol import MotorGate, command_from_decision

    scenario = scenario or direction
    if motion_permission not in ('no_motion','bounded_probe'):
        raise ValueError('Invalid straight motion permission')
    if scenario not in ('straight','blocked') and motion_permission!='no_motion':
        raise ValueError('Bounded probe permission is only for straight/blocked scenarios')
    graph, scene = scenario_pair(scenario, disabled, goal)
    preview = scenario == "topology"
    route_mode = scenario == "route"
    vision_mode = scenario == 'vision'
    if scenario not in ("topology", "straight", "blocked"):
        direction = "right" if scenario.endswith("right") else "left"
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    for folder in ("public/frames", "public/overview", "public/up", "public/down", "audit"):
        (output / folder).mkdir(parents=True)
    session = "gazebo-" + uuid.uuid4().hex
    os.environ["GZ_PARTITION"] = session
    os.environ.setdefault("GZ_IP", "127.0.0.1")
    world = output / "world.sdf"
    from simulation.gazebo_receiver import install_interlock, BINARY, BUILD, SOURCES
    world.write_text(install_interlock(build_scenario_world(scenario, graph, scene, hidden_markers), session))
    (output / "public/topology.json").write_text(json.dumps(graph, indent=2)+"\n")
    (output / "public/scene.json").write_text(json.dumps(scene, indent=2)+"\n")
    (output / "public/planned_route.json").write_text(json.dumps(plan(graph), indent=2)+"\n")
    (output / "gui.config").write_text(gui_config(scenario))
    frozen = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in
              (ROOT / "roadvision/continuous_straight.py", Path(__file__).resolve(),
               ROOT / "simulation/gazebo_actuator.py",
               ROOT / "ros2_ws/src/toponav_bridge/toponav_bridge/protocol.py",
               ROOT / "simulation/gazebo_scenes.py", output / "gui.config", output / "public/topology.json",
               output / "public/scene.json", world) if p.is_relative_to(ROOT)}
    for path in (ROOT/'simulation/gazebo_receiver.py', *(ROOT/n for n in SOURCES), BINARY, BUILD):
        if path.is_file():
            frozen[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    if scenario in ('straight','blocked'):
        task_path=output/'public/motion-task.json'
        task_path.write_text(json.dumps(dict(scenario=scenario,motion_permission=motion_permission,
                                            initial_clearance_confirmed=confirmed),indent=2)+'\n')
        frozen['public/motion-task.json']=hashlib.sha256(task_path.read_bytes()).hexdigest()
    (output / "freeze.json").write_text(json.dumps(frozen, indent=2) + "\n")
    if route_mode:
        from roadvision.landmark_route import LandmarkRouteController
        from simulation.gazebo_route_fixture import registry, MARKER_SIZE
        labels = registry(graph)
        controller = LandmarkRouteController(graph, labels, MARKER_SIZE)
        (output / "public/markers.json").write_text(json.dumps(dict(labels=labels, size_m=MARKER_SIZE), indent=2))
        frozen['public/markers.json'] = hashlib.sha256((output/'public/markers.json').read_bytes()).hexdigest()
        frozen['public/topology.json'] = hashlib.sha256((output/'public/topology.json').read_bytes()).hexdigest()
        for p in (ROOT / "roadvision/landmark_route.py", ROOT / "simulation/gazebo_route_fixture.py"):
            frozen[str(p.relative_to(ROOT))] = hashlib.sha256(p.read_bytes()).hexdigest()
        (output / "freeze.json").write_text(json.dumps(frozen, indent=2)+"\n")
    else:
        controller = make_controller(scenario, confirmed, graph,motion_permission=motion_permission)
    if vision_mode:
        for name in ('roadvision/model_route.py','roadvision/visual_corridor_geometry.py',
                     'roadvision/learn.py','roadvision/contract.py','simulation/gazebo_vision_fixture.py',
                     'simulation/structural_localization.py','simulation/path_planning.py',
                     'simulation/topology.py','simulation/dijkstra.py','roadvision/data.py',
                     'models/road_structure_frozen_v1_20261014/model.pt',
                     'models/road_structure_frozen_v1_20261014/metadata.json'):
            frozen[name] = hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
        frozen['public/topology.json'] = hashlib.sha256((output/'public/topology.json').read_bytes()).hexdigest()
        (output/'freeze.json').write_text(json.dumps(frozen,indent=2)+'\n')
    gate = MotorGate(session, lease_s=.8)
    process = None
    gui_process = None
    gui_log = None
    io = None
    rows = []
    previous = -1
    error = None
    pending = None
    failure = None
    independent_stop_observation = None
    cleanup_stop = dict(attempted=False, zero_command_submitted=False,
                        steps_requested=10, steps_completed=0, error=None,
                        pose_samples_before_zero=None, zero_requested_sim_s=None,
                        last_step_started_sim_s=None, final_sim_s=None)
    started = time.monotonic()
    try:
        if render_mode == "display" and not os.environ.get("DISPLAY"):
            raise RuntimeError("DISPLAY is required; use tools/run_gazebo_cross_validation.sh for owned Xvfb rendering")
        with (output / "gazebo.log").open("w") as log:
            process = subprocess.Popen(["gz", "sim", "-s", *(["--headless-rendering"] if render_mode == "egl" else []),
                                        "--physics-engine", "gz-physics-dartsim-plugin", "-v", "4", str(world)],
                                        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            io = GazeboIO(route=route_mode, session_id=session,
                          actuator_audit=output/'audit/actuator.jsonl')
            # Bounded discovery / loading period, never guessed sensor data.
            deadline = time.monotonic() + 45
            while True:
                if process.poll() is not None:
                    raise RuntimeError("Gazebo exited before world initialization; see gazebo.log")
                ready = "/world/toponav/control" in io.node.service_list()
                with io.condition:
                    ready = ready and all(io.cameras.views.values()) and io.clock_messages > 0
                if ready:
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError("Gazebo discovery/sensor initialization timed out: " +
                        str(dict(services=io.node.service_list(), views={v:len(x) for v,x in io.cameras.views.items()}, clocks=io.clock_messages)))
                time.sleep(.1)
            if gui:
                gui_log = (output / "gui.log").open("w")
                gui_process = subprocess.Popen(["gz", "sim", "-g", "--gui-config",
                    str(output / "gui.config"), "-v", "4"],
                    stdout=gui_log, stderr=subprocess.STDOUT, start_new_session=True)
                # Allow the native window to load its scene before motion.
                time.sleep(5)
                if gui_process.poll() is not None:
                    raise RuntimeError("Gazebo GUI exited; see gui.log")
            for _ in range(5):
                io.step(12)
                time.sleep(.1)
            # Read-only camera preview before any navigation command. Original
            # capture stamps/pixels, not a fabricated frame or node evidence.
            preview_stamp, preview_rgb, _ = io.next_frame(-1)
            preview_dir = output/'public/viewer'
            preview_dir.mkdir()
            PNG.fromarray(preview_rgb).save(preview_dir/'initial.png')
            preview_input = dict(rgb_path='viewer/initial.png',captured_sim_s=preview_stamp/1e9,auxiliary={})
            if route_mode:
                for view in ('up','down'):
                    PNG.fromarray(io.cameras.views[view][preview_stamp]).save(preview_dir/(view+'.png'))
                    preview_input['auxiliary'][view] = dict(rgb_path=f'viewer/{view}.png',captured_sim_s=preview_stamp/1e9)
            (output/'public/camera_preview.json').write_text(json.dumps(preview_input))
            if (route_mode or vision_mode) and gui and wait_for_start:
                io.command_context = {'idle': True}  # Zero only, no lease renewal.
                io.command(0, 0)
                (output / 'ready.json').write_text(json.dumps(dict(status='awaiting_operator_start')))
                print('场景就绪：等待点击“开始导航”；当前命令为零。', flush=True)
                while not (output / 'start.requested').exists():
                    if gui_process.poll() is not None:
                        raise RuntimeError('Gazebo closed before navigation start')
                    write_display(output, graph, scene, io, '场景就绪 · 请点击开始导航')
                    time.sleep(.2)
                io.refresh_disarmed_receiver()
            for tick in range(min(max_ticks, 5) if preview else max_ticks):
                tick_started = time.monotonic()
                if gui_process is not None and gui_process.poll() is not None:
                    raise RuntimeError("Gazebo window closed during motion; stopping")
                timestamp, rgb, now = io.next_frame(previous)
                frame = dict(frame_id=f"{session}/{timestamp}", captured_sim_s=timestamp / 1e9, rgb=rgb)
                auxiliary = {}
                if route_mode:
                    for view in ("up", "down"):
                        pixels = io.cameras.views[view][timestamp]
                        frame[view+"_rgb"] = pixels
                        relative_aux = f"{view}/{tick:04d}.png"
                        PNG.fromarray(pixels).save(output / "public" / relative_aux)
                        auxiliary[view] = dict(rgb_path=relative_aux, captured_sim_s=timestamp/1e9,
                                              rgb_sha256=hashlib.sha256(pixels.tobytes()).hexdigest())
                # Save the exact frame BEFORE a backend/gate/transport can
                # fail. Failed attempts are not silently dropped or inserted
                # into the successful-decision replay prefix.
                relative = f"frames/{tick:04d}.png"
                PNG.fromarray(rgb).save(output / "public" / relative)
                reference = dict(frame_id=frame["frame_id"], captured_sim_s=frame["captured_sim_s"],
                                 rgb_path=relative, rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest())
                if route_mode:
                    reference['auxiliary'] = auxiliary
                pending = dict(tick=tick, sim_s=now, input=reference, stage='controller_update',
                               decision=None, command=None)
                d = controller.update(frame, now)
                pending['decision'] = copy.deepcopy(d)
                pending['stage'] = 'actuator_gate'
                wall = time.monotonic()
                gate.arm(frame["frame_id"], frame["captured_sim_s"], wall)
                command = command_from_decision(session, tick + 1, d)
                # The existing wire gate names a zero observation hold
                # "observing"; CornerController calls that hold "braking".
                # Preserve the original decision; adapt only the wire enum.
                if command["state"] in ("braking", "waiting_evidence"):
                    if command["v_m_s"] or command["omega_rad_s"]:
                        raise ValueError("A braking hold must be zero")
                    command["state"] = "observing"
                elif command["state"] in ("bootstrapping", "following"):
                    command["state"] = "moving"
                pending['command'] = copy.deepcopy(command)
                issued_wall = time.monotonic()
                v, omega = gate.accept(command, now, issued_wall)
                pending['actuator_gate'] = dict(latched=gate.latched, reason=gate.reason, wheels=list(gate.wheels))
                pending['stage'] = 'command_submission'
                # The executor uses the ORIGINAL absolute hold deadline, not
                # receipt time or a periodic heartbeat. Expired commands can
                # never restart motion after a delayed/resumed navigation loop.
                io.command_context = None if gate.latched else dict(
                    command=copy.deepcopy(command), sim_s=now,
                    issued_steady_s=issued_wall, expires_steady_s=gate.hold_deadline)
                io.command(v, omega)
                pending['stage'] = 'gate_result'
                if gate.latched and d["state"] not in ("aborted", "visual_stop"):
                    raise RuntimeError("Actuator gate latched: " + gate.reason)
                if io.overview is not None:
                    PNG.fromarray(io.overview[1]).save(output / f"public/overview/{tick:04d}.png")
                row = dict(tick=tick, sim_s=now, input=dict(frame_id=frame["frame_id"],
                       captured_sim_s=frame["captured_sim_s"], rgb_path=relative,
                       rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest()), decision=d,
                       command=command, actuator_gate=dict(latched=gate.latched, reason=gate.reason))
                if route_mode:
                    row["input"]["auxiliary"] = auxiliary
                # Commit before optional display work. An observer failure
                # cannot erase an already submitted command from the trail.
                with (output / "decisions.jsonl").open("a") as f:
                    f.write(json.dumps(row, allow_nan=False) + "\n")
                rows.append(row)
                pending = None
                if tick % 5 == 0 or d["state"] in ("aborted", "visual_stop"):
                    label = "仅场景/规划预览，不自动驾驶" if preview else d["state"]+" / "+d.get("phase", "straight")
                    if route_mode or vision_mode:
                        label = '视觉确认: '+','.join(d['visited_nodes'])+' / '+d['phase']
                    write_display(output, graph, scene, io, label)
                previous = timestamp
                io.step(12)
                if gui:
                    # Approximately real-time motion, rather than a fast smoke run.
                    time.sleep(max(0, .1 - (time.monotonic() - tick_started)))
                if tick % 10 == 0 or d["state"] in ("aborted", "visual_stop"):
                    print(tick, round(now, 3), d["state"], d["reason"], flush=True)
                if d["state"] in ("aborted", "visual_stop"):
                    break
            else:
                controller.cancel(io.now_ns / 1e9)
            if gui_process is not None:
                io.command(0, 0)
                print("Motion finished. Gazebo window stays open; close it to finish saving the run.", flush=True)
                while gui_process.poll() is None:
                    label = "仅场景/规划预览，不自动驾驶" if preview else controller.state
                    if route_mode or vision_mode:
                        label = '视觉确认: '+','.join(controller.visited)+' / '+controller.state
                    write_display(output, graph, scene, io, label)
                    time.sleep(.5)
    except KeyboardInterrupt:
        terminal = controller.state in ("visual_stop", "aborted")
        controller.cancel(io.now_ns / 1e9 if io else 0.)
        if not terminal and not (preview and rows):
            error = "Operator cancelled before task completion; commanded zero in cleanup"
        print("Window/session stop requested; saving and cleaning up.", flush=True)
    except Exception:
        error = traceback.format_exc()
        failure = _failure_stop(output, controller, gate, io, pending, error, len(rows))
        print(error, flush=True)
    finally:
        if io is not None:
            cleanup_stop.update(attempted=True, pose_samples_before_zero=len(io.poses),
                                zero_requested_sim_s=io.now_ns / 1e9)
            try:
                io.command(0, 0)
                cleanup_stop['zero_command_submitted'] = True
                for _ in range(10):
                    cleanup_stop['last_step_started_sim_s'] = io.now_ns / 1e9
                    io.step(12)
                    cleanup_stop['steps_completed'] += 1
                    time.sleep(.03)
            except Exception as exc:
                cleanup_stop['error'] = f'{type(exc).__name__}: {exc}'
                # Keep the original execution exception when both fail. A
                # terminal controller latch is not successful stop execution.
                if error is None:
                    error = 'Final stop cleanup failed: ' + cleanup_stop['error']
                if (not cleanup_stop['zero_command_submitted']
                        and getattr(getattr(io, 'actuator', None), 'failed', False) is True):
                    independent_stop_observation = _observe_failed_actuator_stop(io)
                    (output/'audit/independent-stop.json').write_text(
                        json.dumps(independent_stop_observation, indent=2, allow_nan=False)+'\n')
            cleanup_stop['final_sim_s'] = io.now_ns / 1e9
        (output / 'audit/cleanup-stop.json').write_text(json.dumps(cleanup_stop, indent=2)+'\n')
        if gui_process is not None and gui_process.poll() is None:
            os.killpg(gui_process.pid, signal.SIGINT)
            try:
                gui_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(gui_process.pid, signal.SIGKILL)
                gui_process.wait()
        if gui_log is not None:
            gui_log.close()
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        if io is not None:
            io.stepper.stdin.close()
            try:
                io.stepper.wait(timeout=5)
            except subprocess.TimeoutExpired:
                io.stepper.kill()
                io.stepper.wait()
            actuator = getattr(io, 'actuator', None)
            if actuator is not None:
                try:
                    actuator.close()  # Owned world is already stopped above.
                except Exception as exc:
                    cleanup_stop['actuator_close_error'] = f'{type(exc).__name__}: {exc}'
                    if error is None:
                        error = 'Independent actuator cleanup failed: '+cleanup_stop['actuator_close_error']
                    (output/'audit/cleanup-stop.json').write_text(json.dumps(cleanup_stop, indent=2)+'\n')
    actuator_audit = None
    if io is not None and getattr(io, 'actuator', None) is not None:
        from simulation.gazebo_actuator import summarize_audit
        actuator_audit = summarize_audit(output/'audit/actuator.jsonl', io.actuator.process.pid,
                                         expected_status=io.actuator.status)
    receiver_audit = None
    if io is not None and getattr(io, 'receiver_evidence', None) is not None:
        with io.condition:
            receiver_audit = io.receiver_evidence.summary(
                cleanup_stop.get('last_step_started_sim_s'), cleanup_stop.get('final_sim_s'))
            receiver_events = list(io.receiver_evidence.events)
            if io.receiver_evidence.bind_submissions:
                (output/'audit/receiver-submissions.json').write_text(
                    json.dumps(io.receiver_evidence.submissions, indent=2, allow_nan=False)+'\n')
        (output/'audit/receiver.json').write_text(json.dumps(receiver_events, indent=2)+'\n')
    poses = io.poses if io else []
    contacts = io.contacts if io else []
    contact_not_retained = getattr(io, '_contact_not_retained', 0) if io else 0
    pose_not_retained = getattr(io, '_pose_not_retained', 0) if io else 0
    if pose_not_retained and error is None:
        error = 'Gazebo pose history capacity exceeded; physical audit incomplete'
    if contact_not_retained and error is None:
        error = 'Gazebo contact history capacity exceeded; contact audit incomplete'
    if io is not None and getattr(io, 'error', None) is not None and error is None:
        error = 'Gazebo I/O callback failure: '+str(io.error)
    audit = dict(pose_samples=len(poses), observed_wall_contacts=len(contacts),
                 contact_history=dict(limit=MAX_GAZEBO_CONTACTS, retained=len(contacts),
                     capacity_exceeded=bool(contact_not_retained), not_retained=contact_not_retained,
                     not_retained_count_is_lower_bound=contact_not_retained == MAX_CONTACT_NOT_RETAINED,
                     scope='Bounded observed wall-contact prefix only; later entries are UNVALIDATED, '
                           'not classified/counted as wall contacts. Retained is NOT total wall contacts; '
                           'NOT collision coverage, input/name-size, frame/transport/disk/whole-process bounds'),
                 pose_history=dict(limit=MAX_GAZEBO_POSES, retained=len(poses),
                     capacity_exceeded=bool(pose_not_retained), not_retained=pose_not_retained,
                     not_retained_count_is_lower_bound=pose_not_retained == MAX_POSE_NOT_RETAINED,
                     scope='Bounded native robot-pose prefix only; omitted matching-name entries are UNVALIDATED, '
                           'NOT fresh physical/stop evidence; NOT contact/frame/transport/disk/whole-process bounds'),
                 contact_messages=dict(io.contact_messages) if io else {},
                 contacts_scope="observed native contact messages only; a silent base sensor does not prove complete collision coverage",
                 collision_free_certified=False, stationary=False,
                 post_stop_pose_pair_fresh=False)
    if len(poses) >= 2:
        final, before = poses[-1], poses[-2]
        dt = final["sim_s"] - before["sim_s"]
        speed = float(np.linalg.norm(np.array(final["position_m"][:2]) - before["position_m"][:2]) / dt) if dt > 0 else None
        yawrate = math.radians(abs((final["yaw_deg"] - before["yaw_deg"] + 180) % 360 - 180)) / dt if dt > 0 else None
        sign = 1 if direction == "left" else -1
        heading_error = abs((final["yaw_deg"] - sign * 90 + 180) % 360 - 180)
        # Two newly received, ordered samples from the LAST cleanup step
        # must demonstrate final velocity, not a quiet interval before zero.
        # Bounds use the same native nanosecond-to-second conversion as poses;
        # no age grace period or additional simulation steps are introduced.
        fresh = (not pose_not_retained and cleanup_stop['zero_command_submitted']
                 and len(poses) - cleanup_stop['pose_samples_before_zero'] >= 2
                 and cleanup_stop['last_step_started_sim_s'] is not None
                 and cleanup_stop['last_step_started_sim_s'] < before['sim_s']
                 < final['sim_s'] <= cleanup_stop['final_sim_s'])
        stop_executed = (cleanup_stop['error'] is None
                         and cleanup_stop['steps_completed'] == cleanup_stop['steps_requested'])
        stopped = stop_executed and fresh and speed is not None and speed < .01 and yawrate < .02
        geometric = abs(final["position_m"][0] - 2.5) < .20 and sign * final["position_m"][1] > 1.10 and heading_error < 6
        if scenario in ("straight", "blocked", "topology"):
            end_x = 2.525 if scenario == "blocked" else 4.
            gap = end_x - final["position_m"][0] - .2
            geometric = not preview and .8 <= gap <= 1.7 and abs(final["position_m"][1]) < .2 and abs(final["yaw_deg"]) < 6
            heading_error = abs(final["yaw_deg"])
        audit.update(initial=poses[0], final=final, final_speed_m_s=speed,
                     final_yaw_rate_rad_s=yawrate, stationary=stopped,
                     post_stop_pose_pair_fresh=fresh,
                     normal_geometric_stop_pass=geometric, heading_error_deg=heading_error)
        if scenario not in ("straight", "blocked", "topology"):
            audit["corner_geometric_exit_pass"] = geometric
    if (route_mode or vision_mode) and poses:
        positions = scene['geometry']['node_positions_m']
        goal_xy = positions[graph['task']['goal_node']]
        goal_error = math.dist(poses[-1]['position_m'][:2], goal_xy)
        checks = []
        for event in controller.result()['node_events']:
            pose_at = min(poses, key=lambda p: abs(p['sim_s']-event['captured_sim_s']))
            checks.append(dict(node_id=event['node_id'], captured_sim_s=event['captured_sim_s'],
                               distance_m=math.dist(pose_at['position_m'][:2], positions[event['node_id']])))
        audit['node_confirmation_checks'] = checks
        audit['goal_error_m'] = goal_error
        audit['normal_geometric_stop_pass'] = goal_error < .15 and bool(checks) and all(c['distance_m'] < .15 for c in checks)
        audit.pop('corner_geometric_exit_pass', None)
        audit.pop('heading_error_deg', None)
    log_text = (output / "gazebo.log").read_text(errors="replace") if (output / "gazebo.log").exists() else ""
    dart_loaded = "gz::physics::dartsim::Plugin" in log_text
    result = dict(version="gazebo_cross_check_v1", simulator="Gazebo Harmonic 8 / DART / Ogre2",
        run_config=dict(initial_clearance_confirmed=confirmed, motion_permission=motion_permission, max_ticks=max_ticks,
                        render_mode=render_mode, gui=gui, scenario=scenario,
                        hidden_markers=list(hidden_markers),
                        physics_engine="gz-physics-dartsim-plugin"),
        mode="frozen_model_rgb_route" if vision_mode else "landmark_assisted_full_route" if route_mode else "scene_preview_only" if preview else "local_rgb_primitive_cross_check",
        preview_loaded=preview and error is None and dart_loaded and bool(rows),
        dart_plugin_loaded=dart_loaded,
        direction=direction, session_id=session, controller_result=controller.result(), audit=audit,
        exception=error, cleanup_stop=cleanup_stop, actuator_watchdog=actuator_audit, receiver_interlock=receiver_audit,
        independent_stop_observation=independent_stop_observation,
        elapsed_wall_s=time.monotonic() - started,
        scope="original frozen 17-class model, original topology localization/planner, new bounded RGB geometry servo; no tags/paint/pose; not formal S2/S6 or general-vision acceptance" if vision_mode else "new marker/guide-line-assisted RGB baseline, not original model, formal S2/S6 or ROS acceptance" if route_mode else "scene and planning preview only, zero commands, no autonomous navigation" if preview else "engineering cross-simulator smoke, unchanged RGB controller; not formal S2/S6 or ROS acceptance",
        controller_inputs=["native four-view RGB and original sim stamps", "frozen original 17-class model/threshold", "public topology/task", "declared optical/floor/width calibration"] if vision_mode else ["native synchronized up/down RGB", "original capture time", "public topology and cardinal exits", "marker ID labels/size (no XY)"] if route_mode else [] if preview else ["four synchronized Gazebo RGB images", "original simulation capture time", "declared initial clearance", "declared local primitive task"],
        completed=not preview and error is None and dart_loaded and controller.state == "visual_stop" and audit.get("normal_geometric_stop_pass", False) and audit.get("stationary", False) and not contacts and not (actuator_audit and actuator_audit['fault_latched']) and not (receiver_audit and (receiver_audit['fault_latched'] or not receiver_audit['final_step_fresh'] or not receiver_audit['final_zero'])))
    if failure is not None:
        result['failure_record'] = failure
    (output / "audit/poses.json").write_text(json.dumps(poses, indent=2) + "\n")
    (output / "audit/contacts.json").write_text(json.dumps(contacts, indent=2) + "\n")
    (output / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps(result, indent=2), flush=True)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--direction", choices=("left", "right"), default="left")
    parser.add_argument("--max-ticks", type=int, default=650)
    parser.add_argument("--confirm-initial-clearance", action="store_true")
    parser.add_argument('--motion-permission',choices=('no_motion','bounded_probe'),default='no_motion',
                        help='Explicit straight/blocked probe authorization; clearance alone never grants motion')
    parser.add_argument("--render-mode", choices=("egl", "display"), default="display")
    parser.add_argument("--gui", action="store_true", help="Open a native Gazebo window, pace motion, keep open until closed")
    parser.add_argument('--wait-for-start', action='store_true', help='GUI route waits for output/start.requested (native scenario app button)')
    parser.add_argument('--hide-marker', action='append', default=[], help='Route fixture fault: physically omit a ceiling tag; controller is not told')
    from simulation.gazebo_scenes import SCENARIOS
    parser.add_argument("--scenario", choices=SCENARIOS, help="Local primitive, topology preview, or new marker-assisted route")
    parser.add_argument("--disable", action="append", default=[], help="Physically block / disable a channel in the six-node fixture")
    parser.add_argument("--goal", help="Task/planning goal in the six-node fixture")
    args = parser.parse_args(argv)
    if args.scenario in ('route','vision') and args.max_ticks == 650:
        args.max_ticks = 2200
    if args.max_ticks < 1 or args.max_ticks > (2200 if args.scenario in ('route','vision') else 900):
        parser.error("max-ticks must be in 1..900 (route: 1..2200)")
    if args.gui and args.render_mode != "display":
        parser.error("--gui requires --render-mode display")
    if args.motion_permission!='no_motion' and args.scenario not in ('straight','blocked'):
        parser.error('--motion-permission bounded_probe requires --scenario straight or blocked')
    result = run(args.output, args.direction, args.max_ticks, args.confirm_initial_clearance,
                 args.render_mode, args.gui, args.scenario, args.disable, args.goal, args.wait_for_start, args.hide_marker,
                 motion_permission=args.motion_permission)
    return 0 if result["completed"] or result["preview_loaded"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
