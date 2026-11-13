"""6B-1 differential-drive corridor physics and RGB-only motion primitive.

Physics, camera mounting and one-way audit own truth; the controller never does.
This deliberately does NOT adapt partial motion into the old atomic-edge API.
"""
import argparse
import copy
import hashlib
import importlib.metadata
import json
import math
import os
import platform
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import pybullet as p

from roadvision.corridor_control import CorridorController, VisionConfig


@dataclass(frozen=True)
class CorridorConfig:
    length_m: float = 6.0
    width_m: float = 1.8
    initial_offset_m: float = 0.0
    initial_yaw_deg: float = 0.0
    obstacle: str = "none"

    def __post_init__(self):
        for name in ("length_m", "width_m", "initial_offset_m", "initial_yaw_deg"):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"Invalid {name}")
        if self.length_m < 4 or self.width_m < 1.4 or abs(self.initial_offset_m) > self.width_m / 2 - 0.3:
            raise ValueError("Invalid corridor dimensions or initial clearance")
        if abs(self.initial_yaw_deg) > 15 or self.obstacle not in {"none", "narrow", "full_width"}:
            raise ValueError("Only near-aligned corridor starts and declared obstacles are supported")


class CorridorWorld:
    physics_dt = 1 / 120
    control_dt = 0.1
    wheel_radius_m = 0.08
    track_m = 0.30

    def __init__(self, config=None):
        self.config = config or CorridorConfig()
        self.client = p.connect(p.DIRECT)
        self.steps = 0
        self.wall_contact_physics_steps = 0
        self.last_wheels = [0.0, 0.0]
        try:
            p.setGravity(0, 0, -9.81, physicsClientId=self.client)
            p.setTimeStep(self.physics_dt, physicsClientId=self.client)
            p.setPhysicsEngineParameter(numSolverIterations=80, deterministicOverlappingPairs=1,
                                       physicsClientId=self.client)
            length, halfwidth = self.config.length_m, self.config.width_m / 2
            self.floor = self._box((length / 2 + 0.2, halfwidth + 0.2, 0.05), (length / 2, 0, -0.05), (0.20, 0.29, 0.40))
            self.walls = [self._box((length / 2 + 0.1, 0.05, 0.8), (length / 2, y, 0.8), (0.65, 0.62, 0.58))
                          for y in (-halfwidth - 0.05, halfwidth + 0.05)]
            self.walls += [self._box((0.05, halfwidth, 0.8), (x, 0, 0.8), (0.65, 0.62, 0.58))
                           for x in (-0.05, length + 0.05)]
            if self.config.obstacle != "none":
                width = halfwidth if self.config.obstacle == "full_width" else 0.25
                self.walls.append(self._box((0.05, width, 0.8), (3.05, 0, 0.8), (0.65, 0.62, 0.58)))
            self.robot = self._robot()
            self.drive(0, 0)
            for _ in range(60):
                self._step()
        except Exception:
            p.disconnect(self.client)
            raise

    def _box(self, half, center, color):
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=self.client)
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=(*color, 1), physicsClientId=self.client)
        return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual,
                                 basePosition=center, physicsClientId=self.client)

    def _robot(self):
        base_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.16, 0.11, 0.06), physicsClientId=self.client)
        base_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.16, 0.11, 0.06),
                                         rgbaColor=(0.2, 0.25, 0.3, 1), physicsClientId=self.client)
        rotation = p.getQuaternionFromEuler((math.pi / 2, 0, 0))
        wheel_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.08, height=0.04,
                                                collisionFrameOrientation=rotation, physicsClientId=self.client)
        wheel_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.08, length=0.04,
                                           visualFrameOrientation=rotation, rgbaColor=(0.1, 0.1, 0.1, 1),
                                           physicsClientId=self.client)
        caster = p.createCollisionShape(p.GEOM_SPHERE, radius=0.03, physicsClientId=self.client)
        robot = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=base_collision,
            baseVisualShapeIndex=base_visual, basePosition=(0.5, self.config.initial_offset_m, 0.16),
            baseOrientation=p.getQuaternionFromEuler((0, 0, math.radians(self.config.initial_yaw_deg))),
            linkMasses=[0.1, 0.1, 0.01, 0.01], linkCollisionShapeIndices=[wheel_collision, wheel_collision, caster, caster],
            linkVisualShapeIndices=[wheel_visual, wheel_visual, -1, -1],
            linkPositions=[(0, 0.15, -0.08), (0, -0.15, -0.08), (0.13, 0, -0.13), (-0.13, 0, -0.13)],
            linkOrientations=[(0, 0, 0, 1)] * 4, linkInertialFramePositions=[(0, 0, 0)] * 4,
            linkInertialFrameOrientations=[(0, 0, 0, 1)] * 4, linkParentIndices=[0] * 4,
            linkJointTypes=[p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_FIXED, p.JOINT_FIXED],
            linkJointAxis=[(0, 1, 0), (0, 1, 0), (0, 0, 0), (0, 0, 0)], physicsClientId=self.client)
        for link in (0, 1):
            p.changeDynamics(robot, link, lateralFriction=1.5, rollingFriction=0,
                             spinningFriction=0, physicsClientId=self.client)
        for link in (2, 3):
            p.changeDynamics(robot, link, lateralFriction=0.001, physicsClientId=self.client)
        return robot

    @property
    def sim_s(self):
        return self.steps * self.physics_dt

    def _step(self):
        p.stepSimulation(physicsClientId=self.client)
        self.steps += 1
        # One-way audit at the physics rate: a brief collision must not be
        # lost between 10 Hz camera ticks. Never feed this into drive/control.
        if any(p.getContactPoints(self.robot, wall, physicsClientId=self.client) for wall in self.walls):
            self.wall_contact_physics_steps += 1

    def step_control(self):
        for _ in range(12):
            self._step()

    def drive(self, v, omega):
        if any(type(x) not in (int, float) or not math.isfinite(x) for x in (v, omega)) or abs(v) > 0.5 or abs(omega) > 1.0:
            raise ValueError("Velocity command exceeds primitive limits")
        left = (v - omega * self.track_m / 2) / self.wheel_radius_m
        right = (v + omega * self.track_m / 2) / self.wheel_radius_m
        self.last_wheels = [left, right]
        p.setJointMotorControlArray(self.robot, [0, 1], p.VELOCITY_CONTROL,
                                   targetVelocities=self.last_wheels, forces=[1.5, 1.5], physicsClientId=self.client)

    def capture(self):
        # Simulator-only mounting transform; no physical pose leaves this packet.
        position, orientation = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
        origin, camera_q = p.multiplyTransforms(position, orientation, (0.20, 0, 0.29), (0, 0, 0, 1))
        matrix = np.asarray(p.getMatrixFromQuaternion(camera_q)).reshape(3, 3)
        images = []
        projection = p.computeProjectionMatrixFOV(90, 1, 0.05, 30)
        for angle in (0, math.pi / 2, -math.pi / 2, math.pi):
            forward = matrix @ np.array((math.cos(angle), math.sin(angle), 0))
            target = np.array(origin) + forward
            view = p.computeViewMatrix(origin, target.tolist(), matrix[:, 2].tolist())
            pixels = p.getCameraImage(128, 128, view, projection, renderer=p.ER_TINY_RENDERER,
                                      lightDirection=(1, -1, 3), physicsClientId=self.client)[2]
            images.append(np.asarray(pixels, dtype=np.uint8).reshape(128, 128, 4)[..., :3])
        rgb = np.concatenate((np.concatenate(images[:2], axis=1), np.concatenate(images[2:], axis=1)), axis=0)
        return {"frame_id": f"physics-frame-{self.steps}", "captured_sim_s": self.sim_s, "rgb": rgb}

    def audit_state(self):
        position, orientation = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
        velocity, angular = p.getBaseVelocity(self.robot, physicsClientId=self.client)
        contacts = sum(bool(p.getContactPoints(self.robot, wall, physicsClientId=self.client)) for wall in self.walls)
        camera_position, _ = p.multiplyTransforms(position, orientation, (0.20, 0, 0.29), (0, 0, 0, 1))
        return {"sim_s": self.sim_s, "position_m": list(position),
                "camera_position_m": list(camera_position),
                "yaw_deg": math.degrees(p.getEulerFromQuaternion(orientation)[2]),
                "speed_m_s": float(np.linalg.norm(velocity[:2])), "yaw_rate_rad_s": angular[2],
                "wall_contacts": contacts}

    def close(self):
        if p.isConnected(self.client):
            p.disconnect(self.client)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


FAULTS = ("none", "missing_frame", "missing_view", "occluded", "stale_frame", "future_frame", "replay_frame")


def run_corridor(config=None, vision_config=None, fault="none", fault_cycle=15, output_dir=None):
    config, vision_config = config or CorridorConfig(), vision_config or VisionConfig()
    if fault not in FAULTS or type(fault_cycle) is not int or fault_cycle < 0:
        raise ValueError("Invalid camera fault")
    if fault == "replay_frame" and fault_cycle == 0:
        raise ValueError("Replay fault requires a preceding frame")
    if output_dir is not None:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=False)
        (output_dir / "frames").mkdir()
    controller = CorridorController(vision_config)
    started = time.perf_counter()
    steps, braking, previous = [], [], None
    with CorridorWorld(config) as world:
        initial = world.audit_state()
        try:
            for index in range(math.ceil(vision_config.max_duration_s / world.control_dt) + 2):
                capture_started = time.perf_counter()
                captured = world.capture()
                capture_elapsed = time.perf_counter() - capture_started
                rgb_original = captured["rgb"]
                injection = fault if index == fault_cycle else "none"
                if injection == "missing_frame":
                    frame = None
                elif injection == "replay_frame" and previous is not None:
                    frame = copy.deepcopy(previous)
                else:
                    frame = copy.deepcopy(captured)
                    if injection == "missing_view":
                        frame["rgb"][:128, :128] = 0
                    elif injection == "occluded":
                        frame["rgb"][:] = 140
                    elif injection == "stale_frame":
                        frame["captured_sim_s"] -= 0.5
                    elif injection == "future_frame":
                        frame["captured_sim_s"] += 0.5
                if frame is not None:
                    previous = copy.deepcopy(frame)
                control_started = time.perf_counter()
                output = controller.update(frame, world.sim_s) if frame is not None else controller.missing(world.sim_s)
                control_elapsed = time.perf_counter() - control_started
                before = world.audit_state()  # Below this line truth is logging only.
                world.drive(output["v_m_s"], output["omega_rad_s"])
                frame_path = None
                if output_dir is not None:
                    from PIL import Image
                    frame_path = f"frames/{index:04d}.png"
                    Image.fromarray(frame["rgb"] if frame is not None else rgb_original).save(output_dir / frame_path)
                steps.append({"index": index, "frame_id": frame["frame_id"] if frame else None,
                    "captured_sim_s": frame["captured_sim_s"] if frame else None,
                    "frame_sha256": hashlib.sha256((frame["rgb"] if frame else rgb_original).tobytes()).hexdigest(),
                    "frame_path": frame_path, "injection": injection, "controller": output,
                    "capture_wall_elapsed_s": capture_elapsed, "control_wall_elapsed_s": control_elapsed,
                    "wheel_targets_rad_s": list(world.last_wheels), "audit_before": before})
                if output["state"] in {"visual_stop", "aborted"}:
                    break
                world.step_control()
            else:
                output = controller.missing(world.sim_s)
        finally:
            world.drive(0, 0)
            # Physical braking/settling is logged, never used to repair controller status.
            for _ in range(10):
                world.step_control()
                braking.append(world.audit_state())
        final = world.audit_state()
        physics_contact_steps = world.wall_contact_physics_steps
    truth = [s["audit_before"] for s in steps] + braking
    contact_frames = sum(s["wall_contacts"] > 0 for s in truth)
    distance = sum(math.dist(a["position_m"][:2], b["position_m"][:2]) for a, b in zip([initial] + truth, truth))
    end_distance = config.length_m - final["camera_position_m"][0]
    verified = (controller.state == "visual_stop" and 0.9 <= end_distance <= 1.7
                and abs(final["position_m"][1]) < 0.2 and abs(final["yaw_deg"]) < 8
                and final["speed_m_s"] < 0.03 and abs(final["yaw_rate_rad_s"]) < 0.03
                and physics_contact_steps == 0)
    status = ("completed_visual_stop" if controller.state == "visual_stop" else
              "aborted_motion_uncertain" if controller.motion_commands else "rejected_no_motion")
    result = {"version": "corridor_motion_6B1_v1", "state": controller.state, "reason": controller.reason,
        "scene_config": asdict(config), "vision_config": asdict(vision_config), "fault": fault,
        "fault_cycle": fault_cycle, "feedback": {"status": status, "atomic_edge_compatible": False,
                                                  "motion_was_commanded": bool(controller.motion_commands)},
        "environment": {"python": sys.version, "executable": sys.executable, "platform": platform.platform(),
            "packages": {name: importlib.metadata.version(name) for name in ("numpy", "pybullet", "Pillow")},
            "threads": {name: os.environ.get(name) for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")},
            "renderer": "PyBullet TinyRenderer / DIRECT", "randomization": "none; deterministic engineering fixtures"},
        "clock_domain": "deterministic simulation seconds; not Unix capture times or hard real time",
        "wall_elapsed_s": time.perf_counter() - started, "physics_dt_s": 1/120, "control_dt_s": 0.1,
        "steps": steps, "braking_audit": braking,
        "audit": {"end_wall_verified": verified, "false_end_wall_claim": controller.state == "visual_stop" and not verified,
            "initial": initial, "final": final, "distance_travelled_m": distance,
            "final_speed_m_s": final["speed_m_s"], "final_yaw_deg": final["yaw_deg"],
            "final_yaw_rate_rad_s": final["yaw_rate_rad_s"],
            "camera_to_end_wall_m": end_distance, "wall_contact_frames": contact_frames,
            "wall_contact_physics_steps": physics_contact_steps,
            "max_lateral_offset_m": max(abs(s["position_m"][1]) for s in truth)},
        "scope": "Fixed-colour calibrated RGB corridor prototype; no topology arrival, no real-car or arbitrary-material claim"}
    if output_dir is not None:
        with (output_dir / "run.json").open("x") as file:
            json.dump(result, file, indent=2, allow_nan=False)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New directory for JSON and RGB frames")
    parser.add_argument("--offset", type=float, default=0)
    parser.add_argument("--yaw", type=float, default=0)
    parser.add_argument("--obstacle", choices=("none", "narrow", "full_width"), default="none")
    parser.add_argument("--fault", choices=FAULTS, default="none")
    parser.add_argument("--fault-cycle", type=int, default=15)
    parser.add_argument("--max-duration", type=float, default=45.0, help="Bound in simulation seconds")
    args = parser.parse_args(argv)
    try:
        result = run_corridor(CorridorConfig(initial_offset_m=args.offset, initial_yaw_deg=args.yaw,
                                            obstacle=args.obstacle),
                              VisionConfig(max_duration_s=args.max_duration), fault=args.fault,
                              fault_cycle=args.fault_cycle, output_dir=args.output)
        print(json.dumps({k: result[k] for k in ("state", "reason", "feedback", "audit")}))
    except (ValueError, OSError) as error:
        print(json.dumps({"state": "invalid", "error": str(error)}))
        return 2
    return 0 if result["audit"]["end_wall_verified"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
