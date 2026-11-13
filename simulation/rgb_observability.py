"""Controlled PyBullet RGB generator and *audit-only* projection mathematics.

Camera/plane poses are scripted, not executed by wheel motors. Scene/pose
records stay in audit/; only RGB and a strict sequence manifest go to the
frozen probe. No diagnostic threshold, sensor or learned model is changed.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import re

import cv2
import numpy as np
from PIL import Image
import pybullet as p

from roadvision.corridor_rgb_probe import CALIBRATION_ID


CASE_KEYS = {"id", "scene", "texture", "motion", "seed", "rendered_fov_deg", "pitch_rad", "declared_calibration_id"}
SCENES = {"wall", "ground", "room", "paint", "barrier"}
TEXTURES = {"unique", "uniform", "repeated"}
MOTIONS = {"static", "forward", "slow_forward", "tiny_forward", "yaw", "lateral", "moving_wall"}
ANGLES = {"front": 0., "left": math.pi / 2, "right": -math.pi / 2, "back": math.pi}


def validate_case(raw):
    if not isinstance(raw, dict) or set(raw) != CASE_KEYS:
        raise ValueError("Expected declared case fields only; no control overrides")
    if not isinstance(raw["id"], str) or not re.fullmatch(r"[a-z0-9_]{1,64}", raw["id"]):
        raise ValueError("Invalid case ID")
    for key, allowed in (("scene", SCENES), ("texture", TEXTURES), ("motion", MOTIONS)):
        if not isinstance(raw[key], str) or raw[key] not in allowed:
            raise ValueError(f"Invalid {key}")
    if type(raw["seed"]) is not int or not 0 <= raw["seed"] < 2**31:
        raise ValueError("Invalid texture seed")
    for key, low, high in (("rendered_fov_deg", 50, 110), ("pitch_rad", -.3, .3)):
        value = raw[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"Invalid {key}")
    identity = raw["declared_calibration_id"]
    if not isinstance(identity, str) or not identity.strip() or len(identity) > 256:
        raise ValueError("Invalid declared calibration ID")
    if raw["motion"] == "moving_wall" and raw["scene"] != "wall":
        raise ValueError("Dynamic confuser is restricted to the single wall")
    return copy.deepcopy(raw)


def load_suite(path):
    data = json.loads(Path(path).read_text())
    if not isinstance(data, dict) or set(data) != {"version", "frame_count", "capture_dt_s", "cases"}:
        raise ValueError("Invalid suite fields")
    if data["version"] != "rgb_observability_suite_v1" or data["frame_count"] != 5 or data["capture_dt_s"] != .1:
        raise ValueError("Unsupported version or scripted sampling schedule")
    if not isinstance(data["cases"], list) or not 1 <= len(data["cases"]) <= 64:
        raise ValueError("Expected 1..64 cases")
    cases = [validate_case(case) for case in data["cases"]]
    if len({c["id"] for c in cases}) != len(cases):
        raise ValueError("Duplicate case IDs")
    return {**data, "cases": cases}


def make_texture(kind, seed):
    if kind == "unique":
        rng = np.random.default_rng(seed)
        gray = cv2.GaussianBlur(rng.integers(20, 235, (512, 512), dtype=np.uint8), (3, 3), .6)
    elif kind == "uniform":
        gray = np.full((512, 512), 128, np.uint8)
    elif kind == "repeated":
        y, x = np.indices((512, 512))
        gray = ((y // 32 + x // 32) % 2 * 150 + 40).astype(np.uint8)
    else:
        raise ValueError("Unknown texture")
    return np.repeat(gray[:, :, None], 3, axis=2)


def camera_sample(case, index):
    case = validate_case(case)
    if type(index) is not int or not 0 <= index < 5:
        raise ValueError("Expected capture index 0..4")
    position, yaw, shift = [0., 0., .45], 0., [0., 0., 0.]
    motion = case["motion"]
    if motion in {"forward", "slow_forward", "tiny_forward"}:
        position[0] = index * {"forward": .1, "slow_forward": .035, "tiny_forward": .0005}[motion]
    elif motion == "lateral":
        position[1] = index * .1
    elif motion == "yaw":
        yaw = index * .025
    elif motion == "moving_wall":
        shift[0] = -index * .1
    return {"captured_sim_s": round(index * .1, 10), "position_m": position, "yaw_rad": yaw,
            "pitch_rad": case["pitch_rad"], "rendered_fov_deg": case["rendered_fov_deg"], "surface_shift_m": shift}


def view_basis(sample, view):
    yaw, pitch, angle = sample["yaw_rad"], sample["pitch_rad"], ANGLES[view]
    forward = np.array([math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), math.sin(pitch)])
    left = np.array([-math.sin(yaw), math.cos(yaw), 0.])
    up = np.cross(forward, left)
    direction = math.cos(angle) * forward + math.sin(angle) * left
    right = np.cross(direction, up)
    return direction, up, np.column_stack((right, -up, direction))


class ControlledWorld:
    """Static visual meshes; no robot, collision shapes or motion controller."""
    def __init__(self, case, assets):
        self.case = validate_case(case)
        self.assets = Path(assets)
        self.assets.mkdir(parents=True, exist_ok=False)
        self.client = p.connect(p.DIRECT)
        self.bodies, self.surface_records = [], []
        try:
            self._build()
        except Exception:
            p.disconnect(self.client)
            raise

    def __enter__(self):
        return self

    def __exit__(self, *args):
        p.disconnect(self.client)

    def _quad(self, name, vertices, *, contrast=False):
        vertices = np.asarray(vertices, dtype=float)
        normal = np.cross(vertices[1] - vertices[0], vertices[2] - vertices[0])
        normal /= np.linalg.norm(normal)
        obj, texture = self.assets / f"{name}.obj", self.assets / f"{name}.png"
        text = "".join("v " + " ".join(map(str, v)) + "\n" for v in vertices)
        text += "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\n"
        text += "vn " + " ".join(map(str, normal)) + "\n"
        text += "f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n"
        obj.write_text(text)
        rgb = (np.full((512, 512, 3), 96, np.uint8) if contrast else
               make_texture(self.case["texture"], self.case["seed"] + len(self.bodies)))
        Image.fromarray(rgb).save(texture)
        shape = p.createVisualShape(p.GEOM_MESH, fileName=str(obj), rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0, 0, 0], physicsClientId=self.client)
        body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=shape, baseCollisionShapeIndex=-1,
                                 physicsClientId=self.client)
        texture_id = p.loadTexture(str(texture), physicsClientId=self.client)
        p.changeVisualShape(body, -1, textureUniqueId=texture_id, physicsClientId=self.client)
        self.bodies.append(body)
        self.surface_records.append({"name": name, "vertices_world_m": vertices.tolist(),
            "normal_world": normal.tolist(), "point_world_m": vertices[0].tolist(),
            "texture_sha256": hashlib.sha256(texture.read_bytes()).hexdigest(),
            "mesh_sha256": hashlib.sha256(obj.read_bytes()).hexdigest()})

    def _build(self):
        scene = self.case["scene"]
        if scene == "wall":
            self._quad("reference_wall", [(1.5, 2, -1.55), (1.5, -2, -1.55), (1.5, -2, 2.45), (1.5, 2, 2.45)])
            return
        self._quad("ground", [(-2, -2, 0), (5, -2, 0), (5, 2, 0), (-2, 2, 0)])
        if scene == "ground":
            return
        self._quad("left_wall", [(-2, 1.2, 0), (3.2, 1.2, 0), (3.2, 1.2, 2.5), (-2, 1.2, 2.5)])
        self._quad("right_wall", [(3.2, -1.2, 0), (-2, -1.2, 0), (-2, -1.2, 2.5), (3.2, -1.2, 2.5)])
        self._quad("end_wall", [(3.2, 1.2, 0), (3.2, -1.2, 0), (3.2, -1.2, 2.5), (3.2, 1.2, 2.5)])
        if scene == "paint":
            self._quad("contrast_surface", [(1.4, -1.2, .001), (1.9, -1.2, .001),
                                             (1.9, 1.2, .001), (1.4, 1.2, .001)], contrast=True)
        elif scene == "barrier":
            self._quad("contrast_surface", [(1.4, 1.2, 0), (1.4, -1.2, 0),
                                             (1.4, -1.2, 2.5), (1.4, 1.2, 2.5)], contrast=True)

    def render(self, sample, *, include_view_audit=False):
        if self.case["motion"] == "moving_wall":
            p.resetBasePositionAndOrientation(self.bodies[0], sample["surface_shift_m"], [0, 0, 0, 1],
                                             physicsClientId=self.client)
        images, views = [], {}
        origin = np.asarray(sample["position_m"])
        projection = p.computeProjectionMatrixFOV(sample["rendered_fov_deg"], 1, .05, 30)
        for name in ANGLES:
            forward, up, basis = view_basis(sample, name)
            view = p.computeViewMatrix(origin.tolist(), (origin + forward).tolist(), up.tolist())
            pixels = p.getCameraImage(128, 128, view, projection, renderer=p.ER_TINY_RENDERER,
                flags=p.ER_NO_SEGMENTATION_MASK, lightAmbientCoeff=1., lightDiffuseCoeff=0.,
                lightSpecularCoeff=0., shadow=0, physicsClientId=self.client)[2]
            images.append(np.asarray(pixels, np.uint8).reshape(128, 128, 4)[..., :3])
            views[name] = {"origin_m": origin.tolist(), "forward_world": forward.tolist(),
                           "up_world": up.tolist(), "optical_to_world": basis.tolist()}
        panel = np.concatenate((np.concatenate(images[:2], axis=1), np.concatenate(images[2:], axis=1)), axis=0)
        return (panel, views) if include_view_audit else panel


def generate_sequence(case, output_dir):
    case = validate_case(case)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=False)
    public, audit = output / "public", output / "audit"
    (public / "frames").mkdir(parents=True)
    audit.mkdir()
    references, captures = [], []
    with ControlledWorld(case, audit / "assets") as world:
        surfaces = copy.deepcopy(world.surface_records)
        for index in range(5):
            sample = camera_sample(case, index)
            rgb, views = world.render(sample, include_view_audit=True)
            relative = f"frames/{index:04d}.png"
            Image.fromarray(rgb).save(public / relative)
            references.append({"frame_id": f"capture-{index}", "captured_sim_s": sample["captured_sim_s"],
                               "rgb_path": relative, "rgb_sha256": hashlib.sha256(rgb.tobytes()).hexdigest()})
            captures.append({**sample, "views": views})
    manifest = {"version": "rgb_probe_sequence_v1", "calibration_id": case["declared_calibration_id"], "frames": references}
    (public / "sequence.json").write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n")
    (audit / "case.json").write_text(json.dumps(case, indent=2, allow_nan=False) + "\n")
    truth = {"scope": "audit only; never passed to the RGB probe", "surfaces": surfaces, "captures": captures,
             "clock_domain": "scripted simulation capture labels, not wheel execution or real-time deadlines",
             "renderer": "PyBullet DIRECT TinyRenderer, visual meshes only, no robot or motor",
             "raster_intrinsics": "f=64/tan(FOV/2), principal point=(63.5,63.5); nominal probe uses (64,64)"}
    (audit / "truth.json").write_text(json.dumps(truth, indent=2, allow_nan=False) + "\n")
    return manifest


def plane_transfer(surface, first, last, view):
    """AUDIT ONLY: analytic plane-induced transfer, never a predictor input."""
    normal, point = np.asarray(surface["normal_world"]), np.asarray(surface["point_world_m"])
    _, _, first_basis = view_basis(first, view)
    _, _, last_basis = view_basis(last, view)
    first_position, last_position = np.asarray(first["position_m"]), np.asarray(last["position_m"])
    normal_first = first_basis.T @ normal
    distance = float(normal @ (point + np.asarray(first["surface_shift_m"]) - first_position))
    if abs(distance) < 1e-9:
        raise ValueError("Camera is on audit plane")
    rotation = last_basis.T @ first_basis
    shift = np.asarray(last["surface_shift_m"]) - np.asarray(first["surface_shift_m"])
    translation = last_basis.T @ (first_position - last_position + shift)
    def intrinsics(sample):
        f = 64 / math.tan(math.radians(sample["rendered_fov_deg"]) / 2)
        return np.array([[f, 0, 63.5], [0, f, 63.5], [0, 0, 1.]])
    return intrinsics(last) @ (rotation + np.outer(translation, normal_first) / distance) @ np.linalg.inv(intrinsics(first))
