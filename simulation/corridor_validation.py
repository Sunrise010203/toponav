"""Frozen-controller corridor stress study; no tuning, training, or extra sensor.

The world adapter changes only declared scenery/rendering. A serialized scoped
world-factory substitution reuses the original run loop without editing it.
Study execution success is NOT navigation success; see outcome denominators.
"""
import argparse
from collections import Counter
import copy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import re
import traceback
from unittest.mock import patch

import numpy as np
from PIL import Image
import pybullet as p

from roadvision.corridor_control import CorridorController, VisionConfig
from simulation import corridor_motion as motion

ROOT = Path(__file__).resolve().parents[1]
GROUPS = {"anchor", "geometry", "material", "lighting", "combined", "paint_alias", "obstacle"}


def _number(value, name, low=None, high=None):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"Invalid {name}")
    if (low is not None and value < low) or (high is not None and value > high):
        raise ValueError(f"Out of bounds: {name}")
    return value


def _vector(value, name, rgb=False):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"Expected three components: {name}")
    return [_number(v, name, 0 if rgb else None, 1 if rgb else None) for v in value]


def validate_case(raw):
    if not isinstance(raw, dict) or set(raw) != {"id", "group", "scene", "appearance"}:
        raise ValueError("Only id/group/scene/appearance are allowed, no control overrides")
    if not isinstance(raw["id"], str) or not re.fullmatch(r"[a-z0-9_]{1,64}", raw["id"]):
        raise ValueError("Invalid case ID")
    if raw["group"] not in GROUPS or not isinstance(raw["scene"], dict):
        raise ValueError("Invalid case group/scene")
    if set(raw["scene"]) - set(motion.CorridorConfig.__dataclass_fields__):
        raise ValueError("Unknown scene field")
    config = motion.CorridorConfig(**raw["scene"])
    appearance = raw["appearance"]
    allowed = {"floor_rgb", "wall_rgb", "barrier_rgb", "barrier_front_m", "lighting", "paint"}
    if not isinstance(appearance, dict) or set(appearance) - allowed:
        raise ValueError("Unknown appearance field")
    normalized = {
        "floor_rgb": _vector(appearance.get("floor_rgb", [.20, .29, .40]), "floor_rgb", True),
        "wall_rgb": _vector(appearance.get("wall_rgb", [.65, .62, .58]), "wall_rgb", True),
        "lighting": {"lightDirection": [1, -1, 3]}, "paint": None,
        "barrier_front_m": None, "barrier_rgb": None,
    }
    lighting = appearance.get("lighting", {})
    light_fields = {"lightDirection", "lightColor", "lightAmbientCoeff", "lightDiffuseCoeff",
                    "lightSpecularCoeff", "shadow"}
    if not isinstance(lighting, dict) or set(lighting) - light_fields:
        raise ValueError("Invalid lighting keys")
    for key, value in lighting.items():
        if key in {"lightDirection", "lightColor"}:
            value = _vector(value, key, key == "lightColor")
            if key == "lightDirection" and not any(value):
                raise ValueError("Zero light direction")
        elif key == "shadow":
            if type(value) is not int or value not in (0, 1):
                raise ValueError("Invalid shadow flag")
        else:
            value = _number(value, key, 0, 1)
        normalized["lighting"][key] = value
    if "barrier_front_m" in appearance or "barrier_rgb" in appearance:
        if config.obstacle == "none":
            raise ValueError("Barrier appearance requires an obstacle")
        if "barrier_front_m" in appearance:
            normalized["barrier_front_m"] = _number(appearance["barrier_front_m"], "barrier front", 1.8, config.length_m - .5)
        if "barrier_rgb" in appearance:
            normalized["barrier_rgb"] = _vector(appearance["barrier_rgb"], "barrier_rgb", True)
    if "paint" in appearance:
        paint = appearance["paint"]
        if not isinstance(paint, dict) or set(paint) != {"x_start", "x_end", "y_center", "width_m", "rgb"}:
            raise ValueError("Invalid paint fields")
        paint = copy.deepcopy(paint)
        for key in ("x_start", "x_end", "y_center", "width_m"):
            _number(paint[key], key)
        if not (0 <= paint["x_start"] < paint["x_end"] <= config.length_m and paint["width_m"] > 0):
            raise ValueError("Invalid paint extent")
        if abs(paint["y_center"]) + paint["width_m"] / 2 > config.width_m / 2 + 1e-10:
            raise ValueError("Paint outside corridor")
        paint["rgb"] = _vector(paint["rgb"], "paint rgb", True)
        normalized["paint"] = paint
    return {"id": raw["id"], "group": raw["group"], "scene": asdict(config), "appearance": normalized}


def load_suite(path):
    data = json.loads(Path(path).read_text())
    if not isinstance(data, dict) or set(data) != {"version", "scope", "case_count", "novel_case_count", "cases"}:
        raise ValueError("Invalid suite fields")
    if data["version"] != "corridor_validation_v1" or not isinstance(data["cases"], list):
        raise ValueError("Unsupported suite")
    cases = [validate_case(case) for case in data["cases"]]
    if len({case["id"] for case in cases}) != len(cases):
        raise ValueError("Duplicate case IDs")
    anchors = sum(case["group"] == "anchor" for case in cases)
    if anchors != 1 or data["case_count"] != len(cases) or data["novel_case_count"] != len(cases) - anchors:
        raise ValueError("Invalid denominators / anchor count")
    return {**data, "cases": cases}


class ValidationWorld(motion.CorridorWorld):
    """Same dynamics and calibrated camera; only scenery / render light vary."""
    def __init__(self, config=None, appearance=None):
        config = config or motion.CorridorConfig()
        # Accept the raw appearance, not the already normalized defaults.
        self.appearance = validate_case({"id": "world", "group": "anchor", "scene": asdict(config),
                                         "appearance": appearance or {}})["appearance"]
        super().__init__(config)
        try:
            paint = self.appearance["paint"]
            self.paint_body = None
            if paint is not None:
                visual = p.createVisualShape(p.GEOM_BOX,
                    halfExtents=((paint["x_end"]-paint["x_start"])/2, paint["width_m"]/2, .0005),
                    rgbaColor=(*paint["rgb"], 1), physicsClientId=self.client)
                # Visual paint only: no physical wall / curb / collision shape.
                self.paint_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                    baseVisualShapeIndex=visual,
                    basePosition=((paint["x_start"]+paint["x_end"])/2, paint["y_center"], .001),
                    physicsClientId=self.client)
        except Exception:
            self.close()
            raise

    def _box(self, half, center, color):
        color = self.appearance["floor_rgb"] if center[2] < 0 else self.appearance["wall_rgb"]
        if self.config.obstacle != "none" and tuple(center) == (3.05, 0, .8):
            front = self.appearance["barrier_front_m"]
            if front is not None:
                center = (front + half[0], center[1], center[2])
            if self.appearance["barrier_rgb"] is not None:
                color = self.appearance["barrier_rgb"]
        return super()._box(half, center, color)

    def capture(self):
        # Exactly the original mounting / calibration / panel order. The
        # baseline parity test compares both pixels and physics after motion.
        position, orientation = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
        origin, camera_q = p.multiplyTransforms(position, orientation, (.20, 0, .29), (0, 0, 0, 1))
        matrix = np.asarray(p.getMatrixFromQuaternion(camera_q)).reshape(3, 3)
        projection = p.computeProjectionMatrixFOV(90, 1, .05, 30)
        images = []
        for angle in (0, math.pi/2, -math.pi/2, math.pi):
            forward = matrix @ np.array((math.cos(angle), math.sin(angle), 0))
            view = p.computeViewMatrix(origin, (np.array(origin)+forward).tolist(), matrix[:, 2].tolist())
            pixels = p.getCameraImage(128, 128, view, projection, renderer=p.ER_TINY_RENDERER,
                physicsClientId=self.client, **self.appearance["lighting"])[2]
            images.append(np.asarray(pixels, dtype=np.uint8).reshape(128, 128, 4)[..., :3])
        rgb = np.concatenate((np.concatenate(images[:2], axis=1), np.concatenate(images[2:], axis=1)), axis=0)
        return {"frame_id": f"physics-frame-{self.steps}", "captured_sim_s": self.sim_s, "rgb": rgb}


def verify_lock(path):
    lock = json.loads(Path(path).read_text())
    required = {"roadvision/corridor_control.py", "simulation/corridor_motion.py", "simulation/corridor_validation.py"}
    if not required <= set(lock["files"]):
        raise ValueError("Incomplete source freeze")
    for name, expected in lock["files"].items():
        if Path(name).is_absolute() or not (ROOT / name).resolve().is_relative_to(ROOT):
            raise ValueError("Frozen file outside workspace")
        if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f"Frozen file changed: {name}")
    if lock["vision_config"] != asdict(VisionConfig()):
        raise ValueError("Frozen policy defaults changed")
    return lock


def classify_run(record):
    """Post-run truth audit only. Refusal and visually plausible stops differ."""
    audit, final = record["audit"], record["audit"]["final"]
    distance = record["scene_config"]["length_m"] - final["camera_position_m"][0]
    conditions = {"visual_stop": record["state"] == "visual_stop", "end_distance": .9 <= distance <= 1.7,
        "lateral": abs(final["position_m"][1]) < .2, "yaw": abs(final["yaw_deg"]) < 8,
        "speed": final["speed_m_s"] < .03, "yaw_rate": abs(final["yaw_rate_rad_s"]) < .03,
        "contact": audit["wall_contact_physics_steps"] == 0}
    safe = all(conditions[key] for key in ("speed", "yaw_rate", "contact"))
    if not safe:
        outcome = "unsafe"
    elif all(conditions.values()):
        outcome = "end_wall_verified"
    elif record["state"] == "visual_stop":
        outcome = "visual_stop_unverified"
    elif record["state"] == "aborted":
        outcome = "safe_rejection"
    else:
        raise ValueError("Primitive did not reach a terminal state")
    return {"outcome": outcome, "physically_safe_in_this_run": safe,
            "endpoint_gate_failures": [key for key, value in conditions.items() if not value]}


def _raw_appearance(normalized):
    # Omit absent optional values; raw validation deliberately rejects nulls.
    return {key: copy.deepcopy(value) for key, value in normalized.items() if value is not None}


def run_case(case, output_dir):
    config = motion.CorridorConfig(**case["scene"])
    appearance = _raw_appearance(case["appearance"])
    factory = lambda config: ValidationWorld(config, appearance)
    # Evaluation-only dependency substitution; never run concurrently in this
    # process. The original controller and run loop remain byte-identical.
    with patch.object(motion, "CorridorWorld", factory):
        record = motion.run_corridor(config, output_dir=output_dir)
    with (Path(output_dir) / "case.json").open("x") as file:
        json.dump(case, file, indent=2, allow_nan=False)
    return record


def audit_case_evidence(directory, policy):
    """Replay saved RGB, not poses, through a fresh frozen controller."""
    directory = Path(directory)
    record = json.loads((directory / "run.json").read_text())
    if record["vision_config"] != policy or record["fault"] != "none":
        raise ValueError("Policy / sensor conditions drifted")
    if not record["steps"]:
        raise ValueError("Empty evidence")
    controller = CorridorController()
    for step in record["steps"]:
        rgb = np.asarray(Image.open(directory / step["frame_path"]).convert("RGB"))
        if hashlib.sha256(rgb.tobytes()).hexdigest() != step["frame_sha256"]:
            raise ValueError("Saved image hash mismatch")
        frame = {"frame_id": step["frame_id"], "captured_sim_s": step["captured_sim_s"], "rgb": rgb}
        if step["captured_sim_s"] != step["audit_before"]["sim_s"]:
            raise ValueError("Unexpected clock skew in no-fault study")
        output = controller.update(frame, frame["captured_sim_s"])
        if output != step["controller"]:
            raise ValueError("RGB-only replay differs from executed control")
        v, omega = output["v_m_s"], output["omega_rad_s"]
        for actual, expected in zip(step["wheel_targets_rad_s"], ((v-omega*.15)/.08, (v+omega*.15)/.08)):
            if not math.isclose(actual, expected, abs_tol=1e-12):
                raise ValueError("Wheel command mismatch")
        if output["state"] in {"braking", "aborted", "visual_stop"} and (v != 0 or omega != 0):
            raise ValueError("Nonzero terminal/braking command")
    if controller.state != record["state"] or controller.reason != record["reason"]:
        raise ValueError("Terminal state does not match image evidence")
    audit = classify_run(record)
    if record["audit"]["end_wall_verified"] != (audit["outcome"] == "end_wall_verified"):
        raise ValueError("Endpoint audit drift")
    if record["feedback"]["atomic_edge_compatible"] is not False:
        raise ValueError("Incorrect atomic feedback bridge")
    return {"frames_verified_and_replayed": len(record["steps"]), **audit}


def run_study(suite_path, lock_path, output_dir):
    suite, lock = load_suite(suite_path), verify_lock(lock_path)
    if hashlib.sha256(Path(suite_path).read_bytes()).hexdigest() != lock["suite_sha256"]:
        raise ValueError("Unfrozen suite")
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=False)
    rows = []
    for case in suite["cases"]:
        try:
            record = run_case(case, output_dir / case["id"])
            evidence = audit_case_evidence(output_dir / case["id"], lock["vision_config"])
            row = {"id": case["id"], "group": case["group"], **evidence,
                "state": record["state"], "reason": record["reason"], "feedback": record["feedback"],
                "primitive_exit_code_equivalent": 0 if record["audit"]["end_wall_verified"] else 1,
                "wall_elapsed_s": record["wall_elapsed_s"], "audit": record["audit"]}
        except Exception:
            row = {"id": case["id"], "group": case["group"], "outcome": "execution_error",
                   "error": traceback.format_exc()}
            with (output_dir / f"{case['id']}.error.json").open("x") as file:
                json.dump(row, file, indent=2)
        rows.append(row)
        print(case["id"], row["outcome"], row.get("reason", ""), flush=True)
    verify_lock(lock_path)
    novel = [row for row in rows if row["group"] != "anchor"]
    groups = {group: {"total": sum(row["group"] == group for row in rows),
                     "outcomes": dict(Counter(row["outcome"] for row in rows if row["group"] == group))}
              for group in sorted(GROUPS)}
    summary = {"version": suite["version"], "scope": suite["scope"],
        "study_executed": all(row["outcome"] != "execution_error" for row in rows),
        "suite_sha256": lock["suite_sha256"], "lock": lock,
        "all_cases": {"total": len(rows), "outcomes": dict(Counter(row["outcome"] for row in rows))},
        "new_conditions_only": {"total": len(novel), "outcomes": dict(Counter(row["outcome"] for row in novel))},
        "groups": groups, "cases": rows,
        "interpretation": "Exit 0 means study executed with intact evidence, NOT all cases navigated successfully. No tuning/retries."}
    with (output_dir / "summary.json").open("x") as file:
        json.dump(summary, file, indent=2, allow_nan=False)
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", type=Path, required=True)
    parser.add_argument("--lock", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="Must not already exist")
    args = parser.parse_args(argv)
    try:
        summary = run_study(args.suite, args.lock, args.output)
    except (ValueError, OSError, KeyError, TypeError) as error:
        print(json.dumps({"study_executed": False, "error": str(error)}))
        return 2
    print(json.dumps({key: summary[key] for key in ("study_executed", "all_cases", "new_conditions_only")}))
    return 0 if summary["study_executed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
