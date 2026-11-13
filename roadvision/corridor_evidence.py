"""Design-stage RGB evidence contract and motion/arrival semantic separation.

This module checks declarations, NOT image geometry or producer authenticity.
It neither extracts image features nor controls a robot. Every review leaves
motion unauthorized and topology arrival unverified. No existing entrypoint
uses this prototype; geometry producers remain future work.
"""
import math
import re


VERSION = "corridor_evidence_v1"
MAX_FRAME_AGE_S = 0.25  # Same simulation-clock envelope as the frozen primitive.
VIEWS = {"front", "left", "right", "back"}


def _fields(value, names):
    if not isinstance(value, dict) or set(value) != set(names):
        raise ValueError(f"Expected only fields: {', '.join(names)}")


def _identity(value):
    if not isinstance(value, str) or not value.strip() or len(value) > 256:
        raise ValueError("Invalid evidence/frame/calibration identity")


def _number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("Expected a finite JSON number, not bool")


def _views(value):
    if not isinstance(value, list) or any(not isinstance(v, str) or v not in VIEWS for v in value):
        raise ValueError("Invalid supporting views")
    if len(set(value)) != len(value):
        raise ValueError("Duplicate supporting view")


def review_corridor_evidence(packet, *, now_s, calibration_id):
    """Validate one declared packet; this is not cross-packet event dedup.

    Temporal geometry requires distinct recent capture times and image hashes.
    Those are necessary metadata checks, never proof of actual parallax,
    correct planes, trusted timestamps, or sufficient texture.
    """
    _fields(packet, ("version", "evidence_id", "calibration_id", "frames", "appearance_boundary",
                     "corridor_boundary", "front_surface"))
    _number(now_s)
    if now_s < 0 or packet["version"] != VERSION:
        raise ValueError("Invalid simulated clock / contract version")
    _identity(packet["evidence_id"])
    _identity(packet["calibration_id"])
    _identity(calibration_id)
    if packet["calibration_id"] != calibration_id:
        raise ValueError("Calibration mismatch")
    frames = packet["frames"]
    if not isinstance(frames, list) or not 1 <= len(frames) <= 16:
        raise ValueError("Expected 1..16 bounded frame references")
    seen, hashes, previous_capture = set(), set(), -1.0
    for frame in frames:
        _fields(frame, ("frame_id", "captured_sim_s", "rgb_sha256"))
        _identity(frame["frame_id"])
        captured = frame["captured_sim_s"]
        _number(captured)
        if captured < 0 or captured > now_s or now_s - captured > MAX_FRAME_AGE_S:
            raise ValueError("Negative/future/stale capture time")
        if frame["frame_id"] in seen or captured <= previous_capture:
            raise ValueError("Replayed/unordered frame reference")
        digest = frame["rgb_sha256"]
        if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ValueError("Expected SHA256 of actual RGB panel bytes")
        seen.add(frame["frame_id"])
        hashes.add(digest)
        previous_capture = captured

    appearance = packet["appearance_boundary"]
    _fields(appearance, ("state", "method"))
    if (appearance["state"], appearance["method"]) not in (("unknown", "none"), ("candidate", "rgb_colour")):
        raise ValueError("Invalid appearance provenance")

    corridor = packet["corridor_boundary"]
    _fields(corridor, ("state", "method", "views", "offset_m", "heading_rad"))
    _views(corridor["views"])
    if corridor["state"] == "unknown":
        if (corridor["method"] != "none" or corridor["views"] or corridor["offset_m"] is not None
                or corridor["heading_rad"] is not None):
            raise ValueError("Unknown corridor geometry must have null metrics / no supporting claim")
    elif corridor["state"] == "candidate":
        if corridor["method"] != "rgb_temporal_geometry" or not {"left", "right"} <= set(corridor["views"]):
            raise ValueError("Corridor candidate needs geometric two-sided support, not colour")
        _number(corridor["offset_m"])
        _number(corridor["heading_rad"])
        if abs(corridor["heading_rad"]) > math.pi:
            raise ValueError("Invalid heading range")
    else:
        raise ValueError("Unknown corridor state")

    front = packet["front_surface"]
    _fields(front, ("state", "method", "views", "range_m"))
    _views(front["views"])
    if front["state"] == "unknown":
        if front["method"] != "none" or front["views"] or front["range_m"] is not None:
            raise ValueError("Unknown front surface must have null range / no supporting claim")
    elif front["state"] in ("ground_consistent", "non_ground_candidate"):
        if front["method"] != "rgb_temporal_geometry" or "front" not in front["views"]:
            raise ValueError("Front surface needs front-view geometry, not colour")
        if front["state"] == "ground_consistent" and front["range_m"] is not None:
            raise ValueError("A ground transition is not an obstacle distance")
        if front["range_m"] is not None:
            _number(front["range_m"])
            if front["range_m"] <= 0:
                raise ValueError("Invalid candidate surface range")
    else:
        raise ValueError("Unknown front surface state")
    if (corridor["state"] == "candidate" or front["state"] != "unknown") and (len(frames) < 2 or len(hashes) < 2):
        raise ValueError("Temporal geometry needs multiple distinct captures/images, not static relabeling")

    notes = []
    if appearance["state"] == "candidate" and corridor["state"] == "unknown":
        notes.append("appearance_is_not_corridor_geometry")
    if front["state"] == "ground_consistent":
        notes.append("ground_transition_is_not_wall")
    elif front["state"] == "non_ground_candidate":
        notes.append("surface_is_not_endpoint_identity")
    notes.append("corridor_geometry_unverified" if corridor["state"] == "unknown" else
                 "candidate_requires_validated_producer_and_controller")
    return {"corridor_geometry": corridor["state"], "front_surface": front["state"], "review_notes": notes,
            "motion_authorized": False, "arrival_status": "unverified", "atomic_edge_compatible": False}


def split_motion_feedback(feedback):
    """Interpret only the original public motion payload, never audit/pose.

    No automatic legacy succeeded/failed/unknown conversion is sound for an
    intermediate corridor pose. Even no commanded motion proves no node ID.
    Command/event binding and cross-event dedup remain future integration work.
    """
    _fields(feedback, ("status", "atomic_edge_compatible", "motion_was_commanded"))
    if feedback["atomic_edge_compatible"] is not False or type(feedback["motion_was_commanded"]) is not bool:
        raise ValueError("Expected non-atomic motion feedback with a boolean command flag")
    mapping = {"completed_visual_stop": "visual_stop_supported",
               "aborted_motion_uncertain": "motion_aborted_uncertain", "rejected_no_motion": "not_commanded"}
    status, commanded = feedback["status"], feedback["motion_was_commanded"]
    if not isinstance(status, str) or status not in mapping:
        raise ValueError("No legacy atomic status is accepted")
    if (status == "rejected_no_motion" and commanded) or (status == "aborted_motion_uncertain" and not commanded):
        raise ValueError("Contradictory motion feedback")
    return {"motion_outcome": mapping[status], "pose_status": "unverified", "arrival_status": "unverified",
            "atomic_edge_compatible": False, "legacy_feedback": None}
