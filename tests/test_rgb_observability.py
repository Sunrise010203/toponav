"""Renderer/experiment infrastructure, never probe-threshold selection."""
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pybullet as p
import pytest

from simulation.rgb_observability import (
    ControlledWorld, camera_sample, generate_sequence, load_suite, make_texture,
    plane_transfer, validate_case,
)


SUITE = Path(__file__).resolve().parents[1] / "configs/evaluation/rgb_observability_v1.json"


def case(**changes):
    raw = copy.deepcopy(json.loads(SUITE.read_text())["cases"][0])
    raw.update(changes)
    return raw


def test_predeclared_twenty_cases_and_no_override_keys():
    suite = load_suite(SUITE)
    assert len(suite["cases"]) == 20 and suite["frame_count"] == 5
    assert len({c["id"] for c in suite["cases"]}) == 20


@pytest.mark.parametrize("change", [
    {"id": "../escape"}, {"scene": "other"}, {"texture": "other"}, {"motion": "other"},
    {"seed": True}, {"seed": -1}, {"rendered_fov_deg": float("nan")}, {"pitch_rad": float("inf")},
    {"audit": {}}, {"control_threshold": .1}, {"declared_calibration_id": ""},
    {"scene": "room", "motion": "moving_wall"},
])
def test_bad_case_rejected_without_tuning(change):
    with pytest.raises(ValueError):
        validate_case(case(**change))


def test_duplicate_cases_and_invalid_schedule_rejected(tmp_path):
    data = json.loads(SUITE.read_text())
    data["cases"][1]["id"] = data["cases"][0]["id"]
    path = tmp_path / "suite.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_suite(path)
    data = json.loads(SUITE.read_text())
    data["capture_dt_s"] = .2
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_suite(path)


def test_textures_are_deterministic_nonuniform_and_not_screen_noise():
    unique = make_texture("unique", 104)
    assert unique.shape == (512, 512, 3) and unique.dtype == np.uint8
    assert np.array_equal(unique, make_texture("unique", 104))
    assert not np.array_equal(unique, make_texture("unique", 701))
    assert np.std(unique) > 5
    assert np.std(make_texture("uniform", 104)) == 0
    repeated = make_texture("repeated", 104)
    assert np.array_equal(repeated[:-64], repeated[64:])


@pytest.mark.parametrize("motion,delta", [("forward", .4), ("slow_forward", .14), ("tiny_forward", .002)])
def test_scripted_translation_is_declared_not_wheel_motion(motion, delta):
    a, b = camera_sample(case(motion=motion), 0), camera_sample(case(motion=motion), 4)
    assert b["position_m"][0] - a["position_m"][0] == pytest.approx(delta)
    assert b["captured_sim_s"] == pytest.approx(.4)


def test_yaw_and_dynamic_wall_separate_camera_from_surface():
    a, b = camera_sample(case(motion="yaw"), 0), camera_sample(case(motion="yaw"), 4)
    assert a["position_m"] == b["position_m"]
    assert b["yaw_rad"] == pytest.approx(.1)
    a = camera_sample(case(motion="moving_wall"), 4)
    assert a["position_m"] == [0, 0, .45] and a["surface_shift_m"] == [-.4, 0, 0]


def test_rendered_texture_really_changes_pixels_and_static_is_identical(tmp_path):
    with ControlledWorld(case(motion="static"), tmp_path / "unique") as world:
        first = world.render(camera_sample(case(motion="static"), 0))
        second = world.render(camera_sample(case(motion="static"), 1))
        assert np.array_equal(first, second)
        assert first.shape == (256, 256, 3) and first.dtype == np.uint8
        assert np.std(first[:128, :128]) > 5
        assets_before = {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in world.assets.glob('*')}
        world.render(camera_sample(case(motion="forward"), 2))
        assert assets_before == {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in world.assets.glob('*')}
    with ControlledWorld(case(texture="uniform"), tmp_path / "uniform") as world:
        uniform = world.render(camera_sample(case(texture="uniform"), 0))
    assert not np.array_equal(first, uniform)


def test_camera_views_share_origin_and_known_panel_order(tmp_path):
    with ControlledWorld(case(), tmp_path / "assets") as world:
        sample = camera_sample(case(), 0)
        rgb, views = world.render(sample, include_view_audit=True)
        assert list(views) == ["front", "left", "right", "back"]
        assert all(v["origin_m"] == sample["position_m"] for v in views.values())
        assert views["front"]["forward_world"] == pytest.approx([1, 0, 0])
        assert views["left"]["forward_world"] == pytest.approx([0, 1, 0], abs=1e-12)
        assert views["right"]["forward_world"] == pytest.approx([0, -1, 0], abs=1e-12)
        assert rgb.shape == (256, 256, 3)


def test_public_sequence_has_no_audit_and_all_pixels_bind(tmp_path):
    out = tmp_path / "generated"
    generate_sequence(case(), out)
    public = json.loads((out / "public/sequence.json").read_text())
    assert set(public) == {"version", "calibration_id", "frames"}
    assert len(public["frames"]) == 5
    for ref in public["frames"]:
        assert set(ref) == {"frame_id", "captured_sim_s", "rgb_path", "rgb_sha256"}
        rgb = np.asarray(Image.open(out / "public" / ref["rgb_path"]))
        assert hashlib.sha256(rgb.tobytes()).hexdigest() == ref["rgb_sha256"]
    audit = json.loads((out / "audit/truth.json").read_text())
    assert len(audit["captures"]) == 5 and "surfaces" in audit
    with pytest.raises(FileExistsError):
        generate_sequence(case(), out)


def test_paint_is_horizontal_and_barrier_vertical(tmp_path):
    for scene, expected in (("paint", [0, 0, 1]), ("barrier", [-1, 0, 0])):
        with ControlledWorld(case(scene=scene), tmp_path / scene) as world:
            target = [s for s in world.surface_records if s["name"] == "contrast_surface"][0]
            assert target["normal_world"] == pytest.approx(expected)


def test_resource_closes_on_exception(tmp_path):
    client = None
    with pytest.raises(RuntimeError):
        with ControlledWorld(case(), tmp_path / "assets") as world:
            client = world.client
            raise RuntimeError("fixture")
    assert not p.isConnected(client)


def test_audit_plane_transfer_handles_static_translation_and_surface_motion(tmp_path):
    with ControlledWorld(case(), tmp_path / "assets") as world:
        surface = world.surface_records[0]
        a = camera_sample(case(), 0)
        h = plane_transfer(surface, a, a, "front")
        assert h / h[2, 2] == pytest.approx(np.eye(3), abs=1e-12)
        b = camera_sample(case(), 2)
        h = plane_transfer(surface, a, b, "front")
        assert h[0, 0] / h[2, 2] > 1  # approaching wall expands image
        dynamic = camera_sample(case(motion="moving_wall"), 2)
        dh = plane_transfer(surface, a, dynamic, "front")
        assert dh / dh[2, 2] == pytest.approx(h / h[2, 2], abs=1e-12)
