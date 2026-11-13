import numpy as np
import pytest

from roadvision.contract import CLASS_IDS, to_observation
from roadvision.scene import LAYOUTS, World, layout_fingerprint, topology_pair
from roadvision.data import validate_manifest


def test_fixed_classes_and_direction_mapping():
    assert len(CLASS_IDS) == 17
    for i in range(16):
        probabilities = [0.0] * 17
        probabilities[i] = 1.0
        result = to_observation(probabilities)
        assert set(result) == {"relative_open", "confidence"}
        assert list(result["relative_open"].values()) == [bit == "1" for bit in f"{i:04b}"]


@pytest.mark.parametrize("kind", ["low", "unknown", "missing", "stale"])
def test_uncertain_observations_are_not_closed_exits(kind):
    probabilities = [1 / 17] * 17 if kind == "low" else [0.0] * 17
    if kind != "low":
        probabilities[16 if kind == "unknown" else 15] = 1.0
    result = to_observation(probabilities, valid=kind != "missing", age_s=2 if kind == "stale" else 0)
    assert result == {"relative_open": dict.fromkeys(("front", "left", "right", "back")), "confidence": 0.0}


def test_layouts_are_not_renamed_copies():
    assert len({layout_fingerprint(layout) for layout in LAYOUTS.values()}) == len(LAYOUTS)
    assert {layout["split"] for layout in LAYOUTS.values()} == {"train", "val", "test"}
    assert sum(layout["split"] == "train" for layout in LAYOUTS.values()) >= 2


def test_manifest_rejects_layout_or_batch_leakage():
    good = [{"layout_id": "a", "batch_id": "a1", "split": "train", "path": "a.png", "class_index": 0},
            {"layout_id": "b", "batch_id": "b1", "split": "val", "path": "b.png", "class_index": 1},
            {"layout_id": "c", "batch_id": "c1", "split": "test", "path": "c.png", "class_index": 2}]
    validate_manifest(good)
    with pytest.raises(ValueError):
        validate_manifest(good + [{**good[0], "split": "test", "path": "d.png"}])
    with pytest.raises(ValueError):
        validate_manifest(good + [{**good[0], "layout_id": "d", "split": "test", "path": "e.png"}])


def test_rendered_sensor_labels_match_physical_rays():
    pytest.importorskip("pybullet")
    with World("train_grid") as world:
        for mask in (0, 1, 3, 9, 13, 15):
            image, audit = world.sample(mask, np.random.default_rng(mask))
            assert image.shape == (256, 256, 3)
            assert image.dtype == np.uint8
            assert audit["ray_open"] == [bit == "1" for bit in f"{mask:04b}"]
            assert image.std() > 5


def test_unknown_samples_use_actual_camera_views():
    pytest.importorskip("pybullet")
    with World("val_branch") as world:
        image, audit = world.sample(16, np.random.default_rng(2), unknown_mode="turning")
        assert image.std() > 5
        assert audit["phase"] == "turning"


def test_shared_backbone_stays_frozen_and_has_fixed_output():
    import torch
    from roadvision.learn import RoadModel
    torch.set_num_threads(2)
    model = RoadModel().train()
    assert not model.features.training
    assert all(not parameter.requires_grad for parameter in model.features.parameters())
    assert all(parameter.requires_grad for parameter in model.head.parameters())
    with torch.no_grad():
        assert model(torch.zeros(1, 4, 3, 224, 224)).shape == (1, 17)


def test_predictor_packet_and_rejection_reasons(tmp_path):
    import hashlib
    import json
    import torch
    from roadvision.learn import Predictor, RoadModel
    from PIL import Image
    torch.set_num_threads(2)
    model = RoadModel().eval()
    with torch.no_grad():
        model.head[1].weight.zero_()
        model.head[1].bias.zero_()
        model.head[1].bias[15] = 10
    torch.save(model.state_dict(), tmp_path / "model.pt")
    (tmp_path / "metadata.json").write_text(json.dumps({
        "class_ids": CLASS_IDS, "threshold": 0.6,
        "model_sha256": hashlib.sha256((tmp_path / "model.pt").read_bytes()).hexdigest()}))
    predictor = Predictor(tmp_path)
    image = Image.new("RGB", (256, 256), (120, 120, 120))
    packet = predictor.predict(image, frame_id="test-frame")
    assert packet["frame_id"] == "test-frame"
    assert len(packet["probabilities"]) == 17
    assert packet["accepted"] and packet["rejection_reason"] is None
    assert set(packet["observation"]) == {"relative_open", "confidence"}
    assert predictor.predict(image, captured_at_ns=0)["rejection_reason"] == "stale_frame"
    assert predictor.predict(Image.new("RGB", (256, 256)))["rejection_reason"] == "missing_view"
    with torch.no_grad():
        predictor.model.head[1].bias.zero_()
    assert predictor.predict(image)["rejection_reason"] == "low_confidence"
    with torch.no_grad():
        predictor.model.head[1].bias[16] = 10
    assert predictor.predict(image)["rejection_reason"] == "unobservable"


@pytest.mark.parametrize("layout", list(LAYOUTS))
def test_map_size_does_not_change_class_dimension(layout):
    graph, scene, cells = topology_pair(layout)
    assert len(graph["nodes"]) == len(cells) == len(LAYOUTS[layout]["cells"])
    assert len(scene["channels"]) == len(graph["edges"])
    assert len(CLASS_IDS) == 17


def test_image_driver_never_calls_ideal_sensor(monkeypatch):
    pytest.importorskip("pybullet")
    simulator = pytest.importorskip("simulation.structural_navigation")
    from roadvision.navigation import run_navigation
    def forbidden(*args, **kwargs):
        raise AssertionError("Ideal observation sensor must not be used")
    monkeypatch.setattr(simulator.StructuralEnvironment, "observe", forbidden)
    class LowConfidence:
        def predict(self, image, captured_at_ns, frame_id):
            assert image.shape == (256, 256, 3)
            assert captured_at_ns > 0 and frame_id
            return {"observation": to_observation([1/17]*17)}
    result = run_navigation(LowConfidence(), "test_locked", "R000", 0, "R007")
    assert result["status"] == "observation_uncertain"
    assert result["steps"][0]["decision"]["action"] == "stop"
    assert result["audit"]["final_truth"]["node_id"] == "R000"
    assert result["steps"][0]["localization"]["candidates"] == [{"node_id": "R000", "heading_deg": 0}]
