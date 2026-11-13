"""Frozen shared MobileNet training, held-out evaluation and image-only inference."""
import argparse
import copy
import csv
import hashlib
import json
import math
import platform
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from sklearn.metrics import accuracy_score, confusion_matrix
from torch import nn
from torch.utils.data import DataLoader, Dataset
from torchvision.models import MobileNet_V3_Small_Weights, mobilenet_v3_small

from roadvision.contract import CLASS_IDS, to_observation
from roadvision.data import validate_manifest


def prepare(image):
    image = image.convert("RGB") if isinstance(image, Image.Image) else Image.fromarray(image)
    if image.size != (256, 256):
        raise ValueError("Expected the synchronized 256x256 four-view panel")
    views = []
    valid = True
    for index in range(4):
        y, x = divmod(index, 2)
        view = image.crop((x*128, y*128, (x+1)*128, (y+1)*128))
        valid = valid and np.asarray(view).max() > 0
        array = np.asarray(view.resize((224, 224), Image.Resampling.BILINEAR), dtype=np.float32).copy()/255
        views.append(torch.from_numpy(array.transpose(2, 0, 1)))
    tensors = torch.stack(views)
    return (tensors-torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1))/torch.tensor(
        [0.229, 0.224, 0.225]).view(1, 3, 1, 1), bool(valid)


class RoadModel(nn.Module):
    def __init__(self, pretrained=False):
        super().__init__()
        backbone = mobilenet_v3_small(weights=MobileNet_V3_Small_Weights.DEFAULT if pretrained else None)
        self.features = backbone.features
        self.head = nn.Sequential(nn.Dropout(0.1), nn.Linear(576*4, 17))
        self.features.requires_grad_(False)
        self.features.eval()

    def train(self, mode=True):
        super().train(mode)
        self.features.eval()  # Never update frozen BatchNorm running statistics.
        return self

    def encode(self, inputs):
        batch = inputs.shape[0]
        features = self.features(inputs.reshape(-1, 3, 224, 224)).mean(dim=(-2, -1))
        return features.reshape(batch, 576*4)

    def forward(self, inputs):
        return self.head(self.encode(inputs))


class Frames(Dataset):
    def __init__(self, root, rows):
        self.root, self.rows = root, rows

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, index):
        row = self.rows[index]
        if hashlib.sha256((self.root/row["path"]).read_bytes()).hexdigest() != row["image_sha256"]:
            raise ValueError("Image checksum mismatch")
        with Image.open(self.root/row["path"]) as image:
            inputs, valid = prepare(image)
        return inputs, row["class_index"], valid


def extract(model, root, rows, device):
    tensors, labels, valid = [], [], []
    loader = DataLoader(Frames(root, rows), batch_size=32, shuffle=False, num_workers=0)
    model.eval()
    with torch.no_grad():
        for inputs, targets, validity in loader:
            tensors.append(model.encode(inputs.to(device)).cpu())
            labels.append(targets)
            valid.append(validity)
    return torch.cat(tensors).to(device), torch.cat(labels).to(device), torch.cat(valid).numpy()


def summary(labels, probabilities, valid, threshold):
    predictions = probabilities.argmax(1)
    accepted = (predictions != 16) & (probabilities.max(1) >= threshold) & valid
    return {"sample_count": len(labels), "accuracy": float(accuracy_score(labels, predictions)),
            "confusion_matrix": confusion_matrix(labels, predictions, labels=range(17)).tolist(),
            "support": np.bincount(labels, minlength=17).tolist(),
            "known_accuracy": float(accuracy_score(labels[labels != 16], predictions[labels != 16])),
            "accepted_count": int(accepted.sum()), "coverage": float(accepted.mean()),
            "accepted_accuracy": float((predictions[accepted] == labels[accepted]).mean()) if accepted.any() else None,
            "unknown_false_accept_count": int(((labels == 16) & accepted).sum()), "threshold": threshold}


def train(args):
    manifest_bytes = (args.data/"manifest.json").read_bytes()
    manifest = json.loads(manifest_bytes)
    validate_manifest(manifest["rows"])
    if manifest.get("preview_only", True) or tuple(manifest["class_ids"]) != CLASS_IDS:
        raise ValueError("Formal dataset with matching classes is required")
    args.run.mkdir(parents=True, exist_ok=False)
    torch.manual_seed(20261014)
    model = RoadModel(pretrained=True).to(args.device)
    selected = {split: [r for r in manifest["rows"] if r["split"] == split] for split in ("train", "val")}
    # Test image paths are never opened during training/validation feature extraction.
    started = time.perf_counter()
    train_x, train_y, _ = extract(model, args.data, selected["train"], args.device)
    val_x, val_y, val_valid = extract(model, args.data, selected["val"], args.device)
    print(f"Frozen features extracted in {time.perf_counter()-started:.1f}s", flush=True)
    optimizer = torch.optim.AdamW(model.head.parameters(), lr=0.003, weight_decay=0.0001)
    best_loss, best_head, best_epoch, history = float("inf"), None, None, []
    for epoch in range(args.epochs):
        model.train()
        permutation = torch.randperm(len(train_y), device=args.device)
        for ids in permutation.split(128):
            optimizer.zero_grad(set_to_none=True)
            loss = nn.functional.cross_entropy(model.head(train_x[ids]), train_y[ids])
            loss.backward()
            optimizer.step()
        model.eval()
        with torch.no_grad():
            logits = model.head(val_x)
            val_loss = nn.functional.cross_entropy(logits, val_y).item()
            val_accuracy = (logits.argmax(1) == val_y).float().mean().item()
        history.append({"epoch": epoch+1, "val_loss": val_loss, "val_accuracy": val_accuracy})
        if val_loss < best_loss:
            best_loss, best_head = val_loss, copy.deepcopy(model.head.state_dict())
            best_epoch = epoch+1
        if (epoch+1) % 10 == 0:
            print(f"Epoch {epoch+1}: val_accuracy={val_accuracy:.4f}, val_loss={val_loss:.4f}", flush=True)
    model.head.load_state_dict(best_head)
    model.eval()
    torch.save({key: value.cpu() for key, value in model.state_dict().items()}, args.run/"model.pt")
    with torch.no_grad():
        probabilities = model.head(val_x).softmax(1).cpu().numpy()
    metadata = {"class_ids": CLASS_IDS, "view_order": ["front", "left", "right", "back"],
                "architecture": "Shared MobileNetV3-Small features x4 + linear 17-class head",
                "backbone_frozen": True, "batchnorm_frozen": True,
                "pretrained_weights": "MobileNet_V3_Small_Weights.IMAGENET1K_V1",
                "preprocessing": "RGB 256x256 panel: TL front, TR left, BL right, BR back; split128, bilinear resize224, ImageNet mean/std",
                "seed": 20261014, "best_epoch": best_epoch,
                "label_policy": manifest["label_policy"], "max_frame_age_s": 1.0,
                "probability_note": "Uncalibrated softmax scores, not guaranteed correctness probabilities",
                "threshold": 0.6, "threshold_policy": "Fixed existing third-block minimum; not tuned on test",
                "epochs": args.epochs, "manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
                "model_sha256": hashlib.sha256((args.run/"model.pt").read_bytes()).hexdigest(),
                "device": args.device, "training_seconds": time.perf_counter()-started,
                "torch_version": torch.__version__, "python_version": platform.python_version(),
                "gpu": torch.cuda.get_device_name() if args.device == "cuda" else None,
                "peak_gpu_allocated_mb": torch.cuda.max_memory_allocated()/1024**2 if args.device == "cuda" else None,
                "validation": summary(val_y.cpu().numpy(), probabilities, val_valid, 0.6)}
    (args.run/"metadata.json").write_text(json.dumps(metadata, indent=2)+"\n")
    with (args.run/"history.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=history[0].keys())
        writer.writeheader()
        writer.writerows(history)
    print(json.dumps(metadata, indent=2), flush=True)


class Predictor:
    def __init__(self, run, device="cpu"):
        started = time.perf_counter()
        self.metadata = json.loads((run/"metadata.json").read_text())
        if tuple(self.metadata["class_ids"]) != CLASS_IDS:
            raise ValueError("Model class order mismatch")
        if hashlib.sha256((run/"model.pt").read_bytes()).hexdigest() != self.metadata["model_sha256"]:
            raise ValueError("Model checksum mismatch")
        self.device, self.model = device, RoadModel().to(device)
        self.model.load_state_dict(torch.load(run/"model.pt", map_location=device, weights_only=True))
        self.model.eval()
        self.loading_ms = (time.perf_counter()-started)*1000

    @staticmethod
    def _timestamp_ns(value):
        if type(value) is not int or value < 0:
            raise ValueError("Timestamp must be nonnegative integer Unix nanoseconds")
        # Preserve integer timestamps, without float coercion or retiming.
        # Keep source clock values within the finite numeric diagnostic domain.
        try:
            finite = math.isfinite(value)
        except OverflowError:
            finite = False
        if not finite:
            raise ValueError("Timestamp exceeds the finite diagnostic range")
        return value

    def predict(self, image, captured_at_ns=None, frame_id=None):
        captured_at_ns = self._timestamp_ns(time.time_ns() if captured_at_ns is None else captured_at_ns)
        started = time.perf_counter()
        inputs, valid = prepare(image)
        with torch.no_grad():
            probabilities = self.model(inputs.unsqueeze(0).to(self.device)).softmax(1)[0].cpu().tolist()
        ended = self._timestamp_ns(time.time_ns())
        age_s = (ended-captured_at_ns)/1e9
        observation = to_observation(probabilities, self.metadata["threshold"], valid, age_s)
        index = int(np.argmax(probabilities))
        reason = ("missing_view" if not valid else "invalid_timestamp" if age_s < 0 else
                  "stale_frame" if age_s > 1 else "unobservable" if index == 16 else
                  "low_confidence" if probabilities[index] < self.metadata["threshold"] else None)
        return {"frame_id": frame_id, "class_id": CLASS_IDS[index], "class_ids": CLASS_IDS,
                "probabilities": probabilities, "captured_at_ns": captured_at_ns,
                "predicted_at_ns": ended, "inference_ms": (time.perf_counter()-started)*1000,
                "frame_age_s": age_s, "rejection_reason": reason,
                "accepted": observation["confidence"] > 0, "observation": observation}


def benchmark(predictor, image):
    inputs, _ = prepare(image)
    inputs = inputs.unsqueeze(0).to(predictor.device)
    sync = torch.cuda.synchronize if predictor.device == "cuda" else lambda: None
    forward, full = [], []
    with torch.no_grad():
        for index in range(55):
            sync()
            started = time.perf_counter()
            predictor.model(inputs).softmax(1)
            sync()
            elapsed = (time.perf_counter()-started)*1000
            result = predictor.predict(image)
            if index >= 5:
                forward.append(elapsed)
                full.append(result["inference_ms"])
    stats = lambda values: {"mean_ms": float(np.mean(values)), "p50_ms": float(np.percentile(values, 50)),
                            "p95_ms": float(np.percentile(values, 95))}
    return {"device": predictor.device, "torch_version": torch.__version__, "threads": torch.get_num_threads(),
            "platform": platform.platform(), "processor": platform.processor(),
            "gpu": torch.cuda.get_device_name() if predictor.device == "cuda" else None,
            "batch_size": 1, "four_views_per_sample": 4, "warmup": 5, "repeats": 50,
            "loading_ms": predictor.loading_ms, "forward": stats(forward), "preprocess_and_forward": stats(full)}


def evaluate(args):
    manifest_bytes = (args.data/"manifest.json").read_bytes()
    manifest = json.loads(manifest_bytes)
    predictor = Predictor(args.run, args.device)
    if hashlib.sha256(manifest_bytes).hexdigest() != predictor.metadata["manifest_sha256"]:
        raise ValueError("Dataset manifest changed since model/threshold were locked")
    rows = [row for row in manifest["rows"] if row["split"] == "test"]
    features, labels, valid = extract(predictor.model, args.data, rows, args.device)
    with torch.no_grad():
        probabilities = predictor.model.head(features).softmax(1).cpu().numpy()
    report = summary(labels.cpu().numpy(), probabilities, valid, predictor.metadata["threshold"])
    with Image.open(args.data/rows[0]["path"]) as image:
        report["latency"] = benchmark(predictor, image.convert("RGB"))
    report["layout_ids"] = sorted({row["layout_id"] for row in rows})
    report["class_ids"] = CLASS_IDS
    report["scope"] = "Independent simulated layouts only; no real-world/geometric/control claim"
    destination = args.run/f"test_{args.device}.json"
    with destination.open("x") as file:
        json.dump(report, file, indent=2)
    with (args.run/f"test_predictions_{args.device}.csv").open("x", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["frame_id", "label", "prediction"]+list(CLASS_IDS))
        for row, values in zip(rows, probabilities):
            writer.writerow([row["frame_id"], CLASS_IDS[row["class_index"]], CLASS_IDS[int(values.argmax())]]+values.tolist())
    print(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("train", "evaluate"))
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    parser.add_argument("--epochs", type=int, default=80)
    args = parser.parse_args()
    torch.set_num_threads(2)
    (train if args.mode == "train" else evaluate)(args)


if __name__ == "__main__":
    main()
