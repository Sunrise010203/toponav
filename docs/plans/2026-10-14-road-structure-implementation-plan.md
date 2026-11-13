# Four-view Road Structure Implementation Plan

> Execute task-by-task with the executing-plans workflow. No Git repository is present; do not create commits or overwrite existing baselines.

**Goal:** Train a pretrained, frozen MobileNetV3-Small on physical simulator RGB observations, evaluate locked independent layouts, and provide the existing third-block observation contract.

**Architecture:** One shared MobileNetV3-Small extracts features from each of four ordered camera views. Concatenated view features feed one 17-class linear head; this preserves direction information that global pooling of a tiled panel can lose. Rendering/label generation own scene truth; inference sees images only, while the controller receives only relative_open/confidence.

**Tech Stack:** Existing Python 3.11 CUDA environment for training; PyBullet TinyRenderer in the existing Python 3.10 Linux environment for capture/deployment; PyTorch/torchvision; Pillow; sklearn. Official PyBullet 3.2.7 wheels exist for Linux but not Windows, so no Windows source compilation is attempted.

## Task 1 — Contract, renderer and isolated formal data

- [x] Write `tests/test_roadvision.py`: class mapping, low confidence/unknown, stale/missing views, topology-disjoint split manifest, renderer ray/label agreement.
- [x] Run `.venv/Scripts/python.exe -m pytest tests/test_roadvision.py -q`; expect missing roadvision module before implementation.
- [x] Create `roadvision/contract.py`, `roadvision/scene.py`, `roadvision/data.py`; add isolated simulator dependency requirement. Preserve all existing source files and models.
- [x] Install PyBullet from official PyPI in the existing remote project environment; do not change GPU packages or system packages. Copy generated images to the local environment for training.
- [x] Render and visually inspect sample views, then generate a new versioned dataset directory. Minimum two training layouts, one validation layout, one locked final test layout. Batch/split and topology fingerprints must be checked before training.
- [x] Run the tests again; require all contract/render/split assertions to pass. Random tensor checks remain diagnostics, never dataset entries.

## Task 2 — Frozen training, model selection and locked evaluation

- [x] Create `roadvision/learn.py`: four-view shared pretrained backbone, frozen BatchNorm, feature extraction, classifier-only training, validation-only checkpoint choice.
- [x] Save a new run directory with weights, class order, preprocessing, split hashes, history and fixed rejection threshold >=0.6. Never overwrite an old model.
- [x] Freeze chosen weights/configuration before opening final test images. Report full class support, 17-class confusion matrix, raw accuracy, rejection coverage and accepted accuracy separately.
- [x] Benchmark batch=1 after warmup; report average/P50/P95 for model forward and preprocessing+forward. State hardware and loading time separately.
- [x] If validation is insufficient, report evidence before considering partial fine-tuning; never decide based on final test results.

## Task 3 — Remote adapter validation and delivery

- [x] Create a separate `roadvision/navigation.py` adapter/driver rather than overwrite the third-block or legacy simulator.
- [x] Upload only new modules, model and needed versioned artifacts, refusing to overwrite existing destinations. Check transfer hashes.
- [x] Verify identical CPU predictions locally/remotely within tolerance and strict third-block payload validation; run selected image-driven reachable tasks and low-confidence/stale-frame stopping checks.
- [x] Measure CPU latency on the VM separately from GPU latency. Run existing remote regression tests again and compare preserved source hashes.
- [x] Save actual results and limitations. Do not claim geometric estimation, real-world transfer, complete control or high navigation performance without measurements.

## Measured outcome (2026-10-14)

All three implementation tasks verified. See `D:\大学资料\大二上\科研项目\local_vision\docs\ROAD_STRUCTURE_V1_RESULTS.md`. This is a completed first structural-classification baseline, not completed real-car safety: held-out accuracy 95.59%, known classes 99.22%, stable-node navigation 60/60; unknown false accepts 8/24 remain a material limitation. No fine-tuning or test-driven threshold/model changes were made.
