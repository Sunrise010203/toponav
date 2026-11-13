# Topological Navigation Environment and Map Implementation Plan

**Goal:** Configure a dedicated VM project environment and six-node map without adding application source, then verify loading, neighbors and illegal connections.

**Architecture:** Reuse ROS 2 Humble and isolate pip dependencies in a project-local virtualenv. Keep the designed physical scene separate from the topology JSON. Use installed CLI programs and declarative JSON Schema; retain transient acceptance commands in the README, not source files.

**Tech Stack:** Ubuntu 22.04, Python 3.10, ROS 2 Humble, NumPy 1.26, NetworkX, OpenCV, CPU PyTorch/torchvision, ONNX Runtime, jq, JSON Schema, Graphviz.

## 1. Project isolation
Create `/home/sunrise/toponav_project` and the guidance-book directories. Inspect existing files before writing. Bootstrap virtualenv from official PyPA over HTTPS because passwordless sudo is unavailable. Verify `.venv/bin/python -m pip --version`; retain bootstrap hash.

## 2. Dependency installation
Save requirements files. Install scientific/vision/validation dependencies, then CPU-only PyTorch from its official dedicated index. Verify pip check, imports, ROS interop, array/image handling, plotting, MobileNetV3 forward, ONNX inference and FFmpeg runtime. Lock installed local pip distributions; document inherited ROS/system tools separately.

## 3. Scene and map
Save `configs/scene.json`, `configs/graph.json`, `configs/graph.schema.json` and explicit invalid-case fixtures. Render `reports/topology.svg` and `reports/topology.png` using Graphviz. Verify 6 nodes, 6 bidirectional channels, N01 start, N06 goal and both 8m routes.

## 4. Acceptance
Run JSON Schema and jq checks on the valid map. Query N02 and N04. Run the same checks against each invalid fixture, require rejection and the named expected failure checks. Include a temporary disabled-channel example that demonstrates an alternate enabled route and distinguish operational blockage from a malformed connection. Persist logs and the acceptance summary.

## 5. Handoff
Save a README with exact activation, query, validation, dependency reinstallation and environment boundaries. Copy the small deliverables (not `.venv`) to the Windows workspace for review. Audit remote artifacts and results before marking complete. Do not make Git commits, install system packages, alter SSH configuration or add navigation source.
