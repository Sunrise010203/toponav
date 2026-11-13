# Minimal Navigation Loop Implementation Plan

> Execution: use executing-plans in this session, without subagents. The remote project is not a Git repository; do not create a repository or commit.

**Goal:** Connect simulated recognition probabilities, localization, planning, decisions and a stateful 2D environment; verify ideal-observation goal arrival and fresh observations after movement.

**Architecture:** Add one `simulation/navigation.py` module beside the existing planner. The environment executes absolute direction commands from its actual position; localization and decision operate only on probabilities and map data. Preserve map/configuration files and existing Dijkstra code.

**Tech Stack:** existing Python 3.10 venv, standard library, pytest, Matplotlib Agg.

---

### Task 1: Specify the closed-loop contract

Create `/home/sunrise/toponav_project/tests/test_navigation.py`.
1. Test `env.step("east")` moves N01 (0,0) to N02 (2,0), and a newly generated observation changes from N01 one-hot to N02 one-hot.
2. Test ideal `run_navigation(graph, scene)` finishes at N06 with the final N06 observation, four movement records and 8 metres of real travel.
3. Cover blocked moves, closure detour, unreachable stop, probability-only localization, confidence stop, limits, reverse directions and start=goal.
4. Compare all 64 closure subsets × 36 start/goal pairs with Dijkstra reachability, asserting actual arrival and per-step observation/position linkage.
5. Run `.venv/bin/python -m pytest tests/test_navigation.py -q`; expect initial import failure because the implementation is missing.

### Task 2: Implement minimal components

Create `/home/sunrise/toponav_project/simulation/navigation.py`.
1. `Environment2D`: scene positions/physical channels, map allowed channels, current node/heading/distance; apply direction from actual position, never a planner-provided destination.
2. `SimulatedRecognizer.observe(environment)`: fresh full-node probability dictionary based on the environment's current node, default confidence 1.
3. `localize(probabilities, threshold)`: validate a normalized distribution and return estimated node/confidence or uncertainty, without environment access.
4. `decide(graph, localization, goal, disabled_channels)`: reuse `plan`; return one absolute exit direction or a terminal state.
5. `run_navigation`: observe, localize, replan, decide, step, repeat; log probabilities, localization, route, command and before/after truth. Always observe after the last movement before reporting success.
6. CLI: `--map`, `--scene`, `--start`, `--goal`, `--disable`, `--confidence`, `--max-steps`, `--output`, `--plot`. Exit 0 for verified arrival, 1 for non-arrival, 2 for invalid input. Optional PNG uses actual executed positions, not the planned route.
7. Run new tests then `.venv/bin/python -m pytest -q`; all old tests must remain green.

### Task 3: Remote demonstrations and delivery

Create `/home/sunrise/toponav_project/docs/NAVIGATION.md` and new reports; do not overwrite existing historical acceptance reports.
1. Run ideal navigation with JSON and PNG output, repeat with C02 disabled and C06 disabled (last expected exit 1).
2. Inspect JSON records independently: each before-position matches the observed one-hot node; each action changes position along a legal enabled physical channel; the next observation corresponds to the previous after-position; the final observation matches N06.
3. Render and inspect PNG; record tests, commands, real route and simulator limitations in `reports/NAVIGATION_ACCEPTANCE.md`.
4. Download newly created source/tests/docs/reports into the local delivery directory and compare SHA256 with remote files. No system package installation, SSH configuration changes or service management.
