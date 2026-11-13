# Structural Navigation Implementation Plan

> Use executing-plans in this session. No subagents, new repository or commits: the remote project is not a Git repository.

**Goal:** Replace direct node-probability observations in the third block with uncertain relative road structure and history/feedback-based candidate localization, keeping the former implementation as a runnable baseline.

**Architecture:** Add a pure map/candidate/decision module and a separate truth-owning simulator/runner. The simulator reuses old Environment2D physics, not its node-probability recognizer. Preserve old navigation.py and tests byte-for-byte; update the navigation guide with a baseline copy and the new default instructions.

**Tech Stack:** Existing Python 3.10 venv, pytest, standard library and existing Matplotlib plotting.

## Task 1: Specify contracts in failing tests

Create `/home/sunrise/toponav_project/tests/test_structural_navigation.py`.
- Assert `observation` contains only four relative Boolean/unknown directions and confidence; no node labels, position, true heading or channel IDs.
- Assert N02/east and N04/west have identical structure but remain distinct candidates; decision must stop when ambiguous. Check temporal prediction rules out impossible same-type locations.
- Assert success advances candidates; failure leaves position/heading hypotheses unchanged; unknown/sent/missing feedback preserves stay+move possibilities. Contradictory success reports transition mismatch.
- Assert partial/low-confidence observations and impossible observations have explicit states, never truth resets or all-map jumps.
- Test full ideal loop, detour, reverse routes, all 64 closures x 36 start/goal pairs, four headings, actual motion/next-observation linkage, failure/unknown/sent simulations, and task prior independent of truth.
- Run `.venv/bin/python -m pytest tests/test_structural_navigation.py -q`; expect missing new module before implementation.

## Task 2: Implement pure candidate localization and decision

Create `/home/sunrise/toponav_project/simulation/structural_localization.py`.
- Implement heading/relative action conversion and enabled map exits from graph topology/actions only.
- `update_candidates(graph, previous_candidates, observation, previous_action=None, feedback=None, disabled_channels=())` predicts from execution feedback, filters by reliable known structural directions and returns predicted/matching candidates, estimate, status and reason.
- `decide(graph, localization, goal, disabled_channels=())` only moves for a unique, observation-confirmed candidate; reuse Dijkstra and convert first exit to a relative command. Stop for failures, uncertainty, mismatches and goal arrival.
- Validate observation and feedback schemas, reject truth-bearing/extra fields. No simulator, scene or true position parameters or imports.

## Task 3: Wire simulator and feedback loop

Create `/home/sunrise/toponav_project/simulation/structural_navigation.py`.
- StructuralEnvironment wraps old physics, observes physical outgoing relative directions, executes atomic relative movement with succeeded/failed/unknown/sent feedback. Physical result is logged separately, not passed to localization.
- `run_navigation` initializes prior from task parameters; every observation uses only previous hypotheses, last command and public feedback to update localization. Counts sent commands without using true motion to control the loop.
- Add deterministic scenarios ideal/execution_failed/execution_unknown/execution_sent/ambiguous/mismatch with a fault step for repeatable acceptance, not per-location identity injection.
- Keep controller status separate from terminal truth audit. Adapt truth trace to the old plot helper without passing truth back to the controller.
- New CLI `python -m simulation.structural_navigation` with map/scene/start/heading/goal/disable/scenario/fault-step/max-steps/output/plot. Exit 0 only for controller arrival verified by audit, 1 safe stop/audit failure, 2 invalid input.
- Run targeted and full tests. Fix only task-related failures; preserve original file hashes.

## Task 4: Remote acceptance and delivery

- Copy original `docs/NAVIGATION.md` to `docs/NAVIGATION_NODE_PROBABILITY_BASELINE.md` after checking hashes and saving a non-overwriting remote backup. Update `docs/NAVIGATION.md` to distinguish new third block and old baseline.
- Run ideal, detour, failed, unknown, sent, ambiguous, mismatch scenarios; save new `reports/structural_*` artifacts without overwriting historic reports. Save CLI return codes, suite log, baseline hashes, same-structure candidate evidence and truth isolation evidence.
- Independently audit true vs estimated state, next-observation linkage, no advance on failed/sent, explicit stop reasons, and structural-only payloads. Inspect actual-route PNG.
- Download new source/docs/tests/artifacts and verify local/remote SHA256. Do not install packages, change SSH/system configuration or restart services.
