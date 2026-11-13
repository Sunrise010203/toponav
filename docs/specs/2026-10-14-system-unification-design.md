# Unified topology navigation v2 design

Approved by the user on 2026-10-14 (Asia/Shanghai).

## Scope
Unify the existing map, candidate localization, route planner and discrete executor; no ROS, camera, continuous robot control, new dependencies or system/SSH changes. Preserve legacy fixtures and historical evidence. No Git repository is present; no commit/push.

## Contract
The default map/scene use v2 task/exits/optional geometry. Explicit legacy inputs remain supported. The node-probability baseline keeps v1 fixtures under configs/legacy. V2 plans minimize hop count and report cost_unit=hops, hop_count and length_m=null; physical metres belong to scene-based evaluation, never inferred from display coordinates. V1 retains its weighted length contract.

The active flow is relative structure and execution status -> candidate localization -> path_planning.plan_from_estimate -> path_planning.next_action -> discrete simulator. Movement commands are straight/left/right/uturn/stop. Observations retain front/left/right/back (sensor directions, not commands); old front/back/u_turn commands are accepted only at the input compatibility boundary and normalized in new logs.

V2 exits and physical scene endpoint directions determine departure/arrival headings, including curved channels. Display geometry never determines sensing, localization or control. With no geometry, discrete navigation still runs; coordinates and physical distance are null. Plotting without required geometry reports an explicit error rather than fabricated geometry. Missing task values require explicit start/goal/heading; unknown orientation never defaults east.

## Safety and validation
Validate schemas, references, reciprocal neighbors, exit bindings, independent scene channels and geometry metadata before executing. Disabled channels do not make topology illegal. Uncertain/multiple/mismatched localization and unconfirmed/failed execution stop without truth repair. Simulator truth is only for sensing, physics and audit.

## Acceptance
Preserve all historical reports and original files in a timestamped backup. Keep previous tests, updating only explicit legacy fixture paths and intentional canonical-command expectations. Add v2 end-to-end, geometry independence, missing geometry/task, arbitrary ID/size, closure, curved-channel, safety, real planning-call, CLI and invalid-map regressions. Run full pytest, real CLI cases, fresh result audits and file hashes. Promote default configs only after v2 tests pass, then retest defaults.
