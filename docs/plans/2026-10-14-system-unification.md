# Unified Topology Navigation Implementation Plan

**Goal:** Make map v2, localization, planning, simulation and documentation use one verified system contract.
**Architecture:** Shared topology/direction helpers; native v2 hop planning with v1 compatibility; structural navigation uses the public planning block and independent physical endpoint directions.
**Tech Stack:** Existing Python 3.10, pytest, JSON Schema; no dependency changes.

1. Back up sources/configs/docs/reports and preserve v1 fixtures. Add integration regression tests and record their failure.
2. Add shared topology validation/direction helpers; adapt Dijkstra and relative planning. Verify v1 planning and new v2 tests.
3. Normalize movement commands at boundaries; wire structural decision through path_planning; make physical sensing/execution independent of display geometry. Verify safety, truth isolation and all closure pairs.
4. Promote existing v2 map/schema to defaults; keep legacy baseline explicit. Update active docs and archive superseded descriptions without rewriting historical acceptance records.
5. Run complete tests and real CLI JSON/plot/error cases, verify historical hashes and fresh acceptance artifacts, then report results and remaining scope limits.

No speculative features, dependency installation, system changes or Git operations.

## Completion
All five steps completed on 2026-10-14. Full regression: 529 passed (403 retained + 126 integration), 18 existing dependency deprecation warnings. Fresh CLI/audit and file-preservation evidence: reports/system_unification/ACCEPTANCE.md and acceptance.json. No Git repository, commits or system changes.
