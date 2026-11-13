"""Explicit current/historical/all test scopes, with all as unfiltered default.

Never changes pytest discovery configuration or expected failures. Every run
gets new logs, JUnit, collection inventory and a source-separated audit.
"""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid


ROOT = Path(__file__).resolve().parents[1]
SCOPES = {"all": (".",), "current": ("tests",), "historical": ("reports",)}


def write_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def summarize(audit, scope, returncode):
    groups = audit["groups"]
    selected = sum(group["selected"] for group in groups.values())
    passed = sum(group["passed"] for group in groups.values())
    complete = bool(selected and selected == passed and selected == audit["selected"]
                    and audit["deselected"] == 0 and audit["collection_accounting_complete"]
                    and not audit["collection_errors"] and not audit["internal_errors"]
                    and not audit["collection_skips"]
                    and not audit["discovery_restrictions"] and not audit["ignored_test_paths"]
                    and not audit["interrupted"] and returncode == audit["pytest_exit_code"] == 0)
    allowed = {"all": {"current", "historical", "other"},
               "current": {"current"}, "historical": {"historical"}}[scope]
    complete = complete and not any(group["selected"] for name, group in groups.items() if name not in allowed)
    return dict(
        version=1, scope=scope, project_root=audit["project_root"],
        status="passed" if complete else "not_passed", verification_complete=complete,
        pytest_exit_code=returncode, exit_code=returncode if 0 < returncode < 256 else 0 if complete else 1,
        groups=groups, selected=selected, deselected=audit["deselected"],
        failures=[dict(nodeid=t["nodeid"], group=t["group"], outcome=t["outcome"])
                  for t in audit["tests"] if t["outcome"] != "passed"],
        collection_errors=audit["collection_errors"], internal_errors=audit["internal_errors"],
        collection_skips=audit["collection_skips"],
        collection_accounting_complete=audit["collection_accounting_complete"], interrupted=audit["interrupted"],
        discovery=audit["discovery"], discovery_restrictions=audit["discovery_restrictions"],
        ignored_test_paths=audit["ignored_test_paths"],
        all_scopes_passed=scope == "all" and complete,
        unselected_groups=sorted(set(groups) - allowed),
        formal_navigation_acceptance=False,
        scope_notice="Only this explicit scope is evaluated; no historical failure is whitelisted or reclassified.")


def run_suite(scope="all", output=None, *, project_root=ROOT):
    if scope not in SCOPES:
        raise ValueError("Unknown test scope")
    root = Path(project_root).resolve()
    for name in ("PYTEST_ADDOPTS", "PYTEST_PLUGINS"):
        if os.environ.get(name, "").strip():
            raise ValueError(f"Unset {name} before an auditable unfiltered run")
    for target in SCOPES[scope]:
        if not (root / target).is_dir():
            raise ValueError(f"Missing scope directory: {root / target}")
    if output is None:
        output = root / "reports" / ("test-scope-" + datetime.now().strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:8])
    output = Path(output).resolve()
    # Within this project use a NEW direct report child, not an old frozen
    # report's subdirectory or a source/test directory. External paths may be
    # used for CI artifacts, but must also be new.
    if output.is_relative_to(root) and output.parent != root / "reports":
        raise ValueError("Project output must be a new direct child of reports/")
    output.mkdir(parents=True, exist_ok=False)
    command = [sys.executable, "-m", "pytest", "-q", "--rootdir", str(root),
               "-p", "tools.project_test_audit", "--junitxml", str(output / "junit.xml"), *SCOPES[scope]]
    env = os.environ.copy()
    env["TOPONAV_TEST_AUDIT_DIR"] = str(output)
    # Historical ROS tests must not depend on tests/test_ros2_bridge.py being
    # collected first and mutating sys.path. Use the same local package in
    # every scope, without requiring a ROS workspace installation.
    bridge_root = root / "ros2_ws/src/toponav_bridge"
    local_package_roots = [str(bridge_root)] if bridge_root.is_dir() else []
    pythonpath = [str(root), *local_package_roots]
    # Also load the recorder in the runner's isolated miniature projects.
    if ROOT != root:
        pythonpath.append(str(ROOT))
    if env.get("PYTHONPATH"):
        pythonpath.extend(env["PYTHONPATH"].split(os.pathsep))
    env["PYTHONPATH"] = os.pathsep.join(pythonpath)
    write_json(output / "command.json", dict(command=command, cwd=str(root), scope=scope,
        pytest_addopts=None, pytest_plugins=None, pythonpath=pythonpath,
        local_package_roots=local_package_roots, discovery_configuration_changed=False))
    runner_error = None
    try:
        with (output / "pytest.log").open("xb") as log:
            process = subprocess.run(command, cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT, check=False)
        returncode = process.returncode
    except OSError as error:
        returncode, runner_error = 1, f"{type(error).__name__}: {error}"
    except KeyboardInterrupt:
        returncode, runner_error = 130, "Runner interrupted; no completion claim"
    try:
        audit = json.loads((output / "pytest-audit.json").read_text(encoding="utf-8"))
        summary = summarize(audit, scope, returncode)
    except (OSError, ValueError, KeyError, TypeError) as error:
        summary = dict(version=1, scope=scope, status="not_passed", verification_complete=False,
                       pytest_exit_code=returncode, exit_code=returncode if 0 < returncode < 256 else 1,
                       all_scopes_passed=False, formal_navigation_acceptance=False,
                       audit_error=f"{type(error).__name__}: {error}")
    if runner_error is not None:
        summary.update(runner_error=runner_error, status="not_passed", verification_complete=False,
                       all_scopes_passed=False, exit_code=returncode)
    summary["output_directory"] = str(output)
    summary["artifacts_sha256"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in sorted(output.iterdir()) if p.is_file()}
    write_json(output / "summary.json", summary)
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scope", choices=SCOPES, default="all",
                        help="all (default): ordinary unfiltered discovery; current: tests/; historical: reports/")
    parser.add_argument("--output", type=Path, help="New artifact directory; never reuse a historical report")
    args = parser.parse_args(argv)
    try:
        summary = run_suite(args.scope, args.output)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return summary["exit_code"]


if __name__ == "__main__":
    raise SystemExit(main())
