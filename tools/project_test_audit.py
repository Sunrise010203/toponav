"""Opt-in pytest audit plugin for run_project_tests; never filters tests.

Not auto-loaded by ordinary pytest. Records actual node IDs, source files,
collection failures and setup/call/teardown outcomes without rewriting them.
"""
import hashlib
import fnmatch
import json
import os
from pathlib import Path

import pytest


GROUPS = ("current", "historical", "other")
OUTCOMES = ("passed", "failed", "error", "skipped", "xfailed", "xpassed", "not_run")
DEFAULT_PATTERNS = dict(python_files=["test_*.py", "*_test.py"], python_classes=["Test"], python_functions=["test"],
                        norecursedirs=["*.egg", ".*", "_darcs", "build", "CVS", "dist", "node_modules", "venv", "{arch}"])


def source_group(path, root):
    try:
        parts = Path(path).resolve().relative_to(Path(root).resolve()).parts
    except ValueError:
        return "other"
    return {"tests": "current", "reports": "historical"}.get(parts[0] if parts else "", "other")


def case_outcome(phases):
    if any(p["outcome"] == "failed" and p["when"] != "call" for p in phases):
        return "error"
    if any(p["outcome"] == "failed" for p in phases):
        return "failed"
    if any(p["outcome"] == "skipped" and p["wasxfail"] is not None for p in phases):
        return "xfailed"
    if any(p["outcome"] == "skipped" for p in phases):
        return "skipped"
    calls = [p for p in phases if p["when"] == "call"]
    if any(p["outcome"] == "passed" and p["wasxfail"] is not None for p in calls):
        return "xpassed"
    if any(p["outcome"] == "passed" for p in calls):
        return "passed" if any(p["when"] == "teardown" for p in phases) else "not_run"
    return "not_run"


def write_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


class TestAudit:
    def __init__(self, config, output):
        self.root = config.rootpath.resolve()
        self.output = output
        self.collected = []
        self.deselected = []
        self.phases = {}
        self.collection_errors = []
        self.collection_skips = []
        self.internal_errors = []
        self.interrupted = False
        self.sources = {}
        self.inventory = None
        self.ignored_paths = set()
        self.discovery = {name: config.getini(name) for name in (*DEFAULT_PATTERNS, "testpaths")}
        self.discovery["config_file"] = str(config.inipath) if config.inipath else None
        self.restrictions = ["nondefault_" + name for name, default in DEFAULT_PATTERNS.items()
                             if self.discovery[name] != default]
        for name in ("ignore", "ignore_glob", "deselect", "keyword", "markexpr", "lf", "stepwise"):
            value = config.getoption(name, default=None)
            if value:
                self.restrictions.append(name)

    @pytest.hookimpl(wrapper=True)
    def pytest_ignore_collect(self, collection_path, config):
        ignored = yield
        if ignored:
            try:
                relative = collection_path.resolve().relative_to(self.root)
            except ValueError:
                return ignored
            normal_exclusion = any(part == "__pycache__" or any(fnmatch.fnmatch(part, pattern)
                                   for pattern in DEFAULT_PATTERNS["norecursedirs"]) for part in relative.parts)
            candidate = collection_path.is_dir() or any(fnmatch.fnmatch(collection_path.name, pattern)
                                                       for pattern in DEFAULT_PATTERNS["python_files"])
            if candidate and not normal_exclusion:
                self.ignored_paths.add(str(collection_path.resolve()))
        return ignored

    def item(self, item):
        path = Path(item.path).resolve()
        if path not in self.sources:
            self.sources[path] = hashlib.sha256(path.read_bytes()).hexdigest()
        return dict(nodeid=item.nodeid, source=str(path), group=source_group(path, self.root),
                    source_sha256=self.sources[path])

    def pytest_itemcollected(self, item):
        self.collected.append(self.item(item))

    def pytest_deselected(self, items):
        self.deselected.extend(self.item(item) for item in items)

    def pytest_collectreport(self, report):
        if report.failed:
            self.collection_errors.append(dict(nodeid=report.nodeid, message=str(report.longrepr)))
        elif report.skipped:
            # Module-level skip/importorskip can yield no test items at all.
            # They must not disappear behind unrelated passing tests.
            self.collection_skips.append(dict(nodeid=report.nodeid, message=str(report.longrepr)))

    def pytest_collection_finish(self, session):
        self.inventory = dict(version=1, project_root=str(self.root),
                              collected=self.collected, selected=[self.item(i) for i in session.items],
                              deselected=self.deselected, collection_errors=self.collection_errors,
                              collection_skips=self.collection_skips,
                              discovery=self.discovery, discovery_restrictions=self.restrictions,
                              ignored_test_paths=sorted(self.ignored_paths))
        write_json(self.output / "collection.json", self.inventory)

    def pytest_runtest_logreport(self, report):
        self.phases.setdefault(report.nodeid, []).append(dict(
            when=report.when, outcome=report.outcome, wasxfail=getattr(report, "wasxfail", None),
            duration_s=report.duration,
            message=str(report.longrepr) if report.failed or report.skipped else None))

    def pytest_internalerror(self, excrepr, excinfo):
        self.internal_errors.append(str(excrepr))

    def pytest_keyboard_interrupt(self, excinfo):
        self.interrupted = True

    @pytest.hookimpl(trylast=True)
    def pytest_sessionfinish(self, session, exitstatus):
        if self.inventory is None:
            self.pytest_collection_finish(session)
        selected = self.inventory["selected"]
        groups = {name: {key: 0 for key in ("selected", *OUTCOMES)} for name in GROUPS}
        results = []
        for item in selected:
            phases = self.phases.get(item["nodeid"], [])
            outcome = case_outcome(phases)
            groups[item["group"]]["selected"] += 1
            groups[item["group"]][outcome] += 1
            results.append(dict(**item, outcome=outcome, phases=phases))
        collected_ids = [item["nodeid"] for item in self.collected]
        selected_ids = [item["nodeid"] for item in selected]
        deselected_ids = [item["nodeid"] for item in self.deselected]
        accounting = (len(set(collected_ids)) == len(collected_ids)
                      and len(set(selected_ids)) == len(selected_ids)
                      and not (set(selected_ids) & set(deselected_ids))
                      and set(collected_ids) == set(selected_ids) | set(deselected_ids)
                      and set(self.phases) <= set(selected_ids))
        write_json(self.output / "pytest-audit.json", dict(
            version=1, project_root=str(self.root), pytest_exit_code=int(exitstatus),
            collected=len(self.collected), selected=len(selected), deselected=len(self.deselected),
            collection_accounting_complete=accounting, groups=groups, tests=results,
            discovery=self.discovery, discovery_restrictions=self.restrictions,
            ignored_test_paths=sorted(self.ignored_paths),
            collection_errors=self.collection_errors, internal_errors=self.internal_errors,
            collection_skips=self.collection_skips,
            interrupted=self.interrupted))


def pytest_configure(config):
    output = os.environ.get("TOPONAV_TEST_AUDIT_DIR")
    if not output or not Path(output).is_dir():
        raise pytest.UsageError("project_test_audit requires a new run_project_tests output directory")
    config.pluginmanager.register(TestAudit(config, Path(output)), "toponav-test-audit-recorder")
