"""Miniature pytest projects exercise scope accounting, never old evidence."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

from tools import project_test_audit as audit
from tools import run_project_tests as runner


@pytest.fixture
def project(tmp_path, monkeypatch):
    for name in ("PYTEST_ADDOPTS", "PYTEST_PLUGINS"):
        monkeypatch.delenv(name, raising=False)
    root = tmp_path / "project"
    (root / "tests").mkdir(parents=True)
    (root / "reports/archive").mkdir(parents=True)
    (root / "tests/test_live.py").write_text("def test_current(): assert True\n")
    (root / "reports/archive/test_frozen.py").write_text("def test_old_snapshot(): assert False, 'live source evolved'\n")
    (root / "test_external.py").write_text("def test_other(): assert True\n")
    return root


@pytest.mark.parametrize("scope,selected,passed,failed,code", [
    ("current", 1, 1, 0, 0), ("historical", 1, 0, 1, 1), ("all", 3, 2, 1, 1),
])
def test_explicit_scopes_keep_failures_and_default_all_is_unfiltered(project, scope, selected, passed, failed, code):
    old = project / "reports/archive/test_frozen.py"
    original = old.read_bytes()
    kwargs = {} if scope == "all" else {"scope": scope}
    summary = runner.run_suite(output=project / "reports/result", project_root=project, **kwargs)
    assert summary["scope"] == scope and summary["exit_code"] == code
    assert summary["selected"] == selected and summary["deselected"] == 0
    assert sum(g["passed"] for g in summary["groups"].values()) == passed
    assert sum(g["failed"] for g in summary["groups"].values()) == failed
    assert summary["verification_complete"] is (code == 0)
    assert summary["all_scopes_passed"] is False
    assert old.read_bytes() == original
    path = Path(summary["output_directory"])
    inventory = json.loads((path / "collection.json").read_text())
    assert len(inventory["selected"]) == selected and inventory["deselected"] == []
    for item in inventory["selected"]:
        assert item["source_sha256"] == hashlib.sha256(Path(item["source"]).read_bytes()).hexdigest()
    assert len(list(ET.parse(path / "junit.xml").iter("testcase"))) == selected
    if scope == "all":
        assert summary["groups"]["other"]["passed"] == 1
        assert summary["groups"]["historical"]["failed"] == 1
        assert summary["failures"][0]["nodeid"] == "reports/archive/test_frozen.py::test_old_snapshot"
    else:
        assert "other" in summary["unselected_groups"]
    command = json.loads((path / "command.json").read_text())
    assert command["discovery_configuration_changed"] is False
    for name, expected in summary["artifacts_sha256"].items():
        assert hashlib.sha256((path / name).read_bytes()).hexdigest() == expected


def test_all_scopes_passed_requires_actual_all_scope_execution(project):
    (project / "reports/archive/test_frozen.py").write_text("def test_old_snapshot(): assert True\n")
    summary = runner.run_suite(output=project / "reports/all-good", project_root=project)
    assert summary["exit_code"] == 0 and summary["all_scopes_passed"] is True
    assert summary["formal_navigation_acceptance"] is False


@pytest.mark.parametrize("scope", ["current", "historical", "all"])
def test_local_ros_bridge_import_is_independent_of_other_test_collection(project, scope):
    package_root = project / "ros2_ws/src/toponav_bridge"
    package = package_root / "toponav_bridge"
    package.mkdir(parents=True)
    (package / "__init__.py").write_text("PACKAGE_SOURCE = 'local-bridge-fixture'\n")
    body = ("from toponav_bridge import PACKAGE_SOURCE\n"
            "def test_import(): assert PACKAGE_SOURCE == 'local-bridge-fixture'\n")
    for relative in ("tests/test_live.py", "reports/archive/test_frozen.py", "test_external.py"):
        (project / relative).write_text(body)
    summary = runner.run_suite(scope, project / "reports/bridge-import", project_root=project)
    assert summary["exit_code"] == 0 and summary["verification_complete"] is True
    command = json.loads((Path(summary["output_directory"]) / "command.json").read_text())
    assert str(package_root) in command["pythonpath"]
    assert command["local_package_roots"] == [str(package_root)]


@pytest.mark.parametrize("body,outcome", [
    ("import pytest\n@pytest.mark.skip(reason='fixture skip')\ndef test_case(): pass\n", "skipped"),
    ("import pytest\n@pytest.mark.xfail(reason='not acceptance')\ndef test_case(): assert False\n", "xfailed"),
    ("import pytest\n@pytest.mark.xfail(reason='unexpected pass')\ndef test_case(): assert True\n", "xpassed"),
])
def test_skips_xfails_and_non_strict_xpasses_do_not_make_a_complete_green_scope(project, body, outcome):
    (project / "tests/test_live.py").write_text(body)
    summary = runner.run_suite("current", project / "reports/gaps", project_root=project)
    assert summary["pytest_exit_code"] == 0 and summary["exit_code"] == 1
    assert summary["groups"]["current"][outcome] == 1
    assert summary["status"] == "not_passed" and summary["verification_complete"] is False
    assert summary["failures"][0]["outcome"] == outcome


@pytest.mark.parametrize("stage", ["setup", "teardown"])
def test_fixture_phase_errors_are_not_counted_as_passing_tests(project, stage):
    fixture = "raise RuntimeError('setup failure')" if stage == "setup" else "yield\n    raise RuntimeError('teardown failure')"
    (project / "tests/test_live.py").write_text(
        "import pytest\n@pytest.fixture\ndef broken():\n    " + fixture + "\ndef test_case(broken): pass\n")
    summary = runner.run_suite("current", project / "reports/phase-error", project_root=project)
    assert summary["exit_code"] == 1
    assert summary["groups"]["current"]["error"] == 1 and summary["groups"]["current"]["passed"] == 0


def test_collection_error_and_partial_inventory_cannot_be_green(project):
    (project / "tests/test_broken.py").write_text("def bad syntax\n")
    summary = runner.run_suite("current", project / "reports/collect-error", project_root=project)
    assert summary["exit_code"] == summary["pytest_exit_code"] == 2
    assert summary["collection_errors"] and not summary["verification_complete"]


@pytest.mark.parametrize("statement", [
    "pytest.skip('module unavailable', allow_module_level=True)",
    "pytest.importorskip('toponav_intentionally_missing_fixture_dependency')",
])
def test_module_collection_skip_cannot_disappear_behind_other_passing_tests(project, statement):
    # Some installed ROS collection plugins import modules while collecting
    # their directory. Keep a sibling passing module outside that collector.
    unavailable = project / "tests/unavailable"
    unavailable.mkdir()
    (unavailable / "test_dependency.py").write_text("import pytest\n" + statement + "\n")
    summary = runner.run_suite("current", project / "reports/module-skipped", project_root=project)
    assert summary["pytest_exit_code"] == 0 and summary["exit_code"] == 1
    assert summary["groups"]["current"]["passed"] == 1
    assert not summary["verification_complete"]
    assert len(summary["collection_skips"]) == 1
    assert summary["collection_skips"][0]["nodeid"] in (
        "tests/unavailable", "tests/unavailable/test_dependency.py")


def test_no_tests_keeps_pytest_no_tests_exit_code(project):
    (project / "tests/test_live.py").unlink()
    summary = runner.run_suite("current", project / "reports/empty", project_root=project)
    assert summary["exit_code"] == summary["pytest_exit_code"] == 5
    assert summary["selected"] == 0 and not summary["verification_complete"]


def test_parametrized_nodeids_are_classified_by_source_not_classname_text(project):
    (project / "tests/test_live.py").write_text(
        "import pytest\n@pytest.mark.parametrize('value',['a.b','x::y'])\ndef test_case(value): assert value\n")
    summary = runner.run_suite("current", project / "reports/parameters", project_root=project)
    assert summary["groups"]["current"]["passed"] == 2
    collected = json.loads((Path(summary["output_directory"]) / "collection.json").read_text())
    assert [item["nodeid"] for item in collected["selected"]] == [
        "tests/test_live.py::test_case[a.b]", "tests/test_live.py::test_case[x::y]"]


def test_configured_deselection_is_visible_and_incomplete_not_silently_passed(project):
    (project / "pytest.ini").write_text("[pytest]\naddopts = -k current\n")
    summary = runner.run_suite(output=project / "reports/deselected", project_root=project)
    assert summary["pytest_exit_code"] == 0 and summary["exit_code"] == 1
    assert summary["deselected"] == 2 and not summary["verification_complete"]
    inventory = json.loads((Path(summary["output_directory"]) / "collection.json").read_text())
    assert len(inventory["collected"]) == 3 and len(inventory["deselected"]) == 2


@pytest.mark.parametrize("config", [
    "addopts = --ignore=reports", "norecursedirs = reports", "python_files = test_live.py test_external.py",
    "python_functions = test_current test_other", "addopts = --lf",
])
def test_config_cannot_hide_historical_tests_and_claim_complete_all_scope(project, config):
    (project / "pytest.ini").write_text("[pytest]\n" + config + "\n")
    summary = runner.run_suite(output=project / "reports/config-filter", project_root=project)
    assert summary["exit_code"] != 0 and not summary["verification_complete"]
    assert summary["discovery_restrictions"]


def test_conftest_ignore_is_recorded_without_changing_its_behavior(project):
    (project / "conftest.py").write_text("collect_ignore = ['reports']\n")
    summary = runner.run_suite(output=project / "reports/conftest-ignore", project_root=project)
    assert summary["pytest_exit_code"] == 0 and summary["exit_code"] == 1
    assert summary["ignored_test_paths"] == [str(project / "reports")]
    assert not summary["verification_complete"]


def test_byte_identical_ignored_copy_is_still_disclosed_not_whitelisted(project):
    archive = project / "reports/archive"
    body = "def test_old_snapshot(): assert True\n"
    (archive / "test_frozen.py").write_text(body)
    (archive / "test_frozen_copy.py").write_text(body)
    (archive / "conftest.py").write_text("collect_ignore = ['test_frozen.py']\n")
    summary = runner.run_suite("historical", project / "reports/duplicate", project_root=project)
    assert summary["pytest_exit_code"] == 0 and summary["exit_code"] == 1
    assert summary["groups"]["historical"]["passed"] == 1
    assert summary["ignored_test_paths"] == [str(archive / "test_frozen.py")]
    assert not summary["verification_complete"]


def test_all_explicit_root_ignores_configured_default_testpaths(project):
    (project / "pytest.ini").write_text("[pytest]\ntestpaths = tests\n")
    summary = runner.run_suite(output=project / "reports/explicit-root", project_root=project)
    assert summary["groups"]["historical"]["failed"] == 1
    assert summary["groups"]["other"]["passed"] == 1


@pytest.mark.parametrize("name", ["PYTEST_ADDOPTS", "PYTEST_PLUGINS"])
def test_hidden_environment_selection_is_rejected_before_creating_artifacts(project, monkeypatch, name):
    monkeypatch.setenv(name, "-k nothing" if name == "PYTEST_ADDOPTS" else "some_plugin")
    output = project / "reports/unused"
    with pytest.raises(ValueError, match=name):
        runner.run_suite(output=output, project_root=project)
    assert not output.exists()


@pytest.mark.parametrize("relative", ["reports/archive/new", "tests/output", "roadvision/output"])
def test_runner_cannot_write_inside_old_reports_or_source_trees(project, relative):
    with pytest.raises(ValueError, match="new direct child"):
        runner.run_suite(output=project / relative, project_root=project)
    assert not (project / relative).exists()


def test_existing_output_is_never_overwritten(project):
    output = project / "reports/existing"
    output.mkdir()
    (output / "summary.json").write_text("historic bytes\n")
    with pytest.raises(FileExistsError):
        runner.run_suite(output=output, project_root=project)
    assert (output / "summary.json").read_text() == "historic bytes\n"


def test_missing_scope_directory_fails_before_launch(project):
    for path in (project / "tests").iterdir():
        path.unlink()
    (project / "tests").rmdir()
    with pytest.raises(ValueError, match="Missing scope"):
        runner.run_suite("current", project / "reports/unused", project_root=project)


@pytest.mark.parametrize("returncode", [0, 1, -9])
def test_missing_audit_cannot_be_reported_as_success(project, monkeypatch, returncode):
    monkeypatch.setattr(runner.subprocess, "run", lambda *a, **kw: subprocess.CompletedProcess([], returncode))
    summary = runner.run_suite(output=project / "reports/missing-audit", project_root=project)
    assert summary["exit_code"] != 0 and summary["pytest_exit_code"] == returncode
    assert summary["status"] == "not_passed" and summary["audit_error"]


def test_launch_error_is_recorded_as_failure(project, monkeypatch):
    def broken(*args, **kwargs):
        raise OSError("synthetic launch error")
    monkeypatch.setattr(runner.subprocess, "run", broken)
    summary = runner.run_suite(output=project / "reports/launch-error", project_root=project)
    assert summary["exit_code"] == 1 and "launch error" in summary["runner_error"]


def test_cli_rejects_extra_pytest_filter_arguments_before_running(tmp_path):
    result = subprocess.run([sys.executable, str(runner.ROOT / "tools/run_project_tests.py"),
                             "--output", str(tmp_path / "unused"), "-k", "only_one"],
                            capture_output=True, text=True)
    assert result.returncode == 2 and not (tmp_path / "unused").exists()


@pytest.mark.parametrize("relative,group", [
    ("tests/test_live.py", "current"), ("reports/archive/test_frozen.py", "historical"),
    ("test_external.py", "other"), ("experiments/test_new.py", "other"),
])
def test_grouping_uses_real_source_path(project, relative, group):
    assert audit.source_group(project / relative, project) == group


def test_outside_source_and_symlink_are_not_misclassified_as_current(project, tmp_path):
    external = tmp_path / "external.py"
    external.write_text("pass\n")
    link = project / "tests/test_link.py"
    link.symlink_to(external)
    assert audit.source_group(external, project) == "other"
    assert audit.source_group(link, project) == "other"


def test_call_pass_without_teardown_is_incomplete():
    assert audit.case_outcome([dict(when="call", outcome="passed", wasxfail=None)]) == "not_run"
