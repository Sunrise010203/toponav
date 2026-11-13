"""Fictional timing arithmetic and read-only source order, no native runtime."""
import copy
from dataclasses import FrozenInstanceError, asdict
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from tools import precompute_deadline_cases as cases
from tools.precompute_deadline_cases import SOURCE_FILES, CASES, INVALID, audit_sources, exercise, schedule, sha
from tools.precompute_deadline_contract import ContractError, LEASE_NS, compare_schedule

ROOT=Path(__file__).resolve().parents[1]
AUTH=('provenance_authenticated','shadow_admission_authorized','native_live_observation_authorized','motion_authority')


def windows(result):return {w.name:w for w in result.windows}


def safe(result):
    assert result.actual_exposure_wall_s is None and not any(getattr(result,k) for k in AUTH)
    assert not result.exposure_age_bounded and not result.independent_enforcement_observed
    assert not result.physical_stop_proved and not result.production_guard_fixed


@pytest.mark.parametrize('name',CASES+INVALID)
def test_declared_synthetic_cases(name,tmp_path):
    row=exercise(name,tmp_path/name)
    assert row['valid']==(name in CASES)
    if row['valid']:
        assert row['result']['actual_exposure_wall_s'] is None and not any(row['result'][k] for k in AUTH)


def test_first_slow_result_reproduces_post_compute_gap_without_running_controller():
    doc=schedule('first_slow_compute');r=compare_schedule(doc);w=windows(r)
    assert w['gazebo_postcompute'].new_pending_deadline_before_work_ns is None
    assert w['gazebo_postcompute'].effective_deadline_during_work_ns is None
    assert w['gazebo_postcompute'].reported_result_within_window is True
    assert w['ros_existing_pending'].reported_result_within_window is False
    assert w['candidate_fixed_availability'].reported_result_within_window is False
    assert r.work_elapsed_since_available_ns>LEASE_NS;safe(r)


@pytest.mark.parametrize('name,expected',[('one_ns_before',True),('exact_boundary',False),('one_ns_after',False)])
def test_exact_integer_boundary_has_no_clock_grace(name,expected):
    w=windows(compare_schedule(schedule(name)))
    assert w['candidate_fixed_availability'].reported_result_within_window is expected
    assert w['ros_existing_pending'].reported_result_within_window is expected


def test_original_available_deadline_is_not_rebased_at_completion():
    doc=schedule('completion_rebase_counterexample');w=windows(compare_schedule(doc));d=doc['frame_available_ns']+LEASE_NS
    assert w['candidate_fixed_availability'].hypothetical_hold_after_timely_result_ns==d
    assert w['ros_existing_pending'].hypothetical_hold_after_timely_result_ns==doc['compute_returned_ns']+LEASE_NS>d
    assert w['gazebo_postcompute'].hypothetical_hold_after_timely_result_ns>d


@pytest.mark.parametrize('name',['held_expires_first','held_exact_return','held_already_expired'])
def test_existing_held_deadline_is_not_extended_by_new_pending(name):
    doc=schedule(name);r=compare_schedule(doc)
    for w in r.windows:
        assert w.effective_deadline_during_work_ns==doc['held_deadline_ns']
        assert w.reported_result_within_window is False and w.hypothetical_hold_after_timely_result_ns is None


def test_timely_result_may_replace_old_hold_with_new_bound_budget():
    doc=schedule('held_timely_replacement');w=windows(compare_schedule(doc));c=w['candidate_fixed_availability']
    assert c.effective_deadline_during_work_ns==doc['held_deadline_ns']
    assert c.reported_result_within_window is True and c.hypothetical_hold_after_timely_result_ns>doc['held_deadline_ns']


def test_slow_serialization_is_consumed_but_capture_wait_is_not_covered():
    w=windows(compare_schedule(schedule('slow_serialization')))
    assert w['candidate_fixed_availability'].reported_result_within_window is False
    r=compare_schedule(schedule('slow_capture'));assert r.capture_wait_ns>LEASE_NS and not r.new_frame_budget_covers_capture_wait
    assert windows(r)['candidate_fixed_availability'].reported_result_within_window is True
    assert not r.exposure_age_bounded


@pytest.mark.parametrize('name',['never_returns','owner_stopped'])
def test_expired_arithmetic_without_owner_progress_is_not_enforcement(name):
    r=compare_schedule(schedule(name));c=windows(r)['candidate_fixed_availability']
    assert c.work_deadline_due_at_review is True and c.reported_result_within_window is None
    assert c.hypothetical_hold_after_timely_result_ns is None;safe(r)


@pytest.mark.parametrize('name',['wrong_frame','wrong_sim_identity','wrong_view_digest'])
def test_complete_binding_required_even_for_timely_result(name):
    r=compare_schedule(schedule(name));assert r.reported_binding_matches is False
    assert all(w.reported_result_within_window is False for w in r.windows)


def test_sim_stamp_only_changes_identity_not_host_budget():
    doc=schedule('stationary_sim_slow_host');r=compare_schedule(doc)
    for key in ('input_binding','result_binding'):doc[key]['captured_sim_ns']=0
    changed=compare_schedule(doc);assert changed.windows==r.windows and changed.work_elapsed_since_available_ns==r.work_elapsed_since_available_ns


def test_repeat_is_not_anti_replay_or_sticky_owner_and_inputs_unchanged():
    doc=schedule('quick');before=copy.deepcopy(doc);r=compare_schedule(doc)
    assert r==compare_schedule(doc) and doc==before;safe(r)
    with pytest.raises(FrozenInstanceError):r.motion_authority=True


@pytest.fixture
def source_copy(tmp_path):
    root=tmp_path/'sources';root.mkdir()
    for n in SOURCE_FILES:
        p=root/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/n,p)
    return root,{n:sha(root/n) for n in SOURCE_FILES}


def test_real_source_lexical_order_is_explicit_and_unchanged():
    pins={n:sha(ROOT/n) for n in SOURCE_FILES};result=audit_sources(ROOT,pins)
    assert len(result['claims'])==6 and result['original_lease_s']==.8 and result['original_ack_s']==.4
    assert result['file_sha256']==pins and result['source_only']
    for row in result['claims']:
        for a in row['anchors']:assert a['line']>0 and a['line']<=len((ROOT/row['path']).read_text().splitlines())


@pytest.mark.parametrize('kind',['missing','extra','malformed'])
def test_all_source_pins_required_before_reads(source_copy,monkeypatch,kind):
    root,pins=source_copy
    if kind=='missing':pins.pop(SOURCE_FILES[-1])
    elif kind=='extra':pins['../do-not-read']='0'*64
    else:pins[SOURCE_FILES[0]]='X'*64
    def forbidden(*a,**k):raise AssertionError('pin schema must precede reads')
    monkeypatch.setattr(cases,'read_index',forbidden)
    with pytest.raises(ContractError,match='source_six_pins_required'):audit_sources(root,pins)


def test_changed_source_pin_refused(source_copy):
    root,pins=source_copy;p=root/SOURCE_FILES[0];p.write_bytes(p.read_bytes()+b'\n')
    with pytest.raises(ContractError,match='source_pin_mismatch'):audit_sources(root,pins)


@pytest.mark.parametrize('kind',['comment','duplicate','budget','ros_arm'])
def test_ast_audit_not_satisfied_by_comments_duplicate_or_wrong_rules(source_copy,kind):
    root,pins=source_copy
    if kind in ('comment','duplicate'):
        p=root/SOURCE_FILES[0];text=p.read_text();text=text.replace('d = controller.update(frame, now)',
            '# controller.update(frame, now)\n                d = {}' if kind=='comment' else 'd = controller.update(frame, now)\n                controller.update(frame, now)')
    elif kind=='budget':p=root/SOURCE_FILES[3];text=p.read_text().replace('LEASE_S = .8','LEASE_S = .9')
    else:p=root/SOURCE_FILES[1];text=p.read_text().replace("if not terminal:self.gate.arm(f['frame_id'],self.sim_s,time.monotonic())",'# self.gate.arm removed from this synthetic copy')
    p.write_text(text);pins[str(p.relative_to(root))]=sha(p)
    with pytest.raises(ContractError):audit_sources(root,pins)


def test_source_changes_after_ast_analysis_refused(source_copy,monkeypatch):
    root,pins=source_copy;original=cases.read_index;calls=[]
    def changed(path):
        raw=original(path);calls.append(1);return raw if len(calls)<=len(SOURCE_FILES) else raw+b' '
    monkeypatch.setattr(cases,'read_index',changed)
    with pytest.raises(ContractError,match='source_changed_during_audit'):audit_sources(root,pins)


def test_symlink_source_refused(source_copy,tmp_path):
    from tools.offline_scope_review import BundleError
    root,pins=source_copy;p=root/SOURCE_FILES[0];p.unlink();p.symlink_to(ROOT/SOURCE_FILES[0])
    with pytest.raises(BundleError):audit_sources(root,pins)


def test_no_clock_threads_or_runtime_imports(monkeypatch):
    import time
    import threading
    def forbidden(*a,**k):raise AssertionError('runtime clock/thread')
    monkeypatch.setattr(time,'time',forbidden);monkeypatch.setattr(time,'monotonic',forbidden);monkeypatch.setattr(threading,'Thread',forbidden)
    assert compare_schedule(schedule('quick')).classification=='synthetic_arithmetic_only'
    code="import sys; import tools.precompute_deadline_cases; assert not any(n.startswith(('simulation','roadvision','rclpy','PIL','numpy','tools.frame_deadline_proposal','tools.straight_deadline_audit')) for n in sys.modules)"
    p=subprocess.run([sys.executable,'-c',code],cwd=ROOT,capture_output=True,check=False);assert p.returncode==0,p.stderr


def test_model_has_no_io_or_transport_imports():
    import ast
    tree=ast.parse((ROOT/'tools/precompute_deadline_contract.py').read_text())
    imports=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
    assert imports==['dataclasses'] and not any(isinstance(n,ast.Import) for n in ast.walk(tree))
    for folder in ('simulation','roadvision','ros2_ws/src/toponav_bridge/toponav_bridge'):
        for p in (ROOT/folder).rglob('*.py'):assert 'precompute_deadline_contract' not in p.read_text()
