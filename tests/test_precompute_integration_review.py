"""Static design coverage only; future runtime scenarios remain NOT executed."""
import ast
import copy
import hashlib
import json
from pathlib import Path

import pytest

from tools import precompute_integration_review as review
from tools.precompute_deadline_contract import ContractError

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture
def plan():return json.loads((ROOT/review.DESIGN).read_text())


def test_canonical_design_is_only_structurally_consistent(plan):
    out=review.validate_plan(plan)
    assert out['classification']=='offline_design_structure_only'
    assert out['requirements']==14 and out['transitions']==12 and out['future_test_scenarios']==25
    assert out['new_runtime_experiments']==0 and out['actual_exposure_wall_s'] is None
    assert all(out[k] is False for k in review.FALSE_FIELDS)
    assert set(out['blockers'])==set(review.BLOCKERS)


@pytest.mark.parametrize('case',review.INVALID)
def test_mutated_design_refused_not_implicitly_approved(plan,case):
    with pytest.raises(ContractError):review.validate_plan(review.invalid_plan(plan,case))


@pytest.mark.parametrize('identity',review.ANCHORS)
def test_pinned_source_anchor_is_unique_in_declared_scope(identity):
    raw={n:(ROOT/n).read_bytes() for n in review.SOURCE_FILES}
    row=next(x for x in review.audit_anchors(raw) if x['id']==identity)
    lines=raw[row['path']].decode().splitlines()
    assert row['needle'] in lines[row['line']-1]
    assert 'no control-flow/runtime proof' in row['scope']


def test_anchor_change_refuses_instead_of_searching_other_methods():
    raw={n:(ROOT/n).read_bytes() for n in review.SOURCE_FILES}
    raw[review.G]=raw[review.G].replace(b'd = controller.update(frame, now)',b'd = replacement(frame, now)')
    with pytest.raises(ContractError,match='text_anchor_not_unique:main_update'):review.audit_anchors(raw)


def test_repeated_anchor_in_same_scope_refused():
    raw={n:(ROOT/n).read_bytes() for n in review.SOURCE_FILES}
    needle=b'd = controller.update(frame, now)'
    raw[review.G]=raw[review.G].replace(needle,needle+b'\n                '+needle)
    with pytest.raises(ContractError,match='text_anchor_not_unique:main_update'):review.audit_anchors(raw)


def test_prior_evidence_read_does_not_reexecute_backend_or_model():
    raw={n:(ROOT/n).read_bytes() for n in review.EVIDENCE_FILES}
    out=review.inspect_prior(raw)
    assert out==dict(prior_source_claims=6,prior_cases_read=3,new_guard_execution=False,prior_counterexamples_preserved=True)


def test_prior_case_tamper_refused():
    raw={n:(ROOT/n).read_bytes() for n in review.EVIDENCE_FILES}
    path=review.OLD+'late_joint_resign_counterexample/result.json';d=json.loads(raw[path]);d['python'][0]['deadline_steady_s']=10.8
    raw[path]=json.dumps(d).encode()
    with pytest.raises(ContractError):review.inspect_prior(raw)


def test_no_future_runtime_case_mislabeled_as_new_run(plan):
    assert all(t['status']=='planned_not_executed' for t in plan['test_plan'])
    assert all(r['status']=='not_implemented' for r in plan['requirements'])
    assert all(t['creates_authority'] is False for t in plan['transitions'])


def test_hash_reservation_and_seal_barriers_are_explicit(plan):
    steps=plan['steps']
    assert steps.index('reserve_ack')<steps.index('hash_all_views')<steps.index('seal_ack')
    assert steps.index('seal_ack')<steps.index('save_png')<steps.index('controller_update')
    assert steps.index('poll_and_validate_result')<steps.index('issue_original_deadline')<steps.index('consume_both')


def test_stop_does_not_need_new_frame_but_normal_zero_does(plan):
    rows={r['id']:r for r in plan['transitions']}
    assert rows['stop_without_frame']['frame_required'] is False
    assert rows['normal_zero']['frame_required'] is True
    assert plan['handshake']['deadline_rebased'] is False and plan['handshake']['compatibility_fallback'] is False


@pytest.mark.parametrize('key',['lease_ns','audit_ack_ns','sim_age_ns'])
def test_float_or_boolean_not_a_declared_integer_budget(plan,key):
    for value in (float(plan['limits'][key]),True):
        d=copy.deepcopy(plan);d['limits'][key]=value
        with pytest.raises(ContractError):review.validate_plan(d)


@pytest.mark.parametrize('key',['python_and_native_required','deadline_rebased','compatibility_fallback'])
def test_numeric_boolean_not_accepted_in_handshake(plan,key):
    plan['handshake'][key]=int(plan['handshake'][key])
    with pytest.raises(ContractError):review.validate_plan(plan)


def test_extra_top_level_authority_field_refused(plan):
    plan['allow_motion']=True
    with pytest.raises(ContractError):review.validate_plan(plan)


def test_unknown_and_duplicate_input_sets_have_no_success_path(tmp_path,monkeypatch):
    pin=tmp_path/'pins.json';pin.write_text('{}');out=tmp_path/'out'
    monkeypatch.setattr(review,'audit_anchors',lambda *a:pytest.fail('must validate pins first'))
    for digest in ('0'*64,hashlib.sha256(pin.read_bytes()).hexdigest()):
        with pytest.raises(ContractError):review.run(ROOT,out,pin,digest)
    assert not out.exists()


def test_changed_input_digest_refused_before_output(tmp_path):
    pin=tmp_path/'pins.json';pin.write_text(json.dumps({n:'0'*64 for n in review.INPUT_FILES}));out=tmp_path/'out'
    with pytest.raises(ContractError,match='input_pin_mismatch'):
        review.run(ROOT,out,pin,hashlib.sha256(pin.read_bytes()).hexdigest())
    assert not out.exists()


def test_mutations_do_not_modify_original(plan):
    before=copy.deepcopy(plan)
    for name in review.INVALID:review.invalid_plan(plan,name)
    assert plan==before
    with pytest.raises(ContractError):review.invalid_plan(plan,'not-a-mutation')


def test_tool_has_no_audited_runtime_import_or_native_call():
    tree=ast.parse((ROOT/'tools/precompute_integration_review.py').read_text())
    imports={n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)}
    assert not any(x and x.startswith(('simulation','ros2_ws','tools.frame_deadline_proposal','tools.work_budget_receiver_probe')) for x in imports)
    calls={ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n,ast.Call)}
    assert not calls.intersection({'subprocess.run','time.monotonic','time.sleep','socket.socket','LeaseGuard','ReceiverLease','controller.update'})


def test_all_ids_unique_and_all_declared_anchor_inputs_pinned():
    assert len(review.ANCHORS)==20
    assert len(review.INPUT_FILES)==len(set(review.INPUT_FILES))
    assert {v[0] for v in review.ANCHORS.values()}==set(review.SOURCE_FILES)
