"""A local offline ledger is not one-time consumption or a cross-run owner."""
import ast
import copy
import json
from pathlib import Path
import pytest
from tools import pending_archive_state as m
from tools import pending_archive_state_cases as c

ROOT=Path(__file__).resolve().parents[1]
CASES=c.corpus(ROOT)
BY_NAME={r['case']:r for r in CASES}
BASE=BY_NAME['sequence--exact-repeat']['trace_raw']


@pytest.mark.parametrize('row',CASES,ids=[r['case'] for r in CASES])
def test_closed_decisions_and_offline_state_boundaries(row):
    assert c.check(row,m.review_state_trace(row['trace_raw']),m.review_state_trace(row['trace_raw']))


@pytest.mark.parametrize('value',[None,True,{},[],bytearray(b'{}'),memoryview(b'{}'),'{}',1])
def test_nonbytes_before_hash(value,monkeypatch):
    monkeypatch.setattr(m,'sha',lambda *a:pytest.fail('hash before bytes'))
    assert m.review_state_trace(value)['error']=='state_trace_bytes'


def test_byte_subclass_and_cap_before_hash(monkeypatch):
    class B(bytes):pass
    monkeypatch.setattr(m,'sha',lambda *a:pytest.fail('hash before bound'))
    assert m.review_state_trace(B(BASE))['error']=='state_trace_bytes'
    assert m.review_state_trace(b'x'*(m.TRACE_LIMIT+1))['error']=='state_trace_byte_bound'


@pytest.mark.parametrize('name',['suffix--bad-slot','suffix--extra-key','suffix--bad-text','bound--slots-over','bound--events-over'])
def test_invalid_suffix_never_returns_a_partially_usable_ledger(name):
    out=m.review_state_trace(BY_NAME[name]['trace_raw'])
    assert out['stage']=='state_trace_refused' and out['slots']==out['events']==[]
    assert sum(out['counts'].values())==0 and not out['local_model_review_complete']


@pytest.mark.parametrize('name',['sequence--refused-terminal','sequence--null-unknown-terminal','sequence--stage-unknown-terminal'])
def test_refused_and_unknown_do_not_upgrade_after_late_positive(name):
    out=m.review_state_trace(BY_NAME[name]['trace_raw'])
    assert len({e['after_phase'] for e in out['events']})==1
    assert out['events'][1]['action']=='terminal_not_upgraded'


@pytest.mark.parametrize('name',['sequence--whitespace-is-different','sequence--positive-then-refusal','sequence--positive-then-unknown','sequence--repin-same-slot'])
def test_changed_bytes_close_recorded_slot_and_cannot_reopen(name):
    out=m.review_state_trace(BY_NAME[name]['trace_raw'])
    assert out['slots'][0]['phase']=='local_conflict' and out['slots'][0]['initial_phase']=='local_recorded'
    assert out['events'][1]['action']=='changed_bytes_closed_slot' and out['events'][2]['action']=='conflict_terminal_retained'


def test_duplicate_identity_uses_exact_text_not_sha_equality(monkeypatch):
    monkeypatch.setattr(m,'sha',lambda *a:'0'*64)
    out=m.review_state_trace(BY_NAME['sequence--whitespace-is-different']['trace_raw'])
    assert out['slots'][0]['phase']=='local_conflict'
    assert {e['decision_sha256'] for e in out['events']}=={'0'*64}


def test_same_trace_fresh_invocations_do_not_share_state_or_consume():
    first=m.review_state_trace(BASE)
    assert all(m.review_state_trace(BASE)==first for _ in range(8))
    assert first['slots'][0]['duplicate_count']==2 and first['slots'][0]['changed_bytes_count']==0
    assert not first['persistent_ledger_installed'] and not first['one_time_result_consumption_enforced']


def test_new_scope_and_new_slot_are_not_authenticated_owners():
    aliases=m.review_state_trace(BY_NAME['limitation--alias-new-slot']['trace_raw'])
    renamed=m.review_state_trace(BY_NAME['limitation--renamed-scope']['trace_raw'])
    assert len(aliases['slots'])==2 and renamed['slots'][0]['phase']=='local_recorded'
    assert not aliases['caller_scope_authenticated'] and not renamed['cross_process_owner_enforced']


@pytest.mark.parametrize('name',['limitation--coherent-forgery','limitation--fabricated-metadata','limitation--cross-call-consistent-splice','limitation--same-record-replay'])
def test_old_consistent_forgery_splice_replay_are_not_mistaken_for_fixed(name):
    out=m.review_state_trace(BY_NAME[name]['trace_raw'])
    assert out['slots'][0]['phase']=='local_recorded'
    assert not out['result_tampering_excluded'] and not out['cross_call_splice_excluded']
    assert not out['cross_run_replay_prevented'] and not out['runtime_terminal_latch_installed']


def test_outputs_do_not_alias_future_calls():
    out=m.review_state_trace(BASE);saved=copy.deepcopy(out)
    out['slots'][0]['phase']='forged';out['events'].clear();out['counts'].clear()
    assert m.review_state_trace(BASE)==saved


def test_old_error_null_and_unknown_metadata_not_invented():
    unknown=m.review_state_trace(BY_NAME['sequence--null-unknown-terminal']['trace_raw'])['slots'][0]
    assert unknown['phase']=='local_unknown' and unknown['decision_error'] is unknown['reader_error'] is None
    refused=m.review_state_trace(BY_NAME['sequence--refused-terminal']['trace_raw'])['slots'][0]
    assert refused['decision_error']=='reader_top_level_not_checked' and refused['reader_error']=='parent_digest_mismatch'


def test_old_evidence_and_runtime_gaps_unchanged():
    old=json.loads((ROOT/c.PREVIOUS/'SUMMARY.json').read_bytes())
    assert old['same_label_links']==16 and old['source_error_differences_preserved']==66 and old['old_execution_sites_observed']==0
    assert old['original_four_mismatches_retained'] and old['unknown_late_preserved'] and old['late_joint_resign_original_preserved']
    assert old['old_12266_refusal_preserved'] and old['old_2X_initial_1064_failures_preserved']
    assert old['runtime_requirements_not_implemented']==14 and old['runtime_scenarios_planned_not_executed']==25


def test_bad_pins_precede_model_and_output(tmp_path,monkeypatch):
    p=tmp_path/'pins';p.write_bytes(b'{}');output=tmp_path/'audit'
    monkeypatch.setattr(m,'review_state_trace',lambda *a:pytest.fail('model before pins'))
    with pytest.raises(ValueError,match='input_pin_mismatch'):c.run(ROOT,output,p,c.sha(p.read_bytes()))
    assert not output.exists()


def test_no_live_io_clocks_persistence_or_old_review_calls():
    tree=ast.parse((ROOT/c.SOURCE_FILES[0]).read_bytes())
    assert {a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names}=={'hashlib','json','re'}
    assert {n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)}=={'tools'}
    forbidden={'open','read','read_bytes','write','mkdir','unlink','rename','replace','socket','sleep','time','monotonic',
        'review_reader_decision','read_projection_archive','review_projection','review_archive_binding','review_envelope','eval','exec','__import__'}
    assert not any(isinstance(n,ast.Call) and (n.func.id if isinstance(n.func,ast.Name) else n.func.attr if isinstance(n.func,ast.Attribute) else '') in forbidden for n in ast.walk(tree))
    spec=json.loads((ROOT/'docs/PENDING_ARCHIVE_STATE_V1.json').read_bytes())
    assert spec['case_count']==len(CASES) and spec['old_flags_remain_false']==list(m.FALSE_FIELDS)
