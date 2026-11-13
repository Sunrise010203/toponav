"""Offline caller transcript claims only; never invoke a runtime backend."""
import ast
from dataclasses import asdict, FrozenInstanceError
import hashlib
import json
from pathlib import Path

import pytest

from tools import pending_protocol_draft as p
from tools import pending_trace_review as t
from tools import pending_trace_cases as c

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('name',c.VALID)
def test_consistent_claims_have_no_authority(name):
    result=c.evaluate(c.fixture(name),name=='repeat_document');review=result['review']
    assert review['consistent'] and not review['violations']
    assert review['classification']=='caller_transcript_order_only'
    assert review['actual_exposure_wall_s'] is None and all(review[k] is False for k in t.FALSE_FIELDS)


@pytest.mark.parametrize('name',c.INVALID)
def test_invalid_trace_refused_and_semantic_evidence_kept(name):
    try:review=t.review_trace(c.fixture(name))
    except p.DraftError:return
    assert not review.consistent and review.violations and review.disposition=='rejected_transcript'
    assert review.event_count==len(json.loads(c.fixture(name))['events'])
    assert all(asdict(review)[k] is False for k in t.FALSE_FIELDS)


def test_unknown_result_preserves_partial_confirmation_without_upgrade():
    review=t.review_trace(c.fixture('unknown_result_native'))
    rows=[s for s in review.submissions if s.kind=='result_ref']
    assert [(s.backend,s.outcome) for s in rows]==[('python','declared_confirmed'),('native','unknown_outcome')]
    assert review.disposition=='failed_claim' and not review.runtime_result_consumed


def test_late_reply_cannot_turn_unknown_into_confirmed():
    review=t.review_trace(c.fixture('late_reply_retains_unknown'))
    entry=next(s for s in review.submissions if s.kind=='reserve' and s.backend=='native')
    assert entry.outcome=='unknown_outcome' and len(entry.reply_indices)==1
    assert review.failures and review.disposition=='failed_claim'


@pytest.mark.parametrize('name',['wrong_send_actor','wrong_send_target','duplicate_send','retry_after_unknown'])
def test_bad_send_attempts_not_silently_dropped_from_ledger(name):
    raw=c.fixture(name);review=t.review_trace(raw);doc=json.loads(raw)
    attempted=sum(e['action']=='send' for e in doc['events'])
    assert len(review.submissions)==attempted


@pytest.mark.parametrize('name',['hash_failure','save_failure','compute_failure','cancel_before_compute'])
def test_original_exception_and_stop_before_cancel_are_claims_only(name):
    raw=c.fixture(name);doc=json.loads(raw);review=t.review_trace(raw)
    assert review.disposition=='failed_claim' and review.failures
    failure=next(e['error'] for e in doc['events'] if e['action']=='failure')
    assert failure in [error for _,error in review.failures]
    cancel=next(i for i,e in enumerate(doc['events']) if e['action']=='cancel')
    assert all(s.send_index<cancel for s in review.submissions if s.kind=='stop')
    assert not review.physical_stop_proved


@pytest.mark.parametrize('name',['stop_no_frame','stop_after_old_expiry'])
def test_stop_needs_no_origin_or_live_budget_and_does_not_prove_stop(name):
    review=t.review_trace(c.fixture(name))
    assert review.consistent and review.disposition=='stopped_claim'
    assert {s.kind for s in review.submissions}=={'stop'}
    assert all(s.outcome=='unconfirmed' for s in review.submissions)
    assert not review.physical_stop_proved


def test_jointly_replaced_context_is_still_not_authenticated():
    old=json.loads(c.fixture('normal'));new=json.loads(c.fixture('joint_context_replacement'))
    assert json.loads(new['events'][0]['wire'])['record']['deadline_ns']>json.loads(old['events'][0]['wire'])['record']['deadline_ns']
    assert t.review_trace(c.fixture('joint_context_replacement')).consistent
    assert not t.review_trace(c.fixture('joint_context_replacement')).provenance_authenticated


def test_repeat_document_is_not_cross_call_consumption_or_replay_control():
    raw=c.fixture('normal');a=t.review_trace(raw);b=t.review_trace(raw)
    assert a==b and not a.runtime_one_time_consumption_enforced and not a.cross_run_replay_prevented


def test_timely_replacement_does_not_permanently_clip_new_completion_to_old_hold():
    raw=c.fixture('finish_after_replaced_old_hold');doc=json.loads(raw)
    assert doc['events'][-1]['at_ns']>doc['old_hold_deadline_ns']
    assert all(e['at_ns']<doc['old_hold_deadline_ns'] for e in doc['events'][:-1])
    assert t.review_trace(raw).consistent


def test_failed_prefix_not_repaired_by_success_looking_suffix():
    review=t.review_trace(c.fixture('success_suffix_after_failure'))
    assert not review.consistent and review.failures and review.disposition=='rejected_transcript'


def test_observation_must_follow_claimed_send_not_only_precede_review():
    review=t.review_trace(c.fixture('reply_observed_before_send'))
    assert 'reply_before_send' in [code for _,code in review.violations]


@pytest.mark.parametrize('action',['freeze','send','reply','hash_begin','hash_done','save_begin','save_done',
    'compute_begin','compute_done','result','finish'])
def test_exact_role_direction_for_every_normal_event(action):
    doc=c.normal();index=c.locate(doc,action);doc['events'][index]['actor']='unauthorized-role'
    review=t.review_trace(c.wire(doc).encode())
    assert (index,'event_direction') in review.violations or (index,'reply_actor') in review.violations


@pytest.mark.parametrize('raw',[None,True,{},bytearray(b'{}'),'{}',b'',b'\xff',b'{} {}',b'null'])
def test_only_bounded_utf8_bytes_document(raw):
    with pytest.raises(p.DraftError):t.review_trace(raw)


@pytest.mark.parametrize('value',[True,-1,1.,2**63,'10',None])
def test_bad_event_time_rejected(value):
    doc=c.normal();doc['events'][0]['at_ns']=value
    with pytest.raises(p.DraftError):t.review_trace(c.wire(doc).encode())


@pytest.mark.parametrize('value',[True,-1,1.,2**63,'10'])
def test_bad_old_hold_rejected(value):
    doc=c.normal();doc['old_hold_deadline_ns']=value
    with pytest.raises(p.DraftError):t.review_trace(c.wire(doc).encode())


def test_resource_limits_checked_before_json_loader(monkeypatch):
    monkeypatch.setattr(t.json,'loads',lambda *a,**k:pytest.fail('must reject first'))
    for raw in (b' '*(t.MAX_BYTES+1),b'['*(t.MAX_DEPTH+1)+b'0'+b']'*(t.MAX_DEPTH+1)):
        with pytest.raises(p.DraftError):t.review_trace(raw)


def test_event_capacity_is_offline_resource_limit_not_production_pending_capacity():
    doc=c.normal();doc['events']=[doc['events'][0]]*(t.MAX_EVENTS+1)
    with pytest.raises(p.DraftError,match='trace_event_bound'):t.review_trace(c.wire(doc).encode())


def test_frozen_output_is_not_a_token():
    review=t.review_trace(c.fixture('normal'))
    with pytest.raises(FrozenInstanceError):review.consistent=False
    with pytest.raises(FrozenInstanceError):review.submissions[0].outcome='applied'
    with pytest.raises(p.DraftError):t.review_trace(review)


def test_pure_imports_no_runtime_clock_or_io():
    tree=ast.parse((ROOT/'tools/pending_trace_review.py').read_bytes())
    imports={n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)}
    imports|={a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names}
    assert imports=={'dataclasses','json','tools'}
    calls={ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n,ast.Call)}
    assert not calls.intersection({'open','time.monotonic','time.sleep','socket.socket','subprocess.run'})


def test_case_names_are_unique_and_unknown_case_rejected():
    assert len(c.VALID+c.INVALID)==len(set(c.VALID+c.INVALID))
    with pytest.raises(p.DraftError):c.fixture('unknown')


def test_pin_failures_precede_output_or_execution(tmp_path,monkeypatch):
    pins=tmp_path/'pins.json';pins.write_text('{}');out=tmp_path/'out'
    monkeypatch.setattr(c,'fixture',lambda *a:pytest.fail('pins first'))
    for digest in ('0'*64,hashlib.sha256(pins.read_bytes()).hexdigest()):
        with pytest.raises(p.DraftError):c.run(ROOT,out,pins,digest)
    assert not out.exists()
