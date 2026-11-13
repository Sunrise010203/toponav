"""PURE offline transcript lint, not an owner, registry, permit or execution gate.

All events (including sends, ACKs and timestamps) are caller claims. Local
variables remember order ONLY within one supplied document. A second call is
independent; no real pending, consumption, stopping or authentication happens.
"""
from dataclasses import dataclass, field
import json

from tools import pending_protocol_draft as p

FORMAT='pending_trace_claims_v1'
SCOPE='offline_transcript_only'
MAX_BYTES=262144
MAX_EVENTS=128
MAX_DEPTH=12
BACKENDS=('python','native')
WIRE_ACTIONS=('freeze','hash_done','result','send','reply','late_reply')
LOCAL_ACTIONS=('hash_begin','save_begin','save_done','cancel')
FALSE_FIELDS=p.FALSE_FIELDS+('runtime_one_time_consumption_enforced','cross_run_replay_prevented')


@dataclass(frozen=True)
class Submission:
    request_id: int
    kind: str
    backend: str
    send_index: int
    send_ns: int
    outcome: str
    reply_indices: tuple[int,...]


@dataclass(frozen=True)
class TraceReview:
    consistent: bool
    disposition: str
    event_count: int
    violations: tuple[tuple[int,str],...]
    failures: tuple[tuple[int,str],...]
    submissions: tuple[Submission,...]
    classification: str = field(default='caller_transcript_order_only',init=False)
    actual_exposure_wall_s: None = field(default=None,init=False)
    provenance_authenticated: bool = field(default=False,init=False)
    exposure_age_bounded: bool = field(default=False,init=False)
    shadow_admission_authorized: bool = field(default=False,init=False)
    native_live_observation_authorized: bool = field(default=False,init=False)
    motion_authority: bool = field(default=False,init=False)
    independent_enforcement_observed: bool = field(default=False,init=False)
    physical_stop_proved: bool = field(default=False,init=False)
    production_guard_fixed: bool = field(default=False,init=False)
    runtime_pending_installed: bool = field(default=False,init=False)
    runtime_result_consumed: bool = field(default=False,init=False)
    runtime_one_time_consumption_enforced: bool = field(default=False,init=False)
    cross_run_replay_prevented: bool = field(default=False,init=False)


def _parse(raw):
    p.check(type(raw) is bytes and 1<=len(raw)<=MAX_BYTES,'trace_byte_bound')
    try:text=raw.decode('utf-8')
    except UnicodeError as exc:raise p.DraftError('trace_utf8') from exc
    depth=0;quoted=False;escaped=False
    for c in text:
        if quoted:
            if escaped:escaped=False
            elif c=='\\':escaped=True
            elif c=='"':quoted=False
        elif c=='"':quoted=True
        elif c in '[{':
            depth+=1;p.check(depth<=MAX_DEPTH,'trace_depth_bound')
        elif c in ']}':depth-=1
    try:return json.loads(text,object_pairs_hook=p._pairs,parse_float=p._no_number,parse_constant=p._no_number)
    except (ValueError,RecursionError) as exc:
        if isinstance(exc,p.DraftError):raise
        raise p.DraftError('trace_json') from exc


def _schema(raw):
    doc=_parse(raw)
    p.shape(doc,('format','scope','session_id','required_views','old_hold_deadline_ns','events'),'trace_fields')
    p.check(doc['format']==FORMAT and doc['scope']==SCOPE and p.identity(doc['session_id']),'trace_namespace')
    p.views(doc['required_views'])
    p.check(doc['old_hold_deadline_ns'] is None or p.uint(doc['old_hold_deadline_ns']),'trace_old_hold')
    p.check(type(doc['events']) is list and 1<=len(doc['events'])<=MAX_EVENTS,'trace_event_bound')
    for e in doc['events']:
        p.check(type(e) is dict and type(e.get('action')) is str,'event_action')
        action=e['action'];fields=('action','actor','target','at_ns')
        if action in WIRE_ACTIONS:fields+=('wire',)
        elif action in ('failure','diagnostic'):fields+=('error',)
        elif action=='ack_unknown':fields+=('request_id','backend','error')
        elif action=='finish':fields+=('outcome',)
        else:p.check(action in LOCAL_ACTIONS+('compute_begin','compute_done'),'event_action')
        p.shape(e,fields,'event_fields')
        p.check(type(e['actor']) is str and type(e['target']) is str and p.uint(e['at_ns']),'event_identity_time')
        if action in WIRE_ACTIONS:
            p.check(type(e['wire']) is str and len(e['wire'])<=p.MAX_BYTES,'event_wire')
        if 'error' in e:
            p.check(type(e['error']) is str and 1<=len(e['error'])<=256
                and all(32<=ord(c)<=126 for c in e['error']),'event_error')
        if action=='ack_unknown':
            p.check(p.uint(e['request_id'],p.MAX_ID) and e['request_id']>0
                and type(e['backend']) is str and e['backend'] in BACKENDS,'unknown_ack_identity')
        if action=='finish':p.check(type(e['outcome']) is str and e['outcome'] in
            ('completed_claim','failed_claim','stopped_claim'),'finish_outcome')
    return doc


def review_trace(raw):
    """Review one bounded caller-authored trace, including failed/unknown ledgers.

    Schema failures raise DraftError (raw bytes must be retained by the caller).
    Semantic errors accumulate in the returned report; they cannot be repaired
    by a later successful-looking suffix. Every decodable send attempt is kept.
    """
    doc=_schema(raw);events=doc['events'];configured=tuple(doc['required_views'])
    reserve=seal=result=None;requests={};sent=[];violations=[];failures=[]
    fault=False;stopping=False;finished=False;cancelled=False;diagnosed=False
    hash_started=save_started=saved=compute_started=computed=False
    previous=-1;disposition='incomplete_claim'

    def both(kind):
        return all(any(s['kind']==kind and s['backend']==b and s['outcome']=='declared_confirmed'
            for s in sent) for b in BACKENDS)

    def stop_sent():
        return all(any(s['kind']=='stop' and s['backend']==b for s in sent) for b in BACKENDS)

    def register(message,wire):
        p.check(message.request_id not in requests and (not requests or message.request_id>max(requests)),
            'request_reuse_or_regression')
        requests[message.request_id]=(message,wire)

    def budget(now,check_old_hold=True):
        p.check(reserve is not None,'origin_missing')
        record=p.decode_message(reserve).record
        deadlines=[record.deadline_ns]
        if check_old_hold and doc['old_hold_deadline_ns'] is not None:deadlines.append(doc['old_hold_deadline_ns'])
        p.check(record.available_ns<=now<min(deadlines),'trace_budget_exhausted')

    for index,e in enumerate(events):
        action=e['action'];now=e['at_ns'];message=wire=None;submission=None
        try:
            # Capture sends before semantic checks so a bad direction, duplicate
            # or already-failed suffix does not erase an attempted submission.
            if action in WIRE_ACTIONS:
                try:wire=e['wire'].encode('utf-8')
                except UnicodeError as exc:raise p.DraftError('event_wire_utf8') from exc
                message=p.decode_message(wire)
                if action=='send':
                    submission=dict(request_id=message.request_id,kind=message.kind,backend=e['target'],
                        send_index=index,send_ns=now,outcome='unconfirmed',reply_indices=[])
                    sent.append(submission)
                p.check(message.session_id==doc['session_id'],'trace_session')
            p.check(now>=previous,'trace_time_regression');previous=max(now,previous)
            p.check(not finished,'event_after_finish')
            if action=='send':expected=('owner',e['target']);p.check(e['target'] in BACKENDS,'send_backend')
            elif action in ('reply','late_reply'):expected=(e['actor'],'owner');p.check(e['actor'] in BACKENDS,'reply_actor')
            elif action=='freeze':expected=('capture','owner')
            elif action in ('result','compute_done'):expected=('controller','owner')
            elif action=='compute_begin':expected=('owner','controller')
            else:expected=('owner','local')
            p.check((e['actor'],e['target'])==expected,'event_direction')
            p.check(not (fault or stopping) or action in ('send','reply','late_reply','ack_unknown','failure',
                'cancel','diagnostic','finish'),'work_after_failure_or_stop')

            if action=='freeze':
                p.check(reserve is None and message.kind=='reserve','single_origin_only')
                p.review_reference(wire,wire,required_views=configured)
                p.check(now==message.record.available_ns,'origin_claim_time')
                register(message,wire);reserve=wire;budget(now)
            elif action=='hash_begin':
                budget(now);p.check(both('reserve') and not hash_started,'reserve_barrier_before_hash');hash_started=True
            elif action=='hash_done':
                budget(now);p.check(hash_started and seal is None and message.kind=='seal','hash_done_stage')
                p.review_reference(wire,reserve,required_views=configured);register(message,wire);seal=wire
            elif action=='save_begin':
                budget(now);p.check(both('seal') and not save_started,'seal_barrier_before_save');save_started=True
            elif action=='save_done':
                budget(now);p.check(save_started and not saved,'save_done_stage');saved=True
            elif action=='compute_begin':
                budget(now);p.check(both('seal') and saved and not compute_started,'seal_save_barrier_before_compute');compute_started=True
            elif action=='compute_done':
                budget(now);p.check(compute_started and not computed,'compute_done_stage');computed=True
            elif action=='result':
                budget(now);p.check(computed and result is None and message.kind=='result_ref','result_stage_or_duplicate')
                p.review_reference(wire,reserve,seal,required_views=configured);register(message,wire);result=wire
            elif action=='send':
                if message.kind=='stop':
                    if not stopping:register(message,wire)
                    stopping=True
                else:
                    p.check(not fault and not stopping,'nonstop_after_failure_or_stop');budget(now)
                    wanted={'reserve':reserve,'seal':seal,'result_ref':result}.get(message.kind)
                    p.check(wanted is not None and wanted==wire,'send_unprepared_or_changed')
                p.check(message.request_id in requests and requests[message.request_id][1]==wire,'send_request_reference')
                p.check(not any(s is not submission and s['request_id']==message.request_id
                    and s['backend']==e['target'] for s in sent),'duplicate_send_no_retry')
            elif action in ('reply','late_reply','ack_unknown'):
                backend=e['backend'] if action=='ack_unknown' else e['actor']
                request_id=e['request_id'] if action=='ack_unknown' else message.request_id
                matches=[s for s in sent if s['request_id']==request_id and s['backend']==backend]
                p.check(len(matches)==1 and request_id in requests,'unsolicited_or_ambiguous_reply')
                entry=matches[0];request,request_wire=requests[request_id]
                if action=='late_reply':
                    p.check(fault and entry['outcome']!='declared_confirmed','late_reply_requires_uncertainty')
                    p.check(message.kind in ('ack','status') and message.snapshot.backend==backend,'late_reply_backend_kind')
                    if message.kind=='ack':p.check(message.for_kind==request.kind,'late_reply_for_kind')
                    entry['reply_indices'].append(index)  # Deliberately do NOT upgrade it.
                elif action=='ack_unknown':
                    p.check(entry['outcome']=='unconfirmed','unknown_ack_stage')
                    entry['outcome']='unknown_outcome';fault=True;failures.append((index,e['error']))
                else:
                    p.check(entry['outcome']=='unconfirmed','duplicate_or_post_unknown_reply')
                    p.check(not fault and not stopping or request.kind=='stop','reply_after_failure')
                    reviewed=p.review_reply(request_wire,wire,reserve,seal,backend=backend,review_ns=now,
                        required_views=configured,old_hold_deadline_ns=doc['old_hold_deadline_ns'])
                    # The 2N comparison only checked <= review; a transcript must
                    # additionally put an observation after its claimed send.
                    p.check(message.snapshot.observed_ns>=entry['send_ns'],'reply_before_send')
                    entry['reply_indices'].append(index)
                    if reviewed.classification=='declared_refusal_not_execution':
                        entry['outcome']='declared_refusal';fault=True;failures.append((index,'declared_refusal'))
                    else:entry['outcome']='declared_confirmed'
            elif action=='failure':fault=True;failures.append((index,e['error']))
            elif action=='cancel':
                p.check(stop_sent() and not cancelled,'stop_before_cancel');cancelled=True
            elif action=='diagnostic':
                p.check(stop_sent() and cancelled and fault and not diagnosed,'stop_cancel_before_diagnostic')
                p.check(e['error'] in [v for _,v in failures],'diagnostic_preserves_original_error');diagnosed=True
            elif action=='finish':
                outcome=e['outcome']
                if outcome=='completed_claim':
                    p.check(not fault and not stopping and both('result_ref'),'completed_requires_both_result_claims')
                    # Both timely result confirmations are already in the
                    # transcript. Do not permanently clip the claimed new
                    # frame's completion to the replaced OLD hold.
                    budget(now,check_old_hold=False)
                else:
                    p.check(stop_sent() and cancelled,'finish_requires_stop_and_cancel')
                    p.check((fault and diagnosed) if outcome=='failed_claim' else not fault,'finish_failure_classification')
                disposition=outcome;finished=True
        except p.DraftError as exc:
            violations.append((index,str(exc)));fault=True
    if not finished:violations.append((len(events),'missing_finish'))
    if violations:disposition='rejected_transcript'
    rows=tuple(Submission(s['request_id'],s['kind'],s['backend'],s['send_index'],s['send_ns'],s['outcome'],
        tuple(s['reply_indices'])) for s in sent)
    return TraceReview(not violations,disposition,len(events),tuple(violations),tuple(failures),rows)
