"""PURE, OFFLINE protocol draft; NOT the production wire, owner or admission gate.

Only bytes + caller-provided synthetic snapshots. No IO, runtime imports, clock,
callbacks, registry, one-time consumption or authentication. Even a matching
ACK/status is a CLAIM, not evidence that a backend installed/consumed anything.
"""
from dataclasses import dataclass, field
import json

PROTOCOL='toponav_pending_offline_draft_v1'
SCOPE='offline_schema_only'
CAPABILITIES=('reserve','seal','result_ref','pending_status')
MAX_BYTES=8192
MAX_DEPTH=8
MAX_NS=2**63-1
MAX_ID=2**31-1
LEASE_NS=800_000_000
AUDIT_ACK_NS=400_000_000
CLOCK='synthetic_monotonic_ns'
BASE=('protocol','scope','kind','session_id','request_id')
RECORD_KEYS=('clock_domain','host_boot_id','generation','frame_id','captured_sim_ns','required_views','available_ns','deadline_ns')
FALSE_FIELDS=('provenance_authenticated','exposure_age_bounded','shadow_admission_authorized',
    'native_live_observation_authorized','motion_authority','independent_enforcement_observed',
    'physical_stop_proved','production_guard_fixed','runtime_pending_installed','runtime_result_consumed')


class DraftError(ValueError):pass


def check(ok,code):
    if not ok:raise DraftError(code)


def shape(doc,keys,code):check(type(doc) is dict and set(doc)==set(keys),code)
def uint(value,maximum=MAX_NS):return type(value) is int and 0<=value<=maximum


def identity(value):
    return type(value) is str and 1<=len(value)<=128 and value.isascii() and all(
        c.isalnum() or c in '-_/.:' for c in value)


def views(value):
    check(type(value) is list and 4<=len(value)<=6 and all(type(v) is str for v in value),'view_schema')
    check(value[:4]==['front','left','right','back'] and len(set(value))==len(value)
        and value[4:] in ([],['up'],['down'],['up','down']),'view_order_or_duplicate')
    return tuple(value)


@dataclass(frozen=True)
class BudgetRecord:
    clock_domain: str
    host_boot_id: str
    generation: int
    frame_id: str
    captured_sim_ns: int
    required_views: tuple[str,...]
    available_ns: int
    deadline_ns: int


@dataclass(frozen=True)
class Snapshot:
    backend: str
    phase: str
    record: BudgetRecord | None
    view_digests: tuple[tuple[str,str],...]
    seal_request_id: int | None
    observed_ns: int


@dataclass(frozen=True)
class DraftMessage:
    kind: str
    session_id: str
    request_id: int
    record: BudgetRecord | None = None
    view_digests: tuple[tuple[str,str],...] = ()
    seal_request_id: int | None = None
    for_kind: str | None = None
    declared_accepted: bool | None = None
    snapshot: Snapshot | None = None


@dataclass(frozen=True)
class DraftReview:
    classification: str
    message_kind: str
    synthetic_reference_matches: bool
    backend_claim: str | None = None
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


def _record(doc):
    shape(doc,RECORD_KEYS,'record_schema')
    check(doc['clock_domain']==CLOCK and identity(doc['host_boot_id']),'synthetic_clock_identity')
    check(uint(doc['generation'],MAX_ID) and doc['generation']>0 and identity(doc['frame_id']),'record_identity')
    check(uint(doc['captured_sim_ns']) and uint(doc['available_ns'],MAX_NS-LEASE_NS)
        and uint(doc['deadline_ns']),'record_ns')
    check(doc['deadline_ns']==doc['available_ns']+LEASE_NS,'original_work_budget')
    return BudgetRecord(doc['clock_domain'],doc['host_boot_id'],doc['generation'],doc['frame_id'],
        doc['captured_sim_ns'],views(doc['required_views']),doc['available_ns'],doc['deadline_ns'])


def _digests(value,record):
    check(type(value) is list and len(value)==len(record.required_views),'digest_view_count')
    result=[]
    for pair,expected in zip(value,record.required_views):
        check(type(pair) is list and len(pair)==2 and pair[0]==expected,'digest_view_order')
        h=pair[1];check(type(h) is str and len(h)==64 and all(c in '0123456789abcdef' for c in h),'digest_schema')
        result.append((pair[0],h))
    return tuple(result)


def _snapshot(doc):
    shape(doc,('backend','phase','record','view_digests','seal_request_id','observed_ns'),'snapshot_schema')
    check(doc['backend'] in ('python','native') and type(doc['backend']) is str,'backend_schema')
    phase=doc['phase'];check(type(phase) is str and phase in ('disarmed','reserved','sealed','consumed','tripped'),'phase_schema')
    check(uint(doc['observed_ns']),'snapshot_ns')
    record=None if doc['record'] is None else _record(doc['record'])
    sealed=doc['seal_request_id'] is not None
    check(not sealed or uint(doc['seal_request_id'],MAX_ID) and doc['seal_request_id']>0,'seal_reference_schema')
    if phase=='disarmed':check(record is None and not sealed,'disarmed_has_no_record')
    elif phase in ('reserved','sealed','consumed'):
        check(record is not None and sealed==(phase!='reserved'),'phase_record_binding')
    else:check(record is not None or not sealed,'tripped_record_binding')
    if sealed:
        check(record is not None,'sealed_record_required');digests=_digests(doc['view_digests'],record)
    else:
        check(type(doc['view_digests']) is list and doc['view_digests']==[],'unsealed_no_digests');digests=()
    return Snapshot(doc['backend'],phase,record,digests,doc['seal_request_id'],doc['observed_ns'])


def _pairs(pairs):
    check(len(pairs)<=16,'object_field_bound');out={}
    for key,value in pairs:
        check(type(key) is str and len(key)<=64,'object_key_bound')
        check(key not in out,'duplicate_field');out[key]=value
    return out


def _no_number(value):raise DraftError('integer_json_only')


def _json(raw):
    check(type(raw) is bytes and 1<=len(raw)<=MAX_BYTES,'message_byte_bound')
    try:text=raw.decode('utf-8')
    except UnicodeError as exc:raise DraftError('message_utf8') from exc
    depth=0;quoted=False;escaped=False
    for c in text:
        if quoted:
            if escaped:escaped=False
            elif c=='\\':escaped=True
            elif c=='"':quoted=False
        elif c=='"':quoted=True
        elif c in '[{':
            depth+=1;check(depth<=MAX_DEPTH,'json_depth_bound')
        elif c in ']}':depth-=1
    try:return json.loads(text,object_pairs_hook=_pairs,parse_float=_no_number,parse_constant=_no_number)
    except (ValueError,RecursionError) as exc:
        if isinstance(exc,DraftError):raise
        raise DraftError('message_json') from exc


def decode_message(raw):
    """Bounded syntax/schema validation only. No registration or authentication."""
    doc=_json(raw);check(type(doc) is dict and type(doc.get('kind')) is str,'message_object_kind')
    kind=doc['kind'];check(kind in ('reserve','seal','result_ref','ack','status','stop'),'message_kind')
    fields={
        'reserve':('capabilities','record'),
        'seal':('capabilities','record','view_digests'),
        'result_ref':('capabilities','record','view_digests','seal_request_id'),
        'ack':('capabilities','for_kind','accepted','status'),
        'status':('capabilities','status'),
        'stop':('reason',),
    }
    shape(doc,BASE+fields[kind],'message_fields')
    check(doc['protocol']==PROTOCOL and doc['scope']==SCOPE,'offline_protocol_only')
    check(identity(doc['session_id']) and uint(doc['request_id'],MAX_ID) and doc['request_id']>0,'message_identity')
    if kind=='stop':
        check(type(doc['reason']) is str and doc['reason'] in ('cancel','terminal','failure','shutdown'),'stop_reason')
        return DraftMessage(kind,doc['session_id'],doc['request_id'])
    check(type(doc['capabilities']) is list and doc['capabilities']==list(CAPABILITIES),'no_capability_downgrade')
    if kind in ('reserve','seal','result_ref'):
        record=_record(doc['record']);digests=() if kind=='reserve' else _digests(doc['view_digests'],record)
        seal=doc.get('seal_request_id')
        if kind=='result_ref':check(uint(seal,MAX_ID) and seal>0,'seal_reference_schema')
        return DraftMessage(kind,doc['session_id'],doc['request_id'],record,digests,seal)
    snapshot=_snapshot(doc['status'])
    if kind=='status':return DraftMessage(kind,doc['session_id'],doc['request_id'],snapshot=snapshot)
    check(type(doc['for_kind']) is str and doc['for_kind'] in ('reserve','seal','result_ref','stop'),'ack_kind')
    check(type(doc['accepted']) is bool,'ack_accepted_bool')
    if doc['accepted']:
        check(snapshot.phase==dict(reserve='reserved',seal='sealed',result_ref='consumed',stop='tripped')[doc['for_kind']],'ack_phase')
    else:check(snapshot.phase=='tripped','refusal_phase')
    return DraftMessage(kind,doc['session_id'],doc['request_id'],for_kind=doc['for_kind'],
        declared_accepted=doc['accepted'],snapshot=snapshot)


def _reference(message,reserve,seal,required_views):
    check(type(required_views) is tuple,'frozen_view_config')
    configured=views(list(required_views))
    check(message.kind in ('reserve','seal','result_ref') and reserve.kind=='reserve','reference_kind')
    check(message.session_id==reserve.session_id and message.record==reserve.record,'original_record_reference')
    check(reserve.record.required_views==configured,'frozen_view_config_mismatch')
    if message.kind=='reserve':check(message.request_id==reserve.request_id,'reserve_request_reference')
    else:check(message.request_id>reserve.request_id,'request_order')
    if message.kind=='result_ref':
        check(seal is not None and seal.kind=='seal','seal_reference_required')
        _reference(seal,reserve,None,required_views)
        check(message.view_digests==seal.view_digests and message.seal_request_id==seal.request_id
            and message.request_id>seal.request_id,'sealed_result_reference')


def review_reference(message_raw,reserve_raw=None,seal_raw=None,*,required_views=('front','left','right','back')):
    """Compare caller-provided snapshots, NOT a trusted registry or state machine.

    Repeating this function consumes nothing. Jointly replacing the caller's
    original reserve AND subsequent messages can remain structurally consistent.
    STOP needs no reserve/seal/views. No result carries velocity/issued/expires.
    """
    message=decode_message(message_raw)
    if message.kind=='stop':return DraftReview('frame_free_stop_schema_only','stop',True)
    reserve=decode_message(reserve_raw);seal=None if seal_raw is None else decode_message(seal_raw)
    _reference(message,reserve,seal,required_views)
    return DraftReview('caller_snapshot_reference_only',message.kind,True)


def review_reply(request_raw,reply_raw,reserve_raw=None,seal_raw=None,*,backend,review_ns,
                 required_views=('front','left','right','back'),old_hold_deadline_ns=None):
    """Compare a synthetic ACK/status CLAIM with a request and supplied context.

    Timeliness is arithmetic in one caller-declared fictional clock domain.
    No actual poll, previous-call memory, backend installation or consumption.
    """
    request=decode_message(request_raw);reply=decode_message(reply_raw)
    check(request.kind in ('reserve','seal','result_ref','stop') and reply.kind in ('ack','status'),'reply_kind')
    check(type(backend) is str and backend in ('python','native') and reply.snapshot.backend==backend,'reply_backend')
    check(uint(review_ns) and (old_hold_deadline_ns is None or uint(old_hold_deadline_ns)),'review_ns')
    check(reply.session_id==request.session_id and reply.request_id==request.request_id,'reply_request_reference')
    check(reply.snapshot.observed_ns<=review_ns,'future_reply_claim')
    if reply.kind=='ack':check(reply.for_kind==request.kind,'reply_for_kind')
    if reply.kind=='ack' and reply.declared_accepted is False:
        return DraftReview('declared_refusal_not_execution',request.kind,False,backend)
    expected=dict(reserve='reserved',seal='sealed',result_ref='consumed',stop='tripped')[request.kind]
    check(reply.snapshot.phase==expected,'reply_phase')
    if request.kind!='stop':
        reserve=decode_message(reserve_raw);seal=None if seal_raw is None else decode_message(seal_raw)
        _reference(request,reserve,seal,required_views)
        snapshot=reply.snapshot
        check(snapshot.record==request.record,'reply_record_reference')
        check(snapshot.view_digests==request.view_digests,'reply_digest_reference')
        wanted=None if request.kind=='reserve' else request.request_id if request.kind=='seal' else request.seal_request_id
        check(snapshot.seal_request_id==wanted,'reply_seal_reference')
        deadline=min(x for x in (request.record.deadline_ns,old_hold_deadline_ns) if x is not None)
        check(request.record.available_ns<=snapshot.observed_ns<=review_ns<deadline,'reply_budget_exhausted')
    return DraftReview('synthetic_reply_claim_only',request.kind,True,backend)
