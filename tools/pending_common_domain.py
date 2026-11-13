"""PURE opt-in TPT1 common-domain v1; not a runtime endpoint or old API fix.

Domain preflight always runs, including for STOP/refusal. Inner packets remain
opaque bytes until the unchanged reference reviewer decides to decode them.
"""
from dataclasses import asdict
import json
import struct
from tools import pending_protocol_draft as p

PROFILE='toponav_pending_common_domain_v1'
MAGIC=b'TPT1'
LIMIT=4+5*4+4096+4*(p.MAX_BYTES+1)
KEYS={'operation','required_views','views_frozen','backend','review_ns','old_hold_deadline_ns'}
FALSE_FIELDS=(*p.FALSE_FIELDS,'runtime_one_time_consumption_enforced','cross_run_replay_prevented')


class DomainError(ValueError):
    """Refused before invoking the original message/schema/reference reviewer."""


def need(condition,reason):
    if not condition:raise DomainError(reason)


def _controls(raw):
    # A deliberately closed canonical ASCII dialect, not arbitrary Python JSON.
    # A single contract error is intentional; inner reviewer errors are NOT mapped.
    try:
        need(all(c<128 for c in raw),'domain_controls')
        def integer(s):
            need(len(s)<=19,'domain_controls');n=int(s)
            need(0<=n<=p.MAX_NS,'domain_controls');return n
        def reject(_):raise DomainError('domain_controls')
        c=json.loads(raw,parse_int=integer,parse_float=reject,parse_constant=reject)
        need(type(c) is dict and set(c)==KEYS,'domain_controls')
        need(c['operation'] in ('schema','reference','repeat','reply'),'domain_controls')
        need(c['views_frozen'] is True,'domain_controls')
        views=c['required_views']
        need(type(views) is list and views in [
            ['front','left','right','back'],['front','left','right','back','up'],
            ['front','left','right','back','down'],['front','left','right','back','up','down']], 'domain_controls')
        if c['operation']=='reply':
            need(c['backend'] in ('python','native'),'domain_controls')
            need(type(c['review_ns']) is int and 0<=c['review_ns']<=p.MAX_NS,'domain_controls')
            n=c['old_hold_deadline_ns']
            need(n is None or type(n) is int and 0<=n<=p.MAX_NS,'domain_controls')
        else:need(all(c[k] is None for k in ('backend','review_ns','old_hold_deadline_ns')),'domain_controls')
        need(json.dumps(c,sort_keys=True,separators=(',',':'),ensure_ascii=True,allow_nan=False).encode()==raw,'domain_controls')
        return c
    except (ValueError,TypeError,RecursionError,UnicodeError) as exc:
        raise DomainError('domain_controls') from exc


def preflight(raw):
    """Return controls and unmodified byte slices; acceptance is NOT schema success."""
    need(type(raw) is bytes,'domain_bytes')
    need(len(raw)<=LIMIT,'domain_byte_bound')
    need(raw[:4]==MAGIC,'domain_magic');pos=4;fields=[]
    for i in range(5):
        need(pos+4<=len(raw),'domain_length');n=struct.unpack('>I',raw[pos:pos+4])[0];pos+=4
        if n==0xffffffff:
            need(i>=2,'domain_required_field');fields.append(None);continue
        need(n<=(4096 if i==0 else p.MAX_BYTES+1) and n<=len(raw)-pos,'domain_field_bound')
        fields.append(raw[pos:pos+n]);pos+=n
    need(pos==len(raw),'domain_trailing');controls=_controls(fields[0])
    need(controls['operation']!='reply' or fields[4] is not None,'domain_reply_required')
    return controls,tuple(fields[1:])


def review_envelope(raw):
    """Pure result with explicit domain/review stages; no clock reads or IO."""
    out=dict(profile=PROFILE,classification='offline_common_domain_not_admission',
        domain_accepted=False,stage='domain_refused',consistent=False,result=None,error=None,
        actual_exposure_wall_s=None,**{k:False for k in FALSE_FIELDS})
    try:c,(message,reserve,seal,reply)=preflight(raw)
    except DomainError as exc:out['error']=str(exc);return out
    out.update(domain_accepted=True,stage='review_refused')
    try:
        op=c['operation'];kwargs=dict(required_views=tuple(c['required_views']))
        if op=='schema':
            decoded=p.decode_message(message)
            review=p.DraftReview('decoded_schema_not_admission',decoded.kind,False)
        elif op=='reply':
            review=p.review_reply(message,reply,reserve,seal,backend=c['backend'],review_ns=c['review_ns'],
                old_hold_deadline_ns=c['old_hold_deadline_ns'],**kwargs)
        else:review=p.review_reference(message,reserve,seal,**kwargs)
        repeated=asdict(p.review_reference(message,reserve,seal,**kwargs)) if op=='repeat' else None
        out.update(stage='review_returned',consistent=True,result=dict(review=asdict(review),repeated_review=repeated))
    except p.DraftError as exc:out['error']=str(exc)
    return out
