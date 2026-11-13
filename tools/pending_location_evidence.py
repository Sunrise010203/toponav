"""PURE opt-in readonly byte evidence; NEVER observes an old validator's PC.

The supplied review snapshot remains an untrusted caller claim. An independently
computed structural prediction is not an execution trace or a permission.
"""
import hashlib
import json
import struct
from tools.pending_common_boundary_review import valid, FALSE_FIELDS

PROFILE='toponav_pending_location_evidence_v1'
ENVELOPE_LIMIT=36892
INSPECTION_LIMIT=ENVELOPE_LIMIT+1
OBSERVATION_ERROR_MAX_CHARS=4096
NAMES=('controls','message','reserve','seal','reply')
EXTRA_FALSE=('observation_input_binding_authenticated','old_validation_site_observed',
    'unique_field_localization_observed','old_validator_executed_by_diagnostic')
canonical=lambda x:json.dumps(x,sort_keys=True,separators=(',',':'),ensure_ascii=True,allow_nan=False)
sha=lambda raw:hashlib.sha256(raw).hexdigest()


def _site(reason,field_index,offset):
    return dict(reason=reason,field_index=field_index,byte_offset=offset,
        basis='readonly_byte_scan_contract_prediction_not_execution')


def _scan(raw):
    facts=dict(byte_count=len(raw),within_envelope_total_cap=len(raw)<=ENVELOPE_LIMIT,
        magic_matches=raw[:4]==b'TPT1',fields=[],trailing_byte_count=None,
        scan_disposition=None,source='supplied_bytes_not_runtime_trace')
    if len(raw)>ENVELOPE_LIMIT:
        facts['scan_disposition']='not_scanned_total_oversize'
        return facts,_site('domain_byte_bound',None,0)
    if raw[:4]!=b'TPT1':
        facts['scan_disposition']='not_scanned_magic'
        return facts,_site('domain_magic',None,0)
    pos=4;first=None
    for i,name in enumerate(NAMES):
        start=pos
        if pos+4>len(raw):
            facts['scan_disposition']='stopped_at_incomplete_header'
            return facts,first or _site('domain_length',i,start)
        n=struct.unpack('>I',raw[pos:pos+4])[0];pos+=4
        null=n==0xffffffff;cap=4096 if i==0 else 8193
        field=dict(field_index=i,field_name=name,length_header_offset=start,payload_offset=pos,
            declared_length=n,is_null=null,field_cap=cap,required_null_violation=null and i<2,
            within_declared_field_cap=None if null else n<=cap,
            observed_payload_bytes=0,observed_payload_sha256=None,payload_complete=None if null else n<=len(raw)-pos)
        if null:
            if i<2 and first is None:first=_site('domain_required_field',i,start)
        else:
            count=min(n,len(raw)-pos);field['observed_payload_bytes']=count
            field['observed_payload_sha256']=sha(raw[pos:pos+count])
            if (n>cap or not field['payload_complete']) and first is None:first=_site('domain_field_bound',i,start)
            pos+=count
        facts['fields'].append(field)
        if not null and not field['payload_complete']:
            facts['scan_disposition']='stopped_at_incomplete_payload'
            return facts,first
    facts['scan_disposition']='all_five_storage_fields_observed'
    facts['trailing_byte_count']=len(raw)-pos
    if pos!=len(raw) and first is None:first=_site('domain_trailing',None,pos)
    return facts,first


def _bounded_snapshot(observation):
    # Cap container work before calling the sealed JSON-shape checker. Error
    # labels are never truncated; oversized caller snapshots are out of profile.
    if type(observation) is not dict or len(observation)!=20:return False
    error=observation.get('error')
    if type(error) is str and len(error)>OBSERVATION_ERROR_MAX_CHARS:return False
    result=observation.get('result')
    if result is not None:
        if type(result) is not dict or len(result)!=2:return False
        if any(r is not None and (type(r) is not dict or len(r)!=15) for r in result.values()):return False
    return True


def inspect_evidence(raw,observation):
    """Return bounded byte facts plus full, cloned UNTRUSTED old review snapshot.

    Accepted evidence is not an accepted message. Controls/packets are not JSON
    decoded, no old reviewer is called, and unknown sites always remain null.
    """
    out=dict(profile=PROFILE,classification='offline_byte_evidence_not_validation_or_admission',
        evidence_stage='evidence_refused',error=None,input_sha256=None,observation_sha256=None,
        observation_snapshot=None,byte_facts=None,predicted_first_structural_failure=None,
        reported_reason_agrees_with_prediction=None,observed_validation_site=None,
        prediction_scope='envelope_structure_only_controls_and_inner_packets_not_inspected',
        actual_exposure_wall_s=None,**{k:False for k in (*FALSE_FIELDS,*EXTRA_FALSE)})
    if type(raw) is not bytes:out['error']='evidence_bytes';return out
    if len(raw)>INSPECTION_LIMIT:out['error']='evidence_byte_bound';return out
    if not _bounded_snapshot(observation) or not valid(observation):out['error']='evidence_observation_shape';return out
    # Shape validity authenticates nothing and does not bind observation to raw.
    encoded=canonical(observation).encode();snapshot=json.loads(encoded)
    facts,prediction=_scan(raw)
    agrees=None if prediction is None else snapshot['stage']=='domain_refused' and snapshot['error']==prediction['reason']
    out.update(evidence_stage='evidence_returned',input_sha256=sha(raw),observation_sha256=sha(encoded),
        observation_snapshot=snapshot,byte_facts=facts,predicted_first_structural_failure=prediction,
        reported_reason_agrees_with_prediction=agrees)
    return out
