"""Opt-in READ-ONLY closed fixture evidence review. Never a live/safety gate.

No IO/controller/probe construction, callbacks, workers, current clock or motion.
Only stdlib file reads and the existing pure-data receipt adapter are used.
Quiescent regular files are required. Local hashes are NOT authentication.
"""
import argparse
from contextlib import contextmanager
from dataclasses import asdict, dataclass, field
import hashlib
import json
import math
import os
from pathlib import Path
import stat

from tools.receipt_snapshot_adapter import AdapterError, InputReference, ScopeAnnotation, adapt, snapshot_digest
from tools.time_provider_contract import CameraSpec, ExpectedBinding, ProviderSpec, digest, identity

FILES=('begin.json','events.jsonl','snapshot.json','scope.json','inputs.json','receipts.json','status.json')
KINDS=('callbacks','sync','selections','computations')
STATUSES=('body_failed','worker_failed','cleanup_failed','operation_failed','diagnostic_rejected',
          'receipt_only_recorded','observation_only','empty')
AUTH=('provenance_authenticated','shadow_admission_authorized','native_live_observation_authorized','motion_authority')
MAX_FILE_BYTES=2*1024*1024
MAX_TOTAL_BYTES=8*1024*1024


class BundleError(ValueError):
    pass


def check(condition,code):
    if not condition:raise BundleError(code)


def shape(obj,keys,code):
    check(type(obj) is dict and set(obj)==set(keys.split()),code)


def uint(value,maximum=2**63-1):
    return type(value) is int and 0<=value<=maximum


def sample(value):
    return type(value) in (float,int) and math.isfinite(value) and 0<=value<=1e12


def same(a,b):
    # JSON equality with strict bool/int and int/float spellings, not Python True==1.
    return json.dumps(a,sort_keys=True,separators=(',',':'),allow_nan=False)==json.dumps(b,sort_keys=True,separators=(',',':'),allow_nan=False)


def _pairs(pairs):
    result={}
    for key,value in pairs:
        check(key not in result,'duplicate_json_key');result[key]=value
    return result


def _constant(_):raise BundleError('nonfinite_json_number')


def _integer(value):
    check(len(value)<=21,'json_integer_limit');return int(value)


def parse(data):
    """Bound JSON before schema processing; duplicate keys and NaN are rejected."""
    check(type(data) is bytes and len(data)<=MAX_FILE_BYTES,'json_byte_limit')
    try:value=json.loads(data.decode('utf-8'),object_pairs_hook=_pairs,parse_constant=_constant,parse_int=_integer)
    except (ValueError,UnicodeError,RecursionError) as exc:
        if isinstance(exc,BundleError):raise
        raise BundleError('invalid_json') from exc
    nodes=0
    def visit(obj,depth=0):
        nonlocal nodes
        nodes+=1;check(depth<=24 and nodes<=60000,'json_structure_limit')
        if type(obj) is dict:
            check(len(obj)<=128,'json_object_limit')
            for key,item in obj.items():check(len(key)<=256,'json_key_limit');visit(item,depth+1)
        elif type(obj) is list:
            check(len(obj)<=1024,'json_array_limit')
            for item in obj:visit(item,depth+1)
        elif type(obj) is str:check(len(obj)<=2048,'json_string_limit')
        elif type(obj) is float:check(math.isfinite(obj),'nonfinite_json_number')
        else:check(obj is None or type(obj) in (bool,int),'json_type')
    visit(value);return value


@contextmanager
def _directory(path):
    """Walk every path component with NOFOLLOW. Refuse '..', including ancestors."""
    path=Path(path).absolute();check('..' not in path.parts,'parent_path_refused')
    fd=None
    try:
        fd=os.open('/',os.O_RDONLY|os.O_DIRECTORY|os.O_CLOEXEC)
        for component in path.parts[1:]:
            new=os.open(component,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=fd)
            os.close(fd);fd=new
        yield fd
    finally:
        if fd is not None:os.close(fd)


def _signature(info):
    return info.st_dev,info.st_ino,info.st_size,info.st_mtime_ns,info.st_ctime_ns


def _read(fd,name):
    # NONBLOCK avoids hanging on a FIFO; fstat rejects all non-regular inputs.
    with os.fdopen(os.open(name,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK|os.O_CLOEXEC,dir_fd=fd),'rb') as stream:
        before=os.fstat(stream.fileno())
        check(stat.S_ISREG(before.st_mode),'nonregular_file:'+name)
        check(before.st_size<=MAX_FILE_BYTES,'file_size_limit:'+name)
        data=stream.read(MAX_FILE_BYTES+1);after=os.fstat(stream.fileno())
        check(len(data)<=MAX_FILE_BYTES,'file_size_limit:'+name)
        check(_signature(before)==_signature(after) and len(data)==after.st_size,'file_changed:'+name)
        return data,_signature(after)


def read_files(path):
    """Fixed seven names only; ignores extras and never follows manifest paths."""
    try:
        with _directory(path) as fd:
            first={name:_read(fd,name) for name in FILES}
            check(sum(len(item[0]) for item in first.values())<=MAX_TOTAL_BYTES,'bundle_size_limit')
            for name,(data,signature) in first.items():
                repeated,sig=_read(fd,name)
                check(data==repeated and signature==sig,'bundle_changed:'+name)
            return {name:item[0] for name,item in first.items()}
    except OSError as exc:raise BundleError('unreadable_or_unsafe_bundle') from exc


def read_pins(path):
    """Read a caller-selected pin file, never paths supplied inside a scope."""
    path=Path(path)
    try:
        with _directory(path.parent) as fd:data,_=_read(fd,path.name)
    except OSError as exc:raise BundleError('unreadable_or_unsafe_pins') from exc
    return parse(data)


@dataclass(frozen=True)
class ScopeReview:
    classification: str
    evidence_consistent: bool
    complete_evidence: bool
    recorded_status: str | None
    trusted_bytes_match: bool | None
    external_inputs_match: bool | None
    diagnostics: tuple[str,...]
    file_sha256: tuple[tuple[str,str],...]
    adapted_receipts: int = 0
    rejected_receipts: int = 0
    actual_exposure_wall_s: None = field(default=None,init=False)
    provenance_authenticated: bool = field(default=False,init=False)
    shadow_admission_authorized: bool = field(default=False,init=False)
    native_live_observation_authorized: bool = field(default=False,init=False)
    motion_authority: bool = field(default=False,init=False)


def _error(obj):
    if obj is None:return
    shape(obj,'type message','error_schema')
    check(identity(obj['type']) and type(obj['message']) is str and len(obj['message'])<=512,'error_schema')


def _expected(obj,names):
    shape(obj,'frame_id captured_sim_ns view_digests','input_expected_schema')
    check(identity(obj['frame_id']) and uint(obj['captured_sim_ns']),'input_identity')
    views=obj['view_digests']
    check(type(views) is list and len(views)==len(names) and all(type(v) is list and len(v)==2
          and type(v[0]) is str and digest(v[1]) for v in views),'input_view_digests')
    check([v[0] for v in views]==list(names),'input_view_digests')
    return ExpectedBinding(obj['frame_id'],obj['captured_sim_ns'],tuple(tuple(v) for v in views))


def _reference(obj,scope,names,known):
    shape(obj,'scope_id expected composite_sha256 selection_id computation_id','reference_schema')
    exp=_expected(obj['expected'],names)
    check(obj['scope_id']==scope.scope_id and exp.frame_id in known,'reference_scope_or_frame')
    source=known[exp.frame_id]
    check(same(obj['expected'],source['expected']) and obj['composite_sha256']==source['composite_sha256'],'reference_input_binding')
    check(uint(obj['selection_id']) and obj['selection_id']>0 and uint(obj['computation_id']) and obj['computation_id']>0,'reference_ids')
    return InputReference(scope.scope_id,exp,obj['composite_sha256'],obj['selection_id'],obj['computation_id'])


def _validate(doc,events,external_inputs):
    begin,scope_data,snapshot,inputs,receipts,status=(doc[n] for n in ('begin.json','scope.json','snapshot.json','inputs.json','receipts.json','status.json'))
    shape(begin,'scope_id spec source_kind annotation_kind started_s counts_before host_clock actual_exposure_wall_s provenance_authenticated motion_authority','begin_schema')
    shape(begin['spec'],'config_id provider_id session_id host_clock_id host_boot_id cameras','spec_schema')
    spec_data=begin['spec'];cameras=spec_data['cameras']
    check(all(identity(spec_data[k]) for k in ('config_id','provider_id','session_id','host_clock_id','host_boot_id')),'spec_identity')
    check(type(cameras) is list and 4<=len(cameras)<=6,'camera_schema')
    for camera in cameras:
        shape(camera,'view sensor_id device_clock_id device_boot_id mapping_sha256','camera_schema')
        check(identity(camera['sensor_id']) and all(camera[k] is None for k in ('device_clock_id','device_boot_id','mapping_sha256')),'receipt_only_camera')
    names=tuple(c['view'] for c in cameras)
    check(names[:4]==('front','left','right','back') and len(set(names))==len(names) and set(names[4:])<={'up','down'},'camera_set')
    spec=ProviderSpec(**{**spec_data,'cameras':tuple(CameraSpec(**c) for c in cameras)})
    shape(begin['host_clock'],'implementation resolution_s monotonic adjustable','host_clock_schema')
    clock=begin['host_clock']
    check(spec.host_clock_id=='CLOCK_MONOTONIC' and clock['implementation']=='clock_gettime(CLOCK_MONOTONIC)'
          and clock['monotonic'] is True and clock['adjustable'] is False and sample(clock['resolution_s']) and clock['resolution_s']>0,'host_clock_metadata')
    shape(scope_data,'scope_id config_id provider_id session_id host_clock_id host_boot_id source_kind annotation_kind started_s closed_s counts_before snapshot_sha256','scope_schema')
    baseline=scope_data['counts_before']
    check(type(baseline) is list and len(baseline)==4 and all(type(p) is list and len(p)==2 and type(p[0]) is str and uint(p[1]) for p in baseline),'counter_baseline')
    check([p[0] for p in baseline]==list(KINDS),'counter_baseline')
    scope=ScopeAnnotation(**{**scope_data,'counts_before':tuple(tuple(p) for p in baseline)})
    check(identity(scope.scope_id) and begin['scope_id']==scope.scope_id,'scope_identity')
    for key in ('config_id','provider_id','session_id','host_clock_id','host_boot_id'):
        check(getattr(spec,key)==getattr(scope,key),'scope_spec_binding')
    check(scope.source_kind in ('isolated_synthetic_delivery','replay_delivery') and begin['source_kind']==scope.source_kind,'offline_source_only')
    check(scope.annotation_kind==begin['annotation_kind']=='independently_recorded_fixture','original_annotation_required')
    check(sample(scope.started_s) and sample(scope.closed_s) and scope.started_s<=scope.closed_s
          and same(begin['started_s'],scope.started_s) and same(begin['counts_before'],baseline),'scope_boundaries')
    check(begin['actual_exposure_wall_s'] is None and begin['provenance_authenticated'] is False and begin['motion_authority'] is False,'begin_authority_or_exposure')
    shape(status,'status scope_id session_id body_error worker_errors cleanup_errors event_errors workers_launched workers_alive all_owned_workers_joined probe_attached_once wrappers_restored events_closed completed_computations failed_attempts fixture_error receipt_adapted receipt_rejected artifact_files artifact_files_complete actual_exposure_wall_s provenance_authenticated shadow_admission_authorized native_live_observation_authorized motion_authority','status_schema')
    check(status['status'] in STATUSES and status['scope_id']==scope.scope_id and status['session_id']==spec.session_id,'status_identity')
    check(all(status[k] is False for k in AUTH) and status['actual_exposure_wall_s'] is None,'status_authority_or_exposure')
    check(all(status[k] is True for k in ('all_owned_workers_joined','probe_attached_once','wrappers_restored','events_closed')) and status['workers_alive']==[],'recorded_resources_not_closed')
    check(uint(status['workers_launched'],32),'worker_count')
    _error(status['body_error'])
    for key in ('worker_errors','cleanup_errors','event_errors'):
        check(type(status[key]) is list and len(status[key])<=256,'error_list')
        for error in status[key]:check(error is not None,'null_error');_error(error)
    check(status['fixture_error'] is None or type(status['fixture_error']) is str,'fixture_error')
    check(type(status['artifact_files_complete']) is bool,'complete_flag_type')
    complete=status['artifact_files_complete'];log_fault=bool(status['event_errors'])
    check(complete==not_errors(status),'complete_flag_inconsistent')
    check(digest(scope.snapshot_sha256) and snapshot_digest(snapshot)==scope.snapshot_sha256,'snapshot_pin')
    check(snapshot.get('source_kind')==scope.source_kind and snapshot.get('required_views')==list(names),'snapshot_source_or_views')
    check(snapshot.get('actual_exposure_wall_s','missing') is None and snapshot.get('clock_mapping_verified') is False,'snapshot_exposure')
    check(type(snapshot.get('observer_errors')) is int and snapshot['observer_errors']>=0 and type(snapshot.get('counts_are_lower_bounds')) is bool,'snapshot_flags')
    check(type(snapshot.get('capacity')) is int and snapshot['capacity']==64,'snapshot_capacity')
    for key in ('counts','histories','dropped'):shape(snapshot.get(key),' '.join(KINDS),'snapshot_history_schema')
    for kind in KINDS:
        count=snapshot['counts'][kind];rows=snapshot['histories'][kind];dropped=snapshot['dropped'][kind]
        check(uint(count) and type(rows) is list and len(rows)==min(count,64) and uint(dropped)
              and dropped==count-len(rows) and count>=dict(baseline)[kind],'snapshot_counts')
        check(all(type(row) is dict and type(row.get('record_id')) is int and row['record_id']==i
              for i,row in enumerate(rows,count-len(rows)+1)),'history_record_ids')
    shape(inputs,'inputs attempts','inputs_schema')
    check(type(inputs['inputs']) is list and len(inputs['inputs'])<=32 and type(inputs['attempts']) is list and len(inputs['attempts'])<=32,'input_or_attempt_capacity')
    known={}
    for item in inputs['inputs']:
        shape(item,'scope_id registered_s expected composite_sha256','input_schema');exp=_expected(item['expected'],names)
        check(item['scope_id']==scope.scope_id and exp.frame_id not in known,'duplicate_or_foreign_input')
        check(sample(item['registered_s']) and scope.started_s<=item['registered_s']<=scope.closed_s and digest(item['composite_sha256']),'input_registration')
        known[exp.frame_id]=item
    external_match=None
    if external_inputs is not None:
        check(type(external_inputs) is list and len(external_inputs)==len(known),'external_input_set')
        external={}
        for obj in external_inputs:
            shape(obj,'expected composite_sha256','external_input_schema');exp=_expected(obj['expected'],names)
            check(digest(obj['composite_sha256']),'external_input_digest')
            check(exp.frame_id not in external,'duplicate_external_input');external[exp.frame_id]=obj
        check(set(external)==set(known),'external_input_set')
        for fid,item in known.items():
            check(same(external[fid]['expected'],item['expected']) and external[fid]['composite_sha256']==item['composite_sha256'],'external_pixel_or_identity_mismatch')
        external_match=True
    refs=[];returned=[];returned_ids=set()
    for index,attempt in enumerate(inputs['attempts']):
        shape(attempt,'reference outcome error','attempt_schema');ref=_reference(attempt['reference'],scope,names,known)
        check(attempt['outcome'] in ('requested','returned','raised'),'attempt_outcome');_error(attempt['error'])
        check((attempt['error'] is not None)==(attempt['outcome']=='raised'),'attempt_error_binding')
        check(dict(baseline)['selections']<ref.selection_id<=snapshot['counts']['selections']+1
              and dict(baseline)['computations']<ref.computation_id<=snapshot['counts']['computations']+1,'attempt_counter_bounds')
        refs.append(ref)
        if attempt['outcome']=='returned':
            pair=(ref.selection_id,ref.computation_id);check(pair not in returned_ids,'duplicate_returned_reference')
            returned_ids.add(pair);returned.append(index)
    check(type(receipts) is list and len(receipts)==len(returned),'receipt_set')
    adapted=rejected=0
    for saved,index in zip(receipts,returned):
        ref=refs[index]
        try:result=asdict(adapt(spec,scope,ref,snapshot))
        except AdapterError as exc:
            shape(saved,'adapted error reference motion_authority','rejected_receipt_schema')
            check(saved['adapted'] is False and saved['motion_authority'] is False and saved['error']==str(exc),'rejected_receipt_mismatch');rejected+=1
        else:
            shape(saved,'adapted value reference','receipt_schema')
            check(saved['adapted'] is True and same(saved['value'],result),'saved_receipt_mismatch');adapted+=1
        check(same(saved['reference'],inputs['attempts'][index]['reference']),'receipt_reference_mismatch')
    for key,value in (('completed_computations',len(returned)),('failed_attempts',len(refs)-len(returned)),('receipt_adapted',adapted),('receipt_rejected',rejected)):
        check(type(status[key]) is int and status[key]==value,'status_accounting')
    expected_status=('body_failed' if status['body_error'] is not None else 'worker_failed' if status['worker_errors'] else
        'cleanup_failed' if status['cleanup_errors'] or log_fault else 'operation_failed' if status['failed_attempts'] or status['fixture_error'] is not None else
        'diagnostic_rejected' if rejected else 'receipt_only_recorded' if receipts else 'observation_only' if known else 'empty')
    check(status['status']==expected_status,'recorded_status_mismatch')
    _events(events,scope,inputs,refs,returned,names,snapshot,log_fault,status['fixture_error'])
    return complete,external_match,adapted,rejected


def not_errors(status):
    return not status['cleanup_errors'] and not status['event_errors']


def _events(events,scope,inputs,refs,returned,names,snapshot,log_fault,fixture_error):
    check(2<=len(events)<=256,'event_capacity_or_missing_attach')
    registered=[];requests=[];returns=[];delivery=[];last_time=scope.started_s;drained=False
    for event in events:
        check(type(event) is dict and type(event.get('kind')) is str and sample(event.get('observed_s')),'event_schema')
        kind=event['kind'];now=event['observed_s']
        check(now>=last_time and not drained,'event_order');last_time=now
        keys={'before_attach':'scope_id','attached':'','input_registered':'input','delivery_requested':'frame_id background views',
              'compute_requested':'reference','compute_returned':'frame_id','drained_and_detached':'workers_alive wrappers_restored'}
        check(kind in keys,'unknown_event');shape(event,'kind observed_s '+keys[kind],'event_schema')
        if kind=='before_attach':check(not registered and event is events[0] and event['scope_id']==scope.scope_id,'attach_event_order')
        elif kind=='attached':check(event is events[1],'attach_event_order')
        elif kind=='input_registered':
            i=len(registered);check(i<len(inputs['inputs']) and same(event['input'],inputs['inputs'][i]),'input_event_binding')
            check(event['input']['registered_s']<=now<=scope.closed_s,'input_event_time');registered.append(event['input']['expected']['frame_id'])
        elif kind=='delivery_requested':
            check(event['frame_id'] in registered and type(event['background']) is bool,'delivery_event_binding')
            views=event['views'];check(views is None or type(views) is list and all(type(v) is str for v in views)
                and len(views)==len(set(views)) and set(views)<=set(names),'delivery_event_views')
            delivery.append(event)
        elif kind=='compute_requested':
            i=len(requests);check(i<len(refs) and same(event['reference'],inputs['attempts'][i]['reference']),'compute_event_binding')
            check(event['reference']['expected']['frame_id'] in registered,'compute_before_registration');requests.append(event)
        elif kind=='compute_returned':
            i=len(requests)-1
            check(i in returned and i not in returns and event['frame_id']==refs[i].expected.frame_id,'compute_return_event')
            comp=next((c for c in snapshot['histories']['computations'] if c['record_id']==refs[i].computation_id),None)
            selected=next((s for s in snapshot['histories']['selections'] if s['record_id']==refs[i].selection_id),None)
            check(comp is not None and selected is not None and requests[i]['observed_s']<=selected['next_frame_started_s']
                  <=selected['next_frame_finished_s']<=comp['compute_started_s']<=comp['compute_finished_s']<=now<=scope.closed_s,'compute_event_time')
            returns.append(i)
        elif kind=='drained_and_detached':
            check(event is events[-1] and event['workers_alive']==[] and event['wrappers_restored'] is True and now>=scope.closed_s,'detach_event');drained=True
        if kind!='drained_and_detached':check(now<=scope.closed_s,'event_after_close')
    check(events[0]['kind']=='before_attach' and events[1]['kind']=='attached','attach_event_order')
    if not log_fault:
        check(drained and len(registered)==len(inputs['inputs']) and len(requests)==len(refs) and returns==returned,'missing_lifecycle_event')
    baseline=dict(scope.counts_before)
    for cb in snapshot['histories']['callbacks']:
        check(cb.get('kind')=='callback' and cb.get('source_kind')==scope.source_kind and cb.get('actual_exposure_wall_s','missing') is None
              and cb.get('exposure_clock_mapping','missing') is None,'callback_metadata')
        check(sample(cb.get('callback_entry_s')) and sample(cb.get('callback_finished_s'))
              and cb['callback_entry_s']<=cb['callback_finished_s']<=scope.closed_s,'callback_boundary')
        if cb['record_id']<=baseline['callbacks']:continue
        check(cb['callback_entry_s']>=scope.started_s,'callback_outside_scope')
        if 'captured_sim_ns' not in cb:
            check(fixture_error is not None and cb.get('io_error_present') is True,'unbound_callback');continue
        matching=[item for item in inputs['inputs'] if item['expected']['captured_sim_ns']==cb['captured_sim_ns']
                  and dict(item['expected']['view_digests']).get(cb.get('view'))==cb.get('rgb_sha256')]
        check(any(item['registered_s']<=cb['callback_entry_s'] and any(d['frame_id']==item['expected']['frame_id']
              and (d['views'] is None or cb['view'] in d['views']) and d['observed_s']<=cb['callback_entry_s'] for d in delivery)
              for item in matching),'callback_before_registration_or_delivery')


def review_scope(path,*,trusted_files=None,external_inputs=None):
    """Return diagnostics; never repair or write. Pins must cover all seven files.

    evidence_consistent means complete local diagnostic consistency, NOT success,
    authenticated provenance, anti-replay, exposure freshness or motion authority.
    Recorded incomplete log-fault bundles retain their original failure status.
    """
    hashes=();recorded=None;pinned=None
    try:
        raw=read_files(path);hashes=tuple((n,hashlib.sha256(raw[n]).hexdigest()) for n in FILES)
        doc={name:parse(data) for name,data in raw.items() if name!='events.jsonl'}
        status=doc['status.json']
        if type(status) is dict and type(status.get('status')) is str and status['status'] in STATUSES:recorded=status['status']
        if trusted_files is not None:
            check(type(trusted_files) is dict and set(trusted_files)==set(FILES)
                  and all(digest(v) for v in trusted_files.values()),'trusted_file_set')
            pinned=all(trusted_files[n]==h for n,h in hashes);check(pinned,'trusted_hash_mismatch')
        artifact=status.get('artifact_files') if type(status) is dict else None
        check(type(artifact) is dict and set(artifact)==set(FILES)-{'status.json'} and all(digest(v) for v in artifact.values()),'artifact_file_set')
        check(all(artifact[n]==dict(hashes)[n] for n in artifact),'artifact_hash_mismatch')
        lines=raw['events.jsonl'].splitlines()
        check(len(lines)<=256 and all(lines) and raw['events.jsonl'].endswith(b'\n'),'event_lines')
        events=[parse(line) for line in lines]
        complete,external,adapted,rejected=_validate(doc,events,external_inputs)
        diagnostics=tuple(c for c,flag in (('unpinned_local_consistency_only',pinned is None),('no_external_pixel_comparison',external is None),
            ('recorded_incomplete_failure',not complete)) if flag)
        classification=('recorded_incomplete' if not complete else 'recorded_failure' if recorded.endswith('failed') else
                        'recorded_rejection' if recorded=='diagnostic_rejected' else 'receipt_only' if recorded=='receipt_only_recorded' else 'observation_only')
        return ScopeReview(classification,complete,complete,recorded,pinned,external,diagnostics,hashes,adapted,rejected)
    except (BundleError,AdapterError,KeyError,TypeError,ValueError,OverflowError,RecursionError) as exc:
        code=str(exc) if isinstance(exc,BundleError) else 'invalid_schema_or_snapshot'
        return ScopeReview('invalid_bundle',False,False,recorded,pinned,None,(code,),hashes)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--scope',type=Path,required=True)
    parser.add_argument('--pins',type=Path);args=parser.parse_args()
    try:result=review_scope(args.scope,trusted_files=read_pins(args.pins) if args.pins else None)
    except BundleError as exc:result=ScopeReview('invalid_bundle',False,False,None,None,None,(str(exc),),())
    print(json.dumps(asdict(result),allow_nan=False));return 0 if result.evidence_consistent else 2


if __name__=='__main__':raise SystemExit(main())
