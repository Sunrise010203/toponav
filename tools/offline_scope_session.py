"""Owned OFFLINE fixture lifecycle, NOT a provider, watchdog or motion gate.

Only constructs GazeboIO.__new__ via the existing isolated fixture. Joins owned
callbacks BEFORE restoring wrappers; a stuck callback can block shutdown forever.
Local UUID/boot records are diagnostic provenance, never sensor credentials.
"""
from contextlib import contextmanager
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import threading
import time
import uuid

import numpy as np

from tools.capture_timing_isolation import isolated_io, deliver, Echo
from tools.capture_timing_probe import CaptureTimingProbe, rgb_digest
from tools.receipt_snapshot_adapter import ScopeAnnotation, InputReference, AdapterError, adapt, snapshot_digest
from tools.time_provider_contract import CameraSpec, ExpectedBinding, ProviderSpec, identity, integer

MAX_ITEMS = 32
MAX_EVENTS = 256


class ScopeLifecycleError(RuntimeError):
    pass


def require(condition, reason):
    if not condition:
        raise ScopeLifecycleError(reason)


def error_info(error):
    if error is None:return None
    try:message=str(error)
    except BaseException:message='<message unavailable>'
    return dict(type=type(error).__name__,message=message[:512])


def write_json(path,value):
    with Path(path).open('x') as stream:
        json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')


@dataclass(frozen=True)
class PreparedFrame:
    scope_id: str
    expected: ExpectedBinding
    composite_sha256: str
    rgb: object


class OfflineScopeSession:
    """One owned IO/probe/Echo, sequential scopes only. No native IO accepted.

    Private object handles are not a sandbox: foreign/direct callbacks or runtime
    monkeypatches are unsupported outside controlled isolation tests.
    """
    def __init__(self, *, six=False, source_kind='isolated_synthetic_delivery'):
        require(type(six) is bool, 'six_must_be_bool')
        require(type(source_kind) is str and source_kind in
                ('isolated_synthetic_delivery','replay_delivery'), 'offline_source_only')
        clock=time.get_clock_info('monotonic')
        require(clock.monotonic and not clock.adjustable
                and clock.implementation=='clock_gettime(CLOCK_MONOTONIC)', 'unsupported_host_clock')
        boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        require(identity(boot), 'host_boot_unavailable')
        self._io=isolated_io(('up','down') if six else ())
        self._echo=Echo();self._probe=CaptureTimingProbe(source_kind)
        self._views=tuple(self._io.cameras.views)
        self._spec=ProviderSpec('offline-scope-config','owned-offline-fixture',uuid.uuid4().hex,
            'CLOCK_MONOTONIC',boot,tuple(CameraSpec(v,'logical-camera-'+v) for v in self._views))
        self._clock=dict(implementation=clock.implementation,resolution_s=clock.resolution,
                         monotonic=clock.monotonic,adjustable=clock.adjustable)
        self._lock=threading.Lock();self._active=None;self._tainted=False
        self.last_close_error=None

    def _open_events(self,path):
        return path.open('x')

    def _deliver_packet(self,packet,views):
        deliver(self._io,packet.rgb,packet.expected.captured_sim_ns,views=views)

    @contextmanager
    def scope(self,output):
        with self._lock:
            require(self._active is None, 'scope_already_active')
            require(not self._tainted and self._io.error is None, 'failed_owner_requires_explicit_new_owner')
            run=ScopeRun(self,output);self._active=run
        primary=None;failure=None
        try:
            run._start()
            yield run
        except BaseException as exc:
            primary=exc
            if run._state=='entering' and run._created:
                self._tainted=True  # Partial startup evidence must not be reused.
            raise
        finally:
            try:failure=run._finish(primary)
            except BaseException as exc:
                failure=exc;self._tainted=True
            finally:
                with self._lock:self._active=None
            self.last_close_error=failure
            if primary is None and failure is not None:raise failure


class ScopeRun:
    def __init__(self,owner,output):
        self.owner=owner;self.output=Path(output);self.scope_id=uuid.uuid4().hex
        self._thread_id=threading.get_ident();self._state='entering'
        self._created=False;self._begin_written=False;self._attached=False
        self._events=None;self._event_count=0;self._event_errors=[]
        self._packets=[];self._inputs=[];self._attempts=[];self._completed=[]
        self._workers=[];self._worker_errors=[];self._worker_lock=threading.Lock()
        self._draining=threading.Event();self._attachment=None;self._originals=[]
        self.status=None;self.scope_annotation=None;self.snapshot=None;self.receipts=[]

    def _main_active(self):
        require(threading.get_ident()==self._thread_id, 'scope_control_thread_only')
        require(self._state=='active', 'scope_not_active')
        require(not self._event_errors, 'event_log_failed')

    def _event(self,kind,**fields):
        try:
            require(self._event_count<MAX_EVENTS, 'event_capacity')
            self._events.write(json.dumps(dict(kind=kind,observed_s=time.monotonic(),**fields),allow_nan=False)+'\n')
            self._events.flush();self._event_count+=1
        except BaseException as exc:
            self._event_errors.append(error_info(exc));raise

    def _start(self):
        self.output.mkdir(exist_ok=False);self._created=True
        owner=self.owner
        before=tuple(owner._probe.snapshot()['counts'].items())
        started=time.monotonic()
        self.begin=dict(scope_id=self.scope_id,spec=asdict(owner._spec),source_kind=owner._probe.source_kind,
            annotation_kind='independently_recorded_fixture',started_s=started,counts_before=before,
            host_clock=owner._clock,actual_exposure_wall_s=None,provenance_authenticated=False,motion_authority=False)
        write_json(self.output/'begin.json',self.begin);self._begin_written=True
        self._events=owner._open_events(self.output/'events.jsonl')
        self._event('before_attach',scope_id=self.scope_id)
        for obj,names in ((owner._io,('condition','image','pixels','next_frame')),
                          (owner._io.cameras,('put','latest')),(owner._echo,('update',))):
            for name in names:self._originals.append((obj,name,name in vars(obj),vars(obj).get(name)))
        self._attachment=owner._probe.attach(owner._io,owner._echo)
        self._attachment.__enter__();self._attached=True;self._state='active'
        self._event('attached')

    def prepare(self,frame_id,rgb,captured_sim_ns):
        self._main_active()
        require(identity(frame_id) and integer(captured_sim_ns), 'frame_identity')
        require(len(self._packets)<MAX_ITEMS and all(p.expected.frame_id!=frame_id for p in self._packets), 'input_capacity_or_duplicate')
        rgb_digest(rgb)  # Original fixed dtype/shapes; composite must be 256-square.
        require(rgb.shape==(256,256,3), 'composite_shape')
        copied=rgb.copy();copied.setflags(write=False)
        tiles=(copied[:128,:128],copied[:128,128:],copied[128:,:128],copied[128:,128:])
        pixels=dict(zip(('front','left','right','back'),tiles));pixels.update(up=copied,down=copied)
        expected=ExpectedBinding(frame_id,captured_sim_ns,tuple((v,rgb_digest(pixels[v])) for v in self.owner._views))
        packet=PreparedFrame(self.scope_id,expected,rgb_digest(copied),copied)
        entry=dict(scope_id=self.scope_id,registered_s=time.monotonic(),expected=asdict(expected),composite_sha256=packet.composite_sha256)
        self._packets.append(packet);self._inputs.append(entry)
        self._event('input_registered',input=entry)
        return packet

    def _packet(self,packet):
        require(type(packet) is PreparedFrame and packet.scope_id==self.scope_id
                and any(p is packet for p in self._packets), 'packet_outside_scope')

    def deliver(self,packet,*,views=None,background=False):
        self._main_active();self._packet(packet)
        require(type(background) is bool, 'background_must_be_bool')
        if views is not None:
            require(type(views) in (tuple,list) and all(type(v) is str for v in views)
                    and set(views)<=set(self.owner._views) and len(set(views))==len(views), 'delivery_view_subset')
            views=tuple(views)
        self._event('delivery_requested',frame_id=packet.expected.frame_id,background=background,views=views)
        if not background:
            self.owner._deliver_packet(packet,views);return
        require(len(self._workers)<MAX_ITEMS, 'worker_capacity')
        def work():
            try:self.owner._deliver_packet(packet,views)
            except BaseException as exc:
                with self._worker_lock:self._worker_errors.append(exc)
        thread=threading.Thread(target=work,name='offline-scope-'+self.scope_id[:12])
        self._workers.append(thread);thread.start()

    def _join(self):
        for thread in self._workers:
            if thread.ident is not None:thread.join()  # NO bounded-shutdown claim.

    def wait(self):
        self._main_active();self._join()
        if self._worker_errors:raise self._worker_errors[0]

    def compute(self,packet):
        self._main_active();self._packet(packet);self.wait()
        require(len(self._attempts)<MAX_ITEMS, 'computation_capacity')
        counts=self.owner._probe.snapshot()['counts']
        ref=InputReference(self.scope_id,packet.expected,packet.composite_sha256,
                           counts['selections']+1,counts['computations']+1)
        attempt=dict(reference=asdict(ref),outcome='requested',error=None);self._attempts.append(attempt)
        self._event('compute_requested',reference=asdict(ref))
        try:
            io=self.owner._io
            with io.condition:io.now_ns=packet.expected.captured_sim_ns
            # Fixture is pre-delivered: zero WAIT only, not a production budget edit.
            stamp,rgb,now=io.next_frame(packet.expected.captured_sim_ns-1,timeout=0)
            frame=dict(frame_id=packet.expected.frame_id,captured_sim_s=stamp/1e9,rgb=rgb)
            for view in ('up','down'):
                if view in self.owner._views:frame[view+'_rgb']=io.cameras.views[view][stamp]
            result=self.owner._echo.update(frame,now)
        except BaseException as exc:
            attempt.update(outcome='raised',error=error_info(exc));raise
        attempt['outcome']='returned';self._completed.append(ref)
        self._event('compute_returned',frame_id=packet.expected.frame_id)
        return result

    def _finish(self,primary):
        require(threading.get_ident()==self._thread_id, 'scope_control_thread_only')
        errors=[]
        def attempt(fn):
            try:return fn()
            except BaseException as exc:errors.append(exc);return None
        self._state='draining';self._draining.set()
        attempt(self._join)
        alive=[t.name for t in self._workers if t.is_alive()]
        if self._attached and not alive:
            attempt(lambda:self._attachment.__exit__(None,None,None))
        restored=(not self.owner._probe.attached and all((name in vars(obj))==present and
            (not present or vars(obj).get(name) is value) for obj,name,present,value in self._originals))
        if not restored:errors.append(ScopeLifecycleError('wrappers_not_restored'))
        closed=time.monotonic();self._state='detached' if restored else 'cleanup_failed'
        if self._events is not None:
            attempt(lambda:self._event('drained_and_detached',workers_alive=alive,wrappers_restored=restored))
            attempt(self._events.close)
        events_closed=self._events is None or bool(getattr(self._events,'closed',False))
        if not events_closed:errors.append(ScopeLifecycleError('event_stream_not_closed'))
        if self._created:
            files={}
            def save(name,value):
                def action():
                    write_json(self.output/name,value)
                    files[name]=hashlib.sha256((self.output/name).read_bytes()).hexdigest()
                attempt(action)
            self.snapshot=attempt(self.owner._probe.snapshot)
            if self._begin_written and self.snapshot is not None:
                self.scope_annotation=attempt(lambda:ScopeAnnotation(self.scope_id,self.owner._spec.config_id,
                    self.owner._spec.provider_id,self.owner._spec.session_id,self.owner._spec.host_clock_id,
                    self.owner._spec.host_boot_id,self.owner._probe.source_kind,'independently_recorded_fixture',
                    self.begin['started_s'],closed,tuple(self.begin['counts_before']),snapshot_digest(self.snapshot)))
            if self.scope_annotation is not None and restored and not alive:
                for ref in self._completed:
                    try:value=adapt(self.owner._spec,self.scope_annotation,ref,self.snapshot)
                    except AdapterError as exc:self.receipts.append(dict(adapted=False,error=str(exc),reference=asdict(ref),motion_authority=False))
                    else:self.receipts.append(dict(adapted=True,value=asdict(value),reference=asdict(ref)))
            if self.snapshot is not None:save('snapshot.json',self.snapshot)
            if self.scope_annotation is not None:save('scope.json',asdict(self.scope_annotation))
            save('inputs.json',dict(inputs=self._inputs,attempts=self._attempts))
            save('receipts.json',self.receipts)
            for name in ('begin.json','events.jsonl'):
                if (self.output/name).is_file():
                    value=attempt(lambda name=name:hashlib.sha256((self.output/name).read_bytes()).hexdigest())
                    if value is not None:files[name]=value
            rejected=sum(not r['adapted'] for r in self.receipts)
            failed_attempts=sum(a['outcome']!='returned' for a in self._attempts)
            status=('body_failed' if primary is not None else 'worker_failed' if self._worker_errors else
                    'cleanup_failed' if errors or self._event_errors else
                    'operation_failed' if failed_attempts or self.owner._io.error is not None else
                    'diagnostic_rejected' if rejected else 'receipt_only_recorded' if self.receipts else
                    'observation_only' if self._inputs else 'empty')
            self.status=dict(status=status,scope_id=self.scope_id,session_id=self.owner._spec.session_id,
                body_error=error_info(primary),worker_errors=[error_info(e) for e in self._worker_errors],
                cleanup_errors=[error_info(e) for e in errors],event_errors=self._event_errors,
                workers_launched=len(self._workers),workers_alive=alive,all_owned_workers_joined=not alive,
                probe_attached_once=self._attached,wrappers_restored=restored,events_closed=events_closed,
                completed_computations=len(self._completed),failed_attempts=failed_attempts,
                fixture_error=self.owner._io.error,receipt_adapted=len(self.receipts)-rejected,receipt_rejected=rejected,
                artifact_files=files,artifact_files_complete=(not errors and not self._event_errors
                    and self._begin_written and events_closed and len(files)==6),
                actual_exposure_wall_s=None,provenance_authenticated=False,shadow_admission_authorized=False,
                native_live_observation_authorized=False,motion_authority=False)
            attempt(lambda:write_json(self.output/'status.json',self.status))
        self._state='closed'
        if errors or self._event_errors or alive or not restored or not events_closed:self.owner._tainted=True
        return self._worker_errors[0] if self._worker_errors else errors[0] if errors else None
