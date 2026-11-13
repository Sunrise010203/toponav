"""Offline scope lifecycle only: no devices, navigation benchmark or safety claim."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from tools import offline_scope_session as module
from tools.offline_scope_session import OfflineScopeSession, ScopeLifecycleError, MAX_ITEMS
from tools.offline_scope_cases import CASES, PUBLIC_CASES, FaultStream, capture, exercise, pixels
from tools.receipt_snapshot_adapter import snapshot_digest


def read(path):return json.loads(path.read_text())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def assert_receipt(value):
    assert value['source_kind'] in ('isolated_synthetic_delivery','replay_delivery')
    for obj in (value,value['contract_review']):
        assert not any(obj[k] for k in ('provenance_authenticated','shadow_admission_authorized',
            'native_live_observation_authorized','motion_authority'))
    assert value['contract_review']['oldest_declared_lower_ns'] is None
    for view in value['bundle']['views']:
        assert view['source_kind']=='receipt_only'
        assert view['mapping'] is view['exposure_start_tick'] is view['exposure_end_tick'] is view['exposure_scope'] is None


@pytest.mark.parametrize('name',CASES)
def test_original_method_lifecycle_scenarios_and_closed_artifacts(name,tmp_path):
    result=exercise(name,tmp_path/name)
    assert result['all_owned_workers_joined'] and result['wrappers_restored'] and result['event_streams_closed']
    for relative in result['scopes']:
        out=tmp_path/name/relative;status=read(out/'status.json')
        assert status['actual_exposure_wall_s'] is None
        assert not any(status[k] for k in ('provenance_authenticated','shadow_admission_authorized',
            'native_live_observation_authorized','motion_authority'))
        for file,h in status['artifact_files'].items():assert sha(out/file)==h
        for receipt in read(out/'receipts.json'):
            if receipt['adapted']:assert_receipt(receipt['value'])
    if name in ('compute_exception','worker_exception','log_write_failure','log_close_failure','body_and_log_failure','body_exception_while_callback'):
        assert result['original_exception_identity_preserved'] is True


def test_begin_and_input_sidecars_precede_attach_and_original_callback(tmp_path,monkeypatch):
    owner=OfflineScopeSession();out=tmp_path/'scope';original_attach=owner._probe.attach
    def attach(io,echo):
        assert read(out/'begin.json')['counts_before']==[[k,0] for k in owner._probe.snapshot()['counts']]
        assert [json.loads(x)['kind'] for x in (out/'events.jsonl').read_text().splitlines()]==['before_attach']
        return original_attach(io,echo)
    monkeypatch.setattr(owner._probe,'attach',attach)
    original_delivery=owner._deliver_packet
    def deliver(packet,views):
        events=[json.loads(x) for x in (out/'events.jsonl').read_text().splitlines()]
        assert events[-2]['kind']=='input_registered' and events[-1]['kind']=='delivery_requested'
        assert events[-2]['input']['composite_sha256']==packet.composite_sha256
        return original_delivery(packet,views)
    monkeypatch.setattr(owner,'_deliver_packet',deliver)
    with owner.scope(out) as run:capture(run)
    begin=read(out/'begin.json');scope=read(out/'scope.json');snap=read(out/'snapshot.json')
    assert begin['spec']['host_boot_id']==Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    assert begin['host_clock']['implementation']=='clock_gettime(CLOCK_MONOTONIC)'
    assert begin['host_clock']['monotonic'] and not begin['host_clock']['adjustable']
    assert begin['spec']['session_id']==scope['session_id'] and begin['scope_id']==scope['scope_id']
    assert scope['snapshot_sha256']==snapshot_digest(snap)
    registered=read(out/'inputs.json')['inputs'][0]['registered_s']
    assert scope['started_s']<=registered<=min(c['callback_entry_s'] for c in snap['histories']['callbacks'])
    assert max(c['callback_finished_s'] for c in snap['histories']['callbacks'])<=scope['closed_s']
    assert all(c['mapping_sha256'] is c['device_boot_id'] is c['device_clock_id'] is None for c in begin['spec']['cameras'])


def test_local_session_scope_and_boot_labels_are_not_authentication(tmp_path):
    a,b=OfflineScopeSession(),OfflineScopeSession()
    assert a._spec.session_id!=b._spec.session_id
    with a.scope(tmp_path/'a') as run:capture(run)
    assert_receipt(run.receipts[0]['value'])
    assert run.status['provenance_authenticated'] is False


@pytest.mark.parametrize('source',['live','native_live','receipt_only','',None,True,1])
def test_no_live_or_ambiguous_source_allowed(source):
    with pytest.raises(ScopeLifecycleError,match='offline_source_only'):OfflineScopeSession(source_kind=source)


@pytest.mark.parametrize('six',[None,0,1,'six'])
def test_camera_set_requires_explicit_bool(six):
    with pytest.raises(ScopeLifecycleError,match='six_must_be_bool'):OfflineScopeSession(six=six)


@pytest.mark.parametrize('clock',[
    SimpleNamespace(monotonic=False,adjustable=False,implementation='clock_gettime(CLOCK_MONOTONIC)'),
    SimpleNamespace(monotonic=True,adjustable=True,implementation='clock_gettime(CLOCK_MONOTONIC)'),
    SimpleNamespace(monotonic=True,adjustable=False,implementation='unknown'),
])
def test_clock_assumptions_not_silently_relabelled(clock,monkeypatch):
    monkeypatch.setattr(module.time,'get_clock_info',lambda _:clock)
    with pytest.raises(ScopeLifecycleError,match='unsupported_host_clock'):OfflineScopeSession()


@pytest.mark.parametrize('operation',['prepare','deliver','compute','wait'])
def test_foreign_control_thread_refused_without_callbacks(operation,tmp_path):
    owner=OfflineScopeSession();errors=[]
    with owner.scope(tmp_path/'scope') as run:
        packet=run.prepare('frame',pixels(),100_000_000)
        def foreign():
            try:
                if operation=='prepare':run.prepare('foreign',pixels(),200_000_000)
                elif operation=='wait':run.wait()
                else:getattr(run,operation)(packet)
            except BaseException as exc:errors.append(exc)
        thread=threading.Thread(target=foreign);thread.start();thread.join()
        assert len(errors)==1 and str(errors[0])=='scope_control_thread_only'
    assert run.snapshot['counts']['callbacks']==0


@pytest.mark.parametrize('operation',['deliver','compute'])
def test_foreign_scope_packet_refused(operation,tmp_path):
    owner=OfflineScopeSession()
    with owner.scope(tmp_path/'old') as old:packet=old.prepare('frame',pixels(),100_000_000)
    with owner.scope(tmp_path/'new') as new:
        with pytest.raises(ScopeLifecycleError,match='packet_outside_scope'):getattr(new,operation)(packet)
        with pytest.raises(ScopeLifecycleError,match='packet_outside_scope'):getattr(new,operation)(replace(packet,scope_id=new.scope_id))
    assert new.snapshot['counts']['callbacks']==0


def test_prepared_pixels_are_copied_readonly_and_pre_delivery_digested(tmp_path):
    rgb=pixels();original=rgb.copy();owner=OfflineScopeSession()
    with owner.scope(tmp_path/'scope') as run:
        packet=run.prepare('frame',rgb,100_000_000);rgb[:]=0
        assert not packet.rgb.flags.writeable
        assert packet.composite_sha256==hashlib.sha256(original.tobytes()).hexdigest()
        with pytest.raises(ValueError):packet.rgb[:]=0
        run.deliver(packet);result=run.compute(packet)
        assert (result['rgb']==original).all()
    assert run.status['receipt_adapted']==1


@pytest.mark.parametrize('frame_id,stamp',[('',1),(None,1),('f',True),('f',-1),('f',1.0),('f',2**63)])
def test_invalid_input_identity_not_backfilled(frame_id,stamp,tmp_path):
    owner=OfflineScopeSession()
    with owner.scope(tmp_path/'scope') as run:
        with pytest.raises(ScopeLifecycleError,match='frame_identity'):run.prepare(frame_id,pixels(),stamp)
    assert not run._inputs and run.status['status']=='empty'


def test_duplicate_and_capacity_refused_without_eviction(tmp_path):
    owner=OfflineScopeSession()
    with owner.scope(tmp_path/'scope') as run:
        first=run.prepare('f0',pixels(),1)
        with pytest.raises(ScopeLifecycleError,match='input_capacity_or_duplicate'):run.prepare('f0',pixels(),2)
        for index in range(1,MAX_ITEMS):run.prepare('f'+str(index),pixels(index),index+1)
        with pytest.raises(ScopeLifecycleError,match='input_capacity_or_duplicate'):run.prepare('overflow',pixels(),33)
    assert len(run._inputs)==MAX_ITEMS and run._packets[0] is first


@pytest.mark.parametrize('views',[('unknown',),('front','front'),[1],'front'])
def test_delivery_view_subset_strict(views,tmp_path):
    owner=OfflineScopeSession()
    with owner.scope(tmp_path/'scope') as run:
        packet=run.prepare('f',pixels(),1)
        with pytest.raises(ScopeLifecycleError,match='delivery_view_subset'):run.deliver(packet,views=views)
    assert run.snapshot['counts']['callbacks']==0


def test_caught_log_failure_blocks_more_work_and_taints_owner(tmp_path):
    owner=OfflineScopeSession();error=OSError('same original error')
    with pytest.raises(OSError) as caught:
        with owner.scope(tmp_path/'scope') as run:
            capture(run);run._events=FaultStream(run._events,error,'write')
            with pytest.raises(OSError):run.prepare('f1',pixels(),200_000_000)
            with pytest.raises(ScopeLifecycleError,match='event_log_failed'):run.prepare('f2',pixels(),300_000_000)
    assert caught.value is error and len(run._event_errors)==2
    with pytest.raises(ScopeLifecycleError,match='failed_owner_requires_explicit_new_owner'):
        with owner.scope(tmp_path/'again'):pass
    assert run._events.closed


def test_open_stream_failure_releases_active_owner_and_keeps_original_error(tmp_path,monkeypatch):
    owner=OfflineScopeSession();error=OSError('cannot open')
    def fail(path):raise error
    monkeypatch.setattr(owner,'_open_events',fail)
    with pytest.raises(OSError) as caught:
        with owner.scope(tmp_path/'scope'):pass
    assert caught.value is error and owner._active is None and not owner._probe.attached
    assert owner._tainted
    status=read(tmp_path/'scope/status.json')
    assert status['status']=='body_failed' and not status['artifact_files_complete'] and status['events_closed']


def test_body_error_not_masked_by_artifact_storage_failure(tmp_path,monkeypatch):
    owner=OfflineScopeSession();body=KeyboardInterrupt('original interrupt');storage=OSError('snapshot write')
    original=module.write_json
    def fail(path,value):
        if path.name=='snapshot.json':raise storage
        return original(path,value)
    monkeypatch.setattr(module,'write_json',fail)
    with pytest.raises(KeyboardInterrupt) as caught:
        with owner.scope(tmp_path/'scope') as run:capture(run);raise body
    assert caught.value is body and owner.last_close_error is storage and run._events.closed
    assert run.status['body_error']['type']=='KeyboardInterrupt' and not run.status['artifact_files_complete']
    assert not owner._probe.attached and owner._tainted


def test_no_output_write_fds_after_exit(tmp_path):
    owner=OfflineScopeSession()
    with owner.scope(tmp_path/'scope') as run:capture(run,background=True)
    assert run._events.closed and all(not t.is_alive() for t in run._workers)
    for fd in Path('/proc/self/fd').iterdir():
        try:target=fd.resolve();info=Path('/proc/self/fdinfo',fd.name).read_text()
        except OSError:continue
        if target.is_relative_to(tmp_path):
            flags=int(next(l.split()[1] for l in info.splitlines() if l.startswith('flags:')),8)
            assert flags&3==0,(target,flags)


def test_opt_in_not_imported_by_ordinary_production():
    root=Path(__file__).resolve().parents[1]
    for directory in ('simulation','roadvision'):
        for path in (root/directory).rglob('*.py'):
            assert 'offline_scope_session' not in path.read_text() and 'offline_scope_cases' not in path.read_text()


def test_event_capacity_explicit_failure_not_silent_eviction(tmp_path,monkeypatch):
    monkeypatch.setattr(module,'MAX_EVENTS',4);owner=OfflineScopeSession()
    with pytest.raises(ScopeLifecycleError,match='event_capacity'):
        with owner.scope(tmp_path/'scope') as run:
            packet=run.prepare('f',pixels(),1);run.deliver(packet,views=())
            run.deliver(packet,views=())
    events=(tmp_path/'scope/events.jsonl').read_text().splitlines()
    assert len(events)==4 and len(run._event_errors)==2
    assert run._events.closed and owner._tainted and not run.status['artifact_files_complete']


def test_worker_capacity_explicit_failure_all_workers_still_joined(tmp_path):
    owner=OfflineScopeSession()
    with owner.scope(tmp_path/'scope') as run:
        packet=run.prepare('f',pixels(),1)
        for _ in range(MAX_ITEMS):run.deliver(packet,views=(),background=True)
        with pytest.raises(ScopeLifecycleError,match='worker_capacity'):run.deliver(packet,views=(),background=True)
    assert len(run._workers)==32 and all(not t.is_alive() and not t.daemon for t in run._workers)
    assert run.snapshot['counts']['callbacks']==0 and run.status['all_owned_workers_joined']


def test_computation_capacity_retains_all_attempts_without_eviction(tmp_path):
    owner=OfflineScopeSession()
    with owner.scope(tmp_path/'scope') as run:
        packet=capture(run)
        for _ in range(MAX_ITEMS-1):run.compute(packet)
        with pytest.raises(ScopeLifecycleError,match='computation_capacity'):run.compute(packet)
    assert len(run._attempts)==32 and len(run.receipts)==32
    assert run.status['completed_computations']==32


def test_start_failure_does_not_create_additional_scope(tmp_path,monkeypatch):
    owner=OfflineScopeSession();original=module.write_json;error=OSError('begin write failure')
    def fail(path,value):
        if path.name=='begin.json':raise error
        return original(path,value)
    monkeypatch.setattr(module,'write_json',fail)
    with pytest.raises(OSError) as caught:
        with owner.scope(tmp_path/'scope'):pass
    assert caught.value is error and owner._tainted and not owner._probe.attached
    with pytest.raises(ScopeLifecycleError,match='failed_owner_requires_explicit_new_owner'):
        with owner.scope(tmp_path/'new'):pass
    assert not (tmp_path/'new').exists()
