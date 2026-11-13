"""Once-only offline lifecycle fixtures; all injected faults remain evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import threading

import numpy as np
from PIL import Image

from tools.capture_timing_isolation import deliver, message
from tools.offline_scope_session import OfflineScopeSession, ScopeLifecycleError, write_json

CASES=('four','six','async_compute','threaded_drain','body_exception_while_callback',
    'compute_exception','worker_exception','missing_view_timeout','caught_missing_frame',
    'preexisting_cache','reattach','history_drop','log_write_failure','log_close_failure',
    'body_and_log_failure','nested_refused','preexisting_output_refused','empty',
    'closed_run_refused','malformed_packet')
PUBLIC_CASES=(
    ('baseline_known_center','reports/straight-followup-20261101-01/round2/after-known-center'),
    ('new_center_gradient_transfer','reports/straight-followup-20261101-03/comparison/new_center-gradient_transfer'),
    ('new_offset_plus_wide_reference','reports/straight-followup-20261101-03/comparison/new_offset_plus-wide_reference'))


class FaultStream:
    """Fixture-only fault proxy; CLOSE faults close the real fd before raising."""
    def __init__(self,stream,error,mode):self.stream,self.error,self.mode=stream,error,mode
    @property
    def closed(self):return self.stream.closed
    def write(self,value):
        if self.mode=='write':raise self.error
        return self.stream.write(value)
    def flush(self):return self.stream.flush()
    def close(self):
        self.stream.close()
        if self.mode=='close':raise self.error


def pixels(index=0):
    return np.random.default_rng(915+index).integers(0,256,(256,256,3),dtype=np.uint8)


def capture(run,index=0,*,background=False):
    rgb=pixels(index);packet=run.prepare('frame-'+str(index),rgb,(index+1)*100_000_000)
    run.deliver(packet,background=background);returned=run.compute(packet)
    assert np.array_equal(returned['rgb'],rgb)
    return packet


def exercise(name,output):
    if name not in CASES:raise ValueError('Unknown lifecycle fixture')
    output=Path(output);output.mkdir(exist_ok=False)
    owner=OfflineScopeSession(six=name=='six');runs=[];extra={}
    primary=RuntimeError('injected original body/compute failure')
    worker=RuntimeError('injected original worker failure')
    log_error=OSError('injected original event log failure')
    caught=None;expected_error=None;helper=None;release=threading.Event()
    try:
        if name=='preexisting_output_refused':
            target=output/'scope';target.mkdir();(target/'preserve.txt').write_text('must remain unchanged\n')
            expected_error=FileExistsError
            with owner.scope(target):raise AssertionError('must not enter')
        elif name=='reattach':
            for index in (0,1):
                with owner.scope(output/f'scope-{index}') as run:runs.append(run);capture(run,index)
            assert runs[0].scope_id!=runs[1].scope_id
            assert runs[1].begin['counts_before']==tuple(runs[0].snapshot['counts'].items())
            extra['same_session_new_scope']=True
        else:
            if name=='preexisting_cache':deliver(owner._io,pixels(),100_000_000)
            if name=='compute_exception':
                def original_update(frame,now):raise primary
                owner._echo.update=original_update;expected_error=primary
            if name=='worker_exception':
                def broken(packet,views):raise worker
                owner._deliver_packet=broken;expected_error=worker
            with owner.scope(output/'scope') as run:
                runs.append(run)
                if name in ('threaded_drain','body_exception_while_callback'):
                    entered=threading.Event();observed=[];original=owner._deliver_packet
                    def delayed(packet,views):
                        entered.set();release.wait();original(packet,views)
                    owner._deliver_packet=delayed
                    def release_while_draining():
                        run._draining.wait();observed.append(owner._probe.attached);release.set()
                    helper=threading.Thread(target=release_while_draining,name='controlled-fixture-release');helper.start()
                    packet=run.prepare('frame-0',pixels(),100_000_000);run.deliver(packet,background=True)
                    assert entered.wait(2.)
                    extra['probe_attached_at_drain']=observed
                    if name=='body_exception_while_callback':expected_error=primary;raise primary
                elif name=='worker_exception':
                    packet=run.prepare('frame-0',pixels(),100_000_000);run.deliver(packet,background=True)
                elif name in ('missing_view_timeout','caught_missing_frame'):
                    packet=run.prepare('frame-0',pixels(),100_000_000);run.deliver(packet,views=('front','left','right'))
                    if name=='missing_view_timeout':expected_error=TimeoutError;run.compute(packet)
                    else:
                        try:run.compute(packet)
                        except TimeoutError:extra['caught_original_timeout']=True
                elif name=='preexisting_cache':
                    packet=run.prepare('frame-0',pixels(),100_000_000);run.compute(packet)
                elif name=='history_drop':
                    for index in range(17):capture(run,index)
                elif name=='empty':pass
                elif name=='malformed_packet':
                    packet=run.prepare('frame-0',pixels(),100_000_000)
                    bad=message(pixels()[:128,:128],100_000_000);bad.data=b''
                    owner._io.image('front',bad)  # Same-thread, owned original fixture only.
                    expected_error=RuntimeError;run.compute(packet)
                else:
                    capture(run,background=name=='async_compute')
                    if name in ('log_write_failure','log_close_failure','body_and_log_failure'):
                        run._events=FaultStream(run._events,log_error,'write' if name=='log_write_failure' else 'close')
                        if name=='log_write_failure':expected_error=log_error;run.prepare('frame-1',pixels(1),200_000_000)
                        elif name=='body_and_log_failure':expected_error=primary;raise primary
                        else:expected_error=log_error
                    if name=='nested_refused':
                        try:
                            with owner.scope(output/'must-not-exist'):raise AssertionError('nested entry')
                        except ScopeLifecycleError as exc:
                            assert str(exc)=='scope_already_active';extra['nested_refused_while_outer_attached']=owner._probe.attached
                    if name=='closed_run_refused':extra['test_closed_after_exit']=True
    except BaseException as exc:
        caught=exc
        if isinstance(expected_error,type) and issubclass(expected_error,BaseException):assert isinstance(exc,expected_error),(name,exc)
        else:assert exc is expected_error,(name,exc,expected_error)
    finally:
        release.set()
        if helper is not None:helper.join(2.);assert not helper.is_alive()
    assert (caught is None)==(expected_error is None),name
    if name=='closed_run_refused':
        before={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in runs[0].output.iterdir()}
        try:runs[0].prepare('after-close',pixels(),200_000_000)
        except ScopeLifecycleError as exc:assert str(exc)=='scope_not_active';extra['closed_run_refused']=True
        else:raise AssertionError('closed run accepted work')
        assert before=={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in runs[0].output.iterdir()}
    if name=='preexisting_output_refused':
        assert (output/'scope/preserve.txt').read_text()=='must remain unchanged\n'
        assert [p.name for p in (output/'scope').iterdir()]==['preserve.txt']
        extra['preexisting_output_unchanged']=True
    if name=='malformed_packet':
        try:
            with owner.scope(output/'no-reset'):raise AssertionError('error was reset')
        except ScopeLifecycleError as exc:assert str(exc)=='failed_owner_requires_explicit_new_owner'
        extra['original_io_error_not_reset']=True
    assert owner._active is None and not owner._probe.attached
    if name=='compute_exception':assert owner._echo.update is original_update
    for run in runs:
        assert run.status['wrappers_restored'] and run.status['events_closed'] and run.status['all_owned_workers_joined']
        assert not run.status['workers_alive']
        assert not run.status['motion_authority'] and run.status['actual_exposure_wall_s'] is None
    if name in ('threaded_drain','body_exception_while_callback'):assert extra['probe_attached_at_drain']==[True]
    if name=='preexisting_cache':assert runs[0].status['receipt_rejected']==1
    if name=='history_drop':
        assert runs[0].snapshot['dropped']['callbacks']==4 and runs[0].status['receipt_rejected']==17
        assert runs[0].status['receipt_adapted']==0
    if name=='caught_missing_frame':assert runs[0].status['status']=='operation_failed'
    if name in ('log_write_failure','log_close_failure','body_and_log_failure'):
        assert not runs[0].status['artifact_files_complete'] and runs[0].status['cleanup_errors']
    result=dict(case=name,caught_type=type(caught).__name__ if caught else None,
        original_exception_identity_preserved=caught is expected_error if isinstance(expected_error,BaseException) else None,
        scopes=[str(r.output.relative_to(output)) for r in runs],statuses=[r.status['status'] for r in runs],extra=extra,
        all_owned_workers_joined=True,wrappers_restored=True,event_streams_closed=True,
        native_transport=False,movement=False,production_changes=False)
    write_json(output/'case.json',result);return result


def run(root,output):
    root=Path(root);output=Path(output);output.mkdir(exist_ok=False)
    cases=[exercise(name,output/name) for name in CASES]
    public=[]
    for name,relative in PUBLIC_CASES:
        base=root/relative;rows=[json.loads(line) for line in (base/'decisions.jsonl').read_text().splitlines()][:4]
        owner=OfflineScopeSession(source_kind='replay_delivery')
        with owner.scope(output/('public-'+name)) as scope:
            for row in rows:
                ref=row['input'];path=(base/'public'/ref['rgb_path']).resolve();assert path.is_relative_to((base/'public').resolve())
                with Image.open(path) as im:rgb=np.array(im)
                assert hashlib.sha256(rgb.tobytes()).hexdigest()==ref['rgb_sha256']
                packet=scope.prepare(ref['frame_id'],rgb,round(ref['captured_sim_s']*1e9))
                scope.deliver(packet);result=scope.compute(packet);assert np.array_equal(result['rgb'],rgb)
        assert scope.status['receipt_adapted']==4 and scope.status['receipt_rejected']==0
        public.append(dict(case=name,scope='public-'+name,adapted=4))
    summary=dict(scenarios=len(cases),cases=cases,public=public,public_receipts=12,
        actual_exposure_wall_s=None,all_emitted_authority_false=True,native_transport=False,movement=False,
        production_changes=False,note='Lifecycle observations include expected body/worker/log failures, not safety passes. Fresh scopes describe offline replay delivery, never historical exposure.')
    write_json(output/'summary.json',summary);return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();print(json.dumps(run(Path(__file__).resolve().parents[1],args.output)))
