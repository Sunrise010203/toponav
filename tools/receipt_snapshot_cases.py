"""Offline saved-trace replay and isolated real-method fixtures, no native IO."""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image

from tools.capture_timing_probe import CaptureTimingProbe, rgb_digest
from tools.capture_timing_isolation import isolated_io, deliver, Echo
from tools.receipt_snapshot_adapter import (ScopeAnnotation, InputReference, AdapterError,
    KINDS, snapshot_digest, adapt)
from tools.time_provider_contract import ProviderSpec, CameraSpec, ExpectedBinding

CASES=('four_ok','six_ok','wrong_pixel','wrong_composite','wrong_auxiliary','wrong_frame','wrong_sim',
    'wrong_session','wrong_boot','wrong_scope','old_scope','unobserved_cache','missing_view',
    'history_dropped','observer_error','exposure_claim','live_source','callback_missing',
    'callback_ambiguous','phase_regression','missing_sync','snapshot_tamper',
    'repeated_snapshot_consistent','mutually_relabelled_context')
PUBLIC_CASES=(
    ('baseline_known_center','reports/straight-followup-20261101-01/round2/after-known-center'),
    ('new_center_gradient_transfer','reports/straight-followup-20261101-03/comparison/new_center-gradient_transfer'),
    ('new_offset_plus_wide_reference','reports/straight-followup-20261101-03/comparison/new_offset_plus-wide_reference'))


def inputs(rgb, names):
    tiles=(rgb[:128,:128],rgb[:128,128:],rgb[128:,:128],rgb[128:,128:])
    values=dict(zip(('front','left','right','back'),tiles))
    values.update(up=rgb,down=rgb)
    return tuple((name,rgb_digest(values[name])) for name in names)


def receipt_spec(names, label, boot):
    return ProviderSpec('offline-receipt-config','offline-receipt-provider',label,
        'CLOCK_MONOTONIC',boot,tuple(CameraSpec(v,'logical-camera-'+v) for v in names))


def fixture(six=False):
    io=isolated_io(('up','down') if six else ());io.now_ns=100_000_000
    rgb=np.random.default_rng(514).integers(0,256,(256,256,3),dtype=np.uint8)
    names=tuple(io.cameras.views);echo=Echo();probe=CaptureTimingProbe('isolated_synthetic_delivery')
    before=tuple(probe.snapshot()['counts'].items());started=time.monotonic()
    with probe.attach(io,echo):
        deliver(io,rgb,io.now_ns);stamp,selected,now=io.next_frame(-1)
        assert np.array_equal(selected,rgb)
        frame=dict(frame_id='isolated-frame',captured_sim_s=stamp/1e9,rgb=selected)
        if six:frame.update(up_rgb=rgb,down_rgb=rgb)
        assert echo.update(frame,now) is frame
    closed=time.monotonic();snapshot=probe.snapshot()
    spec=receipt_spec(names,'isolated-session','fixture-boot-declaration')
    scope=ScopeAnnotation('isolated-scope',spec.config_id,spec.provider_id,spec.session_id,
        spec.host_clock_id,spec.host_boot_id,probe.source_kind,'independently_recorded_fixture',
        started,closed,before,snapshot_digest(snapshot))
    expected=ExpectedBinding(frame['frame_id'],stamp,inputs(rgb,names))
    ref=InputReference(scope.scope_id,expected,rgb_digest(rgb),1,1)
    return spec,scope,ref,snapshot


def scenario(name):
    if name not in CASES:raise ValueError('Unknown receipt scenario')
    spec,scope,ref,snapshot=fixture(name in ('six_ok','wrong_auxiliary'))
    expected=ref.expected;error=None;calls=1
    if name=='wrong_pixel':
        ref=replace(ref,expected=replace(expected,view_digests=(('front','0'*64),)+expected.view_digests[1:]));error='selected_view_binding'
    elif name=='wrong_composite':ref=replace(ref,composite_sha256='0'*64);error='computation_input'
    elif name=='wrong_auxiliary':
        ref=replace(ref,expected=replace(expected,view_digests=expected.view_digests[:-1]+(('down','0'*64),)));error='auxiliary_input'
    elif name=='wrong_frame':ref=replace(ref,expected=replace(expected,frame_id='other-frame'));error='computation_input'
    elif name=='wrong_sim':ref=replace(ref,expected=replace(expected,captured_sim_ns=123));error='computation_input'
    elif name=='wrong_session':scope=replace(scope,session_id='other-session');error='scope_session_id'
    elif name=='wrong_boot':scope=replace(scope,host_boot_id='other-boot');error='scope_host_boot_id'
    elif name=='wrong_scope':ref=replace(ref,scope_id='other-scope');error='scope_identity'
    elif name=='old_scope':scope=replace(scope,counts_before=tuple(snapshot['counts'].items()));error='record_outside_scope'
    elif name=='unobserved_cache':snapshot['histories']['computations'][0]['selection_binding_verified']=False;error='computation_binding'
    elif name=='missing_view':snapshot['histories']['selections'][0]['views'].pop();error='selected_view_set'
    elif name=='history_dropped':snapshot['dropped']['callbacks']=1;error='history_incomplete'
    elif name=='observer_error':snapshot['observer_errors']=1;error='observer_errors'
    elif name=='exposure_claim':snapshot['actual_exposure_wall_s']=1.;error='unexpected_exposure_claim'
    elif name=='live_source':
        snapshot['source_kind']='live_callback_delivery';scope=replace(scope,source_kind='live_callback_delivery');error='offline_source_only'
    elif name=='callback_missing':
        rows=snapshot['histories']['callbacks'];rows.pop(0)
        for i,r in enumerate(rows,1):r['record_id']=i
        snapshot['counts']['callbacks']-=1;error='callback_missing_or_ambiguous'
    elif name=='callback_ambiguous':
        rows=snapshot['histories']['callbacks'];rows.append(dict(rows[0],record_id=len(rows)+1))
        snapshot['counts']['callbacks']+=1;error='callback_missing_or_ambiguous'
    elif name=='phase_regression':
        c=snapshot['histories']['computations'][0];c['compute_finished_s']=c['compute_started_s']-.01;error='phase_or_scope_times'
    elif name=='missing_sync':snapshot['histories']['sync'].clear();snapshot['counts']['sync']=0;error='synchronized_set_missing'
    elif name=='snapshot_tamper':snapshot['scope']='tampered after pin';error='snapshot_pin'
    elif name=='repeated_snapshot_consistent':calls=2
    elif name=='mutually_relabelled_context':
        spec=replace(spec,session_id='relabelled-session')
        scope=replace(scope,session_id=spec.session_id,scope_id='relabelled-scope')
        ref=replace(ref,scope_id=scope.scope_id)
    if name!='snapshot_tamper':scope=replace(scope,snapshot_sha256=snapshot_digest(snapshot))
    return spec,scope,ref,snapshot,error,calls


def evaluate(name):
    spec,scope,ref,snapshot,error,calls=scenario(name);results=[]
    for _ in range(calls):
        try:result=adapt(spec,scope,ref,snapshot)
        except AdapterError as exc:
            assert str(exc)==error,(name,str(exc),error)
            results.append(dict(adapted=False,error=str(exc),motion_authority=False))
        else:
            assert error is None,name
            assert not result.contract_review.all_declared_bounds_present
            results.append(dict(adapted=True,value=asdict(result)))
    return dict(case=name,spec=asdict(spec),scope=asdict(scope),reference=asdict(ref),
        snapshot=snapshot,expected_error=error,results=results)


def public_batches(root):
    root=Path(root);old=root/'reports/straight-followup-20261102-12'
    boot=json.loads((old/'ENVIRONMENT.json').read_text())['boot_id']
    for repeat in (0,1):
        for name,relative in PUBLIC_CASES:
            source=old/f'measurements/{repeat}-{name}-instrumented.json'
            saved=json.loads(source.read_text());snapshot=saved['trace'];names=tuple(snapshot['required_views'])
            # Retrospective ENCLOSURE, not recovered original attachment/session.
            started=min(r['callback_entry_s'] for r in snapshot['histories']['callbacks'])
            closed=max(r['compute_finished_s'] for r in snapshot['histories']['computations'])
            label=f'offline-task2C-{repeat}-{name}'
            spec=receipt_spec(names,label,boot)
            scope=ScopeAnnotation(label,spec.config_id,spec.provider_id,spec.session_id,spec.host_clock_id,
                spec.host_boot_id,'replay_delivery','retrospective_offline',started,closed,
                tuple((k,0) for k in KINDS),snapshot_digest(snapshot))
            base=root/relative
            rows=[json.loads(line) for line in (base/'decisions.jsonl').read_text().splitlines()][:12]
            references=[];files={source,base/'decisions.jsonl',old/'ENVIRONMENT.json'}
            for index,row in enumerate(rows,1):
                original=row['input'];path=(base/'public'/original['rgb_path']).resolve()
                assert path.is_relative_to((base/'public').resolve())
                with Image.open(path) as im:rgb=np.array(im)
                assert rgb_digest(rgb)==original['rgb_sha256']
                expected=ExpectedBinding(original['frame_id'],round(original['captured_sim_s']*1e9),inputs(rgb,names))
                references.append(InputReference(label,expected,rgb_digest(rgb),index,index));files.add(path)
            assert len(references)==12
            yield name,repeat,spec,scope,snapshot,references,files


def run(root,output):
    root=Path(root);output=Path(output);output.mkdir(exist_ok=False)
    def write(name,value):
        with (output/name).open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')
    synth=[]
    for name in CASES:
        value=evaluate(name);write(name+'.json',value)
        synth.append(dict(case=name,calls=len(value['results']),adapted=value['results'][0]['adapted']))
    public=[]
    for name,repeat,spec,scope,snapshot,references,files in public_batches(root):
        records=[asdict(adapt(spec,scope,ref,snapshot)) for ref in references]
        write(f'public-{repeat}-{name}.json',dict(case=name,repeat=repeat,scope=asdict(scope),spec=asdict(spec),
            references=[asdict(r) for r in references],adaptations=records,
            read_files=sorted(str(p.relative_to(root)) for p in files)))
        public.append(dict(case=name,repeat=repeat,adapted=len(records)))
    summary=dict(synthetic=synth,synthetic_scenarios=len(synth),synthetic_calls=sum(r['calls'] for r in synth),
        synthetic_adapted_scenarios=sum(r['adapted'] for r in synth),public=public,
        public_adaptations=sum(r['adapted'] for r in public),actual_exposure_wall_s=None,
        production_changes=False,native_transport=False,movement=False,
        note='saved replay observations only; retrospective context is not an attested original session. No controller/performance/native capture/stop claim.')
    write('summary.json',summary);return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();print(json.dumps(run(Path(__file__).resolve().parents[1],args.output)))
