"""Synthetic declarations only; no provider discovery, transport or movement."""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path

from tools.time_provider_contract import (BASE_VIEWS, MAX_INT, ClockMap, CameraSpec,
    ProviderSpec, ViewClaim, BundleClaim, ExpectedBinding, ContractError, mapping_digest, review)

CASES = ('receipt_only_unknown', 'four_declared_bounds', 'six_declared_bounds',
    'mixed_missing_exposure', 'missing_required_view', 'same_stamp_other_pixels',
    'other_session', 'host_reboot', 'device_reboot', 'receipt_promoted_to_exposure',
    'expired_mapping', 'mapping_relabelled', 'sim_clock_mapping', 'partial_pixel_timestamp',
    'exposure_in_future', 'same_bundle_repeated', 'mutually_forged_digest',
    'pre_anchor_rate_interval', 'fractional_round_outward', 'mapped_overflow')


def fixture(*, six=False, receipt=False):
    names = BASE_VIEWS + (('up','down') if six else ())
    cameras=[];views=[]
    for index,name in enumerate(names):
        # These are SYNTHETIC identities/calibration tags, never real credentials.
        sensor='synthetic-sensor-'+name
        m=ClockMap('synthetic-map-'+name,'c'*64,'synthetic-provider',sensor,
            'synthetic-device-clock','synthetic-device-boot','CLOCK_MONOTONIC','synthetic-host-boot',
            1000,1_000_000_000-index*10_000,1_000_000_100-index*10_000,
            999,1,1001,1,0,2000,999_000_000,1_010_000_000)
        sha=hashlib.sha256(('synthetic-pixels-'+name).encode()).hexdigest()
        cameras.append(CameraSpec(name,sensor) if receipt else CameraSpec(name,sensor,
            m.device_clock_id,m.device_boot_id,mapping_digest(m)))
        views.append(ViewClaim(name,sensor,sha,1_002_000_000) if receipt else
            ViewClaim(name,sensor,sha,1_002_000_000,'device_exposure_ticks',1100,1101,
                      'all_pixel_exposure_envelope',m))
    spec=ProviderSpec('synthetic-config','synthetic-provider','synthetic-session',
                      'CLOCK_MONOTONIC','synthetic-host-boot',tuple(cameras))
    bundle=BundleClaim(spec.config_id,spec.provider_id,spec.session_id,spec.host_clock_id,
                      spec.host_boot_id,'synthetic-frame',100_000_000,tuple(views))
    expected=ExpectedBinding(bundle.frame_id,bundle.captured_sim_ns,tuple((v.view,v.rgb_sha256) for v in views))
    return spec,bundle,expected,1_003_000_000


def repin(spec,bundle,mapping):
    camera=replace(spec.cameras[0],mapping_sha256=mapping_digest(mapping))
    spec=replace(spec,cameras=(camera,)+spec.cameras[1:])
    bundle=replace(bundle,views=(replace(bundle.views[0],mapping=mapping),)+bundle.views[1:])
    return spec,bundle


def scenario(name):
    if name not in CASES:raise ValueError('Unknown contract scenario')
    spec,bundle,expected,now=fixture(six=name=='six_declared_bounds',receipt=name=='receipt_only_unknown')
    error=None;complete=name!='receipt_only_unknown';calls=1
    v=bundle.views[0];m=v.mapping
    if name=='mixed_missing_exposure':
        bundle=replace(bundle,views=(replace(v,source_kind='receipt_only',exposure_start_tick=None,
            exposure_end_tick=None,exposure_scope=None,mapping=None),)+bundle.views[1:]);complete=False
    elif name=='missing_required_view':bundle=replace(bundle,views=bundle.views[:-1]);error='claim_view_set'
    elif name=='same_stamp_other_pixels':
        bundle=replace(bundle,views=(replace(v,rgb_sha256='0'*64),)+bundle.views[1:]);error='view_pixel_binding'
    elif name=='other_session':bundle=replace(bundle,session_id='other-session');error='bundle_session_id'
    elif name=='host_reboot':bundle=replace(bundle,host_boot_id='other-boot');error='bundle_host_boot_id'
    elif name=='device_reboot':
        spec,bundle=repin(spec,bundle,replace(m,device_boot_id='other-boot'));error='mapping_clock_sensor_binding'
    elif name=='receipt_promoted_to_exposure':
        bundle=replace(bundle,views=(replace(v,source_kind='receipt_only'),)+bundle.views[1:]);error='receipt_cannot_claim_exposure'
    elif name=='expired_mapping':now=m.valid_host_until_ns;error='mapping_expired_or_not_yet_valid'
    elif name=='mapping_relabelled':
        bundle=replace(bundle,views=(replace(v,mapping=replace(m,reference_upper_ns=m.reference_upper_ns+1)),)+bundle.views[1:]);error='unpinned_or_changed_mapping'
    elif name=='sim_clock_mapping':
        bundle=replace(bundle,views=(replace(v,mapping=replace(m,clock_kind='simulation_time')),)+bundle.views[1:]);error='not_device_exposure_clock'
    elif name=='partial_pixel_timestamp':
        bundle=replace(bundle,views=(replace(v,exposure_scope='first_row_only'),)+bundle.views[1:]);error='exposure_scope'
    elif name=='exposure_in_future':
        bundle=replace(bundle,views=(replace(v,callback_host_ns=m.reference_lower_ns),)+bundle.views[1:]);error='exposure_after_callback'
    elif name=='same_bundle_repeated':calls=2
    elif name=='mutually_forged_digest':
        # A matching lie passes consistency. No authentication or authority follows.
        bundle=replace(bundle,views=(replace(v,rgb_sha256='f'*64),)+bundle.views[1:])
        expected=replace(expected,view_digests=(('front','f'*64),)+expected.view_digests[1:])
    elif name=='pre_anchor_rate_interval':
        bundle=replace(bundle,views=(replace(v,exposure_start_tick=800,exposure_end_tick=801),)+bundle.views[1:])
    elif name=='fractional_round_outward':
        spec,bundle=repin(spec,bundle,replace(m,reference_upper_ns=m.reference_lower_ns,
            rate_lower_num=1,rate_lower_den=3,rate_upper_num=2,rate_upper_den=3))
        bundle=replace(bundle,views=(replace(bundle.views[0],exposure_start_tick=1001,exposure_end_tick=1002),)+bundle.views[1:])
    elif name=='mapped_overflow':
        spec,bundle=repin(spec,bundle,replace(m,reference_lower_ns=MAX_INT-1,
            reference_upper_ns=MAX_INT-1,valid_host_from_ns=0,valid_host_until_ns=MAX_INT))
        bundle=replace(bundle,views=(replace(bundle.views[0],callback_host_ns=MAX_INT-1),)+bundle.views[1:])
        now=MAX_INT-1;error='mapped_host_range'
    return spec,bundle,expected,now,error,complete,calls


def evaluate(name):
    spec,bundle,expected,now,wanted,complete,calls=scenario(name)
    results=[]
    for _ in range(calls):
        try:
            result=review(spec,bundle,expected,now)
        except ContractError as exc:
            assert str(exc)==wanted,(name,str(exc),wanted)
            results.append(dict(schema_consistent=False,error=str(exc),motion_authority=False,
                provenance_authenticated=False,shadow_admission_authorized=False,
                native_live_observation_authorized=False))
        else:
            assert wanted is None and result.all_declared_bounds_present==complete,name
            assert not any((result.motion_authority,result.provenance_authenticated,
                result.shadow_admission_authorized,result.native_live_observation_authorized))
            results.append(asdict(result))
    return dict(case=name,synthetic_only=True,declared_spec=asdict(spec),declared_bundle=asdict(bundle),
        expected_binding=asdict(expected),now_host_ns=now,expected_error=wanted,results=results)


def run(output):
    output=Path(output);output.mkdir(exist_ok=False)
    rows=[]
    for name in CASES:
        value=evaluate(name)
        with (output/(name+'.json')).open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')
        rows.append(dict(case=name,calls=len(value['results']),schema_consistent=value['results'][0]['schema_consistent']))
    summary=dict(cases=rows,scenario_count=len(rows),review_calls=sum(r['calls'] for r in rows),
        consistent_scenarios=sum(r['schema_consistent'] for r in rows),
        actual_exposure_wall_s=None,provenance_authenticated=False,native_live_observation_authorized=False,
        motion_authority=False,scope='synthetic standard-library contract only, no real source or replay protection')
    with (output/'summary.json').open('x') as f:json.dump(summary,f,indent=2);f.write('\n')
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    print(json.dumps(run(parser.parse_args().output)))
