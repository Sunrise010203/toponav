"""No real time providers, native transport, sleeps or motion in these tests."""
from dataclasses import FrozenInstanceError, asdict, replace
from fractions import Fraction
import json
from pathlib import Path

import pytest

from tools.time_provider_contract import (ContractError, ContractReview, MAX_INT,
    MAX_RATE_COMPONENT, map_envelope, mapping_digest, review)
from tools.time_provider_contract_cases import CASES, evaluate, fixture, repin


@pytest.mark.parametrize('name',CASES)
def test_synthetic_contract_scenarios(name):
    case=evaluate(name)
    assert case['synthetic_only']
    for result in case['results']:
        assert not result['motion_authority'] and not result['provenance_authenticated']
        assert not result['shadow_admission_authorized'] and not result['native_live_observation_authorized']


def test_receipt_only_retains_unknown_everywhere_and_does_not_derive_age():
    spec,bundle,expected,now=fixture(receipt=True)
    result=review(spec,bundle,expected,now)
    assert result.schema_consistent and not result.all_declared_bounds_present
    assert result.oldest_declared_lower_ns is None
    assert all(lo is hi is None for _,lo,hi in result.declared_exposure_envelopes_ns)
    wire=json.loads(json.dumps(asdict(result),allow_nan=False))
    assert wire['declared_exposure_envelopes_ns'][0][1:] == [None,None]


def test_all_six_required_views_include_oldest_auxiliary_not_only_four_composite():
    spec,bundle,expected,now=fixture(six=True)
    result=review(spec,bundle,expected,now)
    assert len(result.declared_exposure_envelopes_ns)==6
    assert result.oldest_declared_lower_ns==result.declared_exposure_envelopes_ns[-1][1]
    assert result.oldest_declared_lower_ns<result.declared_exposure_envelopes_ns[0][1]


@pytest.mark.parametrize('delta',[-9,-1,0,1,9])
@pytest.mark.parametrize('rates',[(1,3,2,3),(3,2,5,2),(1,1,1,1)])
def test_interval_encloses_reference_and_rate_corners_including_before_anchor(delta,rates):
    _,bundle,_,_=fixture();m=bundle.views[0].mapping
    m=replace(m,rate_lower_num=rates[0],rate_lower_den=rates[1],
        rate_upper_num=rates[2],rate_upper_den=rates[3])
    start=m.anchor_tick+delta;end=start+1
    lo,hi=map_envelope(m,start,end)
    for tick in (start,end):
        for ref in (m.reference_lower_ns,m.reference_upper_ns):
            for rate in (Fraction(rates[0],rates[1]),Fraction(rates[2],rates[3])):
                actual=ref+(tick-m.anchor_tick)*rate
                assert lo<=actual<=hi


def test_fractional_lower_rounds_down_upper_rounds_up_not_nearest():
    case=evaluate('fractional_round_outward')['results'][0]
    assert case['declared_exposure_envelopes_ns'][0]==('front',1_000_000_000,1_000_000_002)


@pytest.mark.parametrize('value',[True,False,-1,MAX_INT+1,float('nan'),float('inf'),1.0,None,'100'])
def test_now_requires_bounded_integer_ns_no_bool_float_or_nan(value):
    spec,bundle,expected,_=fixture()
    with pytest.raises(ContractError,match='now_host_range'):review(spec,bundle,expected,value)


@pytest.mark.parametrize('field,value,reason',[
    ('rate_lower_den',0,'mapping_rate_range'),
    ('rate_upper_num',-1,'mapping_rate_range'),
    ('rate_lower_num',True,'mapping_rate_range'),
    ('rate_upper_den',MAX_RATE_COMPONENT+1,'mapping_rate_range'),
    ('rate_lower_num',1002,'rate_interval'),
    ('anchor_tick',2001,'device_validity_interval'),
    ('reference_upper_ns',999_999_999,'reference_interval'),
    ('valid_host_until_ns',1_000_000_100,'host_validity_interval'),
    ('valid_device_upper_tick',False,'mapping_integer_range'),
    ('device_boot_id',None,'mapping_identity'),
    ('calibration_evidence_sha256','unknown','calibration_evidence_digest'),
])
def test_invalid_mapping_fields_are_not_normalized_or_given_grace(field,value,reason):
    _,bundle,_,_=fixture();m=replace(bundle.views[0].mapping,**{field:value})
    with pytest.raises(ContractError,match=reason):map_envelope(m,1100,1101)


@pytest.mark.parametrize('start,end',[(1101,1100),(-1,1),(1999,2001),(True,1),(1.5,2),(1,MAX_INT+1)])
def test_exposure_tick_window_and_counter_wrap_are_not_repaired(start,end):
    _,bundle,_,_=fixture()
    with pytest.raises(ContractError):map_envelope(bundle.views[0].mapping,start,end)


@pytest.mark.parametrize('field,value',[
    ('provider_id','other-provider'),('sensor_id','other-sensor'),
    ('device_clock_id','other-clock'),('device_boot_id','other-device-boot'),
    ('host_clock_id','CLOCK_REALTIME'),('host_boot_id','other-host-boot'),
])
def test_mapping_identity_must_match_owner_even_when_changed_map_is_repinned(field,value):
    spec,bundle,expected,now=fixture()
    spec,bundle=repin(spec,bundle,replace(bundle.views[0].mapping,**{field:value}))
    with pytest.raises(ContractError,match='mapping_clock_sensor_binding'):review(spec,bundle,expected,now)


@pytest.mark.parametrize('change',['duplicate_view','extra_view','missing_expected','duplicate_expected',
    'wrong_frame','wrong_sim','bool_sim','future_callback','wrong_sensor','unknown_source','missing_pin',
    'short_camera_set','duplicate_sensor','partial_pin'])
def test_frozen_camera_and_input_bindings_fail_on_mismatch(change):
    spec,bundle,expected,now=fixture();v=bundle.views[0]
    if change=='duplicate_view':bundle=replace(bundle,views=(v,)+bundle.views[:-1])
    elif change=='extra_view':bundle=replace(bundle,views=bundle.views+(replace(v,view='up'),))
    elif change=='missing_expected':expected=replace(expected,view_digests=expected.view_digests[:-1])
    elif change=='duplicate_expected':expected=replace(expected,view_digests=(expected.view_digests[0],)+expected.view_digests[:-1])
    elif change=='wrong_frame':bundle=replace(bundle,frame_id='other-frame')
    elif change=='wrong_sim':bundle=replace(bundle,captured_sim_ns=123)
    elif change=='bool_sim':bundle=replace(bundle,captured_sim_ns=True)
    elif change=='future_callback':bundle=replace(bundle,views=(replace(v,callback_host_ns=now+1),)+bundle.views[1:])
    elif change=='wrong_sensor':bundle=replace(bundle,views=(replace(v,sensor_id='other-sensor'),)+bundle.views[1:])
    elif change=='unknown_source':bundle=replace(bundle,views=(replace(v,source_kind='replay_is_exposure'),)+bundle.views[1:])
    elif change=='missing_pin':spec=replace(spec,cameras=(replace(spec.cameras[0],mapping_sha256=None,device_clock_id=None,device_boot_id=None),)+spec.cameras[1:])
    elif change=='short_camera_set':spec=replace(spec,cameras=spec.cameras[:-1])
    elif change=='duplicate_sensor':spec=replace(spec,cameras=(replace(spec.cameras[0],sensor_id=spec.cameras[1].sensor_id),)+spec.cameras[1:])
    elif change=='partial_pin':spec=replace(spec,cameras=(replace(spec.cameras[0],device_boot_id=None),)+spec.cameras[1:])
    with pytest.raises(ContractError):review(spec,bundle,expected,now)


def test_review_does_not_mutate_inputs_and_accepts_reordering_only_with_complete_set():
    spec,bundle,expected,now=fixture(six=True)
    original=(asdict(spec),asdict(bundle),asdict(expected))
    a=review(spec,bundle,expected,now)
    b=review(spec,replace(bundle,views=tuple(reversed(bundle.views))),
        replace(expected,view_digests=tuple(reversed(expected.view_digests))),now)
    assert a==b and original==(asdict(spec),asdict(bundle),asdict(expected))


def test_missing_sim_identity_stays_none_not_converted_to_host_time():
    spec,bundle,expected,now=fixture()
    result=review(spec,replace(bundle,captured_sim_ns=None),replace(expected,captured_sim_ns=None),now)
    assert result.schema_consistent


def test_unknown_one_view_does_not_report_oldest_of_only_available_subset():
    r=evaluate('mixed_missing_exposure')['results'][0]
    assert not r['all_declared_bounds_present'] and r['oldest_declared_lower_ns'] is None


def test_expiry_boundary_is_exclusive_no_slack():
    spec,bundle,expected,_=fixture();until=bundle.views[0].mapping.valid_host_until_ns
    assert review(spec,bundle,expected,until-1).schema_consistent
    with pytest.raises(ContractError,match='mapping_expired_or_not_yet_valid'):review(spec,bundle,expected,until)


def test_map_pin_hashes_whole_content_not_id_and_never_authenticates_evidence():
    _,bundle,_,_=fixture();m=bundle.views[0].mapping
    assert mapping_digest(m)==mapping_digest(replace(m))
    assert mapping_digest(m)!=mapping_digest(replace(m,reference_upper_ns=m.reference_upper_ns+1))
    assert mapping_digest(m)!=mapping_digest(replace(m,calibration_evidence_sha256='d'*64))


def test_schema_consistency_is_deliberately_not_replay_or_forgery_protection():
    for name in ('same_bundle_repeated','mutually_forged_digest'):
        case=evaluate(name)
        for r in case['results']:
            assert r['schema_consistent'] and r['all_declared_bounds_present']
            assert 'stateless_no_replay_protection' in r['warnings']
            assert not r['provenance_authenticated'] and not r['shadow_admission_authorized']


def test_authority_flags_are_not_constructor_options_and_results_are_frozen():
    spec,bundle,expected,now=fixture();r=review(spec,bundle,expected,now)
    with pytest.raises(FrozenInstanceError):r.motion_authority=True
    with pytest.raises(TypeError):ContractReview(True,(),False,None,(),motion_authority=True)


def test_contract_is_standard_library_only_and_not_imported_by_production():
    import ast
    root=Path(__file__).resolve().parents[1]
    tree=ast.parse((root/'tools/time_provider_contract.py').read_text())
    imports={n.module.split('.')[0] for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)}
    imports.update(a.name.split('.')[0] for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names)
    assert imports<= {'dataclasses','fractions','hashlib','json'}
    for folder in ('roadvision','simulation','ros2_ws/src/toponav_bridge/toponav_bridge'):
        for p in (root/folder).glob('*.py'):
            assert 'time_provider_contract' not in p.read_text()
