"""Receipt-only diagnostic adaptation, never native capture or authority."""
import copy
from dataclasses import asdict, FrozenInstanceError, replace
from fractions import Fraction
import json
from pathlib import Path

import pytest

from tools.receipt_snapshot_adapter import AdapterError, adapt, receipt_ns, snapshot_digest
from tools.receipt_snapshot_cases import CASES, evaluate, fixture, public_batches


@pytest.mark.parametrize('name',CASES)
def test_isolated_real_method_snapshot_cases(name):
    value=evaluate(name)
    for r in value['results']:
        if r['adapted']:
            result=r['value'];review=result['contract_review']
            assert not review['all_declared_bounds_present'] and review['oldest_declared_lower_ns'] is None
            for obj in (result,review):
                assert not any(obj[k] for k in ('provenance_authenticated','shadow_admission_authorized',
                    'native_live_observation_authorized','motion_authority'))
            assert all(v['source_kind']=='receipt_only' and v['mapping'] is None
                and v['exposure_start_tick'] is v['exposure_end_tick'] is v['exposure_scope'] is None
                for v in result['bundle']['views'])
        else:assert not r['motion_authority']


def test_quiescent_inputs_not_mutated_json_snapshot_supported_and_result_frozen():
    spec,scope,ref,snapshot=fixture(True);original=copy.deepcopy(snapshot)
    a=adapt(spec,scope,ref,snapshot)
    b=adapt(spec,scope,ref,json.loads(json.dumps(snapshot,allow_nan=False)))
    assert a==b and snapshot==original
    assert len(a.callback_record_ids)==6
    with pytest.raises(FrozenInstanceError):a.motion_authority=True


def test_independent_public_rgb_binds_all_72_saved_computations_without_controller_calls():
    root=Path(__file__).resolve().parents[1];total=0
    for _,_,spec,scope,snapshot,refs,_ in public_batches(root):
        assert scope.annotation_kind=='retrospective_offline'
        for ref in refs:
            result=adapt(spec,scope,ref,snapshot)
            assert result.bundle.frame_id==ref.expected.frame_id
            assert result.contract_review.oldest_declared_lower_ns is None
            assert result.source_kind=='replay_delivery' and not result.provenance_authenticated
            total+=1
    assert total==72


@pytest.mark.parametrize('value',[0,1,1.0,1/3,1e-10,10000.123456789])
def test_receipt_ns_floors_exact_sample_not_float_multiplication_or_exposure(value):
    result=receipt_ns(value);exact=Fraction(*value.as_integer_ratio())*10**9
    assert result<=exact<result+1


@pytest.mark.parametrize('value',[True,False,-1,-.1,float('nan'),float('inf'),1e100,None,'1'])
def test_invalid_receipt_samples_not_filled_from_sim_or_current_clock(value):
    with pytest.raises(AdapterError):receipt_ns(value)


@pytest.mark.parametrize('field,value',[
    ('capacity',64.0),('capacity',128),('counts_are_lower_bounds',True),
    ('clock_mapping_verified',True),('observer_errors',True),('actual_exposure_wall_s',1),
])
def test_snapshot_flags_are_strict_and_never_downgraded(field,value):
    spec,scope,ref,s=fixture();s[field]=value;scope=replace(scope,snapshot_sha256=snapshot_digest(s))
    with pytest.raises(AdapterError):adapt(spec,scope,ref,s)


@pytest.mark.parametrize('change',['missing_callback_metadata','bad_callback_outcome','exposure_in_callback',
    'missing_scope_counter','duplicate_scope_counter','scope_starts_late','scope_closes_early',
    'bad_record_ids','unknown_views','missing_sync_time','cached_after_selection','unexpected_auxiliary',
    'device_mapping_config','input_subset','boolean_record_id'])
def test_incomplete_ambiguous_or_cross_scope_metadata_cannot_be_reconstructed(change):
    spec,scope,ref,s=fixture();cb=s['histories']['callbacks'][0];sel=s['histories']['selections'][0]
    if change=='missing_callback_metadata':cb.pop('callback_entry_s')
    elif change=='bad_callback_outcome':cb['outcome']='raised'
    elif change=='exposure_in_callback':cb['exposure_clock_mapping']='invented'
    elif change=='missing_scope_counter':scope=replace(scope,counts_before=scope.counts_before[:-1])
    elif change=='duplicate_scope_counter':scope=replace(scope,counts_before=(scope.counts_before[0],)*4)
    elif change=='scope_starts_late':scope=replace(scope,started_s=s['histories']['computations'][0]['compute_started_s'])
    elif change=='scope_closes_early':scope=replace(scope,closed_s=scope.started_s)
    elif change=='bad_record_ids':cb['record_id']=2
    elif change=='unknown_views':s['required_views']=list(s['required_views'])+['unknown']
    elif change=='missing_sync_time':s['histories']['sync'][0].pop('synchronized_set_observed_s')
    elif change=='cached_after_selection':sel['views'][0]['cached_observed_s']=sel['selected_observed_s']+1
    elif change=='unexpected_auxiliary':s['histories']['computations'][0]['auxiliary_sha256']={'down':'f'*64}
    elif change=='device_mapping_config':
        spec=replace(spec,cameras=(replace(spec.cameras[0],device_clock_id='device',device_boot_id='boot',mapping_sha256='f'*64),)+spec.cameras[1:])
    elif change=='input_subset':ref=replace(ref,expected=replace(ref.expected,view_digests=ref.expected.view_digests[:-1]))
    elif change=='boolean_record_id':ref=replace(ref,selection_id=True)
    scope=replace(scope,snapshot_sha256=snapshot_digest(s))
    with pytest.raises(AdapterError):adapt(spec,scope,ref,s)


@pytest.mark.parametrize('value',[{'a':'x'*257},{'a':object()},{'a':float('nan')},{'a':-1},
    {'a':[0]*65},{'a':[[[[[[[[[0]]]]]]]]]},{1:'non-string-key'}])
def test_snapshot_canonicalization_is_bounded_and_refuses_unknown_objects(value):
    with pytest.raises(AdapterError):snapshot_digest(value)


def test_repeated_and_mutually_relabelled_context_are_explicit_non_authentication_limits():
    repeated=evaluate('repeated_snapshot_consistent')['results']
    assert len(repeated)==2 and repeated[0]==repeated[1] and repeated[0]['adapted']
    relabelled=evaluate('mutually_relabelled_context')['results'][0]
    assert relabelled['adapted'] and not relabelled['value']['provenance_authenticated']


def test_retrospective_annotation_cannot_be_mistaken_for_independent_fixture_boundary():
    spec,scope,ref,snapshot=fixture()
    result=adapt(spec,replace(scope,annotation_kind='retrospective_offline'),ref,snapshot)
    assert result.annotation_kind=='retrospective_offline' and not result.provenance_authenticated


def test_adapter_is_pure_and_not_imported_by_production():
    import ast
    root=Path(__file__).resolve().parents[1]
    tree=ast.parse((root/'tools/receipt_snapshot_adapter.py').read_text())
    modules={n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)}
    modules.update(a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names)
    assert modules <= {'dataclasses','hashlib','json','math','tools.time_provider_contract'}
    for folder in ('roadvision','simulation','ros2_ws/src/toponav_bridge/toponav_bridge'):
        for p in (root/folder).glob('*.py'):assert 'receipt_snapshot_' not in p.read_text()
