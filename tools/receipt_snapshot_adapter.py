"""Opt-in OFFLINE snapshot consistency adapter, never an exposure provider.

No clock/device/file I/O, runtime attachment, gate, token, or authority. Inputs
must be quiescent JSON-compatible snapshots and independently held annotations.
Caller-forged annotations can be mutually consistent: this is not authentication.
"""
from dataclasses import dataclass, field
import hashlib
import json
import math

from tools.time_provider_contract import (ProviderSpec, CameraSpec, ExpectedBinding,
    ViewClaim, BundleClaim, ContractReview, ContractError, review, identity, integer, digest, MAX_INT)

KINDS = ('callbacks', 'sync', 'selections', 'computations')
OFFLINE_SOURCES = ('replay_delivery', 'isolated_synthetic_delivery')
PHASES = ('callback_entry_s','lock_acquired_s','decode_started_s','decoded_s','cached_observed_s')


class AdapterError(ValueError):
    pass


def check(condition, reason):
    if not condition:
        raise AdapterError(reason)


def sample(value):
    return type(value) in (int,float) and 0 <= value <= MAX_INT/1e9 and math.isfinite(value)


def receipt_ns(value):
    """Floor the EXACT binary sample to ns, NOT recovered clock precision."""
    check(sample(value), 'invalid_time_sample')
    num, den = value.as_integer_ratio()
    result = num*1_000_000_000//den
    check(integer(result), 'receipt_ns_range')
    return result


def snapshot_digest(snapshot):
    """Bounded canonical JSON digest; no unrestricted object serialization."""
    remaining = 30000
    def visit(value, depth):
        nonlocal remaining
        remaining -= 1
        check(remaining >= 0 and depth <= 8, 'snapshot_size')
        if value is None or type(value) is bool:
            return
        if type(value) is str:
            check(len(value) <= 256, 'snapshot_string_size');return
        if type(value) is int:
            check(0 <= value <= 2**64-1, 'snapshot_integer_range');return
        if type(value) is float:
            check(sample(value), 'snapshot_float_range');return
        if type(value) in (list,tuple):
            check(len(value) <= 64, 'snapshot_sequence_size')
            for v in value:visit(v,depth+1)
            return
        check(type(value) is dict and len(value) <= 32, 'snapshot_object_type')
        for k,v in value.items():
            check(type(k) is str and len(k) <= 64, 'snapshot_key')
            visit(v,depth+1)
    check(type(snapshot) is dict, 'snapshot_type')
    visit(snapshot,0)
    data=json.dumps(snapshot,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
    check(len(data) <= 1_048_576, 'snapshot_size')
    return hashlib.sha256(data).hexdigest()


@dataclass(frozen=True)
class ScopeAnnotation:
    scope_id: str
    config_id: str
    provider_id: str
    session_id: str
    host_clock_id: str
    host_boot_id: str
    source_kind: str
    annotation_kind: str  # independently_recorded_fixture / retrospective_offline
    started_s: float
    closed_s: float
    counts_before: tuple[tuple[str,int], ...]
    snapshot_sha256: str


@dataclass(frozen=True)
class InputReference:
    scope_id: str
    expected: ExpectedBinding
    composite_sha256: str
    selection_id: int
    computation_id: int


@dataclass(frozen=True)
class ReceiptAdaptation:
    scope_id: str
    annotation_kind: str
    snapshot_sha256: str
    source_kind: str
    callback_record_ids: tuple[tuple[str,int], ...]
    review_at_recorded_close_ns: int
    bundle: BundleClaim
    contract_review: ContractReview
    receipt_quantization: str = field(default='floor(exact binary sample * 1e9); observation only, not original ns precision',init=False)
    provenance_authenticated: bool = field(default=False,init=False)
    shadow_admission_authorized: bool = field(default=False,init=False)
    native_live_observation_authorized: bool = field(default=False,init=False)
    motion_authority: bool = field(default=False,init=False)


def adapt(spec, scope, reference, snapshot):
    check(type(spec) is ProviderSpec and type(scope) is ScopeAnnotation
          and type(reference) is InputReference and type(reference.expected) is ExpectedBinding, 'adapter_types')
    check(identity(scope.scope_id) and type(reference.scope_id) is str
          and scope.scope_id == reference.scope_id, 'scope_identity')
    for name in ('config_id','provider_id','session_id','host_clock_id','host_boot_id'):
        check(identity(getattr(spec,name)) and type(getattr(scope,name)) is str
              and getattr(spec,name) == getattr(scope,name), 'scope_'+name)
    check(scope.source_kind in OFFLINE_SOURCES and type(scope.source_kind) is str, 'offline_source_only')
    check(type(scope.annotation_kind) is str and scope.annotation_kind in
          ('independently_recorded_fixture','retrospective_offline'), 'annotation_kind')
    check(sample(scope.started_s) and sample(scope.closed_s) and scope.started_s <= scope.closed_s, 'scope_times')
    check(type(spec.cameras) is tuple and 4 <= len(spec.cameras) <= 6
          and all(type(c) is CameraSpec for c in spec.cameras), 'camera_spec')
    names=tuple(c.view for c in spec.cameras)
    check(all(identity(v) for v in names) and len(set(names)) == len(names), 'camera_spec')
    check(all(c.device_clock_id is c.device_boot_id is c.mapping_sha256 is None for c in spec.cameras), 'receipt_only_spec')
    check(digest(scope.snapshot_sha256) and snapshot_digest(snapshot) == scope.snapshot_sha256, 'snapshot_pin')
    check(snapshot.get('source_kind') == scope.source_kind, 'snapshot_source')
    check(snapshot.get('actual_exposure_wall_s','missing') is None
          and snapshot.get('clock_mapping_verified') is False, 'unexpected_exposure_claim')
    check(type(snapshot.get('observer_errors')) is int and snapshot['observer_errors'] == 0, 'observer_errors')
    check(snapshot.get('counts_are_lower_bounds') is False and type(snapshot.get('capacity')) is int
          and snapshot['capacity'] == 64, 'history_saturated_or_capacity')
    check(type(snapshot.get('required_views')) in (list,tuple) and tuple(snapshot['required_views']) == names, 'required_camera_set')
    before=scope.counts_before
    check(type(before) is tuple and len(before) == 4 and all(type(x) is tuple and len(x) == 2
          and type(x[0]) is str and integer(x[1]) for x in before), 'scope_counter_baseline')
    before=dict(before)
    check(set(before) == set(KINDS), 'scope_counter_baseline')
    for key in ('histories','counts','dropped'):
        check(type(snapshot.get(key)) is dict and set(snapshot[key]) == set(KINDS), 'history_schema')
    histories=snapshot['histories']
    for kind in KINDS:
        rows=histories[kind];count=snapshot['counts'][kind];dropped=snapshot['dropped'][kind]
        check(type(rows) in (list,tuple) and integer(count) and count <= 64
              and type(dropped) is int and dropped == 0 and len(rows) == count, 'history_incomplete')
        check(before[kind] <= count and all(type(r) is dict and type(r.get('record_id')) is int
              and r['record_id'] == i for i,r in enumerate(rows,1)), 'record_ids_or_scope_baseline')
    def record(kind, rid):
        check(integer(rid) and before[kind] < rid <= len(histories[kind]), 'record_outside_scope')
        return histories[kind][rid-1]
    def times(row, fields):
        values=[row.get(k) for k in fields]
        check(all(sample(v) for v in values) and scope.started_s <= values[0]
              and values[-1] <= scope.closed_s and values == sorted(values), 'phase_or_scope_times')
        return values
    expected=reference.expected
    check(identity(expected.frame_id) and integer(expected.captured_sim_ns) and digest(reference.composite_sha256), 'input_identity')
    check(type(expected.view_digests) is tuple and len(expected.view_digests) == len(names)
          and all(type(x) is tuple and len(x) == 2 and identity(x[0]) and digest(x[1]) for x in expected.view_digests), 'input_view_digests')
    expected_digests=dict(expected.view_digests)
    check(len(expected_digests) == len(names) and set(expected_digests) == set(names), 'input_view_digests')
    comp=record('computations',reference.computation_id)
    selected=record('selections',reference.selection_id)
    check(comp.get('outcome') == 'returned' and comp.get('selection_binding_verified') is True
          and type(comp.get('selection_id')) is int and comp['selection_id'] == reference.selection_id, 'computation_binding')
    sim=comp.get('captured_sim_s')
    check(comp.get('frame_id') == expected.frame_id and sample(sim)
          and round(sim*1e9) == expected.captured_sim_ns and comp.get('rgb_sha256') == reference.composite_sha256, 'computation_input')
    aux={k:expected_digests[k] for k in names if k in ('up','down')}
    check(comp.get('auxiliary_sha256') == aux, 'auxiliary_input')
    check(selected.get('all_view_bindings_verified') is True and selected.get('kind') == 'selection'
          and type(selected.get('captured_sim_ns')) is int and selected['captured_sim_ns'] == expected.captured_sim_ns
          and selected.get('rgb_sha256') == reference.composite_sha256, 'selected_input')
    phase=times(selected,('next_frame_started_s','selected_observed_s','next_frame_finished_s'))
    compute_times=times(comp,('compute_started_s','compute_finished_s'))
    check(phase[-1] <= compute_times[0], 'selection_after_computation')
    views=selected.get('views')
    check(type(views) in (list,tuple) and len(views) == len(names)
          and all(type(v) is dict and identity(v.get('view')) for v in views), 'selected_view_set')
    by_view={v['view']:v for v in views}
    check(len(by_view) == len(names) and set(by_view) == set(names), 'selected_view_set')
    claims=[];callback_ids=[];cache_times=[]
    for camera in spec.cameras:
        v=by_view[camera.view]
        check(v.get('matches_observed_cache_pixels') is True and type(v.get('captured_sim_ns')) is int
              and v['captured_sim_ns'] == expected.captured_sim_ns and v.get('rgb_sha256') == expected_digests[camera.view], 'selected_view_binding')
        values=times(v,PHASES)
        cache_times.append(values[-1])
        check(values[-1] <= phase[1], 'cache_after_selection')
        matches=[r for r in histories['callbacks'] if r['record_id'] > before['callbacks']
                 and r.get('view') == camera.view and r.get('captured_sim_ns') == expected.captured_sim_ns
                 and r.get('rgb_sha256') == v['rgb_sha256'] and all(r.get(k) == v[k] for k in PHASES)]
        check(len(matches) == 1, 'callback_missing_or_ambiguous')
        callback=matches[0]
        check(type(callback.get('captured_sim_ns')) is int and callback.get('kind') == 'callback' and callback.get('source_kind') == scope.source_kind
              and callback.get('outcome') == 'returned' and callback.get('io_error_present') is False, 'callback_outcome')
        check(callback.get('actual_exposure_wall_s','missing') is None
              and callback.get('exposure_clock_mapping','missing') is None, 'callback_exposure_claim')
        times(callback,PHASES+('lock_released_s','callback_finished_s'))
        claims.append(ViewClaim(camera.view,camera.sensor_id,v['rgb_sha256'],receipt_ns(values[0])))
        callback_ids.append((camera.view,callback['record_id']))
    check(any(r['record_id'] > before['sync'] and type(r.get('captured_sim_ns')) is int
              and r['captured_sim_ns'] == expected.captured_sim_ns and r.get('all_required_views_observed') is True
              and sample(r.get('synchronized_set_observed_s'))
              and max(cache_times) <= r['synchronized_set_observed_s'] <= phase[1]
              for r in histories['sync']), 'synchronized_set_missing')
    bundle=BundleClaim(spec.config_id,spec.provider_id,spec.session_id,spec.host_clock_id,
        spec.host_boot_id,expected.frame_id,expected.captured_sim_ns,tuple(claims))
    closed_ns=receipt_ns(scope.closed_s)
    try:
        result=review(spec,bundle,expected,closed_ns)
    except ContractError as exc:
        raise AdapterError('contract:'+str(exc)) from exc
    return ReceiptAdaptation(scope.scope_id,scope.annotation_kind,scope.snapshot_sha256,scope.source_kind,
        tuple(callback_ids),closed_ns,bundle,result)
