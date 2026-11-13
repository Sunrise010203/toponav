"""OFFLINE DATA CONTRACT ONLY. No real provider, watchdog, I/O or authority.

Schema consistency of caller-supplied declarations is NOT source authentication.
Receipt-only observations retain unknown exposure. No adapter to any gate/token
is provided. All times here are integer ns, or raw device ticks as named.
"""
from dataclasses import asdict, dataclass, field
from fractions import Fraction
import hashlib
import json

MAX_INT = 2**63-1
MAX_RATE_COMPONENT = 10**12
BASE_VIEWS = ('front', 'left', 'right', 'back')


class ContractError(ValueError):
    pass


def require(condition, reason):
    if not condition:
        raise ContractError(reason)


def identity(value):
    return type(value) is str and 0 < len(value) <= 256


def integer(value, maximum=MAX_INT):
    return type(value) is int and 0 <= value <= maximum


def digest(value):
    return type(value) is str and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


@dataclass(frozen=True)
class ClockMap:
    mapping_id: str
    calibration_evidence_sha256: str
    provider_id: str
    sensor_id: str
    device_clock_id: str
    device_boot_id: str
    host_clock_id: str
    host_boot_id: str
    anchor_tick: int
    reference_lower_ns: int
    reference_upper_ns: int
    rate_lower_num: int
    rate_lower_den: int
    rate_upper_num: int
    rate_upper_den: int
    valid_device_lower_tick: int
    valid_device_upper_tick: int
    valid_host_from_ns: int
    valid_host_until_ns: int
    clock_kind: str = 'device_exposure_counter'


@dataclass(frozen=True)
class CameraSpec:
    view: str
    sensor_id: str
    device_clock_id: str | None = None
    device_boot_id: str | None = None
    mapping_sha256: str | None = None


@dataclass(frozen=True)
class ProviderSpec:
    config_id: str
    provider_id: str
    session_id: str
    host_clock_id: str
    host_boot_id: str
    cameras: tuple[CameraSpec, ...]


@dataclass(frozen=True)
class ViewClaim:
    view: str
    sensor_id: str
    rgb_sha256: str
    callback_host_ns: int
    source_kind: str = 'receipt_only'
    exposure_start_tick: int | None = None
    exposure_end_tick: int | None = None
    exposure_scope: str | None = None
    mapping: ClockMap | None = None


@dataclass(frozen=True)
class BundleClaim:
    config_id: str
    provider_id: str
    session_id: str
    host_clock_id: str
    host_boot_id: str
    frame_id: str
    captured_sim_ns: int | None
    views: tuple[ViewClaim, ...]


@dataclass(frozen=True)
class ExpectedBinding:
    """Independently held input identity/digests, not trusted just by type."""
    frame_id: str
    captured_sim_ns: int | None
    view_digests: tuple[tuple[str, str], ...]


@dataclass(frozen=True)
class ContractReview:
    schema_consistent: bool
    declared_exposure_envelopes_ns: tuple[tuple[str, int | None, int | None], ...]
    all_declared_bounds_present: bool
    oldest_declared_lower_ns: int | None
    warnings: tuple[str, ...]
    provenance_authenticated: bool = field(default=False, init=False)
    shadow_admission_authorized: bool = field(default=False, init=False)
    native_live_observation_authorized: bool = field(default=False, init=False)
    motion_authority: bool = field(default=False, init=False)


def validate_mapping(mapping):
    require(type(mapping) is ClockMap, 'mapping_type')
    for name in ('mapping_id', 'provider_id', 'sensor_id', 'device_clock_id',
                 'device_boot_id', 'host_clock_id', 'host_boot_id'):
        require(identity(getattr(mapping, name)), 'mapping_identity')
    require(digest(mapping.calibration_evidence_sha256), 'calibration_evidence_digest')
    require(type(mapping.clock_kind) is str and mapping.clock_kind == 'device_exposure_counter', 'not_device_exposure_clock')
    for name in ('anchor_tick', 'reference_lower_ns', 'reference_upper_ns',
                 'valid_device_lower_tick', 'valid_device_upper_tick',
                 'valid_host_from_ns', 'valid_host_until_ns'):
        require(integer(getattr(mapping, name)), 'mapping_integer_range')
    for name in ('rate_lower_num', 'rate_lower_den', 'rate_upper_num', 'rate_upper_den'):
        value = getattr(mapping, name)
        require(integer(value, MAX_RATE_COMPONENT) and value > 0, 'mapping_rate_range')
    require(mapping.reference_lower_ns <= mapping.reference_upper_ns, 'reference_interval')
    require(mapping.valid_device_lower_tick <= mapping.anchor_tick <= mapping.valid_device_upper_tick,
            'device_validity_interval')
    require(mapping.valid_host_from_ns <= mapping.reference_lower_ns <= mapping.reference_upper_ns
            < mapping.valid_host_until_ns, 'host_validity_interval')
    require(Fraction(mapping.rate_lower_num, mapping.rate_lower_den)
            <= Fraction(mapping.rate_upper_num, mapping.rate_upper_den), 'rate_interval')


def mapping_digest(mapping):
    """Pin all declared map fields, not merely an arbitrary mapping name."""
    validate_mapping(mapping)
    data = json.dumps(asdict(mapping), sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    return hashlib.sha256(data).hexdigest()


def map_envelope(mapping, start_tick, end_tick):
    """Conservative floor(lower)/ceil(upper); handles negative anchor deltas.

    Rates are positive bounded rational ns/tick over the WHOLE validity window.
    This verifies declared math only, never the physical calibration premise.
    """
    validate_mapping(mapping)
    require(integer(start_tick) and integer(end_tick), 'exposure_tick_range')
    require(mapping.valid_device_lower_tick <= start_tick <= end_tick <= mapping.valid_device_upper_tick,
            'exposure_outside_device_validity')
    rates = (Fraction(mapping.rate_lower_num, mapping.rate_lower_den),
             Fraction(mapping.rate_upper_num, mapping.rate_upper_den))
    lower = mapping.reference_lower_ns + min((start_tick-mapping.anchor_tick)*r for r in rates)
    upper = mapping.reference_upper_ns + max((end_tick-mapping.anchor_tick)*r for r in rates)
    lo = lower.numerator // lower.denominator
    hi = -((-upper.numerator) // upper.denominator)
    require(integer(lo) and integer(hi) and lo <= hi, 'mapped_host_range')
    require(mapping.valid_host_from_ns <= lo <= hi < mapping.valid_host_until_ns,
            'exposure_outside_host_validity')
    return lo, hi


def review(spec, bundle, expected, now_host_ns):
    """Stateless, non-authorizing schema/binding/math review. No clock reads.

    ExpectedBinding and ProviderSpec must eventually be independently held;
    supplying mutually forged objects still passes consistency, not trust.
    Replay and device-counter rollover require a future independent owner.
    """
    require(type(spec) is ProviderSpec and type(bundle) is BundleClaim
            and type(expected) is ExpectedBinding, 'contract_type')
    require(integer(now_host_ns), 'now_host_range')
    for name in ('config_id', 'provider_id', 'session_id', 'host_clock_id', 'host_boot_id'):
        require(identity(getattr(spec, name)) and type(getattr(bundle, name)) is str
                and getattr(bundle, name) == getattr(spec, name), 'bundle_'+name)
    cameras = spec.cameras
    require(type(cameras) is tuple and 4 <= len(cameras) <= 6
            and all(type(c) is CameraSpec for c in cameras), 'camera_spec_type')
    names = tuple(c.view for c in cameras)
    require(all(identity(v) for v in names) and names[:4] == BASE_VIEWS
            and len(set(names)) == len(names) and set(names[4:]) <= {'up','down'}, 'required_camera_set')
    for camera in cameras:
        require(identity(camera.sensor_id), 'sensor_identity')
        fields = (camera.device_clock_id, camera.device_boot_id, camera.mapping_sha256)
        require(all(v is None for v in fields) or
                (identity(fields[0]) and identity(fields[1]) and digest(fields[2])), 'camera_mapping_pin')
    require(len({c.sensor_id for c in cameras}) == len(cameras), 'duplicate_sensor_identity')
    require(identity(expected.frame_id) and type(bundle.frame_id) is str
            and bundle.frame_id == expected.frame_id, 'frame_binding')
    for sim in (expected.captured_sim_ns, bundle.captured_sim_ns):
        require(sim is None or integer(sim), 'sim_identity_range')
    require(bundle.captured_sim_ns == expected.captured_sim_ns, 'sim_binding')
    require(type(expected.view_digests) is tuple and len(expected.view_digests) == len(names), 'expected_view_set')
    for item in expected.view_digests:
        require(type(item) is tuple and len(item) == 2 and identity(item[0]) and digest(item[1]), 'expected_digest')
    input_digests = dict(expected.view_digests)
    require(len(input_digests) == len(names) and set(input_digests) == set(names), 'expected_view_set')
    require(type(bundle.views) is tuple and len(bundle.views) == len(names)
            and all(type(v) is ViewClaim and identity(v.view) for v in bundle.views), 'claim_view_set')
    claims = {v.view:v for v in bundle.views}
    require(len(claims) == len(names) and set(claims) == set(names), 'claim_view_set')
    intervals = []
    warnings = ['declarations_not_authenticated', 'stateless_no_replay_protection']
    for camera in cameras:
        v = claims[camera.view]
        require(type(v.sensor_id) is str and v.sensor_id == camera.sensor_id, 'view_sensor_binding')
        require(digest(v.rgb_sha256) and v.rgb_sha256 == input_digests[v.view], 'view_pixel_binding')
        require(integer(v.callback_host_ns) and v.callback_host_ns <= now_host_ns, 'callback_host_range')
        require(type(v.source_kind) is str and v.source_kind in ('receipt_only', 'device_exposure_ticks'), 'source_kind')
        if v.source_kind == 'receipt_only':
            require(all(x is None for x in (v.exposure_start_tick, v.exposure_end_tick,
                    v.exposure_scope, v.mapping)), 'receipt_cannot_claim_exposure')
            intervals.append((v.view, None, None));warnings.append('exposure_unknown:'+v.view)
            continue
        require(type(v.exposure_scope) is str and v.exposure_scope == 'all_pixel_exposure_envelope', 'exposure_scope')
        m = v.mapping
        validate_mapping(m)
        require(camera.mapping_sha256 is not None and mapping_digest(m) == camera.mapping_sha256,
                'unpinned_or_changed_mapping')
        require((m.provider_id, m.sensor_id, m.device_clock_id, m.device_boot_id,
                 m.host_clock_id, m.host_boot_id) ==
                (spec.provider_id, camera.sensor_id, camera.device_clock_id, camera.device_boot_id,
                 spec.host_clock_id, spec.host_boot_id), 'mapping_clock_sensor_binding')
        require(m.valid_host_from_ns <= v.callback_host_ns <= now_host_ns < m.valid_host_until_ns,
                'mapping_expired_or_not_yet_valid')
        lo, hi = map_envelope(m, v.exposure_start_tick, v.exposure_end_tick)
        require(hi <= v.callback_host_ns, 'exposure_after_callback')
        intervals.append((v.view, lo, hi))
    complete = all(lo is not None for _,lo,_ in intervals)
    oldest = min(lo for _,lo,_ in intervals) if complete else None
    return ContractReview(True, tuple(intervals), complete, oldest, tuple(warnings))
