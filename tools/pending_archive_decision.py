"""Pure opt-in review of a CALLER-SUPPLIED closed 2Z record, never authority.

A consistent forgery or replay can pass. This is neither a trusted consumer
nor a verifier of filesystem IO, provenance, freshness or protocol execution.
"""
import hashlib
import json
import re

PROFILE = 'toponav_pending_archive_decision_v1'
RECORD_LIMIT = 131072
ROLES = ('parent_manifest', 'projection_manifest', 'raw', 'observation')
ARTIFACT_ROLES = ('parent', 'projection', 'raw', 'observation')
CAPS = (8388608, 8192, 36893, 65536)
FALSE_FIELDS = tuple(('provenance_authenticated exposure_age_bounded shadow_admission_authorized '
    'native_live_observation_authorized motion_authority independent_enforcement_observed '
    'physical_stop_proved production_guard_fixed runtime_pending_installed runtime_result_consumed '
    'runtime_one_time_consumption_enforced cross_run_replay_prevented observation_input_binding_authenticated '
    'old_validation_site_observed unique_field_localization_observed old_validator_executed_by_diagnostic '
    'archive_origin_authenticated archive_filesystem_read_observed observation_semantics_validated '
    'pair_execution_link_observed').split())
PROJECTION_FALSE = tuple(('atomic_multi_file_snapshot_observed parent_manifest_origin_authenticated '
    'projection_parent_link_authenticated projection_filesystem_read_observed').split())
READER_FALSE = tuple(dict.fromkeys(('atomic_multi_file_snapshot_observed', 'root_origin_authenticated',
    'hostile_os_resistance_proved', 'hard_realtime_read_bound_proved', *PROJECTION_FALSE)))
EXTRA_FALSE = tuple(dict.fromkeys((*READER_FALSE, 'serialized_record_origin_authenticated',
    'caller_claim_authenticated', 'freshness_proved', 'result_tampering_excluded', 'cross_call_splice_excluded',
    'runtime_decision_enforced', 'one_time_result_consumption_enforced')))
READER_KEYS = set(('profile stage error filesystem_error_role os_errno claim_snapshot files projection_result '
    'local_regular_file_reads_complete sequential_samples_only observed_validation_site actual_exposure_wall_s').split()) | set(FALSE_FIELDS) | set(READER_FALSE)
PROJECTION_KEYS = set(('profile stage error classification claim_snapshot artifact_sha256 artifact_byte_counts '
    'parent_digest_matches projection_digest_matches raw_parent_projection_digest_matches '
    'observation_parent_projection_digest_matches projection_members_agree_with_parent reference_result '
    'pairing_basis pin_basis parent_reference_basis observed_validation_site actual_exposure_wall_s').split()) | set(FALSE_FIELDS) | set(PROJECTION_FALSE)
BINDING_KEYS = set(('profile stage error classification claim_snapshot artifact_sha256 artifact_byte_counts '
    'manifest_digest_matches raw_reference_digest_matches observation_reference_digest_matches file_references_agree '
    'pairing_declared pairing_basis manifest_pin_basis observation_encoding observed_validation_site actual_exposure_wall_s').split()) | set(FALSE_FIELDS)
FACT_KEYS = set(('role byte_count sha256 before_signature after_signature final_recheck_agreed source reference').split())


def _digest(v):
    return type(v) is str and re.fullmatch('[0-9a-f]{64}', v) is not None


def _claim(value):
    keys = {'profile', 'pairing_basis', 'parent_manifest_sha256', 'projection_sha256', *(k+'_ref' for k in ROLES)}
    if type(value) is not dict or set(value) != keys or any(type(v) is not str for v in value.values()): return None
    c = dict(value)
    if c['profile'] != 'toponav_pending_archive_projection_v1' or c['pairing_basis'] != 'caller_declared_only': return None
    if not _digest(c['parent_manifest_sha256']) or not _digest(c['projection_sha256']): return None
    refs = [c[k+'_ref'] for k in ROLES]
    if len(set(refs)) != 4: return None
    for ref in refs:
        if not 0 < len(ref) <= 512 or re.fullmatch('[A-Za-z0-9._+/-]+', ref) is None: return None
        parts = ref.split('/')
        if len(parts) > 32 or any(p in ('', '.', '..') for p in parts): return None
    return c


def _pairs(pairs):
    value = {}
    for k, v in pairs:
        if k in value: raise ValueError('duplicate')
        value[k] = v
    return value


def _noninteger(value):
    raise ValueError('noninteger_json_number')


def _flags(value, extras=()):
    return (all(value.get(k) is False for k in (*FALSE_FIELDS, *extras))
            and value.get('observed_validation_site') is None and value.get('actual_exposure_wall_s') is None)


def _mapping(value, expected, integers=False):
    return (type(value) is dict and set(value) == set(expected) and value == expected
            and (not integers or all(type(v) is int for v in value.values())))


def review_reader_decision(record_raw, expected_claim):
    """Fail-closed STRUCTURAL consistency only. No IO, clocks, state or consume.

    Check top-level refusal before consulting nested positives/claim/facts.
    Repetition is deterministic; fully consistent fabricated/spliced records
    can pass. Both the record and expected claim are UNAUTHENTICATED inputs.
    """
    out = dict(profile=PROFILE, stage='decision_refused', error=None,
        offline_reference_review_ready=False, diagnostic_only=True, stateful_consumption=False,
        decision_basis='caller_supplied_record_and_claim_not_trust_anchor', input_record_sha256=None,
        reader_stage=None, reader_error=None, reader_local_reads_complete=None, expected_claim_snapshot=None,
        sampled_artifact_sha256=None, observed_validation_site=None, actual_exposure_wall_s=None,
        **{k: False for k in (*FALSE_FIELDS, *EXTRA_FALSE)})
    def refuse(error):
        out['error'] = error
        return out
    if type(record_raw) is not bytes: return refuse('decision_record_bytes')
    if len(record_raw) > RECORD_LIMIT: return refuse('decision_record_byte_bound')
    out['input_record_sha256'] = hashlib.sha256(record_raw).hexdigest()
    try:
        record = json.loads(record_raw.decode('utf-8'), object_pairs_hook=_pairs,
            parse_constant=_noninteger, parse_float=_noninteger)
    except (ValueError, RecursionError): return refuse('decision_record_json')
    if type(record) is not dict or set(record) != {'result', 'claim', 'events', 'initial_sha256'}:
        return refuse('decision_record_shape')
    r = record['result']
    if type(r) is not dict: return refuse('decision_reader_shape')
    # Preserve null/unknown as null; never invent an old error or site.
    for source, dest in (('stage', 'reader_stage'), ('error', 'reader_error')):
        if type(r.get(source)) is str: out[dest] = r[source]
    if type(r.get('local_regular_file_reads_complete')) is bool:
        out['reader_local_reads_complete'] = r['local_regular_file_reads_complete']
    if r.get('stage') != 'projection_reader_checked': return refuse('reader_top_level_not_checked')
    if set(r) != READER_KEYS or r['profile'] != 'toponav_pending_archive_projection_reader_v1':
        return refuse('decision_reader_shape')
    if (r['error'] is not None or r['filesystem_error_role'] is not None or r['os_errno'] is not None
            or r['local_regular_file_reads_complete'] is not True or r['sequential_samples_only'] is not True
            or not _flags(r, READER_FALSE)):
        return refuse('decision_reader_state')
    c = _claim(expected_claim)
    if c is None: return refuse('decision_expected_claim_shape')
    if record['claim'] != c or r['claim_snapshot'] != c: return refuse('decision_claim_mismatch')
    out['expected_claim_snapshot'] = c
    # Harness metadata is not an evidence source or a permission. Only its
    # container shape is checked; events do not create a temporal guarantee.
    initial = record['initial_sha256']
    if (type(record['events']) is not list or len(record['events']) > 64
            or (initial is not None and (type(initial) is not dict or set(initial) != set(ROLES)
                or any(not _digest(v) for v in initial.values())))):
        return refuse('decision_record_metadata')
    files = r['files']
    if type(files) is not list or len(files) != 4: return refuse('decision_file_facts')
    hashes, counts = {}, {}
    for role, artifact, cap, f in zip(ROLES, ARTIFACT_ROLES, CAPS, files):
        if type(f) is not dict or set(f) != FACT_KEYS: return refuse('decision_file_facts')
        sig = f['before_signature']
        if (f['role'] != role or f['reference'] != c[role+'_ref'] or not _digest(f['sha256'])
                or type(f['byte_count']) is not int or not 0 <= f['byte_count'] <= cap
                or f['final_recheck_agreed'] is not True
                or f['source'] != 'local_sequential_read_not_authenticated_origin'
                or type(sig) is not list or len(sig) != 7 or any(type(v) is not int for v in sig)
                or type(f['after_signature']) is not list or any(type(v) is not int for v in f['after_signature'])
                or sig != f['after_signature'] or sig[0] < 0 or sig[1] < 0
                or sig[2] & 0o170000 != 0o100000 or sig[3] != f['byte_count'] or sig[6] < 1):
            return refuse('decision_file_facts')
        hashes[artifact] = f['sha256']; counts[artifact] = f['byte_count']
    if hashes['parent'] != c['parent_manifest_sha256'] or hashes['projection'] != c['projection_sha256']:
        return refuse('decision_pin_mismatch')
    p = r['projection_result']
    if type(p) is not dict or set(p) != PROJECTION_KEYS: return refuse('decision_projection_shape')
    if (p['profile'] != 'toponav_pending_archive_projection_v1' or p['stage'] != 'projection_reference_checked'
            or p['error'] is not None or p['claim_snapshot'] != c or not _flags(p, PROJECTION_FALSE)
            or any(p[k] is not True for k in ('parent_digest_matches', 'projection_digest_matches',
                'raw_parent_projection_digest_matches', 'observation_parent_projection_digest_matches', 'projection_members_agree_with_parent'))
            or p['classification'] != 'offline_catalog_selection_not_authenticated_binding'
            or p['pairing_basis'] != 'caller_declared_only' or p['pin_basis'] != 'caller_supplied_not_trust_anchor'
            or p['parent_reference_basis'] != 'declarative_name_not_filesystem_observation'
            or not _mapping(p['artifact_sha256'], hashes) or not _mapping(p['artifact_byte_counts'], counts, True)):
        return refuse('decision_projection_state')
    b = p['reference_result']
    if type(b) is not dict or set(b) != BINDING_KEYS: return refuse('decision_binding_shape')
    oldclaim = dict(profile='toponav_pending_archive_binding_v1', manifest_ref=c['projection_manifest_ref'],
        manifest_sha256=c['projection_sha256'], raw_ref=c['raw_ref'], observation_ref=c['observation_ref'], pairing_basis='caller_declared_only')
    bh = dict(manifest=hashes['projection'], raw=hashes['raw'], observation=hashes['observation'])
    bn = dict(manifest=counts['projection'], raw=counts['raw'], observation=counts['observation'])
    if (b['profile'] != 'toponav_pending_archive_binding_v1' or b['stage'] != 'archive_reference_checked'
            or b['error'] is not None or b['claim_snapshot'] != oldclaim or not _flags(b)
            or any(b[k] is not True for k in ('manifest_digest_matches', 'raw_reference_digest_matches',
                'observation_reference_digest_matches', 'file_references_agree', 'pairing_declared'))
            or b['classification'] != 'offline_reference_agreement_not_authenticated_binding'
            or b['pairing_basis'] != 'caller_declared_only' or b['manifest_pin_basis'] != 'caller_supplied_not_trust_anchor'
            or b['observation_encoding'] != 'opaque_archive_bytes_not_parsed'
            or not _mapping(b['artifact_sha256'], bh) or not _mapping(b['artifact_byte_counts'], bn, True)):
        return refuse('decision_binding_state')
    out.update(stage='offline_reference_consistent', offline_reference_review_ready=True, sampled_artifact_sha256=hashes)
    return out
