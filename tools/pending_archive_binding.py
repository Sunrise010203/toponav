"""Pure opt-in archive-reference agreement, NOT authenticated input binding.

All bytes, names and the manifest pin are supplied by the caller. No path is
opened. In particular, replacing the entire manifest/pin/pair can still agree.
"""
import hashlib
import json
import re

PROFILE = 'toponav_pending_archive_binding_v1'
MANIFEST_LIMIT = 4 * 1024 * 1024
RAW_LIMIT = 36893
OBSERVATION_LIMIT = 65536
ENTRY_LIMIT = 12000
PATH_LIMIT = 512
FALSE_FIELDS = (
    'provenance_authenticated', 'exposure_age_bounded', 'shadow_admission_authorized',
    'native_live_observation_authorized', 'motion_authority',
    'independent_enforcement_observed', 'physical_stop_proved', 'production_guard_fixed',
    'runtime_pending_installed', 'runtime_result_consumed',
    'runtime_one_time_consumption_enforced', 'cross_run_replay_prevented',
    'observation_input_binding_authenticated', 'old_validation_site_observed',
    'unique_field_localization_observed', 'old_validator_executed_by_diagnostic',
    'archive_origin_authenticated', 'archive_filesystem_read_observed',
    'observation_semantics_validated', 'pair_execution_link_observed',
)
sha = lambda raw: hashlib.sha256(raw).hexdigest()


def _digest(value):
    return type(value) is str and len(value) == 64 and re.fullmatch('[0-9a-f]{64}', value) is not None


def _path(value):
    # A lexical reference only, not a sandbox, realpath or file access check.
    return (type(value) is str and 0 < len(value) <= PATH_LIMIT
            and re.fullmatch('[A-Za-z0-9._+/-]+', value) is not None
            and all(p not in ('', '.', '..') for p in value.split('/')))


def _pairs(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError('duplicate_json_key')
        value[key] = item
    return value


def _constant(value):
    raise ValueError('nonfinite_json_constant')


def _manifest(raw):
    # Strict UTF-8, no BOM, duplicate members or nonfinite constants. This is
    # this NEW contract's shape, not an amendment to any old manifest/schema.
    value = json.loads(raw.decode('utf-8'), object_pairs_hook=_pairs, parse_constant=_constant)
    if type(value) is not dict or set(value) != {'scope', 'files'}:
        raise ValueError('manifest_shape')
    if type(value['scope']) is not str or len(value['scope']) > 4096:
        raise ValueError('manifest_scope')
    files = value['files']
    if type(files) is not dict or not 1 <= len(files) <= ENTRY_LIMIT:
        raise ValueError('manifest_files')
    if any(not _path(k) or not _digest(v) for k, v in files.items()):
        raise ValueError('manifest_entry')
    return files


def review_archive_binding(manifest_raw, claim, raw, observation_raw):
    """Check exact artifact bytes against a caller-pinned archive manifest.

    observation_raw is OPAQUE: a digest match says neither that it is a valid
    snapshot nor that an old execution produced it from raw. Even a checked
    pair is only a caller-declared pairing; hashes do not authenticate it.
    """
    out = dict(profile=PROFILE, stage='archive_reference_refused', error=None,
               classification='offline_reference_agreement_not_authenticated_binding',
               claim_snapshot=None, artifact_sha256=None, artifact_byte_counts=None,
               manifest_digest_matches=None, raw_reference_digest_matches=None,
               observation_reference_digest_matches=None, file_references_agree=False,
               pairing_declared=False, pairing_basis='caller_declared_only',
               manifest_pin_basis='caller_supplied_not_trust_anchor',
               observation_encoding='opaque_archive_bytes_not_parsed',
               observed_validation_site=None, actual_exposure_wall_s=None,
               **{k: False for k in FALSE_FIELDS})
    # Refuse oversized/type-invalid byte inputs before any hashing or decoding.
    for name, value, limit in (('manifest', manifest_raw, MANIFEST_LIMIT),
                               ('raw', raw, RAW_LIMIT),
                               ('observation', observation_raw, OBSERVATION_LIMIT)):
        if type(value) is not bytes:
            out['error'] = name + '_bytes'
            return out
        if len(value) > limit:
            out['error'] = name + '_byte_bound'
            return out
    keys = {'profile', 'manifest_ref', 'manifest_sha256', 'raw_ref', 'observation_ref', 'pairing_basis'}
    if (type(claim) is not dict or len(claim) != len(keys) or set(claim) != keys
            or type(claim['profile']) is not str or claim['profile'] != PROFILE
            or type(claim['pairing_basis']) is not str or claim['pairing_basis'] != 'caller_declared_only'
            or not _digest(claim['manifest_sha256'])
            or any(not _path(claim[k]) for k in ('manifest_ref', 'raw_ref', 'observation_ref'))
            or len({claim[k] for k in ('manifest_ref', 'raw_ref', 'observation_ref')}) != 3):
        out['error'] = 'claim_shape'
        return out
    out['claim_snapshot'] = dict(claim)
    out['pairing_declared'] = True
    blobs = dict(manifest=manifest_raw, raw=raw, observation=observation_raw)
    out['artifact_sha256'] = {k: sha(v) for k, v in blobs.items()}
    out['artifact_byte_counts'] = {k: len(v) for k, v in blobs.items()}
    out['manifest_digest_matches'] = out['artifact_sha256']['manifest'] == claim['manifest_sha256']
    if not out['manifest_digest_matches']:
        out['error'] = 'manifest_digest_mismatch'
        return out
    try:
        files = _manifest(manifest_raw)
    except (ValueError, UnicodeError, RecursionError):
        out['error'] = 'manifest_shape'
        return out
    for role, flag in (('raw', 'raw_reference_digest_matches'),
                       ('observation', 'observation_reference_digest_matches')):
        ref = claim[role + '_ref']
        if ref not in files:
            out['error'] = role + '_reference_missing'
            return out
        out[flag] = out['artifact_sha256'][role] == files[ref]
        if not out[flag]:
            out['error'] = role + '_digest_mismatch'
            return out
    out.update(stage='archive_reference_checked', file_references_agree=True)
    return out
