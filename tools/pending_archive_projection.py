"""Pure opt-in parent/minimal-catalog agreement, NOT trusted provenance.

The bounded large-parent profile is NEW. Neither the sealed 2W parser/caps nor
2X's filesystem reader are modified, bypassed or silently retried.
"""
import hashlib
import json
import re
from tools import pending_archive_binding as binding

PROFILE = 'toponav_pending_archive_projection_v1'
PARENT_LIMIT = 8 * 1024 * 1024
PARENT_ENTRY_LIMIT = 24000
PROJECTION_LIMIT = 8192
PROJECTION_SCOPE = 'toponav_pending_archive_projection_v1: caller-selected members, not authenticated provenance'
EXTRA_FALSE = ('atomic_multi_file_snapshot_observed', 'parent_manifest_origin_authenticated',
               'projection_parent_link_authenticated', 'projection_filesystem_read_observed')
sha = lambda raw: hashlib.sha256(raw).hexdigest()


def _path(value):
    return (type(value) is str and 0 < len(value) <= 512
            and re.fullmatch('[A-Za-z0-9._+/-]+', value) is not None
            and all(p not in ('', '.', '..') for p in value.split('/')))


def _digest(value):
    return type(value) is str and len(value) == 64 and re.fullmatch('[0-9a-f]{64}', value) is not None


def _pairs(pairs):
    result = {}
    for k, v in pairs:
        if k in result: raise ValueError('duplicate_json_key')
        result[k] = v
    return result


def _constant(value):
    raise ValueError('nonfinite_json_constant')


def _catalog(raw, limit):
    value = json.loads(raw.decode('utf-8'), object_pairs_hook=_pairs, parse_constant=_constant)
    if type(value) is not dict or set(value) != {'scope', 'files'}:
        raise ValueError('catalog_shape')
    if type(value['scope']) is not str or len(value['scope']) > 4096:
        raise ValueError('catalog_scope')
    files = value['files']
    if type(files) is not dict or not 1 <= len(files) <= limit:
        raise ValueError('catalog_files')
    if any(not _path(k) or not _digest(v) for k, v in files.items()):
        raise ValueError('catalog_entry')
    return value


def _claim(claim):
    keys = {'profile', 'parent_manifest_ref', 'parent_manifest_sha256', 'projection_manifest_ref',
            'projection_sha256', 'raw_ref', 'observation_ref', 'pairing_basis'}
    if type(claim) is not dict or len(claim) != len(keys): return None
    c = dict(claim)
    if set(c) != keys or any(type(v) is not str for v in c.values()): return None
    if c['profile'] != PROFILE or c['pairing_basis'] != 'caller_declared_only': return None
    if not _digest(c['parent_manifest_sha256']) or not _digest(c['projection_sha256']): return None
    refs = [c[k] for k in ('parent_manifest_ref', 'projection_manifest_ref', 'raw_ref', 'observation_ref')]
    if any(not _path(v) for v in refs) or len(set(refs)) != 4: return None
    return c


def review_projection(parent_raw, projection_raw, claim, raw, observation_raw):
    """Check raw bytes -> selected catalog members -> caller-pinned parent.

    Parent/projection pins and references are caller declarations, not trust
    anchors. A complete replacement or an unrelated selected pair may agree.
    No files, clocks, protocol validators, old reader or 2V diagnostic run.
    """
    out = dict(profile=PROFILE, stage='projection_refused', error=None,
               classification='offline_catalog_selection_not_authenticated_binding',
               claim_snapshot=None, artifact_sha256=None, artifact_byte_counts=None,
               parent_digest_matches=None, projection_digest_matches=None,
               raw_parent_projection_digest_matches=None, observation_parent_projection_digest_matches=None,
               projection_members_agree_with_parent=False, reference_result=None,
               pairing_basis='caller_declared_only', pin_basis='caller_supplied_not_trust_anchor',
               parent_reference_basis='declarative_name_not_filesystem_observation',
               observed_validation_site=None, actual_exposure_wall_s=None,
               **{k: False for k in (*binding.FALSE_FIELDS, *EXTRA_FALSE)})
    for role, blob, cap in (('parent', parent_raw, PARENT_LIMIT), ('projection', projection_raw, PROJECTION_LIMIT),
                            ('raw', raw, binding.RAW_LIMIT), ('observation', observation_raw, binding.OBSERVATION_LIMIT)):
        if type(blob) is not bytes:
            out['error'] = role + '_bytes'; return out
        if len(blob) > cap:
            out['error'] = role + '_byte_bound'; return out
    c = _claim(claim)
    if c is None:
        out['error'] = 'projection_claim_shape'; return out
    out['claim_snapshot'] = c
    blobs = dict(parent=parent_raw, projection=projection_raw, raw=raw, observation=observation_raw)
    out['artifact_sha256'] = {k: sha(v) for k, v in blobs.items()}
    out['artifact_byte_counts'] = {k: len(v) for k, v in blobs.items()}
    out['parent_digest_matches'] = out['artifact_sha256']['parent'] == c['parent_manifest_sha256']
    if not out['parent_digest_matches']:
        out['error'] = 'parent_digest_mismatch'; return out
    try:
        parent = _catalog(parent_raw, PARENT_ENTRY_LIMIT)['files']
    except (ValueError, RecursionError):
        out['error'] = 'parent_catalog_shape'; return out
    out['projection_digest_matches'] = out['artifact_sha256']['projection'] == c['projection_sha256']
    if not out['projection_digest_matches']:
        out['error'] = 'projection_digest_mismatch'; return out
    try:
        projection = _catalog(projection_raw, 2)
        if projection['scope'] != PROJECTION_SCOPE or set(projection['files']) != {c['raw_ref'], c['observation_ref']}:
            raise ValueError('projection_selection_shape')
    except (ValueError, RecursionError):
        out['error'] = 'projection_catalog_shape'; return out
    for role in ('raw', 'observation'):
        ref = c[role + '_ref']
        if ref not in parent:
            out['error'] = role + '_parent_reference_missing'; return out
        flag = role + '_parent_projection_digest_matches'
        out[flag] = parent[ref] == projection['files'][ref]
        if not out[flag]:
            out['error'] = role + '_parent_projection_digest_mismatch'; return out
    out['projection_members_agree_with_parent'] = True
    old_claim = dict(profile=binding.PROFILE, manifest_ref=c['projection_manifest_ref'],
                     manifest_sha256=c['projection_sha256'], raw_ref=c['raw_ref'],
                     observation_ref=c['observation_ref'], pairing_basis='caller_declared_only')
    result = binding.review_archive_binding(projection_raw, old_claim, raw, observation_raw)
    out['reference_result'] = result
    if not result['file_references_agree']:
        # The old error label remains exactly in the nested old result.
        out['error'] = 'projection_reference_refused'; return out
    out.update(stage='projection_reference_checked')
    return out
