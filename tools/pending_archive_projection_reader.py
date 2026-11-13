"""Isolated Linux four-file sequential reader; NOT atomic/authenticated/authority.

Uses unchanged sealed 2X low-level descriptor helpers, never its pair reader.
The only intentional follow is a pinned regular inode via /proc/<pid>/fd.
"""
import os
from tools import pending_archive_reader as io
from tools import pending_archive_projection as projection
from tools import pending_archive_binding as binding

PROFILE = 'toponav_pending_archive_projection_reader_v1'
ROOT_LIMIT, ROOT_DEPTH, REFERENCE_DEPTH = io.ROOT_LIMIT, io.ROOT_DEPTH, io.REFERENCE_DEPTH
EXTRA_FALSE = tuple(dict.fromkeys((*io.EXTRA_FALSE, *projection.EXTRA_FALSE)))
_Refusal = io._Refusal
_open, _directory, _read_regular, _recheck = io._open, io._directory, io._read_regular, io._recheck


def _claim_snapshot(claim):
    c = projection._claim(claim)
    if c is None or any(len(c[k].split('/')) > REFERENCE_DEPTH for k in
            ('parent_manifest_ref', 'projection_manifest_ref', 'raw_ref', 'observation_ref')):
        return None
    return c


def read_projection_archive(root, claim):
    """Read a caller-selected local pair and apply the SEALED 2Y pure contract.

    Linux+mounted /proc is required; no fallback. Stability is sampled before,
    after and at final recheck, NOT an atomic snapshot, a lease or provenance.
    Hard links, bind mounts, change-and-restore and hostile OS are NOT excluded.
    No descriptor or file bytes escape; every owned descriptor closes on exit.
    """
    out = dict(profile=PROFILE, stage='projection_reader_refused', error=None, filesystem_error_role=None,
               os_errno=None, claim_snapshot=None, files=[], projection_result=None,
               local_regular_file_reads_complete=False, sequential_samples_only=True,
               observed_validation_site=None, actual_exposure_wall_s=None,
               **{k: False for k in (*binding.FALSE_FIELDS, *EXTRA_FALSE)})
    if (type(root) is not str or not 0 < len(root) <= ROOT_LIMIT or not root.startswith('/')
            or '\0' in root or any(0xD800 <= ord(c) <= 0xDFFF for c in root)
            or (root != '/' and any(p in ('', '.', '..') for p in root.split('/')[1:]))
            or len(root.split('/')[1:]) > ROOT_DEPTH):
        out['error'] = 'projection_reader_root_shape'
        return out
    c = _claim_snapshot(claim)
    if c is None:
        out['error'] = 'projection_reader_claim_shape'
        return out
    out['claim_snapshot'] = c
    if (os.name != 'posix' or any(not hasattr(os, k) for k in ('O_PATH', 'O_NOFOLLOW', 'O_DIRECTORY', 'O_CLOEXEC', 'O_NONBLOCK'))
            or os.open not in os.supports_dir_fd or os.stat not in os.supports_dir_fd
            or os.stat not in os.supports_follow_symlinks):
        out['error'] = 'projection_reader_platform_unsupported'
        return out
    state = dict(fds=[], links=[], files=[], proc_fd=None)
    try:
        anchor = _open('/', os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, state, 'root')
        root_fd = anchor
        if root != '/':
            for part in root.split('/')[1:]: root_fd = _directory(root_fd, part, state, 'root')
        # These fixed kernel paths do not incorporate a caller path. Retain
        # their directory descriptors; only the final numeric fd is followed.
        proc = _directory(anchor, 'proc', state, 'proc')
        proc = _directory(proc, str(os.getpid()), state, 'proc')
        state['proc_fd'] = _directory(proc, 'fd', state, 'proc')
        blobs = {}
        for role, cap in (('parent_manifest', projection.PARENT_LIMIT),
                          ('projection_manifest', projection.PROJECTION_LIMIT),
                          ('raw', binding.RAW_LIMIT), ('observation', binding.OBSERVATION_LIMIT)):
            ref = c[role + '_ref']; parts = ref.split('/'); parent = root_fd
            for part in parts[:-1]: parent = _directory(parent, part, state, role)
            blob, fact = _read_regular(parent, parts[-1], cap, role, state)
            fact['reference'] = ref; out['files'].append(fact); blobs[role] = blob
        # Check both before and after the pure review. A positive nested result
        # is NOT usable if the final filesystem recheck refuses the top level.
        _recheck(state)
        out['projection_result'] = projection.review_projection(blobs['parent_manifest'],
            blobs['projection_manifest'], c, blobs['raw'], blobs['observation'])
        _recheck(state)
        out['local_regular_file_reads_complete'] = True
        result = out['projection_result']
        out['stage'] = 'projection_reader_checked' if result['stage'] == 'projection_reference_checked' else 'projection_reader_reference_refused'
        out['error'] = result['error']
    except _Refusal as error:
        out.update(error=error.error, filesystem_error_role=error.role, os_errno=error.number)
    finally:
        for fd in reversed(state['fds']):
            try: os.close(fd)
            except OSError as error:
                # Never retry close: on Linux an interrupted close may already
                # have released the descriptor, which could then be reused.
                out.update(stage='projection_reader_refused', error='archive_close',
                           filesystem_error_role='cleanup', os_errno=error.errno,
                           local_regular_file_reads_complete=False)
    return out
