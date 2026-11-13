"""Linux opt-in bounded local reads; NOT an atomic or authenticated archive.

User path components use NOFOLLOW. O_PATH pins a leaf before type inspection;
only a pinned REGULAR inode is reopened via Linux /proc/<pid>/fd. That one
kernel descriptor magic-link dereference is intentional, never a user path.
"""
import hashlib
import os
import re
import stat
from tools import pending_archive_binding as binding

PROFILE = 'toponav_pending_archive_reader_v1'
ROOT_LIMIT = 4096
ROOT_DEPTH = 64
REFERENCE_DEPTH = 32
EXTRA_FALSE = ('atomic_multi_file_snapshot_observed', 'root_origin_authenticated',
               'hostile_os_resistance_proved', 'hard_realtime_read_bound_proved')


class _Refusal(Exception):
    def __init__(self, error, role=None, number=None):
        self.error, self.role, self.number = error, role, number


def _signature(s):
    return [s.st_dev, s.st_ino, s.st_mode, s.st_size, s.st_mtime_ns, s.st_ctime_ns, s.st_nlink]


def _identity(s):
    return (s.st_dev, s.st_ino, s.st_mode)


def _call(operation, role, fn, *args, **kwargs):
    try:
        return fn(*args, **kwargs)
    except OSError as error:
        raise _Refusal('archive_' + operation, role, error.errno) from None


def _open(name, flags, state, role, parent=None):
    fd = _call('open', role, os.open, name, flags, **({} if parent is None else {'dir_fd': parent}))
    state['fds'].append(fd)
    return fd


def _directory(parent, name, state, role):
    fd = _open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC | os.O_NONBLOCK,
               state, role, parent)
    state['links'].append((parent, name, fd, role))
    return fd


def _read_regular(parent, name, cap, role, state):
    # O_PATH does not invoke a FIFO/device's ordinary open operation. Refuse
    # nonregular inodes BEFORE a read-capable descriptor is obtained.
    pin = _open(name, os.O_PATH | os.O_NOFOLLOW | os.O_CLOEXEC, state, role, parent)
    before = _call('stat', role, os.fstat, pin)
    if not stat.S_ISREG(before.st_mode):
        raise _Refusal('archive_file_type', role)
    if not 0 <= before.st_size <= cap:
        raise _Refusal('archive_byte_bound', role)
    fd = _open(str(pin), os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC, state, role, state['proc_fd'])
    opened = _call('stat', role, os.fstat, fd)
    if _signature(before) != _signature(opened):
        raise _Refusal('archive_file_changed', role)
    state['links'].append((parent, name, pin, role))
    data = bytearray()
    while True:
        chunk = _call('read', role, os.read, fd, min(65536, cap + 1 - len(data)))
        if not chunk:
            break
        data.extend(chunk)
        if len(data) > cap:
            raise _Refusal('archive_byte_bound', role)
    after = _call('stat', role, os.fstat, fd)
    if _signature(before) != _signature(after) or len(data) != before.st_size:
        raise _Refusal('archive_file_changed', role)
    raw = bytes(data)
    fact = dict(role=role, byte_count=len(raw), sha256=hashlib.sha256(raw).hexdigest(),
                before_signature=_signature(before), after_signature=_signature(after),
                final_recheck_agreed=None, source='local_sequential_read_not_authenticated_origin')
    state['files'].append((fd, fact, _signature(before)))
    return raw, fact


def _recheck(state):
    for _, fact, _ in state['files']:
        fact['final_recheck_agreed'] = None
    for parent, name, fd, role in state['links']:
        try:
            entry = os.stat(name, dir_fd=parent, follow_symlinks=False)
        except OSError as error:
            raise _Refusal('archive_path_changed', role, error.errno) from None
        pinned = _call('stat', role, os.fstat, fd)
        if _identity(entry) != _identity(pinned):
            raise _Refusal('archive_path_changed', role)
    for fd, fact, signature in state['files']:
        final = _call('stat', fact['role'], os.fstat, fd)
        fact['final_recheck_agreed'] = _signature(final) == signature
        if not fact['final_recheck_agreed']:
            raise _Refusal('archive_file_changed', fact['role'])


def _claim_snapshot(claim):
    if type(claim) is not dict or len(claim) != 6:
        return None
    c = dict(claim)
    keys = {'profile', 'manifest_ref', 'manifest_sha256', 'raw_ref', 'observation_ref', 'pairing_basis'}
    if set(c) != keys or any(type(v) is not str for v in c.values()):
        return None
    if c['profile'] != binding.PROFILE or c['pairing_basis'] != 'caller_declared_only':
        return None
    if len(c['manifest_sha256']) != 64 or re.fullmatch('[0-9a-f]{64}', c['manifest_sha256']) is None:
        return None
    refs = [c[k] for k in ('manifest_ref', 'raw_ref', 'observation_ref')]
    for value in refs:
        if not 0 < len(value) <= binding.PATH_LIMIT or re.fullmatch('[A-Za-z0-9._+/-]+', value) is None:
            return None
        parts = value.split('/')
        if len(parts) > REFERENCE_DEPTH or any(p in ('', '.', '..') for p in parts):
            return None
    if len(set(refs)) != 3:
        return None
    return c


def read_archive_pair(root, claim):
    """Read a caller-selected local pair and apply the SEALED 2W pure contract.

    Linux+mounted /proc is required; no fallback. Stability is sampled before,
    after and at final recheck, NOT an atomic snapshot, a lease or provenance.
    Hard links, bind mounts, change-and-restore and hostile OS are NOT excluded.
    No descriptor or file bytes escape; every owned descriptor closes on exit.
    """
    out = dict(profile=PROFILE, stage='reader_refused', error=None, filesystem_error_role=None,
               os_errno=None, claim_snapshot=None, files=[], reference_result=None,
               local_regular_file_reads_complete=False, sequential_samples_only=True,
               observed_validation_site=None, actual_exposure_wall_s=None,
               **{k: False for k in (*binding.FALSE_FIELDS, *EXTRA_FALSE)})
    if (type(root) is not str or not 0 < len(root) <= ROOT_LIMIT or not root.startswith('/')
            or '\0' in root or any(0xD800 <= ord(c) <= 0xDFFF for c in root)
            or (root != '/' and any(p in ('', '.', '..') for p in root.split('/')[1:]))
            or len(root.split('/')[1:]) > ROOT_DEPTH):
        out['error'] = 'reader_root_shape'
        return out
    c = _claim_snapshot(claim)
    if c is None:
        out['error'] = 'reader_claim_shape'
        return out
    out['claim_snapshot'] = c
    if (os.name != 'posix' or any(not hasattr(os, k) for k in ('O_PATH', 'O_NOFOLLOW', 'O_DIRECTORY', 'O_CLOEXEC', 'O_NONBLOCK'))
            or os.open not in os.supports_dir_fd or os.stat not in os.supports_dir_fd
            or os.stat not in os.supports_follow_symlinks):
        out['error'] = 'reader_platform_unsupported'
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
        for role, cap in (('manifest', binding.MANIFEST_LIMIT), ('raw', binding.RAW_LIMIT),
                          ('observation', binding.OBSERVATION_LIMIT)):
            ref = c[role + '_ref']; parts = ref.split('/'); parent = root_fd
            for part in parts[:-1]: parent = _directory(parent, part, state, role)
            blob, fact = _read_regular(parent, parts[-1], cap, role, state)
            fact['reference'] = ref; out['files'].append(fact); blobs[role] = blob
        # Check both before and after the pure review. A positive nested result
        # is NOT usable if the final filesystem recheck refuses the top level.
        _recheck(state)
        out['reference_result'] = binding.review_archive_binding(blobs['manifest'], c, blobs['raw'], blobs['observation'])
        _recheck(state)
        out['local_regular_file_reads_complete'] = True
        result = out['reference_result']
        out['stage'] = 'reader_reference_checked' if result['file_references_agree'] else 'reader_reference_refused'
        out['error'] = result['error']
    except _Refusal as error:
        out.update(error=error.error, filesystem_error_role=error.role, os_errno=error.number)
    finally:
        for fd in reversed(state['fds']):
            try: os.close(fd)
            except OSError as error:
                # Never retry close: on Linux an interrupted close may already
                # have released the descriptor, which could then be reused.
                out.update(stage='reader_refused', error='archive_close',
                           filesystem_error_role='cleanup', os_errno=error.errno,
                           local_regular_file_reads_complete=False)
    return out
