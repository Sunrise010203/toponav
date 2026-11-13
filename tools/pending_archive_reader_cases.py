"""Closed archive + TEMPORARY filesystem fixtures, never production writes."""
import argparse
import contextlib
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
from unittest.mock import patch
from tools import pending_archive_reader as reader

ARCHIVE = 'reports/straight-followup-20261103-30'
PREVIOUS = 'reports/straight-followup-20261103-32'
MANIFEST = 'reports/straight-followup-20261103-31/DELIVERY_MANIFEST.json'
OVERSIZED_CATALOG = PREVIOUS + '/DELIVERY_MANIFEST.json'
SOURCE_FILES = ('tools/pending_archive_reader.py', 'tools/pending_archive_reader_cases.py',
                'docs/PENDING_ARCHIVE_READER_V1.json', 'docs/PENDING_ARCHIVE_READER_V1.md',
                'tests/test_pending_archive_reader.py')
PRESERVED = ('reports/straight-followup-20261103-31/SAME_LABEL_EVIDENCE.json',
             'docs/PENDING_CPP_DIFFERENCES.json', 'docs/PRECOMPUTE_INTEGRATION_REVIEW.json',
             'reports/straight-followup-20261103-28/SUMMARY.json',
             *(f'reports/straight-followup-20261103-28/cpp-differential-audit/preserve--{n}/python.json'
               for n in ('unknown_result_native', 'late_reply_retains_unknown',
                         'edge--late_unupgraded_future_observation', 'edge--late_unupgraded_wrong_original_record')),
             *(f'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample/{n}.json'
               for n in ('input', 'result')))
sha = lambda blob: hashlib.sha256(blob).hexdigest()
encode = lambda value: json.dumps(value, sort_keys=True, separators=(',', ':')).encode()


def input_files(root):
    names = json.loads((Path(root) / ARCHIVE / 'PLAN.json').read_bytes())['cases']
    assert len(names) == 532
    assert all(n and all(c.isascii() and (c.isalnum() or c in '-_+.') for c in n) and n not in ('.', '..') for n in names)
    return tuple(dict.fromkeys((*SOURCE_FILES, MANIFEST, OVERSIZED_CATALOG, PREVIOUS + '/SUMMARY.json', ARCHIVE + '/PLAN.json',
        'tools/pending_archive_binding.py', *PRESERVED,
        *(f'{ARCHIVE}/boundary-audit/{n}/{part}' for n in names for part in ('input.bin', 'python.json', 'cpp.stdout.bin')))))


def corpus(root):
    names = json.loads((Path(root) / ARCHIVE / 'PLAN.json').read_bytes())['cases']
    rows = [dict(case='archive--' + name + '--' + origin, family='sealed_archive', source_case=name,
                 origin=origin, action=None, expected_error=None, expected_stage='reader_reference_checked')
            for name in names for origin in ('python', 'cpp')]
    rows.append(dict(case='archive-catalog--sealed_2W_over_entry_cap', family='sealed_catalog_refusal',
                     source_case='baseline--original--reserve', origin='python', action=None,
                     expected_error='manifest_shape', expected_stage='reader_reference_refused'))
    actions = {
        'regular': None, 'empty': None, 'artifact_caps': None, 'manifest_cap': None, 'hardlink_pair': None,
        'bad_pin': 'manifest_digest_mismatch', 'raw_digest_mismatch': 'raw_digest_mismatch',
        'missing_manifest': 'archive_open', 'missing_raw': 'archive_open', 'missing_observation': 'archive_open',
        'symlink_manifest': 'archive_file_type', 'symlink_raw': 'archive_file_type', 'symlink_observation': 'archive_file_type',
        'symlink_directory': 'archive_open', 'symlink_root': 'archive_open', 'symlink_root_ancestor': 'archive_open',
        'fifo': 'archive_file_type', 'directory_leaf': 'archive_file_type',
        'oversize_manifest': 'archive_byte_bound', 'oversize_raw': 'archive_byte_bound', 'oversize_observation': 'archive_byte_bound',
        'deep_reference': 'reader_claim_shape', 'relative_root': 'reader_root_shape',
        'raw_inplace': 'archive_file_changed', 'raw_replace': 'archive_path_changed', 'raw_unlink': 'archive_path_changed',
        'ancestor_replace': 'archive_path_changed', 'root_replace': 'archive_path_changed',
        'symlink_replace': 'archive_path_changed', 'during_read_mutation': 'archive_file_changed',
        'transient_ancestor_restore': None,
    }
    for action, error in actions.items():
        stage = ('reader_reference_checked' if error is None else
                 'reader_reference_refused' if error in ('manifest_digest_mismatch', 'raw_digest_mismatch') else 'reader_refused')
        rows.append(dict(case='fixture--' + action, family='temporary_filesystem', source_case=None,
                         origin=None, action=action, expected_error=error, expected_stage=stage))
    return rows


def _claim(m, raw_ref, obs_ref, manifest_ref):
    return dict(profile=reader.binding.PROFILE, manifest_ref=manifest_ref, manifest_sha256=sha(m),
                raw_ref=raw_ref, observation_ref=obs_ref, pairing_basis='caller_declared_only')


def _fixture(base, action):
    root = base / 'real' / 'root'; (root / 'data').mkdir(parents=True); (root / 'catalog').mkdir()
    raw, obs = b'opaque raw fixture', b'opaque observation fixture'
    if action == 'empty': raw = obs = b''
    if action == 'artifact_caps': raw, obs = b'x' * reader.binding.RAW_LIMIT, b'y' * reader.binding.OBSERVATION_LIMIT
    if action == 'hardlink_pair': raw = obs = b'same inode is not authentic pairing'
    refs = dict(raw='data/input.bin', observation='data/observation.bin', manifest='catalog/manifest.json')
    m = encode(dict(scope='TEMP fixture only, not an authentic archive',
                    files={refs['raw']: sha(raw), refs['observation']: sha(obs)}))
    if action == 'manifest_cap': m += b' ' * (reader.binding.MANIFEST_LIMIT - len(m))
    initial = dict(manifest=m, raw=raw, observation=obs)
    for role, blob in initial.items(): (root / refs[role]).write_bytes(blob)
    claim = _claim(m, refs['raw'], refs['observation'], refs['manifest'])
    requested = root
    if action.startswith('missing_'): (root / refs[action[8:]]).unlink()
    if action.startswith('symlink_') and action[8:] in refs:
        role = action[8:]; target = base / 'outside.bin'; target.write_bytes(initial[role])
        p = root / refs[role]; p.unlink(); p.symlink_to(target)
    if action == 'symlink_directory':
        (root / 'data').rename(root / 'actual'); (root / 'data').symlink_to(root / 'actual', target_is_directory=True)
    if action == 'symlink_root': requested = base / 'alias'; requested.symlink_to(root, target_is_directory=True)
    if action == 'symlink_root_ancestor':
        (base / 'alias').symlink_to(base / 'real', target_is_directory=True); requested = base / 'alias' / 'root'
    if action in ('fifo', 'directory_leaf'):
        p = root / refs['raw']; p.unlink()
        if action == 'fifo': os.mkfifo(p)
        else: p.mkdir()
    if action.startswith('oversize_'):
        role = action[9:]; cap = dict(manifest=reader.binding.MANIFEST_LIMIT, raw=reader.binding.RAW_LIMIT,
                                     observation=reader.binding.OBSERVATION_LIMIT)[role]
        with (root / refs[role]).open('wb') as f: f.truncate(cap + 1)
    if action == 'hardlink_pair':
        (root / refs['observation']).unlink(); os.link(root / refs['raw'], root / refs['observation'])
    if action == 'bad_pin': claim['manifest_sha256'] = '0' * 64
    if action == 'raw_digest_mismatch': (root / refs['raw']).write_bytes(raw + b'!')
    if action == 'deep_reference': claim['raw_ref'] = 'a/' * reader.REFERENCE_DEPTH + 'raw'
    if action == 'relative_root': requested = Path('relative/root')
    return root, str(requested), refs, initial, claim


def _interleave(action, root, refs, events):
    raw = root / refs['raw']
    if action == 'raw_inplace':
        old = raw.stat(); raw.write_bytes(b'!' + raw.read_bytes()[1:]); os.utime(raw, ns=(old.st_atime_ns, old.st_mtime_ns + 10000000))
    elif action == 'raw_replace':
        replacement = root / 'new.bin'; replacement.write_bytes(raw.read_bytes()); replacement.replace(raw)
    elif action == 'raw_unlink': raw.unlink()
    elif action == 'symlink_replace':
        target = root / 'outside.bin'; target.write_bytes(raw.read_bytes()); raw.unlink(); raw.symlink_to(target)
    elif action in ('ancestor_replace', 'transient_ancestor_restore'):
        held = root / 'held'; (root / 'data').rename(held); (root / 'data').mkdir()
        (root / 'data' / 'input.bin').write_bytes(b'different intermediate bytes')
        if action == 'transient_ancestor_restore': shutil.rmtree(root / 'data'); held.rename(root / 'data')
    elif action == 'root_replace':
        root.rename(root.parent / 'held_root'); root.mkdir()
    else: return
    events.append(dict(action=action, phase='after_pure_review_before_final_recheck', fixture_only=True))


def evaluate_once(root, row):
    if row['family'] != 'temporary_filesystem':
        name = row['source_case']; raw_ref = f'{ARCHIVE}/boundary-audit/{name}/input.bin'
        obs_ref = f'{ARCHIVE}/boundary-audit/{name}/' + ('python.json' if row['origin'] == 'python' else 'cpp.stdout.bin')
        manifest_ref = OVERSIZED_CATALOG if row['family'] == 'sealed_catalog_refusal' else MANIFEST
        m = (Path(root) / manifest_ref).read_bytes(); claim = _claim(m, raw_ref, obs_ref, manifest_ref)
        result = reader.read_archive_pair(str(Path(root)), claim)
        return dict(result=result, events=[], claim=claim, initial_sha256=None)
    with tempfile.TemporaryDirectory(prefix='toponav-2x-fixture-') as temp:
        fixture, requested, refs, initial, claim = _fixture(Path(temp), row['action']); events = []
        review = reader.binding.review_archive_binding
        def after_review(*args):
            result = review(*args); _interleave(row['action'], fixture, refs, events); return result
        original_read = os.read
        inode = (fixture / refs['raw']).stat().st_ino if row['action'] == 'during_read_mutation' else None
        def during_read(fd, count):
            chunk = original_read(fd, count)
            if chunk and not events and os.fstat(fd).st_ino == inode:
                p = fixture / refs['raw']; old = p.stat(); p.write_bytes(b'!' + p.read_bytes()[1:])
                os.utime(p, ns=(old.st_atime_ns, old.st_mtime_ns + 10000000))
                events.append(dict(action=row['action'], phase='during_raw_read', fixture_only=True))
            return chunk
        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(reader.binding, 'review_archive_binding', after_review))
            if inode is not None: stack.enter_context(patch.object(reader.os, 'read', during_read))
            result = reader.read_archive_pair(requested, claim)
        return dict(result=result, events=events, claim=claim,
                    initial_sha256={k: sha(v) for k, v in initial.items()})


def projection(record):
    # Fresh temporary fixtures have distinct inode/time identities. Only those
    # seven sampled metadata values are excluded, never flags/hash/stage/errors.
    result = json.loads(json.dumps(record))
    for f in result['result']['files']:
        f.pop('before_signature'); f.pop('after_signature')
    return result


def check(row, first, repeat):
    result = first['result']
    assert (result['stage'], result['error']) == (row['expected_stage'], row['expected_error']), row['case']
    assert projection(first) == projection(repeat), row['case']
    if row['family'] == 'sealed_archive': assert first == repeat
    assert all(result[k] is False for k in (*reader.binding.FALSE_FIELDS, *reader.EXTRA_FALSE))
    assert result['observed_validation_site'] is result['actual_exposure_wall_s'] is None
    if result['local_regular_file_reads_complete']:
        assert len(result['files']) == 3 and all(f['final_recheck_agreed'] is True for f in result['files'])
    for f in result['files']: assert f['before_signature'] == f['after_signature']
    return True


def write(path, value):
    with Path(path).open('x') as f: json.dump(value, f, indent=2, allow_nan=False); f.write('\n')


def run(root, output, pinpath, pinhash):
    root, output = Path(root), Path(output); pinraw = Path(pinpath).read_bytes()
    if sha(pinraw) != pinhash: raise ValueError('pin_digest')
    pins = json.loads(pinraw)
    if set(pins) != set(input_files(root)) or any(sha((root / n).read_bytes()) != h for n, h in pins.items()):
        raise ValueError('input_pin_mismatch')
    output.mkdir(exist_ok=False); results = []; contract_calls = 0
    original = reader.binding.review_archive_binding
    def counted(*args):
        nonlocal contract_calls
        contract_calls += 1; return original(*args)
    with patch.object(reader.binding, 'review_archive_binding', counted):
        for row in corpus(root):
            folder = output / row['case']; folder.mkdir()
            first = evaluate_once(root, row); repeat = evaluate_once(root, row); check(row, first, repeat)
            write(folder / 'case.json', row); write(folder / 'read.json', first); write(folder / 'read-repeat.json', repeat)
            results.append(dict(case=row['case'], family=row['family'], stage=first['result']['stage'],
                                error=first['result']['error'], output_sha256=sha((folder / 'read.json').read_bytes())))
    assert all(sha((root / n).read_bytes()) == h for n, h in pins.items())
    summary = dict(case_count=len(results), cases=results, reader_calls=2 * len(results),
        sealed_2W_pure_contract_calls=contract_calls, formal_runs=1, formal_retries=0,
        old_protocol_validator_calls=0, old_2V_diagnostic_calls=0, formal_compilations=0,
        portable_cpp_executions=0, new_runtime_backend_experiments=0,
        temporary_fixture_interleavings=sum(row['family']=='temporary_filesystem' and bool(row['action']) and row['action'] in
            ('raw_inplace','raw_replace','raw_unlink','ancestor_replace','root_replace','symlink_replace','during_read_mutation','transient_ancestor_restore') for row in corpus(root)),
        production_changes=False, native_transport=False, movement=False)
    write(output / 'summary.json', summary)
    return {k: v for k, v in summary.items() if k != 'cases'}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True); p.add_argument('--input-pins', type=Path, required=True)
    p.add_argument('--input-pins-sha256', required=True); args = p.parse_args()
    print(json.dumps(run(Path(__file__).resolve().parents[1], args.output, args.input_pins, args.input_pins_sha256)))
