"""Closed artifact corpus: new Python reference contract only, no old execution."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
from tools import pending_archive_binding as b

ARCHIVE = 'reports/straight-followup-20261103-30'
PREVIOUS = 'reports/straight-followup-20261103-31'
MANIFEST = PREVIOUS + '/DELIVERY_MANIFEST.json'
SOURCE_FILES = ('tools/pending_archive_binding.py', 'tools/pending_archive_binding_cases.py',
                'docs/PENDING_ARCHIVE_BINDING_V1.json', 'docs/PENDING_ARCHIVE_BINDING_V1.md',
                'tests/test_pending_archive_binding.py')
PRESERVED = ('docs/PENDING_CPP_DIFFERENCES.json', 'docs/PRECOMPUTE_INTEGRATION_REVIEW.json',
             'reports/straight-followup-20261103-28/SUMMARY.json',
             *(f'reports/straight-followup-20261103-28/cpp-differential-audit/preserve--{n}/python.json'
               for n in ('unknown_result_native', 'late_reply_retains_unknown',
                         'edge--late_unupgraded_future_observation', 'edge--late_unupgraded_wrong_original_record')),
             *(f'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample/{n}.json'
               for n in ('input', 'result')))
sha = lambda raw: hashlib.sha256(raw).hexdigest()
encode = lambda value: json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def input_files(root):
    names = json.loads((Path(root) / ARCHIVE / 'PLAN.json').read_bytes())['cases']
    assert len(names) == 532
    assert all(n and all(c.isascii() and (c.isalnum() or c in '-_+.') for c in n) and n not in ('.', '..') for n in names)
    return tuple(dict.fromkeys((*SOURCE_FILES, MANIFEST, ARCHIVE + '/PLAN.json',
        PREVIOUS + '/SUMMARY.json', PREVIOUS + '/SAME_LABEL_EVIDENCE.json', *PRESERVED,
        *(f'{ARCHIVE}/boundary-audit/{name}/{part}' for name in names
          for part in ('input.bin', 'python.json', 'cpp.stdout.bin')))))


def claim_for(manifest, raw_ref, observation_ref):
    return dict(profile=b.PROFILE, manifest_ref=MANIFEST, manifest_sha256=sha(manifest),
                raw_ref=raw_ref, observation_ref=observation_ref, pairing_basis='caller_declared_only')


def corpus(root):
    root = Path(root)
    manifest = (root / MANIFEST).read_bytes()
    names = json.loads((root / ARCHIVE / 'PLAN.json').read_bytes())['cases']
    rows = []

    def add(name, m, claim, raw, obs, *, family='contract_boundary', error=None, source_case=None, origin=None):
        rows.append(dict(case=name, family=family, manifest_raw=m, claim=claim, raw=raw,
                         observation_raw=obs, expected_error=error, source_case=source_case, origin=origin))

    for name in names:
        raw_ref = f'{ARCHIVE}/boundary-audit/{name}/input.bin'
        raw = (root / raw_ref).read_bytes()
        for origin, filename in (('python', 'python.json'), ('cpp', 'cpp.stdout.bin')):
            obs_ref = f'{ARCHIVE}/boundary-audit/{name}/{filename}'
            add('archive--' + name + '--' + origin, manifest, claim_for(manifest, raw_ref, obs_ref),
                raw, (root / obs_ref).read_bytes(), family='sealed_archive', source_case=name, origin=origin)
    base = rows[0]
    raw, obs = base['raw'], base['observation_raw']
    raw_ref, obs_ref = 'artifacts/input.bin', 'artifacts/observation.bin'
    mini = dict(scope='caller-owned synthetic catalog, not authentic provenance',
                files={raw_ref: sha(raw), obs_ref: sha(obs)})
    m = encode(mini)
    c = claim_for(m, raw_ref, obs_ref)

    def variation(name, *, m2=m, c2=None, raw2=raw, obs2=obs, error=None, family='contract_boundary'):
        add(name, m2, copy.deepcopy(c if c2 is None else c2), raw2, obs2, error=error, family=family)

    variation('claim--baseline', family='caller_pairing')
    variation('claim--repeat', family='caller_pairing')
    # Both real files are sealed, but nothing establishes this cross-case pair.
    other = next(r for r in rows if r['source_case'] == 'priority--0--message_null' and r['origin'] == 'python')
    paired = dict(base['claim'], observation_ref=other['claim']['observation_ref'])
    variation('claim--unrelated_sealed_pair', m2=manifest, c2=paired, obs2=other['observation_raw'], family='caller_pairing')
    variation('mismatch--manifest_pin', c2=dict(c, manifest_sha256='0' * 64), error='manifest_digest_mismatch')
    variation('mismatch--raw', raw2=raw + b'!', error='raw_digest_mismatch')
    variation('mismatch--observation', obs2=obs + b' ', error='observation_digest_mismatch')
    variation('missing--raw', c2=dict(c, raw_ref='missing/raw.bin'), error='raw_reference_missing')
    variation('missing--observation', c2=dict(c, observation_ref='missing/obs.bin'), error='observation_reference_missing')
    pretty = json.dumps(mini, indent=2).encode()
    variation('manifest--whitespace_without_repin', m2=pretty, error='manifest_digest_mismatch')
    variation('manifest--whitespace_with_repin', m2=pretty, c2=claim_for(pretty, raw_ref, obs_ref), family='caller_pairing')
    variation('claim--manifest_ref_is_declarative', c2=dict(c, manifest_ref='not-opened/other-manifest.json'), family='caller_pairing')
    for name, value in [('absolute', '/etc/passwd'), ('parent', '../input.bin'), ('empty_component', 'a//b'),
                         ('dot_component', 'a/./b'), ('trailing_slash', 'a/'), ('backslash', 'a\\b'),
                         ('nul', 'a\0b'), ('unicode', '档案.bin'), ('overlong', 'a' * 513), ('empty', '')]:
        variation('path--' + name, c2=dict(c, raw_ref=value), error='claim_shape')
    for name, change in [('profile', dict(profile='old_profile')), ('pairing', dict(pairing_basis='authenticated')),
                          ('upper_digest', dict(manifest_sha256='A' * 64)), ('bool_digest', dict(manifest_sha256=True)),
                          ('same_roles', dict(observation_ref=raw_ref)), ('manifest_alias', dict(manifest_ref=raw_ref)),
                          ('extra_authority', dict(motion_authority=True))]:
        variation('shape--' + name, c2=dict(c, **change), error='claim_shape')
    missing = dict(c); del missing['pairing_basis']
    variation('shape--missing_key', c2=missing, error='claim_shape')
    variation('shape--non_dict', c2=[], error='claim_shape')
    malformed = {
        'root_list': b'[]', 'invalid_utf8': b'\xff', 'bom': b'\xef\xbb\xbf{}',
        'duplicate_root': b'{"scope":"a","scope":"b","files":{}}',
        'duplicate_entry': b'{"scope":"a","files":{"a":"x","a":"y"}}',
        'nonfinite': b'{"scope":NaN,"files":{}}', 'trailing': m + b'!',
        'deep': b'[' * 1500 + b'0' + b']' * 1500,
        'empty_files': encode(dict(scope='a', files={})),
        'unsafe_entry': encode(dict(scope='a', files={'../a': '0' * 64})),
        'bad_digest': encode(dict(scope='a', files={'a': 'G' * 64})),
        'scope_bound': encode(dict(scope='x' * 4097, files=mini['files'])),
        'entry_bound': encode(dict(scope='a', files={f'a/{i}': '0' * 64 for i in range(b.ENTRY_LIMIT + 1)})),
    }
    for name, blob in malformed.items():
        variation('manifest--' + name, m2=blob, c2=claim_for(blob, raw_ref, obs_ref), error='manifest_shape')
    for role, value in [('manifest', bytearray(m)), ('raw', bytearray(raw)), ('observation', bytearray(obs))]:
        variation('bytes--' + role, **{dict(manifest='m2', raw='raw2', observation='obs2')[role]: value}, error=role + '_bytes')
    for role, size in [('manifest', b.MANIFEST_LIMIT), ('raw', b.RAW_LIMIT), ('observation', b.OBSERVATION_LIMIT)]:
        variation('bound--' + role, **{dict(manifest='m2', raw='raw2', observation='obs2')[role]: b'x' * (size + 1)}, error=role + '_byte_bound')
    # A caller can repin an entire unrelated set, including non-JSON evidence.
    for name, newraw, newobs in [('joint_replacement', b'not a packet', b'not JSON'),
                                  ('empty_artifacts', b'', b''),
                                  ('at_artifact_caps', b'x' * b.RAW_LIMIT, b'y' * b.OBSERVATION_LIMIT)]:
        newm = encode(dict(scope='replacement', files={raw_ref: sha(newraw), obs_ref: sha(newobs)}))
        variation('claim--' + name, m2=newm, c2=claim_for(newm, raw_ref, obs_ref),
                  raw2=newraw, obs2=newobs, family='caller_pairing')
    return rows


def evaluate(row):
    args = (row['manifest_raw'], row['claim'], row['raw'], row['observation_raw'])
    return b.review_archive_binding(*args), b.review_archive_binding(*args)


def check(row, output, repeat):
    assert output == repeat
    assert output['error'] == row['expected_error'], (row['case'], output['error'], row['expected_error'])
    assert output['stage'] == ('archive_reference_checked' if row['expected_error'] is None else 'archive_reference_refused')
    assert output['file_references_agree'] is (row['expected_error'] is None)
    assert output['actual_exposure_wall_s'] is output['observed_validation_site'] is None
    assert all(output[k] is False for k in b.FALSE_FIELDS)
    if output['file_references_agree']:
        assert output['pairing_declared'] and output['pairing_basis'] == 'caller_declared_only'
        assert all(output[k] is True for k in ('manifest_digest_matches', 'raw_reference_digest_matches', 'observation_reference_digest_matches'))
    return True


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False); stream.write('\n')


def run(root, output, pinpath, pinhash):
    root, output = Path(root), Path(output)
    pinraw = Path(pinpath).read_bytes()
    if sha(pinraw) != pinhash:
        raise ValueError('pin_digest')
    pins = json.loads(pinraw)
    if set(pins) != set(input_files(root)) or any(sha((root / n).read_bytes()) != h for n, h in pins.items()):
        raise ValueError('input_pin_mismatch')
    output.mkdir(exist_ok=False)
    # Shared manifests are content-addressed; this is storage dedup, not trust.
    manifests = output / 'manifests'; manifests.mkdir()
    results = []
    for row in corpus(root):
        folder = output / row['case']; folder.mkdir()
        meta = {k: v for k, v in row.items() if k not in ('manifest_raw', 'claim', 'raw', 'observation_raw')}
        for key, name in [('manifest_raw', 'manifest'), ('raw', 'input'), ('observation_raw', 'observation')]:
            blob = row[key]
            # Non-exact bytes remain visibly typed in metadata, not silently
            # upgraded on replay. Closed analyzer never reruns the checker.
            meta[name + '_type'] = type(blob).__name__
            meta[name + '_sha256'] = sha(blob)
            dest = manifests / (sha(blob) + '.bin') if name == 'manifest' else folder / (name + '.bin')
            if not dest.exists():
                with dest.open('xb') as stream: stream.write(blob)
        write(folder / 'case.json', meta); write(folder / 'claim.json', row['claim'])
        result, repeat = evaluate(row)
        write(folder / 'reference.json', result); write(folder / 'reference-repeat.json', repeat)
        check(row, result, repeat)
        results.append(dict(case=row['case'], family=row['family'], error=result['error'],
                            output_sha256=sha((folder / 'reference.json').read_bytes())))
    assert all(sha((root / n).read_bytes()) == h for n, h in pins.items())
    summary = dict(profile=b.PROFILE, case_count=len(results), cases=results,
                   checked=sum(r['error'] is None for r in results), refused=sum(r['error'] is not None for r in results),
                   pure_contract_calls=2 * len(results), formal_runs=1, formal_retries=0,
                   old_protocol_validator_calls=0, old_diagnostic_calls=0, formal_compilations=0,
                   portable_cpp_executions=0, new_runtime_backend_experiments=0,
                   input_binding_authenticated=False, production_changes=False, movement=False)
    write(output / 'summary.json', summary)
    return {k: v for k, v in summary.items() if k != 'cases'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--input-pins', type=Path, required=True)
    parser.add_argument('--input-pins-sha256', required=True)
    args = parser.parse_args()
    print(json.dumps(run(Path(__file__).resolve().parents[1], args.output, args.input_pins, args.input_pins_sha256)))
