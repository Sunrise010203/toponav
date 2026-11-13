"""Closed four-file audit. All mutation fixtures live in TemporaryDirectory."""
import argparse
import contextlib
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
from unittest.mock import patch
from tools import pending_archive_projection_reader as reader

PREVIOUS = 'reports/straight-followup-20261103-34'
ARCHIVE = 'reports/straight-followup-20261103-30'
PARENT = 'reports/straight-followup-20261103-32/DELIVERY_MANIFEST.json'
SOURCE_FILES = ('tools/pending_archive_projection_reader.py', 'tools/pending_archive_projection_reader_cases.py',
    'docs/PENDING_ARCHIVE_PROJECTION_READER_V1.json', 'docs/PENDING_ARCHIVE_PROJECTION_READER_V1.md',
    'tests/test_pending_archive_projection_reader.py')
ROLES = ('parent_manifest', 'projection_manifest', 'raw', 'observation')
CAPS = dict(zip(ROLES, (8388608, 8192, 36893, 65536)))
sha = lambda b: hashlib.sha256(b).hexdigest()
encode = lambda v: json.dumps(v, sort_keys=True, separators=(',', ':')).encode()


def input_files(root):
    root = Path(root); names = json.loads((root / ARCHIVE / 'PLAN.json').read_bytes())['cases']
    assert len(names) == 532 and all(n and all(c.isascii() and (c.isalnum() or c in '-_+.') for c in n) for n in names)
    refs = [*SOURCE_FILES, 'tools/pending_archive_reader.py', 'tools/pending_archive_projection.py',
            'tools/pending_archive_binding.py', ARCHIVE + '/PLAN.json', PARENT,
            *(PREVIOUS + '/' + n for n in ('DELIVERY_MANIFEST.json', 'SUMMARY.json', 'PROJECTION_ANALYSIS.json'))]
    for name in names:
        refs.extend(f'{ARCHIVE}/boundary-audit/{name}/{p}' for p in ('input.bin', 'python.json', 'cpp.stdout.bin'))
        for origin in ('python', 'cpp'):
            ref = f'{PREVIOUS}/archive-projection-audit/archive--{name}--{origin}/claim.json'
            claim = json.loads((root / ref).read_bytes()); refs.append(ref)
            refs.append(f"{PREVIOUS}/archive-projection-audit/blobs/{claim['projection_sha256']}.bin")
    return tuple(dict.fromkeys(refs))


def corpus(root):
    names = json.loads((Path(root) / ARCHIVE / 'PLAN.json').read_bytes())['cases']
    rows = [dict(case='archive--' + name + '--' + origin, family='sealed_four_files', source_case=name,
        origin=origin, action='archive', role=None, expected_error=None, expected_stage='projection_reader_checked')
        for name in names for origin in ('python', 'cpp')]
    def add(action, error=None, role=None, reference=False):
        rows.append(dict(case='fixture--' + action + ('--' + role if role else ''), family='temporary_filesystem',
            source_case=None, origin=None, action=action, role=role, expected_error=error,
            expected_stage='projection_reader_checked' if error is None else
                'projection_reader_reference_refused' if reference else 'projection_reader_refused'))
    for role in ROLES:
        for a, e in [('missing','archive_open'),('symlink','archive_file_type'),('fifo','archive_file_type'),
                     ('directory_leaf','archive_file_type'),('oversize','archive_byte_bound'),
                     ('inplace_after','archive_file_changed'),('replace_after','archive_path_changed'),
                     ('unlink_after','archive_path_changed'),('mutate_read','archive_file_changed'),
                     ('replace_before_review','archive_path_changed')]: add(a,e,role)
    for a in ('regular','empty','artifact_caps','parent_cap','projection_cap','hardlink_pair',
              'joint_repin','unrelated_sealed_pair','transient_ancestor_restore'): add(a)
    for a in ('ancestor_replace','root_replace','symlink_directory','symlink_root','symlink_root_ancestor'): add(a,'archive_path_changed' if a in ('ancestor_replace','root_replace') else 'archive_open')
    for a, e in [('bad_parent_pin','parent_digest_mismatch'),('bad_projection_pin','projection_digest_mismatch'),
                 ('raw_digest_mismatch','projection_reference_refused')]: add(a,e,reference=True)
    add('deep_reference','projection_reader_claim_shape');add('relative_root','projection_reader_root_shape')
    return rows


def fixture(base, action, role=None, project_root=None):
    root = base / 'real' / 'root'; (root / 'catalog').mkdir(parents=True); (root / 'data').mkdir()
    refs = dict(parent_manifest='catalog/parent.json', projection_manifest='catalog/minimal.json',
                raw='data/input.bin', observation='data/observation.bin')
    raw, obs = b'opaque raw', b'opaque invalid JSON observation'
    if action == 'empty': raw = obs = b''
    if action == 'artifact_caps': raw, obs = b'x' * CAPS['raw'], b'y' * CAPS['observation']
    if action == 'hardlink_pair': raw = obs = b'same inode does not authenticate pairing'
    if action == 'joint_repin': raw, obs = b'whole replacement', b'unrelated replacement'
    if action == 'unrelated_sealed_pair':
        raw = (Path(project_root) / ARCHIVE / 'boundary-audit/baseline--original--reserve/input.bin').read_bytes()
        obs = (Path(project_root) / ARCHIVE / 'boundary-audit/priority--0--message_null/python.json').read_bytes()
    parent = encode(dict(scope='caller owned, unauthenticated',files={refs['raw']:sha(raw),refs['observation']:sha(obs)}))
    proj = encode(dict(scope=reader.projection.PROJECTION_SCOPE,files=json.loads(parent)['files']))
    if action == 'parent_cap': parent += b' ' * (CAPS['parent_manifest'] - len(parent))
    if action == 'projection_cap': proj += b' ' * (CAPS['projection_manifest'] - len(proj))
    blobs = dict(parent_manifest=parent, projection_manifest=proj, raw=raw, observation=obs)
    claim = dict(profile=reader.projection.PROFILE, pairing_basis='caller_declared_only',
        parent_manifest_sha256=sha(parent), projection_sha256=sha(proj), **{k+'_ref':v for k,v in refs.items()})
    for k, b in blobs.items(): (root / refs[k]).write_bytes(b)
    requested = str(root)
    if action in ('missing','symlink','fifo','directory_leaf','oversize'):
        path = root / refs[role];path.unlink()
        if action == 'symlink':
            target = base / 'outside';target.write_bytes(blobs[role]);path.symlink_to(target)
        elif action == 'fifo': os.mkfifo(path)
        elif action == 'directory_leaf': path.mkdir()
        elif action == 'oversize':
            with path.open('wb') as f: f.truncate(CAPS[role]+1)
    if action == 'hardlink_pair':
        (root / refs['observation']).unlink();os.link(root / refs['raw'],root / refs['observation'])
    if action == 'symlink_directory':
        (root / 'catalog').rename(root / 'held');(root / 'catalog').symlink_to(root / 'held',target_is_directory=True)
    if action in ('symlink_root','symlink_root_ancestor'):
        (base / 'alias').symlink_to(root if action == 'symlink_root' else root.parent,target_is_directory=True)
        requested = str(base / 'alias') if action == 'symlink_root' else str(base / 'alias' / 'root')
    if action == 'bad_parent_pin': claim['parent_manifest_sha256']='0'*64
    if action == 'bad_projection_pin': claim['projection_sha256']='0'*64
    if action == 'raw_digest_mismatch': (root / refs['raw']).write_bytes(raw+b'!')
    if action == 'deep_reference': claim['raw_ref']='a/'*32+'raw'
    if action == 'relative_root': requested='relative/root'
    return root, requested, refs, blobs, claim


def evaluate_once(project_root, row):
    if row['family'] == 'sealed_four_files':
        folder = Path(project_root) / PREVIOUS / 'archive-projection-audit' / row['case']
        claim = json.loads((folder / 'claim.json').read_bytes())
        # 2Y's synthetic derivative name is NOT a physical path. Rebind only
        # this name to the existing sealed content-addressed projection blob.
        claim['projection_manifest_ref'] = f"{PREVIOUS}/archive-projection-audit/blobs/{claim['projection_sha256']}.bin"
        return dict(result=reader.read_projection_archive(str(project_root),claim),claim=claim,events=[],initial_sha256=None)
    with tempfile.TemporaryDirectory(prefix='toponav-2z-fixture-') as temp:
        root, requested, refs, blobs, claim = fixture(Path(temp),row['action'],row['role'],project_root)
        events=[];action=row['action'];role=row['role'];review=reader.projection.review_projection
        def interleave():
            if action in ('inplace_after','replace_after','unlink_after','replace_before_review'):
                p=root/refs[role]
                if action=='inplace_after':
                    old=p.stat();p.write_bytes(b'!'+p.read_bytes()[1:]);os.utime(p,ns=(old.st_atime_ns,old.st_mtime_ns+10000000))
                elif action=='unlink_after':p.unlink()
                else:
                    fresh=root/'new';fresh.write_bytes(p.read_bytes());fresh.replace(p)
            elif action in ('ancestor_replace','transient_ancestor_restore'):
                (root/'catalog').rename(root/'held');(root/'catalog').mkdir();(root/'catalog'/'parent.json').write_bytes(b'other')
                if action=='transient_ancestor_restore':shutil.rmtree(root/'catalog');(root/'held').rename(root/'catalog')
            elif action=='root_replace':root.rename(root.parent/'held_root');root.mkdir()
            else:return
            events.append(dict(action=action,role=role,phase='before_first_recheck' if action=='replace_before_review' else 'after_review',fixture_only=True))
        def after_review(*args):
            result=review(*args);interleave();return result
        read_regular=reader._read_regular
        def after_read(parent,name,cap,r,state):
            result=read_regular(parent,name,cap,r,state)
            if action=='replace_before_review' and r=='observation':interleave()
            return result
        read=os.read;inode=(root/refs[role]).stat().st_ino if action=='mutate_read' else None
        def during_read(fd,count):
            chunk=read(fd,count)
            if chunk and not events and os.fstat(fd).st_ino==inode:
                p=root/refs[role];old=p.stat();p.write_bytes(b'!'+p.read_bytes()[1:]);os.utime(p,ns=(old.st_atime_ns,old.st_mtime_ns+10000000))
                events.append(dict(action=action,role=role,phase='during_read',fixture_only=True))
            return chunk
        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(reader.projection,'review_projection',after_review))
            stack.enter_context(patch.object(reader,'_read_regular',after_read))
            if inode is not None:stack.enter_context(patch.object(reader.os,'read',during_read))
            result=reader.read_projection_archive(requested,claim)
        return dict(result=result,claim=claim,events=events,initial_sha256={k:sha(v) for k,v in blobs.items()})


def normalized(record):
    value=json.loads(json.dumps(record))
    for f in value['result']['files']:
        f.pop('before_signature');f.pop('after_signature')
    return value


def check(row,a,b):
    out=a['result'];assert (out['stage'],out['error'])==(row['expected_stage'],row['expected_error']),row['case']
    assert normalized(a)==normalized(b),row['case']
    if row['family']=='sealed_four_files':assert a==b
    assert all(out[k] is False for k in (*reader.binding.FALSE_FIELDS,*reader.EXTRA_FALSE))
    assert out['observed_validation_site'] is out['actual_exposure_wall_s'] is None
    if out['local_regular_file_reads_complete']:
        assert [f['role'] for f in out['files']]==list(ROLES)
        assert all(f['final_recheck_agreed'] is True for f in out['files'])
    assert all(f['before_signature']==f['after_signature'] for f in out['files'])
    return True


def write(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')


def run(root,output,pinpath,pinhash):
    root,output=Path(root),Path(output);pinraw=Path(pinpath).read_bytes();pins=json.loads(pinraw)
    if sha(pinraw)!=pinhash:raise ValueError('pin_digest')
    if set(pins)!=set(input_files(root)) or any(sha((root/n).read_bytes())!=h for n,h in pins.items()):raise ValueError('input_pin_mismatch')
    output.mkdir(exist_ok=False);rows=[];calls=dict(projection=0,binding=0)
    p=reader.projection.review_projection;b=reader.binding.review_archive_binding
    def counted_p(*args):calls['projection']+=1;return p(*args)
    def counted_b(*args):calls['binding']+=1;return b(*args)
    with patch.object(reader.projection,'review_projection',counted_p),patch.object(reader.binding,'review_archive_binding',counted_b):
        for row in corpus(root):
            folder=output/row['case'];folder.mkdir();a=evaluate_once(root,row);b2=evaluate_once(root,row);check(row,a,b2)
            write(folder/'case.json',row);write(folder/'read.json',a);write(folder/'read-repeat.json',b2)
            rows.append(dict(case=row['case'],family=row['family'],stage=a['result']['stage'],error=a['result']['error'],output_sha256=sha((folder/'read.json').read_bytes())))
    assert all(sha((root/n).read_bytes())==h for n,h in pins.items())
    summary=dict(case_count=len(rows),cases=rows,reader_calls=2*len(rows),projection_calls=calls['projection'],
        sealed_2W_pure_contract_calls=calls['binding'],formal_runs=1,formal_retries=0,old_2X_pair_reader_calls=0,
        sealed_2X_descriptor_helpers_reused=True,old_2V_diagnostic_calls=0,old_protocol_validator_calls=0,
        formal_compilations=0,portable_cpp_executions=0,new_runtime_backend_experiments=0,
        production_changes=False,native_transport=False,movement=False)
    write(output/'summary.json',summary);return {k:v for k,v in summary.items() if k!='cases'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--input-pins',type=Path,required=True);parser.add_argument('--input-pins-sha256',required=True);a=parser.parse_args()
    print(json.dumps(run(Path(__file__).resolve().parents[1],a.output,a.input_pins,a.input_pins_sha256)))
