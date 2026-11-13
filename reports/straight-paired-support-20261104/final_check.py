"""Read-only final restoration, immutable measured bytes and test-scope audit."""
import ast
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent


def read(path):return json.loads(path.read_bytes())

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    previous=read(HERE/'BEFORE_PINS.json')['files']
    assert all(sha(ROOT/p)==expected for p,expected in previous.items())
    replacements={p:HERE/'candidate'/p for p in (
        'roadvision/paired_descriptor_checks.py','roadvision/native_patch_contrast.py',
        'roadvision/prepared_support_mask.py','tests/test_prepared_support_mask.py')}
    measured=read(HERE/'FREEZE.json')['files']
    assert all(sha(replacements.get(p,ROOT/p))==expected for p,expected in measured.items())
    assert (HERE/'TIMING_ANALYSIS.json').read_bytes()==(HERE/'TIMING_ANALYSIS_REPRODUCED.json').read_bytes()
    gate=read(HERE/'benchmark/SUMMARY.json')
    assert gate['complete_controller_performance_gate_passed'] is False and gate['all_outputs_exact']
    assert gate['measured_decisions']==1560 and gate['warmup_decisions']==390
    assert not (HERE/'REPLAY.json').exists() and not (HERE/'replay-progress.jsonl').exists()
    preflight=read(HERE/'PREFLIGHT.json')
    for case in preflight['cases']:
        old,new=case['variants']['before_explicit'],case['variants']['after_explicit']
        for field in ('matches','support_samples','h_ncc','i_ncc','support_sha256'):
            assert old[field]==new[field]
        assert old['prepared_requests']==0 and new['conversions']<=2
    def tests(path):
        cases=list(ET.parse(path).iter('testcase'))
        failed=[c for c in cases if c.find('failure') is not None or c.find('error') is not None]
        skipped=[c for c in cases if c.find('skipped') is not None]
        return dict(total=len(cases),passed=len(cases)-len(failed)-len(skipped),failed=len(failed),skipped=len(skipped))
    initial=tests(HERE/'targeted.xml');final=tests(HERE/'final-targeted.xml');expanded=tests(HERE/'final-expanded.xml')
    assert initial==dict(total=286,passed=286,failed=0,skipped=0)
    assert final==dict(total=287,passed=287,failed=0,skipped=0)
    assert expanded==dict(total=1384,passed=1351,failed=33,skipped=0)
    assert read(HERE/'FAILURE_COMPARISON.json')['failed_nodes_and_messages_unchanged']
    def functions(path):
        return {n.name:ast.dump(n,include_attributes=False) for n in ast.parse(path.read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name.startswith('test_')}
    original=functions(HERE/'candidate/tests/test_prepared_support_mask.py')
    current=functions(ROOT/'tests/test_prepared_support_mask.py')
    assert all(current[name]==body for name,body in original.items())
    assert set(current)-set(original)=={'test_default_pair_and_contrast_remain_original_bytes'}
    for p in ('roadvision/paired_descriptor_checks.py','roadvision/native_patch_contrast.py'):
        assert (ROOT/p).read_bytes()==(HERE/'before'/p).read_bytes()
        assert 'prepared_support_mask' not in (ROOT/p).read_text()
    full=read(ROOT/'reports/straight-paired-support-full-current-20261104/summary.json')
    assert full['verification_complete'] is False and full['exit_code']==2 and full['groups']['current']['selected']==0
    result=dict(candidate_promoted=False,scope='Rejected region-local I support-mask preparation',
        preexisting_files_byte_identical=len(previous),measured_freeze_verified_via_snapshots=len(measured),
        restored_production_files=2,default_does_not_import_new_helper=True,
        independent_experimental_helper_retained=True,measured_test_function_asts_unchanged=True,
        final_tests_explicitly_bind_archived_candidate=True,added_default_original_bytes_assertion=True,
        initial_targeted=initial,final_targeted=final,expanded=expanded,old_failures_unchanged=True,
        full_current_exit=2,full_current_selected=0,analysis_reproduced_byte_identical=True,
        measured_decisions=1560,warmup_decisions=390,all_samples_retained=True,all_serialized_outputs_exact=True,
        performance_gate=False,conditional_replay_executed=False,verification_complete=False,
        physical_execution=False,real_time_budget_proved=False)
    with (HERE/'FINAL_VERIFICATION.json').open('x') as f:f.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
