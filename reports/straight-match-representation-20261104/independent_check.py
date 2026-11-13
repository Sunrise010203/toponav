"""Arithmetic/provenance audit of saved maps and rows; no match reexecution."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_bytes())


def sha(a):
    return hashlib.sha256(a.tobytes()).hexdigest()


def main():
    pins = {}
    for name in ('PREEXISTING_PINS.json','FREEZE.json','FALLBACK_FREEZE.json','ANALYSIS_INPUT_PINS.json'):
        files = read(HERE/name)['files']
        for p, expected in files.items():
            assert hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == expected, p
        pins[name] = len(files)
    plan=read(HERE/'PLAN.json')
    by_mode={};witnesses=0
    for directory,expected_rows in (('numerical',1106),('fallback-numerical',10)):
        out=HERE/directory
        rows=[json.loads(line) for line in (out/'rows.jsonl').read_text().splitlines()]
        assert len(rows)==expected_rows and [r['number'] for r in rows] == list(range(expected_rows))
        assert [r['group'] for r in rows[:10]] == ['synthetic/'+name for name in plan['synthetic_cases']]
        assert all(r['eligible'] and r['method']==5 for r in rows)
        counts=Counter()
        for row in rows:
            c=row['comparison']
            counts['maps']+=1;counts['elements']+=c['elements'];counts['different_elements']+=c['different_elements']
            assert c['shape'] == [118,118] and c['elements']==118*118
            if c['bit_exact']:
                assert c['different_elements']==0 and c['baseline_sha256']==c['alternate_sha256']
                assert c['first_difference'] is None
                continue
            counts['different_maps']+=1
            assert c['baseline_sha256']!=c['alternate_sha256']
            assert 'witness' in row  # In the fixed failing corpus each map has its own group.
            with np.load(out/row['witness'],allow_pickle=False) as data:
                target,template=data['target'],data['template']
                first,second=data['baseline'],data['alternate']
                assert list((sha(target),sha(template))) == row['input_sha256']
                assert target.dtype==template.dtype==np.float32
                assert target.astype(np.uint8).astype(np.float32).tobytes()==target.tobytes()
                assert template.astype(np.uint8).astype(np.float32).tobytes()==template.tobytes()
                assert sha(first)==c['baseline_sha256'] and sha(second)==c['alternate_sha256']
                diff=first.view(np.uint32)!=second.view(np.uint32)
                assert int(diff.sum())==c['different_elements']
                at=tuple(np.argwhere(diff)[0]);stored=c['first_difference']
                assert list(at)==stored['index']
                assert f'{int(first.view(np.uint32)[at]):08x}'==stored['baseline_bits']
                assert f'{int(second.view(np.uint32)[at]):08x}'==stored['alternate_bits']
                assert float(np.abs(first.astype(float)-second.astype(float)).max())==c['maximum_finite_absolute_difference']
            witnesses+=1
        summary=read(out/'SUMMARY.json')
        assert summary['records']==counts['maps'] and summary['different_maps']==counts['different_maps']
        assert summary['different_elements']==counts['different_elements'] and summary['compared_elements']==counts['elements']
        assert summary['ordinary_timing_runs']==0 and summary['controller_sequences']==0 and not summary['physical_execution']
        by_mode[directory]=dict(counts)
    first=read(HERE/'numerical/SUMMARY.json');second=read(HERE/'fallback-numerical/SUMMARY.json')
    assert first['different_maps']==0 and first['opencv_ipp'] is True
    assert second['unconditional_dtype_substitution_rejected'] and second['different_maps']==6
    assert second['ipp_restored'] and second['threads_restored'] and not second['backend_configuration_deployed']
    locals=first['local_cases']
    assert [(c['case'],c['dispatch']) for c in locals]==[(n,d) for n in plan['local_cases'] for d in ('implicit','explicit')]
    for c in locals:
        assert c['full_serialized_output_exact']
        group='local/'+c['case']+'/'+c['dispatch']
        rows=[json.loads(line) for line in (HERE/'numerical/rows.jsonl').read_text().splitlines()]
        assert sum(r['group']==group for r in rows)==c['match_calls']
    assert sum(c['match_calls'] for c in locals)==1096
    tests={}
    for name in ('tests.xml','fallback-tests.xml'):
        cases=list(ET.parse(HERE/name).iter('testcase'))
        assert all(len(c)==0 for c in cases)
        tests[name]=len(cases)
    assert tests=={'tests.xml':198,'fallback-tests.xml':4}
    result=dict(pins_verified=pins,mode_totals=by_mode,all_six_raw_counterexamples_verified=True,
        raw_witnesses=witnesses,tests_passed=tests,local_full_outputs_exact=12,
        no_cv2_match_or_controller_reexecution_in_audit=True,unconditional_substitution_rejected=True,
        ipp_enabled_universal_equivalence_proved=False,production_changes=False,
        performance_gate_run=False,verification_complete=False,physical_execution=False)
    with (HERE/'INDEPENDENT_CHECK.json').open('x') as f:f.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
