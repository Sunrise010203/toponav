"""Frozen task4 swept-path ablation; not a navigation acceptance substitute.

Truth/physics and expectations are read ONLY after the wheel-driven run.
Unknown support must stop; loss of all following is reported as functional
failure, not credited as successful navigation just because it is safe.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from tools.run_straight_task3 import assess,verify_candidate


def run_one(suite,case,variant,output,suite_sha):
    import cv2
    cv2.setNumThreads(1)
    from simulation.continuous_straight_run import run_case
    from simulation.corridor_motion import CorridorConfig
    from simulation.continuous_straight_acceptance import replay_safety_public,verify_public_records
    verify_candidate(suite)
    if output.exists():raise ValueError('Never overwrite prior task4 runs')
    args=dict(case['arguments']);args['scene']=CorridorConfig(**args['scene'])
    started=time.monotonic();result=run_case(output,**args,straight_variant=variant,terminal_check_ticks=3)
    rows=[json.loads(l) for l in (output/'decisions.jsonl').read_text().splitlines()]
    trace=json.loads((output/'audit/physics-trace.json').read_text())
    truth=[json.loads(l) for l in (output/'audit/truth.live.jsonl').read_text().splitlines()]
    item=assess(case,result,rows,trace,truth)
    supports=[];reasons=Counter()
    for row in rows:
        d=row['decision'];g=d.get('trajectory_guard',{}).get('evidence')
        if g and g['geometry_reference_frame_id']==d['control_frame_id']:
            supports.append(g);reasons[g['reason']]+=1
    if variant=='swept' and case['expectation']=='progress_and_safe_stop' and not supports:
        item['functional_or_coverage_errors'].append('swept_guard_not_exercised')
    if case['expectation']=='fault_stop':
        injected=[i for i,row in enumerate(rows) if row.get('injected_fault')==case['arguments']['fault']['kind']]
        active=any(i>0 and (rows[i-1]['decision']['v_m_s'] or rows[i-1]['decision']['omega_rad_s']) for i in injected)
        item['fault_while_commanded_motion']=bool(active)
        if not active:item['functional_or_coverage_errors'].append('fault_not_injected_while_commanded_motion')
    item['trajectory_support']=dict(current_assessments=len(supports),reasons=dict(reasons),
        fully_patch_supported=sum(g['status']=='conditional_patch_supported_sweep' for g in supports),
        maximum_unknown_cells=max((g.get('coverage',{}).get('unknown_cells',0) for g in supports),default=0),
        maximum_required_cells=max((g.get('coverage',{}).get('required_cells',0) for g in supports),default=0),
        maximum_unseen_queries=max((g.get('rgb_support',{}).get('outside_all_supported_views',0) for g in supports),default=0),
        final_current_assessment=supports[-1] if supports else None,
        guard_exercised=bool(supports),bootstrap_authority='existing explicit operator initial swept-path prior, not RGB coverage')
    # Guarded nonzero following MUST match a current positive patch/plane
    # support receipt and the exact requested candidate. No bypass/fallback.
    for row in rows:
        d=row['decision']
        if variant=='swept' and d['state']=='following' and (d['v_m_s'] or d['omega_rad_s']):
            g=d.get('trajectory_guard',{}).get('evidence')
            if (not g or g['status']!='conditional_patch_supported_sweep' or
                (g['geometry_reference_frame_id'],g['geometry_time_s'])!=(d['control_frame_id'],d['captured_sim_s']) or
                (g['requested_v_m_s'],g['requested_omega_rad_s'])!=(d['v_m_s'],d['omega_rad_s'])):
                item['safety_errors'].append('following_without_current_swept_support')
    try:
        verify_public_records(output,rows);item['public_replay']=replay_safety_public(output,result,rows)
    except Exception as error:item['safety_errors'].append('public_replay_failed: '+str(error))
    verify_candidate(suite)
    item.update(suite_sha256=suite_sha,total_run_and_replay_wall_s=time.monotonic()-started)
    (output/'ASSESSMENT.json').write_text(json.dumps(item,indent=2,allow_nan=False)+'\n')
    print(json.dumps(item,allow_nan=False),flush=True)
    return item


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--suite',type=Path,required=True)
    p.add_argument('--case',required=True);p.add_argument('--variant',choices=['baseline','swept'],required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args(argv)
    suite=json.loads(a.suite.read_text())
    if suite['scope']!='task4_swept_coverage_not_s2_s6_acceptance':raise ValueError('Wrong task4 suite scope')
    case=next(c for c in suite['cases'] if c['id']==a.case)
    r=run_one(suite,case,a.variant,a.output,hashlib.sha256(a.suite.read_bytes()).hexdigest())
    return 1 if r['safety_errors'] else 0


if __name__=='__main__':raise SystemExit(main())
