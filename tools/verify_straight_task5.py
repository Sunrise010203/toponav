"""Frozen task5 rear/active-proposal study; NOT recovery-execution acceptance.

Fixture truth is read only after wheel-driven closed loop. Safe stopping,
RGB-consistent rear, and proposal eligibility are separate from navigation.
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
    if output.exists():raise ValueError('Never overwrite task5 runs')
    args=dict(case['arguments']);args['scene']=CorridorConfig(**args['scene'])
    start=time.monotonic();result=run_case(output,**args,straight_variant=variant,terminal_check_ticks=3)
    rows=[json.loads(l) for l in (output/'decisions.jsonl').read_text().splitlines()]
    trace=json.loads((output/'audit/physics-trace.json').read_text())
    truth=[json.loads(l) for l in (output/'audit/truth.live.jsonl').read_text().splitlines()]
    item=assess(case,result,rows,trace,truth)
    rear=[];following_sweeps=[];active=[];active_sweeps=[]
    for row in rows:
        d=row['decision'];e=d.get('rear_observation',{})
        for collection,key in [(rear,'rear'),(following_sweeps,'following_sweep'),(active,'active_review'),(active_sweeps,'active_sweep')]:
            r=e.get(key)
            if r and (r.get('geometry_reference_frame_id'),r.get('geometry_time_s'))==(d['control_frame_id'],d['captured_sim_s']):collection.append(r)
        if variant in ('rear','rear_active') and d['state']=='following' and (d['v_m_s'] or d['omega_rad_s']):
            r=e.get('rear') or {};s=e.get('following_sweep') or {}
            if (r.get('status')!='conditional_rear_consistent' or s.get('status')!='conditional_patch_supported_sweep' or
                any((x.get('geometry_reference_frame_id'),x.get('geometry_time_s'))!=(d['control_frame_id'],d['captured_sim_s']) for x in [r,s]) or
                (s.get('requested_v_m_s'),s.get('requested_omega_rad_s'))!=(d['v_m_s'],d['omega_rad_s'])):
                item['safety_errors'].append('rear_following_without_exact_current_extra_evidence')
        if e.get('active_executed'):item['safety_errors'].append('proposal_executed_despite_no_executor')
        if variant in ('rear','rear_active') and d['state']=='waiting_evidence' and (d['v_m_s'] or d['omega_rad_s']):
            item['safety_errors'].append('proposal_bypassed_wait_zero')
    all_sweeps=following_sweeps+active_sweeps
    item['rear_study']=dict(current_rear_assessments=len(rear),statuses=dict(Counter(r['status'] for r in rear)),
        reasons=dict(Counter(r['reason'] for r in rear)),following_sweeps=len(following_sweeps),active_sweeps=len(active_sweeps),
        supported_sweeps=sum(r['status']=='conditional_patch_supported_sweep' for r in all_sweeps),
        maximum_unknown_cells=max((r.get('coverage',{}).get('unknown_cells',0) for r in all_sweeps),default=0),
        maximum_required_cells=max((r.get('coverage',{}).get('required_cells',0) for r in all_sweeps),default=0),
        back_query_patches=sum(sum(p['view']=='back' for p in r.get('rgb_support',{}).get('patches',[])) for r in all_sweeps),
        active_reviews=len(active),active_statuses=dict(Counter(r['status'] for r in active)),
        failed_active_checks=dict(Counter(c for r in active for c in r['failed_checks'])),
        active_executed=0,active_execution_coverage=False,
        final_current_rear=rear[-1] if rear else None,final_current_sweep=all_sweeps[-1] if all_sweeps else None,
        final_current_proposal=active[-1] if active else None)
    if variant in ('rear','rear_active') and case.get('rear_required') and not rear:
        item['functional_or_coverage_errors'].append('rear_assessment_not_exercised')
    if variant=='rear_active' and case.get('active_review_required') and not active:
        item['functional_or_coverage_errors'].append('active_proposal_review_not_exercised')
    if case.get('role')=='rear_fault':
        kind=case['arguments']['fault']['kind'];injected=[i for i,r in enumerate(rows) if r.get('injected_fault')==kind]
        item['rear_fault_exercised']=bool(injected)
        item['fault_after_supported_following']=any(i>0 and rows[i-1]['decision']['state']=='following' and rows[i-1]['decision']['v_m_s'] for i in injected)
        if not injected:item['functional_or_coverage_errors'].append('rear_fault_not_injected')
        if variant in ('rear','rear_active') and not item['fault_after_supported_following']:
            item['functional_or_coverage_errors'].append('rear_fault_not_exercised_after_accepted_guarded_following')
    try:
        verify_public_records(output,rows);item['public_replay']=replay_safety_public(output,result,rows)
    except Exception as error:item['safety_errors'].append('public_replay_failed: '+str(error))
    verify_candidate(suite)
    item.update(suite_sha256=suite_sha,total_run_and_replay_wall_s=time.monotonic()-start)
    (output/'ASSESSMENT.json').write_text(json.dumps(item,indent=2,allow_nan=False)+'\n')
    print('TASK5_ASSESSMENT',case['id'],variant,'safety',item['safety_errors'],'functional',item['functional_or_coverage_errors'],flush=True)
    return item


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--suite',type=Path,required=True)
    p.add_argument('--case',required=True);p.add_argument('--variant',choices=['baseline','swept','rear','rear_active'],required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args(argv)
    suite=json.loads(a.suite.read_text())
    if suite['scope']!='task5_rear_and_proposal_study_not_recovery_execution_acceptance':raise ValueError('Wrong suite scope')
    case=next(c for c in suite['cases'] if c['id']==a.case)
    r=run_one(suite,case,a.variant,a.output,hashlib.sha256(a.suite.read_bytes()).hexdigest())
    return 1 if r['safety_errors'] else 0


if __name__=='__main__':raise SystemExit(main())
