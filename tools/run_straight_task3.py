"""One pre-frozen RGB-only task3 closed-loop ablation and independent replay.

Physics is read ONLY after run_case returns. All variants use the same task
fixtures, not forced identical sensor streams after their actions diverge.
No parameter search, alternate-fit retries, acceptance promotion or old writes.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
VARIANTS=('baseline','history','control','combined')


def verify_candidate(suite):
    for rel,expected in suite['candidate_files'].items():
        p=(ROOT/rel).resolve()
        if not p.is_relative_to(ROOT) or hashlib.sha256(p.read_bytes()).hexdigest()!=expected:raise ValueError('Frozen task3 candidate changed: '+rel)


def assess(case,result,rows,trace,truth):
    from tools.verify_straight_task2 import assess as boundary_assess
    from simulation.continuous_straight_acceptance import audit_safety_physics
    boundary=boundary_assess(case,result,rows,trace);physics=audit_safety_physics(result,rows,trace)
    # Separate navigation completion from progress-only boundary checks.
    following=[r for r in rows if r['decision']['state']=='following' and r['decision']['v_m_s']]
    rejection=Counter(r['decision']['reason'] for r in rows if r['decision']['state']=='waiting_evidence')
    history=Counter();selection_unknown=0;perception_failures=Counter()
    for r in rows:
        s=r['decision'].get('experiment',{}).get('history_selection')
        if s and (not s['selected_frame_ids'] or s['selected_frame_ids'][-1]==r['decision']['control_frame_id']):
            if s['selected_size'] is None:selection_unknown+=1
            else:history[str(s['selected_size'])]+=1
        d=r['decision'];packet=d.get('perception')
        if d['state']=='waiting_evidence' and packet and packet.get('geometry_reference_frame_id')==d['control_frame_id']:
            for name,evidence in [('ground',packet.get('ground_motion',{})),('paired',packet.get('paired_corridor',{})),('corridor',packet.get('corridor_geometry',{})),('free',d.get('free_path') or {})]:
                perception_failures[name+':'+str(evidence.get('status'))+':'+str(evidence.get('reason'))]+=1
    omega=np.array([r['decision']['omega_rad_s'] for r in following]);nonzero=omega[np.abs(omega)>.002]
    signs=int(np.sum(nonzero[1:]*nonzero[:-1]<0))
    # Common full-run metrics and follow-only metrics are both retained. A
    # short failed run must not masquerade as a better tracking controller.
    def errors(samples):
        if not samples:return None
        return dict(samples=len(samples),rms_lateral_m=float(np.sqrt(np.mean([t['position_m'][1]**2 for t in samples]))),
                    max_abs_lateral_m=max(abs(t['position_m'][1]) for t in samples),
                    rms_yaw_deg=float(np.sqrt(np.mean([t['yaw_deg']**2 for t in samples]))),
                    max_abs_yaw_deg=max(abs(t['yaw_deg']) for t in samples))
    follow_times={r['sim_s'] for r in following}
    item=dict(case=case['id'],variant=result['safety_checks']['configuration'].get('straight_variant','baseline'),
        state=result['controller_result']['state'],reason=result['controller_result']['reason'],
        completed_geometric_task=bool(result['audit']['normal_geometric_stop_pass']),
        displacement_m=result['audit']['displacement_m'],decision_count=len(rows),following_commands=len(following),
        following_duration_s=.1*len(following),recovery_count=rows[-1]['decision']['permission']['recovery_count'],
        rejection_reasons=dict(rejection),perception_failures=dict(perception_failures),history_windows=dict(history),selection_unknown_decisions=selection_unknown,
        steering_sign_changes=signs,following_omega_total_variation=float(np.abs(np.diff(omega)).sum()),
        follow_only_truth_error=errors([t for t in truth if any(abs(t['sim_s']-s)<1e-7 for s in follow_times)]),
        whole_run_truth_error=errors(truth),safety_boundary=boundary,independent_physics=physics,
        wall_elapsed_s=result['elapsed_wall_s'],simulated_duration_s=result['audit']['final']['sim_s']-result['audit']['initial']['sim_s'])
    item['safety_errors']=sorted(set(physics['errors']+[e for e in boundary['errors'] if e not in
        ('positive_case_did_not_follow','fault_not_exercised_after_motion','wrong_fault_result','front_visual_stop_not_observed','finite_budget_not_exercised')]))
    item['functional_or_coverage_errors']=[e for e in boundary['errors'] if e not in item['safety_errors']]
    return item


def run(suite,case,variant,output,suite_hash):
    import cv2
    cv2.setNumThreads(1)
    from simulation.continuous_straight_run import run_case
    from simulation.corridor_motion import CorridorConfig
    from simulation.continuous_straight_acceptance import replay_safety_public,verify_public_records
    verify_candidate(suite)
    if output.exists():raise ValueError('Never overwrite task3 output')
    arguments=dict(case['arguments']);arguments['scene']=CorridorConfig(**arguments['scene'])
    t=time.monotonic();r=run_case(output,**arguments,straight_variant=variant,terminal_check_ticks=3)
    rows=[json.loads(l) for l in (output/'decisions.jsonl').read_text().splitlines()]
    trace=json.loads((output/'audit/physics-trace.json').read_text())
    truth=[json.loads(l) for l in (output/'audit/truth.live.jsonl').read_text().splitlines()]
    item=assess(case,r,rows,trace,truth)
    try:
        verify_public_records(output,rows);item['public_replay']=replay_safety_public(output,r,rows)
    except Exception as error:item['safety_errors'].append('public_replay_failed: '+str(error))
    verify_candidate(suite)
    item.update(suite_sha256=suite_hash,total_run_and_replay_wall_s=time.monotonic()-t)
    (output/'ASSESSMENT.json').write_text(json.dumps(item,indent=2,allow_nan=False)+'\n')
    print(json.dumps(item,allow_nan=False),flush=True)
    return item


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--suite',type=Path,required=True);p.add_argument('--case',required=True)
    p.add_argument('--variant',choices=VARIANTS,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(argv);suite=json.loads(a.suite.read_text())
    if suite['scope']!='task3_ablation_not_s2_s6_acceptance':raise ValueError('Wrong suite scope')
    case=next(c for c in suite['cases'] if c['id']==a.case)
    item=run(suite,case,a.variant,a.output,hashlib.sha256(a.suite.read_bytes()).hexdigest())
    return 1 if item['safety_errors'] else 0


if __name__=='__main__':raise SystemExit(main())
