"""Run a pre-frozen task-2 boundary suite; not historical S2/S6 acceptance.

Conditions and candidate hashes must be frozen BEFORE results are exposed.
The controller sees run_case's RGB plus explicit task priors only. Physics
and expectations below are used strictly after each run, never for decisions.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))


def verify_candidate(suite):
    for relative,expected in suite['candidate_files'].items():
        path=(ROOT/relative).resolve()
        if not path.is_relative_to(ROOT) or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            raise ValueError('Frozen task-2 candidate changed: '+relative)


def assess(case,result,rows,trace):
    audit=result['audit'];final=result['controller_result'];errors=[]
    if result['exception'] is not None:errors.append('runner_exception')
    if not audit['safe_final_stop'] or audit['wall_contact_physics_steps']:errors.append('unsafe_final_physics')
    if audit['maximum_audited_speed_m_s']>.18:errors.append('actuator_speed_assumption_exceeded')
    if result['safety_checks']['caught_input_errors']:errors.append('unexpected_input_exception')
    if not result['safety_checks']['terminal_latched']:errors.append('terminal_not_latched')
    braking=audit['braking_episodes']
    if any(b['resumed_before_stationary'] for b in braking):errors.append('resumed_before_physical_stop')
    moving=[r for r in rows if r['decision']['v_m_s'] or r['decision']['omega_rad_s']]
    following=[r for r in moving if r['decision']['state']=='following']
    for r in moving:
        d=r['decision'];p=d['permission']
        if d['state']=='bootstrapping':
            if p['mode']!='bounded_probe' or p['probe_distance_bound_m']>.140001 or p['probe_distance_bound_m']+p['stopping_reserve_m']>.300001:
                errors.append('probe_contract_exceeded')
        elif d['state']!='following':errors.append('motion_in_disallowed_state')
    reserve=rows[-1]['decision']['permission']['stopping_reserve_m']
    if any(b['distance_m'] is None or b['distance_m']>reserve for b in braking):errors.append('brake_distance_outside_reserve')
    if audit['bootstrap_max_displacement_m']>.14:errors.append('physical_probe_exceeded_command_envelope')
    # A visual-stop result is not enough: compare with high-rate physics at
    # the actual decision time, not just the runner's later finally settling.
    visual_stop=[]
    for r in rows:
        if r['decision']['state']=='visual_stop':
            sample=min(trace,key=lambda t:abs(t['sim_s']-r['sim_s']))
            ok=sample['speed_m_s']<.01 and abs(sample['yaw_rate_rad_s'])<.02
            visual_stop.append(dict(sim_s=r['sim_s'],physical_stationary=ok))
            if not ok:errors.append('visual_stop_before_physical_stationary')
    expected=case['expectation']
    if expected=='no_motion':
        if moving or audit['displacement_m']>=.001:errors.append('negative_case_moved')
    elif expected=='progress_and_safe_stop':
        if not following or audit['displacement_m']<.05:errors.append('positive_case_did_not_follow')
    elif expected=='fault_stop':
        if not moving or not any(r['injected_fault']==case['arguments']['fault']['kind'] for r in rows):errors.append('fault_not_exercised_after_motion')
        if final['reason']!=case['expected_reason']:errors.append('wrong_fault_result')
    elif expected=='front_visual_stop':
        if not moving or final['state']!='visual_stop' or not visual_stop:errors.append('front_visual_stop_not_observed')
    elif expected=='finite_budget_stop':
        if not following or final['reason']!='execution_cancelled':errors.append('finite_budget_not_exercised')
    else:raise ValueError('Unknown task-2 expectation')
    return dict(case=case['id'],expectation=expected,passed=not errors,errors=sorted(set(errors)),
                state=final['state'],reason=final['reason'],motion_commands=len(moving),following_commands=len(following),
                displacement_m=audit['displacement_m'],braking_episodes=len(braking),
                resumed_before_stationary=sum(b['resumed_before_stationary'] for b in braking),
                maximum_brake_distance_m=max((b['distance_m'] or 0 for b in braking),default=0),
                visual_stop_checks=visual_stop,completed_geometric_task=audit['normal_geometric_stop_pass'])


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--suite',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args(argv)
    suite=json.loads(a.suite.read_text());verify_candidate(suite)
    if suite['scope']!='task2_boundary_checks_not_s2_s6_acceptance':raise ValueError('Wrong suite scope')
    if a.output.exists():raise ValueError('Never overwrite prior run artifacts')
    a.output.mkdir(parents=True)
    from simulation.continuous_straight_run import run_case
    from simulation.corridor_motion import CorridorConfig
    from simulation.continuous_straight_acceptance import replay_safety_public,verify_public_records
    results=[]
    for case in suite['cases']:
        verify_candidate(suite);arguments=dict(case['arguments']);arguments['scene']=CorridorConfig(**arguments['scene'])
        base=a.output/case['id'];r=run_case(base,**arguments,terminal_check_ticks=3)
        rows=[json.loads(s) for s in (base/'decisions.jsonl').read_text().splitlines()]
        trace=json.loads((base/'audit/physics-trace.json').read_text());item=assess(case,r,rows,trace)
        try:
            verify_public_records(base,rows);item['public_replay']=replay_safety_public(base,r,rows)
        except Exception as error:
            item['errors'].append('public_replay_failed: '+str(error));item['passed']=False
        results.append(item)
        (a.output/'progress.json').write_text(json.dumps(results,indent=2)+'\n')
        print('TASK2_CHECK',case['id'],item['passed'],item['errors'],flush=True)
    verify_candidate(suite)
    summary=dict(scope=suite['scope'],suite_sha256=hashlib.sha256(a.suite.read_bytes()).hexdigest(),
                 passed=all(r['passed'] for r in results),case_count=len(results),results=results,
                 limitations=['not full-navigation acceptance','no real-hardware watchdog claim',
                              'same simulator/generator family, new frozen seeds and conditions',
                              'RGB stop support is conditional, not physical motion measurement'])
    (a.output/'ASSESSMENT.json').write_text(json.dumps(summary,indent=2)+'\n')
    return 0 if summary['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
