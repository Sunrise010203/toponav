"""Post-prediction S1 gate and SAME-RGB frozen I/H/G diagnostic comparison."""
import argparse
import json
import math
from pathlib import Path
from simulation.ground_history_study import audit
from simulation.rgb_observability_run import verify_freeze


def assess_metrics(case,summary,metrics):
    errors=[];group=case['evaluation_group'];windows=summary['window_count']
    if windows!=5:errors.append('expected_exactly5_windows')
    motion=summary['ground_status_counts'].get('conditional_ground_motion',0)
    corridor=summary['corridor_status_counts'].get('conditional_corridor_geometry',0)
    front=summary['front_status_counts'].get('conditional_front_surface',0)
    if group in ('straight','near_front','motion') and motion<math.ceil(.8*windows):errors.append('motion_coverage_below80percent')
    if group in ('straight','near_front') and corridor<math.ceil(.8*windows):errors.append('corridor_coverage_below80percent')
    if group=='near_front' and front<math.ceil(.6*windows):errors.append('front_coverage_below60percent')
    if group=='texture_reject' and (summary['ground_status_counts']!={'unknown':5} or summary['corridor_status_counts']!={'unknown':5} or summary['front_status_counts']!={'unknown':5}):errors.append('texture_counterexample_not_unknown')
    limits=dict(translation_error_m=.015,yaw_error_rad=math.radians(1.5),offset_error_m=.08,
                heading_error_rad=math.radians(3),width_error_m=.18,front_error_m=.15)
    if group!='uncertified_prior_mismatch':
        for key,limit in limits.items():
            if metrics[key]['count'] and metrics[key]['max']>limit:errors.append(key+'_above_declared_limit')
    return errors


def _rows(path):return [json.loads(s) for s in Path(path).read_text().splitlines()]


def _diagnostic(result):
    return dict(region_candidate=any(r['status']=='planar_flow_candidate' for r in result['regions'].values()),
                shared_candidate=result['shared_motion']['status']=='shared_motion_candidate',
                anchored_candidate=result.get('anchored_support',{}).get('status')=='anchored_motion_candidate')


def evaluate(generated,main,short,frozen_i,freeze,output):
    generated,main,short,frozen_i=map(Path,(generated,main,short,frozen_i));verify_freeze(freeze)
    index=json.loads((generated/'public-index.json').read_text())['sequences'];ids={x['sequence_id'] for x in index}
    phases=[]
    for root,method in ((main,'adaptive'),(short,'history'),(frozen_i,'I')):
        phase=json.loads((root/'phase-complete.json').read_text())
        if phase['method']!=method or len(phase['sequences'])!=len(ids) or {x['sequence_id'] for x in phase['sequences']}!=ids:raise ValueError('ALL prediction phases must complete before any case/truth audit')
        phases.append({x['sequence_id']:x['exit_code'] for x in phase['sequences']})
    out=Path(output);out.mkdir(parents=True,exist_ok=False)
    result=audit(generated,main,out/'metric-audit.json');failures=[];comparison=[];assessment=[]
    for row in result['cases']:
        case=row['case'];name=case['id'];exits=[phase[name] for phase in phases]
        if case['evaluation_group']=='envelope_reject':
            if exits!=[2,2,2]:failures.append(dict(case=name,errors=['wrong_ID_not_rejected_by_all_CLI']))
            assessment.append(dict(case=name,scope='input_protocol_rejection',exit_codes=exits));continue
        if any(exits):failures.append(dict(case=name,errors=['unexpected_predictor_exit'],exit_codes=exits));continue
        errors=assess_metrics(case,row['summary'],row['metrics'])
        if row['unauthorized_claims']:errors.append('motion_or_arrival_authorized')
        if errors:failures.append(dict(case=name,errors=errors))
        assessment.append(dict(case=name,scope=case['evaluation_group'],errors=errors,summary=row['summary'],metrics=row['metrics']))
        newest=_rows(main/name/'windows.jsonl');old=_rows(frozen_i/name/'windows.jsonl');small=_rows(short/name/'windows.jsonl')
        if len(newest)!=5 or len(old)!=11 or len(small)!=11:failures.append(dict(case=name,errors=['incomplete_comparison_window_counts']))
        old_by_last={r['result']['evidence_packet']['frames'][-1]['frame_id']:r for r in old if 'evidence_packet' in r['result']}
        small_by_last={r['result']['frames'][-1]['frame_id']:r for r in small if 'frames' in r['result']}
        matched=[]
        for r in newest:
            rr=r['result']
            if 'frames' not in rr:failures.append(dict(case=name,errors=['main_window_input_rejected']));continue
            frame=rr['frames'][-1]['frame_id']
            if frame not in old_by_last or frame not in small_by_last:failures.append(dict(case=name,errors=['missing_same_RGB_comparison']));continue
            ii=old_by_last[frame];ss=small_by_last[frame];ir=ii['result'];hr=ir['baseline'];gr=hr['baseline']
            p=ir['evidence_packet'];review=ir['review']
            if (review['motion_authorized'] is not False or review['arrival_status']!='unverified' or p['corridor_boundary']['state']!='unknown' or p['front_surface']['state']!='unknown'):failures.append(dict(case=name,errors=['frozen_physical_contract_changed']))
            def statuses(z):return {k:z[key]['status'] for k,key in [('ground','ground_motion'),('corridor','corridor_geometry'),('front','front_surface')]}
            matched.append(dict(latest_frame_id=frame,G=_diagnostic(gr),H=_diagnostic(hr),I=_diagnostic(ir),
                new_short=statuses(ss['result']),new_adaptive=statuses(rr),
                compute_wall_s=dict(I_including_H_G=ii['elapsed_wall_s'],new_short=ss['elapsed_wall_s'],new_adaptive=r['elapsed_wall_s']),
                selected_history_model=rr['history_model_selection']['model']))
        comparison.append(dict(case=name,common_latest_window_count=len(matched),matched=matched,
            different_priors_and_horizons=True,old_horizon_s=.2,new_short_horizon_s=.2,new_adaptive_horizon_s=.8,
            inputs='same manifest and exact RGB hashes; frozen I embeds unchanged H/G on .2s triplets'))
    final=dict(stage='S1',passed=not failures,case_count=len(result['cases']),failures=failures,assessment=assessment,
               comparison=comparison,not_navigation_or_control_acceptance=True,
               limitations=['stronger explicit calibration/ground/Manhattan priors','conditional noise intervals exclude prior/model mismatch',
                            'hidden calibration and dynamic-scene mismatch not certified','overlapping windows are not independent trials',
                            'camera_only cases are not motor execution','no motion or arrival authorization'])
    (out/'acceptance.json').write_text(json.dumps(final,indent=2)+'\n');return final


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('generated','main','short','frozen-i','freeze','output'):p.add_argument('--'+name,required=True,type=Path)
    a=p.parse_args(argv);r=evaluate(a.generated,a.main,a.short,a.frozen_i,a.freeze,a.output)
    print(json.dumps(dict(passed=r['passed'],case_count=r['case_count'],failures=r['failures']),indent=2));return 0 if r['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
