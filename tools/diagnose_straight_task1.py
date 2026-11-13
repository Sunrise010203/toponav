"""Diagnose recorded public straight evidence, or regenerate KNOWN S2 cases.

No production algorithm, threshold, permission or sensor is modified. Historical
RGB/JSONL were deliberately deleted in the 2026-10-18 cleanup. Regeneration is
explicitly diagnostic, goes to a fresh directory, and compares every RGB hash,
capture identity/time and final controller/physical audit to retained history.
This is not an unseen test, a new acceptance batch, or a safety certification.
"""
from __future__ import annotations

import argparse
from collections import Counter
import contextlib
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    with Path(path).open('x') as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write('\n')


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def verify_frozen(history):
    files = read(history / 'freeze.json')['files']
    mismatches = [name for name, expected in files.items()
                  if not (ROOT / name).is_file() or sha(ROOT / name) != expected]
    if mismatches:
        raise ValueError('Historical frozen files differ: ' + ', '.join(mismatches))
    return dict(entries=len(files), mismatches=mismatches)


def evidence(row):
    """Read recorded PUBLIC decision fields only; never read physical truth.

    Carried-forward diagnostics are marked stale, not counted as a new fit.
    Multiple blockers are preserved, not collapsed into one guessed cause.
    """
    decision = row['decision']
    perception = decision.get('perception') or {}
    free = decision.get('free_path') or {}
    captured = decision.get('captured_sim_s', -1)

    def current(packet):
        stamp = packet.get('geometry_time_s')
        return (type(stamp) in (int, float) and type(captured) in (int, float)
                and math.isclose(stamp, captured, abs_tol=1e-9, rel_tol=0)
                # S2 is a synchronous capture/decision loop. Early rejection
                # can retain BOTH the previous capture stamp and perception;
                # comparing those two alone would count a cached fault result
                # as a new fit. It must also belong to THIS decision tick.
                and math.isclose(stamp, row['sim_s'], abs_tol=1e-9, rel_tol=0))

    perception_current, free_current = current(perception), current(free)
    startup = (decision.get('permission') or {}).get('startup_texture') or {}
    startup_frames = startup.get('frames') or []
    inp = row.get('input') or {}
    # Startup precedes the first geometry fit. Do not silently drop its
    # rejection, or count carried startup metadata as a new assessment.
    # Older synchronous records lack evaluated_sim_s and per-frame times;
    # their fallback is deliberately limited to capture == decision time.
    startup_current = bool(
        len(startup_frames) == 3 and inp
        and decision['reason'] in ('startup_texture_unverified', 'bounded_probe_inside_explicit_task_clearance')
        and startup_frames[-1].get('frame_id') == decision.get('control_frame_id') == inp.get('frame_id')
        and inp.get('captured_sim_s') == captured
        and startup_frames[-1].get('captured_sim_s', captured) == captured
        and startup.get('evaluated_sim_s', captured) == row['sim_s']
        and decision.get('decision_sim_s', row['sim_s']) == row['sim_s']
        and type(captured) in (int, float) and 0 <= row['sim_s'] - captured <= .25)
    motion = perception.get('ground_motion') or {}
    corridor = perception.get('corridor_geometry') or {}
    paired = perception.get('paired_corridor') or {}
    blockers = []
    if startup_current and startup.get('accepted') is False:
        blockers.append('startup_texture_unverified')
    if perception_current:
        if motion.get('status') != 'conditional_ground_motion':
            blockers.append('ground_motion_unavailable')
        if corridor.get('status') != 'conditional_corridor_geometry':
            blockers.append('corridor_geometry_unavailable')
    if free_current and free.get('status') != 'conditional_free_ground_path':
        blockers.append('free_ground_unavailable')
    moving = bool(decision['v_m_s'] or decision['omega_rad_s'])
    state = decision['state']
    route = ('startup_probe' if moving and state == 'bootstrapping' else
             'supported_following' if moving and state == 'following' else
             'other_motion' if moving else 'zero_command')
    diagnostic = dict(
        tick=row['tick'], sim_s=row['sim_s'], frame_id=decision.get('control_frame_id'),
        state=state, reason=decision['reason'], moving=moving, authorization=route,
        v_m_s=decision['v_m_s'], omega_rad_s=decision['omega_rad_s'],
        perception_current=perception_current, free_check_current=free_current,
        blockers=blockers,
        startup_texture_current=startup_current,
    )
    if startup_current:
        diagnostic.update(startup_texture_accepted=startup.get('accepted'),
                          startup_texture_frames=startup_frames)
    if perception_current:
        diagnostic.update(
            ground_status=motion.get('status'), ground_reason=motion.get('reason'),
            ground_translation_m=motion.get('translation_m'),
            ground_yaw_delta_rad=motion.get('yaw_delta_rad'),
            corridor_status=corridor.get('status'), corridor_reason=corridor.get('reason'),
            paired_status=paired.get('status'), paired_reason=paired.get('reason'),
            paired_rejections=paired.get('reasons', []),
            heading_error_rad=corridor.get('heading_error_rad'), offset_m=corridor.get('offset_m'),
            history_model=(perception.get('history_model_selection') or {}).get('model'),
            view_reasons={view: (values.get('plane_region') or {}).get('reasons', [])
                          for view, values in perception.get('diagnostics', {}).items()},
            ground_diagnostics=motion.get('views', motion.get('view_checks', {})),
        )
    if free_current:
        diagnostic.update(free_status=free.get('status'), free_reason=free.get('reason'),
                          rejected_cells=[dict(index=i, reason=cell.get('reason'),
                                               reference_accepted=cell.get('reference_accepted'),
                                               residual_px=cell.get('ground_flow_residual_px'))
                                          for i, cell in enumerate(free.get('cells', []))
                                          if not cell.get('accepted')])
    return diagnostic


def summarize(rows):
    records = [evidence(row) for row in rows]
    fresh = [r for r in records if r['perception_current']]
    waiting = [r for r in records if r['state'] == 'waiting_evidence']
    failed = [r for r in records if r['blockers']]
    # A cached/duplicated assessment is one event, not three new failures
    # on later ticks. Geometry attribution retains its existing semantics.
    startup_seen = set(); startup_assessments = []; blockers = Counter()
    for record in records:
        first_startup = False
        if record['startup_texture_current']:
            key = tuple(f['frame_id'] for f in record['startup_texture_frames'])
            first_startup = key not in startup_seen
            startup_seen.add(key)
            if first_startup: startup_assessments.append(record)
        blockers.update(b for b in record['blockers']
                        if b != 'startup_texture_unverified' or first_startup)
    return dict(
        decision_count=len(records), fresh_perception_count=len(fresh),
        authorization_counts=dict(Counter(r['authorization'] for r in records)),
        state_counts=dict(Counter(r['state'] for r in records)),
        blocker_counts=dict(blockers), startup_assessment_count=len(startup_assessments),
        paired_reason_counts=dict(Counter(r['paired_reason'] for r in fresh)),
        ground_reason_counts=dict(Counter(r['ground_reason'] for r in fresh)),
        free_reason_counts=dict(Counter(r.get('free_reason') for r in fresh if r['free_check_current'])),
        first_motion=next((r for r in records if r['moving']), None),
        first_evidence_failure=failed[0] if failed else None,
        first_wait=waiting[0] if waiting else None,
        terminal=records[-1],
    ), records


def diagnose_recorded(source, output):
    """Read-only public-log attribution; no inference/physics/acceptance claim.

    Verify the declared frame sequence and decoded RGB bytes BEFORE creating
    output. Never substitute terminal images for missing startup evidence.
    """
    import numpy as np
    from PIL import Image

    source, output = Path(source).resolve(), Path(output).resolve()
    if output == source or source in output.parents:
        raise ValueError('Diagnostic output must not be inside captured evidence')
    rows = [json.loads(line) for line in (source / 'decisions.jsonl').read_text().splitlines()]
    if not rows: raise ValueError('Empty decision evidence')
    inputs = [r['input'] for r in rows if r.get('input') is not None]
    sequence = read(source / 'public/sequence.json')
    if not inputs or sequence.get('frames') != inputs:
        raise ValueError('Public sequence differs from decision inputs or is empty')
    for reference in inputs:
        path = (source / 'public' / reference['rgb_path']).resolve()
        if not path.is_relative_to(source / 'public/frames'):
            raise ValueError('Only captured public RGB frame paths are allowed')
        with Image.open(path) as image:
            pixels = np.array(image)
        if pixels.dtype != np.uint8 or pixels.shape != (256, 256, 3):
            raise ValueError('Invalid captured RGB array')
        if hashlib.sha256(pixels.tobytes()).hexdigest() != reference['rgb_sha256']:
            raise ValueError('Captured RGB hash differs from decision input')
    summary, records = summarize(rows)
    result = dict(mode='recorded_public_evidence_diagnostic', source=str(source), summary=summary,
                  verified_public_frames=len(inputs), formal_navigation_acceptance=False,
                  recorded_perception_recomputed=False, physics_rerun=False,
                  source_sha256={name: sha(source / name) for name in ('decisions.jsonl', 'public/sequence.json')},
                  scope='Recorded diagnostics only; RGB hashes verified, no truth read, no new model or clearance acceptance')
    output.mkdir(parents=True, exist_ok=False)
    write(output / 'diagnosis.json', result)
    with (output / 'diagnostics.jsonl').open('x') as stream:
        for record in records:
            stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
    return result


def compare_history(history_case, regenerated_case):
    old_sequence = read(history_case / 'public/sequence.json')
    new_sequence = read(regenerated_case / 'public/sequence.json')
    old_result = read(history_case / 'result.json')
    new_result = read(regenerated_case / 'result.json')
    checks = dict(
        public_sequence_exact=old_sequence == new_sequence,
        frame_count_exact=old_result['frame_count'] == new_result['frame_count'],
        decision_count_exact=old_result['decision_count'] == new_result['decision_count'],
        controller_result_exact=old_result['controller_result'] == new_result['controller_result'],
        physical_audit_exact=old_result['audit'] == new_result['audit'],
        no_exception=new_result['exception'] is None,
    )
    checks['passed'] = all(checks.values())
    checks['historical_decisions_available'] = (history_case / 'decisions.jsonl').is_file()
    checks['historical_per_decision_equality_claimed'] = False
    # Old JSONL no longer exists: matching frames/results is not a direct
    # comparison to historical intermediate decisions. Preserve that boundary.
    return checks


def run(history, output, case_ids=None):
    history, output = Path(history).resolve(), Path(output).resolve()
    if output == history or history in output.parents:
        raise ValueError('Diagnostic output must not be inside historical evidence')
    verify = verify_frozen(history)
    suite = read(history / 's2-suite.json')
    selected = [c for c in suite['cases'] if case_ids is None or c['id'] in case_ids]
    if not selected or (case_ids is not None and set(case_ids) != {c['id'] for c in selected}):
        raise ValueError('Unknown or empty case selection')
    # Match the retained historical pipeline's numerical-library thread caps.
    # This must precede the first numpy/scipy/simulation import in the CLI.
    numerical_environment = {name: '2' for name in
                             ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS')}
    os.environ.update(numerical_environment)
    output.mkdir(parents=True, exist_ok=False)
    write(output / 'STARTED.json', dict(
        started_at=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
        task='1. 细分失败原因并审查现有许可链路', mode='known_condition_diagnostic_regeneration',
        historical_root=str(history), historical_freeze=verify, cases=[c['id'] for c in selected],
        production_changes=False, independent_acceptance=False,
        numerical_environment=numerical_environment,
        reason='Historical RGB and JSONL intentionally removed; see reports/CLEANUP_2026-10-18.md',
    ))
    from simulation.continuous_straight_run import run_case
    from simulation.corridor_motion import CorridorConfig
    summaries = []
    for case in selected:
        verify_frozen(history)
        case_output = output / 'runs' / case['id']
        with (output / (case['id'] + '.log')).open('x') as log, contextlib.redirect_stdout(log):
            result = run_case(case_output, scene=CorridorConfig(**case['scene']),
                              appearance=case['appearance'], seed=case['seed'],
                              pattern=case['pattern'], max_ticks=case['max_ticks'],
                              operator_confirmed=case['operator_confirmed'], fault=case['fault'],
                              narrow_obstacle_width_m=case.get('narrow_obstacle_width_m', .5))
        comparison = compare_history(history / 's2' / case['id'], case_output)
        rows = [json.loads(line) for line in (case_output / 'decisions.jsonl').read_text().splitlines()]
        summary, diagnostics = summarize(rows)
        summary.update(case=case['id'], expectation=case['expectation'], comparison=comparison,
                       result=result['controller_result'], audit=result['audit'])
        write(case_output / 'diagnosis.json', summary)
        with (case_output / 'diagnostics.jsonl').open('x') as stream:
            for record in diagnostics:
                stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
        summaries.append(summary)
        # Durable partial work; an interruption never becomes a silent success.
        (output / 'progress.json').write_text(json.dumps(dict(cases=[s['case'] for s in summaries]), indent=2) + '\n')
        print(case['id'], 'matched=' + str(comparison['passed']),
              'states=' + str(summary['state_counts']), flush=True)
        if not comparison['passed']:
            write(output / 'MISMATCH.json', dict(case=case['id'], checks=comparison))
            raise ValueError('Historical reproduction mismatch; stop without tuning')
    write(output / 'diagnostic-summary.json', dict(
        task=1, independent_acceptance=False, cases=summaries,
        total_decisions=sum(s['decision_count'] for s in summaries),
        all_history_comparisons_passed=all(s['comparison']['passed'] for s in summaries),
        final_historical_freeze=verify_frozen(history),
    ))
    return summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--history', type=Path, default=ROOT / 'reports/block6-acceptance-20261018')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cases', nargs='+')
    parser.add_argument('--recorded-run', type=Path,
                        help='Read existing public RGB/decisions without historical regeneration or physics')
    args = parser.parse_args()
    if args.recorded_run is not None:
        if args.cases is not None: parser.error('--cases is only for historical regeneration')
        result = diagnose_recorded(args.recorded_run, args.output)
        print(json.dumps(result['summary'], ensure_ascii=False, indent=2))
    else:
        run(args.history, args.output, args.cases)


if __name__ == '__main__':
    main()
