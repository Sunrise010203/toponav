"""Only after a passed performance gate: replay old recordings, no physics run."""
import json
from pathlib import Path

from tools.straight_pipeline_profile import verify_freeze

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_bytes())


def main():
    if read(HERE/'benchmark/SUMMARY.json')['complete_controller_performance_gate_passed'] is not True:
        raise ValueError('failed_performance_gate_no_candidate_replay')
    freeze = read(HERE/'REPLAY_FREEZE.json')
    verify_freeze(ROOT, freeze)
    plan = read(HERE/'REPLAY_PLAN.json')
    if (HERE/'replay-progress.jsonl').exists() or (HERE/'REPLAY.json').exists():
        raise ValueError('exclusive_replay_output')
    import cv2
    from simulation.continuous_straight_acceptance import replay_safety_public, audit_safety_physics
    previous = cv2.getNumThreads()
    cv2.setNumThreads(1)
    records = []
    try:
        for name in plan['cases']:
            base = ROOT/name
            result = read(base/'result.json')
            rows = [json.loads(line) for line in (base/'decisions.jsonl').read_text().splitlines()]
            replay = replay_safety_public(base, result, rows)
            physics = audit_safety_physics(result, rows, read(base/'audit/physics-trace.json'))
            if physics['errors']:
                raise AssertionError((name, physics))
            record = dict(case=name, replay=replay, old_physics=physics)
            records.append(record)
            with (HERE/'replay-progress.jsonl').open('a') as stream:
                stream.write(json.dumps(record, allow_nan=False)+'\n')
            print(name, 'exact', replay, flush=True)
        verify_freeze(ROOT, freeze)
    finally:
        cv2.setNumThreads(previous)
    decisions = sum(r['replay']['decisions'] for r in records)
    terminal = sum(r['replay']['terminal_frames'] for r in records)
    assert decisions == plan['decisions'] and terminal == plan['terminal_frames']
    with (HERE/'REPLAY.json').open('x') as stream:
        stream.write(json.dumps(dict(cases=records, decisions=decisions, terminal_frames=terminal,
            all_public_outputs_and_final_results_equal=True, old_physics_errors=[],
            frozen_files_unchanged=True, opencv_threads_restored=cv2.getNumThreads()==previous,
            scope='Old recordings and old 120Hz traces only; no new closed-loop physics or actuator execution'),
            indent=2, allow_nan=False)+'\n')
    print('complete', len(records), decisions, terminal, flush=True)


if __name__ == '__main__':
    main()
