"""Exact public event replay; no simulator, sensor, image or audit is read."""
import argparse
import json
from pathlib import Path

from experiments.block7.common import file_hash, read_json, read_rows, write_json
from experiments.block7.runtime import EventClock
from experiments.block7.session import ExperimentConfig, ExperimentSession
from simulation.navigation_session import NavigationSession, SessionConfig


def replay(output, compare_core=False, _core=False):
    output = Path(output)
    header = read_json(output / 'header.json')
    if file_hash(output / 'public-map.json') != header['public_map_sha256']:
        raise ValueError('Public map hash changed')
    clock = EventClock(); clock.load(header['initial_clock'])
    config = dict(header['config'])
    if _core:
        if header['method'] != 'M1':
            raise ValueError('Only M1 is equivalent to the unmodified production core')
        config.pop('passive_recovery')
    cls, config_cls = (NavigationSession, SessionConfig) if _core else (ExperimentSession, ExperimentConfig)
    session = cls(read_json(output / 'public-map.json'), config=config_cls(**config),
                  clock=lambda: clock.mono, wall_ns=lambda: clock.wall, **header['constructor'])
    rows = read_rows(output / 'events.jsonl')
    errors = []
    allowed = {'observe', 'feedback', 'tick', 'missing_capture', 'abort'}
    for index, row in enumerate(rows):
        if row['index'] != index or row['kind'] not in allowed:
            raise ValueError('Invalid public event ordering or kind')
        clock.load(row['clock'])
        try:
            function = getattr(session, 'tick' if _core and row['kind'] == 'missing_capture' else row['kind'])
            actual = function(row['payload']) if row['payload'] is not None else function()
            exception = None
        except (ValueError, TypeError) as exc:
            actual = None; exception = {'type': type(exc).__name__, 'message': str(exc)}
        if actual != row['output'] or exception != row['exception']:
            errors.append({'event': index, 'reason': 'output_or_exception_mismatch'})
    expected_report = read_json(output / 'controller.json')
    if _core:
        expected_report['config'].pop('passive_recovery')
    if session.report() != expected_report:
        errors.append({'event': 'final', 'reason': 'controller_report_mismatch'})
    core = replay(output, _core=True) if compare_core else None
    if core and not core['passed']:
        errors.append({'event': 'core', 'reason': 'M1_not_equivalent_to_original_session'})
    return {'passed': not errors, 'events': len(rows), 'errors': errors,
            'original_core_equivalence_passed': core['passed'] if core else None,
            'truth_or_model_used': False, 'scope': 'public_session_replay_not_pixel_inference_reproduction'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--compare-core', action='store_true', help='M1 only: also replay original production session')
    args = parser.parse_args()
    result = replay(args.episode, compare_core=args.compare_core)
    if args.output:
        write_json(args.output, result)
    print(json.dumps(result))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
