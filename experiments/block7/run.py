"""Bounded, serial offline runner. Default development smoke, no final test.

Exit 0: bookkeeping/evidence valid, including expected refusals/timeouts.
Exit 1: incomplete run or evidence/infrastructure failure. Exit 2: bad input.
Navigation success is a per-episode field, never inferred from batch exit 0.
"""
import argparse
import csv
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

from experiments.block7.common import CONFIG, ROOT, atomic_json, check_resources, file_hash, read_json, read_rows, write_json
from experiments.block7.replay import replay
from experiments.block7.runtime import development_task, episode
from experiments.block7.validate import (freeze_candidate, ledger_summary, validate_suite,
                                         verify_files, verify_freeze)


def safe_child(request, output, timeout_s):
    """Bound the entire import/model/render process, not only the event loop."""
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', OMP_NUM_THREADS='2',
               OPENBLAS_NUM_THREADS='2', MKL_NUM_THREADS='2', MPLBACKEND='Agg')
    stdout_path = Path(output).parent / (Path(output).name + '.stdout.log')
    with stdout_path.open('x') as stdout:
        process = subprocess.Popen([sys.executable, '-m', 'experiments.block7.run', '_episode',
                                    '--request', str(request), '--output', str(output)],
                                   cwd=ROOT, env=env, stdout=stdout, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            return process.wait(timeout=timeout_s), False
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
            return process.returncode, True
        except BaseException:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
            raise


def batch(output, suite=None, split='development', freeze=None, allow_final_test=False):
    """Every planned row exists as not_run before any child is spawned."""
    if split not in {'development', 'validation', 'test'}:
        raise ValueError('Unknown split')
    if split != 'development':
        if suite is None or not validate_suite(suite)['passed']:
            raise ValueError('A valid prepared suite is required')
    if split == 'test':
        if not allow_final_test or freeze is None:
            raise ValueError('Final test is locked: explicit --allow-final-test and --freeze required')
        binding = verify_freeze(freeze)
        if Path(binding['suite']).resolve() != Path(suite).resolve():
            raise ValueError('Suite differs from candidate binding')
    elif allow_final_test:
        raise ValueError('Final-test flag only applies to test split')
    resources = check_resources()
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
    (output / 'episodes').mkdir(); (output / 'requests').mkdir()
    authorization = None
    if split == 'test':
        receipt = Path(suite) / 'FIRST_N_FINAL_GENERATION_STARTED.json'
        write_json(receipt, {'scope': 'N_final_test', 'freeze_sha256': file_hash(freeze),
                             'explicit_final_run_flag': True, 'output': str(output),
                             'created_at': datetime.datetime.now(datetime.timezone.utc).isoformat()})
        authorization = {'freeze': str(Path(freeze).resolve()), 'receipt': str(receipt.resolve())}
    tasks = ([development_task(factor, 'smoke') for factor in ('normal', 'drop_once')]
             if split == 'development' else [t for t in read_rows(Path(suite) / 'tasks.jsonl') if t['split'] == split])
    jobs = [(task, method) for task in tasks
            for method in task.get('method_order', ['M0', 'M1', 'M2', 'M3'])]
    ledger = {'scope': 'atomic_discrete_only', 'split': split, 'resources_at_start': resources,
              'suite': str(Path(suite).resolve()) if suite else None, 'episodes': [
                  {'task_id': task['task_id'], 'method': method, 'factor': task['factor'],
                   'physical_reachable': task['physical_reachable'], 'status': 'not_run',
                   'reason': 'not_started', 'output': f'episodes/{index:05}'}
                  for index, (task, method) in enumerate(jobs)]}
    atomic_json(output / 'ledger.json', ledger)
    budget = yaml.safe_load((CONFIG / 'budget.yaml').read_text())
    write_json(output / 'run-config.json', {'split': split, 'suite': ledger['suite'], 'freeze': freeze,
               'authorization': authorization, 'budget_sha256': file_hash(CONFIG / 'budget.yaml'),
               'methods_sha256': file_hash(CONFIG / 'methods.yaml'), 'command': sys.argv,
               'model_path': yaml.safe_load((CONFIG / 'methods.yaml').read_text())['shared']['model_path']})
    started = time.monotonic()
    try:
        for index, ((task, method), row) in enumerate(zip(jobs, ledger['episodes'])):
            check_resources(start=False)
            if time.monotonic() - started >= budget['resources']['max_wall_hours_per_stage'] * 3600:
                row['reason'] = 'stage_wall_budget'; break
            if sum(p.stat().st_size for p in output.rglob('*') if p.is_file()) >= budget['resources']['max_new_evidence_bytes']:
                row['reason'] = 'storage_budget'; break
            request = output / 'requests' / (str(index) + '.json')
            write_json(request, {'task': task, 'method': method, 'suite': str(suite) if suite else None,
                                 'final_authorization': authorization})
            destination = output / row['output']
            code, timed_out = safe_child(request, destination, budget['navigation']['limits']['external_wall_timeout_s'])
            row.update(exit_code=code)
            if timed_out:
                row.update(status='timeout', reason='external_process_timeout')
            elif not (destination / 'result.json').is_file():
                row.update(status='error', reason='missing_child_result')
            else:
                result = read_json(destination / 'result.json')
                row.update(status=result['status'], reason=result['reason'])
                replayed = replay(destination, compare_core=method == 'M1')
                write_json(destination / 'public-replay.json', replayed)
                integrity = verify_files(destination, read_json(destination / 'manifest.json'))
                if code != 0 or integrity or not replayed['passed']:
                    row.update(status='error', reason='exit_integrity_or_replay_failure')
                if result['false_arrival'] or result['mislocalized_commands']:
                    row.update(status='error', reason='false_arrival_or_wrong_pose_command')
            atomic_json(output / 'ledger.json', ledger)
            print(json.dumps({'episode': index + 1, 'planned': len(jobs), **row}), flush=True)
            # Safe refusal/timeout does not erase other denominator rows; true
            # infrastructure/evidence faults stop the stage per frozen protocol.
            if row['status'] == 'error':
                break
    except BaseException as exc:
        write_json(output / 'INTERRUPTED.json', {'type': type(exc).__name__, 'message': str(exc)})
        atomic_json(output / 'ledger.json', ledger)
        raise
    finally:
        atomic_json(output / 'ledger.json', ledger)
        summary = ledger_summary(ledger)
        if split == 'development':
            summary['smoke_expected_outcomes_passed'] = all(
                row['status'] == ('success' if task['factor'] == 'normal' or method in {'M1', 'M2'} else 'refused')
                for (task, method), row in zip(jobs, ledger['episodes']))
        write_json(output / 'summary.json', summary)
        with (output / 'episodes.csv').open('x') as file:
            fields = ['task_id', 'method', 'factor', 'physical_reachable', 'status', 'reason', 'output']
            writer = csv.DictWriter(file, fieldnames=fields, extrasaction='ignore')
            writer.writeheader(); writer.writerows(ledger['episodes'])
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest='command', required=True)
    smoke = subs.add_parser('smoke'); smoke.add_argument('--output', type=Path, required=True)
    run = subs.add_parser('navigation'); run.add_argument('--output', type=Path, required=True)
    run.add_argument('--suite', type=Path, required=True); run.add_argument('--split', choices=['validation', 'test'], required=True)
    run.add_argument('--freeze', type=Path); run.add_argument('--allow-final-test', action='store_true')
    frozen = subs.add_parser('freeze'); frozen.add_argument('--suite', type=Path, required=True)
    frozen.add_argument('--qualification', type=Path, required=True); frozen.add_argument('--output', type=Path, required=True)
    child = subs.add_parser('_episode'); child.add_argument('--request', type=Path, required=True)
    child.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.command == '_episode':
            request = read_json(args.request)
            result = episode(output=args.output, **request)
            return 1 if result['error'] else 0
        if args.command == 'freeze':
            freeze_candidate(args.suite, args.output, args.qualification)
            return 0
        if args.command == 'smoke':
            result = batch(args.output)
        else:
            result = batch(args.output, args.suite, args.split,
                           str(args.freeze.resolve()) if args.freeze else None, args.allow_final_test)
        return 0 if (result['complete'] and not result['counts']['error'] and
                     result.get('smoke_expected_outcomes_passed', True)) else 1
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        print(json.dumps({'status': 'invalid_or_blocked', 'error': str(exc)}))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
