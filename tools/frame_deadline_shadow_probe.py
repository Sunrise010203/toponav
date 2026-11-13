"""Opt-in private-process probes of a SHADOW design, never a motor publisher.

Synthetic exposure bounds, injected sleeps and private sockets only. No sensor
capture, controller work, production actuator integration or physical stopping.
"""
from dataclasses import asdict
import argparse
import json
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import time

from simulation.gazebo_actuator import POLL_S
from tools.frame_deadline_proposal import (
    DeadlineProposal, FrameEvidence, ViewEvidence, ResultBinding, Rejected,
)

CASES = ('unknown_capture', 'slow_first', 'held_command', 'retimestamped_result', 'uncertain_mapping', 'owner_stopped')
BLOCK_S = 1.1
ALLOWANCE_S = .15  # Observation only; never passed to the model.


def write_json(path, value):
    with Path(path).open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False); f.write('\n')


def packet(channel, values):
    raw = json.dumps(values, allow_nan=False).encode()
    if len(raw) > 16384 or channel.send(raw) != len(raw):
        raise RuntimeError('Invalid private shadow packet')


def receive(channel):
    if not select.select([channel], [], [], 5.)[0]:
        raise TimeoutError('Shadow owner response missing')
    raw = channel.recv(16385)
    if not raw or len(raw) > 16384:
        raise RuntimeError('Shadow owner channel closed/oversized')
    return json.loads(raw)


def worker(fd, log_path):
    owner = DeadlineProposal('shadow-only', 'synthetic-same-host-monotonic')
    with socket.socket(fileno=fd) as channel, Path(log_path).open('x', buffering=1) as log:
        def record(event):
            log.write(json.dumps(dict(event=event, wall_s=time.monotonic(), **owner.snapshot()))+'\n')
        def poll():
            before = owner.latched
            owner.poll(time.monotonic())
            if owner.latched and not before:
                record('independent_shadow_latch')
        packet(channel, dict(ready=True, pid=os.getpid(), motion_authority=False))
        while True:
            poll()
            if not select.select([channel], [], [], POLL_S)[0]:
                continue
            request = receive(channel)
            poll()  # A resumed owner polls before examining any late result.
            if request == {'op': 'quit'}:
                break
            response = {}
            try:
                if request.get('op') == 'admit' and set(request) == {'op', 'frame_id', 'sim_s', 'source', 'age_s'}:
                    now = time.monotonic()
                    age = request['age_s']
                    if type(age) not in (int, float) or not 0 <= age <= 1.:
                        owner.reject('invalid_synthetic_age')
                    views = (ViewEvidence('forward', 'a'*64, now, request['source'], now-age, now),)
                    frame = FrameEvidence('shadow-only', request['frame_id'], request['sim_s'],
                                          'synthetic-same-host-monotonic', views)
                    response['token'] = owner.admit(frame, now)
                    response['binding'] = asdict(owner.pending['binding'])
                elif request.get('op') == 'complete' and set(request) == {'op', 'token', 'binding'}:
                    binding = request['binding'].copy()
                    binding['view_digests'] = tuple(tuple(x) for x in binding['view_digests'])
                    response['permit'] = asdict(owner.complete(request['token'], ResultBinding(**binding), time.monotonic()))
                else:
                    owner.reject('caller_timestamp_or_unknown_fields_refused')
            except (Rejected, TypeError, KeyError, ValueError, AttributeError) as exc:
                owner.trip('malformed_shadow_request')
                response['error'] = str(exc)
            record('request_processed')
            packet(channel, dict(**response, state=owner.snapshot()))
        record('owner_exit')


def run_one(case, folder):
    if case not in CASES:
        raise ValueError('Unknown shadow case')
    folder.mkdir()
    a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    result = dict(case=case, error=None, actual_exposure_measured=False,
                  actual_controller_compute_measured=False, motion_authority=False)
    process = None
    with (folder/'worker.log').open('x') as log:
        try:
            process = subprocess.Popen([sys.executable, '-u', '-m', 'tools.frame_deadline_shadow_probe',
                '--worker', str(b.fileno()), '--log', str(folder/'owner.jsonl')],
                pass_fds=(b.fileno(),), stdout=log, stderr=subprocess.STDOUT)
            b.close()
            ready = receive(a)
            if ready != dict(ready=True, pid=process.pid, motion_authority=False):
                raise RuntimeError('Bad shadow startup identity')
            result['pid'] = process.pid
            def request(value):
                packet(a, value); return receive(a)
            def admit(frame_id='f1', sim_s=.1, source='bounded_exposure_mapping', age_s=.03):
                return request(dict(op='admit', frame_id=frame_id, sim_s=sim_s, source=source, age_s=age_s))
            first = result['first_admit'] = admit(source='receipt_only' if case == 'unknown_capture' else 'bounded_exposure_mapping',
                age_s=.2 if case == 'uncertain_mapping' else .03)
            if case != 'unknown_capture':
                result['first_deadline'] = first['state']['effective_deadline']
                pending = first
                if case in ('held_command', 'uncertain_mapping'):
                    result['early_complete'] = request(dict(op='complete', token=first['token'], binding=first['binding']))
                    if case == 'held_command':
                        pending = result['second_admit'] = admit('f2', .2, age_s=0.)
                if case == 'owner_stopped':
                    os.kill(process.pid, signal.SIGSTOP)
                    pid, status = os.waitpid(process.pid, os.WUNTRACED)
                    if pid != process.pid or not os.WIFSTOPPED(status):
                        raise RuntimeError('Owned shadow process did not stop')
                    result['owner_stop_confirmed_s'] = time.monotonic()
                result['block_started_s'] = time.monotonic()
                time.sleep(BLOCK_S)
                result['block_finished_s'] = time.monotonic()
                if case == 'owner_stopped':
                    result['resume_requested_s'] = time.monotonic()
                    os.kill(process.pid, signal.SIGCONT)
                value = dict(op='complete', token=pending['token'], binding=pending['binding'])
                if case == 'retimestamped_result':
                    value.update(issued_steady_s=time.monotonic(), expires_steady_s=time.monotonic()+.8)
                result['late_complete'] = request(value)
                result['post_expiry_new_frame'] = admit('new-after-expiry', .3, age_s=0.)
            packet(a, dict(op='quit'))
            process.wait(timeout=5.)
        except Exception as exc:
            result['error'] = f'{type(exc).__name__}: {exc}'
        finally:
            b.close(); a.close()
            if process is not None and process.poll() is None:
                os.kill(process.pid, signal.SIGCONT)
                process.kill(); process.wait(timeout=5.)
    result['child_returncode'] = process.returncode if process else None
    result['child_reaped'] = process is not None and process.returncode is not None
    rows = [json.loads(s) for s in (folder/'owner.jsonl').read_text().splitlines()] if (folder/'owner.jsonl').exists() else []
    hit = next((r for r in rows if r['event'] == 'independent_shadow_latch'), None)
    result['latch_event'] = hit
    checks = dict(no_unexpected_error=result['error'] is None, normal_child_exit=result['child_returncode'] == 0)
    if case == 'unknown_capture':
        checks['unknown_age_no_token'] = ('token' not in result.get('first_admit', {})
            and result.get('first_admit', {}).get('state', {}).get('reason') == 'exposure_age_unknown')
    else:
        lag = hit['wall_s']-result['first_deadline'] if hit else None
        result['latch_lag_s'] = lag
        checks['late_result_no_permit'] = 'permit' not in result.get('late_complete', {}) and result.get('late_complete', {}).get('state', {}).get('latched') is True
        checks['new_frame_cannot_rearm'] = 'token' not in result.get('post_expiry_new_frame', {}) and result.get('post_expiry_new_frame', {}).get('state', {}).get('latched') is True
        if case == 'owner_stopped':
            checks['paused_owner_not_timely'] = (lag is not None and lag > ALLOWANCE_S
                and hit['wall_s'] >= result['resume_requested_s'])
            result['timely_latch_proved'] = False
        else:
            checks['independent_latch_during_block'] = (lag is not None and 0 <= lag < ALLOWANCE_S
                and result['block_started_s'] <= hit['wall_s'] < result['block_finished_s'])
        if case == 'held_command':
            state = result.get('second_admit', {}).get('state', {})
            checks['new_pending_preserves_old_hold'] = (state.get('effective_deadline') == result.get('first_deadline')
                and state.get('pending_deadline', -1.) > result.get('first_deadline', float('inf')))
        if case == 'uncertain_mapping':
            permit = result.get('early_complete', {}).get('permit', {})
            checks['early_zero_shadow_permit_capped_to_oldest_exposure'] = (permit.get('expires_steady_s') == result.get('first_deadline')
                and permit.get('motion_authority') is False)
    result['checks'] = checks
    result['declared_observations_met'] = all(checks.values())
    write_json(folder/'result.json', result)
    return result


def run(output, repeats=3):
    if type(repeats) is not int or repeats < 1:
        raise ValueError('Positive integer repeats required')
    output = Path(output).resolve(); output.mkdir(exist_ok=False)
    results = []
    for repeat in range(repeats):
        for case in (CASES if repeat % 2 == 0 else CASES[::-1]):
            results.append(run_one(case, output/f'{repeat}-{case}'))
    result = dict(results=results, declared_observations_met=all(r['declared_observations_met'] for r in results),
        all_children_reaped=all(r['child_reaped'] for r in results), motion_authority=False,
        scope='shadow design observations only; not production repair, sensor provenance, or physical stop')
    write_json(output/'summary.json', result)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path); p.add_argument('--repeats', type=int, default=3)
    p.add_argument('--worker', type=int); p.add_argument('--log', type=Path)
    args = p.parse_args()
    if args.worker is not None:
        worker(args.worker, args.log); return 0
    if args.output is None:
        p.error('--output required')
    result = run(args.output, args.repeats)
    print(json.dumps(dict(declared_observations_met=result['declared_observations_met'], all_children_reaped=result['all_children_reaped'])))
    return 0 if result['declared_observations_met'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
