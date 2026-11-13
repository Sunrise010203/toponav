"""Real OS pipes, synthetic helper replies; no physics/navigation acceptance."""
from contextlib import contextmanager
import json
import subprocess
import sys
import threading
import time
from types import SimpleNamespace as NS

import pytest

from simulation import gazebo_cross_validation as runner


@contextmanager
def helper(script):
    process = subprocess.Popen([sys.executable, '-u', '-c', script],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    try:
        yield process
    finally:
        if process.poll() is None: process.kill()
        try:
            process.stdin.close()
        except OSError:
            pass
        process.stdout.close()
        process.wait(timeout=5)


def io_for(process, timeout=1.):
    io = runner.GazeboIO.__new__(runner.GazeboIO)
    io.stepper = process
    io.condition = threading.Condition()
    io.now_ns = 0
    # The baseline uses only stepper.readline; the repaired runner delegates
    # communication to the bounded client. No baseline code is replaced.
    if hasattr(runner, 'StepServiceClient'):
        io._step_service = runner.StepServiceClient(process, timeout_s=timeout)
    return io


def advance(io):
    timer = threading.Timer(.015, lambda: setattr(io, 'now_ns', 100_000_000))
    timer.start()
    return timer


@pytest.mark.parametrize('partial', [False, True])
def test_missing_or_partial_helper_line_cannot_bypass_reply_deadline(partial):
    prefix = "print('{\"ok\":', end='', flush=True)" if partial else 'pass'
    suffix = "print('true}', flush=True)" if partial else "print('{\"ok\": true}', flush=True)"
    script = f"import sys,time\nsys.stdin.readline()\n{prefix}\ntime.sleep(.4)\n{suffix}\n"
    with helper(script) as process:
        io = io_for(process, timeout=.08)
        timer = advance(io)
        started = time.monotonic()
        try:
            with pytest.raises(TimeoutError, match='helper'):
                io.step(12)
            assert time.monotonic()-started < .35
            assert io._step_service.failed
        finally:
            timer.join()


def test_late_ack_is_not_consumed_as_cleanup_reply(tmp_path):
    requests = tmp_path/'requests'
    script = ("import sys,time\n"
              f"log=open({str(requests)!r},'w')\n"
              "for line in sys.stdin:\n"
              " log.write(line);log.flush()\n"
              " time.sleep(.3)\n"
              " print('{\"ok\": true}',flush=True)\n")
    with helper(script) as process:
        io = io_for(process, timeout=.08)
        timer = advance(io)
        try:
            try: io.step(12)
            except TimeoutError: pass
            time.sleep(.35)  # The old ACK has now actually reached the pipe.
            # Also let the baseline's next physics interval advance: its
            # missing failure latch must be exposed without a 30s clock wait.
            following = threading.Timer(.015, lambda: setattr(io, 'now_ns', 200_000_000))
            following.start()
            try:
                with pytest.raises(RuntimeError, match='unavailable'):
                    io.step(12)
            finally:
                following.join()
            assert len(requests.read_text().splitlines()) == 1
        finally:
            timer.join()


@pytest.mark.parametrize('reply', ['{"ok": false}', 'not-json', '[]', '{"ok": "yes"}', '{"ok": 1}'])
def test_bad_helper_response_latches_transport_failure(reply, tmp_path):
    requests = tmp_path/'requests'
    script = ("import sys\n"+f"log=open({str(requests)!r},'w')\n"
              "for line in sys.stdin:\n log.write(line);log.flush()\n"+f" print({reply!r},flush=True)\n")
    with helper(script) as process:
        io = io_for(process)
        timer = advance(io)
        try:
            with pytest.raises(RuntimeError, match='helper'):
                io.step(12)
            with pytest.raises(RuntimeError, match='unavailable'):
                io.step(12)
            assert len(requests.read_text().splitlines()) == 1
        finally:
            timer.join()


def test_eof_is_explicit_helper_failure_not_a_json_accident():
    with helper('import sys\nsys.stdin.readline()\n') as process:
        io = io_for(process)
        with pytest.raises(RuntimeError, match='helper.*closed'):
            io.step(12)
        with pytest.raises(RuntimeError, match='unavailable'):
            io.step(12)


def test_dead_helper_write_does_not_raise_again_in_pipe_cleanup():
    with helper('pass') as process:
        process.wait(timeout=5)
        io = io_for(process)
        with pytest.raises(BrokenPipeError):
            io.step(12)
        # This is the same close used by run(). A buffered failed write must
        # not throw here and discard the already captured primary exception.
        process.stdin.close()
        assert process.stdin.closed
        with pytest.raises(RuntimeError, match='unavailable'):
            io.step(12)


def test_acknowledged_but_stalled_clock_prevents_more_queued_steps(monkeypatch, tmp_path):
    requests = tmp_path/'requests'
    script = ("import sys\n"+f"log=open({str(requests)!r},'w')\n"
              "for line in sys.stdin:\n log.write(line);log.flush()\n print('{\"ok\": true}',flush=True)\n")
    with helper(script) as process:
        io = io_for(process, timeout=1000.)
        clock = NS(now=0.)
        def monotonic():
            clock.now += 31.
            return clock.now
        monkeypatch.setattr(runner, 'time', NS(monotonic=monotonic))
        with pytest.raises(TimeoutError, match='clock did not advance'):
            io.step(12)
        with pytest.raises(RuntimeError, match='unavailable'):
            io.step(12)
        assert len(requests.read_text().splitlines()) == 1


def test_valid_reply_still_requires_and_accepts_real_clock_advance():
    with helper("import sys\nsys.stdin.readline()\nprint('{\"ok\": true}',flush=True)\n") as process:
        io = io_for(process, timeout=1.)
        timer = advance(io)
        try:
            io.step(12)
            assert io.now_ns == 100_000_000
            assert not io._step_service.failed
        finally:
            timer.join()


@pytest.mark.parametrize('script', ["import time;time.sleep(.5)",
                                   "print('{\"ready\":',end='',flush=True);import time;time.sleep(.5)",
                                   "print('{\"ready\": \"yes\"}',flush=True)"])
def test_startup_handshake_is_bounded_and_cleans_owned_helper(script, monkeypatch):
    # Start exactly the helper child, without native imports or world creation.
    original = subprocess.Popen
    processes = []
    def spawn(*args, **kwargs):
        process = original([sys.executable, '-u', '-c', script], **kwargs)
        processes.append(process)
        return process
    monkeypatch.setattr(runner.subprocess, 'Popen', spawn)
    monkeypatch.setattr(runner, 'STEP_HELPER_TIMEOUT_S', .08, raising=False)
    with pytest.raises((RuntimeError, TimeoutError), match='helper'):
        runner.start_step_service()
    assert processes and processes[0].poll() is not None


def test_valid_startup_and_multiple_replies_remain_usable():
    script = ("import sys\nprint('{\"ready\": true}',flush=True)\n"
              "for line in sys.stdin:\n print('{\"ok\": true}',flush=True)\n")
    with helper(script) as process:
        client = runner.StepServiceClient(process, timeout_s=1.)
        client.receive('ready')
        client.request(12)
        client.request(12)
        assert not client.failed
