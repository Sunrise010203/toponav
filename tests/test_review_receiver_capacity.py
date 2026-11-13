"""Finite native replay history; exhaustion is terminal, not evidence eviction."""
import json
from pathlib import Path
import subprocess

import pytest

from simulation.gazebo_receiver import ReceiverEvidence, MAX_RECEIVER_FRAMES
from simulation import gazebo_cross_validation as runner
from tests.test_review_receiver_frame import strict, echo
from tests.test_review_receiver_binding import push
from tests.test_review_receiver_freshness import adapter
from tests.test_review_gazebo_failure_capture import harness
from tests.test_review_gazebo_watchdog import packet

ROOT = Path(__file__).resolve().parents[1]
LIMIT = 4096
REASON = 'receiver_capacity_exceeded'
META = ('command_seq', 'deadline_steady_s', 'frame_id_hex', 'captured_sim_s', 'issued_steady_s')


def capacity_status(**changes):
    item = echo(reason=REASON, latched=True, wheels=[0., 0.], command_seq=LIMIT,
                frame_id_hex=f'f{LIMIT}'.encode().hex(), captured_sim_s=LIMIT*.001,
                issued_steady_s=10.+LIMIT*.0001, deadline_steady_s=10.+LIMIT*.0001+.8,
                wall_s=10.5, sim_s=LIMIT*.001)
    item.update(changes)
    return item


def full_evidence(latest=LIMIT):
    e = strict()
    for seq in range(1, latest+1):
        p = packet(request_id=seq, seq=seq, stamp=seq*.001, issued=10.+seq*.0001)
        e.record_submission(p, p['issued_steady_s']+.00001)
    return e


HARNESS = r'''
#include "simulation/native/receiver_status.hh"
#include <iostream>
#include <limits>
#include <string>
static_assert(toponav::ReceiverLease::kMaxCommandFrames == 4096);
int main(int argc, char **argv) {
  if (argc!=5) return 2;
  const unsigned count=std::stoul(argv[1]), width=std::stoul(argv[3]);
  const std::string action=argv[2];
  const double velocity=std::stod(argv[4]);
  toponav::ReceiverLease g("capacity-test");
  auto packet=[&](unsigned n) {
    toponav::Envelope p;
    p.kind="command";p.session="capacity-test";p.seq=n;
    p.frame=std::to_string(n);p.frame.resize(width,'x');
    p.captured=n*.001;p.issued=10.+n*.0001;p.expires=p.issued+.8;p.v=velocity;p.w=0.;
    return p;
  };
  auto emit=[&](bool ok, double now) {
    std::cout<<"{\"ok\":"<<(ok?"true":"false")<<",\"frame_count\":"<<g.FrameCount()
      <<",\"status\":"<<toponav::ReceiverStatus(g,"capacity-test",now,20.)<<"}\n";
  };
  for (unsigned i=1;i<=count;++i) {
    const auto p=packet(i);
    if (!g.Accept(p,p.issued)) return 3;
  }
  const double last=10.+count*.0001;
  emit(true,last);
  auto p=packet(count+1);double now=p.issued;bool ok=false;
  if (action=="none") return 0;
  if (action=="stop") {p.kind="stop";p.seq=999999;p.frame="not-accepted";p.v=p.w=0;}
  if (action=="idle") {p.kind="idle";p.seq=0;p.v=0;}
  if (action=="reuse_first") p.frame=packet(1).frame;
  if (action=="reuse_middle") p.frame=packet(2048).frame;
  if (action=="same_capture") p.captured=count*.001;
  if (action=="bad_seq") ++p.seq;
  if (action=="oversized") p.frame=std::string(257,'x');
  if (action=="empty") p.frame="";
  if (action=="session") p.session="other";
  if (action=="bad_velocity") p.v=.151;
  if (action=="nan_velocity") p.v=std::numeric_limits<double>::quiet_NaN();
  if (action=="future_issue") p.issued=now+.1;
  if (action=="expired_envelope") p.expires=now;
  if (action=="extend") p.expires=p.issued+.8001;
  if (action=="before_expiry" || action=="at_expiry") {
    now=g.deadline-(action=="before_expiry"?.0001:0.);p.issued=now;p.expires=now+.8;
  }
  if (action=="poll_expiry") {now=g.deadline;g.Poll(now);}
  else if (action=="regress") {now=last-.1;g.Poll(now);}
  else ok=g.Accept(p,now);
  emit(ok,now);
  // A rejected/malformed packet and STOP must never clear a terminal latch.
  p=packet(count+2);p.issued=now+.01;p.expires=p.issued+.8;
  ok=g.Accept(p,p.issued);emit(ok,p.issued);
  p.kind="stop";p.v=p.w=0;p.issued+=.01;
  ok=g.Accept(p,p.issued);emit(ok,p.issued);
}
'''


@pytest.fixture(scope='module')
def native(tmp_path_factory):
    folder = tmp_path_factory.mktemp('native-capacity')
    source, binary = folder/'probe.cc', folder/'probe'
    source.write_text(HARNESS)
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-I', str(ROOT), str(source), '-o', str(binary)], check=True, capture_output=True)
    def run(count=LIMIT, action='next', width=256, velocity=.15):
        result = subprocess.run([str(binary), str(count), action, str(width), str(velocity)],
                                capture_output=True, text=True, check=True)
        return [json.loads(line) for line in result.stdout.splitlines()]
    return run


@pytest.mark.parametrize('count', [0, 1, 2200, LIMIT-1, LIMIT])
@pytest.mark.parametrize('velocity', [0., .15])
def test_capacity_covers_current_task_budget_without_proactive_latch(native, count, velocity):
    item, = native(count, 'none', velocity=velocity)
    assert item['ok'] and item['frame_count'] == count
    assert not item['status']['latched'] and item['status']['command_seq'] == count
    assert item['status']['wheels'] == ([velocity, 0.] if count else [0., 0.])


@pytest.mark.parametrize('velocity', [0., .15])
@pytest.mark.parametrize('width', [16, 256])
def test_limit_accepts_last_slot_then_fails_closed_without_changing_accepted_metadata(native, width, velocity):
    before, rejected, later, stop = native(width=width, velocity=velocity)
    assert before['frame_count'] == LIMIT and not before['status']['latched']
    for item in (rejected, later, stop):
        assert not item['ok'] and item['frame_count'] == LIMIT
        status = item['status']
        assert status['latched'] and status['reason'] == REASON and status['wheels'] == [0., 0.]
        assert all(status[k] == before['status'][k] for k in META)


@pytest.mark.parametrize('velocity', [0., .15])
def test_capacity_does_not_steal_the_last_available_slot(native, velocity):
    before, accepted, full, stop = native(LIMIT-1, velocity=velocity)
    assert before['frame_count'] == LIMIT-1 and accepted['ok']
    assert accepted['frame_count'] == LIMIT and not accepted['status']['latched']
    assert not full['ok'] and full['frame_count'] == LIMIT and full['status']['reason'] == REASON
    assert stop['status']['reason'] == REASON


@pytest.mark.parametrize('count', [1, 2200, LIMIT])
def test_stop_at_full_capacity_remains_valid_and_needs_no_new_history_entry(native, count):
    before, stopped, rejected, stopped_again = native(count, 'stop')
    assert stopped['ok'] and stopped['status']['reason'] == 'remote_stop'
    for item in (stopped, rejected, stopped_again):
        assert item['frame_count'] == count and item['status']['wheels'] == [0., 0.]
        assert all(item['status'][k] == before['status'][k] for k in META)
    assert not rejected['ok'] and not stopped_again['ok']


@pytest.mark.parametrize('action', ['idle', 'reuse_first', 'reuse_middle', 'same_capture', 'bad_seq',
    'oversized', 'empty', 'session', 'bad_velocity', 'nan_velocity', 'future_issue', 'expired_envelope', 'extend'])
def test_full_history_still_rejects_original_invalid_envelopes_without_eviction(native, action):
    before, bad, later, stop = native(action=action)
    for item in (bad, later, stop):
        assert not item['ok'] and item['frame_count'] == LIMIT
        assert item['status']['reason'] == 'invalid_receiver_envelope'
        assert all(item['status'][k] == before['status'][k] for k in META)


@pytest.mark.parametrize('action,reason', [('poll_expiry', 'command_lease_expired'),
    ('at_expiry', 'command_lease_expired'), ('regress', 'steady_clock_regression'),
    ('before_expiry', REASON)])
def test_original_poll_and_lease_expiry_precede_capacity_and_remain_sticky(native, action, reason):
    before, rejected, later, stop = native(action=action)
    for item in (rejected, later, stop):
        assert item['status']['reason'] == reason and item['status']['wheels'] == [0., 0.]
        assert item['frame_count'] == LIMIT
        assert all(item['status'][k] == before['status'][k] for k in META)


def test_actual_native_capacity_status_enters_structural_audit_as_fault_not_unknown_schema(native):
    before, rejected, later, stop = native()
    e = ReceiverEvidence('capacity-test')
    for item in (before, rejected, later, stop): push(e, item['status'])
    summary = e.summary(0., 20.)
    assert not e.error and summary['fault_latched'] and summary['final_zero']
    assert summary['final_step_fresh'] and 'stationary' not in summary
    with pytest.raises(RuntimeError): e.require_fresh(stop['status']['wall_s'])


@pytest.mark.parametrize('final_seq', [LIMIT, LIMIT+1])
def test_capacity_fault_binds_only_prior_accepted_candidate_not_rejected_latest(final_seq):
    e = full_evidence(final_seq)
    push(e, capacity_status())
    summary = e.summary(0., LIMIT*.001)
    assert not e.error and summary['fault_latched'] and summary['final_zero']
    assert summary['submission_binding']['final_authority_bound'] == (final_seq == LIMIT)
    assert summary['submission_binding']['frame_echo_verified'] == (final_seq == LIMIT)
    with pytest.raises(RuntimeError): e.require_fresh(10.6)


@pytest.mark.parametrize('changes', [dict(latched=False), dict(wheels=[.1, 0.]),
    dict(command_seq=0, deadline_steady_s=0., frame_id_hex=None, captured_sim_s=None, issued_steady_s=None)])
def test_impossible_capacity_authority_never_grants_motion(changes):
    e = full_evidence(); item = capacity_status(**changes)
    push(e, item)
    # seq0 is not capacity exhaustion: a finite history cannot be full before a command.
    assert e.error and not e.events


def test_real_adapter_rejects_capacity_fault_but_preserves_zero_stop(monkeypatch):
    io = adapter(monkeypatch, None, now=10.6)
    io.receiver_evidence = full_evidence()
    push(io.receiver_evidence, capacity_status())
    with pytest.raises(RuntimeError): io.command(.1, 0.)
    io.actuator.request.assert_not_called()
    io.command(0., 0.); io.actuator.request.assert_called_once_with('stop')


def test_runner_capacity_fault_cancels_task_and_preserves_cleanup_and_failed_tick(tmp_path, harness):
    control, _, calls = harness
    original = runner.GazeboIO
    class IO(original):
        def __init__(self, **kw):
            super().__init__(**kw)
            self.receiver_evidence = full_evidence()
            push(self.receiver_evidence, capacity_status())
        def command(self, v, omega):
            if v or omega: self.receiver_evidence.require_fresh(10.6)
            return super().command(v, omega)
    runner.GazeboIO = IO
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=3)
    assert result['exception'] and not result['completed'] and control.cancelled
    assert result['controller_result']['state'] == 'aborted'
    assert result['failure_record']['pending_tick']['stage'] == 'command_submission'
    assert result['failure_record']['zero_command_submitted'] and result['cleanup_stop']['steps_completed'] == 10
    assert result['receiver_interlock']['fault_latched']
    assert not any(c[0] == 'command' and (c[1] or c[2]) for c in calls)


def test_capacity_is_native_fixed_policy_not_world_or_cli_tuning():
    # Existing CLI's longest declared task budget is 2200, smaller than 4096.
    source = (ROOT/'simulation/gazebo_cross_validation.py').read_text()
    assert 'args.max_ticks = 2200' in source
    plugin = (ROOT/'simulation/native/gazebo_receiver_interlock.cc').read_text()
    assert 'kMaxCommandFrames' not in plugin and 'FrameCount' not in plugin
    assert MAX_RECEIVER_FRAMES == LIMIT


@pytest.mark.parametrize('seq', [0, 1, LIMIT-1, LIMIT+1, 2**64-1])
def test_capacity_reason_cannot_claim_an_impossible_history_length(seq):
    e = ReceiverEvidence('capacity-test')
    item = capacity_status(command_seq=seq, session_id='capacity-test')
    if seq == 0:
        item.update(deadline_steady_s=0., frame_id_hex=None, captured_sim_s=None, issued_steady_s=None)
    push(e, item)
    assert e.error and not e.events
