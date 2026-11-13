"""Opt-in isolation evidence, not movement/braking/navigation acceptance."""
import json
from pathlib import Path
import subprocess

import pytest

from tools import straight_deadline_audit as audit
from simulation import gazebo_actuator as act


@pytest.fixture(scope='module')
def binary(tmp_path_factory):
    return audit.compile_receiver(tmp_path_factory.mktemp('deadline-native'))


@pytest.mark.parametrize('case', audit.CASES)
def test_real_zero_only_client_worker_and_independent_compiled_receiver(tmp_path, binary, case):
    original_popen = subprocess.Popen
    result = audit.run_one(case, tmp_path/case, binary)
    assert subprocess.Popen is original_popen
    assert result['error'] is None and result['expected_observations_met']
    assert result['all_children_reaped']
    assert len({result[k] for k in ('worker_pid', 'audit_pid', 'receiver_pid')}) == 3
    assert result['capture_to_motor_latency_s'] is None
    assert result['actual_capture_wall_s'] is None
    assert not result['physical_stop_measured'] and not result['wall_capture_freshness_proved']
    folder = tmp_path/case
    publications = audit.read_rows(folder/'publications.jsonl')
    assert publications and all(p['v'] == p['w'] == 0 for p in publications)
    native = audit.read_rows(folder/'receiver.jsonl')
    assert all(r['wheels'] == [0., 0.] for r in native)
    assert native[0]['event'] == 'ready' and native[-1]['event'] == 'exit'
    recv = audit.read_rows(folder/'receives.jsonl')[0]['wall_s']
    ack = result['first_ack']
    native_first = next(r for r in native if r['reason'] == 'accepted')
    assert result['issued_steady_s'] <= recv <= ack['published_steady_s'] <= result['first_ack_returned_s']
    assert native_first['received_steady_s'] >= result['issued_steady_s']
    assert ack['status']['deadline_steady_s'] == native_first['deadline_steady_s'] == result['deadline_steady_s']
    probe = json.loads((folder/'client-probe.json').read_text())
    assert probe['recent'][0]['receipt_argument_s'] <= result['first_ack_returned_s']
    if case == 'zero_slow_first':
        # This is an exposed limitation, NOT a desired wall-freshness guarantee.
        assert result['compute_finished_s']-result['compute_started_s'] > act.LEASE_S
        assert ack['accepted'] and native_first['reason'] == 'accepted'
    if case == 'zero_stopped_publisher':
        # Do not silently count the late worker as a timely zero publication.
        assert result['worker_expiry_lag_s'] is None or result['worker_expiry_lag_s'] > audit.ALLOWANCE_S
        assert 'worker_expiry_observed' not in result['checks']
        assert result['post_resume_status']['latched']
        assert result['post_resume_status']['wheels'] == [0., 0.]
    if case == 'zero_blocked_producer':
        assert result['client_failed'] and 'late_ack' not in result


@pytest.mark.parametrize('v,w', [(0.1, 0.), (0., .1), (-.1, 0.), (float('nan'), 0.)])
def test_private_publisher_refuses_nonzero_even_without_external_connection(v, w):
    with pytest.raises(ValueError, match='zero-only'):
        audit.wire_metadata({}, v, w)


def test_zero_packet_original_deadline_and_synthetic_sim_clock():
    packet = audit.zero_packet(1, 10.)
    assert packet['issued_steady_s'] == 10. and packet['expires_steady_s'] == 10.+act.LEASE_S
    assert packet['command']['v_m_s'] == packet['command']['omega_rad_s'] == 0.
    assert packet['command']['captured_sim_s'] == packet['sim_s'] == .1


@pytest.mark.parametrize('case', audit.CASES)
def test_missing_expiry_or_error_cannot_support_positive_observation(case):
    result = audit.evaluate(case, dict(error='startup failed'), [], [])
    assert not result['expected_observations_met']
    assert result['worker_expiry'] is result['receiver_expiry'] is None
    assert result['capture_to_motor_latency_s'] is None


def event(wall=10.8, **kw):
    return dict(reason='command_lease_expired', wall_s=wall, wheels=[0., 0.], **kw)


@pytest.mark.parametrize('lag', [-.001, .151, 1.])
def test_before_deadline_or_late_is_not_timely_expiry(lag):
    rows = [event(10.8+lag)]
    result = audit.evaluate('zero_quick', dict(deadline_steady_s=10.8), rows, rows)
    assert not result['expected_observations_met']


def test_explicit_late_worker_exclusion_does_not_excuse_late_receiver():
    result = dict(deadline_steady_s=10.8, publisher_stop_confirmed_s=10.1, resume_requested_s=11.2)
    out = audit.evaluate('zero_stopped_publisher', result, [event(11.2)], [event(11.2)])
    assert not out['expected_observations_met']


def test_expiry_alone_does_not_prove_held_velocity_became_physical_zero():
    result = audit.evaluate('zero_quick', dict(deadline_steady_s=10.8), [event()], [event()])
    assert result['expected_observations_met']
    assert not result['physical_stop_measured'] and not result['wall_capture_freshness_proved']


def test_bad_case_creates_no_child_or_folder(tmp_path):
    folder = tmp_path/'bad'
    with pytest.raises(ValueError, match='Unknown case'):
        audit.run_one('not-a-case', folder, None)
    assert not folder.exists()


@pytest.mark.parametrize('repeats', [0, -1, True, 1.5])
def test_bad_repeat_count_creates_no_output(tmp_path, repeats):
    with pytest.raises(ValueError, match='integer'):
        audit.run(tmp_path/'output', repeats)
    assert not (tmp_path/'output').exists()


def test_no_external_publisher_or_production_tuning():
    source = Path(audit.__file__).read_text()
    native = (audit.ROOT/'tools/isolated_receiver_timing.cc').read_text()
    assert 'from gz.' not in source and 'import rclpy' not in source
    assert 'ReceiverLease gate' in native and '#include "simulation/native/receiver_lease.hh"' in native
    assert 'act.LEASE_S =' not in source
    assert 'act.POLL_S =' not in source
    assert 'p.v != 0 || p.w != 0' in native


def test_exclusive_output_does_not_overwrite(tmp_path):
    p = tmp_path/'result.json'
    audit.write_json(p, dict(first=True))
    with pytest.raises(FileExistsError):
        audit.write_json(p, {})
    assert json.loads(p.read_text()) == dict(first=True)
