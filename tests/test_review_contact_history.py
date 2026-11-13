"""Bound contact prefix without confusing omitted entries with wall collisions."""
from collections import defaultdict
import json
import threading
from types import SimpleNamespace as NS

import pytest

from simulation import gazebo_cross_validation as runner
from tests.test_review_gazebo_failure_capture import harness
from tests.test_review_gazebo_post_stop import terminal_io
from tests.test_review_gazebo_watchdog import client
from tests.test_review_receiver_freshness import adapter

IO = runner.GazeboIO
LIMIT = 65536


def entry(first='robot::base_collision', second='wall_0::link::collision'):
    return NS(collision1=NS(name=first), collision2=NS(name=second))


def message(entries=None, sec=1):
    return NS(header=NS(stamp=NS(sec=sec, nsec=0)), contact=[entry()] if entries is None else entries)


def io_at(prefix=()):
    io = IO.__new__(IO); io.condition = threading.Condition()
    io.contacts = list(prefix); io._contact_not_retained = 0
    io.contact_messages = defaultdict(int); io.error = None
    return io


@pytest.fixture(scope='module')
def prefix():
    io = io_at(); msg = message()
    for _ in range(LIMIT): IO.contact(io, 'base', msg)
    assert len(io.contacts) == LIMIT and not io.error and not io._contact_not_retained
    return tuple(io.contacts)


def test_actual_full_prefix_is_not_preemptively_faulted(prefix):
    io = io_at(prefix)
    assert runner.MAX_GAZEBO_CONTACTS == LIMIT and runner.MAX_CONTACT_NOT_RETAINED == 2**64-1
    assert len(io.contacts) == LIMIT and not io.error and not io._contact_not_retained
    assert io.contacts[0] == io.contacts[-1] == dict(link='base', sim_s=1., collision1='robot::base_collision', collision2='wall_0::link::collision')


@pytest.mark.parametrize('remaining', [0, 1, 2])
def test_multi_entry_message_cannot_overflow_actual_limit(prefix, remaining):
    io = io_at(prefix[:LIMIT-remaining]); IO.contact(io, 'left_wheel', message([entry()]*3, sec=2))
    assert len(io.contacts) == LIMIT and io._contact_not_retained == 3-remaining and io.error
    if remaining:
        assert io.contacts[-1]['link'] == 'left_wheel' and io.contacts[-1]['sim_s'] == 2.
    assert io.contact_messages['left_wheel'] == 1


@pytest.mark.parametrize('omitted', [NS(), NS(collision1=None), NS(collision1='x'*2_000_001), entry('floor', 'wheel')])
def test_exhausted_input_is_not_parsed_classified_or_copied(prefix, omitted):
    io = io_at(prefix); before = list(io.contacts)
    IO.contact(io, 'base', NS(contact=[omitted]))  # No header, deliberately.
    assert io.contacts == before and io._contact_not_retained == 1
    assert io.error == 'Gazebo contact history capacity exceeded'


def test_nonwall_contacts_before_limit_do_not_consume_slots(prefix):
    io = io_at(prefix[:-1]); IO.contact(io, 'base', message([entry('floor', 'wheel')]*3))
    assert len(io.contacts) == LIMIT-1 and not io._contact_not_retained and not io.error
    IO.contact(io, 'base', message())
    assert len(io.contacts) == LIMIT and not io._contact_not_retained and not io.error


def test_empty_callback_at_limit_is_not_an_omitted_contact(prefix):
    io = io_at(prefix); IO.contact(io, 'base', NS(contact=[]))
    assert not io._contact_not_retained and not io.error and len(io.contacts) == LIMIT
    assert io.contact_messages['base'] == 1


@pytest.mark.parametrize('first,second', [('wall_1', 'robot'), ('robot', 'wall_2'), ('wall_1', 'wall_2')])
def test_original_wall_name_matching_retains_one_record_per_contact(first, second):
    io = io_at(); IO.contact(io, 'front_caster', message([entry(first, second)]))
    assert io.contacts == [dict(link='front_caster', sim_s=1., collision1=first, collision2=second)]


@pytest.mark.parametrize('original', ['camera failed first', 'earlier actuator fault'])
def test_capacity_preserves_original_io_error(prefix, original):
    io = io_at(prefix); io.error = original; IO.contact(io, 'base', NS(contact=[NS()]))
    assert io.error == original and io._contact_not_retained == 1


def test_counter_saturates_and_remains_an_unvalidated_entry_count(prefix):
    io = io_at(prefix); io._contact_not_retained = 2**64-2
    for _ in range(5): IO.contact(io, 'base', NS(contact=[NS()]))
    assert io._contact_not_retained == 2**64-1 and len(io.contacts) == LIMIT


def test_original_condition_serializes_concurrent_last_slot(prefix):
    io = io_at(prefix[:-1]); msg = message()
    threads = [threading.Thread(target=IO.contact, args=(io, 'base', msg)) for _ in range(8)]
    for t in threads: t.start()
    for t in threads: t.join(2.); assert not t.is_alive()
    assert len(io.contacts) == LIMIT and io._contact_not_retained == 7 and io.error


def test_capacity_notifies_camera_waiter_under_the_condition(prefix):
    io = io_at(prefix); woke = threading.Event(); ready = threading.Event()
    def wait():
        with io.condition:
            ready.set(); io.condition.wait_for(lambda: io.error is not None, timeout=2.); woke.set()
    thread = threading.Thread(target=wait); thread.start(); assert ready.wait(2.)
    IO.contact(io, 'base', NS(contact=[NS()]))
    thread.join(2.); assert not thread.is_alive() and woke.is_set() and io.error
    with pytest.raises(RuntimeError, match='contact history capacity'): io.next_frame(-1)


@pytest.mark.parametrize('velocity', [(.1, 0.), (0., .3), (.1, -.3)])
def test_real_adapter_rejects_nonzero_before_receiver_and_actuator(monkeypatch, velocity):
    io = adapter(monkeypatch, None, now=10.2); io._contact_not_retained = 1
    with pytest.raises(RuntimeError, match='contact history capacity'): io.command(*velocity)
    io.actuator.request.assert_not_called(); assert not io.receiver_evidence.error
    io.command(0., 0.); io.actuator.request.assert_called_once_with('stop')


def test_actual_ipc_stop_ack_remains_healthy_after_contact_fault(client, monkeypatch):
    obj, _, _ = client; io = adapter(monkeypatch, None, now=10.2)
    io.actuator = obj; io._contact_not_retained = 1; io.command_context = None
    io.command(0., 0.)
    assert not obj.failed and obj.channel.fileno() >= 0 and obj.status['latched'] and obj.status['wheels'] == [0., 0.]


def test_runner_cancels_nonzero_and_retains_contact_prefix_and_cleanup(tmp_path, harness, prefix):
    control, _, calls = harness; original = runner.GazeboIO
    class Adapter(original):
        def __init__(self, **kw):
            super().__init__(**kw); self.contacts = list(prefix); self._contact_not_retained = 1
        def command(self, v, omega):
            if v or omega: raise RuntimeError('Gazebo contact history capacity exceeded')
            return super().command(v, omega)
    runner.GazeboIO = Adapter
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=1)
    assert not result['completed'] and control.cancelled and result['controller_result']['state'] == 'aborted'
    assert result['failure_record']['pending_tick']['stage'] == 'command_submission'
    assert result['failure_record']['zero_command_submitted'] and result['cleanup_stop']['steps_completed'] == 10
    assert not any(c[0] == 'command' and (c[1] or c[2]) for c in calls)
    audit = result['audit']; assert audit['observed_wall_contacts'] == LIMIT and audit['contact_history']['retained'] == LIMIT
    assert audit['contact_history']['capacity_exceeded'] and audit['contact_history']['not_retained'] == 1
    assert not audit['collision_free_certified']
    assert len(json.loads((tmp_path/'capture/audit/contacts.json').read_text())) == LIMIT


@pytest.mark.parametrize('count', [1, 2**64-1])
def test_contact_incompleteness_does_not_erase_real_stationary_pose_facts(tmp_path, monkeypatch, harness, prefix, count):
    terminal_io(monkeypatch, harness); original = runner.GazeboIO
    class Adapter(original):
        def step(self, iterations):
            super().step(iterations)
            if self.steps == 16:
                self.contacts = list(prefix); self._contact_not_retained = count
    monkeypatch.setattr(runner, 'GazeboIO', Adapter)
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=1)
    assert not result['completed'] and 'contact audit incomplete' in result['exception']
    assert result['controller_result']['state'] == 'visual_stop' and result['audit']['normal_geometric_stop_pass']
    assert result['cleanup_stop']['steps_completed'] == 10 and result['cleanup_stop']['error'] is None
    assert result['audit']['stationary'] and result['audit']['post_stop_pose_pair_fresh']
    assert result['audit']['final_speed_m_s'] == result['audit']['final_yaw_rate_rad_s'] == 0.
    h = result['audit']['contact_history']
    assert h['retained'] == LIMIT and h['not_retained'] == count and h['not_retained_count_is_lower_bound'] is (count == 2**64-1)


def test_contact_summary_never_overwrites_primary_execution_exception(tmp_path, harness):
    _, fault, _ = harness; fault.kind = 'perception'; original = runner.GazeboIO
    class Adapter(original):
        def step(self, iterations):
            super().step(iterations)
            if self.steps >= 7: self._contact_not_retained = 1
    runner.GazeboIO = Adapter
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=3)
    assert 'backend failed' in result['exception'] and not result['completed']
    assert result['audit']['contact_history']['capacity_exceeded']


def test_normal_terminal_contract_remains_possible_without_contact_truncation(tmp_path, monkeypatch, harness):
    terminal_io(monkeypatch, harness)
    result = runner.run(tmp_path/'capture', scenario='left', max_ticks=1)
    assert result['completed'] and result['audit']['stationary']
    h = result['audit']['contact_history']
    assert h['limit'] == LIMIT and h['retained'] == h['not_retained'] == 0 and not h['capacity_exceeded']
    assert not result['audit']['collision_free_certified']
