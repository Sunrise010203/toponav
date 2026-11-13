"""Registration contracts with fake transport; not sensor delivery/stop proof."""
import sys
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from simulation import gazebo_cross_validation as runner
from simulation import gazebo_actuator, gazebo_receiver

IO = runner.GazeboIO
LINKS = ('base', 'left_wheel', 'right_wheel', 'front_caster', 'rear_caster')


def topics(route=False):
    return ['/toponav/receiver_status',
            *('/toponav/camera/'+v for v in (*runner.VIEWS, 'up', 'down') if route or v in runner.VIEWS),
            '/toponav/overview', '/world/toponav/stats', '/world/toponav/clock', '/model/robot/pose',
            *(f'/world/toponav/model/robot/link/{link}/sensor/{link}_contact/contact' for link in LINKS)]


def construction(monkeypatch, *, failed_topic=None, raises=False):
    """Exercise the real constructor while owning no actual node/worker/helper."""
    calls = []; callbacks = {}; types = {}
    def subscribe(kind, topic, callback):
        calls.append(topic); callbacks[topic] = callback; types[topic] = kind
        if topic == failed_topic and raises: raise RuntimeError('transport registration raised')
        return topic != failed_topic
    node = NS(subscribe=subscribe, advertise=Mock(return_value=Mock()))
    message_types = [type(name, (), {}) for name in ('Image', 'Pose_V', 'Contacts', 'Stats', 'Control', 'Boolean', 'Twist')]
    monkeypatch.setattr(runner, 'native_imports', lambda: (lambda: node, *message_types))
    for leaf, name in [('stringmsg', 'StringMsg'), ('clock', 'Clock')]:
        module = ModuleType('gz.msgs10.'+leaf+'_pb2'); setattr(module, name, type(name, (), {}))
        monkeypatch.setitem(sys.modules, module.__name__, module)
    verify = Mock(); monkeypatch.setattr(gazebo_receiver, 'verify_binary', verify)
    actor = Mock(); helper = Mock()
    def make_actor(*args, **kwargs):
        assert calls == topics(len(calls) == 16)
        return actor
    actor_factory = Mock(side_effect=make_actor); step_factory = Mock(return_value=helper)
    monkeypatch.setattr(gazebo_actuator, 'ActuatorClient', actor_factory)
    monkeypatch.setattr(runner, 'start_step_service', step_factory)
    return NS(calls=calls, callbacks=callbacks, types=types, node=node, verify=verify,
              actor=actor, actor_factory=actor_factory, helper=helper, step_factory=step_factory)


@pytest.mark.parametrize('route', [False, True])
@pytest.mark.parametrize('topic', topics(True))
def test_failed_registration_never_constructs_actuator_or_helper(monkeypatch, route, topic):
    if not route and topic in ('/toponav/camera/up', '/toponav/camera/down'):
        # This topic is intentionally absent, not a skipped test.
        h = construction(monkeypatch, failed_topic=topic)
        IO(route=route, session_id='test', actuator_audit='audit')
        assert h.calls == topics(route) and h.actor_factory.call_count == h.step_factory.call_count == 1
        return
    h = construction(monkeypatch, failed_topic=topic)
    with pytest.raises(RuntimeError, match='Could not subscribe') as caught:
        IO(route=route, session_id='test', actuator_audit='audit')
    expected = ('Could not subscribe to native receiver interlock' if topic == '/toponav/receiver_status'
                else 'Could not subscribe to '+topic.rsplit('/', 1)[-1] if topic.startswith('/toponav/camera/')
                else 'Could not subscribe to '+topic)
    assert str(caught.value) == expected
    assert h.calls == topics(route)[:topics(route).index(topic)+1]
    h.actor_factory.assert_not_called(); h.step_factory.assert_not_called(); h.node.advertise.assert_not_called()
    h.verify.assert_called_once_with()


@pytest.mark.parametrize('route', [False, True])
def test_success_preserves_all_topics_message_types_callbacks_and_constructor_arguments(monkeypatch, route):
    h = construction(monkeypatch); obj = IO(route=route, session_id='test', actuator_audit='audit')
    assert h.calls == topics(route) and len(h.calls) == (16 if route else 14)
    assert obj.error is None and obj.poses == obj.contacts == [] and obj.clock_messages == 0
    assert tuple(obj.cameras.views) == (*runner.VIEWS, 'up', 'down') if route else tuple(obj.cameras.views) == runner.VIEWS
    for topic, method in [('/toponav/overview', 'overview_image'), ('/world/toponav/stats', 'stats'),
                          ('/world/toponav/clock', 'clock'), ('/model/robot/pose', 'pose'),
                          ('/toponav/receiver_status', 'receiver_status')]:
        assert h.callbacks[topic] == getattr(obj, method)
    for topic, kind in h.types.items():
        expected = ('StringMsg' if topic.endswith('receiver_status') else 'Image' if topic.startswith('/toponav/')
                    else 'Stats' if topic.endswith('/stats') else 'Clock' if topic.endswith('/clock')
                    else 'Pose_V' if topic.endswith('/pose') else 'Contacts')
        assert kind.__name__ == expected
    h.actor_factory.assert_called_once_with('test', 'audit', submission_observer=obj._record_actuator_submission)
    h.step_factory.assert_called_once_with()
    assert obj.actuator is h.actor and obj.stepper is h.helper.process
    h.node.advertise.assert_called_once_with('/toponav/topology_display', obj.display_type)
    h.actor.request.assert_not_called()


@pytest.mark.parametrize('topic', ['/toponav/camera/'+v for v in (*runner.VIEWS, 'up', 'down')])
def test_direction_callbacks_keep_their_own_original_view(monkeypatch, topic):
    h = construction(monkeypatch); obj = IO(route=True, session_id='test', actuator_audit='audit')
    obj.image = Mock(); msg = object(); h.callbacks[topic](msg)
    obj.image.assert_called_once_with(topic.rsplit('/', 1)[-1], msg)


@pytest.mark.parametrize('link', LINKS)
def test_contact_callbacks_keep_their_own_original_link(monkeypatch, link):
    h = construction(monkeypatch); obj = IO(session_id='test', actuator_audit='audit')
    obj.contact = Mock(); msg = object()
    h.callbacks[f'/world/toponav/model/robot/link/{link}/sensor/{link}_contact/contact'](msg)
    obj.contact.assert_called_once_with(link, msg)


@pytest.mark.parametrize('topic', topics())
def test_transport_exception_is_not_hidden_or_retried(monkeypatch, topic):
    h = construction(monkeypatch, failed_topic=topic, raises=True)
    with pytest.raises(RuntimeError, match='transport registration raised'):
        IO(session_id='test', actuator_audit='audit')
    assert h.calls == topics()[:topics().index(topic)+1]
    h.actor_factory.assert_not_called(); h.step_factory.assert_not_called()


def test_step_helper_construction_failure_still_closes_created_actuator(monkeypatch):
    h = construction(monkeypatch); h.step_factory.side_effect = RuntimeError('helper failed')
    with pytest.raises(RuntimeError, match='helper failed'): IO(session_id='test', actuator_audit='audit')
    h.actor.close.assert_called_once_with(); h.actor.request.assert_not_called()
