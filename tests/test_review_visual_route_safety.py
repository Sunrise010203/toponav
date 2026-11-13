"""Synthetic perception tests exercise real route state machines, not driving accuracy."""
import copy
import json

import numpy as np
import pytest

from roadvision.contract import to_observation
from roadvision import landmark_route, model_route
from simulation.gazebo_route_fixture import registry
from simulation.gazebo_scenes import scenario_pair
from ros2_ws.src.toponav_bridge.toponav_bridge.protocol import MotorGate, command_from_decision


class Prediction:
    metadata = {'model_sha256': 'synthetic-safety-test-only'}
    index = 8
    calls = 0

    def predict(self, frame, now):
        self.calls += 1
        probabilities = [0.] * 17
        probabilities[self.index] = 1.
        observation = to_observation(probabilities, age_s=now-frame['captured_sim_s'])
        return dict(accepted=observation['confidence'] > 0, observation=observation,
                    class_id='unobservable' if self.index == 16 else f'exits_{self.index:04b}', score=1.)


def frame(kind, stamp):
    packet = dict(frame_id=f'safety/{stamp}', captured_sim_s=stamp)
    pixels = np.full((256, 256, 3), 120, np.uint8)
    if kind == 'model':
        packet['rgb'] = pixels
    else:
        packet.update(up_rgb=pixels, down_rgb=pixels.copy())
    return packet


@pytest.fixture(params=['model', 'landmark'])
def route(request, monkeypatch):
    kind = request.param
    # Only replace RGB interpretation. Real confirmation/localizer/planner and
    # update/cancel/result code execute; this is not a Gazebo acceptance trial.
    monkeypatch.setattr(model_route, 'measurements', lambda *a: {
        'front': dict(angle=0., offset=0., support=20)})
    monkeypatch.setattr(model_route, 'incoming_geometry', lambda *a: dict(angle=0., offset=0.))
    monkeypatch.setattr(model_route, 'station_distance', lambda *a: (1., 'synthetic_station'))
    monkeypatch.setattr(model_route, 'front_wall_range', lambda *a: None)
    monkeypatch.setattr(model_route, 'enclosed_side_walls', lambda *a: False)
    monkeypatch.setattr(landmark_route, 'landmarks', lambda *a: {
        'N01': dict(x=0., y=0., yaw=0.)})

    def make(last_edge=False):
        graph, _ = scenario_pair('vision' if kind == 'model' else 'route')
        if last_edge:
            graph['task']['start_node'] = 'N04'
        if kind == 'model':
            return model_route.ModelRouteController(graph, Prediction())
        return landmark_route.LandmarkRouteController(graph, registry(graph))

    return kind, make


def moving(kind, make, *, delayed_last=False):
    controller = make()
    for i in range(1, 8):
        stamp = i / 10
        now = stamp + .2 if delayed_last and i == 7 else stamp
        decision = controller.update(frame(kind, stamp), now)
    assert decision['state'] == 'moving' and decision['v_m_s'] > 0
    assert controller.phase == 'cruise'
    return controller


def assert_zero(decision):
    assert decision['v_m_s'] == decision['omega_rad_s'] == 0.
    json.dumps(decision, allow_nan=False)


def corrupt(kind, defect):
    packet = frame(kind, .8)
    if defect == 'none':
        return None
    if defect == 'empty':
        return {}
    if defect == 'string':
        return 'not-a-frame'
    if defect == 'list':
        return []
    if defect == 'no_id':
        del packet['frame_id']
    if defect == 'no_time':
        del packet['captured_sim_s']
    if defect == 'no_pixels':
        del packet['rgb' if kind == 'model' else 'up_rgb']
    if defect == 'extra_fields':
        packet['actual_pose'] = [0., 0.]
    if defect == 'pixels_list':
        packet['rgb' if kind == 'model' else 'up_rgb'] = []
    return packet


@pytest.mark.parametrize('defect', [
    'none', 'empty', 'string', 'list', 'no_id', 'no_time',
    'no_pixels', 'extra_fields', 'pixels_list',
])
def test_bad_public_frame_after_motion_returns_and_latches_zero(route, defect):
    kind, make = route
    controller = moving(kind, make)
    events = copy.deepcopy(controller.events)
    count = controller.motion_commands
    decision = controller.update(corrupt(kind, defect), .8)
    assert_zero(decision)
    assert decision['state'] == 'aborted'
    assert controller.motion_commands == count and controller.events == events
    assert_zero(controller.update(frame(kind, .9), .9))
    assert controller.state == 'aborted' and controller.events == events


@pytest.mark.parametrize('field', ['captured_sim_s', 'now'])
@pytest.mark.parametrize('value', [True, False, float('nan'), float('inf'), -.1, '1.0', None,
                                 pytest.param(10**400, id='overflowing-integer')])
def test_invalid_clock_values_cannot_start_or_poison_output_json(route, field, value):
    kind, make = route
    controller = make()
    stamp = now = 0. if value is False else 1.
    if value == -.1:
        stamp = now = -.1
    packet = frame(kind, stamp)
    if field == 'captured_sim_s':
        packet[field] = value
    else:
        now = value
    decision = controller.update(packet, now)
    assert_zero(decision)
    assert decision['state'] == 'aborted'
    assert not controller.events and controller.motion_commands == 0


def test_regressing_decision_clock_is_rejected_even_with_fresh_increasing_capture(route):
    kind, make = route
    controller = moving(kind, make, delayed_last=True)
    assert controller.last_stamp == .7
    decision = controller.update(frame(kind, .8), .85)
    assert_zero(decision)
    assert decision['state'] == 'aborted' and decision['reason'] == 'sim_time_regression'


def test_reusing_older_identity_with_new_stamp_is_rejected(route):
    kind, make = route
    controller = moving(kind, make)
    packet = frame(kind, .8)
    packet['frame_id'] = 'safety/0.1'  # Not the immediately previous identity.
    decision = controller.update(packet, .8)
    assert_zero(decision)
    assert decision['state'] == 'aborted'


@pytest.mark.parametrize('identity', [None, '', True, 7, [], pytest.param('x'*257, id='oversized')])
def test_invalid_frame_identity_is_rejected_before_perception(route, identity):
    kind, make = route
    controller = moving(kind, make)
    packet = frame(kind, .8)
    packet['frame_id'] = identity
    decision = controller.update(packet, .8)
    assert_zero(decision)
    assert decision['state'] == 'aborted' and decision['reason'] == 'invalid_frame_id'


def test_unexpected_perception_exception_latches_before_propagating(route, monkeypatch):
    kind, make = route
    controller = moving(kind, make)

    def unavailable(*args):
        raise OSError('synthetic camera/perception backend failure')

    if kind == 'model':
        monkeypatch.setattr(controller.perception, 'predict', unavailable)
    else:
        monkeypatch.setattr(landmark_route, 'landmarks', unavailable)
    with pytest.raises(OSError, match='backend failure'):
        controller.update(frame(kind, .8), .8)
    assert controller.state == 'aborted'
    assert_zero(controller.update(None, float('nan')))


def test_bad_packet_abort_reaches_real_motor_gate_after_nonzero_command(route):
    kind, make = route
    controller = moving(kind, make)
    decision = controller.update(frame(kind, .8), .8)
    gate = MotorGate('route-safety')
    gate.arm(decision['control_frame_id'], decision['captured_sim_s'], 10.)
    assert gate.accept(command_from_decision('route-safety', 1, decision), .8, 10.01)[0] > 0
    # A subsequent frame lease must not prevent an abort bound to the last
    # validated header from stopping the previously accepted wheel command.
    gate.arm('next-camera-frame', .9, 10.1)
    stopped = controller.update(None, .9)
    assert_zero(stopped)
    assert gate.accept(command_from_decision('route-safety', 2, stopped), .9, 10.11) == (0., 0.)
    assert gate.latched and gate.reason == 'remote_abort'
    assert gate.poll(10.2) == (0., 0.)


@pytest.mark.parametrize('terminal', ['visual_stop', 'aborted'])
@pytest.mark.parametrize('defect', ['none', 'empty', 'no_id', 'no_time', 'pixels_list', 'fresh', 'invalid_now'])
def test_terminal_result_and_zero_output_ignore_all_later_packets(route, terminal, defect, monkeypatch):
    kind, make = route
    controller = make(last_edge=terminal == 'visual_stop')
    if terminal == 'visual_stop':
        # Use distinct valid task endpoints. Synthetic observations exercise
        # the real final-edge stop and public goal confirmation, not a forged
        # visual_stop state or an invalid start==goal graph.
        if kind == 'model':
            controller.perception.index = 11  # N04, eastbound final edge.
            for stamp in (.1, .2, .3):
                controller.update(frame(kind, stamp), stamp)
            controller.phase = 'dock'
            controller.phase_start = .3
            controller.departed = True
            controller.perception.index = 1  # N06 dead end, back opening.
            monkeypatch.setattr(model_route, 'station_distance', lambda *a: (.02, 'synthetic_station'))
            stamps = (.4, .5, .6, .7, .8, .9)
        else:
            tag = {'node': 'N04'}
            monkeypatch.setattr(landmark_route, 'landmarks', lambda *a: {
                tag['node']: dict(x=0., y=0., yaw=0.)})
            for stamp in (.1, .2, .3, .4, .5, .6):
                controller.update(frame(kind, stamp), stamp)
            tag['node'] = 'N06'
            stamps = (.7, .8, .9)
        for stamp in stamps:
            controller.update(frame(kind, stamp), stamp)
    else:
        controller.update(frame(kind, .1), .1)
        controller.cancel(.15)
    assert controller.state == terminal
    before = controller.update(frame(kind, 1.), 1.)
    result = copy.deepcopy(controller.result())
    packet = frame(kind, .8) if defect in ('fresh', 'invalid_now') else corrupt(kind, defect)
    decision = controller.update(packet, float('nan') if defect == 'invalid_now' else .8)
    assert_zero(decision)
    assert decision == before
    assert controller.result() == result


def test_native_route_optional_main_panel_is_still_supported(route):
    kind, make = route
    controller = make()
    packet = frame(kind, 0.)
    if kind == 'landmark':
        packet['rgb'] = np.full((256, 256, 3), 120, np.uint8)
    decision = controller.update(packet, .250001)
    assert_zero(decision)
    assert decision['state'] == 'observing'  # Existing inclusive age boundary.


@pytest.mark.parametrize('loss', ['rejected_model', 'wrong_signature', 'missing_station'])
def test_model_docking_needs_three_consecutive_supported_zero_frames(monkeypatch, loss):
    graph, _ = scenario_pair('vision')
    prediction = Prediction()
    controller = model_route.ModelRouteController(graph, prediction)
    monkeypatch.setattr(model_route, 'measurements', lambda *a: {})
    monkeypatch.setattr(model_route, 'incoming_geometry', lambda *a: dict(angle=0., offset=0.))
    monkeypatch.setattr(model_route, 'enclosed_side_walls', lambda *a: False)
    station = [.02]
    monkeypatch.setattr(model_route, 'station_distance', lambda *a: (station[0], 'synthetic_station'))
    for stamp in (.1, .2, .3):
        controller.update(frame('model', stamp), stamp)
    assert controller.visited == ['N01']
    # Isolate docking after a real startup authorization; not a physical arrival.
    controller.phase = 'dock'
    controller.phase_start = .3
    controller.departed = True
    prediction.index = 13  # Expected N02 signature, in the incoming heading.
    for stamp in (.4, .5):
        assert_zero(controller.update(frame('model', stamp), stamp))
    assert controller.stable == 2
    if loss == 'missing_station':
        station[0] = None
    else:
        prediction.index = 16 if loss == 'rejected_model' else 8
    assert_zero(controller.update(frame('model', .6), .6))
    assert controller.stable == 0
    station[0] = .02
    prediction.index = 13
    for stamp, votes in ((.7, 1), (.8, 2)):
        assert_zero(controller.update(frame('model', stamp), stamp))
        assert controller.phase == 'dock' and controller.stable == votes
        assert not any(e['event'] == 'visual_edge_station_stopped' for e in controller.events)
    assert_zero(controller.update(frame('model', .9), .9))
    assert controller.phase == 'confirm_node' and controller.visited == ['N01']
    stops = [e for e in controller.events if e['event'] == 'visual_edge_station_stopped']
    assert len(stops) == 1 and stops[0]['captured_sim_s'] == .9
    for stamp in (1., 1.1):
        assert_zero(controller.update(frame('model', stamp), stamp))
        assert controller.visited == ['N01']
    assert_zero(controller.update(frame('model', 1.2), 1.2))
    assert controller.visited == ['N01', 'N02']
