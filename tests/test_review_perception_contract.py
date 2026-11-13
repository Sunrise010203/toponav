"""Malformed API evidence tests, not an unknown/occlusion accuracy evaluation."""
import json
import math

import numpy as np
import pytest
import torch

from roadvision import contract, learn
from simulation.gazebo_scenes import scenario_pair
from simulation.navigation_session import NavigationSession


NOW = 1_800_000_000_000_000_000
UNKNOWN = {'relative_open': dict.fromkeys(contract.DIRECTIONS), 'confidence': 0.}


def probabilities(index=8, score=1.):
    values = [0.] * 17
    values[index] = score
    return values


@pytest.mark.parametrize('values', [
    [False]*8 + [True] + [False]*8,
    [False] + [0.]*7 + [1.] + [0.]*8,
    [0.]*8 + [1.000001] + [0.]*8,
])
def test_boolean_or_over_one_probabilities_are_not_actionable_distributions(values):
    with pytest.raises(ValueError):
        contract.to_observation(values)


@pytest.mark.parametrize('value', [None, '0', [], complex(0), float('nan'), float('inf'),
                                 -float('inf'), -.01, pytest.param(10**400, id='overflowing-integer')])
def test_malformed_probability_scalar_is_a_boundary_value_error(value):
    values = probabilities()
    values[0] = value
    with pytest.raises(ValueError):
        contract.to_observation(values)


@pytest.mark.parametrize('kind', ['none', 'mapping', 'text', 'bytes', 'generator', 'unordered',
                                'column', 'row', 'short', 'long', 'zero_mass'])
def test_malformed_or_unordered_vector_is_not_coerced_to_class_evidence(kind):
    values = probabilities()
    bad = {
        'none': None, 'mapping': dict(enumerate(values)), 'text': '0'*17,
        'bytes': bytes([0]*8 + [1] + [0]*8), 'generator': (v for v in values),
        'unordered': {i/136 for i in range(17)}, 'column': np.array(values).reshape(17, 1),
        'row': np.array(values).reshape(1, 17), 'short': values[:-1],
        'long': values + [0.], 'zero_mass': [0.]*17,
    }[kind]
    with pytest.raises(ValueError):
        contract.to_observation(bad)


@pytest.mark.parametrize('threshold', [True, False, None, '0.6', float('nan'), float('inf'),
                                     .599999, 1.000001, pytest.param(10**400, id='overflowing-integer')])
def test_invalid_threshold_is_not_a_numeric_boolean_or_implicit_conversion(threshold):
    with pytest.raises(ValueError):
        contract.to_observation(probabilities(), threshold=threshold)


@pytest.mark.parametrize('valid', [False, None, 0, 1, -1, 'false', 'true', float('nan'),
                                 np.bool_(True), {}, [], [True]])
def test_only_explicit_builtin_true_can_assert_valid_sensor_evidence(valid):
    assert contract.to_observation(probabilities(), valid=valid) == UNKNOWN


@pytest.mark.parametrize('age', [pytest.param(10**400, id='huge-positive'),
                               pytest.param(-10**400, id='huge-negative')])
def test_overflowing_frame_age_is_unknown_not_an_exception(age):
    assert contract.to_observation(probabilities(), age_s=age) == UNKNOWN


@pytest.mark.parametrize('index', range(17))
def test_all_class_mappings_and_unknown_semantics_are_unchanged(index):
    observation = contract.to_observation(probabilities(index))
    if index == 16:
        assert observation == UNKNOWN
    else:
        assert observation == dict(relative_open=dict(zip(contract.DIRECTIONS,
            [bit == '1' for bit in f'{index:04b}'])), confidence=1.)


@pytest.mark.parametrize('container', [list, tuple, lambda v: np.array(v, dtype=np.float32),
                                     lambda v: np.array(v, dtype=np.float64)])
def test_ordered_python_and_numpy_real_vectors_remain_accepted_without_mutation(container):
    values = container(probabilities())
    before = list(values)
    assert contract.to_observation(values)['confidence'] == 1.
    assert list(values) == before


@pytest.mark.parametrize('score', [.6, .75, 1.])
@pytest.mark.parametrize('threshold', [.6, .75, 1.])
def test_original_score_threshold_boundaries_are_preserved(score, threshold):
    values = probabilities(score=score)
    values[0] = 1. - score
    result = contract.to_observation(values, threshold=threshold, valid=True, age_s=1.)
    assert result['confidence'] == (score if score >= threshold else 0.)


@pytest.mark.parametrize('valid', ['false', 1, np.bool_(True)])
def test_truthy_malformed_validity_cannot_accumulate_a_real_navigation_command(valid):
    graph, _ = scenario_pair('vision')
    clocks = dict(mono=0., wall=NOW)
    session = NavigationSession(graph, clock=lambda: clocks['mono'], wall_ns=lambda: clocks['wall'])
    for i in range(1, 4):
        clocks.update(mono=i*.1, wall=NOW+i*100_000_000)
        decision = session.observe(dict(frame_id=f'invalid-validity/{i}', captured_at_ns=clocks['wall'],
            observation=contract.to_observation(probabilities(), valid=valid)))
        assert decision['action'] == 'stop'
    assert session.metrics['commands_sent'] == 0 and session.pending is None


@pytest.fixture
def predictor(monkeypatch):
    # Execute the real Predictor.predict and prepare, but feed deterministic
    # synthetic logits. Frozen-weight inference is verified separately.
    class Logits:
        calls = 0
        index = 8
        low = False

        def __call__(self, inputs):
            self.calls += 1
            values = [0.]*17
            if not self.low:
                values[self.index] = 40.
            return torch.tensor([values], dtype=torch.float64)

    instance = learn.Predictor.__new__(learn.Predictor)
    instance.metadata = {'threshold': .6}
    instance.device, instance.model = 'cpu', Logits()
    monkeypatch.setattr(learn.time, 'time_ns', lambda: NOW)
    return instance


def image():
    return np.full((256, 256, 3), 120, np.uint8)


@pytest.mark.parametrize('stamp', [True, False, -1, 1., float('nan'), float('inf'), -float('inf'),
                                 '1', {}, np.int64(NOW), pytest.param(10**400, id='overflowing-integer')])
def test_malformed_capture_timestamp_is_rejected_before_inference(predictor, stamp):
    with pytest.raises(ValueError, match='Timestamp'):
        predictor.predict(image(), captured_at_ns=stamp)
    assert predictor.model.calls == 0


@pytest.mark.parametrize('phase', ['capture_default', 'prediction_end'])
@pytest.mark.parametrize('stamp', [False, float('nan'), float(NOW), pytest.param(10**400, id='overflowing-integer')])
def test_invalid_clock_provider_cannot_create_an_accepted_or_non_json_packet(predictor, monkeypatch, phase, stamp):
    monkeypatch.setattr(learn.time, 'time_ns', lambda: stamp)
    with pytest.raises(ValueError, match='Timestamp'):
        predictor.predict(image(), captured_at_ns=None if phase == 'capture_default' else NOW)
    assert predictor.model.calls == (0 if phase == 'capture_default' else 1)


@pytest.mark.parametrize('stamp,accepted,reason', [
    (NOW, True, None), (NOW-250_000_000, True, None), (NOW-1_000_000_000, True, None),
    (NOW-1_000_000_001, False, 'stale_frame'), (NOW+1, False, 'invalid_timestamp'),
    (0, False, 'stale_frame'), (10**300, False, 'invalid_timestamp'),
])
def test_valid_integer_timestamp_boundaries_and_reasons_remain_consistent(predictor, stamp, accepted, reason):
    packet = predictor.predict(image(), captured_at_ns=stamp, frame_id='timestamp-boundary')
    assert packet['captured_at_ns'] == stamp and packet['predicted_at_ns'] == NOW
    assert packet['accepted'] is accepted and packet['rejection_reason'] == reason
    assert math.isfinite(packet['frame_age_s'])
    if not accepted:
        assert packet['observation'] == UNKNOWN
    json.dumps(packet, allow_nan=False)


def test_optional_timestamp_default_uses_real_clock_without_retiming_explicit_capture(predictor):
    packet = predictor.predict(image())
    assert packet['captured_at_ns'] == NOW and packet['frame_age_s'] == 0.
    assert packet['accepted'] and packet['rejection_reason'] is None


@pytest.mark.parametrize('kind,reason', [('low', 'low_confidence'), ('unknown', 'unobservable'),
                                      ('missing', 'missing_view')])
def test_existing_non_timestamp_rejection_reasons_are_preserved(predictor, kind, reason):
    pixels = image()
    if kind == 'low':
        predictor.model.low = True
    if kind == 'unknown':
        predictor.model.index = 16
    if kind == 'missing':
        pixels[:] = 0
    packet = predictor.predict(pixels, captured_at_ns=NOW)
    assert not packet['accepted'] and packet['rejection_reason'] == reason
    assert packet['observation'] == UNKNOWN
    json.dumps(packet, allow_nan=False)
