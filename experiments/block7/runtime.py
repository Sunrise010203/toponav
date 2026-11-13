"""One shared discrete RGB driver with public-only replayable session events.

The world/sensor own truth; only graph, task, frame and feedback cross the
coordinator boundary. Physical continuous controllers are never imported.
"""
from contextlib import nullcontext
from dataclasses import asdict
import copy
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image
import yaml

from experiments.block7.common import CONFIG, digest, file_hash, seed, write_json
from experiments.block7.dataset import pair, registered, remove_public_edge
from experiments.block7.rendering import RenderedWorld
from experiments.block7.session import ExperimentConfig, ExperimentSession
from roadvision.scene import LAYOUTS, topology_pair
from simulation.navigation_session import TERMINAL
from simulation.structural_navigation import StructuralEnvironment


class EventClock:
    def __init__(self):
        self.mono, self.wall = time.monotonic(), time.time_ns()

    def refresh(self):
        self.mono, self.wall = time.monotonic(), time.time_ns()

    def snapshot(self):
        return {'monotonic_s': self.mono, 'wall_ns': self.wall}

    def load(self, snapshot):
        self.mono, self.wall = snapshot['monotonic_s'], snapshot['wall_ns']


def corrupt_observation(observation):
    """Complete high-score corruption, without consulting simulator truth.

    An originally unknown/wrong prediction could accidentally be corrected;
    the one-way audit must verify actual error effect separately from injection.
    """
    values = {key: bool(observation['relative_open'][key]) for key in ('front', 'left', 'right', 'back')}
    values['front'] = not values['front']
    return {'relative_open': values, 'confidence': .99}


class RGBSensor:
    """Actual PyBullet pixels and frozen Predictor. No ideal fallback."""
    def __init__(self, world, environment, cells, task, output, predictor=None):
        if predictor is None:
            raise ValueError('Frozen predictor must be loaded before session timing begins')
        self.world, self.environment, self.cells = world, environment, cells
        self.task, self.output, self.predictor = task, output, predictor

    def capture(self, index, blocked):
        state = self.environment.state()  # Rendering only; never controller input.
        appearance_seed = seed(self.task['seeds']['appearance'], state['node_id'], state['heading_deg'], index)
        self.world.appearance_variant = self.task.get('appearance_variant', 'nominal')
        image, render_audit = self.world.sample(0, np.random.default_rng(appearance_seed),
                           pose=(self.cells[state['node_id']], state['heading_deg']), blocked=blocked)
        factor = self.task['factor']
        if factor == 'occlusion_once' and index == 0:
            image = image.copy()
            # A declared image-level obstruction, not a fabricated classifier error.
            image[24:104, 24:104] = 45
        captured = time.time_ns()  # Actual completed acquisition/augmentation.
        identity = f"{self.task['task_id']}-f{index}"
        path = self.output / 'rgb' / (str(index) + '.png')
        Image.fromarray(image).save(path)
        packet = self.predictor.predict(image, captured_at_ns=captured, frame_id=identity)
        return ({key: packet[key] for key in ('frame_id', 'captured_at_ns', 'observation')},
                {'prediction': packet, 'rgb': str(path.relative_to(self.output)),
                 'png_sha256': file_hash(path), 'pixel_sha256': hashlib.sha256(image.tobytes()).hexdigest(),
                 'appearance_seed': appearance_seed}, render_audit)


def load_fixture(task, suite=None):
    if task['split'] == 'development':
        if task['layout_id'] not in LAYOUTS:
            raise ValueError('Development smoke must use an exposed historical layout')
        graph, scene, cells = topology_pair(task['layout_id'])
        layout = None
    else:
        if suite is None:
            raise ValueError('New layouts require an explicit validated suite')
        definition = json.loads((Path(suite) / 'layouts' / (task['layout_id'] + '.json')).read_text())
        graph, scene, cells = pair(definition['layout'])
        if graph != definition['graph'] or scene != definition['scene']:
            raise ValueError('Layout export differs from prepared graph/scene')
        layout = definition['layout']
    public = remove_public_edge(graph, task['wrong_map_channel']) if task.get('wrong_map_channel') else copy.deepcopy(graph)
    return public, graph, scene, cells, layout


def episode(task, method, output, suite=None, sensor_factory=None, clock=None, sleeper=time.sleep,
            make_world=True, test_limits=None, final_authorization=None):
    """Tests may inject a labelled scripted sensor; CLI never does so."""
    if task['split'] == 'test':
        from experiments.block7.validate import authorize_final
        authorize_final(task, suite, final_authorization)
    if task['split'] not in {'development', 'validation', 'test'}:
        raise ValueError('Unknown split')
    if sensor_factory is None and (clock is not None or not make_world):
        raise ValueError('Real RGB runs require real clocks and an actual world')
    output = Path(output); output.mkdir(parents=True, exist_ok=False)
    (output / 'rgb').mkdir(); (output / 'audit').mkdir()
    episode_started_wall = time.monotonic()
    predictor = None
    if sensor_factory is None:
        import torch
        from roadvision.learn import Predictor
        torch.set_num_threads(2)
        predictor = Predictor(Path(yaml.safe_load((CONFIG / 'methods.yaml').read_text())['shared']['model_path']), 'cpu')
    model_setup_wall_s = time.monotonic() - episode_started_wall
    public, physical_graph, physical_scene, cells, layout = load_fixture(task, suite)
    write_json(output / 'public-map.json', public)
    budget = yaml.safe_load((CONFIG / 'budget.yaml').read_text())['navigation']['limits']
    fields = {'max_commands', 'max_events', 'max_frame_age_s', 'stage_timeout_s',
              'session_timeout_s', 'max_conflicts', 'max_execution_failures'}
    limits = {k: budget[k] for k in fields}
    if test_limits:
        if sensor_factory is None:
            raise ValueError('Runtime limits cannot be overridden for real RGB experiments')
        limits.update(test_limits)
    config = ExperimentConfig.for_method(method, **limits)
    event_clock = clock or EventClock()
    session_id = 'block7-' + digest([task['task_id'], method])[:24]
    constructor = {'start': task['start'], 'heading': task['heading'], 'goal': task['goal'],
                   'disabled_channels': task.get('public_disabled_channels', []), 'session_id': session_id}
    initial_clock = event_clock.snapshot()
    session = ExperimentSession(public, config=config, clock=lambda: event_clock.mono,
                                wall_ns=lambda: event_clock.wall, **constructor)
    write_json(output / 'header.json', {'task': task, 'method': method, 'config': asdict(config),
               'constructor': constructor, 'initial_clock': initial_clock, 'public_map_sha256': file_hash(output / 'public-map.json'),
               'model_setup_wall_s': model_setup_wall_s,
               'sensor': 'scripted_protocol_fixture' if sensor_factory else 'actual_PyBullet_RGB_frozen_Predictor',
               'scope': 'atomic_discrete_only_no_photos_no_continuous_physics',
               'physical_collision_or_braking_measured': False})
    environment = StructuralEnvironment(physical_graph, physical_scene, task['start'], task['heading'],
                                         task.get('physical_disabled_channels', []))
    event_index, commands, capture_index = 0, 0, 0
    fault_triggered, mislocalized_commands, error = False, 0, None
    fault_effect_verified = None
    factor = task['factor']
    started_wall = time.monotonic()
    registration = registered(layout) if layout else nullcontext(task['layout_id'])
    with (output / 'events.jsonl').open('x') as events, (output / 'audit/trajectory.jsonl').open('x') as truth:
        def dispatch(kind, payload=None):
            nonlocal event_index
            event_clock.refresh()
            try:
                result = getattr(session, kind)(payload) if payload is not None else getattr(session, kind)()
                record = {'index': event_index, 'kind': kind, 'payload': payload,
                          'clock': event_clock.snapshot(), 'output': result, 'exception': None}
            except (ValueError, TypeError) as exc:
                record = {'index': event_index, 'kind': kind, 'payload': payload,
                          'clock': event_clock.snapshot(), 'output': None,
                          'exception': {'type': type(exc).__name__, 'message': str(exc)}}
                events.write(json.dumps(record, allow_nan=False) + '\n'); events.flush()
                event_index += 1
                raise
            events.write(json.dumps(record, allow_nan=False) + '\n'); events.flush()
            event_index += 1
            return result

        try:
            with registration as identity:
                world_context = RenderedWorld(identity) if make_world else nullcontext(None)
                with world_context as world:
                    sensor = (sensor_factory(environment, task, output, event_clock) if sensor_factory else
                              RGBSensor(world, environment, cells, task, output, predictor))
                    for cycle in range(budget['max_cycles']):
                        if session.state in TERMINAL:
                            break
                        if time.monotonic() - started_wall > budget['external_wall_timeout_s']:
                            dispatch('abort', 'external_wall_timeout')
                            break
                        before = environment.state()
                        disabled = set(task.get('physical_disabled_channels', []))
                        if factor == 'temporary_closure' and capture_index < task['fault']['closure_capture_count']:
                            disabled.add(task['fault']['channel']); fault_triggered = True
                        environment._world.disabled = disabled  # Truth-owning executor only.
                        edge_by_id = {e['channel_id']: e for e in physical_graph['edges']}
                        blocked = {frozenset((cells[edge_by_id[c]['source']], cells[edge_by_id[c]['target']]))
                                   for c in disabled}
                        if factor == 'drop_once' and capture_index == 0:
                            frame, packet, render_audit = None, None, None
                            fault_triggered = True
                            result = dispatch('missing_capture')
                        else:
                            frame, packet, render_audit = sensor.capture(capture_index, blocked)
                            if factor == 'confident_wrong_observation_once' and capture_index == 0:
                                frame = copy.deepcopy(frame)
                                frame['observation'] = corrupt_observation(frame['observation'])
                                fault_triggered = True
                            if capture_index == 0 and factor in {'appearance_material_or_lighting', 'occlusion_once', 'wrong_map_edge'}:
                                fault_triggered = True
                            result = dispatch('observe', frame)
                            if factor == 'confident_wrong_observation_once' and capture_index == 0:
                                # Audit only, AFTER the public input/decision. This
                                # label cannot alter frame, decision or candidates.
                                fault_effect_verified = frame['observation']['relative_open'] != environment.observe()['relative_open']
                        execution_event = None
                        if result['command_id'] is not None:
                            loc = result.get('localization') or {}
                            if loc.get('estimate') != {'node_id': before['node_id'], 'heading_deg': before['heading_deg']}:
                                mislocalized_commands += 1
                            outcome = 'normal'
                            if commands == 0 and factor in {'trusted_atomic_execution_failed_once', 'execution_unknown'}:
                                outcome = 'failed' if factor.startswith('trusted') else 'unknown'
                                fault_triggered = True
                            feedback = environment.execute(result['action'], outcome)
                            execution_event = {'event_id': f'exec-{commands}', 'command_id': result['command_id'],
                                               'feedback': feedback}
                            dispatch('feedback', execution_event)
                            commands += 1
                        truth.write(json.dumps({'cycle': cycle, 'capture_index': capture_index,
                                    'last_public_event_index': event_index - 1, 'before': before,
                                    'after': environment.state(), 'physical_disabled': sorted(disabled),
                                    'render_audit': render_audit, 'packet': packet,
                                    'fault_effect_verified': fault_effect_verified,
                                    'execution_event': execution_event}, allow_nan=False) + '\n')
                        truth.flush(); capture_index += 1
                        sleeper(budget['poll_interval_s'])
                    if session.state not in TERMINAL:
                        dispatch('abort', 'cycle_limit')
        except Exception as exc:
            error = {'type': type(exc).__name__, 'message': str(exc)}
            dispatch('abort', 'input_or_runner_error')
        finally:
            environment.execute('stop')
    final = environment.state()
    reached = session.state == 'reached'
    verified = reached and final['node_id'] == task['goal']
    false_arrival = reached and not verified
    reason = session.reason
    status = ('error' if error else 'success' if verified else
              'timeout' if any(w in reason for w in ('timeout', 'limit')) else 'refused')
    report = session.report()
    write_json(output / 'controller.json', report)
    result = {'task_id': task['task_id'], 'method': method, 'split': task['split'], 'factor': factor,
              'status': status, 'reason': reason, 'error': error, 'goal_verified': verified,
              'false_arrival': false_arrival, 'fault_triggered': fault_triggered,
              'fault_effect_verified': fault_effect_verified,
              'physical_reachable': task.get('physical_reachable', True), 'mislocalized_commands': mislocalized_commands,
              'commands': commands, 'capture_slots': capture_index, 'public_events': event_index,
              'wall_elapsed_s': time.monotonic() - started_wall, 'model_inference': sensor_factory is None,
              'end_to_end_wall_s': time.monotonic() - episode_started_wall,
              'model_setup_wall_s': model_setup_wall_s,
              'collision': None, 'braking': None, 'scope': 'atomic_discrete_not_physical_navigation',
              'audit': {'final_truth': final}, 'controller_sha256': file_hash(output / 'controller.json')}
    write_json(output / 'result.json', result)
    files = [p for p in output.rglob('*') if p.is_file()]
    write_json(output / 'manifest.json', {'status': status, 'task_id': task['task_id'], 'method': method,
                'files': [{'path': str(p.relative_to(output)), 'sha256': file_hash(p)} for p in sorted(files)]})
    return result


def development_task(factor='normal', identity='dev'):
    if factor not in {'normal', 'drop_once', 'occlusion_once', 'confident_wrong_observation_once',
                      'trusted_atomic_execution_failed_once', 'execution_unknown', 'appearance_material_or_lighting'}:
        raise ValueError('Unsupported development factor')
    return {'task_id': identity + '-' + factor, 'family_id': 'historical-test_locked',
            'layout_id': 'test_locked', 'split': 'development', 'start': 'R000', 'goal': 'R007',
            'heading': 0, 'factor': factor, 'environment_seed': 7101801,
            'appearance_variant': 'lighting_exposure' if factor == 'appearance_material_or_lighting' else 'nominal',
            'physical_disabled_channels': [], 'public_disabled_channels': [], 'wrong_map_channel': None,
            'fault': {'anchor': 'first_scheduled_capture'}, 'physical_reachable': True,
            'seeds': {stream: seed('development', 7101801, stream)
                      for stream in ('appearance', 'sensor_faults', 'execution_faults')}}
