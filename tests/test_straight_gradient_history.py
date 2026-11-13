"""Explicit composition contracts; stubbed fits are not RGB acceptance."""
import copy
import json

import numpy as np
import pytest

from roadvision import continuous_straight as base
from roadvision import gradient_history_straight as experiment
from roadvision.gradient_straight import GradientTransferStraightController
from roadvision.straight_ablation import HISTORY_SELECTION_POLICY
from tests.test_straight_task3_ablation import frames, metric, SAFETY_TESTS

BaseController = base.StraightController
Controller = experiment.GradientHistoryStraightController


def test_only_history_hook_changes_existing_gradient_perception_and_safety():
    c = Controller()
    assert c.frames.maxlen == 17 and c.stationary_samples.maxlen == 3
    assert c.policy == base.StraightPolicy() and c.motion_permission == 'no_motion'
    for name in ('_update', '_wait', '_confirm_stationary', '_clock', '_stopping_reserve',
                 '_following_command', '_following_guard', '_local_stationary_motion', 'missing', 'cancel',
                 '_assess_startup_texture', '_probe_navigation_history', '_assess_navigation_free_ground'):
        assert getattr(Controller, name) is getattr(GradientTransferStraightController, name)
    assert BaseController().frames.maxlen == GradientTransferStraightController().frames.maxlen == 9


def test_selects_existing_training_window_before_single_fit(monkeypatch):
    import roadvision.straight_ablation as a
    calls = []
    monkeypatch.setattr(a, 'history_training_score', lambda f, *_: (calls.append(len(f)) or metric(len(f))))
    c = Controller(); c.frames.extend(frames()); c.last_frame_id = 'f16'
    selected = c._navigation_history(1.6)
    assert calls == [5, 9, 13, 17] and len(selected) == 13
    assert c.selection_receipt['selected_frame_ids'] == [f['frame_id'] for f in selected]
    assert c.selection_receipt['evaluated_frame_id'] == 'f16'
    assert c.selection_receipt['evaluated_sim_s'] == 1.6
    assert c.selection_receipt['final_geometry_attempts'] == 1


def test_unknown_selection_does_not_fall_back_to_last_nine(monkeypatch):
    import roadvision.straight_ablation as a
    monkeypatch.setattr(a, 'history_training_score', lambda f, *_: metric(len(f), False))
    c = Controller(); c.frames.extend(frames())
    assert c._navigation_history(1.6) is None
    assert c.selection_receipt['final_geometry_attempts'] == 0


@pytest.mark.parametrize('bad', ['truth', 'replay', 'gap', 'stale', 'future'])
def test_all_buffer_packets_validated_before_training_score(monkeypatch, bad):
    import roadvision.straight_ablation as a
    f = frames(); now = 1.6
    if bad == 'truth': f[1]['pose'] = [0., 0.]
    elif bad == 'replay': f[1]['frame_id'] = f[0]['frame_id']
    elif bad == 'gap': f[1]['captured_sim_s'] = f[0]['captured_sim_s']
    elif bad == 'stale': now = 1.9
    elif bad == 'future': now = 1.5
    monkeypatch.setattr(a, 'history_training_score', lambda *args: pytest.fail('scoring before validation'))
    c = Controller(); c.frames.extend(f)
    with pytest.raises(ValueError): c._navigation_history(now)


def stub_perception_hooks(monkeypatch):
    # Preserve the actual safety state machine while routing its image hooks
    # to the existing synthetic test producers. Real inherited hooks are
    # checked by identity above and used in public replay tests below.
    for name in ('_assess_startup_texture', '_probe_navigation_history', '_assess_navigation_free_ground'):
        monkeypatch.setattr(Controller, name, getattr(BaseController, name))


@pytest.mark.parametrize('name', SAFETY_TESTS)
def test_existing_permission_braking_rearm_timeout_contracts(monkeypatch, name):
    from tests import test_straight_task2_permissions as contracts
    import roadvision.straight_ablation as a
    stub_perception_hooks(monkeypatch)
    monkeypatch.setattr(a, 'history_training_score', lambda f, *_: dict(metric(len(f)), score=abs(len(f)-9)))
    monkeypatch.setattr(base, 'StraightController', lambda **kw: Controller(**kw))
    getattr(contracts, name)(monkeypatch)


def test_one_rejected_final_fit_never_tries_other_training_windows(monkeypatch):
    from tests.test_straight_task2_permissions import setup, frame
    import roadvision.straight_ablation as a
    stub_perception_hooks(monkeypatch)
    c, fits = setup(monkeypatch)
    exp = Controller(initial_clearance=base.InitialClearancePrior(confirmed=True), motion_permission='bounded_probe')
    exp._local_stationary_motion = c._local_stationary_motion
    monkeypatch.setattr(a, 'history_training_score', lambda f, *_: dict(metric(len(f)), score=abs(len(f)-9)))
    calls = []; original = base.probe_straight_history
    def probe(f, **kw): calls.append(len(f)); return original(f, **kw)
    monkeypatch.setattr(base, 'probe_straight_history', probe)
    fits['good'] = False
    for i in range(9): out = exp.update(frame(i), i*.1)
    assert calls == [9] and out['state'] == 'waiting_evidence'
    assert out['v_m_s'] == out['omega_rad_s'] == 0.


def test_public_receipts_do_not_alias_selection_or_policy():
    c = Controller(); c.selection_receipt = dict(selected_frame_ids=['a'], selected_size=5)
    saved_policy = copy.deepcopy(HISTORY_SELECTION_POLICY)
    out = c._output(); out['experiment']['history_selection']['selected_frame_ids'].clear()
    out['experiment']['history_selection_policy']['sizes'].clear()
    assert c.selection_receipt['selected_frame_ids'] == ['a']
    assert HISTORY_SELECTION_POLICY == saved_policy
    assert c.result()['experiment']['variant'] == 'gradient_history'
    assert c.result()['experiment']['promoted'] is False


@pytest.mark.parametrize('confirmed,permission,reason', [
    (False, 'bounded_probe', 'initial_clearance_prior_not_confirmed'),
    (True, 'no_motion', 'motion_not_authorized'),
])
def test_explicit_permission_required_and_terminal_latched(confirmed, permission, reason):
    c = Controller(initial_clearance=base.InitialClearancePrior(confirmed=confirmed), motion_permission=permission)
    f = frames(1)[0]; out = c.update(f, 0.)
    assert out['reason'] == reason and out['v_m_s'] == out['omega_rad_s'] == 0.
    saved = c.result()
    assert c.update({}, np.nan)['v_m_s'] == 0. and c.result() == saved


@pytest.mark.parametrize('kwargs', [dict(corner_direction='left'), dict(route_graph={})])
def test_cannot_mix_experiment_into_corner_or_route(tmp_path, kwargs):
    from simulation.continuous_straight_run import run_case
    with pytest.raises(ValueError): run_case(tmp_path/'invalid', straight_variant='gradient_history', **kwargs)
    assert not list(tmp_path.iterdir())


def test_public_task_replay_binds_new_variant_and_real_terminal_frames(tmp_path):
    from simulation.continuous_straight_run import run_case
    from simulation.continuous_straight_acceptance import replay_safety_public
    from tools.straight_pipeline_profile import load_case, controller_for_task
    folder = tmp_path/'run'
    result = run_case(folder, straight_variant='gradient_history', operator_confirmed=True,
                      motion_permission='bounded_probe', max_ticks=10, terminal_check_ticks=3)
    rows = [json.loads(line) for line in (folder/'decisions.jsonl').read_text().splitlines()]
    assert not result['exception'] and result['safety_checks']['terminal_latched']
    assert replay_safety_public(folder, result, rows)['terminal_frames'] == 3
    loaded = load_case(folder)
    assert type(controller_for_task(loaded['task'])) is Controller
    path = folder/'public/task.json'; task = json.loads(path.read_text())
    assert task['straight_variant'] == 'gradient_history'
    task['straight_variant'] = 'gradient_transfer'; path.write_text(json.dumps(task))
    with pytest.raises(ValueError, match='differs'): replay_safety_public(folder, result, rows)
