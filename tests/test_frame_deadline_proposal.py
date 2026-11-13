"""Shadow design invariants; no camera/actuator provenance is certified."""
from dataclasses import replace, FrozenInstanceError
import ast
import json
from pathlib import Path

import pytest

from tools.frame_deadline_proposal import (DeadlineProposal, FrameEvidence, ViewEvidence,
    ResultBinding, ShadowPermit, Rejected, binding_for, LEASE_S, MAX_FRAMES)
from tools import frame_deadline_shadow_probe as probe
from simulation.gazebo_actuator import LeaseGuard
from tests.test_review_gazebo_watchdog import packet, SESSION


def view(**kw):
    return replace(ViewEvidence('forward', 'a'*64, 10., 'bounded_exposure_mapping', 9.9, 9.95), **kw)


def frame(**kw):
    return replace(FrameEvidence(SESSION, 'f1', .1, 'test-boot-monotonic', (view(),)), **kw)


def owner(required_views=('forward',)):
    return DeadlineProposal(SESSION, 'test-boot-monotonic', required_views)


def test_oldest_uncertain_exposure_not_receipt_or_upper_bound_is_anchor():
    o = owner(); f = frame(); token = o.admit(f, 10.)
    assert o.effective_deadline == 9.9+LEASE_S
    p = o.complete(token, binding_for(f), 10.2)
    assert p.issued_steady_s == 10.2 and p.expires_steady_s == 9.9+LEASE_S
    assert p.expires_steady_s < p.issued_steady_s+LEASE_S
    assert not p.motion_authority
    with pytest.raises(FrozenInstanceError): p.expires_steady_s = 100.
    with pytest.raises(TypeError): ShadowPermit(p.binding, 1, 10., 10.8, motion_authority=True)


@pytest.mark.parametrize('source', ['receipt_only', 'simulation_only', 'unknown'])
def test_no_exposure_mapping_cannot_gain_token_even_with_very_recent_receipt(source):
    o = owner()
    with pytest.raises(Rejected, match='exposure_age_unknown'):
        o.admit(frame(views=(view(source=source),)), 10.)
    assert o.latched and o.pending is None and o.seq == 0


def test_two_view_bundle_uses_oldest_view_and_rejects_one_unmapped_view():
    o = owner(('forward', 'down')); f = frame(views=(view(), view(view='down', exposure_lower_s=9.5, exposure_upper_s=9.6)))
    o.admit(f, 10.)
    assert o.effective_deadline == 9.5+LEASE_S
    with pytest.raises(Rejected, match='exposure_age_unknown'):
        owner(('forward','down')).admit(replace(f, views=(view(), view(view='down', source='receipt_only'))), 10.)


def test_producer_cannot_omit_stale_required_camera():
    with pytest.raises(Rejected, match='required_view_set_mismatch'):
        owner(('forward','down')).admit(frame(), 10.)


def test_six_configured_views_are_supported_without_dropping_any():
    names = ('a','b','c','d','up','down')
    o = owner(names)
    o.admit(frame(views=tuple(view(view=name) for name in names)), 10.)
    assert o.effective_deadline == 9.9+LEASE_S


@pytest.mark.parametrize('views', [(), ('forward','forward'), ['forward'], ('a',)*7])
def test_required_view_configuration_is_owned_not_arbitrary(views):
    with pytest.raises(ValueError): owner(views)


@pytest.mark.parametrize('changes', [dict(received_steady_s=10.1), dict(exposure_lower_s=None),
    dict(exposure_lower_s=float('nan')), dict(exposure_upper_s=float('inf')), dict(exposure_lower_s=True),
    dict(exposure_lower_s=10**1000), dict(exposure_lower_s=-1.), dict(exposure_upper_s=10.1),
    dict(exposure_upper_s=9.8), dict(rgb_sha256='wrong'), dict(view=''), dict(received_steady_s=False)])
def test_bad_or_future_capture_evidence_is_sticky_no_motion(changes):
    o = owner()
    with pytest.raises(Rejected): o.admit(frame(views=(view(**changes),)), 10.)
    assert o.latched and o.seq == 0 and o.pending is None


@pytest.mark.parametrize('changes', [dict(session_id='wrong'), dict(clock_domain_id='different-boot'),
    dict(clock_domain_id=''), dict(frame_id=''), dict(captured_sim_s=True), dict(captured_sim_s=float('nan')),
    dict(views=()), dict(views=[view()]), dict(views=(view(), view()))])
def test_frame_clock_session_and_view_shape_are_bound(changes):
    o = owner()
    with pytest.raises(Rejected): o.admit(frame(**changes), 10.)
    assert o.latched and o.pending is None


def test_already_old_frame_never_arms_or_mints_token():
    o = owner()
    with pytest.raises(Rejected, match='capture_budget_exhausted'):
        o.admit(frame(views=(view(exposure_lower_s=9.),)), 10.)
    assert not o.frames and o.pending is None


def test_boundary_is_exact_no_observation_grace_and_new_frame_cannot_rearm():
    o = owner(); f = frame(); t = o.admit(f, 10.)
    d = o.effective_deadline
    with pytest.raises(Rejected, match='expired'): o.complete(t, binding_for(f), d)
    with pytest.raises(Rejected, match='expired'): o.admit(frame(frame_id='f2', captured_sim_s=.2,
        views=(view(received_steady_s=11., exposure_lower_s=11., exposure_upper_s=11.),)), 11.)
    assert o.effective_deadline == d and o.seq == 0


def test_new_pending_does_not_extend_held_command_but_timely_result_can_replace_it():
    o = owner(); f = frame(); t = o.admit(f, 10.)
    p = o.complete(t, binding_for(f), 10.1)
    f2 = frame(frame_id='f2', captured_sim_s=.2, views=(view(received_steady_s=10.2,
        exposure_lower_s=10.2, exposure_upper_s=10.2),))
    t2 = o.admit(f2, 10.2)
    assert o.hold_deadline == o.effective_deadline == p.expires_steady_s
    assert o.pending['deadline'] > p.expires_steady_s
    p2 = o.complete(t2, binding_for(f2), 10.3)
    assert p2.seq == 2 and o.effective_deadline == f2.views[0].exposure_lower_s+LEASE_S


def test_pending_overwrite_is_not_renewal():
    o = owner(); o.admit(frame(), 10.); d = o.effective_deadline
    with pytest.raises(Rejected, match='overwrite'): o.admit(frame(frame_id='f2', captured_sim_s=.2), 10.1)
    assert o.effective_deadline == d and o.seq == 0


@pytest.mark.parametrize('field,value', [('session_id','other'),('frame_id','f2'),('captured_sim_s',.2),
    ('captured_sim_s',True),('view_digests',(('forward','b'*64),)),('view_digests',(('other','a'*64),)),
    ('view_digests',[['forward','a'*64]])])
def test_result_binds_exact_session_frame_sim_and_all_image_digests(field, value):
    o = owner(); f = frame(); t = o.admit(f, 10.)
    with pytest.raises(Rejected, match='unbound'):
        o.complete(t, replace(binding_for(f), **{field:value}), 10.1)
    assert o.seq == 0 and o.latched


def test_consumed_token_and_same_frame_are_never_reissued():
    o = owner(); f = frame(); t = o.admit(f, 10.)
    p = o.complete(t, binding_for(f), 10.1)
    with pytest.raises(Rejected): o.complete(t, binding_for(f), 10.2)
    assert o.seq == 1 and o.hold_deadline == p.expires_steady_s
    o = owner(); t = o.admit(f, 10.); o.complete(t, binding_for(f), 10.1)
    with pytest.raises(Rejected, match='reused'): o.admit(f, 10.2)


@pytest.mark.parametrize('now', [9., False, float('nan'), float('inf')])
def test_clock_regression_or_bad_clock_cannot_extend_lease(now):
    o = owner(); o.admit(frame(), 10.); d = o.effective_deadline
    o.poll(now)
    assert o.latched and o.reason == 'clock_invalid_or_regressed' and o.effective_deadline == d


def test_registry_exhaustion_is_terminal_not_eviction():
    o = owner(); o.frames = {f'old-{i}' for i in range(MAX_FRAMES)}
    with pytest.raises(Rejected, match='capacity'): o.admit(frame(), 10.)
    assert len(o.frames) == MAX_FRAMES and o.pending is None


def test_original_lease_guard_already_honors_shorter_original_expires():
    # Compatibility of existing arithmetic only, NOT capture-token integration.
    o = owner(); f = frame(); t = o.admit(f, 10.); p = o.complete(t, binding_for(f), 10.2)
    original = LeaseGuard(SESSION)
    wire = packet(issued=p.issued_steady_s, expires_steady_s=p.expires_steady_s)
    wire['command'].update(state='observing', v_m_s=0.)
    assert original.handle(wire, 10.3)
    assert original.gate.hold_deadline == p.expires_steady_s
    assert original.poll(p.expires_steady_s) == (0.,0.) and original.gate.latched


@pytest.mark.parametrize('case', probe.CASES)
def test_private_process_shadow_probes_only(tmp_path, case):
    result = probe.run_one(case, tmp_path/case)
    assert result['declared_observations_met'], result
    assert result['child_reaped'] and not result['motion_authority']
    assert not result['actual_exposure_measured'] and not result['actual_controller_compute_measured']
    if case == 'owner_stopped':
        assert result['timely_latch_proved'] is False


def test_existing_output_is_not_overwritten(tmp_path):
    p = tmp_path/'data.json'; probe.write_json(p, {'first':True})
    with pytest.raises(FileExistsError): probe.write_json(p, {})
    assert json.loads(p.read_text()) == {'first':True}


@pytest.mark.parametrize('repeats', [True, 0, -1, 1.5])
def test_invalid_repeat_does_not_launch_or_create_output(tmp_path, repeats):
    with pytest.raises(ValueError): probe.run(tmp_path/'output', repeats)
    assert not (tmp_path/'output').exists()


def test_ros_world_already_arms_before_publish_not_after_navigation():
    # Read-only source-order audit, deliberately no ROS initialization/imports.
    root = Path(__file__).resolve().parents[1]
    tree = ast.parse((root/'ros2_ws/src/toponav_bridge/toponav_bridge/nodes.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'World')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'frame')
    source = ast.unparse(method)
    assert source.index('self.world.capture()') < source.index('self.gate.arm(') < source.index('self.camera.publish(')


def test_gazebo_post_compute_gap_and_production_remain_unmodified_by_shadow():
    root = Path(__file__).resolve().parents[1]
    source = (root/'simulation/gazebo_cross_validation.py').read_text()
    assert source.index('d = controller.update(frame, now)') < source.index('gate.arm(frame["frame_id"]')
    for folder in ('simulation','roadvision','ros2_ws/src/toponav_bridge/toponav_bridge'):
        for path in (root/folder).glob('*.py'):
            assert 'frame_deadline_proposal' not in path.read_text(), path
