import json
import numpy as np
from simulation.continuous_audit_world import ContinuousAuditWorld
from simulation.corridor_motion import CorridorConfig


def test_physics_audit_is_every_step_and_does_not_change_public_envelope(tmp_path):
    with ContinuousAuditWorld(CorridorConfig(length_m=4.),assets=tmp_path/'assets') as w:
        assert len(w.physics_trace)==60
        before=len(w.physics_trace);w.drive(.1,.03);w.step_control()
        assert len(w.physics_trace)==before+12
        t=np.array([r['sim_s'] for r in w.physics_trace]);np.testing.assert_allclose(np.diff(t),1/120,atol=1e-12)
        assert set(w.capture())=={'frame_id','captured_sim_s','rgb'}
        assert w.physics_trace[-1]['speed_m_s']>0.
        assert all(set(r)=={'sim_s','position_m','speed_m_s','yaw_rate_rad_s','wall_contact_physics_steps'} for r in w.physics_trace)
        contacts=[r['wall_contact_physics_steps'] for r in w.physics_trace]
        assert all(type(c) is int and c>=0 for c in contacts)
        assert all(a<=b for a,b in zip(contacts,contacts[1:]))
        assert contacts[-1]==w.wall_contact_physics_steps==0


def test_quarter_meter_blocker_is_actual_collision_geometry_not_a_label(tmp_path):
    import pybullet as p
    with ContinuousAuditWorld(CorridorConfig(length_m=4.,obstacle='narrow'),assets=tmp_path/'assets',narrow_obstacle_width_m=.25) as w:
        geometry=p.getCollisionShapeData(w.walls[-1],-1,physicsClientId=w.client)
        np.testing.assert_allclose(geometry[0][3],[.1,.25,1.6],atol=1e-10)
        assert set(w.capture())=={'frame_id','captured_sim_s','rgb'}
