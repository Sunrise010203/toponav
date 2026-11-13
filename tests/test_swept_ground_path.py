"""Task4 swept-envelope/coverage contracts, not real-world collision proof."""
import math
import numpy as np
import pytest
from roadvision import swept_ground_path as s
from roadvision.continuous_straight import StraightPolicy


def test_full_wheels_and_camera_offset_are_declared():
    p=s.SweepInstallation()
    assert p.half_length_m==.16 and p.half_width_m==.17 and p.camera_forward_m==.2
    r=s.predict_sweep(.15,.2,age_s=0.,policy=StraightPolicy(),installation=p)
    assert r['bounds_m'][0]<-.36 and r['bounds_m'][1]<-.22 and r['bounds_m'][3]>.22
    assert r['certified_clearance'] is False and r['motion_authorized'] is False


def test_reaction_and_braking_are_not_just_a_centerline():
    r=s.predict_sweep(.15,0.,age_s=.1,policy=StraightPolicy())
    assert r['maximum_translation_m']==pytest.approx(.18*.35+.18**2/(2*.25))
    assert r['maximum_abs_yaw_rad']==pytest.approx(.35*.35+.35**2/(2*.5))
    assert r['bounds_m'][3]>.17+.05


@pytest.mark.parametrize('omega',[-.2,0.,.2])
def test_conservative_tube_contains_random_actual_bounded_trajectories(omega):
    r=s.predict_sweep(.15,omega,age_s=.1,policy=StraightPolicy())
    lo_x,lo_y,hi_x,hi_y=r['bounds_m'];rng=np.random.default_rng(45)
    for _ in range(60):
        xy=np.zeros(2);yaw=0.;v=rng.uniform(.1,.18);w=rng.uniform(-.35,.35);dt=.001
        for i in range(1100):
            t=i*dt
            if t>=.35:v=max(0,v-.25*dt);w=math.copysign(max(0,abs(w)-.5*dt),w)
            xy+=v*dt*np.array([math.cos(yaw),math.sin(yaw)]);yaw+=w*dt
            if i%20==0:
                rot=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
                for x in (-.16,.16):
                    for y in (-.17,.17):
                        q=xy+rot@np.array([x,y])-np.array([.2,0])
                        assert lo_x<=q[0]<=hi_x and lo_y<=q[1]<=hi_y


def test_latency_uncertainty_and_larger_body_only_enlarge_tube():
    a=s.predict_sweep(.15,.2,age_s=0.,policy=StraightPolicy())
    b=s.predict_sweep(.15,.2,age_s=.2,policy=StraightPolicy(),translation_uncertainty_m=.03,yaw_uncertainty_rad=.04,
                      installation=s.SweepInstallation(half_width_m=.2))
    for i in (0,1):assert b['bounds_m'][i]<=a['bounds_m'][i]
    for i in (2,3):assert b['bounds_m'][i]>=a['bounds_m'][i]


@pytest.mark.parametrize('kw',[dict(half_width_m=.11),dict(half_length_m=.15),dict(camera_forward_m=.1),
                             dict(margin_m=.01),dict(yaw_speed_bound_rad_s=.2),dict(minimum_yaw_deceleration_rad_s2=.6)])
def test_installation_cannot_silently_omit_wheels_or_reduce_margins(kw):
    with pytest.raises(ValueError):s.SweepInstallation(**kw)


@pytest.mark.parametrize('v,w,age',[(True,0.,0.),(.16,0.,0.),(.1,.21,0.),(.1,0.,.26),(.1,0.,float('nan')),(-.1,0,0)])
def test_invalid_command_or_clock_rejected(v,w,age):
    with pytest.raises(ValueError):s.predict_sweep(v,w,age_s=age,policy=StraightPolicy())


def box(x0,y0,x1,y1):return [[x0,y0],[x1,y0],[x1,y1],[x0,y1]]


def test_no_samples_or_point_samples_cannot_certify_continuous_area():
    r=s.coverage_audit(box(0,0,.1,.1),[],cell_m=.02)
    assert not r['covered'] and r['unknown_cells']==r['required_cells']>0
    with pytest.raises(ValueError):s.coverage_audit(box(0,0,.1,.1),[dict(accepted=True,polygon=[[.05,.05]])],cell_m=.02)


def test_entire_cell_needs_one_supported_patch_not_just_its_center():
    r=s.coverage_audit(box(0,0,.1,.1),[dict(accepted=True,polygon=box(.001,.001,.099,.099))],cell_m=.02)
    assert not r['covered'] and r['unknown_cells']>0
    assert s.coverage_audit(box(0,0,.1,.1),[dict(accepted=True,polygon=box(-.02,-.02,.12,.12))],cell_m=.02)['covered']


def test_rejected_patches_and_holes_never_turn_into_free_space():
    patches=[dict(accepted=True,polygon=box(-.02,-.02,.04,.12)),dict(accepted=True,polygon=box(.06,-.02,.12,.12))]
    assert not s.coverage_audit(box(0,0,.1,.1),patches,cell_m=.02)['covered']
    patches.append(dict(accepted=False,polygon=box(0,0,.1,.1)))
    assert not s.coverage_audit(box(0,0,.1,.1),patches,cell_m=.02)['covered']


def test_side_wall_and_front_intersection_checks_include_corners_and_uncertainty():
    plane=dict(status='conditional_vertical_plane',normal_rig=[0,1,0],distance_m=.23,
               conditional_interval=dict(distance_half_width_m=.01,normal_angle_half_width_rad=.01))
    r=s.assess_plane_clearance(box(-.4,-.28,.15,.28),dict(left=plane))
    assert not r['supported'] and r['checks']['left']['status']=='intersection_or_uncertain'
    plane['distance_m']=.9
    assert s.assess_plane_clearance(box(-.4,-.28,.15,.28),dict(left=plane))['supported']
    plane['conditional_interval']['distance_half_width_m']=.7
    assert not s.assess_plane_clearance(box(-.4,-.28,.15,.28),dict(left=plane))['supported']


def test_missing_wall_proof_is_not_empty_space():
    assert not s.assess_plane_clearance(box(-.4,-.28,.15,.28),{})['supported']
