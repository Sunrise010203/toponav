"""Unit evidence is synthetic; native Gazebo trials are recorded separately."""
import copy
import math
import cv2
import numpy as np
import pytest
from roadvision.landmark_route import LandmarkRouteController, landmarks, guide
from simulation.gazebo_route_fixture import registry
from simulation.gazebo_scenes import scenario_pair, build_scenario_world
from simulation.gazebo_cross_validation import CameraBuffer
import xml.etree.ElementTree as ET


def image(marker=1, yaw=0, x=0, y=0):
    canvas = np.full((256,256,3),220,np.uint8)
    pixels = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),marker,64)
    canvas[96:160,96:160] = pixels[:,:,None]
    # An up-facing camera sees map yaw as negative image angle.
    transform = cv2.getRotationMatrix2D((127.5,127.5),math.degrees(yaw),1.)
    transform[:,2] += np.array([-y*200,x*200])
    return cv2.warpAffine(canvas, transform,(256,256),borderValue=(220,220,220))


def frame(t, marker=1, yaw=0, x=0, y=0):
    return dict(frame_id=f'frame-{t}',captured_sim_s=t,up_rgb=image(marker,yaw,x,y),
                down_rgb=np.full((256,256,3),80,np.uint8))


def controller(disabled=()):
    graph,_=scenario_pair('route',disabled)
    return LandmarkRouteController(graph,registry(graph))


def test_real_marker_decoder_position_and_orientation():
    graph,_=scenario_pair('route')
    p=landmarks(image(3,math.pi/2,.16,.08),registry(graph))['N03']
    assert p['x']==pytest.approx(.16,abs=.01)
    assert p['y']==pytest.approx(.08,abs=.01)
    assert p['yaw']==pytest.approx(math.pi/2,abs=.02)


def test_repeated_frame_and_missing_camera_are_latched_zero():
    c=controller();f=frame(.1)
    c.update(f,.1)
    d=c.update(f,.2)
    assert d['state']=='aborted' and d['v_m_s']==d['omega_rad_s']==0
    assert c.update(frame(.3),.3)['state']=='aborted'
    c=controller();f=frame(.1);del f['up_rgb']
    assert c.update(f,.1)['state']=='aborted'


@pytest.mark.parametrize('now', [float('nan'),1.,-.1])
def test_invalid_clock_cannot_authorize_motion(now):
    d=controller().update(frame(.1),now)
    assert d['state']=='aborted' and d['v_m_s']==d['omega_rad_s']==0


def test_unreachable_task_does_not_move_or_confirm_a_node():
    c=controller(['C06']);d=c.update(frame(.1),.1)
    assert d['reason']=='goal_unreachable_no_motion'
    assert c.visited==[] and c.motion_commands==0


def test_wrong_node_never_counts_as_progress():
    c=controller();d=c.update(frame(.1,marker=4),.1)
    assert d['state']=='aborted' and c.visited==[]


def test_three_fresh_zero_holds_confirm_once_then_align():
    c=controller()
    for t in [.1,.2,.3]:
        d=c.update(frame(t),t)
        assert d['v_m_s']==d['omega_rad_s']==0
    assert c.visited==['N01'] and c.phase=='align'
    for t in [.4,.5,.6]:
        c.update(frame(t),t)
    assert c.phase=='cruise' and c.visited==['N01']
    assert c.update(frame(.7),.7)['v_m_s'] > 0
    assert len(c.events)==1


def test_marker_loss_during_turn_fails_closed():
    c=controller()
    for t in [.1,.2,.3]:c.update(frame(t),t)
    f=frame(.4);f['up_rgb'][:]=220
    d=c.update(f,.4)
    assert d['reason']=='turn_marker_missing' and d['v_m_s']==d['omega_rad_s']==0


def test_absent_target_and_guide_cannot_drive_on_previous_commands():
    c=controller()
    for t in [.1,.2,.3,.4,.5,.6]:c.update(frame(t),t)
    f=frame(.7);f['up_rgb'][:]=220
    d=c.update(f,.7)
    assert d['state']=='aborted' and d['reason']=='guide_line_missing'


def test_goal_requires_full_ordered_fresh_node_confirmations():
    c=controller();t=0.
    # Unit state transitions, not a physics/navigation acceptance result.
    for marker,yaw in [(1,0),(2,0),(3,0),(4,math.pi/2),(6,0)]:
        for _ in range(3):
            t+=.1;c.update(frame(t,marker,yaw),t)
        assert c.visited[-1]=='N%02d'%marker
        if marker !=6:
            for _ in range(3):
                t+=.1;c.update(frame(t,marker,c.heading),t)
            assert c.phase=='cruise'
    assert c.state=='visual_stop'
    assert c.visited==c.route['nodes']
    assert len(set(e['frame_id'] for e in c.events))==5
    d=c.update(frame(t+.1,6),t+.1)
    assert d['v_m_s']==d['omega_rad_s']==0


def test_closure_uses_graph_not_fixture_coordinates():
    c=controller(['C02'])
    assert c.route['nodes']==['N01','N02','N05','N04','N06']
    assert not hasattr(c,'scene')
    assert set(c.labels.values())==set(c.nodes)


def test_guide_missing_and_yellow_strip_centering():
    rgb=np.full((256,256,3),50,np.uint8)
    assert guide(rgb) is None
    rgb[:,120:126]=[250,210,0]
    lateral,angle=guide(rgb)
    assert lateral>0 and abs(angle)<.001


def test_auxiliary_buffer_demands_original_stamp_match():
    c=CameraBuffer(('up','down'))
    for view in c.views:
        c.put(view,20 if view=='up' else 10,np.zeros((128,128,3),np.uint8))
    assert c.latest() is None
    c.put('up',10,np.zeros((256,256,3),np.uint8))
    assert c.latest()[0]==10


def test_augmentation_preserves_original_map_and_physics_and_fault_is_real():
    graph,scene=scenario_pair('route');before=copy.deepcopy((graph,scene))
    tree=ET.fromstring(build_scenario_world('route',graph,scene,('N03',)))
    assert (graph,scene)==before
    assert all(v['width']==.8 for v in scene['geometry']['channel_dimensions_m'].values())
    names={m.get('name') for m in tree.findall('world/model')}
    assert 'tag_back_N01' in names and 'tag_back_N03' not in names
    assert 'guide_C01' in names
    assert tree.find("world/model[@name='guide_C01']/link/collision") is None
    assert tree.find("world/model[@name='observer']/link/sensor/camera/visibility_mask").text=='1'
    assert tree.find("world/model[@name='tag_back_N01']/link/visual/visibility_flags").text=='2'
    assert tree.find("world/model[@name='robot']/link/sensor[@name='up']/camera/visibility_mask") is None
