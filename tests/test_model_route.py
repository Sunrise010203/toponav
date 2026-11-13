"""Closed-loop contract tests. Fake predictions test guards, not model accuracy."""
import copy
import hashlib
import json
import math
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import pytest

from roadvision.contract import to_observation
from roadvision.model_route import (ModelRouteController, FrozenModelPerception,
                                   HEIGHT, WIDTH, CAMERA_OFFSET, MODEL_DIR)
from roadvision.visual_corridor_geometry import measurements, incoming_geometry, floor_mask, enclosed_side_walls
from simulation.gazebo_scenes import scenario_pair, build_scenario_world

ROOT=Path(__file__).resolve().parents[1]
FIXTURES=Path(__file__).parent/'fixtures/model_route'
SHA='1188f35dd20d0773b780b73f2000daa25fede9b2db098b02dc0746ad0de6b03b'


class Prediction:
    metadata={'model_sha256':SHA}
    index=8
    def predict(self,frame,now):
        probabilities=[0.]*17;probabilities[self.index]=1.
        obs=to_observation(probabilities,.6,True,now-frame['captured_sim_s'])
        return dict(accepted=obs['confidence']>0,observation=obs,
                    class_id='unobservable' if self.index==16 else f'exits_{self.index:04b}',score=1.)


def frame(t, pixels=None):
    return dict(frame_id=f'test/{t}',captured_sim_s=t,
                rgb=np.array(Image.open(FIXTURES/'start.png')) if pixels is None else pixels)


def controller():
    return ModelRouteController(scenario_pair('vision')[0],Prediction())


def assert_zero(d):
    assert d['v_m_s']==d['omega_rad_s']==0.


def test_actual_frozen_model_uses_original_preprocessing_threshold_and_sim_clock():
    import torch
    torch.set_num_threads(2)
    p=FrozenModelPerception()
    assert p.metadata['threshold']==.6
    assert hashlib.sha256((MODEL_DIR/'model.pt').read_bytes()).hexdigest()==SHA
    result=p.predict(frame(10),10.1)
    assert result['class_id']=='exits_1000' and result['accepted']
    assert result['model_sha256']==SHA and len(result['probabilities'])==17
    assert result['captured_sim_s']==10 and result['decision_sim_s']==10.1
    assert not p.predict(frame(10),11.01)['accepted']


@pytest.mark.parametrize('index',[0,9,16])
def test_no_node_or_exit_authorization_from_wrong_or_rejected_model(index):
    c=controller();c.perception.index=index
    for i in range(1,20):assert_zero(c.update(frame(i/10),i/10))
    assert c.visited==c.events==[]


def test_three_fresh_zero_model_frames_feed_original_localizer_and_planner():
    c=controller()
    for t in (.1,.2,.3):assert_zero(c.update(frame(t),t))
    assert c.visited==['N01'] and c.phase=='align'
    assert c.localization['status']=='localized'
    assert c.planner['action']=='straight' and c.target=='N02'
    assert [e['event'] for e in c.events]==['model_node_confirmed','model_authorized_exit']
    assert all(e['frame_id']=='test/0.3' for e in c.events)


@pytest.mark.parametrize('bad', ['duplicate','stale','future','missing','nan'])
def test_bad_original_camera_evidence_latches_zero(bad):
    c=controller();c.update(frame(.1),.1)
    f=frame(.2);now=.2
    if bad=='duplicate':f=frame(.1)
    if bad=='stale':now=1.
    if bad=='future':f['captured_sim_s']=2.
    if bad=='missing':f['rgb']=np.zeros((2,2,3),np.uint8)
    if bad=='nan':now=float('nan')
    d=c.update(f,now);assert_zero(d);assert d['state']=='aborted'
    assert_zero(c.update(frame(1.1),1.1))
    assert not c.events


def test_unreachable_public_graph_never_moves_even_with_accepted_model():
    graph,_=scenario_pair('vision',['C06'])
    c=ModelRouteController(graph,Prediction())
    d=c.update(frame(.1),.1)
    assert_zero(d);assert d['reason']=='goal_unreachable_no_motion'
    assert not c.events


def enter_dock(c):
    for t in (.1,.2,.3):c.update(frame(t),t)
    c.phase='dock';c.departed=True;c.phase_start=.3
    c.perception.index=13  # N02 east: front, left, back


def test_arrival_requires_settling_then_separate_fresh_model_confirmation():
    c=controller();enter_dock(c)
    incoming=dict(angle=0.,offset=0.,support=20)
    with patch('roadvision.model_route.incoming_geometry',return_value=incoming), patch('roadvision.model_route.station_distance',return_value=(.02,'side_corridor_center')):
        for t in (.4,.5,.6):assert_zero(c.update(frame(t),t))
        assert c.phase=='confirm_node' and c.visited==['N01']
        assert c.events[-1]['event']=='visual_edge_station_stopped'
        for t in (.7,.8):assert_zero(c.update(frame(t),t))
        assert c.visited==['N01']
        assert_zero(c.update(frame(.9),.9))
    assert c.visited==['N01','N02']
    assert c.events[-2]['captured_sim_s']>.6
    assert len({e['event_id'] for e in c.events})==len(c.events)


def test_model_rejection_during_docking_cannot_complete_edge_and_times_out():
    c=controller();enter_dock(c);c.perception.index=16
    with patch('roadvision.model_route.incoming_geometry',return_value=dict(angle=0.,offset=0.)):
        for t in (.4,1.,2.,3.,4.,4.5):assert_zero(c.update(frame(t),t))
    assert c.state=='aborted' and c.reason=='model_evidence_hold_timeout'
    assert c.visited==['N01']
    assert not any(e['event']=='visual_edge_station_stopped' for e in c.events)


def test_missing_corridor_geometry_stops_without_advancing_localization():
    c=controller();enter_dock(c)
    d=c.update(frame(.4,np.full((256,256,3),120,np.uint8)),.4)
    assert_zero(d);assert d['reason']=='incoming_corridor_geometry_missing'
    assert c.visited==['N01']


def test_even_in_transit_a_missing_view_cannot_be_treated_as_model_unknown():
    c=controller();enter_dock(c);c.phase='cruise'
    rgb=frame(.4)['rgb'];rgb[:128,128:]=0
    d=c.update(frame(.4,rgb),.4)
    assert_zero(d);assert d['reason']=='missing_original_rgb_view'


def test_wall_departure_is_separate_visual_evidence_not_a_fabricated_model_class():
    c=controller();enter_dock(c);c.phase='cruise';c.departed=False;c.perception.index=16
    rgb=frame(.4)['rgb']
    rgb[:128,128:]=(126,123,119);rgb[128:,:128]=(153,150,145)
    assert enclosed_side_walls(rgb)
    with patch('roadvision.model_route.incoming_geometry',return_value=dict(angle=0.,offset=0.)),patch('roadvision.model_route.station_distance',return_value=(1.,'side_corridor_center')):
        for t in (.4,.5,.6):d=c.update(frame(t,rgb),t)
    assert c.departed and c.departure_source=='rgb_both_side_walls_enclosed'
    assert d['model']['class_id']=='unobservable' and not d['model']['accepted']
    assert c.visited==['N01']
    assert not enclosed_side_walls(np.zeros_like(rgb))


def test_short_corridor_uses_observed_near_wall_join_not_junction_silhouette():
    rgb=np.array(Image.open(FIXTURES/'short_corridor.png'))
    m=measurements(rgb,HEIGHT,WIDTH,CAMERA_OFFSET)
    g=incoming_geometry(m)
    assert g is not None and abs(g['angle'])<math.radians(5) and abs(g['offset'])<.03
    assert m['left'] is None and m['right'] is None


def test_hood_gray_wall_and_ceiling_are_not_blue_floor():
    rgb=np.array([[(113,134,156),(99,108,113),(126,123,119),(111,111,111)]],np.uint8)
    assert floor_mask(rgb).tolist()==[[True,False,False,False]]


def test_fixture_is_original_size_without_tags_paint_or_auxiliary_inputs():
    graph,scene=scenario_pair('vision')
    assert graph==json.loads((ROOT/'configs/graph.json').read_text())
    assert scene==json.loads((ROOT/'configs/scene.json').read_text())
    xml=build_scenario_world('vision',graph,scene)
    assert 'marker' not in xml and 'guide' not in xml
    world=ET.fromstring(xml).find('world')
    cams=world.findall("model[@name='robot']/link/sensor[@type='camera']")
    assert [c.get('name') for c in cams]==['front','left','right','back']
    for c in cams:
        x,y,z,*_=map(float,c.findtext('pose').split())
        assert math.hypot(x,y)==pytest.approx(CAMERA_OFFSET)
        assert z+.16==pytest.approx(HEIGHT)
    assert world.find("model[@name='ceiling_0']/link/collision") is not None


def test_controller_sources_have_no_simulator_truth_or_marker_imports():
    import ast
    for name in ('model_route','visual_corridor_geometry'):
        source=(ROOT/f'roadvision/{name}.py').read_text()
        modules=[n.module for n in ast.walk(ast.parse(source)) if isinstance(n,ast.ImportFrom)]
        assert not any(m and ('gazebo' in m or 'landmark' in m or 'pybullet' in m) for m in modules)
        assert 'scene[' not in source and 'position_m' not in source


def test_public_replay_recomputes_model_and_checks_graph_model_and_rgb_hashes(tmp_path):
    from simulation.gazebo_replay import replay
    import torch
    torch.set_num_threads(2)
    graph,_=scenario_pair('vision')
    c=ModelRouteController(graph)
    public=tmp_path/'public';(public/'frames').mkdir(parents=True)
    (public/'topology.json').write_text(json.dumps(graph))
    names=['roadvision/continuous_straight.py','simulation/gazebo_scenes.py',
           'roadvision/model_route.py','roadvision/visual_corridor_geometry.py',
           'roadvision/learn.py','roadvision/contract.py','simulation/gazebo_vision_fixture.py',
           'simulation/structural_localization.py','simulation/path_planning.py',
           'simulation/topology.py','simulation/dijkstra.py','roadvision/data.py',
           'models/road_structure_frozen_v1_20261014/model.pt',
           'models/road_structure_frozen_v1_20261014/metadata.json']
    frozen={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in names}
    frozen['public/topology.json']=hashlib.sha256((public/'topology.json').read_bytes()).hexdigest()
    (tmp_path/'freeze.json').write_text(json.dumps(frozen))
    rows=[]
    for i,t in enumerate((.1,.2,.3)):
        f=frame(t);Image.fromarray(f['rgb']).save(public/f'frames/{i}.png')
        d=c.update(f,t);d['model']['inference_ms']=99999.
        rows.append(dict(tick=i,sim_s=t,input=dict(frame_id=f['frame_id'],captured_sim_s=t,
            rgb_path=f'frames/{i}.png',rgb_sha256=hashlib.sha256(f['rgb'].tobytes()).hexdigest()),decision=d))
    (tmp_path/'decisions.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    r=replay(tmp_path,'left',scenario='vision')
    assert r['all_decisions_match'] and r['excluded_diagnostic_fields']==['model.inference_ms']
    rows[0]['decision']['model']['probabilities'][0]+=.01
    (tmp_path/'decisions.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    assert not replay(tmp_path,'left',scenario='vision')['all_decisions_match']
    (public/'topology.json').write_text('{}')
    with pytest.raises(ValueError,match='Public graph differs'):replay(tmp_path,'left',scenario='vision')
