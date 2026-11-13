import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.dijkstra import plan
from simulation.gazebo_cross_validation import build_world
from simulation.gazebo_scenes import (SCENARIOS, PreviewController, build_scenario_world,
    gui_config, make_controller, road_rectangles, scenario_pair, topology_image, union_cells)
from simulation.topology import validate_pair


@pytest.mark.parametrize("name", SCENARIOS)
def test_each_scene_has_valid_topology_native_wheels_and_physical_walls(name):
    graph,scene = scenario_pair(name)
    validate_pair(graph,scene)
    world = ET.fromstring(build_scenario_world(name,graph,scene)).find("world")
    assert world.find("model[@name='robot']/plugin[@name='gz::sim::systems::DiffDrive']") is not None
    assert len(world.findall("model[@name='robot']/link/sensor[@type='camera']")) == (6 if name == 'route' else 4)
    assert any(m.get("name").startswith("wall_") for m in world.findall("model"))
    assert topology_image(graph,scene).shape == (500,720,3)


@pytest.mark.parametrize("name", ["left","right"])
def test_previously_validated_l_fixture_is_not_rebuilt(name):
    assert build_scenario_world(name) == build_world(name)


def test_six_node_scene_is_from_project_files_not_an_invented_graph():
    root = Path(__file__).resolve().parents[1]
    graph,scene = scenario_pair("topology")
    assert graph == json.loads((root/"configs/graph.json").read_text())
    assert scene == json.loads((root/"configs/scene.json").read_text())
    assert len(graph["nodes"]) == len(graph["edges"]) == 6


def test_closed_channel_replans_and_is_a_real_physical_barrier():
    graph,scene = scenario_pair("topology", ["C02"])
    assert plan(graph)["nodes"] == ["N01","N02","N05","N04","N06"]
    world = ET.fromstring(build_scenario_world("topology",graph,scene)).find("world")
    blocker = world.find("model[@name='wall_blocker_C02']")
    assert blocker.find("link/collision/geometry/box/size").text == "0.12 0.8 1.6"
    assert list(map(float,blocker.find("pose").text.split()[:2])) == [3.,0.]
    assert all(e["enabled"] for e in scenario_pair("topology")[0]["edges"])


def test_closed_goal_channel_is_unreachable_not_fabricated_route():
    graph,scene = scenario_pair("topology", ["C06"])
    assert plan(graph)["status"] == "unreachable"
    assert plan(graph)["nodes"] == []
    assert "wall_blocker_C06" in build_scenario_world("topology",graph,scene)


def test_goal_change_uses_existing_public_planner():
    graph,_ = scenario_pair("topology",goal="N05")
    assert plan(graph)["nodes"] == ["N01","N02","N05"]


@pytest.mark.parametrize("disabled,goal", [(["missing"],None),([],"missing"),([],"N01")])
def test_invalid_preview_overrides_rejected(disabled,goal):
    with pytest.raises(ValueError):
        scenario_pair("topology",disabled,goal)


def test_non_preview_cannot_silently_apply_task_overrides():
    with pytest.raises(ValueError):
        scenario_pair("t-left",["C01"])


def test_t_branch_uses_union_boundary_no_internal_junction_wall():
    xs,ys,cells = union_cells([(-.2,3.4,-.9,.9),(1.6,3.4,-3.5,3.5)])
    # All entry / branch cells touching the shared seam remain neighbors.
    for i,j in cells:
        if xs[i] == 1.6 and ys[j] == -.9:
            assert (i-1,j) in cells
            assert (i,j-1) in cells and (i,j+1) in cells


def test_builder_does_not_draw_diagonal_channel_as_orthogonal_road():
    _,scene = scenario_pair("topology")
    scene["geometry"]["node_positions_m"]["N01"] = [0,1]
    with pytest.raises(ValueError,match="orthogonal"):
        road_rectangles(scene)


def test_preview_never_claims_autonomous_execution_or_emits_motion():
    controller = make_controller("topology",True)
    assert isinstance(controller,PreviewController)
    for t in range(20):
        d = controller.update(dict(frame_id=str(t),captured_sim_s=t),t)
        assert d["v_m_s"] == d["omega_rad_s"] == 0
    assert controller.result()["result_type"] == "scene_preview_only"
    assert controller.result()["arrival_status"] == "unverified"


def test_actual_unchanged_primitive_types_and_t_direction():
    from roadvision.continuous_straight import StraightController,CornerController
    for name in ("straight","blocked"):
        assert type(make_controller(name,False)) is StraightController
    for name in ("t-left","t-right"):
        c = make_controller(name,False)
        assert type(c) is CornerController
        assert c.junction == "t" and c.direction == name[2:]


def test_gui_has_separate_topology_topic_not_a_driver_camera():
    # Gazebo config files have several XML top-level elements.
    text = gui_config("topology").split("?>",1)[1]
    root = ET.fromstring("<config>"+text+"</config>")
    assert root.find("plugin[@filename='ImageDisplay']/topic").text == "/toponav/topology_display"
    assert root.find("plugin[@filename='WorldControl']") is None


def test_audit_pose_only_changes_display_not_graph_or_planned_path():
    graph,scene = scenario_pair("topology")
    before = json.dumps([graph,scene],sort_keys=True)
    a = topology_image(graph,scene)
    b = topology_image(graph,scene,dict(position_m=[2,0,.16],yaw_deg=90))
    assert not np.array_equal(a,b)
    assert json.dumps([graph,scene],sort_keys=True) == before
    assert plan(graph)["nodes"] == ["N01","N02","N03","N04","N06"]


def test_native_chooser_controls_and_prior_guard_without_launching_gazebo():
    pytest.importorskip("PyQt5.QtWidgets")
    root = Path(__file__).resolve().parents[1]
    code = '''
import tempfile
from pathlib import Path
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QImage, QColor
from tools.gazebo_scenarios_app import ScenarioWindow
app=QApplication([])
with tempfile.TemporaryDirectory() as directory:
 w=ScenarioWindow(directory,auto_open=False)
 assert w.scenario.count()==9
 assert w.scenario.currentData()=='route'
 assert not w.navigate.isEnabled()
 w.scenario.setCurrentIndex(w.scenario.findData('vision'))
 assert w.auxiliary_group.isHidden()
 assert not w.model_status.isHidden()
 assert '17' in w.note.text() and w.goal.isEnabled()
 assert w.confirmed.isEnabled()==False
 w.scenario.setCurrentIndex(w.scenario.findData('route'))
 w.closed[1].setChecked(True)
 assert 'N05' in w.route.text()
 w.closed[5].setChecked(True)
 assert '不可达' in w.route.text()
 w.scenario.setCurrentIndex(w.scenario.findData('t-left'))
 w.launch()
 assert '确认' in w.status.text()
 assert w.worker.processId()==0
 # An idle poll must not replace a newly selected map with the old run.
 w.folder=Path(directory)/'old'
 (w.folder/'public').mkdir(parents=True)
 image=QImage(20,20,QImage.Format_RGB888)
 image.fill(QColor('red'))
 image.save(str(w.folder/'public/topology.png'))
 before=w.map.pixmap().toImage()
 w.poll()
 assert w.map.pixmap().toImage()==before
 w.close()
print('chooser controls pass')
'''
    env = dict(os.environ,QT_QPA_PLATFORM="offscreen",PYTHONDONTWRITEBYTECODE="1")
    result = subprocess.run([sys.executable,"-c",code],cwd=root,env=env,capture_output=True,text=True,timeout=30)
    assert result.returncode == 0, result.stdout+result.stderr


def test_native_robot_front_display_is_original_topic_not_a_truth_overlay():
    for name in ('route','vision','topology','t-left'):
        root = ET.fromstring('<config>'+gui_config(name).split('?>',1)[1]+'</config>')
        topics = [p.findtext('topic') for p in root.findall("plugin[@filename='ImageDisplay']")]
        assert '/toponav/camera/front' in topics
        assert '/toponav/topology_display' in topics


def test_first_person_selector_and_four_quadrants_use_real_public_pixels():
    pytest.importorskip('PyQt5.QtWidgets')
    root = Path(__file__).resolve().parents[1]
    code = r'''
import tempfile
from pathlib import Path
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QImage,QColor
from tools.gazebo_scenarios_app import ScenarioWindow
app=QApplication([])
with tempfile.TemporaryDirectory() as directory:
 w=ScenarioWindow(directory,auto_open=False)
 w.folder=Path(directory)/'run';(w.folder/'public').mkdir(parents=True)
 image=QImage(256,256,QImage.Format_RGB888)
 for x,y,color in [(0,0,'red'),(128,0,'green'),(0,128,'blue'),(128,128,'yellow')]:
  for i in range(x,x+128):
   for j in range(y,y+128):image.setPixelColor(i,j,QColor(color))
 image.save(str(w.folder/'public/four.png'))
 w.show_camera_input(dict(rgb_path='four.png',captured_sim_s=12.345))
 assert w.view_tabs.currentIndex()==1
 for key,color in [('front','red'),('left','green'),('right','blue'),('back','yellow')]:
  assert w.horizontal_pixels[key].toImage().pixelColor(64,64)==QColor(color)
  w.fpv_selector.setCurrentIndex(w.fpv_selector.findData(key))
  assert w.fpv.pixmap().toImage().pixelColor(20,20)==QColor(color)
 assert '12.345' in w.camera_stamp.text() and '128' in w.camera_stamp.text()
 assert '仅展示' in w.camera_stamp.text()
 assert w.worker.processId()==0
 # No audit file, traversal or partial PNG may become an observer image.
 assert w.public_image('../audit/pose.png') is None
 (w.folder/'public/partial.png').write_bytes(b'\x89PNG')
 assert w.public_image('partial.png') is None
 before=w.fpv.pixmap().toImage()
 w.show_camera_input(dict(rgb_path='partial.png',captured_sim_s=15.))
 assert w.fpv.pixmap().toImage()==before
 w.close()
print('first-person tests pass')
'''
    env=dict(os.environ,QT_QPA_PLATFORM='offscreen',PYTHONDONTWRITEBYTECODE='1')
    result=subprocess.run([sys.executable,'-c',code],cwd=root,env=env,capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stdout+result.stderr
