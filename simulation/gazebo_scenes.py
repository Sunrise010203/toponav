"""Declared Gazebo fixtures and topology display; never a pose-based driver."""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from simulation.dijkstra import plan
from simulation.topology import validate_pair

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = {
    "route": "六节点全流程 · 视觉路标/引导线辅助导航（新增基线）",
    "vision": "六节点 · 原17类视觉模型导航（无路标/黄线，实验模式）",
    "topology": "六节点地图 · 场景与规划预览（不自动驾驶）",
    "left": "L 型走廊 · 左转 RGB 控制",
    "right": "L 型走廊 · 右转 RGB 控制",
    "t-left": "T 型路口 · 左转 RGB 控制",
    "t-right": "T 型路口 · 右转 RGB 控制",
    "straight": "直走廊 · 原直走控制器交叉检查",
    "blocked": "阻挡走廊 · 原直走控制器拒绝/停车检查",
}


def local_pair(name):
    """Topology describes this fixture, not a claim of visual node arrival."""
    if name not in SCENARIOS or name in ("topology", "route", "vision"):
        raise ValueError("Unknown local scenario")
    if name in ("straight", "blocked"):
        positions = {"S": [.5, 0], "G": [4, 0]}
        pairs = [("S", "G")]
    else:
        sign = -1 if name.endswith("right") else 1
        positions = {"S": [.5, 0], "J": [2.5, 0], "G": [2.5, sign * 3.5]}
        pairs = [("S", "J"), ("J", "G")]
        if name.startswith("t-"):
            positions["B"] = [2.5, -sign * 3.5]
            pairs.append(("J", "B"))
    nodes = {n: dict(node_id=n, node_type="entrance" if n == "S" else "dead_end",
                    neighbors=[], exits=[]) for n in positions}
    channels, edges, dimensions = [], [], {}
    for index, (a, b) in enumerate(pairs, 1):
        channel = f"C{index:02d}"
        dx, dy = np.subtract(positions[b], positions[a])
        direction = "east" if dx > 0 else "west" if dx < 0 else "north" if dy > 0 else "south"
        reverse = dict(east="west", west="east", north="south", south="north")[direction]
        for source, target, d in ((a, b, direction), (b, a, reverse)):
            nodes[source]["neighbors"].append(target)
            nodes[source]["exits"].append(dict(neighbor_id=target, channel_id=channel, direction=d))
        edges.append(dict(channel_id=channel, source=a, target=b, bidirectional=True,
                          enabled=name != "blocked"))
        channels.append(dict(channel_id=channel, source=a, target=b, bidirectional=True,
                             source_direction=direction, target_direction=reverse))
        dimensions[channel] = dict(length=float(np.hypot(dx,dy)), width=1.8)
    if "J" in nodes:
        nodes["J"]["node_type"] = "intersection" if name.startswith("t-") else "corner"
    graph = dict(schema_version="2.0", scene_id="gazebo_" + name, directed=False,
                 exit_direction_frame="map_xy_east_north", nodes=list(nodes.values()), edges=edges,
                 task=dict(start_node="S", goal_node="G", initial_heading_deg=0))
    scene = dict(schema_version="2.0", scene_id=graph["scene_id"],
                 exit_direction_frame=graph["exit_direction_frame"], nodes=[dict(node_id=n) for n in nodes],
                 channels=channels, geometry=dict(units="metre", node_positions_m=positions,
                                                   channel_dimensions_m=dimensions))
    return graph, scene


def scenario_pair(name, disabled=(), goal=None):
    if name not in SCENARIOS:
        raise ValueError("Unknown Gazebo scenario")
    if name in ("topology", "route", "vision"):
        graph = json.loads((ROOT / "configs/graph.json").read_text())
        scene = json.loads((ROOT / "configs/scene.json").read_text())
    else:
        graph, scene = local_pair(name)
    if name not in ("topology", "route", "vision") and (disabled or goal is not None):
        raise ValueError("Task / channel overrides are only for the six-node preview")
    ids = {e["channel_id"] for e in graph["edges"]}
    if set(disabled) - ids:
        raise ValueError("Unknown disabled channel")
    if goal is not None:
        graph["task"]["goal_node"] = goal
    for edge in graph["edges"]:
        if edge["channel_id"] in disabled:
            edge["enabled"] = False
    validate_pair(graph, scene)
    return graph, scene


def road_rectangles(scene):
    positions = scene["geometry"]["node_positions_m"]
    dimensions = scene["geometry"]["channel_dimensions_m"]
    rects = []
    for channel in scene["channels"]:
        a, b = positions[channel["source"]], positions[channel["target"]]
        half = dimensions[channel["channel_id"]]["width"] / 2
        if a[0] != b[0] and a[1] != b[1]:
            raise ValueError("Gazebo fixture builder only supports straight orthogonal channels")
        if a[1] == b[1]:
            rects.append((min(a[0],b[0])-half, max(a[0],b[0])+half, a[1]-half, a[1]+half))
        else:
            rects.append((a[0]-half, a[0]+half, min(a[1],b[1])-half, max(a[1],b[1])+half))
    return rects


def union_cells(rectangles):
    """Non-overlapping floor cells and exposed boundary only (no junction walls)."""
    xs = sorted({r[i] for r in rectangles for i in (0,1)})
    ys = sorted({r[i] for r in rectangles for i in (2,3)})
    cells = {(i,j) for i in range(len(xs)-1) for j in range(len(ys)-1)
             if any(a < (xs[i]+xs[i+1])/2 < b and c < (ys[j]+ys[j+1])/2 < d
                    for a,b,c,d in rectangles)}
    return xs, ys, cells


def add_roads(world, rectangles):
    from simulation.gazebo_cross_validation import box
    xs, ys, cells = union_cells(rectangles)
    for i,j in sorted(cells):
        x, y = (xs[i]+xs[i+1])/2, (ys[j]+ys[j+1])/2
        dx, dy = xs[i+1]-xs[i], ys[j+1]-ys[j]
        box(world, f"floor_{i}_{j}", (dx,dy,.1), (x,y,-.05), (.20,.29,.40))
        for ni,nj,px,py,sx,sy in ((i-1,j,xs[i]-.05,y,.1,dy),
                (i+1,j,xs[i+1]+.05,y,.1,dy), (i,j-1,x,ys[j]-.05,dx,.1),
                (i,j+1,x,ys[j+1]+.05,dx,.1)):
            if (ni,nj) not in cells:
                box(world, f"wall_{i}_{j}_{ni}_{nj}", (sx,sy,1.6), (px,py,.8), (.65,.62,.58))


def build_scenario_world(name, graph=None, scene=None, hidden_markers=()):
    from simulation.gazebo_cross_validation import build_world, box
    if name not in SCENARIOS:
        raise ValueError("Unknown Gazebo scenario")
    if hidden_markers and name != 'route':
        raise ValueError('Marker faults only apply to landmark route mode')
    if name in ("left", "right"):
        return build_world(name)  # Preserve the previously tested L fixtures.
    if graph is None or scene is None:
        graph, scene = scenario_pair(name)
    tree = ET.fromstring(build_world("left"))
    world = tree.find("world")
    for model in list(world.findall("model")):
        if model.get("name") not in ("robot", "observer"):
            world.remove(model)
    if name.startswith("t-"):
        rectangles = [(-.2,3.4,-.9,.9), (1.6,3.4,-3.5,3.5)]
    elif name in ("straight", "blocked"):
        rectangles = [(-.2,4.,-.9,.9)]
    else:
        rectangles = road_rectangles(scene)
        start = scene["geometry"]["node_positions_m"][graph["task"]["start_node"]]
        world.find("model[@name='robot']/pose").text = f"{start[0]} {start[1]} .16 0 0 0"
        positions = scene["geometry"]["node_positions_m"]
        dims = scene["geometry"]["channel_dimensions_m"]
        for edge in graph["edges"]:
            if edge["enabled"]:
                continue
            a,b = positions[edge["source"]],positions[edge["target"]]
            width = dims[edge["channel_id"]]["width"]
            size = (.12,width,1.6) if a[1] == b[1] else (width,.12,1.6)
            box(world, "wall_blocker_"+edge["channel_id"], size,
                ((a[0]+b[0])/2,(a[1]+b[1])/2,.8), (.75,.27,.18))
    add_roads(world, rectangles)
    if name == 'vision':
        from simulation.gazebo_vision_fixture import augment
        augment(tree, rectangles)
    if name == "route":
        from simulation.gazebo_route_fixture import augment
        if set(hidden_markers)-{n['node_id'] for n in graph['nodes']}:
            raise ValueError('Unknown hidden marker node')
        augment(tree, graph, scene, hidden_markers)
    if name == "blocked":
        box(world, "wall_blocker_C01", (.15,1.8,1.6), (2.6,0,.8), (.65,.62,.58))
    bounds = (min(r[0] for r in rectangles),max(r[1] for r in rectangles),
              min(r[2] for r in rectangles),max(r[3] for r in rectangles))
    cx,cy = (bounds[0]+bounds[1])/2,(bounds[2]+bounds[3])/2
    height = max(7,1.5*max(bounds[1]-bounds[0],bounds[3]-bounds[2]))
    world.find("model[@name='observer']/link/sensor/pose").text = f"{cx} {cy} {height} 0 {math.pi/2} {math.pi/2}"
    ET.indent(tree)
    return ET.tostring(tree, encoding="unicode")


def make_controller(name, confirmed, graph=None, *, motion_permission='no_motion'):
    from roadvision.continuous_straight import StraightController, CornerController, InitialClearancePrior
    if motion_permission not in ('no_motion','bounded_probe'):
        raise ValueError('Invalid straight motion permission')
    if name not in ('straight','blocked') and motion_permission!='no_motion':
        raise ValueError('Bounded probe permission is only for straight/blocked scenarios')
    prior = InitialClearancePrior(confirmed=confirmed)
    if name in ("straight", "blocked"):
        return StraightController(initial_clearance=prior,motion_permission=motion_permission)
    if name == 'route':
        from roadvision.landmark_route import LandmarkRouteController
        from simulation.gazebo_route_fixture import registry, MARKER_SIZE
        graph, _ = scenario_pair(name)
        return LandmarkRouteController(graph, registry(graph), MARKER_SIZE)
    if name == 'vision':
        from roadvision.model_route import ModelRouteController
        return ModelRouteController(graph if graph is not None else scenario_pair(name)[0])
    if name == "topology":
        return PreviewController()
    if name not in SCENARIOS:
        raise ValueError("Unknown Gazebo scenario")
    return CornerController(direction="right" if name.endswith("right") else "left",
                            junction="t" if name.startswith("t-") else "l", initial_clearance=prior)


class PreviewController:
    """Zero only. Deliberately does NOT implement global autonomous navigation."""
    state = "observing"

    def update(self, frame, now):
        return dict(state=self.state, phase="map_preview", reason="scene_and_planning_preview_no_autonomous_driver",
                    v_m_s=0., omega_rad_s=0., control_frame_id=frame["frame_id"],
                    captured_sim_s=frame["captured_sim_s"], decision_sim_s=now, motion_commands=0)

    def cancel(self, now):
        pass

    def result(self):
        return dict(result_type="scene_preview_only", state=self.state, motion_commands=0,
                    arrival_status="unverified", atomic_edge_compatible=False,
                    reason="global_Gazebo_navigation_not_connected")


def gui_config(name):
    text = (ROOT / "configs/gazebo_live_gui.config").read_text()
    text = text.replace("<width>1100</width>", "<width>1450</width>")
    if name == "topology":
        text = text.replace("1.5 -3 8 0 1.05 1.57079632679", "4 0 11 0 1.45 1.57079632679")
    if name in ('route','vision'):
        text = text.replace("1.5 -3 8 0 1.05 1.57079632679", "3 -5 8 0 0.90 1.57079632679")
    text += '''\n<plugin filename="ImageDisplay" name="Topology map">
  <gz-gui>
    <title>Topology / planned route (not navigation proof)</title>
    <property type="string" key="state">floating</property>
    <property type="double" key="x">940</property>
    <property type="double" key="y">60</property>
    <property type="double" key="width">490</property>
    <property type="double" key="height">380</property>
  </gz-gui>
  <topic>/toponav/topology_display</topic>
  <topic_picker>false</topic_picker>
</plugin>\n'''
    if name in ('route','vision'):
        text += '''
<plugin filename="ImageDisplay" name="Live physics overhead">
  <gz-gui>
    <title>Live Gazebo camera / observer cutaway (ceiling tags hidden here only)</title>
    <property type="string" key="state">floating</property>
    <property type="double" key="x">10</property>
    <property type="double" key="y">60</property>
    <property type="double" key="width">920</property>
    <property type="double" key="height">650</property>
  </gz-gui>
  <topic>/toponav/overview</topic>
  <topic_picker>false</topic_picker>
</plugin>
'''
    text += '''
<plugin filename="ImageDisplay" name="Robot first person camera">
  <gz-gui>
    <title>Robot FRONT / native camera / display only</title>
    <property type="string" key="state">floating</property>
    <property type="double" key="x">940</property>
    <property type="double" key="y">450</property>
    <property type="double" key="width">490</property>
    <property type="double" key="height">330</property>
  </gz-gui>
  <topic>/toponav/camera/front</topic>
  <topic_picker>false</topic_picker>
</plugin>
'''
    return text


def topology_image(graph, scene, pose=None, label=""):
    """Audit pose may be drawn here only. This image is not a camera input."""
    image = Image.new("RGB", (720,500), "#f8fafc")
    draw = ImageDraw.Draw(image)
    font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    font = ImageFont.truetype(font_path,22)
    small = ImageFont.truetype(font_path,19)
    positions = scene["geometry"]["node_positions_m"]
    points = list(positions.values())
    xmin,xmax = min(p[0] for p in points),max(p[0] for p in points)
    ymin,ymax = min(p[1] for p in points),max(p[1] for p in points)
    scale = min(570/max(xmax-xmin,1),240/max(ymax-ymin,1))
    def xy(point):
        return (360+(point[0]-(xmin+xmax)/2)*scale,205-(point[1]-(ymin+ymax)/2)*scale)
    route = plan(graph)
    draw.text((20,12), "拓扑图 · "+graph["scene_id"], font=font, fill="#0f172a")
    for edge in graph["edges"]:
        a,b = xy(positions[edge["source"]]),xy(positions[edge["target"]])
        colour = "#dc2626" if not edge["enabled"] else "#0284c7" if edge["channel_id"] in route["channels"] else "#94a3b8"
        draw.line([a,b], fill=colour, width=7)
        mid = ((a[0]+b[0])/2,(a[1]+b[1])/2)
        if not edge["enabled"]:
            draw.line([(mid[0]-9,mid[1]-9),(mid[0]+9,mid[1]+9)],fill="#dc2626",width=4)
            draw.line([(mid[0]-9,mid[1]+9),(mid[0]+9,mid[1]-9)],fill="#dc2626",width=4)
        draw.text((mid[0]+7,mid[1]+9), edge["channel_id"], font=small, fill="#334155")
    for node,p in positions.items():
        x,y = xy(p)
        colour = "#059669" if node == graph["task"]["start_node"] else "#7c3aed" if node == graph["task"]["goal_node"] else "#334155"
        draw.ellipse((x-10,y-10,x+10,y+10), fill=colour)
        draw.text((x-19,y-38),node, font=font,fill="#0f172a")
    if pose is not None:
        x,y = xy(pose["position_m"])
        angle = math.radians(pose["yaw_deg"])
        vertices = [(x+17*math.cos(angle),y-17*math.sin(angle)),
                    (x+12*math.cos(angle+2.5),y-12*math.sin(angle+2.5)),
                    (x+12*math.cos(angle-2.5),y-12*math.sin(angle-2.5))]
        draw.polygon(vertices,fill="#ef4444",outline="#991b1b")
    path = " → ".join(route["nodes"]) if route["status"] == "reachable" else "不可达（已禁用通道）"
    draw.text((20,365), "规划："+path, font=font,fill="#0f172a")
    draw.text((20,405),label[:45], font=small, fill="#334155")
    draw.text((20,443),"蓝=计划路径  红箭头=审计真值，仅展示",font=small,fill="#475569")
    draw.text((20,472),"节点确认见实时状态；红箭头不是驾驶输入",font=small,fill="#475569")
    return np.asarray(image).copy()


def write_display(output, graph, scene, io, label):
    """One-way observer output. Never pass its pixels or poses to a driver."""
    pose = copy.deepcopy(io.poses[-1]) if io.poses else None
    image = topology_image(graph, scene, pose, label)
    io.publish_display(image)
    Image.fromarray(image).save(output / "public/topology.png")
    status = dict(pose=pose, label=label, scope="audit visualization only; not localization")
    temporary = output / "audit/display.tmp"
    temporary.write_text(json.dumps(status,ensure_ascii=False))
    temporary.replace(output / "audit/display.json")
