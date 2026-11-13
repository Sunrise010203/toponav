"""Replay saved public RGB / original timestamps; never open pose/contact audit."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image

def replay(folder, direction, confirmed=False, scenario=None, *, motion_permission='no_motion'):
    if motion_permission not in ('no_motion','bounded_probe'):
        raise ValueError('Invalid straight motion permission')
    if scenario not in ('straight','blocked') and motion_permission!='no_motion':
        raise ValueError('Bounded probe permission is only for straight/blocked scenarios')
    folder = Path(folder).resolve()
    root = Path(__file__).resolve().parents[1]
    controller_file = root / "roadvision/continuous_straight.py"
    controller_hash = hashlib.sha256(controller_file.read_bytes()).hexdigest()
    frozen = json.loads((folder / "freeze.json").read_text())
    if 'public/motion-task.json' in frozen:
        task_path=folder/'public/motion-task.json'
        if hashlib.sha256(task_path.read_bytes()).hexdigest()!=frozen['public/motion-task.json']:
            raise ValueError('Public straight task hash differs from capture')
        expected=dict(scenario=scenario,motion_permission=motion_permission,initial_clearance_confirmed=confirmed)
        if json.loads(task_path.read_text())!=expected:
            raise ValueError('Public straight task differs from replay authorization')
    if frozen["roadvision/continuous_straight.py"] != controller_hash:
        raise ValueError("Controller hash differs from capture; replay refused")
    factory = "simulation/gazebo_scenes.py"
    if factory in frozen and hashlib.sha256((root/factory).read_bytes()).hexdigest() != frozen[factory]:
        raise ValueError("Scenario/controller factory hash differs from capture; replay refused")
    from simulation.gazebo_scenes import make_controller
    if scenario == 'route':
        from roadvision.landmark_route import LandmarkRouteController
        for name in ('roadvision/landmark_route.py', 'simulation/gazebo_route_fixture.py'):
            if hashlib.sha256((root/name).read_bytes()).hexdigest() != frozen[name]:
                raise ValueError('Landmark controller/fixture differs from capture')
        graph = json.loads((folder/'public/topology.json').read_text())
        graph_hash = frozen.get('public/topology.json')
        if graph_hash is None and folder.is_relative_to(root):
            graph_hash = frozen.get(str((folder/'public/topology.json').relative_to(root)))
        if hashlib.sha256((folder/'public/topology.json').read_bytes()).hexdigest() != graph_hash:
            raise ValueError('Public graph differs from capture')
        markers = json.loads((folder/'public/markers.json').read_text())
        if hashlib.sha256((folder/'public/markers.json').read_bytes()).hexdigest() != frozen['public/markers.json']:
            raise ValueError('Marker registry differs from capture')
        controller = LandmarkRouteController(graph, markers['labels'], markers['size_m'])
    elif scenario == 'vision':
        from roadvision.model_route import ModelRouteController
        for name in ('roadvision/model_route.py','roadvision/visual_corridor_geometry.py',
                     'roadvision/learn.py','roadvision/contract.py','simulation/gazebo_vision_fixture.py',
                     'simulation/structural_localization.py','simulation/path_planning.py',
                     'simulation/topology.py','simulation/dijkstra.py','roadvision/data.py',
                     'models/road_structure_frozen_v1_20261014/model.pt',
                     'models/road_structure_frozen_v1_20261014/metadata.json'):
            if hashlib.sha256((root/name).read_bytes()).hexdigest() != frozen.get(name):
                raise ValueError('Frozen model/controller dependency differs from capture: '+name)
        graph_path=folder/'public/topology.json'
        if hashlib.sha256(graph_path.read_bytes()).hexdigest()!=frozen.get('public/topology.json'):
            raise ValueError('Public graph differs from capture')
        controller=ModelRouteController(json.loads(graph_path.read_text()))
    else:
        controller = make_controller(scenario or direction, confirmed,motion_permission=motion_permission)
    count = 0
    mismatches = []
    for line in (folder / "decisions.jsonl").read_text().splitlines():
        row = json.loads(line)
        inp = row["input"]
        path = (folder / "public" / inp["rgb_path"]).resolve()
        if not path.is_relative_to(folder / "public/frames"):
            raise ValueError("Only public RGB frame paths may be replayed")
        with Image.open(path) as image:
            rgb = np.array(image.convert("RGB"))
        if hashlib.sha256(rgb.tobytes()).hexdigest() != inp["rgb_sha256"]:
            raise ValueError(f"RGB hash mismatch: {path}")
        frame = dict(frame_id=inp["frame_id"], captured_sim_s=inp["captured_sim_s"], rgb=rgb)
        if scenario == 'route':
            for view in ('up','down'):
                aux = inp['auxiliary'][view]
                path = (folder/'public'/aux['rgb_path']).resolve()
                if not path.is_relative_to(folder/'public'/view) or aux['captured_sim_s'] != inp['captured_sim_s']:
                    raise ValueError('Auxiliary image path/time mismatch')
                pixels = np.array(Image.open(path).convert('RGB'))
                if hashlib.sha256(pixels.tobytes()).hexdigest() != aux['rgb_sha256']:
                    raise ValueError('Auxiliary RGB hash mismatch')
                frame[view+'_rgb'] = pixels
        actual = controller.update(frame, row["sim_s"])
        # Normalize tuples / NumPy scalars exactly as the capture JSON writer.
        expected=json.loads(json.dumps(row['decision']))
        if scenario=='vision':
            # Recompute real inference. Only elapsed wall-clock profiling is
            # excluded; probabilities, timestamps and ALL control fields match.
            for decision in (actual,expected):
                if decision.get('model') is not None:
                    decision['model'].pop('inference_ms',None)
        if json.loads(json.dumps(actual, allow_nan=False)) != expected:
            mismatches.append(row["tick"])
        count += 1
    return dict(source=str(folder), controller_sha256=controller_hash,
                direction=getattr(controller,"direction",None), initial_clearance_confirmed=confirmed,
                motion_permission=motion_permission,
                scenario=scenario or direction,
                frames=count, mismatching_ticks=mismatches,
                all_decisions_match=count > 0 and not mismatches,
                excluded_diagnostic_fields=['model.inference_ms'] if scenario=='vision' else [],
                controller_result=controller.result(),
                scope="public RGB and original timestamps only; no physics rerun or truth audit")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument("--direction", choices=("left", "right"), default="left")
    from simulation.gazebo_scenes import SCENARIOS
    parser.add_argument("--scenario", choices=SCENARIOS)
    parser.add_argument("--confirm-initial-clearance", action="store_true")
    parser.add_argument('--motion-permission',choices=('no_motion','bounded_probe'),default='no_motion')
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    result = replay(args.folder, args.direction, args.confirm_initial_clearance, args.scenario,
                    motion_permission=args.motion_permission)
    payload = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if args.output:
        with args.output.open("x") as stream:
            stream.write(payload)
    print(payload, end="")
    return 0 if result["all_decisions_match"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
