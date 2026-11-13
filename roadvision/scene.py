"""Physical corridor worlds rendered with four synchronized PyBullet cameras."""
import hashlib
import json
import math
from collections import defaultdict

import networkx as nx
import numpy as np


def _layout(width, height, split, omitted=()):
    cells = [(x, y) for x in range(width) for y in range(height)]
    omitted = {frozenset(edge) for edge in omitted}
    edges = [(a, b) for a in cells for b in cells if (b[0]-a[0], b[1]-a[1]) in ((1, 0), (0, 1))
             and frozenset((a, b)) not in omitted]
    return {"cells": cells, "edges": edges, "split": split}


LAYOUTS = {
    "train_grid": _layout(3, 3, "train"),
    "train_bypass": _layout(4, 3, "train", [((1, 0), (2, 0)), ((2, 1), (3, 1))]),
    "val_branch": _layout(4, 4, "val", [((2, 0), (3, 0)), ((3, 2), (3, 3)), ((0, 2), (0, 3))]),
    "test_locked": _layout(5, 3, "test", [((3, 0), (4, 0)), ((3, 1), (4, 1)), ((2, 1), (2, 2))]),
}


def layout_fingerprint(layout):
    graph = nx.Graph()
    graph.add_nodes_from(layout["cells"])
    graph.add_edges_from(layout["edges"])
    # Canonical-ish topology fingerprint: node names/coordinates are not hashed.
    payload = (len(graph), graph.number_of_edges(), nx.weisfeiler_lehman_graph_hash(graph))
    return hashlib.sha256(json.dumps(payload).encode()).hexdigest()


def topology_pair(layout_id):
    """Export the physical layout as v2 map and separate simulator scene."""
    layout = LAYOUTS[layout_id]
    names = {cell: f"R{index:03d}" for index, cell in enumerate(layout["cells"])}
    exits = {cell: [] for cell in names}
    edges, channels = [], []
    cardinal = {(1, 0): "east", (0, 1): "north", (-1, 0): "west", (0, -1): "south"}
    for index, (a, b) in enumerate(layout["edges"]):
        cid = f"P{index:03d}"
        edge = {"channel_id": cid, "source": names[a], "target": names[b], "bidirectional": True}
        edges.append({**edge, "enabled": True})
        da, db = cardinal[b[0]-a[0], b[1]-a[1]], cardinal[a[0]-b[0], a[1]-b[1]]
        channels.append({**edge, "source_direction": da, "target_direction": db})
        exits[a].append({"neighbor_id": names[b], "channel_id": cid, "direction": da})
        exits[b].append({"neighbor_id": names[a], "channel_id": cid, "direction": db})
    common = {"schema_version": "2.0", "scene_id": layout_id, "exit_direction_frame": "map_xy_east_north"}
    nodes = []
    for cell, node in names.items():
        directions = {x["direction"] for x in exits[cell]}
        kind = ("intersection" if len(directions) >= 3 else "dead_end" if len(directions) <= 1 else
                "corridor" if directions in ({"east", "west"}, {"north", "south"}) else "corner")
        nodes.append({"node_id": node, "node_type": kind,
                      "neighbors": [x["neighbor_id"] for x in exits[cell]], "exits": exits[cell]})
    graph = {**common, "directed": False, "nodes": nodes, "edges": edges}
    scene = {**common, "nodes": [{"node_id": node} for node in names.values()], "channels": channels,
             "geometry": {"units": "metre", "node_positions_m": {node: [6*cell[0], 6*cell[1]] for cell, node in names.items()},
                          "channel_dimensions_m": {edge["channel_id"]: {"length": 6, "width": 3} for edge in edges}}}
    return graph, scene, {node: cell for cell, node in names.items()}


class World:
    def __init__(self, layout_id):
        global p
        import pybullet as p
        self.layout_id, self.layout = layout_id, LAYOUTS[layout_id]
        self.client = p.connect(p.DIRECT)
        self.bodies, self.temporary = [], []
        self.neighbors = defaultdict(set)
        for a, b in self.layout["edges"]:
            self.neighbors[a].add(b)
            self.neighbors[b].add(a)
        free = set()
        for x, y in self.layout["cells"]:
            free.update((6*x+dx, 6*y+dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1))
        for a, b in self.layout["edges"]:
            if a[0] == b[0]:
                free.update((6*a[0]+dx, yy) for dx in (-1, 0, 1)
                            for yy in range(6*min(a[1], b[1]), 6*max(a[1], b[1])+1))
            else:
                free.update((xx, 6*a[1]+dy) for dy in (-1, 0, 1)
                            for xx in range(6*min(a[0], b[0]), 6*max(a[0], b[0])+1))
        boundaries = defaultdict(set)
        for x, y in free:
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                if (x+dx, y+dy) not in free:
                    axis, line, begin = ("v", x+dx/2, y) if dx else ("h", y+dy/2, x)
                    boundaries[axis, line].add(begin)
        for (axis, line), starts in boundaries.items():
            starts = sorted(starts)
            begin = end = starts[0]
            for value in starts[1:] + [starts[-1]+2]:
                if value == end+1:
                    end = value
                    continue
                middle, length = (begin+end)/2, end-begin+1
                half = (0.06, length/2, 1.5) if axis == "v" else (length/2, 0.06, 1.5)
                center = (line, middle, 1.5) if axis == "v" else (middle, line, 1.5)
                self.bodies.append(self.box(half, center, (0.68, 0.65, 0.6)))
                begin = end = value
        xs, ys = zip(*free)
        center = ((min(xs)+max(xs))/2, (min(ys)+max(ys))/2)
        half = ((max(xs)-min(xs))/2+2, (max(ys)-min(ys))/2+2, 0.06)
        self.floor = self.box(half, (*center, -0.06), (0.28, 0.32, 0.36))
        self.ceiling = self.box(half, (*center, 3.06), (0.5, 0.5, 0.5))

    def box(self, half, center, color):
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=(*color, 1), physicsClientId=self.client)
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=self.client)
        return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual,
                                 basePosition=center, physicsClientId=self.client)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        p.disconnect(self.client)

    def capture(self, position, heading, exposure=1.0):
        images = []
        for offset in (0, 90, -90, 180):
            angle = math.radians(heading+offset)
            target = (position[0]+math.cos(angle), position[1]+math.sin(angle), position[2])
            view = p.computeViewMatrix(position, target, (0, 0, 1))
            projection = p.computeProjectionMatrixFOV(90, 1, 0.05, 60)
            result = p.getCameraImage(128, 128, view, projection, renderer=p.ER_TINY_RENDERER,
                                      lightDirection=(1, -1, 3), physicsClientId=self.client)
            rgb = np.asarray(result[2], dtype=np.uint8).reshape(128, 128, 4)[..., :3]
            images.append(np.clip(rgb.astype(np.float32)*exposure, 0, 255).astype(np.uint8))
        return np.concatenate((np.concatenate(images[:2], axis=1), np.concatenate(images[2:], axis=1)), axis=0)

    def sample(self, label, rng, unknown_mode=None, pose=None, blocked=None):
        for body in self.temporary:
            p.removeBody(body, physicsClientId=self.client)
        self.temporary = []
        mask = int(rng.integers(1, 16)) if label == 16 else label
        wanted = [bit == "1" for bit in f"{mask:04b}"]
        candidates = []
        for cell in self.layout["cells"]:
            for heading in (0, 90, 180, 270):
                vectors = [(round(math.cos(math.radians(heading+offset))),
                            round(math.sin(math.radians(heading+offset)))) for offset in (0, 90, -90, 180)]
                opened = [(cell[0]+dx, cell[1]+dy) in self.neighbors[cell] for dx, dy in vectors]
                if all(not desired or actual for desired, actual in zip(wanted, opened)):
                    candidates.append((cell, heading, vectors, opened))
        if pose is None:
            cell, heading, vectors, opened = candidates[int(rng.integers(len(candidates)))]
        else:
            cell, heading = pose
            vectors = [(round(math.cos(math.radians(heading+offset))), round(math.sin(math.radians(heading+offset))))
                       for offset in (0, 90, -90, 180)]
            opened = [(cell[0]+dx, cell[1]+dy) in self.neighbors[cell] for dx, dy in vectors]
            wanted = [actual and frozenset((cell, (cell[0]+dx, cell[1]+dy))) not in (blocked or set())
                      for actual, (dx, dy) in zip(opened, vectors)]
        wall_color = tuple(rng.uniform(0.45, 0.82, 3))
        for body in self.bodies:
            p.changeVisualShape(body, -1, rgbaColor=(*wall_color, 1), physicsClientId=self.client)
        p.changeVisualShape(self.floor, -1, rgbaColor=(*rng.uniform(0.2, 0.45, 3), 1), physicsClientId=self.client)
        for actual, desired, (dx, dy) in zip(opened, wanted, vectors):
            if actual and not desired:
                half = (0.07, 1.5, 1.5) if dx else (1.5, 0.07, 1.5)
                self.temporary.append(self.box(half, (6*cell[0]+1.65*dx, 6*cell[1]+1.65*dy, 1.5), wall_color))
        position = np.array((6*cell[0], 6*cell[1], 1.0), dtype=float)
        position[:2] += rng.uniform(-0.08, 0.08, 2)
        phase = "stable"
        if label == 16:
            phase = unknown_mode or rng.choice(("turning", "approaching", "passing", "occluded", "missing_view"))
            if phase == "turning":
                heading += float(rng.uniform(25, 65))
            elif phase in ("approaching", "passing"):
                vector = vectors[wanted.index(True)]
                position[:2] += np.array(vector)*(2.3 if phase == "approaching" else 1.15)
            elif phase == "occluded":
                dx, dy = vectors[int(rng.integers(4))]
                self.temporary.append(self.box((0.35, 0.35, 0.7), position+np.array((0.7*dx, 0.7*dy, 0)), (0.2, 0.15, 0.1)))
        ends = [(position[0]+dx*2.5, position[1]+dy*2.5, 1) for dx, dy in vectors]
        rays = p.rayTestBatch([position.tolist()]*4, ends, physicsClientId=self.client)
        ray_open = [hit[0] == -1 for hit in rays]
        if label != 16 and ray_open != wanted:
            raise ValueError("Physical ray/label disagreement")
        image = self.capture(position.tolist(), heading, float(rng.uniform(0.8, 1.15)))
        if phase == "missing_view":
            quadrant = int(rng.integers(4))
            row, column = divmod(quadrant, 2)
            image[row*128:(row+1)*128, column*128:(column+1)*128] = 0
        return image, {"cell": cell, "heading_deg": heading, "phase": phase, "ray_open": ray_open,
                       "camera_position": position.tolist(), "camera_fov_deg": 90}
