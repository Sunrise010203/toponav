"""Static suite preparation. This module never constructs a rendering World.

Layout graphs are screened against historical graph sources by exact unlabelled
isomorphism (stronger than merely coordinate/heading-normalized equality).
All new final-test geometry/tasks can be inspected without generating pixels.
"""
import argparse
from collections import Counter
from contextlib import contextmanager
import copy
import json
from pathlib import Path
import random

import networkx as nx
import yaml

from experiments.block7.common import CONFIG, ROOT, digest, file_hash, read_json, seed, write_json, write_rows
from roadvision.scene import LAYOUTS, topology_pair
from simulation.dijkstra import plan
from simulation.topology import validate_pair


@contextmanager
def registered(layout):
    """Single-worker, temporary registry extension; never replace old layouts."""
    identity = layout['layout_id']
    if identity in LAYOUTS:
        raise ValueError('Refusing to replace a registered historical layout')
    LAYOUTS[identity] = {'cells': [tuple(c) for c in layout['cells']],
                       'edges': [(tuple(a), tuple(b)) for a, b in layout['edges']],
                       'split': layout['split']}
    try:
        yield identity
    finally:
        del LAYOUTS[identity]


def pair(layout):
    with registered(layout) as identity:
        graph, scene, cells = topology_pair(identity)
    validate_pair(graph, scene)
    return graph, scene, cells


def nx_graph(graph):
    g = nx.Graph()
    g.add_nodes_from(n['node_id'] for n in graph['nodes'])
    g.add_edges_from((e['source'], e['target']) for e in graph['edges'])
    return g


def signature(g):
    return len(g), g.number_of_edges(), tuple(sorted(d for _, d in g.degree()))


def same_family(a, b):
    return signature(a) == signature(b) and nx.is_isomorphic(a, b)


def historical_graphs():
    """Explicit coverage receipt; malformed negative fixtures are not truth maps."""
    graphs, sources = [], []
    for identity in sorted(LAYOUTS):
        graph, _, _ = topology_pair(identity)
        graphs.append((identity, nx_graph(graph)))
    paths = set((ROOT / 'configs').rglob('*.json'))
    paths |= set((ROOT / 'reports').rglob('*suite*.json'))
    # Exclude newly prepared definitions, not pre-existing historical files.
    paths = {p for p in paths if 'experiments' not in p.relative_to(ROOT).parts
             and not p.is_relative_to(ROOT / 'reports/block7')}

    def walk(value, label):
        if isinstance(value, dict):
            if (isinstance(value.get('nodes'), list) and value.get('nodes') and
                    isinstance(value.get('edges'), list) and
                    all(isinstance(n, dict) and 'node_id' in n for n in value['nodes']) and
                    all(isinstance(e, dict) and {'source', 'target'} <= e.keys() for e in value['edges'])):
                graphs.append((label, nx_graph(value)))
            for key, child in value.items():
                walk(child, label + '/' + key)
        elif isinstance(value, list):
            for index, child in enumerate(value):
                walk(child, label + '/' + str(index))

    for path in sorted(paths):
        # Invalid examples are deliberately not actual historical worlds.
        if 'invalid_cases' in path.parts or path.name.endswith('.schema.json'):
            continue
        data = read_json(path)
        sources.append({'path': str(path), 'sha256': file_hash(path)})
        walk(data, str(path))
    return graphs, sources


def candidate_layout(identity, split, band, attempt_seed):
    rng = random.Random(attempt_seed)
    count = rng.randint(*band)
    cells = {(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)}
    while len(cells) < count:
        x, y = rng.choice(sorted(cells))
        dx, dy = rng.choice([(1, 0), (-1, 0), (0, 1), (0, -1)])
        cells.add((x + dx, y + dy))
    # Shift only, never scale. Renderer uses the established six-metre lattice.
    xmin, ymin = min(x for x, _ in cells), min(y for _, y in cells)
    origin = (-xmin, -ymin)
    cells = sorted((x - xmin, y - ymin) for x, y in cells)
    edges = sorted((a, b) for a in cells for b in cells
                   if (b[0] - a[0], b[1] - a[1]) in ((1, 0), (0, 1)))
    g = nx.Graph(); g.add_nodes_from(cells); g.add_edges_from(edges)
    # Introduce graph diversity without removing the central degree-four hub:
    # every known local exit mask can be rendered at that physical hub.
    optional = list(edges); rng.shuffle(optional)
    for a, b in optional:
        if origin not in (a, b) and rng.random() < .35:
            g.remove_edge(a, b)
            if not nx.is_connected(g):
                g.add_edge(a, b)
    return {'layout_id': identity, 'family_id': identity, 'split': split, 'node_band': list(band),
            'cells': [list(c) for c in cells],
            'edges': [[list(a), list(b)] for a, b in sorted(tuple(sorted(e)) for e in g.edges())],
            'generation_seed': attempt_seed, 'pixel_generation': 'not_generated'}


def remove_public_edge(graph, channel):
    """Wrong-map experiment: omit a redundant physical edge, preserving validity."""
    result = copy.deepcopy(graph)
    result['edges'] = [e for e in result['edges'] if e['channel_id'] != channel]
    for node in result['nodes']:
        node['exits'] = [e for e in node['exits'] if e['channel_id'] != channel]
        node['neighbors'] = [e['neighbor_id'] for e in node['exits']]
        directions = {e['direction'] for e in node['exits']}
        node['node_type'] = ('intersection' if len(directions) >= 3 else
                             'dead_end' if len(directions) <= 1 else
                             'corridor' if directions in ({'east', 'west'}, {'north', 'south'}) else 'corner')
    # Wrong edge must be redundant; physical world is validated separately.
    from simulation.topology import validate_graph
    validate_graph(result)
    return result


def tasks_for_layout(layout, graph, budget):
    g = nx_graph(graph)
    nodes = sorted(g)
    candidates = [(a, b) for a in nodes for b in nodes
                  if a != b and nx.shortest_path_length(g, a, b) >= 2]
    rng = random.Random(seed('task-order', layout['family_id']))
    order_rng = random.Random(seed(budget['seeds']['method_order_seed'], layout['family_id']))
    rng.shuffle(candidates)
    chosen = candidates[:budget['navigation']['distinct_tasks_per_family']]
    if len(chosen) != 5:
        raise ValueError('Insufficient distinct multi-hop tasks')
    redundant = [e['channel_id'] for e in graph['edges']
                 if not nx.is_bridge(g, (e['source'], e['target']))] if hasattr(nx, 'is_bridge') else []
    if not redundant:
        bridges = {frozenset(e) for e in nx.bridges(g)}
        redundant = [e['channel_id'] for e in graph['edges']
                     if frozenset((e['source'], e['target'])) not in bridges]
    if not redundant:
        raise ValueError('Wrong-map condition requires at least one redundant physical edge')
    rows = []
    split = layout['split']
    seeds = budget['seeds'][split + '_environment']
    for index, (start, goal) in enumerate(chosen):
        for environment_seed in seeds:
            common = {'family_id': layout['family_id'], 'layout_id': layout['layout_id'], 'split': split,
                      'start': start, 'goal': goal, 'heading': (index % 4) * 90,
                      'environment_seed': environment_seed, 'method_ids': ['M0', 'M1', 'M2', 'M3'],
                      'physical_disabled_channels': [], 'public_disabled_channels': [],
                      'wrong_map_channel': None, 'physical_reachable': True}
            route = plan(graph, start, goal)
            factors = ['normal'] + (budget['navigation']['single_factor_conditions'] if index < 2 else [])
            for factor in factors:
                row = copy.deepcopy(common)
                row['factor'] = factor
                row['appearance_variant'] = ('material_albedo' if index == 0 else 'lighting_exposure') if factor == 'appearance_material_or_lighting' else 'nominal'
                row['fault'] = {'anchor': 'first_scheduled_capture', 'channel': route['channels'][0],
                                'closure_capture_count': 3}
                if factor in {'trusted_atomic_execution_failed_once', 'execution_unknown'}:
                    row['fault']['anchor'] = 'first_motion_command'
                if factor == 'wrong_map_edge':
                    # Prefer an incident edge to expose the wrong prior immediately.
                    incident = [e['channel_id'] for e in graph['edges'] if start in (e['source'], e['target'])]
                    row['wrong_map_channel'] = next((c for c in incident if c in redundant), redundant[0])
                    remove_public_edge(graph, row['wrong_map_channel'])
                row['task_id'] = f"{layout['layout_id']}-{index}-{environment_seed}-{factor}"
                row['method_order'] = order_rng.sample(row['method_ids'], len(row['method_ids']))
                row['seeds'] = {stream: seed('N', split, row['family_id'], index, environment_seed, stream)
                                for stream in ('appearance', 'sensor_faults', 'execution_faults')}
                rows.append(row)
    start, goal = chosen[0]
    blocked = [e['channel_id'] for e in graph['edges'] if goal in (e['source'], e['target'])]
    for environment_seed in seeds:
        row = {'family_id': layout['family_id'], 'layout_id': layout['layout_id'], 'split': split,
               'start': start, 'goal': goal, 'heading': 0, 'environment_seed': environment_seed,
               'factor': 'unreachable', 'fault': None, 'wrong_map_channel': None,
               'physical_disabled_channels': blocked, 'public_disabled_channels': blocked,
               'physical_reachable': False, 'method_ids': ['M0', 'M1', 'M2', 'M3'],
               'method_order': order_rng.sample(['M0', 'M1', 'M2', 'M3'], 4),
               'seeds': {stream: seed('N', split, layout['family_id'], 'unreachable', environment_seed, stream)
                         for stream in ('appearance', 'sensor_faults', 'execution_faults')},
               'task_id': f"{layout['layout_id']}-{environment_seed}-unreachable"}
        if plan(graph, start, goal, blocked)['status'] != 'unreachable':
            raise ValueError('Unreachable condition was not physically disconnected')
        rows.append(row)
    return rows


def prepare(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    budget = yaml.safe_load((CONFIG / 'budget.yaml').read_text())
    historical, sources = historical_graphs()
    accepted = []
    layouts, rejection_counts = [], Counter()
    for split in ('validation', 'test'):
        per_band = budget['layout_design'][split + '_families_per_band']
        for band_index, band in enumerate(budget['layout_design']['node_count_bands']):
            for number in range(per_band):
                identity = f'b7-{split}-b{band_index}-{number}'
                for attempt in range(3000):
                    generation_seed = seed(budget['seeds'][split + '_layout_generation_base'], identity, attempt)
                    candidate = candidate_layout(identity, split, band, generation_seed)
                    graph, scene, _ = pair(candidate)
                    g = nx_graph(graph)
                    if nx.is_tree(g):
                        rejection_counts['tree_without_redundant_wrong_map_edge'] += 1
                        continue
                    if any(same_family(g, prior) for _, prior in historical + accepted):
                        rejection_counts['isomorphic_to_historical_or_selected'] += 1
                        continue
                    accepted.append((identity, g)); layouts.append((candidate, graph, scene))
                    break
                else:
                    write_json(output / 'generation-failed.json', {'identity': identity, 'rejections': rejection_counts})
                    raise ValueError('No eligible distinct topology within predeclared generation cap')
    (output / 'layouts').mkdir()
    tasks, samples = [], []
    for layout, graph, scene in layouts:
        identity = layout['layout_id']
        write_json(output / 'layouts' / (identity + '.json'), {'layout': layout, 'graph': graph, 'scene': scene})
        tasks += tasks_for_layout(layout, graph, budget)
        split = layout['split']
        p = budget['perception']
        for label in range(17):
            phases = p['unknown_phases'] if label == 16 else ['stable']
            for phase in phases:
                count = p[split + ('_packages_per_unknown_phase_per_family' if label == 16
                                   else '_packages_per_known_class_per_family')]
                for index in range(count):
                    identity_sample = f'{identity}-{label:02}-{phase}-{index}'
                    samples.append({'sample_id': identity_sample, 'family_id': identity, 'layout_id': identity,
                                    'split': split, 'class_index': label, 'phase': phase,
                                    'appearance_variant': ['nominal', 'material_albedo', 'lighting_exposure'][index % 3],
                                    'seed': seed('P', split, identity, label, phase, index,
                                                 budget['seeds'][split + '_layout_generation_base']),
                                    'image_status': 'not_generated'})
    write_rows(output / 'tasks.jsonl', tasks)
    write_rows(output / 'perception_samples.jsonl', samples)
    write_json(output / 'splits.json', {split: [l['layout_id'] for l, _, _ in layouts if l['split'] == split]
                                       for split in ('validation', 'test')})
    write_json(output / 'seeds.json', {'rule': budget['seeds'], 'all_exact_values': ['tasks.jsonl', 'perception_samples.jsonl']})
    with (output / 'factors.yaml').open('x') as file:
        yaml.safe_dump({'factors': budget['navigation']['single_factor_conditions'],
                        'temporary_closure': 'first planned edge physically closed for first 3 scheduled captures, then opens',
                        'wrong_map_edge': 'redundant physical edge absent from public graph, physical world unchanged',
                        'appearance': 'task 0 changes diffuse wall/floor colour albedo, task 1 changes exposure x0.70; never simultaneous; not realistic material/light certification',
                        'execution_unknown': 'first atomic move executes but its public status is unknown, never guessed complete'}, file)
    write_json(output / 'split_audit.json', {'passed': True, 'new_families': len(layouts),
               'exact_isomorphism_comparisons': True, 'historical_graphs_checked': len(historical),
               'historical_sources': sources, 'historical_source_scope': 'four image LAYOUTS plus valid graph objects in configs and historical *suite*.json; not an exhaustive parse of every archived report',
               'new_vs_historical_isomorphic': 0, 'cross_split_isomorphic': 0,
               'rejected_candidates': dict(rejection_counts), 'pixels_generated': 0,
               'criterion': 'exact unlabelled graph nonisomorphism, stronger than rotation/mirror/renaming rejection'})
    files = [p for p in output.rglob('*') if p.is_file()]
    write_json(output / 'dataset_manifest.json', {'schema_version': 'block7-suite-v1',
               'preparation': 'static_only_no_pixels_or_predictions', 'budget_sha256': file_hash(CONFIG / 'budget.yaml'),
               'counts': {'families': len(layouts), 'base_navigation_tasks': len(tasks),
                          'method_episodes': sum(len(t['method_ids']) for t in tasks), 'perception_packages': len(samples)},
               'files': [{'path': str(p.relative_to(output)), 'sha256': file_hash(p)} for p in sorted(files)]})
    return read_json(output / 'dataset_manifest.json')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = prepare(args.output)
    print(json.dumps(result['counts']))


if __name__ == '__main__':
    main()
