"""Suite/receipt validation, safe denominators and pre-test candidate binding."""
import argparse
from collections import Counter
import json
from pathlib import Path

import yaml

from experiments.block7.common import CONFIG, ROOT, file_hash, read_json, read_rows, write_json
from experiments.block7.dataset import historical_graphs, nx_graph, pair, same_family, remove_public_edge
from simulation.dijkstra import plan


def ratio(numerator, denominator):
    if type(numerator) is not int or type(denominator) is not int or not 0 <= numerator <= denominator:
        raise ValueError('Rate requires integer counts 0 <= numerator <= denominator')
    return numerator / denominator if denominator else None


def ledger_summary(ledger):
    rows = ledger['episodes']
    ids = [(row['task_id'], row['method']) for row in rows]
    if len(set(ids)) != len(ids):
        raise ValueError('Duplicate task/method in ledger')
    statuses = {'success', 'refused', 'timeout', 'error', 'not_run'}
    if any(row['status'] not in statuses for row in rows):
        raise ValueError('Unknown or unfinished ledger status')
    counts = Counter(row['status'] for row in rows)
    eligible = [r for r in rows if r.get('physical_reachable') is True]
    return {'planned': len(rows), 'counts': {s: counts[s] for s in sorted(statuses)},
            'normal_or_fault_reachable_success_rate': ratio(sum(r['status'] == 'success' for r in eligible), len(eligible)),
            'complete': not counts['not_run'], 'scope': 'bookkeeping_only_stratified_statistics_follow_in_7E'}


def verify_files(base, manifest):
    base = Path(base).resolve()
    errors = []
    for row in manifest['files']:
        path = (base / row['path']).resolve()
        if not path.is_relative_to(base):
            errors.append('path_escape:' + row['path'])
        elif not path.is_file() or file_hash(path) != row['sha256']:
            errors.append('hash_or_missing:' + row['path'])
    return errors


def validate_suite(suite):
    suite = Path(suite)
    manifest = read_json(suite / 'dataset_manifest.json')
    errors = verify_files(suite, manifest)
    if manifest['budget_sha256'] != file_hash(CONFIG / 'budget.yaml'):
        errors.append('7A_budget_changed')
    if errors:
        return {'passed': False, 'errors': errors}
    budget = yaml.safe_load((CONFIG / 'budget.yaml').read_text())
    from experiments.block7.session import ExperimentConfig
    definitions = yaml.safe_load((CONFIG / 'methods.yaml').read_text())
    for method in definitions['methods']:
        config = ExperimentConfig.for_method(method['id'])
        if (config.confirmation_frames, config.passive_recovery) != (method['confirmation_frames'], method['passive_recovery']):
            errors.append('method_config_source_drift:' + method['id'])
    split = read_json(suite / 'splits.json')
    all_layouts = []
    for name in ('validation', 'test'):
        if len(split[name]) != budget['layout_design'][name + '_families']:
            errors.append('split_count:' + name)
        for identity in split[name]:
            row = read_json(suite / 'layouts' / (identity + '.json'))
            if row['layout']['split'] != name or row['layout']['family_id'] != identity:
                errors.append('split_identity:' + identity)
            graph, scene, _ = pair(row['layout'])
            if graph != row['graph'] or scene != row['scene']:
                errors.append('layout_export_mismatch:' + identity)
            all_layouts.append((identity, name, nx_graph(graph), graph))
    for index, (identity, _, g, _) in enumerate(all_layouts):
        for prior, _, p, _ in all_layouts[:index]:
            if same_family(g, p):
                errors.append('isomorphic_selected:' + identity + ':' + prior)
    audit = read_json(suite / 'split_audit.json')
    historical, _ = historical_graphs()
    for identity, _, graph, _ in all_layouts:
        if any(same_family(graph, old) for _, old in historical):
            errors.append('isomorphic_to_historical:' + identity)
    for source in audit['historical_sources']:
        if not Path(source['path']).is_file() or file_hash(source['path']) != source['sha256']:
            errors.append('historical_source_changed:' + source['path'])
    tasks = read_rows(suite / 'tasks.jsonl')
    samples = read_rows(suite / 'perception_samples.jsonl')
    if len({t['task_id'] for t in tasks}) != len(tasks):
        errors.append('duplicate_task_id')
    if len({s['sample_id'] for s in samples}) != len(samples):
        errors.append('duplicate_sample_id')
    graph_by_id = {identity: graph for identity, _, _, graph in all_layouts}
    split_by_id = {identity: name for identity, name, _, _ in all_layouts}
    for task in tasks:
        graph = graph_by_id[task['layout_id']]
        if task['split'] != split_by_id[task['layout_id']]:
            errors.append('task_split_mismatch:' + task['task_id'])
        if set(task['method_ids']) != {'M0', 'M1', 'M2', 'M3'} or sorted(task['method_order']) != sorted(task['method_ids']):
            errors.append('method_schedule:' + task['task_id'])
        physical = plan(graph, task['start'], task['goal'], task['physical_disabled_channels'])
        if (physical['status'] == 'reachable') != task['physical_reachable']:
            errors.append('reachability:' + task['task_id'])
        if task['wrong_map_channel']:
            wrong = remove_public_edge(graph, task['wrong_map_channel'])
            if wrong == graph:
                errors.append('wrong_map_not_changed')
        if set(task['seeds']) != {'appearance', 'sensor_faults', 'execution_faults'}:
            errors.append('rng_streams:' + task['task_id'])
    for name in ('validation', 'test'):
        subset = [t for t in tasks if t['split'] == name]
        counts = Counter('normal' if t['factor'] == 'normal' else 'unreachable' if
                         t['factor'] == 'unreachable' else 'single_factor' for t in subset)
        expected = budget['navigation']['per_method'][name]
        if any(counts[k] != expected[k] for k in counts) or len(subset) != expected['total']:
            errors.append('navigation_counts:' + name)
        if sum(s['split'] == name for s in samples) != budget['perception'][name + '_packages']:
            errors.append('perception_counts:' + name)
    for sample in samples:
        if (sample['split'] != split_by_id[sample['layout_id']] or
                type(sample['class_index']) is not int or not 0 <= sample['class_index'] < 17 or
                sample['image_status'] != 'not_generated' or sample.get('appearance_variant') not in
                {'nominal', 'material_albedo', 'lighting_exposure'}):
            errors.append('invalid_sample:' + sample['sample_id'])
    for identity, name, _, _ in all_layouts:
        observed = Counter((s['class_index'], s['phase']) for s in samples if s['layout_id'] == identity)
        p = budget['perception']
        expected = {(label, 'stable'): p[name + '_packages_per_known_class_per_family'] for label in range(16)}
        expected.update({(16, phase): p[name + '_packages_per_unknown_phase_per_family'] for phase in p['unknown_phases']})
        if observed != expected:
            errors.append('per_family_class_support:' + identity)
    return {'passed': not errors, 'errors': errors, 'families': len(all_layouts), 'base_tasks': len(tasks),
            'method_episodes': sum(len(t['method_ids']) for t in tasks), 'perception_packages': len(samples),
            'pixels_generated_by_validation': 0}


def freeze_candidate(suite, output, qualification):
    check = validate_suite(suite)
    if not check['passed']:
        raise ValueError('Suite validation failed')
    qualification = Path(qualification).resolve()
    receipt = read_json(qualification)
    required = ['passed', 'production_core_unchanged', 'actual_RGB_smoke_passed',
                'public_replay_passed', 'test_pixels_not_generated', 'regression_passed']
    if (receipt.get('stage') != '7B' or not all(receipt.get(key) is True for key in required) or
            receipt.get('suite_manifest_sha256') != file_hash(Path(suite) / 'dataset_manifest.json') or
            not receipt.get('source_files')):
        raise ValueError('A passed 7B qualification receipt is required')
    paths = set()
    for prefix in ('experiments/block7', 'simulation', 'roadvision', 'configs/experiments/block7',
                   'tests/experiments/block7'):
        paths.update(p for p in (ROOT / prefix).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    paths.update([qualification, ROOT / 'models/road_structure_frozen_v1_20261014/model.pt',
                  ROOT / 'models/road_structure_frozen_v1_20261014/metadata.json'])
    source_paths = {str(Path(row['path']).resolve()) for row in receipt['source_files']}
    if any(str(p.resolve()) not in source_paths for p in paths if p != qualification):
        raise ValueError('Qualification does not cover every current candidate source/config file')
    for evidence in receipt.get('evidence_files', []) + receipt['source_files']:
        p = Path(evidence['path'])
        if not p.is_file() or file_hash(p) != evidence['sha256']:
            raise ValueError('Qualification evidence changed')
        paths.add(p.resolve())
    paths.update(ROOT.glob('requirements*.txt'))
    for p in Path(suite).rglob('*'):
        if p.is_file(): paths.add(p.resolve())
    write_json(output, {'scope': '7B_P_N_candidate_binding_not_final_test_generation',
               'eligible_tracks': ['P', 'N'], 'continuous_authorized': False,
               'suite': str(Path(suite).resolve()), 'qualification': str(qualification),
               'first_final_test_generation_started': False,
               'files': [{'path': str(p.resolve()), 'sha256': file_hash(p)} for p in sorted(paths)]})


def verify_freeze(freeze):
    value = read_json(freeze)
    errors = [row['path'] for row in value['files']
              if not Path(row['path']).is_file() or file_hash(row['path']) != row['sha256']]
    if value.get('continuous_authorized') is not False:
        errors.append('continuous_authorization_not_allowed')
    if errors:
        raise ValueError('Candidate binding mismatch: ' + repr(errors[:5]))
    return value


def authorize_final(task, suite, authorization):
    if not authorization or set(authorization) != {'freeze', 'receipt'}:
        raise ValueError('Final-test rendering needs explicit candidate and first-generation receipt')
    binding = verify_freeze(authorization['freeze'])
    if Path(binding['suite']).resolve() != Path(suite).resolve():
        raise ValueError('Candidate suite differs')
    receipt = read_json(authorization['receipt'])
    if (receipt['freeze_sha256'] != file_hash(authorization['freeze']) or
            receipt['scope'] != 'N_final_test' or receipt['explicit_final_run_flag'] is not True):
        raise ValueError('Final-generation receipt is not bound')
    prepared = {row['task_id']: row for row in read_rows(Path(suite) / 'tasks.jsonl')}
    if task != prepared.get(task['task_id']):
        raise ValueError('Task differs from frozen final manifest')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = validate_suite(args.suite)
    if args.output: write_json(args.output, result)
    print(json.dumps(result))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
