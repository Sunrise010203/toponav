"""Auditable four-view dataset/inference runner; smoke uses old layouts only.

No training. New final-test rendering is locked behind a matching candidate
binding and an exclusive first-generation receipt. 7B runs only smoke.
"""
import argparse
from contextlib import nullcontext
import csv
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image
import yaml

from experiments.block7.common import CONFIG, atomic_json, check_resources, file_hash, read_json, read_rows, seed, write_json
from experiments.block7.dataset import registered
from experiments.block7.rendering import RenderedWorld
from experiments.block7.validate import validate_suite, verify_freeze


def development_samples():
    phases = ['turning', 'approaching', 'passing', 'occluded', 'missing_view']
    return [{'sample_id': f'dev-P-{label}-{phase}', 'family_id': 'historical-test_locked',
             'layout_id': 'test_locked', 'split': 'development', 'class_index': label,
             'phase': phase, 'appearance_variant': ['nominal', 'material_albedo', 'lighting_exposure'][label % 3],
             'seed': seed('dev-P', label, phase), 'image_status': 'not_generated'}
            for label in range(17) for phase in (phases if label == 16 else ['stable'])]


def run(output, suite=None, split='development', freeze=None, allow_final_test=False):
    if split not in {'development', 'validation', 'test'}:
        raise ValueError('Unknown split')
    if split != 'development' and (suite is None or not validate_suite(suite)['passed']):
        raise ValueError('A valid static suite is required')
    if split == 'test':
        if not allow_final_test or freeze is None:
            raise ValueError('Final perception rendering is locked')
        binding = verify_freeze(freeze)
        if Path(binding['suite']).resolve() != Path(suite).resolve():
            raise ValueError('Suite differs from candidate binding')
    elif allow_final_test:
        raise ValueError('Final-test flag is only valid for test split')
    check_resources()
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
    (output / 'rgb').mkdir()
    if split == 'test':
        write_json(Path(suite) / 'FIRST_P_FINAL_GENERATION_STARTED.json',
                   {'scope': 'P_final_test', 'explicit_final_run_flag': True,
                    'freeze_sha256': file_hash(freeze), 'output': str(output)})
    samples = development_samples() if split == 'development' else [r for r in
               read_rows(Path(suite) / 'perception_samples.jsonl') if r['split'] == split]
    ledger = {'split': split, 'samples': [{**r, 'status': 'not_run', 'reason': 'not_started'} for r in samples]}
    atomic_json(output / 'ledger.json', ledger)
    errors, predicted, count, new_bytes = [], [], 0, 0
    budget = yaml.safe_load((CONFIG / 'budget.yaml').read_text())
    started = time.monotonic()
    model = yaml.safe_load((CONFIG / 'methods.yaml').read_text())['shared']
    write_json(output / 'run-config.json', {'split': split, 'suite': str(suite) if suite else None,
               'freeze': str(freeze) if freeze else None, 'model_sha256': model['model_sha256'],
               'budget_sha256': file_hash(CONFIG / 'budget.yaml'), 'scope': 'P_observation_packages_not_navigation'})
    with (output / 'predictions.jsonl').open('x') as predictions:
        try:
            import torch
            from roadvision.learn import Predictor
            torch.set_num_threads(2)
            predictor = Predictor(Path(model['model_path']), 'cpu')
            for identity in dict.fromkeys(s['layout_id'] for s in samples):
                definition = None if split == 'development' else read_json(Path(suite) / 'layouts' / (identity + '.json'))
                registry = registered(definition['layout']) if definition else nullcontext(identity)
                with registry, RenderedWorld(identity) as world:
                    for row in ledger['samples']:
                        if row['layout_id'] != identity:
                            continue
                        check_resources(start=False)
                        if time.monotonic() - started >= budget['resources']['max_wall_hours_per_stage'] * 3600:
                            raise RuntimeError('Perception stage wall budget reached')
                        if new_bytes >= budget['resources']['max_new_evidence_bytes']:
                            raise RuntimeError('Perception evidence storage budget reached')
                        world.appearance_variant = row['appearance_variant']
                        image, audit = world.sample(row['class_index'], np.random.default_rng(row['seed']),
                                                   unknown_mode=row['phase'] if row['class_index'] == 16 else None)
                        captured = time.time_ns()
                        path = output / 'rgb' / (row['sample_id'] + '.png')
                        Image.fromarray(image).save(path)
                        new_bytes += path.stat().st_size
                        packet = predictor.predict(image, captured_at_ns=captured, frame_id=row['sample_id'])
                        result = {'sample_id': row['sample_id'], 'family_id': row['family_id'],
                                  'class_index': row['class_index'], 'phase': row['phase'],
                                  'appearance_variant': row['appearance_variant'],
                                  'prediction': packet, 'png': str(path.relative_to(output)),
                                  'png_sha256': file_hash(path),
                                  'pixel_sha256': hashlib.sha256(image.tobytes()).hexdigest(), 'render_audit': audit}
                        predictions.write(json.dumps(result, allow_nan=False) + '\n'); predictions.flush()
                        predicted.append({'sample_id': row['sample_id'], 'class_index': row['class_index'],
                                          'predicted_class': packet['class_id'], 'accepted': packet['accepted'],
                                          'reason': packet['rejection_reason'], 'inference_ms': packet['inference_ms']})
                        row.update(status='recorded', reason=packet['rejection_reason'])
                        count += 1
                        atomic_json(output / 'ledger.json', ledger)
        except BaseException as exc:
            errors.append({'type': type(exc).__name__, 'message': str(exc), 'after_samples': count})
        finally:
            atomic_json(output / 'ledger.json', ledger)
    with (output / 'predictions.csv').open('x') as file:
        keys = ['sample_id', 'class_index', 'predicted_class', 'accepted', 'reason', 'inference_ms']
        writer = csv.DictWriter(file, fieldnames=keys); writer.writeheader(); writer.writerows(predicted)
    summary = {'passed': not errors and count == len(samples), 'split': split, 'planned': len(samples),
               'recorded': count, 'not_run': len(samples) - count, 'errors': errors,
               'wall_elapsed_s': time.monotonic() - started, 'model_sha256': model['model_sha256'],
               'scope': 'tool_qualification_only' if split == 'development' else 'P_measurements_no_navigation_claim'}
    write_json(output / 'summary.json', summary)
    paths = [p for p in output.rglob('*') if p.is_file()]
    write_json(output / 'manifest.json', {'files': [{'path': str(p.relative_to(output)), 'sha256': file_hash(p)}
                                                 for p in sorted(paths)]})
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--suite', type=Path)
    parser.add_argument('--split', choices=['development', 'validation', 'test'], default='development')
    parser.add_argument('--freeze', type=Path); parser.add_argument('--allow-final-test', action='store_true')
    args = parser.parse_args()
    try:
        result = run(args.output, args.suite, args.split, args.freeze, args.allow_final_test)
    except (OSError, ValueError, RuntimeError) as exc:
        print(json.dumps({'status': 'invalid_or_blocked', 'error': str(exc)})); return 2
    print(json.dumps(result)); return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
