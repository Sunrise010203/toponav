"""Frozen serial AB/BA population-sharing benchmark; no actuator/acceptance.

Decode and verification are outside timers. Complete controller sequences are
reconstructed for every pass, including warmups, with every alias restored.
"""
import argparse
from contextlib import contextmanager
import hashlib
import importlib.util
import json
from pathlib import Path
import time

import cv2
import numpy as np

from roadvision import paired_descriptor_checks as candidate
from tools.straight_pipeline_profile import (
    controller_for_task, decision_call, discover_bindings, distribution,
    load_case, pipeline_bindings, verify_freeze,
)

ROOT = Path(__file__).resolve().parents[1]


def encode(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def write(path, value):
    with path.open('x') as stream:
        stream.write(json.dumps(value, indent=2, allow_nan=False) + '\n')


def load_before(path):
    spec = importlib.util.spec_from_file_location('_before_paired_populations', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@contextmanager
def replace_aliases(bindings, fn):
    previous = [(owner, name, vars(owner)[name]) for owner, name, _ in bindings]
    try:
        for owner, name, _ in previous:
            setattr(owner, name, fn)
        yield
    finally:
        for owner, name, original in reversed(previous):
            setattr(owner, name, original)


def local_cases():
    rng = np.random.default_rng(817)
    gray = rng.integers(0, 256, (128, 128), dtype=np.uint8)
    cases = []
    for name, pixels, shift, rectified in [
        ('random', gray, 0., False), ('flat', np.full_like(gray, 70), 0., False),
        ('repeated', np.tile(gray[:8, :8], (16, 16)), 0., False),
        ('blur', cv2.GaussianBlur(gray, (7, 7), 1.2), 0., False),
        ('shift_padding', gray, 2.25, False), ('rectified', gray, 0., True),
    ]:
        first = np.repeat(pixels[:, :, None], 3, axis=2)
        images = [cv2.warpAffine(first, np.array([[1., 0., s], [0., 1., 0.]]), (128, 128))
                  for s in (0., shift / 2, shift)]
        tracks = [dict(first=[x+.125,y+.375], middle=[x+.125+shift/2,y+.375], last=[x+.125+shift,y+.375])
                  for y in (15.,35.,60.,85.,110.) for x in (15.,35.,60.,85.,110.)]
        region = dict(view='front', rectified_ground_assumption=rectified, tracks=tracks,
                      coverage_map=[[1.,0.,0.],[0.,1.7,-13.],[0.,0.,1.]])
        cases.append((name, images, region))
    return cases


def stats(values):
    return dict(n=len(values), median=float(np.median(values)), mean=float(np.mean(values)),
                minimum=float(np.min(values)), maximum=float(np.max(values)))


def summarize(samples):
    out = {}
    for label in ('before', 'after'):
        items = [s for s in samples if s['implementation'] == label]
        out[label] = {key: stats([s[key] for s in items]) for key in ('wall_s','cpu_s')}
    out['wall_median_reduction_fraction'] = 1-out['after']['wall_s']['median']/out['before']['wall_s']['median']
    out['wall_mean_reduction_fraction'] = 1-out['after']['wall_s']['mean']/out['before']['wall_s']['mean']
    return out


def run(plan_path, freeze_path, freeze_sha, output):
    raw = freeze_path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != freeze_sha:
        raise ValueError('freeze_digest')
    freeze = json.loads(raw)
    verify_freeze(ROOT, freeze)
    if str(plan_path.resolve().relative_to(ROOT)) not in freeze['files']:
        raise ValueError('unfrozen_plan')
    plan = json.loads(plan_path.read_bytes())
    if not 2 <= plan['controller_rounds'] <= 8 or not 2 <= plan['local_rounds'] <= 16 or plan['warmups'] != 1:
        raise ValueError('bounded_plan')
    before_path = ROOT / plan['before_source']
    if str(before_path.relative_to(ROOT)) not in freeze['files']:
        raise ValueError('unfrozen_before')
    before = load_before(before_path)
    cases = [(c['name'], load_case(ROOT/c['path'])) for c in plan['cases']]
    for _, case in cases:
        if any(str(p.relative_to(ROOT)) not in freeze['files'] for p in case['files']):
            raise ValueError('unfrozen_input')
    pipeline_bindings()  # preload every supported pipeline BEFORE alias discovery
    bindings = discover_bindings([(candidate.filter_pair, 'paired')])
    if not {'roadvision.ground_history', 'roadvision.straight_history_support'}.issubset({m.__name__ for m, _, _ in bindings}):
        raise ValueError('missing_pipeline_alias')
    methods = dict(before=before.filter_pair, after=candidate.filter_pair)
    output.mkdir(parents=True, exist_ok=False)
    write(output/'plan.json', dict(plan, frozen_file_sha256=freeze_sha,
          aliases=[f'{owner.__name__}.{name}' for owner,name,_ in bindings],
          scope='in-memory controller calls only; no decode, transport or physics', opencv_threads=1))
    previous_threads = cv2.getNumThreads()
    local_out, controller_out = [], []
    cv2.setNumThreads(1)
    try:
        for name, images, region in local_cases():
            expected = encode(methods['before'](images,region))
            if encode(methods['after'](images,region)) != expected:
                raise AssertionError(('local_warmup',name))
            samples=[]
            for repeat in range(plan['local_rounds']):
                order = ('before','after') if repeat % 2 == 0 else ('after','before')
                for label in order:
                    cpu=time.process_time_ns(); wall=time.perf_counter_ns()
                    value=methods[label](images,region)
                    wall=(time.perf_counter_ns()-wall)/1e9; cpu=(time.process_time_ns()-cpu)/1e9
                    if encode(value) != expected: raise AssertionError(('local',name,repeat,label))
                    samples.append(dict(repeat=repeat,implementation=label,wall_s=wall,cpu_s=cpu))
            item=dict(case=name, samples=samples, all_outputs_exact=True,
                      input_rgb_sha256=[hashlib.sha256(im.tobytes()).hexdigest() for im in images],
                      output_sha256=hashlib.sha256(expected).hexdigest(), **summarize(samples))
            write(output/(name+'-local.json'),item); local_out.append(item)
            print('local',name,'exact',flush=True)
        for name, case in cases:
            samples=[]
            def execute(label, repeat):
                controller=controller_for_task(case['task'])
                walls=[]; cpus=[]
                with replace_aliases(bindings,methods[label]):
                    for row,frame in zip(case['rows'],case['frames']):
                        cpu=time.process_time_ns(); wall=time.perf_counter_ns()
                        value=decision_call(controller,row,frame)
                        wall=(time.perf_counter_ns()-wall)/1e9; cpu=(time.process_time_ns()-cpu)/1e9
                        if encode(value) != encode(row['decision']):
                            raise AssertionError(('controller',name,label,repeat,row['tick']))
                        walls.append(wall); cpus.append(cpu)
                        record=dict(case=name,implementation=label,repeat=repeat,tick=row['tick'],
                                    wall_s=wall,cpu_s=cpu,exact=True)
                        with (output/'rows.jsonl').open('a') as stream:
                            stream.write(json.dumps(record,allow_nan=False)+'\n')
                print('controller',name,label,repeat,'exact',len(walls),flush=True)
                return dict(implementation=label,repeat=repeat,wall_s=sum(walls),cpu_s=sum(cpus),
                            decision_wall=distribution(walls),decision_cpu=distribution(cpus))
            execute('before',-1); execute('after',-1)
            for repeat in range(plan['controller_rounds']):
                order=('before','after') if repeat % 2 == 0 else ('after','before')
                for label in order:
                    samples.append(execute(label,repeat))
            item=dict(case=name,decisions_per_pass=len(case['rows']),samples=samples,
                      all_outputs_exact=True, **summarize(samples))
            write(output/(name+'-controller.json'),item); controller_out.append(item)
        verify_freeze(ROOT,freeze)
    finally:
        cv2.setNumThreads(previous_threads)
    gate=all(c['wall_median_reduction_fraction']>0 and c['wall_mean_reduction_fraction']>0 for c in controller_out)
    summary=dict(local_cases=local_out,controller_cases=controller_out,
                 complete_controller_performance_gate_passed=gate,all_outputs_exact=True,
                 frozen_files_unchanged=True,opencv_threads_restored=cv2.getNumThreads()==previous_threads,
                 measured_decisions=2*plan['controller_rounds']*sum(len(c['rows']) for _,c in cases),
                 warmup_decisions=2*sum(len(c['rows']) for _,c in cases),
                 navigation_acceptance=False,physical_audit_performed=False,real_time_budget_proved=False,
                 versions=dict(numpy=np.__version__,opencv=cv2.__version__))
    write(output/'SUMMARY.json',summary)
    return {k:v for k,v in summary.items() if k not in ('local_cases','controller_cases')}


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--freeze',type=Path,required=True)
    p.add_argument('--freeze-sha256',required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    print(json.dumps(run(args.plan,args.freeze,args.freeze_sha256,args.output)))
