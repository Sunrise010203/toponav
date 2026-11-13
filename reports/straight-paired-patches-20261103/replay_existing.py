"""Replay existing public recordings; recompute OLD traces, not new physics."""
import hashlib
import json
from pathlib import Path

import cv2
from simulation.continuous_straight_acceptance import replay_safety_public, audit_safety_physics
from tools.straight_pipeline_profile import verify_freeze

ROOT=Path.cwd()
out=ROOT/'reports/straight-paired-patches-20261103'
freeze=json.loads((out/'FREEZE_CORRECTED.json').read_bytes())
verify_freeze(ROOT,freeze)
old=ROOT/'reports/straight-evidence-recovery-20261103'
cases=sorted(p for p in (old/'comparison').iterdir() if p.is_dir())
cases+=sorted(p for p in (old/'development').iterdir() if p.is_dir())
write_path=out/'REPLAY.json'
if write_path.exists() or (out/'replay-progress.jsonl').exists():
    raise ValueError('exclusive_output')
previous=cv2.getNumThreads(); cv2.setNumThreads(1)
records=[]
try:
    for base in cases:
        result=json.loads((base/'result.json').read_bytes())
        rows=[json.loads(line) for line in (base/'decisions.jsonl').read_text().splitlines()]
        replay=replay_safety_public(base,result,rows)
        trace=json.loads((base/'audit/physics-trace.json').read_bytes())
        physics=audit_safety_physics(result,rows,trace)
        if physics['errors']: raise AssertionError((base.name,physics))
        record=dict(case=str(base.relative_to(ROOT)),replay=replay,physics=physics,
                    contacts=result['audit']['wall_contact_physics_steps'],
                    safe_final_stop=result['audit']['safe_final_stop'])
        records.append(record)
        with (out/'replay-progress.jsonl').open('a') as stream:
            stream.write(json.dumps(record,allow_nan=False)+'\n')
        print(base.name,'exact',replay,flush=True)
    verify_freeze(ROOT,freeze)
finally:
    cv2.setNumThreads(previous)
summary=dict(cases=records,decisions=sum(r['replay']['decisions'] for r in records),
             terminal_frames=sum(r['replay']['terminal_frames'] for r in records),
             all_public_outputs_exact=True,physics_errors=[],
             scope='existing recordings replay and independent recomputation of old 120Hz traces, NOT new closed-loop physics',
             opencv_threads_restored=cv2.getNumThreads()==previous,frozen_files_unchanged=True)
with write_path.open('x') as stream:
    stream.write(json.dumps(summary,indent=2,allow_nan=False)+'\n')
print({k:v for k,v in summary.items() if k!='cases'},flush=True)
