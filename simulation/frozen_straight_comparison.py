"""Unmodified fixed-colour controller/loop in SAME initial textured world.

Serial scoped world factory only, as in the existing frozen validation
adapter. Different closed-loop trajectories do NOT mean identical ongoing
RGB. The first-frame SHA check proves the shared initial rendering only.
Speeds/priors/horizons intentionally remain those of each frozen controller;
this is a retained engineering baseline, not a controlled algorithm ablation.
"""
import json
import hashlib
import numpy as np
from PIL import Image
from pathlib import Path
from unittest.mock import patch
from simulation import corridor_motion
from simulation.continuous_audit_world import ContinuousAuditWorld

FAULT_NAMES={'missing':'missing_frame','replay':'replay_frame','stale':'stale_frame','occlusion':'missing_view'}


def run_frozen_case(output,*,scene,appearance=None,seed=313,pattern='unique',fault=None,narrow_obstacle_width_m=.5):
    out=Path(output);assets=out.parent/(out.name+'-assets')
    def factory(config):return ContinuousAuditWorld(config,appearance,assets=assets,seed=seed,pattern=pattern,narrow_obstacle_width_m=narrow_obstacle_width_m)
    with patch.object(corridor_motion,'CorridorWorld',factory):
        r=corridor_motion.run_corridor(scene,fault=FAULT_NAMES[fault['kind']] if fault else 'none',
            fault_cycle=fault['tick'] if fault else 15,output_dir=out)
    return r


def compare_initial(primary_path,frozen_path,output):
    primary_path,frozen_path=Path(primary_path),Path(frozen_path)
    current=json.loads((primary_path/'result.json').read_text());old=json.loads((frozen_path/'run.json').read_text())
    mf=json.loads((primary_path/'public/sequence.json').read_text())
    same=mf['frames'][0]['rgb_sha256']==old['steps'][0]['frame_sha256']
    first_new=np.array(Image.open(primary_path/'public'/mf['frames'][0]['rgb_path']))
    first_old=np.array(Image.open(frozen_path/old['steps'][0]['frame_path']))
    same=same and hashlib.sha256(first_new.tobytes()).hexdigest()==mf['frames'][0]['rgb_sha256'] and hashlib.sha256(first_old.tobytes()).hexdigest()==old['steps'][0]['frame_sha256']
    r=dict(same_initial_rgb_sha256=same,
        initial_only_not_same_closed_loop_frames=True,primary_result=current['controller_result'],
        primary_geometric_stop=current['audit']['normal_geometric_stop_pass'],
        frozen_result=old['feedback'],frozen_end_wall_verified=old['audit']['end_wall_verified'],
        frozen_contact_steps=old['audit']['wall_contact_physics_steps'],
        different_speed_and_evidence_priors=True,scope='engineering baseline; no universal recognition or safety superiority claim')
    with Path(output).open('x') as f:json.dump(r,f,indent=2,allow_nan=False)
    if not same:raise ValueError('Initial render mismatch, not a matched-world comparison')
    return r
