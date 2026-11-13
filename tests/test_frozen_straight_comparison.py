from simulation.frozen_straight_comparison import run_frozen_case,compare_initial
from simulation.continuous_straight_run import run_case
from simulation.corridor_motion import CorridorConfig


def test_frozen_comparison_matches_initial_render_without_changing_old_controller(tmp_path):
    scene=CorridorConfig(length_m=4.)
    run_case(tmp_path/'new',scene=scene,operator_confirmed=False,max_ticks=2)
    run_frozen_case(tmp_path/'old',scene=scene,fault={'kind':'missing','tick':0})
    r=compare_initial(tmp_path/'new',tmp_path/'old',tmp_path/'comparison.json')
    assert r['same_initial_rgb_sha256'] and r['initial_only_not_same_closed_loop_frames']
    assert r['different_speed_and_evidence_priors']


def test_saved_rgb_tampering_cannot_pass_public_log_integrity(tmp_path):
    import json
    import numpy as np
    import pytest
    from PIL import Image
    from simulation.continuous_straight_acceptance import verify_public_records
    p=tmp_path/'case';run_case(p,operator_confirmed=False,max_ticks=2)
    rows=[json.loads(l) for l in (p/'decisions.jsonl').read_text().splitlines()]
    assert verify_public_records(p,rows)==1
    path=p/'public/frames/0000.png';pixels=np.array(Image.open(path));pixels[1,1]=[0,0,0];Image.fromarray(pixels).save(path)
    with pytest.raises(ValueError):verify_public_records(p,rows)
