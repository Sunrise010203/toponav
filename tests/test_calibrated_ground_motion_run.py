import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest

from roadvision.corridor_rgb_probe import CALIBRATION_ID
from roadvision.calibrated_ground_motion_run import run_sequence, main


def manifest(tmp_path, times=(0.,.1,.2,.3)):
    root = tmp_path/'rgb'
    root.mkdir()
    frames = []
    for i,t in enumerate(times):
        rgb = np.full((256,256,3),100,np.uint8)
        Image.fromarray(rgb).save(root/f'{i}.png')
        frames.append(dict(frame_id=str(i),captured_sim_s=t,rgb_path=f'{i}.png',
                           rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest()))
    data = dict(version='rgb_probe_sequence_v1', calibration_id=CALIBRATION_ID, frames=frames)
    p=root/'sequence.json'; p.write_text(json.dumps(data))
    return p,root,data


def test_runner_and_replay(tmp_path):
    p,root,_=manifest(tmp_path)
    a=run_sequence(p,root,tmp_path/'out')
    assert a['window_count']==2 and a['ground_status_counts']=={'unknown':2}
    run_sequence(p,root,tmp_path/'replay')
    def results(out):
        return [json.loads(s)['result'] for s in (out/'windows.jsonl').read_text().splitlines()]
    assert results(tmp_path/'out')==results(tmp_path/'replay')
    with pytest.raises(FileExistsError): run_sequence(p,root,tmp_path/'out')


@pytest.mark.parametrize('kind',['truth','wrong_id','hash','path','duplicate','dtype'])
def test_strict_public_inputs(tmp_path,kind):
    p,root,m=manifest(tmp_path)
    if kind=='truth': m['pose']=[1,2,3]
    if kind=='wrong_id': m['calibration_id']='other'
    if kind=='hash': m['frames'][0]['rgb_sha256']='0'*64
    if kind=='path': m['frames'][0]['rgb_path']='../elsewhere.png'
    if kind=='duplicate': m['frames'][1]['frame_id']='0'
    if kind=='dtype': Image.fromarray(np.zeros((256,256),np.uint8)).save(root/'0.png')
    p.write_text(json.dumps(m))
    with pytest.raises(ValueError): run_sequence(p,root,tmp_path/'out')


def test_gap_is_logged_not_extended(tmp_path):
    p,root,_=manifest(tmp_path,times=(0.,.2,.4))
    result=run_sequence(p,root,tmp_path/'out')
    assert result['ground_status_counts']=={'input_rejected':1}


def test_too_few_frames(tmp_path):
    p,root,_=manifest(tmp_path,times=(0.,))
    assert run_sequence(p,root,tmp_path/'out')['window_count']==0


def test_cli_invalid_input(tmp_path):
    with pytest.raises(SystemExit) as error:
        main(['--manifest',str(tmp_path/'missing.json'),'--rgb-root',str(tmp_path),'--output',str(tmp_path/'o')])
    assert error.value.code==2
