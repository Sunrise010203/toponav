import json
from pathlib import Path
import numpy as np
import pytest
from roadvision.ground_history_run import run_sequence
from roadvision.corridor_rgb_probe import CALIBRATION_ID
from PIL import Image
import hashlib


def test_public_history_runner_keeps_original_times_and_never_authorizes(tmp_path):
    public=tmp_path/'public';public.mkdir();a=np.zeros((256,256,3),np.uint8);Image.fromarray(a).save(public/'a.png')
    frames=[dict(frame_id=f'f{i}',captured_sim_s=i*.1,rgb_path='a.png',rgb_sha256=hashlib.sha256(a.tobytes()).hexdigest()) for i in range(10)]
    manifest=public/'sequence.json';manifest.write_text(json.dumps(dict(version='rgb_probe_sequence_v1',calibration_id=CALIBRATION_ID,frames=frames)))
    out=tmp_path/'out';r=run_sequence(manifest,public,out)
    assert r['window_count']==2 and r['warmup_frame_count']==8
    assert r['ground_status_counts']=={'unknown':2}
    row=json.loads((out/'windows.jsonl').read_text().splitlines()[0])['result']
    assert [f['captured_sim_s'] for f in row['frames']]==[0.,.4,.8]
    assert row['geometry_time_s']==.8 and row['motion_authorized'] is False
    with pytest.raises(FileExistsError):run_sequence(manifest,public,out)
