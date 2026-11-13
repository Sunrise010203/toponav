"""Known RGB only; counterfactual windows never drive motors or select by fit."""
from pathlib import Path
import json
import cv2
from tools.straight_pipeline_profile import load_case
from roadvision.straight_ablation import select_navigation_history
from roadvision.straight_evidence import probe_straight_history
from roadvision.gradient_ground_path import assess_gradient_free_ground
from roadvision.calibrated_ground_motion import GroundCalibration
from roadvision.corridor_rgb_probe import CALIBRATION_ID
p=Path('reports/straight-evidence-recovery-20261103');case=load_case(p/'development/gradient_transfer')
cv2.setNumThreads(1);records=[]
for index,(row,frame) in enumerate(zip(case['rows'],case['frames'])):
 if row['decision']['state']!='waiting_evidence':continue
 buffer=case['frames'][max(0,index-16):index+1]
 selected,receipt=select_navigation_history(buffer,now_s=row['sim_s'],calibration=GroundCalibration())
 out=dict(tick=row['tick'],history=receipt,geometry=None,free_path=None)
 if selected is not None:
  geometry=probe_straight_history(selected,now_s=row['sim_s'],calibration_id=CALIBRATION_ID,side_roi='wide')
  keys=[selected[0],selected[len(selected)//2],selected[-1]]
  free=assess_gradient_free_ground(keys,geometry['ground_motion'],now_s=row['sim_s'],calibration_id=CALIBRATION_ID)
  out.update(geometry=geometry,free_path=free)
 records.append(out)
 print(row['tick'],receipt['selected_size'],None if out['geometry'] is None else out['geometry']['paired_corridor']['reason'],None if out['free_path'] is None else out['free_path']['status'],flush=True)
 with (p/'window-probe-progress.jsonl').open('a') as f:f.write(json.dumps(out,allow_nan=False)+'\n')
with (p/'WINDOW_PROBE.json').open('x') as f:json.dump(records,f,indent=2,allow_nan=False)
