"""Offline public-RGB runner for conditional calibrated ground/plane research."""
import argparse
from collections import Counter, deque
from dataclasses import asdict
import json
from pathlib import Path
import time

import cv2

from roadvision.corridor_rgb_probe_run import load_manifest, _read_rgb
from roadvision.calibrated_ground_motion import GroundCalibration, probe_calibrated_ground


def run_sequence(manifest_path, rgb_root, output_dir, *, calibration=None):
    data=load_manifest(manifest_path);root=Path(rgb_root).resolve();output=Path(output_dir)
    calibration=calibration or GroundCalibration()
    output.mkdir(parents=True,exist_ok=False)
    buffer=deque(maxlen=3);ground=Counter();corridor=Counter();front=Counter();windows=0
    started=time.perf_counter()
    with (output/'windows.jsonl').open('x') as log:
        for ref in data['frames']:
            buffer.append(_read_rgb(ref,root))
            if len(buffer)<3:continue
            tick=time.perf_counter()
            try:
                result=probe_calibrated_ground(list(buffer),now_s=buffer[-1]['captured_sim_s'],
                                              calibration_id=data['calibration_id'],calibration=calibration)
                status=result['ground_motion']['status']
            except ValueError as error:
                status='input_rejected'
                result=dict(error=str(error),motion_authorized=False,arrival_status='unverified',atomic_edge_compatible=False)
            ground[status]+=1
            corridor[result.get('corridor_geometry',{}).get('status','input_rejected')]+=1
            front[result.get('front_surface',{}).get('status','input_rejected')]+=1
            log.write(json.dumps(dict(index=windows,frame_ids=[f['frame_id'] for f in buffer],
                                      elapsed_wall_s=time.perf_counter()-tick,result=result),allow_nan=False,separators=(',',':'))+'\n')
            windows+=1
    summary=dict(version='calibrated_ground_motion_run_v1_development',frame_count=len(data['frames']),window_count=windows,
                 ground_status_counts=dict(ground),corridor_status_counts=dict(corridor),front_status_counts=dict(front),
                 elapsed_wall_s=time.perf_counter()-started,calibration_assumption=asdict(calibration),
                 calibration_verified=False,static_scene_verified=False,motion_authorized=False,arrival_status='unverified')
    (output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    return summary


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',required=True,type=Path);p.add_argument('--rgb-root',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path);p.add_argument('--camera-height-m',type=float,default=.45)
    args=p.parse_args(argv);cv2.setNumThreads(1)
    try:
        result=run_sequence(args.manifest,args.rgb_root,args.output,calibration=GroundCalibration(height_m=args.camera_height_m))
    except (ValueError,OSError) as error:p.exit(2,f'Calibrated-ground input/run error: {error}\n')
    print(json.dumps(result,indent=2,allow_nan=False));return 0


if __name__=='__main__':raise SystemExit(main())
