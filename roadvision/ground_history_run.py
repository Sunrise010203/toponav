"""Strict public-RGB offline history runner. No simulator/audit/model reads."""
import argparse
from collections import Counter,deque
from dataclasses import asdict
import json
from pathlib import Path
import time
import cv2
from roadvision.corridor_rgb_probe_run import load_manifest,_read_rgb
from roadvision.calibrated_ground_motion import GroundCalibration
from roadvision.ground_history import probe_history,probe_chained_history,probe_adaptive_history,HISTORY_POLICY


def run_sequence(manifest_path,rgb_root,output_dir,*,calibration=None,stride=4,chained=False,adaptive=False):
    if type(stride) is not int or not 1<=stride<=8:raise ValueError('Stride must be 1..8')
    if chained and adaptive:raise ValueError('Choose one history policy')
    if (chained or adaptive) and stride not in (2,4,6,8):raise ValueError('Full history requires even stride')
    calibration=calibration or GroundCalibration();data=load_manifest(manifest_path)
    root=Path(rgb_root).resolve();out=Path(output_dir);out.mkdir(parents=True,exist_ok=False)
    buffer=deque(maxlen=2*stride+1);counts={k:Counter() for k in ('ground','corridor','front')};windows=0;start=time.perf_counter()
    with (out/'windows.jsonl').open('x') as log:
        for ref in data['frames']:
            buffer.append(_read_rgb(ref,root))
            if len(buffer)<buffer.maxlen:continue
            selected=list(buffer) if chained or adaptive else [buffer[0],buffer[stride],buffer[-1]]
            tick=time.perf_counter()
            try:
                r=(probe_adaptive_history if adaptive else probe_chained_history if chained else probe_history)(selected,now_s=selected[-1]['captured_sim_s'],calibration_id=data['calibration_id'],calibration=calibration)
            except ValueError as error:r=dict(error=str(error),motion_authorized=False,arrival_status='unverified')
            for name,key in [('ground','ground_motion'),('corridor','corridor_geometry'),('front','front_surface')]:
                counts[name][r.get(key,{}).get('status','input_rejected')]+=1
            log.write(json.dumps(dict(index=windows,elapsed_wall_s=time.perf_counter()-tick,result=r),allow_nan=False,separators=(',',':'))+'\n');windows+=1
    summary=dict(version='ground_history_run_v1_development',frame_count=len(data['frames']),
        window_count=windows,warmup_frame_count=min(2*stride,len(data['frames'])),stride=stride,chained=chained,adaptive=adaptive,
        ground_status_counts=dict(counts['ground']),corridor_status_counts=dict(counts['corridor']),front_status_counts=dict(counts['front']),
        elapsed_wall_s=time.perf_counter()-start,calibration_assumption=asdict(calibration),history_policy=HISTORY_POLICY,
        motion_authorized=False,arrival_status='unverified',calibration_verified=False,static_scene_verified=False)
    (out/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n');return summary


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',required=True,type=Path)
    p.add_argument('--rgb-root',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--stride',type=int,default=4);p.add_argument('--chained',action='store_true');p.add_argument('--adaptive',action='store_true');p.add_argument('--camera-height-m',type=float,default=.45)
    a=p.parse_args(argv);cv2.setNumThreads(1)
    try:r=run_sequence(a.manifest,a.rgb_root,a.output,calibration=GroundCalibration(height_m=a.camera_height_m),stride=a.stride,chained=a.chained,adaptive=a.adaptive)
    except (ValueError,OSError) as e:p.exit(2,f'History run error: {e}\n')
    print(json.dumps(r,indent=2,allow_nan=False));return 0


if __name__=='__main__':raise SystemExit(main())
