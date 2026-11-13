"""Offline public-RGB-only runner for the conditional ground/rig prototype."""
import argparse
from collections import Counter, deque
import json
from pathlib import Path
import time

import cv2

from roadvision.corridor_rgb_probe_run import load_manifest, _read_rgb
from roadvision.rgb_ground_rig import probe_ground_rig


def run_sequence(manifest_path, rgb_root, output_dir):
    data = load_manifest(manifest_path)
    root, output = Path(rgb_root).resolve(), Path(output_dir)
    output.mkdir(parents=True,exist_ok=False)
    buffer, statuses, regional = deque(maxlen=3), Counter(), Counter()
    windows, started = 0, time.perf_counter()
    with (output/'windows.jsonl').open('x') as log:
        for ref in data['frames']:
            buffer.append(_read_rgb(ref,root))
            if len(buffer)<3: continue
            tick = time.perf_counter()
            try:
                result = probe_ground_rig(list(buffer), now_s=buffer[-1]['captured_sim_s'],
                                          calibration_id=data['calibration_id'])
                status = result['shared_motion']['status']
                regional.update(key for key,v in result['regions'].items() if v['status']=='planar_flow_candidate')
            except ValueError as error:
                status = 'input_rejected'
                result = {'error':str(error),'motion_authorized':False,'arrival_status':'unverified'}
            statuses[status]+=1
            log.write(json.dumps({'index':windows,'frame_ids':[f['frame_id'] for f in buffer],
                'elapsed_wall_s':time.perf_counter()-tick,'result':result},allow_nan=False,separators=(',',':'))+'\n')
            windows+=1
    summary = {'version':'rgb_ground_rig_run_v1','frame_count':len(data['frames']),'window_count':windows,
               'shared_status_counts':dict(statuses),'regional_candidate_counts':dict(regional),
               'elapsed_wall_s':time.perf_counter()-started,'metric_geometry_status':'unknown',
               'motion_authorized':False,'arrival_status':'unverified'}
    (output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    return summary


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',required=True,type=Path)
    parser.add_argument('--rgb-root',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args(argv)
    cv2.setNumThreads(1)
    try: result=run_sequence(args.manifest,args.rgb_root,args.output)
    except (ValueError,OSError) as error: parser.exit(2,f'RGB ground/rig input/run error: {error}\n')
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
