"""Real GazeboIO methods, isolated packed-RGB messages, no native constructor.

Replay delivery timestamps are NEW diagnostic timestamps, never historical
camera exposure. No publisher, actuator, step helper or simulator is started.
"""
from contextlib import nullcontext
import argparse
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS

import numpy as np

from simulation.gazebo_cross_validation import GazeboIO,CameraBuffer,VIEWS
from tools.capture_timing_probe import CaptureTimingProbe,rgb_digest
from tools.straight_pipeline_profile import load_case,controller_for_task,decision_call

SYNTHETIC_CASES=('four','six','partial','lock_wait','selection_wait','malformed')


def write_json(path,value):
    with Path(path).open('x') as f:
        json.dump(value,f,indent=2,allow_nan=False);f.write('\n')


def isolated_io(extra_views=()):
    io=GazeboIO.__new__(GazeboIO)
    io.condition=threading.Condition()
    io.cameras=CameraBuffer(extra_views)
    io.error=None;io.now_ns=0
    return io


def message(rgb,stamp):
    h,w,_=rgb.shape
    sec,nsec=divmod(stamp,10**9)
    return NS(width=w,height=h,step=w*3,pixel_format_type=3,data=rgb.tobytes(),
        header=NS(stamp=NS(sec=sec,nsec=nsec)),
        DESCRIPTOR=NS(fields_by_name={'pixel_format_type':NS(enum_type=NS(values_by_number={3:NS(name='RGB_INT8')}))}))


def deliver(io,rgb,stamp,views=None):
    tiles=(rgb[:128,:128],rgb[:128,128:],rgb[128:,:128],rgb[128:,128:])
    for view,pixels in zip(VIEWS,tiles):
        if views is None or view in views:io.image(view,message(pixels,stamp))
    for view in ('up','down'):
        if view in io.cameras.views and (views is None or view in views):io.image(view,message(rgb,stamp))


class Echo:
    def update(self,frame,now):
        return frame


def synthetic_case(case):
    if case not in SYNTHETIC_CASES:raise ValueError('Unknown isolation case')
    io=isolated_io(('up','down') if case=='six' else ())
    rgb=np.random.default_rng(712).integers(0,256,(256,256,3),dtype=np.uint8)
    io.now_ns=100_000_000
    entered=threading.Event();waiting=threading.Event()
    def clock():
        stamp=time.monotonic()
        if threading.current_thread().name=='isolated-callback':entered.set()
        return stamp
    if case=='selection_wait':
        # Notification only: the SAME original Condition.wait still executes.
        original_wait=io.condition.wait
        def wait_notice(*args,**kwargs):
            waiting.set();return original_wait(*args,**kwargs)
        io.condition.wait=wait_notice
    echo=Echo();probe=CaptureTimingProbe('isolated_synthetic_delivery',clock=clock)
    before_condition=io.condition
    threads=[]
    with probe.attach(io,echo):
        if case=='malformed':
            bad=message(rgb[:128,:128],io.now_ns);bad.data=b''
            io.image('front',bad)
            assert io.error=='Gazebo camera must provide packed RGB_INT8'
        else:
            if case=='partial':
                deliver(io,rgb,io.now_ns,views=VIEWS[:3]);assert io.cameras.latest() is None
                deliver(io,rgb,io.now_ns,views=VIEWS[3:])
            elif case=='lock_wait':
                def feed():
                    deliver(io,rgb,io.now_ns)
                with io.condition:
                    t=threading.Thread(target=feed,name='isolated-callback');threads.append(t);t.start()
                    assert entered.wait(1.)
                    time.sleep(.03)
                t.join(2.);assert not t.is_alive()
            elif case=='selection_wait':
                def feed_later():
                    assert waiting.wait(2.)
                    time.sleep(.03);deliver(io,rgb,io.now_ns)
                t=threading.Thread(target=feed_later);threads.append(t);t.start()
            else:deliver(io,rgb,io.now_ns)
            stamp,selected,now=io.next_frame(-1)
            assert stamp==io.now_ns and np.array_equal(selected,rgb)
            frame=dict(frame_id='synthetic',captured_sim_s=stamp/1e9,rgb=selected)
            if case=='six':
                frame.update(up_rgb=io.cameras.views['up'][stamp],down_rgb=io.cameras.views['down'][stamp])
            assert echo.update(frame,now) is frame
        for t in threads:t.join(2.);assert not t.is_alive()
    trace=probe.snapshot()
    assert io.condition is before_condition and not probe.attached
    assert trace['observer_errors']==0 and not trace['clock_mapping_verified']
    if case!='malformed':assert trace['histories']['computations'][0]['selection_binding_verified']
    if case=='lock_wait':
        first=trace['histories']['callbacks'][0]
        assert first['lock_acquired_s']-first['callback_entry_s']>=.02
    if case=='selection_wait':
        selected=trace['histories']['selections'][-1]
        assert selected['condition_reacquired_s']>=selected['condition_wait_started_s']
    return trace


def run(cases,output,*,root,freeze,prefix=12):
    from tools.straight_pipeline_profile import verify_freeze
    import cv2
    root=Path(root);output=Path(output);verify_freeze(root,freeze)
    if type(prefix) is not int or prefix<1:raise ValueError('Positive integer prefix required')
    data={c['name']:load_case(root/c['path']) for c in cases}
    for case in data.values():
        assert all(str(p.relative_to(root)) in freeze['files'] for p in case['files'])
    output.mkdir(exist_ok=False)
    original_threads=cv2.getNumThreads();results=[];pairs={}
    try:
        cv2.setNumThreads(1)
        for repeat in range(3):
            for case in SYNTHETIC_CASES:
                write_json(output/f'synthetic-{repeat}-{case}.json',synthetic_case(case))
        for repeat,modes in enumerate((('ordinary','instrumented'),('instrumented','ordinary'))):
            for name,case in data.items():
                for mode in modes:
                    io=isolated_io();controller=controller_for_task(case['task'])
                    probe=CaptureTimingProbe('replay_delivery')
                    decisions=[];timings=[]
                    with probe.attach(io,controller) if mode=='instrumented' else nullcontext():
                        for row,original in zip(case['rows'][:prefix],case['frames'][:prefix]):
                            assert original is not None and row.get('external_event') is None
                            stamp=round(original['captured_sim_s']*1e9);io.now_ns=round(row['sim_s']*1e9)
                            deliver(io,original['rgb'],stamp)
                            selected_stamp,rgb,now=io.next_frame(stamp-1)
                            assert selected_stamp==stamp and np.array_equal(rgb,original['rgb']) and now==row['sim_s']
                            frame=dict(frame_id=original['frame_id'],captured_sim_s=original['captured_sim_s'],rgb=rgb)
                            start=time.monotonic();decision=decision_call(controller,row,frame);end=time.monotonic()
                            assert decision==row['decision'],(name,mode,row['tick'])
                            decisions.append(decision);timings.append(dict(tick=row['tick'],outer_update_wall_s=end-start))
                    value=dict(case=name,repeat=repeat,mode=mode,decisions=decisions,timings=timings,
                        exact_saved_decisions=True,trace=probe.snapshot() if mode=='instrumented' else None)
                    if mode=='instrumented':
                        assert probe.observer_errors==0 and all(r['selection_binding_verified'] for r in probe.snapshot()['histories']['computations'])
                    write_json(output/f'{repeat}-{name}-{mode}.json',value)
                    pairs[(name,repeat,mode)]=decisions;results.append(dict(case=name,repeat=repeat,mode=mode,count=len(decisions)))
        for name in data:
            for repeat in range(2):assert pairs[(name,repeat,'ordinary')]==pairs[(name,repeat,'instrumented')]
        verify_freeze(root,freeze)
    finally:cv2.setNumThreads(original_threads)
    summary=dict(results=results,exact_decisions=sum(r['count'] for r in results),synthetic_runs=18,
        original_opencv_threads=original_threads,measured_opencv_threads=1,restored_opencv_threads=cv2.getNumThreads(),
        production_edits=False,actual_capture_wall_s=None,physical_motion=False,
        prefix_decisions=prefix,scope='public prefixes through real isolated callback/cache/selector and controller; no native camera or historical exposure')
    write_json(output/'summary.json',summary);return summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--freeze',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();root=Path(__file__).resolve().parents[1]
    plan=json.loads(args.plan.read_text())
    summary=run(plan['public_cases'],args.output,root=root,freeze=json.loads(args.freeze.read_text()),prefix=plan['prefix_decisions'])
    print(json.dumps(summary))


if __name__=='__main__':main()
