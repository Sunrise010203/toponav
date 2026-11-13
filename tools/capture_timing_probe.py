"""Opt-in observational wrappers. NEVER imported by production entrypoints.

These stamps observe Python callback delivery, not sensor exposure or kernel
arrival. Hashing/locking adds overhead. No gate, timeout, controller input,
authority or publication is changed. Attach only to an idle, owned instance.
"""
from collections import deque
from contextlib import contextmanager
import copy
import hashlib
import math
import threading
import time

import numpy as np

CAPACITY = 64
MAX_COUNTER = 2**64-1


def rgb_digest(rgb):
    if type(rgb) is not np.ndarray or rgb.dtype!=np.uint8 or rgb.shape not in ((128,128,3),(256,256,3)):
        raise ValueError('Diagnostic hashing limited to original camera/controller RGB shapes')
    return hashlib.sha256(rgb.tobytes()).hexdigest()


class _Condition:
    def __init__(self, original, probe):
        self.original, self.probe = original, probe
    def __enter__(self):
        result = self.original.__enter__()
        self.probe.observe(self.probe.mark, 'lock_acquired_s')
        return result
    def __exit__(self, *args):
        result = self.original.__exit__(*args)
        self.probe.observe(self.probe.mark, 'lock_released_s')
        return result
    def wait(self, *args, **kwargs):
        self.probe.observe(self.probe.mark, 'condition_wait_started_s')
        result = self.original.wait(*args, **kwargs)
        self.probe.observe(self.probe.mark, 'condition_reacquired_s')
        return result
    def __getattr__(self, name):
        return getattr(self.original, name)


class CaptureTimingProbe:
    """Bounded diagnostic history and per-cache-slot metadata, no RGB retention.

    Same thread-local context as original callback, original IO lock order.
    Snapshot never reads IO state under probe lock. Observer failures only
    invalidate diagnostics; they do not veto/repair a controller operation.
    """
    def __init__(self, source_kind, *, clock=time.monotonic):
        if source_kind not in ('replay_delivery', 'isolated_synthetic_delivery', 'live_callback_delivery'):
            raise ValueError('Explicit delivery provenance required')
        self.source_kind=source_kind
        def validated_clock():
            value=clock()
            if type(value) not in (int,float) or not math.isfinite(value) or value<0:
                raise ValueError('Invalid diagnostic monotonic observation')
            return value
        self.clock=validated_clock
        self.lock = threading.RLock()
        self.local = threading.local()
        self.histories = {name:deque(maxlen=CAPACITY) for name in ('callbacks','sync','selections','computations')}
        self.counts = {name:0 for name in self.histories}
        self.observer_errors = 0
        self.ledger = {}
        self.required_views = ()
        self.attached = False
        self.last_selection = None

    def observe(self, fn, *args):
        try:
            return fn(*args)
        except Exception:
            with self.lock:
                self.observer_errors = min(MAX_COUNTER, self.observer_errors+1)
            return None

    def mark(self, key):
        active = getattr(self.local, 'active', None)
        if active is not None:
            active[key] = self.clock()
            if key=='condition_wait_started_s':
                active.setdefault('first_condition_wait_started_s',active[key])
                active['condition_wait_count']=min(MAX_COUNTER,active.get('condition_wait_count',0)+1)

    def append(self, kind, value):
        with self.lock:
            self.counts[kind] = min(MAX_COUNTER, self.counts[kind]+1)
            value['record_id'] = self.counts[kind]
            self.histories[kind].append(value)

    def record_put(self, cameras, view, stamp, rgb):
        if tuple(cameras.views)!=self.required_views:
            raise ValueError('Diagnostic camera configuration changed')
        if type(stamp) is not int or not 0<=stamp<=MAX_COUNTER:
            raise ValueError('Diagnostic timestamp outside finite metadata bounds')
        now = self.clock()
        active = getattr(self.local, 'active', None)
        if active is None or active.get('kind') != 'callback':
            return
        active.update(captured_sim_ns=stamp, rgb_sha256=rgb_digest(rgb), cached_observed_s=now)
        with self.lock:
            self.ledger[(view,stamp)] = dict(view=view, captured_sim_ns=stamp,
                callback_entry_s=active['callback_entry_s'], lock_acquired_s=active.get('lock_acquired_s'),
                decode_started_s=active.get('decode_started_s'), decoded_s=active.get('decoded_s'),
                cached_observed_s=now, rgb_sha256=active['rgb_sha256'], object_id=id(rgb))
            self.ledger = {key:value for key,value in self.ledger.items()
                           if key[0] in cameras.views and key[1] in cameras.views[key[0]]}
            if all(stamp in cameras.views[v] for v in self.required_views):
                self.append('sync', dict(captured_sim_ns=stamp, synchronized_set_observed_s=self.clock(),
                    all_required_views_observed=all((v,stamp) in self.ledger for v in self.required_views)))

    def record_latest(self, cameras, result):
        if result is None:
            return
        if tuple(cameras.views)!=self.required_views:
            raise ValueError('Diagnostic camera configuration changed')
        stamp, rgb = result
        with self.lock:
            views=[]
            for view in self.required_views:
                pixels=cameras.views[view][stamp]
                metadata=self.ledger.get((view,stamp))
                matched = (metadata is not None and metadata['object_id']==id(pixels)
                           and metadata['rgb_sha256']==rgb_digest(pixels))
                entry={k:v for k,v in metadata.items() if k!='object_id'} if metadata else dict(view=view,captured_sim_ns=stamp)
                entry['matches_observed_cache_pixels']=matched
                views.append(entry)
            record=dict(captured_sim_ns=stamp, selected_observed_s=self.clock(),
                rgb_sha256=rgb_digest(rgb), views=views,
                all_view_bindings_verified=all(v['matches_observed_cache_pixels'] for v in views))
            self.append('selections',record)
            self.last_selection=record.copy()
            active=getattr(self.local,'active',None)
            if active is not None:
                active['selection_id']=record['record_id']

    def compute_input(self, frame):
        if not isinstance(frame,dict) or 'rgb' not in frame:
            return dict(selection_binding_verified=False)
        sha=rgb_digest(frame['rgb'])
        captured=frame.get('captured_sim_s')
        with self.lock:
            selected=self.last_selection
            matched=(selected is not None and type(captured) in (int,float) and math.isfinite(captured)
                     and round(captured*1e9)==selected['captured_sim_ns'] and sha==selected['rgb_sha256'])
            auxiliary={name:rgb_digest(frame[name+'_rgb']) for name in ('up','down') if name+'_rgb' in frame}
            aux_ok=(selected is not None and set(auxiliary)==set(self.required_views)&{'up','down'}
                    and all(any(v['view']==name and v.get('rgb_sha256')==digest
                        and v['matches_observed_cache_pixels'] for v in selected['views']) for name,digest in auxiliary.items()))
            frame_id=frame.get('frame_id')
            safe_sim=captured if type(captured) in (int,float) and math.isfinite(captured) and 0<=captured<=1e12 else None
            return dict(frame_id=frame_id if type(frame_id) is str and len(frame_id)<=256 else None,captured_sim_s=safe_sim,rgb_sha256=sha,
                auxiliary_sha256=auxiliary, selection_id=selected['record_id'] if matched else None,
                selection_binding_verified=bool(matched and aux_ok and selected['all_view_bindings_verified']))

    @contextmanager
    def attach(self, io, controller):
        if self.attached or isinstance(io.condition,_Condition):
            raise ValueError('Attach requires an idle uninstrumented instance')
        views=tuple(io.cameras.views)
        if views[:4]!=('front','left','right','back') or not 4<=len(views)<=6 or not set(views[4:])<= {'up','down'}:
            raise ValueError('Unsupported camera view configuration')
        self.required_views=views
        originals=[]
        def replace(obj,name,value):
            present=name in obj.__dict__
            originals.append((obj,name,present,obj.__dict__.get(name)))
            setattr(obj,name,value)
        image,pixels,next_frame=io.image,io.pixels,io.next_frame
        put,latest,update=io.cameras.put,io.cameras.latest,controller.update
        self.attached=True
        with self.lock:
            self.ledger.clear();self.last_selection=None
        def image_wrapper(view,msg):
            prior=getattr(self.local,'active',None)
            active=dict(kind='callback',view=view if type(view) is str and view in views else None,callback_entry_s=self.observe(self.clock),
                actual_exposure_wall_s=None,exposure_clock_mapping=None,source_kind=self.source_kind,
                outcome='raised')
            self.local.active=active
            try:
                result=image(view,msg)
                active['outcome']='returned'
                return result
            finally:
                self.observe(self.mark,'callback_finished_s')
                active['io_error_present']=io.__dict__.get('error') is not None
                self.observe(self.append,'callbacks',active)
                self.local.active=prior
        def pixels_wrapper(*args,**kwargs):
            self.observe(self.mark,'decode_started_s')
            result=pixels(*args,**kwargs)
            self.observe(self.mark,'decoded_s')
            return result
        def put_wrapper(view,stamp,rgb):
            result=put(view,stamp,rgb)
            self.observe(self.record_put,io.cameras,view,stamp,rgb)
            return result
        def latest_wrapper(*args,**kwargs):
            result=latest(*args,**kwargs)
            self.observe(self.record_latest,io.cameras,result)
            return result
        def next_wrapper(*args,**kwargs):
            prior=getattr(self.local,'active',None)
            active=dict(kind='selection',next_frame_started_s=self.observe(self.clock))
            self.local.active=active
            try:
                return next_frame(*args,**kwargs)
            finally:
                self.observe(self.mark,'next_frame_finished_s')
                with self.lock:
                    if active.get('selection_id') is not None:
                        selected=next((r for r in self.histories['selections'] if r['record_id']==active['selection_id']),None)
                        if selected is not None:selected.update(active)
                self.local.active=prior
        def update_wrapper(frame,*args,**kwargs):
            info=self.observe(self.compute_input,frame) or dict(selection_binding_verified=False)
            info.update(compute_started_s=self.observe(self.clock),outcome='raised')
            try:
                result=update(frame,*args,**kwargs)
                info['outcome']='returned'
                return result
            finally:
                info['compute_finished_s']=self.observe(self.clock)
                self.observe(self.append,'computations',info)
        try:
            replace(io,'condition',_Condition(io.condition,self))
            for obj,name,value in ((io,'image',image_wrapper),(io,'pixels',pixels_wrapper),(io,'next_frame',next_wrapper),
                    (io.cameras,'put',put_wrapper),(io.cameras,'latest',latest_wrapper),(controller,'update',update_wrapper)):
                replace(obj,name,value)
            yield self
        finally:
            for obj,name,present,value in reversed(originals):
                if present:setattr(obj,name,value)
                else:delattr(obj,name)
            self.attached=False

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(dict(source_kind=self.source_kind,required_views=self.required_views,
                capacity=CAPACITY,counts=self.counts,
                counts_are_lower_bounds=any(n==MAX_COUNTER for n in self.counts.values()),
                dropped={k:max(0,n-len(self.histories[k])) for k,n in self.counts.items()},
                histories={k:list(v) for k,v in self.histories.items()},ledger_slots=len(self.ledger),
                observer_errors=self.observer_errors,actual_exposure_wall_s=None,clock_mapping_verified=False,
                scope='instrumented Python delivery/cache/compute only; NOT historical capture, kernel arrival, realtime or physical stop'))
