"""Real old helper, exact far decisions and unchanged full evidence work."""
from contextlib import contextmanager
import hashlib
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(HERE))
from baseline import load_before
from roadvision import paired_descriptor_checks as after, native_patch_contrast as fast
from roadvision.prepared_descriptor_checks import filter_h,filter_i
from tools.straight_paired_population_benchmark import encode,local_cases,write

before=load_before(HERE/'before/roadvision/paired_descriptor_checks.py')


@contextmanager
def counts():
    match,sample,integral,minmax=cv2.matchTemplate,cv2.getRectSubPix,cv2.integral2,cv2.minMaxLoc
    hncc,incc=fast.h._ncc,fast.i._ncc
    stats=dict(matches=0,support_samples=0,h_ncc=0,i_ncc=0,integrals=0,range_checks=0,
               range_accept=0,range_reject=0,range_ambiguous=0,support_sha256=[])
    def matched(*args,**kw):stats['matches']+=1;return match(*args,**kw)
    def sampled(image,*args,**kw):
        out=sample(image,*args,**kw)
        if image.dtype==np.float32 and image.shape==(128,128):
            stats['support_samples']+=1;stats['support_sha256'].append(hashlib.sha256(out.tobytes()).hexdigest())
        return out
    def integrated(*args,**kw):stats['integrals']+=1;return integral(*args,**kw)
    def extrema(raw,*args,**kw):
        result=minmax(raw,*args,**kw);span=int(result[1])-int(result[0]);stats['range_checks']+=1
        decision=True if span>=47 else False if span<=1 else None
        stats['range_accept' if decision is True else 'range_reject' if decision is False else 'range_ambiguous']+=1
        if decision is not None:
            assert decision==(float(raw.astype(np.float32).std())>=2.)
            a=raw.astype(np.int64);n=121*int((a*a).sum())-int(a.sum())**2
            assert (n>9*121**2) if decision else (n<121**2)
        return result
    def hcall(*args):stats['h_ncc']+=1;return hncc(*args)
    def icall(*args):stats['i_ncc']+=1;return incc(*args)
    try:
        cv2.matchTemplate,cv2.getRectSubPix,cv2.integral2,cv2.minMaxLoc=matched,sampled,integrated,extrema
        fast.h._ncc,fast.i._ncc=hcall,icall
        yield stats
    finally:
        cv2.matchTemplate,cv2.getRectSubPix,cv2.integral2,cv2.minMaxLoc=match,sample,integral,minmax
        fast.h._ncc,fast.i._ncc=hncc,incc


rows=[]
for name,images,region in local_cases():
    hashes=[hashlib.sha256(im.tobytes()).hexdigest() for im in images]
    expected=encode(before.filter_pair(images,region));variants={}
    for label,module,kw in [('before_implicit',before,{}),('before_explicit',before,dict(filters=(filter_h,filter_i))),('after_implicit',after,{}),('after_explicit',after,dict(filters=(filter_h,filter_i)))]:
        with counts() as counter:assert encode(module.filter_pair(images,region,**kw))==expected
        variants[label]=counter
    assert variants['before_implicit']==variants['before_explicit']
    assert variants['after_implicit']==variants['after_explicit']
    old,new=variants['before_explicit'],variants['after_explicit']
    for field in ('matches','support_samples','h_ncc','i_ncc','support_sha256'):assert old[field]==new[field],(name,field)
    assert old['range_checks']==0
    assert new['integrals']==new['range_ambiguous']
    assert new['range_checks']==new['range_accept']+new['range_reject']+new['range_ambiguous']
    if name=='random':assert old['matches']==100 and old['integrals']==100 and new['range_accept']==100 and new['integrals']==0
    if name=='flat':assert new['matches']==0 and new['range_reject']==100 and new['integrals']==0
    if name=='rectified':assert new['range_checks']==0
    assert hashes==[hashlib.sha256(im.tobytes()).hexdigest() for im in images]
    rows.append(dict(case=name,variants=variants,all_outputs_exact=True,arrays_unchanged=True))
write(HERE/'PREFLIGHT.json',dict(cases=rows,old_helper_explicitly_bound=True,original_filters_shared=True,all_outputs_exact=True))
print('preflight exact',len(rows),'local cases; original match/NCC/support work, sufficient range decisions checked')
