"""Exact baseline dispatch, retained work, support bytes and lazy conversions."""
from contextlib import contextmanager
import hashlib
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(HERE))
from baseline import load_before
from roadvision import paired_descriptor_checks as after, native_patch_contrast as fast
from roadvision.prepared_descriptor_checks import filter_h,filter_i
from roadvision.prepared_support_mask import PreparedSupportMask
from tools.straight_paired_population_benchmark import encode,local_cases,write

before=load_before(HERE/'before/roadvision/paired_descriptor_checks.py')


@contextmanager
def counts():
    original_match=cv2.matchTemplate;original_sample=cv2.getRectSubPix
    original_canvas=PreparedSupportMask.sampling_canvas
    original_h,original_i=fast.h._ncc,fast.i._ncc
    stats=dict(matches=0,support_samples=0,prepared_requests=0,conversions=0,h_ncc=0,i_ncc=0,support_sha256=[])
    def match(*args,**kwargs):stats['matches']+=1;return original_match(*args,**kwargs)
    def sample(image,*args,**kwargs):
        result=original_sample(image,*args,**kwargs)
        if image.dtype==np.float32 and image.shape==(128,128):
            stats['support_samples']+=1
            stats['support_sha256'].append(hashlib.sha256(result.tobytes()).hexdigest())
        return result
    def canvas(self):
        stats['prepared_requests']+=1
        if self._canvas is None:stats['conversions']+=1
        result=original_canvas(self)
        expected=self.source.astype(np.float32)
        assert result.shape==expected.shape and result.strides==expected.strides and result.tobytes()==expected.tobytes()
        return result
    def hncc(*args):stats['h_ncc']+=1;return original_h(*args)
    def incc(*args):stats['i_ncc']+=1;return original_i(*args)
    try:
        cv2.matchTemplate,cv2.getRectSubPix=match,sample
        PreparedSupportMask.sampling_canvas=canvas
        fast.h._ncc,fast.i._ncc=hncc,incc
        yield stats
    finally:
        cv2.matchTemplate,cv2.getRectSubPix=original_match,original_sample
        PreparedSupportMask.sampling_canvas=original_canvas
        fast.h._ncc,fast.i._ncc=original_h,original_i


rows=[]
for name,images,region in local_cases():
    hashes=[hashlib.sha256(im.tobytes()).hexdigest() for im in images]
    expected=encode(before.filter_pair(images,region));variants={}
    for label,module,kwargs in [('before_implicit',before,{}),('before_explicit',before,dict(filters=(filter_h,filter_i))),('after_implicit',after,{}),('after_explicit',after,dict(filters=(filter_h,filter_i)))]:
        with counts() as count:
            assert encode(module.filter_pair(images,region,**kwargs))==expected
        variants[label]=count
    assert variants['before_implicit']==variants['before_explicit']
    assert variants['after_implicit']==variants['after_explicit']
    for field in ('matches','support_samples','h_ncc','i_ncc','support_sha256'):
        assert variants['before_explicit'][field]==variants['after_explicit'][field],(name,field)
    assert variants['before_explicit']['prepared_requests']==0
    if name=='random':
        assert variants['after_explicit']['matches']==100
        assert variants['after_explicit']['support_samples']==50
        assert variants['after_explicit']['prepared_requests']==50
        assert variants['after_explicit']['conversions']==2
    if name in ('flat','rectified'):assert variants['after_explicit']['conversions']==0
    assert hashes==[hashlib.sha256(im.tobytes()).hexdigest() for im in images]
    rows.append(dict(case=name,variants=variants,all_outputs_exact=True,arrays_unchanged=True))
write(HERE/'PREFLIGHT.json',dict(cases=rows,old_helper_explicitly_bound=True,original_filters_shared=True,all_outputs_exact=True))
print('preflight exact',len(rows),'local cases; baseline old helper, candidate prepared masks')
