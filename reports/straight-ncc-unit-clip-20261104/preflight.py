"""Before timing: actual math operands and complete local outputs are exact."""
from contextlib import contextmanager
import hashlib
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(HERE))
from ncc_binding import make_binding
from tools.straight_function_benchmark import replace_aliases
from roadvision.prepared_descriptor_checks import filter_h,filter_i
from tools.straight_paired_population_benchmark import encode,local_cases,write


def digest(a):
    return (a.dtype.str,a.shape,a.strides,hashlib.sha256(a.tobytes()).hexdigest())


@contextmanager
def observed(binding,function):
    original_match,original_sample=cv2.matchTemplate,cv2.getRectSubPix
    original_norm,original_clip=np.linalg.norm,np.clip
    stats=dict(matches=0,support_samples=0,support_sha256=[],ncc_calls=0,ncc_inputs=[],
               ncc_outputs=[],norm_operands=[],ncc_clip_calls=0)
    depth=0
    def ncc(a,b):
        nonlocal depth
        stats['ncc_calls']+=1;stats['ncc_inputs'].append((digest(a),digest(b)))
        depth+=1
        try: result=function(a,b)
        finally:depth-=1
        stats['ncc_outputs'].append(None if result is None else struct.pack('=d',result).hex())
        return result
    def matched(*a,**kw):stats['matches']+=1;return original_match(*a,**kw)
    def sampled(image,*a,**kw):
        result=original_sample(image,*a,**kw)
        if image.dtype==np.float32 and image.shape==(128,128):
            stats['support_samples']+=1;stats['support_sha256'].append(hashlib.sha256(result.tobytes()).hexdigest())
        return result
    def norm(a,*args,**kw):
        if depth:stats['norm_operands'].append(digest(a))
        return original_norm(a,*args,**kw)
    def clip(*a,**kw):
        if depth:stats['ncc_clip_calls']+=1
        return original_clip(*a,**kw)
    with replace_aliases(binding.function_bindings,ncc), patch.object(cv2,'matchTemplate',matched), \
            patch.object(cv2,'getRectSubPix',sampled),patch.object(np.linalg,'norm',norm),patch.object(np,'clip',clip):
        yield stats


if __name__=='__main__':
    binding=make_binding();cases=[];dispatches=[]
    for name,images,region in local_cases():
        expected=encode(binding.before(images,region));rgb=[digest(im) for im in images];region_bytes=encode(region)
        variants={}
        for label,variant,kw in [('before_implicit',binding.before,{}),('before_explicit',binding.before,dict(filters=(filter_h,filter_i))),
            ('after_implicit',binding.after,{}),('after_explicit',binding.after,dict(filters=(filter_h,filter_i)))]:
            with observed(binding,variant.function) as counters:
                assert encode(binding.pair_function(images,region,**kw))==expected
            variants[label]=counters
        assert variants['before_implicit']==variants['before_explicit']
        assert variants['after_implicit']==variants['after_explicit']
        old,new=variants['before_explicit'],variants['after_explicit']
        for key in old:
            if key!='ncc_clip_calls':assert old[key]==new[key],(name,key)
        assert new['ncc_clip_calls']<=old['ncc_clip_calls']
        if name=='random':assert old['matches']==100 and new['ncc_clip_calls']<old['ncc_clip_calls']
        records=[]
        for variant in (binding.before,binding.after):
            for explicit in (False,True):
                kw=dict(filters=(filter_h,filter_i)) if explicit else {}
                assert encode(variant(images,region,**kw))==expected
                ns=SimpleNamespace(pair=binding.pair_function)
                with binding.controller_context([(ns,'pair','pair')],variant):
                    assert all(vars(owner)[key] is variant.function for owner,key,_ in binding.function_bindings)
                    assert encode(ns.pair(images,region,**kw))==expected
                records.append(dict(variant='before' if variant is binding.before else 'after',explicit=explicit,
                                    local_and_controller_scope_exact=True))
        assert rgb==[digest(im) for im in images] and region_bytes==encode(region)
        assert all(vars(owner)[key] is binding.after.function for owner,key,_ in binding.function_bindings)
        cases.append(dict(case=name,variants=variants,all_outputs_exact=True,inputs_unchanged=True))
        dispatches.append(dict(case=name,records=records))
    write(HERE/'PREFLIGHT.json',dict(cases=cases,symmetric_dispatch_checks=dispatches,baseline_binding=binding.record(),
        all_outputs_exact=True,actual_match_ncc_norm_and_support_work_unchanged=True,aliases_restored=True))
    print('6 cases exact: match/NCC/norm operands/support identical, only identity clip calls omitted; symmetric dispatch exact')
