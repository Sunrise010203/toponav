"""Check exact old bodies AND explicit-filter dispatch before valid timing."""
from contextlib import contextmanager
import hashlib,inspect,json
from pathlib import Path
import cv2
from roadvision import paired_descriptor_checks as candidate
from roadvision.prepared_descriptor_checks import filter_h,filter_i
from tools.straight_paired_patch_benchmark import load_baseline
from tools.straight_paired_population_benchmark import local_cases,encode
r=Path('reports/straight-paired-patches-20261103')
before=load_baseline(r/'before/roadvision/paired_descriptor_checks.py')
assert '_appearance' not in inspect.signature(before.h.patch_evidence).parameters
assert '_appearance' not in inspect.signature(before.i.appearance_evidence).parameters
@contextmanager
def counts(module):
 old_match=cv2.matchTemplate;old_patch=module.h._patch;old_ncc=module.h._ncc
 originals=(module.h._patch,module.i._patch,module.h._ncc,module.i._ncc)
 assert originals[0] is originals[1] and originals[2] is originals[3]
 counter=dict(match=0,patch=0,ncc=0)
 def match(*args,**kwargs): counter['match']+=1;return old_match(*args,**kwargs)
 def patch(*args,**kwargs): counter['patch']+=1;return old_patch(*args,**kwargs)
 def ncc(*args,**kwargs): counter['ncc']+=1;return old_ncc(*args,**kwargs)
 try:
  cv2.matchTemplate=match;module.h._patch=module.i._patch=patch;module.h._ncc=module.i._ncc=ncc
  yield counter
 finally:
  cv2.matchTemplate=old_match;module.h._patch,module.i._patch,module.h._ncc,module.i._ncc=originals
records=[]
for index in [0,4,5]:
 name,images,region=local_cases()[index];region['tracks']=region['tracks'][:1]
 expected=encode(candidate.filter_pair(images,region));item=dict(case=name)
 for label,module,kwargs in [('before_default',before,{}),('before_explicit_original',before,dict(filters=(filter_h,filter_i))),('after_explicit_original',candidate,dict(filters=(filter_h,filter_i)))]:
  with counts(module) as counter:
   value=module.filter_pair(images,region,**kwargs)
  assert encode(value)==expected
  item[label]=counter
 assert item['before_default']==item['before_explicit_original']
 assert item['before_default']['match']==item['after_explicit_original']['match']
 if index in [0,4]:
  assert item['before_default']==dict(match=4,patch=4,ncc=2)
  assert item['after_explicit_original']==dict(match=4,patch=1 if index==0 else 2,ncc=1)
 records.append(item)
with (r/'PREFLIGHT.json').open('x') as stream:json.dump(dict(exact_old_bodies=True,explicit_original_filters_keep_paired_path=True,records=records),stream,indent=2)
# No production/test candidate source evolved while correcting the tool.
freeze=json.loads((r/'FREEZE.json').read_bytes())
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==sha for p,sha in freeze['files'].items() if p.startswith('roadvision/'))
print(records,flush=True)
