"""Post-candidate counts only; instrumented duration is NOT a speed gate."""
import cProfile,json,pstats
from pathlib import Path
import cv2
from tools.straight_pipeline_profile import load_case,controller_for_task,decision_call,verify_freeze
r=Path('reports/straight-paired-patches-20261103')
freeze=json.loads((r/'FREEZE.json').read_bytes());verify_freeze(Path.cwd(),freeze)
case=load_case('reports/straight-evidence-recovery-20261103/comparison/known_center--gradient_transfer')
controller=controller_for_task(case['task']);profiler=cProfile.Profile()
previous=cv2.getNumThreads();cv2.setNumThreads(1)
try:
 for row,frame in zip(case['rows'],case['frames']):
  profiler.enable()
  try: value=decision_call(controller,row,frame)
  finally: profiler.disable()
  assert value==row['decision']
finally: cv2.setNumThreads(previous)
profiler.dump_stats(str(r/'after.prof'))
for sort in ['tottime','cumulative']:
 with (r/('after-'+sort+'.txt')).open('x') as stream:
  pstats.Stats(profiler,stream=stream).sort_stats(sort).print_stats(85)
verify_freeze(Path.cwd(),freeze)
counts=[]
for version in ['before','after']:
 stats=pstats.Stats(str(r/(version+'.prof')))
 counts.append(dict(version=version,functions=[dict(file=k[0],line=k[1],function=k[2],primitive_calls=v[0],calls=v[1]) for k,v in stats.stats.items() if k[2] in ('_patch','_ncc','_supported_patch','<matchTemplate>','<getRectSubPix>') or 'matchTemplate' in k[2] or 'getRectSubPix' in k[2]]))
(r/'PROFILE_COUNTS.json').write_text(json.dumps(counts,indent=2)+'\n')
print('after profile exact',len(case['rows']),flush=True)
