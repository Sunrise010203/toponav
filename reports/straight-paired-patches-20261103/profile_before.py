import cProfile,hashlib,json,pstats
from pathlib import Path
import cv2
from tools.straight_pipeline_profile import load_case,controller_for_task,decision_call
r=Path('reports/straight-paired-patches-20261103')
case=load_case('reports/straight-evidence-recovery-20261103/comparison/known_center--gradient_transfer')
files=case['files']|set(Path('roadvision').glob('*.py'))|{Path(__file__)}
pins={str(p.resolve().relative_to(Path.cwd())):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
(r/'PROFILE_BEFORE_PINS.json').write_text(json.dumps(pins,indent=2)+'\n')
controller=controller_for_task(case['task']); profiler=cProfile.Profile()
previous=cv2.getNumThreads();cv2.setNumThreads(1)
try:
 for row,frame in zip(case['rows'],case['frames']):
  profiler.enable()
  try: value=decision_call(controller,row,frame)
  finally: profiler.disable()
  assert value==row['decision']
finally: cv2.setNumThreads(previous)
profiler.dump_stats(str(r/'before.prof'))
for sort in ['tottime','cumulative']:
 with (r/('before-'+sort+'.txt')).open('x') as stream:
  pstats.Stats(profiler,stream=stream).sort_stats(sort).print_stats(85)
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in pins.items())
print('before current profile exact',len(case['rows']),flush=True)
