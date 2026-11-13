"""Serial development comparisons, public replay and independent post-run audit."""
from pathlib import Path
import hashlib
import json
from simulation.continuous_straight_run import run_case
from simulation.continuous_straight_acceptance import replay_safety_public, audit_safety_physics
from simulation.corridor_motion import CorridorConfig

p=Path('reports/straight-evidence-recovery-20261103')
pins=json.loads((p/'CANDIDATE_PINS.json').read_bytes())
def verify():
 assert all(hashlib.sha256(Path(n).read_bytes()).hexdigest()==h for n,h in pins.items())
verify(); plan=json.loads((p/'CANDIDATE_PLAN.json').read_bytes());summary=[]
for case in plan['runs']:
 name=case['name'];args=case['arguments'].copy();scene=CorridorConfig(**args.pop('scene'))
 folder=p/'comparison'/name
 result=run_case(folder,scene=scene,**args)
 rows=[json.loads(x) for x in (folder/'decisions.jsonl').read_text().splitlines()]
 trace=json.loads((folder/'audit/physics-trace.json').read_bytes())
 audit=audit_safety_physics(result,rows,trace)
 replay=replay_safety_public(folder,result,rows)
 d=result['controller_result'];a=result['audit']
 record=dict(name=name,variant=args['straight_variant'],decisions=len(rows),result=d,
             displacement_m=a['displacement_m'],geometric_completed=a['normal_geometric_stop_pass'],
             safe_final_stop=a['safe_final_stop'],contacts=a['wall_contact_physics_steps'],
             exception=result['exception'],physics=audit,replay=replay,
             following_commands=sum(r['decision']['state']=='following' and bool(r['decision']['v_m_s']) for r in rows),
             selection_sizes=[r['decision']['experiment']['history_selection']['selected_size'] for r in rows
                              if r['decision'].get('experiment',{}).get('history_selection') is not None])
 summary.append(record)
 with (p/'comparison-progress.jsonl').open('a') as f:f.write(json.dumps(record,allow_nan=False)+'\n')
 print('COMPARED',name,'displacement',record['displacement_m'],'completed',record['geometric_completed'],'following',record['following_commands'],'physics_errors',audit['errors'],flush=True)
 verify()
verify()
with (p/'COMPARISON.json').open('x') as f:json.dump(summary,f,indent=2,allow_nan=False)
