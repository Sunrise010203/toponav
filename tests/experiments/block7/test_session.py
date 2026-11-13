"""Mechanism-isolation fixtures; ideal observations are not RGB experiment evidence."""
import copy
import json
from pathlib import Path
import pytest
from simulation.navigation_session import NavigationSession, SessionConfig
from simulation.structural_localization import map_exits
from experiments.block7.session import ExperimentConfig, ExperimentSession
ROOT = Path(__file__).resolve().parents[3]

class Clock:
    def __init__(self): self.mono, self.wall = 0., 1_000_000_000_000
    def advance(self, seconds=.1): self.mono += seconds; self.wall += round(seconds*1e9)

def setup(method='M1', core=False):
    c=Clock(); graph=json.loads((ROOT/'configs/graph.json').read_text())
    cls=NavigationSession if core else ExperimentSession
    config=SessionConfig() if core else ExperimentConfig.for_method(method)
    return cls(graph,config=config,clock=lambda:c.mono,wall_ns=lambda:c.wall,session_id='fixture'),c,graph

def observation(g,node='N01',confidence=1.):
    exits=map_exits(g,{'node_id':node,'heading_deg':0})
    return {'relative_open':{d:d in exits for d in ('front','left','right','back')},'confidence':confidence}

def frame(s,c,g,index,node='N01',confidence=1.,**changes):
    c.advance(); payload={'frame_id':f'f{index}','captured_at_ns':c.wall,'observation':observation(g,node,confidence)}
    payload.update(changes); return s.observe(payload)

@pytest.mark.parametrize('method,k,recovery',[('M0',1,False),('M1',3,True),('M2',1,True),('M3',3,False)])
def test_explicit_factorial(method,k,recovery):
    cfg=ExperimentConfig.for_method(method)
    assert (cfg.confirmation_frames,cfg.passive_recovery)==(k,recovery)
    s,c,g=setup(method)
    for i in range(k): out=frame(s,c,g,i)
    assert out['command_id'] and s.metrics['commands_sent']==1

@pytest.mark.parametrize('bad',[0,-1,True,1.0,None])
def test_configuration_types_and_minimum(bad):
    with pytest.raises(ValueError): ExperimentConfig(confirmation_frames=bad)

def test_production_config_still_forbids_single_frame():
    with pytest.raises(ValueError): SessionConfig(confirmation_frames=1)
    assert SessionConfig().confirmation_frames==3

@pytest.mark.parametrize('method',['M0','M3'])
def test_no_recovery_terminal_cannot_resurrect_inside_observe(method):
    s,c,g=setup(method)
    out=frame(s,c,g,0,confidence=.2)
    assert out['state']=='failed' and out['action']=='stop'
    for i in range(1,7): assert frame(s,c,g,i)['state']=='failed'
    assert s.metrics['commands_sent']==0

@pytest.mark.parametrize('method',['M1','M2'])
def test_recovery_enabled(method):
    s,c,g=setup(method); frame(s,c,g,0,confidence=.2)
    for i in range(1,4): out=frame(s,c,g,i)
    assert s.metrics['commands_sent']==1

@pytest.mark.parametrize('method',['M0','M1','M2','M3'])
def test_feedback_idempotent_unknown_never_unlocked_by_frames(method):
    s,c,g=setup(method)
    for i in range(s.config.confirmation_frames): out=frame(s,c,g,i)
    cmd=out['command_id']; event={'event_id':'sent','command_id':cmd,'feedback':{'status':'unknown'}}
    assert s.feedback(event)['state']=='waiting_execution'
    for i in range(5,10): assert frame(s,c,g,i,node='N02')['action']=='stop'
    assert s.metrics['commands_sent']==1 and s.metrics['transitions_applied']==0
    done={'event_id':'ok','command_id':cmd,'feedback':{'status':'succeeded'}}
    s.feedback(done); s.feedback(done); s.feedback({**done,'event_id':'ok-again'})
    assert s.metrics['transitions_applied']==1

@pytest.mark.parametrize('reason',['stale','future','duplicate','out_of_order'])
@pytest.mark.parametrize('method',['M0','M1','M2','M3'])
def test_metadata_guards_shared(method,reason):
    s,c,g=setup(method)
    # Initial invalid metadata must never issue a command, including M0/M2.
    changes={'captured_at_ns':c.wall-2_000_000_000} if reason=='stale' else {'captured_at_ns':c.wall+9_000_000_000}
    if reason in ('duplicate','out_of_order'):
        changes={'captured_at_ns':s.epoch_wall_ns}
    out=frame(s,c,g,0,**changes)
    assert out['action']=='stop' and s.metrics['commands_sent']==0

def test_M1_exact_core_public_replay_equivalence():
    a,ca,g=setup(core=True); b,cb,_=setup()
    for i,confidence in enumerate([.2,1.,1.,1.]):
        assert frame(a,ca,g,i,confidence=confidence)==frame(b,cb,g,i,confidence=confidence)
    cmd=a.pending['command_id']
    for status,eid in [('unknown','u'),('succeeded','s'),('succeeded','replay')]:
        event={'event_id':eid,'command_id':cmd,'feedback':{'status':status}}
        assert a.feedback(event)==b.feedback(event)
    for i in range(4,7): assert frame(a,ca,g,i,node='N02')==frame(b,cb,g,i,node='N02')
    ra,rb=a.report(),b.report(); rb['config'].pop('passive_recovery')
    assert ra==rb


@pytest.mark.parametrize('method',['M0','M1','M2','M3'])
def test_actual_replayed_frame_never_issues_a_second_command(method):
    s,c,g=setup(method)
    first=frame(s,c,g,0)
    repeated={'frame_id':'f0','captured_at_ns':c.wall,'observation':observation(g)}
    c.advance();out=s.observe(repeated)
    assert out['action']=='stop' and s.metrics['commands_sent'] <= 1
    assert s.metrics['duplicate_frames']==1
    if method in {'M0','M3'}: assert s.state=='failed'


def test_inconsistent_structure_unwinds_no_recovery_before_parent_can_resume():
    s,c,g=setup('M3');frame(s,c,g,0)
    bad=observation(g);bad['relative_open']['front']=not bad['relative_open']['front']
    out=frame(s,c,g,1,observation=bad)
    assert out['state']=='failed' and s.metrics['commands_sent']==0
    assert frame(s,c,g,2)['state']=='failed'


@pytest.mark.parametrize('method',['M0','M3'])
def test_trusted_failed_execution_terminates_without_retry(method):
    s,c,g=setup(method)
    for i in range(s.config.confirmation_frames):out=frame(s,c,g,i)
    event={'event_id':'failed','command_id':out['command_id'],'feedback':{'status':'failed'}}
    assert s.feedback(event)['state']=='failed'
    assert s.metrics['transitions_applied']==0
    for i in range(10,15):assert frame(s,c,g,i)['action']=='stop'


@pytest.mark.parametrize('method',['M0','M3'])
def test_ordinary_tick_is_not_scheduled_camera_loss(method):
    s,c,g=setup(method)
    assert s.tick()['state']!='failed'
    assert s.missing_capture()['state']=='failed'
