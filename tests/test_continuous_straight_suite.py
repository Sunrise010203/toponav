import copy
import json
from pathlib import Path
import pytest
from simulation.continuous_straight_suite import load_suite
ROOT=Path(__file__).resolve().parents[1]


def candidate():return json.loads((ROOT/'configs/evaluation/continuous_straight_s2_v1.json').read_text())


def test_candidate_suite_dry_validation_never_generates_images(tmp_path):
    p=tmp_path/'suite.json';p.write_text(json.dumps(candidate()));s=load_suite(p)
    assert len(s['cases'])==20
    assert len(list(tmp_path.iterdir()))==1


@pytest.mark.parametrize('fault',['truth_field','duplicate_seed','no_motion_as_normal','unreachable_fault','no_initial_prior','missing_case','changed_gate'])
def test_suite_cannot_silently_bypass_denominators_or_input_contract(tmp_path,fault):
    s=candidate()
    if fault=='truth_field':s['cases'][0]['true_goal_pose']=[1,2,3]
    elif fault=='duplicate_seed':s['cases'][1]['seed']=s['cases'][0]['seed']
    elif fault=='no_motion_as_normal':s['cases'][-2]['expectation']='normal'
    elif fault=='unreachable_fault':s['cases'][12]['fault']['tick']=1000
    elif fault=='no_initial_prior':s['cases'][0]['operator_confirmed']=False
    elif fault=='missing_case':s['cases'].pop()
    else:s['normal_minimum_fraction']=.1
    p=tmp_path/'suite.json';p.write_text(json.dumps(s))
    with pytest.raises(ValueError):load_suite(p)


def junction_suite():
    cases=[]
    def add(kind,direction,layout='safety',rep=0,fault=None):
        c=dict(id=f'{kind}-{direction}-{layout}-{rep}',seed=1000+len(cases),expectation=kind,direction=direction,
               layout=layout,geometry=dict(width_m=1.8 if layout!='wide' else 2.,corner_x_m=2.5,exit_length_m=3.5),
               max_ticks=12 if kind=='cancel_stop' else 650,operator_confirmed=kind!='no_motion',fault=fault,
               blocked_branch=direction if kind in ('blocked','clear_after_wait') else
                              ('right' if direction=='left' else 'left') if kind=='alternative_blocked' else None,
               clear_blocker_tick=165 if kind=='clear_after_wait' else None)
        cases.append(c)
    for layout in ('base','wide'):
        for direction in ('left','right'):
            for rep in range(2):add('normal',direction,layout,rep)
    for direction in ('left','right'):
        for i,kind in enumerate(('missing','replay','stale','occlusion')):add('fault_stop',direction,rep=i,fault=dict(kind=kind,tick=175))
    for kind in ('blocked','clear_after_wait','alternative_blocked','no_motion','cancel_stop'):
        for direction in ('left','right'):add(kind,direction)
    return dict(version='t_junction_s4_v1',status='FROZEN_FOR_FIRST_GENERATION',scope='test only, not generated',cases=cases)


def test_t_suite_validation_is_generation_free(tmp_path):
    from simulation.continuous_straight_suite import load_junction_suite
    path=tmp_path/'suite.json';path.write_text(json.dumps(junction_suite()))
    assert len(load_junction_suite(path)['cases'])==26
    assert list(tmp_path.iterdir())==[path]


@pytest.mark.parametrize('mutation',['path','truth','seed','direction','denominator','block','prior','clear','budget','layouts','fault'])
def test_t_frozen_suite_rejects_weakened_or_incoherent_setup(tmp_path,mutation):
    from simulation.continuous_straight_suite import load_junction_suite
    suite=junction_suite();c=suite['cases'][0]
    if mutation=='path':c['id']='../../outside'
    elif mutation=='truth':c['true_pose']=[0,0]
    elif mutation=='seed':c['seed']=suite['cases'][1]['seed']
    elif mutation=='direction':c['direction']='straight'
    elif mutation=='denominator':suite['cases'].pop()
    elif mutation=='block':c['blocked_branch']='left'
    elif mutation=='prior':c['operator_confirmed']=False
    elif mutation=='clear':c['clear_blocker_tick']=165
    elif mutation=='budget':c['max_ticks']=649
    elif mutation=='layouts':
        for case in suite['cases']:case['geometry']['width_m']=1.8
    elif mutation=='fault':suite['cases'][8]['fault']['kind']='unknown'
    path=tmp_path/'suite.json';path.write_text(json.dumps(suite))
    with pytest.raises(ValueError):load_junction_suite(path)


def route_suite():
    # Prospective fixture data; loading/validating this NEVER renders a world.
    return json.loads((ROOT/'reports/multi-node-20261016/suite.json').read_text())


def test_route_suite_validation_is_dry_and_multinode(tmp_path):
    from simulation.continuous_straight_suite import load_route_suite
    p=tmp_path/'suite.json';p.write_text(json.dumps(route_suite()))
    s=load_route_suite(p)
    assert len(s['cases'])==24 and list(tmp_path.iterdir())==[p]


@pytest.mark.parametrize('mutation',['path','truth','seed','denominator','prior','budget','one_edge','missing_layout','fault','transport','wrong_start','wrong_heading','graph_geometry'])
def test_route_suite_cannot_weaken_contract_or_denominators(tmp_path,mutation):
    from simulation.continuous_straight_suite import load_route_suite
    s=route_suite();c=s['cases'][0]
    if mutation=='path':c['id']='../escape'
    elif mutation=='truth':c['actual_pose']=[0,0]
    elif mutation=='seed':c['seed']=s['cases'][1]['seed']
    elif mutation=='denominator':s['cases'].pop()
    elif mutation=='prior':c['operator_confirmed']=False
    elif mutation=='budget':c['max_ticks']=2099
    elif mutation=='one_edge':c['goal_node']='A'
    elif mutation=='missing_layout':
        for x in s['cases']:x['layout']='same'
    elif mutation=='fault':next(x for x in s['cases'] if x['expectation']=='camera')['fault']['tick']=1
    elif mutation=='transport':next(x for x in s['cases'] if x['expectation']=='transport')['transport_fault']['segment']=1
    elif mutation=='wrong_start':next(x for x in s['cases'] if x['expectation']=='wrong_start')['physical_start_node']='T'
    elif mutation=='wrong_heading':
        x=next(x for x in s['cases'] if x['expectation']=='wrong_heading');x['physical_heading_deg']=x['heading_deg']
    else:c['graph']['geometry']={'node_positions_m':{}}
    p=tmp_path/'suite.json';p.write_text(json.dumps(s))
    with pytest.raises(ValueError):load_route_suite(p)


def safety_suite():
    return json.loads((ROOT/'reports/control-safety-20261018/suite.json').read_text())


def test_s6_suite_dry_validation_never_builds_world_or_generates_rgb(tmp_path):
    from simulation.continuous_straight_suite import load_safety_suite
    p=tmp_path/'suite.json';p.write_text(json.dumps(safety_suite()))
    assert len(load_safety_suite(p)['cases'])==67
    assert list(tmp_path.iterdir())==[p]


@pytest.mark.parametrize('mutation',['path','seed','case_count','normal_as_refusal','missing_mode','unknown_argument','no_latch','wrong_prior','camera_kind','camera_tick','fake_blocker','transport','budget','texture','uniform','truth_map'])
def test_s6_suite_rejects_weakened_coverage_and_unbound_setup(tmp_path,mutation):
    from simulation.continuous_straight_suite import load_safety_suite
    s=safety_suite();c=s['cases'][0];a=c['arguments']
    if mutation=='path':c['id']='../escape'
    elif mutation=='seed':a['seed']=s['cases'][1]['arguments']['seed']
    elif mutation=='case_count':s['cases'].pop()
    elif mutation=='normal_as_refusal':c['expectation']='no_prior'
    elif mutation=='missing_mode':c['mode']='route'
    elif mutation=='unknown_argument':a['hidden_truth']=[0,0]
    elif mutation=='no_latch':a['terminal_check_ticks']=0
    elif mutation=='wrong_prior':a['operator_confirmed']=False
    elif mutation=='camera_kind':next(x for x in s['cases'] if x['expectation']=='camera')['arguments']['fault']['kind']='unsupported'
    elif mutation=='camera_tick':next(x for x in s['cases'] if x['expectation']=='camera')['arguments']['fault']['tick']=0
    elif mutation=='fake_blocker':next(x for x in s['cases'] if x['id']=='straight-blocked')['arguments']['scene']['obstacle']='none'
    elif mutation=='transport':next(x for x in s['cases'] if x['expectation']=='transport')['arguments']['transport_fault']['segment']=1
    elif mutation=='budget':a['max_ticks']=5
    elif mutation=='texture':next(x for x in s['cases'] if x['expectation']=='texture_boundary')['arguments']['pattern']='unique'
    elif mutation=='uniform':next(x for x in s['cases'] if x['expectation']=='uniform_boundary')['arguments']['pattern']='unique'
    else:next(x for x in s['cases'] if x['mode']=='route')['arguments']['route_graph']['geometry']={'positions':[]}
    p=tmp_path/'suite.json';p.write_text(json.dumps(s))
    with pytest.raises(ValueError):load_safety_suite(p)
