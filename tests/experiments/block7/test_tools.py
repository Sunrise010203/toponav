"""Static data, public replay, error bookkeeping and final-test lock tests."""
import copy
import json
from pathlib import Path
import signal
import subprocess

import pytest

from experiments.block7 import dataset, run
from experiments.block7.common import read_json, read_rows, write_json
from experiments.block7.replay import replay
from experiments.block7.runtime import development_task, episode
from experiments.block7.runtime import load_fixture, corrupt_observation
from experiments.block7.validate import ledger_summary, ratio, validate_suite, verify_files


@pytest.fixture(scope='module')
def suite(tmp_path_factory):
    target = tmp_path_factory.mktemp('static-suite') / 'definitions'
    dataset.prepare(target)
    return target


def test_static_suite_is_valid_and_counts_fixed(suite):
    result = validate_suite(suite)
    assert result['passed'], result
    assert (result['families'], result['base_tasks'], result['method_episodes'],
            result['perception_packages']) == (18, 1188, 4752, 2520)
    assert not list(suite.rglob('*.png'))


def test_exact_topology_isolation(suite):
    layouts = [read_json(p)['graph'] for p in (suite / 'layouts').glob('*.json')]
    historical, _ = dataset.historical_graphs()
    for index, graph in enumerate(layouts):
        current = dataset.nx_graph(graph)
        assert not any(dataset.same_family(current, old) for _, old in historical)
        assert not any(dataset.same_family(current, dataset.nx_graph(prior)) for prior in layouts[:index])


def test_registry_extension_is_restored_even_on_exception(suite):
    layout = read_json(next((suite / 'layouts').glob('*.json')))['layout']
    old = copy.deepcopy(dataset.LAYOUTS)
    with pytest.raises(RuntimeError):
        with dataset.registered(layout):
            assert layout['layout_id'] in dataset.LAYOUTS
            raise RuntimeError('fixture')
    assert dataset.LAYOUTS == old


def test_no_historical_registry_replacement(suite):
    layout = read_json(next((suite / 'layouts').glob('*.json')))['layout']
    layout['layout_id'] = 'test_locked'
    with pytest.raises(ValueError):
        with dataset.registered(layout): pass


def test_suite_tamper_detected_before_graph_use(suite, tmp_path):
    import shutil
    target = tmp_path / 'corrupt'; shutil.copytree(suite, target)
    (target / 'tasks.jsonl').write_text('{}\n')
    assert not validate_suite(target)['passed']


def test_renamed_mirrored_graphs_are_same_family():
    import networkx as nx
    g = nx.path_graph(7)
    assert dataset.same_family(g, nx.relabel_nodes(g, {i: f'other-{6-i}' for i in g}))


@pytest.mark.parametrize('a,b,expected', [(0,0,None), (0,5,0.), (3,5,.6), (5,5,1.)])
def test_rate_zero_denominator(a,b,expected): assert ratio(a,b) == expected


@pytest.mark.parametrize('a,b', [(True,1), (1,False), (-1,2), (3,2), (1.,2)])
def test_rate_invalid_counts(a,b):
    with pytest.raises(ValueError): ratio(a,b)


def test_denominator_includes_refusals_errors_and_not_run():
    ledger = {'episodes':[{'task_id':str(i),'method':'M1','physical_reachable':True,'status':s}
                          for i,s in enumerate(['success','refused','timeout','error','not_run'])]}
    result = ledger_summary(ledger)
    assert result['planned'] == 5 and result['normal_or_fault_reachable_success_rate'] == .2
    assert result['complete'] is False


def test_duplicate_ledger_rows_are_invalid():
    row = {'task_id':'same','method':'M1','status':'success'}
    with pytest.raises(ValueError): ledger_summary({'episodes':[row,row]})


class Clock:
    def __init__(self): self.mono, self.wall = 0., 1_000_000_000_000
    def refresh(self): self.mono += .05; self.wall += 50_000_000
    def snapshot(self): return {'monotonic_s':self.mono,'wall_ns':self.wall}


class Scripted:
    def __init__(self, env, task, output, clock): self.env, self.clock = env, clock
    def capture(self, index, blocked):
        self.clock.refresh()
        return ({'frame_id':str(index),'captured_at_ns':self.clock.wall,'observation':self.env.observe()},
                {'fixture':'ideal_observation_not_RGB'}, {'fixture':True})


@pytest.mark.parametrize('method', ['M0','M1','M2','M3'])
def test_shared_driver_replays_every_method(tmp_path, method):
    output = tmp_path / method
    result = episode(development_task(), method, output, sensor_factory=Scripted,
                     clock=Clock(), sleeper=lambda _:None, make_world=False)
    assert result['status'] == 'success'
    assert replay(output)['passed']
    if method == 'M1': assert replay(output, compare_core=True)['original_core_equivalence_passed']
    assert result['collision'] is None and result['braking'] is None
    assert read_json(output/'header.json')['sensor']=='scripted_protocol_fixture'


@pytest.mark.parametrize('method,expected',[('M0','refused'),('M1','success'),('M2','success'),('M3','refused')])
def test_drop_mechanism_and_replay(tmp_path,method,expected):
    out=tmp_path/method
    result=episode(development_task('drop_once'),method,out,sensor_factory=Scripted,
                   clock=Clock(),sleeper=lambda _:None,make_world=False)
    assert result['status']==expected and result['fault_triggered']
    assert replay(out)['passed']
    if method == 'M1': assert replay(out, compare_core=True)['original_core_equivalence_passed']


def test_unknown_execution_times_out_without_guessing_and_replays(tmp_path):
    out=tmp_path/'unknown'
    result=episode(development_task('execution_unknown'),'M1',out,sensor_factory=Scripted,
                   clock=Clock(),sleeper=lambda _:None,make_world=False,test_limits={'stage_timeout_s':.5})
    assert result['status']=='timeout' and result['commands']==1
    assert read_json(out/'controller.json')['metrics']['transitions_applied']==0
    assert replay(out)['passed']


def test_input_error_is_preserved_and_terminal_replay_matches(tmp_path):
    class Invalid(Scripted):
        def capture(self,index,blocked):
            frame,packet,audit=super().capture(index,blocked)
            frame['extra_field']=True
            return frame,packet,audit
    out=tmp_path/'invalid'
    result=episode(development_task(),'M1',out,sensor_factory=Invalid,
                   clock=Clock(),sleeper=lambda _:None,make_world=False)
    assert result['status']=='error' and result['error']['type']=='ValueError'
    assert any(row['exception'] for row in read_rows(out/'events.jsonl'))
    assert replay(out)['passed']


def test_output_is_exclusive_and_replay_tamper_fails(tmp_path):
    out=tmp_path/'exclusive'
    args=dict(task=development_task(),method='M1',output=out,sensor_factory=Scripted,
              clock=Clock(),sleeper=lambda _:None,make_world=False)
    episode(**args)
    with pytest.raises(FileExistsError): episode(**args)
    rows=read_rows(out/'events.jsonl'); rows[0]['output']['reason']='corrupt'
    (out/'events.jsonl').write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
    assert not replay(out)['passed']


def test_final_episode_locked_before_any_output(tmp_path):
    task=development_task();task['split']='test'
    with pytest.raises(ValueError):episode(task,'M1',tmp_path/'never-generated')
    assert not (tmp_path/'never-generated').exists()


def test_timeout_kills_only_own_child_process_group(tmp_path,monkeypatch):
    signals=[]
    class Process:
        pid=998811;returncode=-9
        def __init__(self,*a,**kw):self.calls=0; assert kw['start_new_session'] is True
        def wait(self,timeout=None):
            self.calls+=1
            if self.calls<=2:raise subprocess.TimeoutExpired('fixture',timeout)
            return self.returncode
    monkeypatch.setattr(run.subprocess,'Popen',Process)
    monkeypatch.setattr(run.os,'killpg',lambda pid,sig:signals.append((pid,sig)))
    code,timedout=run.safe_child(tmp_path/'request',tmp_path/'child',.001)
    assert code==-9 and timedout and signals==[(998811,signal.SIGTERM),(998811,signal.SIGKILL)]


def test_batch_error_preserves_all_planned_denominators(tmp_path,monkeypatch):
    monkeypatch.setattr(run,'safe_child',lambda *args:(1,False))
    out=tmp_path/'failed'
    result=run.batch(out)
    assert result['planned']==8 and result['counts']['error']==1 and result['counts']['not_run']==7
    assert not result['complete']


def test_manifest_rejects_path_escape(tmp_path):
    assert verify_files(tmp_path,{'files':[{'path':'../outside','sha256':'bad'}]})


def test_wrong_public_map_does_not_modify_physical_world(suite):
    task=next(t for t in read_rows(suite/'tasks.jsonl') if t['factor']=='wrong_map_edge')
    public,physical,scene,cells,layout=load_fixture(task,suite)
    assert len(public['edges'])==len(physical['edges'])-1
    assert len(scene['channels'])==len(physical['edges'])
    assert task['wrong_map_channel'] in {e['channel_id'] for e in physical['edges']}
    assert task['wrong_map_channel'] not in {e['channel_id'] for e in public['edges']}


def test_temporary_closure_releases_physical_edge_without_rewriting_public_map(suite,tmp_path):
    task=next(t for t in read_rows(suite/'tasks.jsonl') if t['factor']=='temporary_closure' and t['split']=='validation')
    out=tmp_path/'closure'
    result=episode(task,'M1',out,suite=suite,sensor_factory=Scripted,
                   clock=Clock(),sleeper=lambda _:None,make_world=False)
    assert result['status']=='success' and result['fault_triggered']
    rows=read_rows(out/'audit/trajectory.jsonl')
    assert all(task['fault']['channel'] in r['physical_disabled'] for r in rows[:3])
    assert all(task['fault']['channel'] not in r['physical_disabled'] for r in rows[3:])
    assert replay(out,compare_core=True)['passed']


@pytest.mark.parametrize('method',['M0','M1','M2','M3'])
def test_unreachable_is_not_success_or_invalid_input(suite,tmp_path,method):
    task=next(t for t in read_rows(suite/'tasks.jsonl') if t['factor']=='unreachable' and t['split']=='validation')
    result=episode(task,method,tmp_path/method,suite=suite,sensor_factory=Scripted,
                   clock=Clock(),sleeper=lambda _:None,make_world=False)
    assert result['status']=='refused' and not result['goal_verified'] and result['error'] is None


def test_perception_final_split_requires_explicit_gate_before_output(suite,tmp_path):
    from experiments.block7.perception import run as perception
    with pytest.raises(ValueError):perception(tmp_path/'locked',suite=suite,split='test')
    assert not (tmp_path/'locked').exists()


def test_registered_renderer_works_on_exposed_geometry_without_new_test_pixels():
    import numpy as np
    from experiments.block7.rendering import RenderedWorld
    from roadvision.scene import LAYOUTS
    historical=LAYOUTS['test_locked']
    alias={'layout_id':'b7-development-registry-alias','split':'development',
           'cells':historical['cells'],'edges':historical['edges']}
    with dataset.registered(alias), RenderedWorld(alias['layout_id']) as world:
        world.appearance_variant='material_albedo'
        image,audit=world.sample(15,np.random.default_rng(12))
        assert image.shape==(256,256,3) and all(audit['ray_open'])
    assert alias['layout_id'] not in LAYOUTS


def test_passed_boolean_alone_does_not_qualify_final_candidate(suite,tmp_path):
    from experiments.block7.validate import freeze_candidate
    receipt=tmp_path/'not-qualified.json';write_json(receipt,{'passed':True})
    with pytest.raises(ValueError):freeze_candidate(suite,tmp_path/'never-frozen.json',receipt)
    assert not (tmp_path/'never-frozen.json').exists()


def test_corruption_is_complete_even_if_original_prediction_unknown():
    original={'relative_open':dict.fromkeys(('front','left','right','back')),'confidence':0.}
    corrupted=corrupt_observation(original)
    assert corrupted['confidence']==.99
    assert all(type(value) is bool for value in corrupted['relative_open'].values())
    assert original['confidence']==0. and all(v is None for v in original['relative_open'].values())


def test_wrong_recognition_injection_has_separate_one_way_effect_audit(tmp_path):
    out=tmp_path/'wrong'
    result=episode(development_task('confident_wrong_observation_once'),'M1',out,sensor_factory=Scripted,
                   clock=Clock(),sleeper=lambda _:None,make_world=False)
    assert result['fault_triggered'] and result['fault_effect_verified'] is True
    assert replay(out,compare_core=True)['passed']


def test_stale_qualification_cannot_bind_new_or_changed_source(suite,tmp_path):
    from experiments.block7.common import file_hash
    from experiments.block7.validate import freeze_candidate
    source=tmp_path/'source.py';source.write_text('old')
    receipt=tmp_path/'stale.json'
    write_json(receipt,{'stage':'7B','passed':True,'production_core_unchanged':True,
                       'actual_RGB_smoke_passed':True,'public_replay_passed':True,
                       'test_pixels_not_generated':True,'regression_passed':True,
                       'suite_manifest_sha256':file_hash(suite/'dataset_manifest.json'),
                       'source_files':[{'path':str(source),'sha256':file_hash(source)}]})
    source.write_text('changed')
    with pytest.raises(ValueError):freeze_candidate(suite,tmp_path/'not-frozen.json',receipt)
