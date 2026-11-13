"""Timing observation transparency; no native sensors or movement here."""
import json
from pathlib import Path

import numpy as np
import pytest

from tools.capture_timing_probe import CaptureTimingProbe,CAPACITY,rgb_digest
from tools.capture_timing_isolation import (isolated_io,message,deliver,Echo,synthetic_case,SYNTHETIC_CASES)


def pixels():
    return np.random.default_rng(167).integers(0,256,(256,256,3),dtype=np.uint8)


@pytest.mark.parametrize('case',SYNTHETIC_CASES)
def test_actual_isolated_callback_cache_selector_cases(case):
    trace=synthetic_case(case)
    assert trace['observer_errors']==0 and trace['actual_exposure_wall_s'] is None
    assert not trace['clock_mapping_verified']
    if case!='malformed':
        for row in trace['histories']['callbacks']:
            assert row['callback_entry_s']<=row['lock_acquired_s']<=row['decode_started_s']<=row['decoded_s']<=row['cached_observed_s']<=row['lock_released_s']<=row['callback_finished_s']
        selected=trace['histories']['selections'][-1]
        computation=trace['histories']['computations'][-1]
        assert selected['all_view_bindings_verified'] and computation['selection_binding_verified']
        assert selected['selected_observed_s']<=computation['compute_started_s']<=computation['compute_finished_s']
    else:
        assert trace['histories']['callbacks'][0]['io_error_present']
        assert trace['counts']['selections']==trace['counts']['computations']==0


def test_original_arrays_results_and_methods_are_not_replaced_or_mutated():
    rgb=pixels();before=rgb.copy();io=isolated_io();echo=Echo();io.now_ns=100
    original={id(obj):obj.__dict__.copy() for obj in (io,io.cameras,echo)}
    p=CaptureTimingProbe('isolated_synthetic_delivery')
    with p.attach(io,echo):
        deliver(io,rgb,100);_,selected,now=io.next_frame(-1)
        frame=dict(frame_id='x',captured_sim_s=100/1e9,rgb=selected)
        assert echo.update(frame,now) is frame
        assert np.array_equal(rgb,before) and np.array_equal(selected,before)
    assert io.condition is original[id(io)]['condition']
    for obj,names in ((io,('image','pixels','next_frame')),(io.cameras,('put','latest')),(echo,('update',))):
        for name in names:assert name not in obj.__dict__
    assert not p.attached


def test_forward_existing_instance_overrides_and_restore_them_on_exception():
    io=isolated_io();echo=Echo();sentinel=RuntimeError('original')
    def update(frame,now):raise sentinel
    echo.update=update
    p=CaptureTimingProbe('replay_delivery')
    with pytest.raises(RuntimeError) as raised:
        with p.attach(io,echo):echo.update(None,0.)
    assert raised.value is sentinel and echo.update is update and not p.attached
    assert p.snapshot()['histories']['computations'][0]['outcome']=='raised'


def test_observer_clock_failure_does_not_prevent_original_delivery_or_update():
    def bad_clock():raise ValueError('observer failure')
    io=isolated_io();echo=Echo();io.now_ns=100
    p=CaptureTimingProbe('replay_delivery',clock=bad_clock)
    with p.attach(io,echo):
        deliver(io,pixels(),100);_,rgb,now=io.next_frame(-1)
        f=dict(frame_id='x',captured_sim_s=100/1e9,rgb=rgb)
        assert echo.update(f,now) is f and io.error is None
    assert p.snapshot()['observer_errors']>0


def test_inplace_cache_mutation_is_not_blessed_by_stamp_or_object_identity():
    io=isolated_io();echo=Echo();io.now_ns=100;p=CaptureTimingProbe('replay_delivery')
    with p.attach(io,echo):
        deliver(io,pixels(),100)
        io.cameras.views['front'][100][0,0,0]^=1
        stamp,rgb,now=io.next_frame(-1)
        echo.update(dict(frame_id='x',captured_sim_s=stamp/1e9,rgb=rgb),now)
    assert not p.snapshot()['histories']['selections'][-1]['all_view_bindings_verified']
    assert not p.snapshot()['histories']['computations'][-1]['selection_binding_verified']


def test_preexisting_unobserved_cache_is_unknown_not_reconstructed():
    io=isolated_io();echo=Echo();io.now_ns=100;deliver(io,pixels(),100)
    p=CaptureTimingProbe('replay_delivery')
    with p.attach(io,echo):
        stamp,rgb,now=io.next_frame(-1)
        echo.update(dict(frame_id='x',captured_sim_s=stamp/1e9,rgb=rgb),now)
    assert p.snapshot()['counts']['callbacks']==0
    assert not p.snapshot()['histories']['computations'][-1]['selection_binding_verified']


@pytest.mark.parametrize('change',['stamp','pixels','auxiliary','missing_auxiliary'])
def test_selected_pixels_and_sim_stamp_and_auxiliary_must_match(change):
    io=isolated_io(('up','down'));echo=Echo();io.now_ns=100;p=CaptureTimingProbe('replay_delivery')
    with p.attach(io,echo):
        deliver(io,pixels(),100);stamp,rgb,now=io.next_frame(-1)
        f=dict(frame_id='x',captured_sim_s=stamp/1e9,rgb=rgb,
            up_rgb=io.cameras.views['up'][stamp],down_rgb=io.cameras.views['down'][stamp])
        if change=='stamp':f['captured_sim_s']+=.1
        elif change=='pixels':f['rgb']=rgb.copy();f['rgb'][0,0,0]^=1
        elif change=='auxiliary':f['up_rgb']=f['up_rgb'].copy();f['up_rgb'][0,0,0]^=1
        else:del f['up_rgb']
        assert echo.update(f,now) is f
    assert not p.snapshot()['histories']['computations'][-1]['selection_binding_verified']


def test_fixed_recent_history_drops_are_explicit_and_ledger_tracks_original_cache():
    io=isolated_io();echo=Echo();p=CaptureTimingProbe('replay_delivery');rgb=pixels()
    with p.attach(io,echo):
        for stamp in range(1,20):deliver(io,rgb,stamp)
    s=p.snapshot()
    assert s['counts']['callbacks']==76 and len(s['histories']['callbacks'])==CAPACITY
    assert s['dropped']['callbacks']==12 and s['ledger_slots']==32
    assert all(len(v)==8 for v in io.cameras.views.values())
    s['histories']['callbacks'].clear()
    assert len(p.snapshot()['histories']['callbacks'])==CAPACITY


def test_reattach_clears_metadata_not_observed_in_new_scope():
    p=CaptureTimingProbe('replay_delivery');io=isolated_io();echo=Echo();io.now_ns=100
    with p.attach(io,echo):deliver(io,pixels(),100)
    with p.attach(io,echo):
        io.next_frame(-1)
    assert not p.snapshot()['histories']['selections'][-1]['all_view_bindings_verified']


def test_double_attach_is_refused_and_outer_originals_restored():
    io=isolated_io();echo=Echo();original=io.condition;p=CaptureTimingProbe('replay_delivery')
    with p.attach(io,echo):
        with pytest.raises(ValueError):
            with CaptureTimingProbe('replay_delivery').attach(io,echo):pass
    assert io.condition is original


@pytest.mark.parametrize('kind',['','exposure','sim_capture','receipt_is_exposure'])
def test_must_declare_delivery_provenance_not_exposure(kind):
    with pytest.raises(ValueError):CaptureTimingProbe(kind)


@pytest.mark.parametrize('rgb',[np.zeros((1024,1024,3),np.uint8),np.zeros((256,256,3),np.float64),None])
def test_diagnostic_hashing_does_not_expand_input_work_bounds(rgb):
    with pytest.raises(ValueError):rgb_digest(rgb)


def test_production_never_imports_the_opt_in_probe_or_driver():
    root=Path(__file__).resolve().parents[1]
    for folder in ('simulation','roadvision','ros2_ws/src/toponav_bridge/toponav_bridge'):
        for path in (root/folder).glob('*.py'):
            source=path.read_text()
            assert 'capture_timing_probe' not in source and 'capture_timing_isolation' not in source


def test_snapshot_is_json_and_exposure_cannot_be_inferred_from_sim_stamp():
    s=synthetic_case('four');wire=json.dumps(s,allow_nan=False)
    assert json.loads(wire)['actual_exposure_wall_s'] is None
    assert not s['clock_mapping_verified']
