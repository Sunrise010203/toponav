"""Sensor-metadata gates and fail-closed callbacks; synthetic inputs, not navigation proof."""
import json
import threading
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest

from simulation import gazebo_cross_validation as runner
from tests.test_review_gazebo_failure_capture import harness
from tests.test_review_gazebo_post_stop import terminal_io
from tests.test_review_gazebo_watchdog import client

IO = runner.GazeboIO
DESCRIPTOR = NS(fields_by_name={'pixel_format_type': NS(enum_type=NS(values_by_number={
    0:NS(name='UNKNOWN_PIXEL_FORMAT'), 1:NS(name='L_INT8'), 3:NS(name='RGB_INT8')}))})


class Message(NS):
    @property
    def data(self):
        self.reads += 1
        if self.poison: raise AssertionError('payload must not be fetched')
        return self.payload


def message(size=128, **changes):
    data=dict(DESCRIPTOR=DESCRIPTOR, width=size, height=size, step=size*3,
              pixel_format_type=3, payload=bytes(size*size*3), reads=0, poison=False,
              header=NS(stamp=NS(sec=1, nsec=2)))
    data.update(changes);return Message(**data)


def adapter():
    io=IO.__new__(IO);io.condition=threading.Condition();io.cameras=runner.CameraBuffer(('up','down'))
    io.overview=None;io.error=None;io.now_ns=2_000_000_000;io.command_context=None
    io.receiver_evidence=Mock();io.actuator=Mock();return io


def deliver(io, kind, msg):
    return io.overview_image(msg) if kind=='overview' else io.image(kind,msg)


@pytest.mark.parametrize('size', [1,128,256,384])
def test_generic_decoder_is_bounded_exact_uint8_and_owns_copy(size):
    msg=message(size);pixels=IO.pixels(msg)
    assert (runner.MAX_GAZEBO_IMAGE_WIDTH,runner.MAX_GAZEBO_IMAGE_HEIGHT)==(768,448)
    assert pixels.shape==(size,size,3) and pixels.dtype==np.uint8 and pixels.flags.owndata
    assert msg.reads==1 and pixels.nbytes==size*size*3<=1032192
    pixels[0,0,0]=7;assert msg.payload[0]==0


@pytest.mark.parametrize('view,size', [(v,128) for v in runner.VIEWS]+[('up',256),('down',256),('overview',384)])
def test_declared_sensor_pixels_and_original_stamp_retained_unchanged(view,size):
    io=adapter();msg=message(size,payload=np.random.default_rng(1).integers(0,256,size*size*3,np.uint8).tobytes())
    deliver(io,view,msg)
    stamp,pixels=io.overview if view=='overview' else next(iter(io.cameras.views[view].items()))
    assert stamp==1_000_000_002 and pixels.tobytes()==msg.payload and msg.reads==1
    assert io.error is None


@pytest.mark.parametrize('axis', ['width','height'])
@pytest.mark.parametrize('value', [0,-1,'above_bound',1024,2**63,True,1.5,'128',None])
def test_invalid_dimensions_rejected_before_payload_enum_or_numpy(monkeypatch,axis,value):
    if value=='above_bound': value=(768 if axis=='width' else 448)+1
    msg=message(**{axis:value},poison=True,DESCRIPTOR=None)
    decode=Mock(side_effect=AssertionError('must not create a NumPy view'))
    monkeypatch.setattr(runner.np,'frombuffer',decode)
    with pytest.raises(ValueError,match='Unexpected camera size'):IO.pixels(msg)
    assert msg.reads==0;decode.assert_not_called()


@pytest.mark.parametrize('view,size', [('front',256),('left',384),('up',128),('down',384),('overview',128),('overview',512),('bogus',128)])
def test_per_topic_contract_rejects_otherwise_packed_sizes_before_payload(monkeypatch,view,size):
    io=adapter();msg=message(size,poison=True)
    decode=Mock(side_effect=AssertionError('must not decode wrong-sized topic'))
    monkeypatch.setattr(runner.np,'frombuffer',decode)
    deliver(io,view,msg)
    assert io.error==('Unknown camera view' if view=='bogus' else 'Unexpected camera size')
    assert msg.reads==0 and io.overview is None
    assert all(not frames for frames in io.cameras.views.values());decode.assert_not_called()


@pytest.mark.parametrize('changes', [dict(pixel_format_type=0),dict(pixel_format_type=1),dict(pixel_format_type=999),dict(step=True),dict(step=0),dict(step=385)])
def test_format_and_stride_rejected_before_fetching_payload(monkeypatch,changes):
    msg=message(**changes,poison=True);decode=Mock()
    monkeypatch.setattr(runner.np,'frombuffer',decode)
    with pytest.raises(ValueError,match='packed RGB_INT8'):IO.pixels(msg)
    assert msg.reads==0;decode.assert_not_called()


@pytest.mark.parametrize('delta', [-1,1,1_000_000])
def test_payload_exact_length_checked_before_numpy(monkeypatch,delta):
    msg=message(payload=bytes(128*128*3+delta));decode=Mock()
    monkeypatch.setattr(runner.np,'frombuffer',decode)
    with pytest.raises(ValueError,match='packed RGB_INT8'):IO.pixels(msg)
    assert msg.reads==1;decode.assert_not_called()


@pytest.mark.parametrize('kind', ['front','up','overview'])
@pytest.mark.parametrize('prior', [None,'earlier I/O fault'])
def test_callback_contains_fault_preserves_first_error_and_never_recovers_authority(kind,prior):
    io=adapter();io.error=prior;deliver(io,kind,message(512))
    assert io.error==(prior or 'Unexpected camera size')
    deliver(io,kind,message(384 if kind=='overview' else 256 if kind=='up' else 128))
    assert io.error==(prior or 'Unexpected camera size')
    with pytest.raises(RuntimeError,match=prior or 'Unexpected camera size'):io.next_frame(-1,timeout=0.)


def test_bad_overview_invalidates_old_observer_image_without_touching_driver_buffer():
    io=adapter();io.image('front',message());before=dict(io.cameras.views['front'])
    io.overview_image(message(384));assert io.overview is not None
    io.overview_image(message(384,step=1))
    assert io.overview is None and io.error=='Gazebo camera must provide packed RGB_INT8'
    assert io.cameras.views['front']==before


@pytest.mark.parametrize('kind', ['front','overview'])
def test_failed_callback_notifies_waiters_without_propagating_thread_exception(kind):
    io=adapter();ready=threading.Event();out=[]
    def wait():
        with io.condition:
            ready.set();out.append(io.condition.wait_for(lambda:io.error is not None,timeout=2.))
    thread=threading.Thread(target=wait);thread.start();assert ready.wait(2.)
    deliver(io,kind,message(512));thread.join(2.)
    assert not thread.is_alive() and out==[True]


@pytest.mark.parametrize('kind', ['front','overview'])
def test_known_image_fault_blocks_nonzero_before_feedback_or_actuator_and_keeps_zero(kind):
    io=adapter();deliver(io,kind,message(512));io.command_context=dict(command={'v_m_s':.1,'omega_rad_s':0.})
    with pytest.raises(RuntimeError,match='I/O callback failure'):io.command(.1,0.)
    io.receiver_evidence.require_fresh.assert_not_called();io.actuator.request.assert_not_called()
    assert io.command_context is None
    io.command(0.,0.);io.actuator.request.assert_called_once_with('stop')


def test_real_independent_worker_still_acknowledges_frame_free_zero_after_image_fault(client):
    io=adapter();io.actuator=client[0];io.overview_image(message(512))
    io.command(0.,0.)
    assert io.actuator.status['latched'] and io.actuator.status['reason']=='remote_stop'
    assert io.actuator.status['wheels']==[0.,0.] and not io.actuator.failed
    assert io.error=='Unexpected camera size'


@pytest.mark.parametrize('kind', ['front','overview'])
def test_late_callback_failure_denies_completion_but_preserves_true_stationary_fact(tmp_path,monkeypatch,harness,kind):
    terminal_io(monkeypatch,harness);original=runner.GazeboIO
    class Adapter(original):
        def step(self,iterations):
            super().step(iterations)
            if self.steps==16:
                self.error=None;self.overview=None
                if kind=='front':self.cameras=runner.CameraBuffer()
                deliver(self,kind,message(512))
        pixels=staticmethod(IO.pixels)
        image=IO.image
        overview_image=IO.overview_image
    monkeypatch.setattr(runner,'GazeboIO',Adapter)
    result=runner.run(tmp_path/'capture',scenario='left',max_ticks=1)
    assert not result['completed'] and 'I/O callback failure: Unexpected camera size' in result['exception']
    assert result['controller_result']['state']=='visual_stop' and result['audit']['normal_geometric_stop_pass']
    assert result['cleanup_stop']['steps_completed']==10 and result['cleanup_stop']['error'] is None
    assert result['audit']['stationary'] and result['audit']['post_stop_pose_pair_fresh']


def test_callback_summary_does_not_overwrite_primary_execution_exception(tmp_path,harness):
    _,fault,_=harness;fault.kind='perception';original=runner.GazeboIO
    class Adapter(original):
        def step(self,iterations):
            super().step(iterations)
            if self.steps>=7:self.error='later image error'
    runner.GazeboIO=Adapter
    result=runner.run(tmp_path/'capture',scenario='left',max_ticks=3)
    assert 'backend failed' in result['exception'] and 'later image error' not in result['exception']
    assert not result['completed']


def test_no_callback_fault_leaves_original_terminal_contract_possible(tmp_path,monkeypatch,harness):
    terminal_io(monkeypatch,harness)
    result=runner.run(tmp_path/'capture',scenario='left',max_ticks=1)
    assert result['completed'] and result['exception'] is None and result['audit']['stationary']


def test_generic_and_callback_accept_original_wide_observer_without_resizing():
    payload=np.random.default_rng(2).integers(0,256,768*448*3,np.uint8).tobytes()
    msg=message(width=768,height=448,step=2304,payload=payload)
    pixels=IO.pixels(msg)
    assert pixels.shape==(448,768,3) and pixels.nbytes==1032192 and pixels.tobytes()==payload
    io=adapter();io.overview_image(msg)
    assert io.error is None and io.overview[0]==1_000_000_002 and io.overview[1].tobytes()==payload


@pytest.mark.parametrize('width,height', [(448,768),(768,384),(384,448),(767,448)])
def test_overview_allows_only_the_two_original_shapes_before_payload(width,height):
    io=adapter();msg=message(width=width,height=height,step=width*3,poison=True)
    io.overview_image(msg)
    assert msg.reads==0 and io.error=='Unexpected camera size' and io.overview is None


def test_every_original_scenario_sensor_shape_fits_its_declared_callback_contract():
    import xml.etree.ElementTree as ET
    from simulation.gazebo_scenes import SCENARIOS,scenario_pair,build_scenario_world
    for scenario in SCENARIOS:
        graph,scene=scenario_pair(scenario)
        for camera in ET.fromstring(build_scenario_world(scenario,graph,scene)).findall('.//sensor[@type="camera"]'):
            topic=camera.findtext('topic');width=int(camera.findtext('camera/image/width'));height=int(camera.findtext('camera/image/height'))
            view=topic.rsplit('/',1)[1];io=adapter()
            msg=message(width=width,height=height,step=width*3,payload=bytes(width*height*3))
            deliver(io,view,msg)
            assert io.error is None,(scenario,view,width,height,io.error)
