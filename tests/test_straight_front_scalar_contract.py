"""S2 producer scalar representation; numerical/permission gates unchanged."""
import copy

import numpy as np
import pytest

from roadvision import straight_evidence as producer
from roadvision.continuous_straight import _finite_vector
from tests.test_straight_evidence import packet


def produce(monkeypatch,interval):
    original=packet()
    monkeypatch.setattr(producer,'prepare_straight_history',lambda *a,**k:original)
    monkeypatch.setattr(producer,'estimate_paired_corridor',lambda *a,**k:dict(status='unknown'))
    monkeypatch.setattr(producer,'estimate_perpendicular_front',lambda *a,**k:dict(status='unknown'))
    monkeypatch.setattr(producer,'_packet',lambda *a,**k:dict(front_surface=dict(
        status='conditional_front_surface',range_interval_m=copy.deepcopy(interval),
        motion_authorized=False,arrival_status='unverified')))
    return producer.probe_straight_history([],now_s=.8,calibration_id='stub')['front_surface']


@pytest.mark.parametrize('dtype',[np.float16,np.float32,np.float64])
def test_numpy_float_intervals_become_equal_native_scalars(monkeypatch,dtype):
    bounds=[dtype(.9982514996105356),dtype(1.1158100356034688)]
    assert not _finite_vector(bounds,2)  # Reproduce the old in-memory mismatch.
    out=produce(monkeypatch,bounds)
    assert all(type(v) is float for v in out['range_interval_m'])
    assert all(a==b for a,b in zip(bounds,out['range_interval_m']))
    assert _finite_vector(out['range_interval_m'],2)
    assert out['motion_authorized'] is False and out['arrival_status']=='unverified'


@pytest.mark.parametrize('bad',[
    True,False,np.bool_(True),'1.0',None,complex(1.,0.),np.longdouble('1.0'),
    np.float64('nan'),np.float64('inf'),np.float64('-inf'),
])
def test_bad_values_cannot_be_coerced_into_supported_intervals(monkeypatch,bad):
    out=produce(monkeypatch,[bad,np.float64(1.2)])
    assert not _finite_vector(out['range_interval_m'],2)


@pytest.mark.parametrize('interval',[None,[],[np.float64(1.)],[np.float64(1.),2.,3.],{'lower':1.,'upper':2.}])
def test_malformed_interval_shape_is_not_repaired(monkeypatch,interval):
    out=produce(monkeypatch,interval)
    assert out['range_interval_m']==interval
    assert not _finite_vector(out['range_interval_m'],2)


def test_native_numbers_and_reverse_order_are_not_reinterpreted(monkeypatch):
    bounds=[1.2,1.]
    out=produce(monkeypatch,bounds)
    assert out['range_interval_m']==bounds
    # The unchanged controller separately requires 0 <= lower <= upper.
    assert out['range_interval_m'][0]>out['range_interval_m'][1]
