import numpy as np
import pytest

from simulation.textured_corridor import TexturedCorridorWorld
from simulation.corridor_motion import CorridorConfig


def test_coarse_textures_change_pixels_not_initial_physics(tmp_path):
    with TexturedCorridorWorld(CorridorConfig(),assets=tmp_path/'first',seed=313) as world:
        a=world.capture()['rgb'];initial=world.audit_state()
        for _ in range(5):world.drive(.12,0);world.step_control()
        b=world.capture()['rgb'];final=world.audit_state()
        assert set(world.capture())=={'frame_id','captured_sim_s','rgb'}
    assert not np.array_equal(a,b)
    assert final['position_m'][0]>initial['position_m'][0]
    assert a[:128,:128].std()>10
    with TexturedCorridorWorld(CorridorConfig(),assets=tmp_path/'second',seed=313) as world:
        np.testing.assert_array_equal(a,world.capture()['rgb'])
        assert world.audit_state()['position_m']==initial['position_m']


def test_invalid_seed_and_existing_output_are_rejected(tmp_path):
    with pytest.raises(ValueError):TexturedCorridorWorld(assets=tmp_path/'bad',seed=True)
    with pytest.raises(FileExistsError):TexturedCorridorWorld(assets=tmp_path)


@pytest.mark.parametrize('pattern',['uniform','repeated'])
def test_counterexample_texture_patterns_are_explicit_and_rgb_only(tmp_path,pattern):
    with TexturedCorridorWorld(assets=tmp_path/pattern,pattern=pattern) as world:
        assert set(world.capture())=={'frame_id','captured_sim_s','rgb'}
        assert world.texture_pattern==pattern
