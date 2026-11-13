"""Exposed RGB regression fixtures, NOT independent safety/acceptance data."""
from pathlib import Path
import hashlib
import json
import numpy as np
from PIL import Image
import pytest
from roadvision.straight_permission import assess_startup_texture
from roadvision.calibrated_ground_motion import GroundCalibration

ROOT=Path(__file__).parent/'fixtures/straight_startup'


@pytest.mark.parametrize('name,accepted',[('center_a',True),('repeated',False),('floor_paint',False)])
def test_reference_only_veto_on_known_regression_rgb(name,accepted):
    rgb=np.array(Image.open(ROOT/(name+'.png')))
    origin=next(x for x in json.loads((ROOT/'manifest.json').read_text()) if x['file']==name+'.png')
    assert hashlib.sha256(rgb.tobytes()).hexdigest()==origin['rgb_sha256']
    out=assess_startup_texture(rgb,GroundCalibration())
    assert out['accepted'] is accepted
    assert out['motion_authorized'] is False and out['certified_clearance'] is False
    assert len(out['cells'])==9


def test_flat_ground_hypothesis_is_unknown_not_free():
    out=assess_startup_texture(np.full((256,256,3),128,np.uint8),GroundCalibration())
    assert not out['accepted'] and out['rectified_supported_cells']==0
