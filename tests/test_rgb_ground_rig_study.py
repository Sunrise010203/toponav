import json
from pathlib import Path
import pytest
from simulation.rgb_ground_rig_study import audit_study,predict_ground_index


def test_no_audit_before_all_predictions(tmp_path):
    # There need not even be a truth file: ordering check must fail first.
    for name in ('generated','baseline','prediction'): (tmp_path/name).mkdir()
    with pytest.raises(FileNotFoundError,match='phase-complete'):
        audit_study(tmp_path/'generated',tmp_path/'baseline',tmp_path/'prediction',tmp_path/'audit.json')


@pytest.mark.parametrize('kind',['extra','duplicate','relative'])
def test_index_whitelist_before_output_creation(tmp_path,kind):
    item={'sequence_id':'test','public_root':str(tmp_path)}
    rows=[item]
    if kind=='extra': item['truth_path']='secret'
    if kind=='duplicate': rows.append(item.copy())
    if kind=='relative': item['public_root']='relative/path'
    index=tmp_path/'index.json'
    index.write_text(json.dumps({'version':'rgb_observability_public_index_v1','sequences':rows}))
    with pytest.raises(ValueError): predict_ground_index(index,tmp_path/'out')
    assert not (tmp_path/'out').exists()


def test_audit_exact_known_plane_image_transfer():
    import numpy as np
    from simulation.rgb_ground_rig_study import audit_region
    from simulation.rgb_observability import plane_transfer
    surface={'name':'ground','point_world_m':[0,0,0],'normal_world':[0,0,1],
             'vertices_world_m':[[-10,-10,0],[10,-10,0],[10,10,0],[-10,10,0]]}
    captures=[dict(position_m=[i*.1,0,.45],yaw_rad=0,pitch_rad=0,rendered_fov_deg=90,surface_shift_m=[0,0,0]) for i in range(3)]
    truth={'captures':captures,'surfaces':[surface]}
    h=plane_transfer(surface,captures[0],captures[2],'front')
    tracks=[{'first':p} for p in [[20,90],[40,110],[90,100]]]
    diagnostic={'tracks':tracks,'models':{'homography':h.tolist(),'homography_inlier_mask':[True]*3}}
    r=audit_region(truth,0,'front',diagnostic)
    assert r['surface_counts']=={'ground':3}
    assert r['transfer_error_median_px']==pytest.approx(0.,abs=1e-9)
