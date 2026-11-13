import json
import pytest
from simulation.rgb_motion_checks_study import audit_study,predict_index


def test_phase_ordering_before_truth(tmp_path):
    with pytest.raises(FileNotFoundError,match='phase-complete'):
        audit_study(tmp_path/'generated',tmp_path/'baseline',tmp_path/'new',tmp_path/'audit.json')


@pytest.mark.parametrize('kind',['extra','duplicate','relative'])
def test_strict_public_index(tmp_path,kind):
    item={'sequence_id':'test','public_root':str(tmp_path)};rows=[item]
    if kind=='extra':item['truth_path']='secret'
    if kind=='duplicate':rows.append(item.copy())
    if kind=='relative':item['public_root']='relative'
    p=tmp_path/'index.json';p.write_text(json.dumps({'version':'rgb_observability_public_index_v1','sequences':rows}))
    with pytest.raises(ValueError):predict_index(p,tmp_path/'out')
    assert not (tmp_path/'out').exists()
