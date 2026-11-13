import json
import pytest
from simulation.rgb_resampling_checks_study import audit_study,predict_index


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


@pytest.mark.parametrize('kind',['missing','duplicate'])
def test_phase_identity_set_validated_before_truth(tmp_path,kind):
    generated=tmp_path/'generated';generated.mkdir()
    (generated/'public-index.json').write_text(json.dumps({'sequences':[{'sequence_id':'test','public_root':str(tmp_path/'no_truth')}]}))
    for name in ('baseline','new'):
        p=tmp_path/name;p.mkdir()
        rows=[] if kind=='missing' else [{'sequence_id':'test','exit_code':0}]*2
        (p/'phase-complete.json').write_text(json.dumps({'sequences':rows}))
    with pytest.raises(ValueError,match='Incomplete'):
        audit_study(generated,tmp_path/'baseline',tmp_path/'new',tmp_path/'out.json')
