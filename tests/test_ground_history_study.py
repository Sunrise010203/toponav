import json
import pytest
from simulation.ground_history_study import predict


@pytest.mark.parametrize('payload',[
    dict(version='wrong',sequences=[]),
    dict(version='rgb_observability_public_index_v1',sequences=[],truth='/hidden'),
    dict(version='rgb_observability_public_index_v1',sequences=[dict(sequence_id='ok',public_root='relative')]),
    dict(version='rgb_observability_public_index_v1',sequences=[dict(sequence_id='x',public_root='/tmp',pose=[0,0])]),
])
def test_study_index_rejects_nonpublic_fields_without_launching(tmp_path,payload,monkeypatch):
    def forbidden(*a,**k):raise AssertionError('Must reject before launching any child')
    monkeypatch.setattr('simulation.ground_history_study.subprocess.run',forbidden)
    manifest=tmp_path/'index.json';manifest.write_text(json.dumps(payload))
    with pytest.raises(ValueError):predict(manifest,tmp_path/'out')
