"""Phase isolation and frozen-input plumbing, no diagnostic outcome tuning."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from simulation.rgb_observability_run import predict_index, verify_freeze, audit_experiment


def test_predictor_subprocess_gets_only_public_manifest_and_rgb_root(tmp_path, monkeypatch):
    public = tmp_path / "public"
    public.mkdir()
    index = tmp_path / "index.json"
    index.write_text(json.dumps({"version": "rgb_observability_public_index_v1", "sequences": [
        {"sequence_id": "sample", "public_root": str(public)}]}))
    calls = []
    def run(command, **kwargs):
        calls.append((command, kwargs))
        assert command[1:3] == ["-m", "roadvision.corridor_rgb_probe_run"]
        assert command[command.index("--manifest") + 1] == str(public / "sequence.json")
        assert command[command.index("--rgb-root") + 1] == str(public)
        assert not any("audit" in argument for argument in command)
        return SimpleNamespace(returncode=2, stdout="", stderr="Unsupported calibration")
    monkeypatch.setattr("simulation.rgb_observability_run.subprocess.run", run)
    output = tmp_path / "predictions"
    result = predict_index(index, output)
    assert len(calls) == 1 and result["sequences"][0]["exit_code"] == 2
    assert (output / "phase-complete.json").exists()
    assert (output / "sample/stderr.log").read_text() == "Unsupported calibration"
    with pytest.raises(FileExistsError):
        predict_index(index, output)


@pytest.mark.parametrize("change", [{"audit": {}}, {"sequences": [{"sequence_id": "../bad", "public_root": "/tmp"}]},
                                    {"sequences": [{"sequence_id": "x", "public_root": "/tmp", "pose": 1}]}])
def test_predict_index_rejects_extra_truth_and_path_id(change, tmp_path):
    data = {"version": "rgb_observability_public_index_v1", "sequences": []}
    data.update(change)
    path = tmp_path / "index.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        predict_index(path, tmp_path / "out")


def test_audit_cannot_run_before_prediction_phase(tmp_path):
    with pytest.raises(FileNotFoundError):
        audit_experiment(tmp_path, tmp_path / "predictions", tmp_path / "audit-result.json")


def test_freeze_rejects_changed_or_escaping_files(tmp_path):
    file = tmp_path / "source.py"
    file.write_text("value=1\n")
    lock = tmp_path / "freeze.json"
    lock.write_text(json.dumps({"files": {"source.py": hashlib.sha256(file.read_bytes()).hexdigest()}}))
    assert verify_freeze(lock, root=tmp_path)["files"]
    file.write_text("value=2\n")
    with pytest.raises(ValueError):
        verify_freeze(lock, root=tmp_path)
    lock.write_text(json.dumps({"files": {"../escape": "0" * 64}}))
    with pytest.raises(ValueError):
        verify_freeze(lock, root=tmp_path)
