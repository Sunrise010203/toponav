import json
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from unittest.mock import Mock, patch

import pytest

from tools.visualize_simulation import PRESETS, Simulation, make_handler, read_jsonl


def test_partial_live_line_is_not_used(tmp_path):
    path = tmp_path / "live.jsonl"
    assert read_jsonl(path) == []
    path.write_bytes(b'{"tick": 1}\n{"tick":')
    assert read_jsonl(path) == [{"tick": 1}]


@pytest.mark.parametrize("preset,confirmed", [("left", False), ("left", 1), ("unknown", True)])
def test_fixture_must_be_confirmed(tmp_path, preset, confirmed):
    simulation = Simulation(tmp_path)
    with pytest.raises(ValueError):
        simulation.start(preset, confirmed)
    assert simulation.process is None


def test_worker_uses_original_runner_new_directory_and_no_shell(tmp_path):
    simulation = Simulation(tmp_path)
    process = Mock()
    process.poll.return_value = None
    with patch("tools.visualize_simulation.subprocess.Popen", return_value=process) as spawn:
        run_id = simulation.start("left", True)
        command = spawn.call_args.args[0]
        assert "simulation.continuous_straight_run" in command
        assert "--confirm-initial-clearance" in command
        assert "shell" not in spawn.call_args.kwargs
        assert (tmp_path / run_id / "worker.log").exists()
        with pytest.raises(RuntimeError):
            simulation.start("right", True)
        assert spawn.call_count == 1
    process.poll.return_value = 0
    simulation.close()


def test_assets_cannot_escape_current_run(tmp_path):
    simulation = Simulation(tmp_path)
    simulation.run_id = "current"
    simulation.directory = tmp_path / "current"
    assert simulation.frame_path("previous", "0000.png") is None
    assert simulation.frame_path("current", "../../README.png") is None
    assert simulation.frame_path("current", "result.json") is None
    assert simulation.frame_path("current", "0000.png").name == "0000.png"


def test_empty_snapshot_has_no_made_up_result(tmp_path):
    state = Simulation(tmp_path).snapshot()
    assert state["poses"] == state["decisions"] == []
    assert not state["running"] and state["run_id"] is None


def test_presets_are_explicit_and_finite():
    assert set(PRESETS) == {"left", "right", "t-left", "t-right", "straight", "blocked"}
    assert all("--max-ticks" in args for args in PRESETS.values())


def test_http_ui_and_cross_origin_guard(tmp_path):
    simulation = Simulation(tmp_path)
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(simulation))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with urllib.request.urlopen(base) as response:
            assert "实际车载相机" in response.read().decode()
        with urllib.request.urlopen(base + "/api/state") as response:
            assert json.load(response)["run_id"] is None
        request = urllib.request.Request(base + "/api/run", data=b'{}',
                    headers={"Origin": "https://other.invalid", "Content-Type": "application/json"})
        with pytest.raises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(request)
        assert error.value.code == 403
        assert simulation.process is None
        request = urllib.request.Request(base + "/api/run", data=b'[]',
                                         headers={"Content-Type": "application/json"})
        with pytest.raises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(request)
        assert error.value.code == 400
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
