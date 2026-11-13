"""Local, read-only visualization of the existing real PyBullet runner.

The worker runs unchanged production code. The browser only reads its public
RGB, decisions and one-way physical audit; it never supplies pose to control.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]
PRESETS = {
    "left": ["--corner", "left", "--max-ticks", "650"],
    "right": ["--corner", "right", "--max-ticks", "650"],
    "t-left": ["--corner", "left", "--junction", "t", "--max-ticks", "900"],
    "t-right": ["--corner", "right", "--junction", "t", "--max-ticks", "900"],
    "straight": ["--length-m", "4", "--max-ticks", "400"],
    "blocked": ["--length-m", "4", "--obstacle", "full_width", "--max-ticks", "400"],
}


def read_jsonl(path):
    """A writer may be halfway through its last line; do not invent a row."""
    try:
        raw = path.read_bytes()
    except FileNotFoundError:
        return []
    rows = []
    for line in raw.splitlines(keepends=True):
        if not line.endswith(b"\n"):
            break
        rows.append(json.loads(line))
    return rows


class Simulation:
    def __init__(self, output_root):
        self.output_root = Path(output_root).resolve()
        self.output_root.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.process = None
        self.directory = None
        self.preset = None
        self.run_id = None
        self.log = None

    def start(self, preset, confirmed, motion_permission='no_motion'):
        if preset not in PRESETS or confirmed is not True:
            raise ValueError("请选择有效场景并确认仿真初始净空前提")
        if motion_permission not in ('no_motion','bounded_probe'):
            raise ValueError('无效的直走运动许可')
        if preset not in ('straight','blocked') and motion_permission!='no_motion':
            raise ValueError('受限探测许可只适用于直走/阻挡场景')
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                raise RuntimeError("当前模拟仍在运行，请等待结束")
            if self.log is not None:
                self.log.close()
            self.run_id = uuid.uuid4().hex
            self.directory = self.output_root / self.run_id
            self.directory.mkdir(exist_ok=False)
            self.preset = preset
            self.log = (self.directory / "worker.log").open("wb")
            env = os.environ.copy()
            env.update(PYTHONDONTWRITEBYTECODE="1", OMP_NUM_THREADS="2",
                       OPENBLAS_NUM_THREADS="2", MKL_NUM_THREADS="2")
            command = [sys.executable, "-u", "-m", "simulation.continuous_straight_run",
                       *PRESETS[preset], "--confirm-initial-clearance",
                       "--output", str(self.directory / "simulation")]
            if preset in ('straight','blocked'):
                command += ['--motion-permission',motion_permission]
            self.process = subprocess.Popen(command, cwd=ROOT, env=env,
                                            stdout=self.log, stderr=subprocess.STDOUT,
                                            start_new_session=True)
            return self.run_id

    def snapshot(self):
        with self.lock:
            if self.directory is None:
                return {"run_id": None, "running": False, "poses": [], "decisions": []}
            folder = self.directory / "simulation"
            poses = read_jsonl(folder / "audit/truth.live.jsonl")
            decisions = read_jsonl(folder / "decisions.live.jsonl")
            result_path = folder / "result.json"
            result = None
            if result_path.exists():
                try:
                    result = json.loads(result_path.read_text())
                except json.JSONDecodeError:
                    pass  # The final JSON is still being written.
            code = self.process.poll()
            if result:
                poses = [result["audit"]["initial"], *poses]
                final = result["audit"]["final"]
                if not poses or final["sim_s"] > poses[-1]["sim_s"]:
                    poses.append(final)
            return dict(run_id=self.run_id, preset=self.preset, running=code is None,
                        exit_code=code, directory=str(self.directory), poses=poses,
                        decisions=[dict(tick=r["tick"], sim_s=r["sim_s"],
                                        state=r["decision"]["state"],
                                        reason=r["decision"]["reason"],
                                        v=r["decision"]["v_m_s"],
                                        omega=r["decision"]["omega_rad_s"],
                                        rgb=r["input"]["rgb_path"] if r["input"] else None)
                                   for r in decisions],
                        result=None if result is None else dict(audit=result["audit"],
                            exception=result["exception"], controller=result["controller_result"]))

    def frame_path(self, run_id, filename):
        with self.lock:
            # Never expose arbitrary filesystem paths or another run's assets.
            if run_id != self.run_id or self.directory is None:
                return None
            if not filename.endswith(".png") or not filename[:-4].isdigit():
                return None
            return self.directory / "simulation/public/frames" / filename

    def close(self):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                os.killpg(self.process.pid, signal.SIGINT)
                try:
                    self.process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait()
            if self.log is not None:
                self.log.close()


def make_handler(simulation):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, status, data, mime="application/json; charset=utf-8"):
            if not isinstance(data, bytes):
                data = json.dumps(data, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/":
                return self.reply(200, (ROOT / "tools/simulation_viewer.html").read_bytes(),
                                  "text/html; charset=utf-8")
            if path == "/api/state":
                return self.reply(200, simulation.snapshot())
            parts = path.split("/")
            if len(parts) == 4 and parts[1] == "frames":
                frame = simulation.frame_path(parts[2], parts[3])
                if frame is not None and frame.is_file():
                    return self.reply(200, frame.read_bytes(), "image/png")
            self.reply(404, {"error": "未找到资源"})

        def do_POST(self):
            origin = self.headers.get("Origin")
            if origin and origin != "http://" + self.headers.get("Host", ""):
                return self.reply(403, {"error": "禁止跨站启动模拟"})
            if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                return self.reply(415, {"error": "需要 JSON 请求"})
            if self.path != "/api/run":
                return self.reply(404, {"error": "未找到接口"})
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 1024:
                    raise ValueError("请求长度无效")
                data = json.loads(self.rfile.read(length))
                if not isinstance(data, dict):
                    raise ValueError("请求应为对象")
                run_id = simulation.start(data.get("preset"), data.get("confirmed"),
                                          data.get('motion_permission','no_motion'))
                self.reply(201, {"run_id": run_id})
            except (ValueError, TypeError) as error:
                self.reply(400, {"error": str(error)})
            except RuntimeError as error:
                self.reply(409, {"error": str(error)})
    return Handler


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--output-root", type=Path, default=ROOT / "reports/visual-simulation")
    parser.add_argument("--demo", choices=PRESETS,
                        help="Start a fresh physical simulation with the declared fixture clearance")
    parser.add_argument('--motion-permission',choices=('no_motion','bounded_probe'),default='no_motion',
                        help='Explicit authorization for a straight/blocked --demo only')
    args = parser.parse_args(argv)
    if args.motion_permission!='no_motion' and args.demo not in ('straight','blocked'):
        parser.error('--motion-permission bounded_probe requires --demo straight or blocked')
    simulation = Simulation(args.output_root)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(simulation))
    try:
        if args.demo:
            simulation.start(args.demo, True,args.motion_permission)
        print(f"模拟可视化：http://127.0.0.1:{server.server_port}", flush=True)
        print("仅用于受限仿真；Ctrl+C 关闭。原始结果：" + str(simulation.output_root), flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        simulation.close()


if __name__ == "__main__":
    main()
