"""Flask service for natural-language control of a lerobot arm through GR00T.

Usage example:

PYTHONPATH=/g/robot/Isaac-GR00T:/g/robot/lerobot/src \
  '/c/Users/Grape/.conda/envs/lerobot/python.exe' demo/gr00t_lerobot_service.py \
  --robot.type=so101_follower \
  --robot.port=COM4 \
  --robot.id=my_awesome_follower_arm \
  --robot.max_relative_target=8.0 \
  --server_host=192.168.31.103 \
  --server_port=5555 \
  --service_host=127.0.0.1 \
  --service_port=5001

Example request:

curl -X POST http://127.0.0.1:5001/command \
  -H 'Content-Type: application/json' \
  -d '{"task": "pick up the cup", "num_chunks": 1}'
"""

from dataclasses import asdict, dataclass
import time
from threading import Lock, Thread

import cv2
import numpy as np

from flask import Flask, Response, jsonify, request

from gr00t.policy.server_client import PolicyClient
from gr00t_lerobot_client import Gr00TLerobotAdapter, Gr00TLerobotClientConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.configs import parser
from lerobot.robots import RobotConfig, so_follower  # noqa: F401
from lerobot.robots.utils import make_robot_from_config
from lerobot.utils.import_utils import register_third_party_plugins

_UI_HTML = """\
<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>GR00T 机械臂控制台</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: 'Segoe UI', system-ui, sans-serif; background: #0f1117; color: #e0e0e0; min-height: 100vh; }
    .header { background: #1a1d27; border-bottom: 1px solid #2d3148; padding: 16px 24px; display: flex; align-items: center; gap: 12px; }
    .header h1 { font-size: 1.2rem; font-weight: 600; }
    .dot { width: 10px; height: 10px; border-radius: 50%; background: #444; flex-shrink: 0; transition: background 0.3s; }
    .dot.ok { background: #4ade80; box-shadow: 0 0 6px #4ade80; }
    .dot.err { background: #f87171; }
    .status-info { font-size: 0.82rem; color: #888; margin-left: auto; }
    .main { max-width: 1050px; margin: 28px auto; padding: 0 20px; }
    .card { background: #1a1d27; border: 1px solid #2d3148; border-radius: 12px; padding: 22px; margin-bottom: 18px; }
    .card-title { font-size: 0.78rem; font-weight: 600; color: #666; text-transform: uppercase; letter-spacing: 0.06em; margin-bottom: 14px; }
    textarea { width: 100%; background: #0f1117; border: 1px solid #2d3148; border-radius: 8px; color: #e0e0e0; font-size: 1rem; padding: 12px 14px; resize: vertical; min-height: 76px; outline: none; transition: border-color 0.2s; font-family: inherit; }
    textarea:focus { border-color: #6366f1; }
    .row { display: flex; gap: 10px; align-items: center; margin-top: 12px; flex-wrap: wrap; }
    .lbl { font-size: 0.82rem; color: #888; white-space: nowrap; }
    input[type=range] { flex: 1; min-width: 100px; accent-color: #6366f1; cursor: pointer; }
    .chunk-val { font-size: 0.9rem; font-weight: 700; color: #a5b4fc; min-width: 18px; text-align: center; }
    .toggle { display: flex; align-items: center; gap: 6px; cursor: pointer; user-select: none; }
    .toggle input { accent-color: #6366f1; width: 15px; height: 15px; cursor: pointer; }
    .btn { padding: 9px 22px; border-radius: 8px; border: none; font-size: 0.92rem; font-weight: 600; cursor: pointer; transition: all 0.15s; }
    .btn-primary { background: #6366f1; color: #fff; margin-left: auto; }
    .btn-primary:hover:not(:disabled) { background: #4f51e0; }
    .btn-primary:disabled { background: #2d2f50; color: #555; cursor: not-allowed; }
    .btn-ghost { background: #2d3148; color: #a0a0c0; }
    .btn-ghost:hover { background: #363a55; }
    .presets { display: flex; gap: 8px; flex-wrap: wrap; margin-bottom: 12px; }
    .preset { padding: 5px 13px; border-radius: 20px; border: 1px solid #2d3148; background: transparent; color: #888; font-size: 0.78rem; cursor: pointer; transition: all 0.15s; }
    .preset:hover { border-color: #6366f1; color: #a5b4fc; background: #1e2035; }
    .log { list-style: none; display: flex; flex-direction: column; gap: 8px; max-height: 420px; overflow-y: auto; }
    .log-item { background: #0f1117; border: 1px solid #2d3148; border-left-width: 3px; border-radius: 8px; padding: 11px 13px; font-size: 0.84rem; }
    .log-item.ok { border-left-color: #4ade80; }
    .log-item.err { border-left-color: #f87171; }
    .log-task { color: #c7d2fe; font-weight: 500; margin-bottom: 3px; }
    .log-meta { color: #555; font-size: 0.73rem; }
    .log-joints { color: #666; font-family: monospace; font-size: 0.72rem; margin-top: 4px; word-break: break-all; }
    .log-err { color: #f87171; font-size: 0.73rem; margin-top: 4px; }
    .empty { color: #444; text-align: center; padding: 28px; font-size: 0.85rem; }
    .top-grid { display: grid; grid-template-columns: 380px 1fr; gap: 18px; margin-bottom: 18px; }
    @media (max-width: 740px) { .top-grid { grid-template-columns: 1fr; } }
    .video-feed { width: 100%; border-radius: 8px; display: block; background: #000; aspect-ratio: 4/3; max-height: 340px; object-fit: contain; }
    .video-label { font-size: 0.72rem; color: #555; text-align: center; margin-top: 8px; }
    .tele-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
    @media (max-width: 540px) { .tele-grid { grid-template-columns: 1fr; } }
    .tele-box { background: #0f1117; border: 1px solid #2d3148; border-radius: 8px; padding: 12px 14px; }
    .tele-box-title { font-size: 0.72rem; font-weight: 600; color: #555; text-transform: uppercase; letter-spacing: 0.05em; margin-bottom: 10px; }
    .tele-row { display: flex; justify-content: space-between; align-items: baseline; gap: 8px; padding: 3px 0; border-bottom: 1px solid #1e2130; }
    .tele-row:last-child { border-bottom: none; }
    .tele-key { font-size: 0.76rem; color: #666; white-space: nowrap; }
    .tele-val { font-family: monospace; font-size: 0.78rem; color: #a5b4fc; text-align: right; word-break: break-all; }
    .tele-val.pos { color: #4ade80; }
    .tele-val.neg { color: #f87171; }
    .tele-val.neutral { color: #94a3b8; }
    .tele-lang { font-size: 0.82rem; color: #c7d2fe; font-style: italic; margin-bottom: 10px; padding: 8px 10px; background: #13162a; border-radius: 6px; }
  </style>
</head>
<body>
  <div class="header">
    <div class="dot" id="dot"></div>
    <h1>GR00T 机械臂控制台</h1>
    <div class="status-info" id="status-text">连接中...</div>
  </div>
  <div class="main">
    <div class="top-grid">
      <div class="card">
        <div class="card-title">摄像头画面</div>
        <img class="video-feed" src="/video_feed" alt="摄像头画面">
        <div class="video-label">实时传送给模型的画面</div>
      </div>
      <div class="card">
        <div class="card-title">发送指令</div>
      <div class="presets">
        <button class="preset" onclick="setTask('把杯子放进碗里')">把杯子放进碗里</button>
        <button class="preset" onclick="setTask('拿起杯子')">拿起杯子</button>
        <button class="preset" onclick="setTask('放下物体')">放下物体</button>
        <button class="preset" onclick="setTask('张开夹爪')">张开夹爪</button>
        <button class="preset" onclick="setTask('闭合夹爪')">闭合夹爪</button>
      </div>
      <textarea id="task-input" placeholder="输入自然语言指令，例如：把红色积木放进碗里（Enter 发送，Shift+Enter 换行）"></textarea>
      <div class="row">
        <span class="lbl">步数</span>
        <input type="range" id="chunks" min="1" max="8" value="1"
               oninput="document.getElementById('chunks-val').textContent=this.value">
        <span class="chunk-val" id="chunks-val">1</span>
        <label class="toggle">
          <input type="checkbox" id="reset-policy" checked>
          <span class="lbl">重置策略</span>
        </label>
        <button class="btn btn-ghost" onclick="doReset()">重置</button>
        <button class="btn btn-primary" id="send-btn" onclick="sendCommand()">执行</button>
      </div>
    </div>
    </div>
    <div class="card">
      <div class="card-title">模型输入 / 输出</div>
      <div id="tele-lang" class="tele-lang" style="display:none"></div>
      <div class="tele-grid">
        <div class="tele-box">
          <div class="tele-box-title">输入 — 状态观测</div>
          <div id="tele-input"><div class="empty" style="padding:16px">等待首次执行...</div></div>
        </div>
        <div class="tele-box">
          <div class="tele-box-title">输出 — 动作指令</div>
          <div id="tele-output"><div class="empty" style="padding:16px">等待首次执行...</div></div>
        </div>
      </div>
    </div>
    <div class="card">
      <div class="card-title">执行历史</div>
      <ul class="log" id="log"><li class="empty">暂无记录</li></ul>
    </div>
  </div>
  <script>
    const history = [];
    function setTask(t) { document.getElementById('task-input').value = t; document.getElementById('task-input').focus(); }
    function esc(s) { return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;'); }

    async function pollStatus() {
      try {
        const d = await fetch('/status').then(r => r.json());
        const dot = document.getElementById('dot');
        const txt = document.getElementById('status-text');
        dot.className = 'dot ' + (d.connected ? 'ok' : 'err');
        txt.textContent = d.connected
          ? `已连接 ${d.server_host}:${d.server_port}${d.busy ? ' · 执行中...' : ''}`
          : '未连接';
      } catch {
        document.getElementById('dot').className = 'dot err';
        document.getElementById('status-text').textContent = '服务不可用';
      }
    }

    async function sendCommand() {
      const task = document.getElementById('task-input').value.trim();
      if (!task) return;
      const num_chunks = parseInt(document.getElementById('chunks').value);
      const reset_policy = document.getElementById('reset-policy').checked;
      const btn = document.getElementById('send-btn');
      btn.disabled = true; btn.textContent = '执行中...';
      const ts = new Date().toLocaleTimeString();
      try {
        const d = await fetch('/command', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({task, num_chunks, reset_policy})
        }).then(r => r.json());
        addLog(task, d, ts);
      } catch(e) {
        addLog(task, {ok: false, error: String(e)}, ts);
      } finally {
        btn.disabled = false; btn.textContent = '执行';
      }
    }

    async function doReset() {
      const btn = event.target;
      btn.disabled = true;
      try { await fetch('/reset', {method: 'POST'}); } finally { btn.disabled = false; }
      pollStatus();
    }

    function addLog(task, result, ts) {
      history.unshift({task, result, ts});
      renderLog();
    }

    function renderLog() {
      const ul = document.getElementById('log');
      if (!history.length) { ul.innerHTML = '<li class="empty">暂无记录</li>'; return; }
      ul.innerHTML = history.slice(0, 40).map(h => {
        const ok = h.result.ok;
        const joints = h.result.joint_radians
          ? h.result.joint_radians.map(v => v.toFixed(3)).join(', ')
          : null;
        return `<li class="log-item ${ok ? 'ok' : 'err'}">
          <div class="log-task">${esc(h.task)}</div>
          <div class="log-meta">${h.ts}${h.result.executed_chunks ? ' &nbsp;·&nbsp; ' + h.result.executed_chunks + ' 步' : ''}</div>
          ${joints ? `<div class="log-joints">[${joints}]</div>` : ''}
          ${h.result.error ? `<div class="log-err">${esc(h.result.error)}</div>` : ''}
        </li>`;
      }).join('');
    }

    document.addEventListener('DOMContentLoaded', () => {
      document.getElementById('task-input').addEventListener('keydown', e => {
        if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); sendCommand(); }
      });
    });

    pollStatus();
    setInterval(pollStatus, 2000);
    setInterval(pollTelemetry, 800);

    async function pollTelemetry() {
      try {
        const d = await fetch('/telemetry').then(r => r.json());
        if (!d.input) return;
        // language
        const langEl = document.getElementById('tele-lang');
        langEl.style.display = '';
        langEl.textContent = '"' + d.input.language + '"';
        // input
        const jp = d.input.joint_position_rad;
        const names = ['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','J6','J7'];
        document.getElementById('tele-input').innerHTML =
          jp.map((v,i) => row(names[i], v.toFixed(4) + ' rad', v)).join('') +
          row('gripper', (d.input.gripper_position * 100).toFixed(1) + ' %', d.input.gripper_position - 0.5);
        // output
        const jd = d.output.joint_delta_rad;
        document.getElementById('tele-output').innerHTML =
          jd.map((v,i) => row('Δ ' + names[i], (v >= 0 ? '+' : '') + v.toFixed(4) + ' rad', v)).join('') +
          row('gripper target', (d.output.gripper_target * 100).toFixed(1) + ' %', d.output.gripper_target - 0.5) +
          row('chunk steps', d.output.chunk_steps, 0);
      } catch {}
    }

    function row(k, v, sign) {
      const cls = sign > 0.001 ? 'pos' : sign < -0.001 ? 'neg' : 'neutral';
      return `<div class="tele-row"><span class="tele-key">${k}</span><span class="tele-val ${cls}">${v}</span></div>`;
    }
  </script>
</body>
</html>
"""


@dataclass
class Gr00TLerobotServiceConfig(Gr00TLerobotClientConfig):
    service_host: str = "127.0.0.1"
    service_port: int = 5001
    debug: bool = False
    command_chunk_limit: int = 8


class LanguageControlService:
    def __init__(self, cfg: Gr00TLerobotServiceConfig):
        self.cfg = cfg
        self.app = Flask(__name__)
        self.client: PolicyClient | None = None
        self.robot = None
        self.adapter: Gr00TLerobotAdapter | None = None
        self.current_q = None
        self.last_task = cfg.task_description
        self.last_result: dict | None = None
        self.last_telemetry: dict | None = None
        self.command_lock = Lock()
        self._latest_frame: np.ndarray | None = None
        self._frame_lock = Lock()
        self._stream_active = False
        self._frame_thread: Thread | None = None
        self._register_routes()

    def connect(self):
        self.client = PolicyClient(host=self.cfg.server_host, port=self.cfg.server_port)
        if not self.client.ping():
            raise ConnectionError(
                f"Unable to reach GR00T server at {self.cfg.server_host}:{self.cfg.server_port}."
            )

        self.robot = make_robot_from_config(self.cfg.robot)
        self.robot.connect()
        self.adapter = Gr00TLerobotAdapter(self.robot, self.cfg)
        _, self.current_q = self.adapter.build_gr00t_observation()
        self._stream_active = True
        self._frame_thread = Thread(target=self._frame_loop, daemon=True, name="frame-capture")
        self._frame_thread.start()

    def disconnect(self):
        self._stream_active = False
        if self.robot is not None:
            self.robot.disconnect()
            self.robot = None

    def reset_policy(self):
        if self.client is None or self.adapter is None:
            raise RuntimeError("Service is not connected.")
        self.client.reset()
        _, self.current_q = self.adapter.build_gr00t_observation()

    def execute_command(self, task: str, num_chunks: int, reset_policy: bool) -> dict:
        if self.client is None or self.adapter is None:
            raise RuntimeError("Service is not connected.")

        if num_chunks < 1:
            raise ValueError("num_chunks must be >= 1")
        if num_chunks > self.cfg.command_chunk_limit:
            raise ValueError(
                f"num_chunks must be <= {self.cfg.command_chunk_limit} for safety"
            )

        with self.command_lock:
            self.adapter.cfg.task_description = task
            self.last_task = task

            if reset_policy:
                self.client.reset()
                _, self.current_q = self.adapter.build_gr00t_observation()

            executed_chunks = 0
            for _ in range(num_chunks):
                gr00t_obs, q_before = self.adapter.build_gr00t_observation()
                with self._frame_lock:
                    self._latest_frame = gr00t_obs["video"]["exterior_image_1_left"][0, 0]
                actions, _ = self.client.get_action(gr00t_obs)
                self.last_telemetry = {
                    "input": {
                        "joint_position_rad": gr00t_obs["state"]["joint_position"][0, 0].tolist(),
                        "gripper_position": float(gr00t_obs["state"]["gripper_position"][0, 0, 0]),
                        "language": gr00t_obs["language"]["annotation.language.language_instruction"][0][0],
                    },
                    "output": {
                        "joint_delta_rad": actions["joint_position"][0, 0, :5].tolist(),
                        "gripper_target": float(actions["gripper_position"][0, 0, 0]),
                        "chunk_steps": int(actions["joint_position"].shape[1]),
                    },
                }
                self.current_q = self.adapter.execute_action_chunk(actions, self.current_q)
                executed_chunks += 1

            self.last_result = {
                "ok": True,
                "task": task,
                "executed_chunks": executed_chunks,
                "joint_radians": [float(v) for v in self.current_q],
            }
            return self.last_result

    def status(self) -> dict:
        return {
            "ok": True,
            "connected": self.client is not None and self.robot is not None,
            "server_host": self.cfg.server_host,
            "server_port": self.cfg.server_port,
            "last_task": self.last_task,
            "busy": self.command_lock.locked(),
            "last_result": self.last_result,
        }

    def _frame_loop(self):
        while self._stream_active:
            if self.adapter is not None and not self.command_lock.locked():
                if self.command_lock.acquire(blocking=False):
                    try:
                        raw = self.adapter.get_raw_observation()
                        frame = self.adapter.get_camera_frame(raw, self.cfg.exterior_camera_key)
                        with self._frame_lock:
                            self._latest_frame = frame
                    except Exception:
                        pass
                    finally:
                        self.command_lock.release()
            time.sleep(0.2)

    def _generate_mjpeg(self):
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        while True:
            with self._frame_lock:
                frame = self._latest_frame
            if frame is None:
                frame = blank
            _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n"
            time.sleep(0.05)

    def _register_routes(self):
        @self.app.get("/")
        def index():
            return Response(_UI_HTML, mimetype="text/html; charset=utf-8")

        @self.app.get("/api")
        def api_docs():
            return jsonify(
                {
                    "service": "gr00t-lerobot-language-control",
                    "routes": {
                        "GET /": "web UI",
                        "GET /health": "service health",
                        "GET /status": "connection and last command status",
                        "POST /reset": "reset GR00T policy state",
                        "POST /command": {
                            "task": "natural language instruction",
                            "num_chunks": "optional, default 1",
                            "reset_policy": "optional, default true",
                        },
                    },
                }
            )

        @self.app.get("/health")
        def health():
            return jsonify({"ok": True})

        @self.app.get("/video_feed")
        def video_feed():
            return Response(
                self._generate_mjpeg(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @self.app.get("/telemetry")
        def telemetry():
            return jsonify(self.last_telemetry or {})

        @self.app.get("/status")
        def get_status():
            return jsonify(self.status())

        @self.app.post("/reset")
        def reset():
            try:
                self.reset_policy()
                return jsonify({"ok": True, "message": "policy reset"})
            except Exception as exc:
                return jsonify({"ok": False, "error": str(exc)}), 500

        @self.app.post("/command")
        def command():
            payload = request.get_json(silent=True) or {}
            task = payload.get("task") or payload.get("text")
            if not task:
                return jsonify({"ok": False, "error": "Missing 'task' in JSON body"}), 400

            num_chunks = int(payload.get("num_chunks", 1))
            reset_policy = bool(payload.get("reset_policy", True))

            try:
                result = self.execute_command(task, num_chunks, reset_policy)
                return jsonify(result)
            except ValueError as exc:
                return jsonify({"ok": False, "error": str(exc)}), 400
            except Exception as exc:
                return jsonify({"ok": False, "error": str(exc)}), 500


@parser.wrap()
def main(cfg: Gr00TLerobotServiceConfig):
    service = LanguageControlService(cfg)
    service.connect()

    print(f"[INFO] GR00T server connected at {cfg.server_host}:{cfg.server_port}")
    print(f"[INFO] Service listening on http://{cfg.service_host}:{cfg.service_port}")

    try:
        service.app.run(host=cfg.service_host, port=cfg.service_port, debug=cfg.debug, use_reloader=False)
    finally:
        service.disconnect()


if __name__ == "__main__":
    register_third_party_plugins()
    main()