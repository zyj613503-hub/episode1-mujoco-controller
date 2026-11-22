#!/usr/bin/env python3
"""上位机 V2 后端：契约 §1..§5 的 /api/v2/*（由 server/app.py 调用，本身不监听端口）。

设计（契约 V2.1）
----
* 上位机是**操控系统**：MuJoCo 是独立应用进程（bridge/sim_bridge.py，有自己窗口），
  这里通过 robot/simapp.py 起/停它，并用 TCP 47999 的 CAN 语义像连真机一样控制它。
  本进程**不建也不步进 MuJoCo 仿真**，也**不把 MuJoCo 画面推进页面**。
* 真值源是仿真应用的读回（0x36 + 4 字节大端 V）。30Hz 轮询一次缓存起来；
  flange/tcp 由 server 侧用 robot/ik.py 的纯运动学 FK 从关节角算出，供页面本地 3D 预览。
* /api/v2/stream.mjpg 只回一张静态占位图（旧页面的 <img> 不会 404），没有任何画面推流。
* 只用标准库；不改动 app.py 里旧的 11 条 /api/*。

对外只暴露两个东西：
    PAGE_ALIASES            页面别名  /v2/base -> web/v2/base.html
    dispatch(handler, ...)  返回 True 表示这个请求已经被 V2 处理完
"""

from __future__ import annotations

import json
import math
import os
import re
import struct
import sys
import threading
import time
import traceback
import zlib
from datetime import datetime
from pathlib import Path
from urllib.parse import parse_qs

# ---------------------------------------------------------------------------
# 路径（与 app.py 保持同一套「向上找 model/episode1.xml」的约定）
# ---------------------------------------------------------------------------
SERVER_DIR = Path(__file__).resolve().parent
RUNNER_DIR = SERVER_DIR.parent
for _p in (str(SERVER_DIR), str(RUNNER_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _find_root(start):
    for parent in [start, *start.parents]:
        if (parent / "model" / "episode1.xml").is_file():
            return parent
    return start.parent


WORK4_DIR = _find_root(RUNNER_DIR)
if str(WORK4_DIR) not in sys.path:
    sys.path.insert(0, str(WORK4_DIR))

WEB_DIR = (RUNNER_DIR / "web").resolve()
# 脚本库 = 项目根的 scripts/（robot/scriptgen.py 就写这里；运行器/scripts 是 UR5e 时代的老脚本，不再对外列出）
SCRIPTS_DIR = (WORK4_DIR / "scripts").resolve()
TEMPLATE_PATH = WORK4_DIR / "action" / "arm_motion.py"

MAX_BODY = 5 * 1024 * 1024
VERSION = "2.1"

DEFAULT_MODEL_XML = "model/episode1.xml"
DEFAULT_SCENE_XML = "model/scene_episode1.xml"

# 契约 §4 的默认视角
DEFAULT_CAM = {"az": 138.0, "el": -19.0, "dist": 1.22, "lookat": [0.20, 0.10, 0.13]}

# 调 Arm 的看门狗：单次调用最多等多久（秒）。state 要快（SSE 10Hz），
# 规划/回零这类可能真慢，给宽一点。超时后 ARM_DEAD_COOLDOWN 秒内不再重试。
ARM_TIMEOUTS = {"state": 1.0, "render_jpeg": 2.0, "seek": 5.0, "load": 20.0}
DEFAULT_ARM_TIMEOUT = 6.0
ARM_DEAD_COOLDOWN = 5.0

IK_TEXT = {
    "ok": "可达",
    "self_collision": "自我干涉",
    "wrist_flip": "不可达（需翻腕）",
    "discontinuity": "不可达（关节突跳）",
    "unreachable": "不可达（够不到）",
}

# mock 用的零位末端位姿（取自 model/episode1.xml 零位一次算出的真实值）
MOCK_FLANGE_MM = [324.0, 0.0, 502.2]
MOCK_FLANGE_DEG = [-4.15, -90.0, -175.85]

# 强制 mock：RUNNER_MOCK=1
MOCK_ENV = os.environ.get("RUNNER_MOCK", "").strip().lower() in ("1", "true", "yes", "on")

# ---------------------------------------------------------------------------
# 依赖：layout（脚本生成/解析）与 robot.convention（换算唯一来源）
# ---------------------------------------------------------------------------
try:
    import layout as layoutlib  # noqa: E402
except Exception:  # noqa: BLE001
    layoutlib = None

# 路点脚本生成走 robot/scriptgen.py（真正的写文件实现）；它不可用时 generate 明确报错
try:
    from robot import scriptgen as scriptgenlib  # noqa: E402
except Exception:  # noqa: BLE001
    scriptgenlib = None

try:
    from robot import convention as C  # noqa: E402
except Exception:  # noqa: BLE001
    try:
        import convention as C  # noqa: E402
    except Exception:  # noqa: BLE001

        class _FallbackConvention(object):
            """convention.py 不可用时的兜底数字（与契约 §1 的 range 一致）。"""

            JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
            JOINT_LABELS = ["J1", "J2", "J3", "J4", "J5", "J6"]
            OFFSET = [180.0, 90.0, 83.0, 30.0, 110.0, 30.0]
            MOTOR_RANGE_DEG = [(0.0, 340.0), (0.0, 180.0), (0.0, 163.0),
                               (0.0, 335.0), (0.0, 220.0), (0.0, 335.0)]
            GRIPPER_RANGE_DEG = (0.0, 110.0)
            DEFAULT_TCP_MM = 120.0

            @staticmethod
            def clamp_deg(i, deg):
                lo, hi = _FallbackConvention.MOTOR_RANGE_DEG[i]
                return max(lo, min(hi, float(deg)))

            @staticmethod
            def deg_to_dh(i, deg):
                return math.radians(float(deg) - _FallbackConvention.OFFSET[i])

            @staticmethod
            def dh_to_deg(i, dh):
                return _FallbackConvention.OFFSET[i] + math.degrees(float(dh))

            @staticmethod
            def readback_step_deg(i):
                return 1.0

        C = _FallbackConvention()


# 独立 MuJoCo 仿真应用的进程管理 + TCP 客户端（robot/simapp.py）
try:
    from robot import simapp as simapplib  # noqa: E402
except Exception:  # noqa: BLE001
    try:
        import simapp as simapplib  # noqa: E402
    except Exception:  # noqa: BLE001
        simapplib = None


JOINT_NAMES = list(getattr(C, "JOINT_LABELS", ["J1", "J2", "J3", "J4", "J5", "J6"]))
JOINT_RANGE = [list(map(float, r)) for r in getattr(C, "MOTOR_RANGE_DEG", [])]
GRIPPER_RANGE = list(map(float, getattr(C, "GRIPPER_RANGE_DEG", (0.0, 110.0))))
TCP_MM = float(getattr(C, "DEFAULT_TCP_MM", 120.0))
OFFSET = [float(v) for v in getattr(C, "OFFSET", [180.0, 90.0, 83.0, 30.0, 110.0, 30.0])]


def _clamp_joint(i, deg):
    try:
        return float(C.clamp_deg(i, deg))
    except Exception:  # noqa: BLE001
        lo, hi = JOINT_RANGE[i]
        return max(lo, min(hi, float(deg)))


LIMIT_EPS_DEG = 0.25  # 距软限位小于这个角度就算「顶到限位」（数值框描红）


def _at_joint_limit(i, deg):
    """该关节当前是否顶在软限位上。前端据此把数值框描红。"""
    try:
        lo, hi = JOINT_RANGE[i]
        v = float(deg)
    except Exception:  # noqa: BLE001
        return False
    return bool(v <= lo + LIMIT_EPS_DEG or v >= hi - LIMIT_EPS_DEG)


def _wp_fields(wp):
    """路点两种写法都吃：[x,y,z,dwell,grip] 或 {x,y,z,hold,gripper}。"""
    if isinstance(wp, dict):
        x, y, z = wp.get("x"), wp.get("y"), wp.get("z")
        hold = wp.get("hold", wp.get("dwell", 0.0))
        grip = wp.get("gripper", wp.get("grip"))
    else:
        seq = list(wp)
        if len(seq) < 3:
            raise ValueError("路点至少要 3 个数")
        x, y, z = seq[0], seq[1], seq[2]
        hold = seq[3] if len(seq) > 3 else 0.0
        grip = seq[4] if len(seq) > 4 else None
    try:
        hold = float(hold or 0.0)
    except (TypeError, ValueError):
        hold = 0.0
    # 夹爪：新数据是 0–110 的数字角度；旧数据 'open'/'close' 仍兼容（0/110）
    if isinstance(grip, str):
        key = grip.strip().lower()
        grip = {"open": 0.0, "close": 110.0}.get(key)
    elif grip is None:
        grip = None
    else:
        try:
            grip = max(0.0, min(110.0, float(grip)))
        except (TypeError, ValueError):
            grip = None
    return {"x": float(x), "y": float(y), "z": float(z), "hold": hold, "grip": grip}


def _norm_path(raw, default):
    """把 body 里的 model/scene 收敛成相对 work4 的路径。

    前端可能给 "model/episode1.xml"，也可能只给 "episode1.xml"——
    后者补上 model/ 前缀，并在存在时优先用它。
    """
    text = str(raw or "").strip().replace("\\", "/")
    if not text:
        return default
    while text.startswith("./"):
        text = text[2:]
    if "/" in text:
        return text
    for cand in ("model/" + text, text):
        if _rel_file(cand) is not None:
            return cand
    return "model/" + text


def _rel_file(text):
    """相对 work4 的路径 -> 绝对 Path（必须落在 work4 内且是文件）。"""
    try:
        candidate = (WORK4_DIR / text).resolve()
        candidate.relative_to(WORK4_DIR)
    except (ValueError, OSError):
        return None
    return candidate if candidate.is_file() else None


# ---------------------------------------------------------------------------
# 日志环形缓冲（/api/v2/logs 与 SSE 的 log 字段共用）
# ---------------------------------------------------------------------------
class LogBook(object):
    def __init__(self, limit=400):
        self._lock = threading.Lock()
        self._items = []
        self._seq = 0
        self._limit = limit

    def add(self, level, text):
        with self._lock:
            self._seq += 1
            item = {"seq": self._seq, "t": round(time.time(), 3),
                    "level": str(level), "text": str(text)}
            self._items.append(item)
            if len(self._items) > self._limit:
                del self._items[: len(self._items) - self._limit]
            return item

    def since(self, seq):
        with self._lock:
            return [dict(i) for i in self._items if i["seq"] > seq]

    def last_seq(self):
        with self._lock:
            return self._seq


LOGS = LogBook()


def _log(level, text):
    return LOGS.add(level, text)


# ---------------------------------------------------------------------------
# mock 帧（无 pillow：手写 PNG，深色渐变 + 一条参考横线）
# ---------------------------------------------------------------------------
_PNG_CACHE = {}
_PNG_LOCK = threading.Lock()


def _png_chunk(tag, data):
    return (struct.pack(">I", len(data)) + tag + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))


def mock_png(w, h):
    w = max(16, int(w))
    h = max(16, int(h))
    key = (w, h)
    with _PNG_LOCK:
        hit = _PNG_CACHE.get(key)
        if hit is not None:
            return hit
    lines = []
    for y in range(h):
        k = y / max(1, h - 1)
        base = int(22 + 34 * k)
        rgb = bytes((base, base + 8, base + 18))
        if abs(y - int(h * 0.5)) <= 1:            # 中间一条浅色参考线
            rgb = bytes((58, 74, 96))
        lines.append(b"\x00" + rgb * w)           # 每行 filter=0
    raw = b"".join(lines)
    png = (b"\x89PNG\r\n\x1a\n"
           + _png_chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + _png_chunk(b"IDAT", zlib.compress(raw, 6))
           + _png_chunk(b"IEND", b""))
    with _PNG_LOCK:
        _PNG_CACHE[key] = png
    return png



# ---------------------------------------------------------------------------
# 仿真应用控制器（上位机只是客户端：MuJoCo 在独立进程里跑）
# ---------------------------------------------------------------------------
# 契约 V2.1：上位机是操控系统，不许在本进程里建/步进 MuJoCo 仿真，
# 也不许把 MuJoCo 画面推进页面。这里只做三件事：
#   1) 通过 robot/simapp.py 起/停独立的 MuJoCo 应用进程；
#   2) 用 TCP 47999 的 CAN 语义下发关节/夹爪、30Hz 读回真值；
#   3) 用 robot/ik.py 纯运动学算 flange/tcp，供页面本地 3D 预览用。
PLAY_HZ = 50.0        # /play 流式下发的频率（契约 §3）
POLL_HZ = 30.0        # read_state 轮询频率
PLAY_TICK = 1.0 / PLAY_HZ


class SimController(object):
    """把「独立 MuJoCo 应用 + TCP 客户端」包成永不抛异常的 HTTP 适配层。"""

    def __init__(self):
        self._lock = threading.RLock()
        self.mode = "sim"
        self.channel = 1
        self.port = int(getattr(simapplib, "DEFAULT_PORT", 47999)) if simapplib else 47999
        self.connected = False
        self.error = ""
        self.estop_on = False
        self.client = None
        self.last = {"deg": None, "gripper": None, "ts": 0.0}
        self.fail_streak = 0
        self.target = list(OFFSET)
        self.grip_target = 0.0
        self._poll_stop = threading.Event()
        self._poll_thread = None
        self._ik = None
        self._ik_tried = False
        self._app_cache = {"at": 0.0, "value": None}
        self._pid_cache = {"at": 0.0, "pid": None}   # 持连接时也要能报出应用 pid
        self.cam = {"az": DEFAULT_CAM["az"], "el": DEFAULT_CAM["el"],
                    "dist": DEFAULT_CAM["dist"], "lookat": list(DEFAULT_CAM["lookat"])}
        self.play = {"traj": [], "state": "idle", "t": 0.0, "duration": 0.0,
                     "speed": 1.0, "loop": False}
        self._play_thread = None
        self._play_stop = threading.Event()
        self._streams = 0
        self._streams_lock = threading.Lock()
        self.started_at = time.time()

    # -- 连接数保护 ------------------------------------------------------
    def acquire_stream(self, limit=16):
        with self._streams_lock:
            if self._streams >= limit:
                return False
            self._streams += 1
            return True

    def release_stream(self):
        with self._streams_lock:
            self._streams = max(0, self._streams - 1)

    # -- 应用进程 --------------------------------------------------------
    def app_status(self):
        """带 0.5s 缓存的应用状态（/state 有 10Hz 的 SSE，别每次都探端口）。

        注意：仿真应用的 bridge 同一时刻只服务一个客户端，所以在上位机已经持有连接时
        再探端口会被对端 RST（reachable 变 False）。持有连接本身就是最可靠的"活着"证据。
        """
        with self._lock:
            live = self.client is not None and self.connected
            cached = dict(self._app_cache["value"] or {})
        if live:
            # 持连接时端口探测会被"只服务一条连接"的对端 RST，所以只按连接本身判定活着；
            # pid 用 lsof 轻量拿（2s 缓存，避免 10Hz 的 /state 拖慢），外部起的进程也能显示。
            pid = cached.get("pid")
            managed = cached.get("managed")
            now0 = time.time()
            if simapplib is not None and (pid is None or now0 - self._pid_cache["at"] > 2.0):
                try:
                    info = simapplib.app_status(port=self.port)
                    pid = info.get("pid") or pid
                    managed = bool(info.get("managed")) if pid is not None else managed
                except Exception:  # noqa: BLE001
                    pass
                with self._lock:
                    self._pid_cache = {"at": now0, "pid": pid}
                    if pid is not None:
                        self._app_cache["value"] = dict(cached or {}, pid=pid,
                                                        running=True, reachable=True,
                                                        external=not bool(managed),
                                                        managed=bool(managed))
            value = {"running": True, "pid": pid, "port": self.port,
                     "reachable": True,
                     "external": bool((cached or {}).get("external", not bool(managed))),
                     "managed": bool(managed) if managed is not None else True}
            with self._lock:
                self._app_cache = {"at": time.time(), "value": dict(value)}
            return value
        now = time.time()
        with self._lock:
            if self._app_cache["value"] is not None and now - self._app_cache["at"] < 0.5:
                return dict(self._app_cache["value"])
        if simapplib is None:
            value = {"running": False, "pid": None, "port": self.port,
                     "reachable": False, "managed": False, "external": False,
                     "error": "robot/simapp.py 不可用"}
        else:
            try:
                value = simapplib.app_status(port=self.port)
            except Exception as exc:  # noqa: BLE001
                value = {"running": False, "pid": None, "port": self.port,
                         "reachable": False, "managed": False,
                         "error": "%s: %s" % (type(exc).__name__, exc)}
        with self._lock:
            self._app_cache = {"at": now, "value": dict(value)}
        return value

    def app(self, action="status"):
        """契约 §3 /app：start / stop / status。"""
        action = str(action or "status").strip().lower()
        if simapplib is None:
            return {"ok": False, "error": "robot/simapp.py 不可用，无法管理仿真应用",
                    "app": self.app_status(), "state": self.state()}
        try:
            if action == "start":
                info = simapplib.ensure_app(port=self.port, timeout=20.0)
                _log("info", "MuJoCo 仿真应用已就绪（pid %s · TCP %s）"
                     % (info.get("pid"), self.port))
                return {"ok": True, "app": self.app_status(), "state": self.state()}
            if action == "stop":
                if self.client is not None:
                    self._drop_client()
                res = simapplib.stop_app(port=self.port)
                _log("info", "已关闭 MuJoCo 仿真应用")
                return {"ok": bool(res.get("ok", True)), "app": self.app_status(),
                        "detail": res, "state": self.state()}
            if action in ("status", ""):
                return {"ok": True, "app": self.app_status(), "state": self.state()}
            return {"ok": False, "error": "action 只能是 start / stop / status",
                    "app": self.app_status(), "state": self.state()}
        except Exception as exc:  # noqa: BLE001
            msg = str(exc) or "%s" % type(exc).__name__
            _log("error", "仿真应用操作失败：%s" % msg)
            return {"ok": False, "error": msg, "app": self.app_status(),
                    "state": self.state()}

    # -- 连接 ------------------------------------------------------------
    def connect(self, mode="sim", channel=None, port=None):
        mode = str(mode or "sim").strip().lower()
        if mode not in ("sim", "real"):
            return {"ok": False, "error": "mode 只能是 sim 或 real", "state": self.state()}
        if mode == "real":
            return {"ok": False, "error": "真机模式还没接（当前只支持 MuJoCo 仿真应用）",
                    "state": self.state()}
        if simapplib is None:
            self.error = "robot/simapp.py 不可用"
            return {"ok": False, "error": self.error, "state": self.state()}
        try:
            self.port = int(port or self.port)
        except (TypeError, ValueError):
            return {"ok": False, "error": "端口号不是整数", "state": self.state()}
        with self._lock:
            self.mode = mode
            if channel is not None:
                self.channel = channel
            if self.client is not None and self.connected:
                return {"ok": True, "state": self.state()}
            if self.client is not None:
                # 残留的死连接（应用已退出、poll 线程已停）：先丢掉再重新拉起
                stale, self.client = self.client, None
                self.connected = False
                self._poll_stop.set()
                try:
                    stale.close()
                except Exception:  # noqa: BLE001
                    pass
            try:
                self.error = ""
                _log("info", "正在启动 MuJoCo 仿真应用（端口 %s）…" % self.port)
                simapplib.ensure_app(port=self.port, timeout=20.0)
                client = simapplib.SimAppClient(port=self.port)
                client.connect()
                self.client = client
                self.connected = True
                self.fail_streak = 0
                self.estop_on = False
                self._poll_stop.clear()
                self._poll_thread = threading.Thread(target=self._poll_loop,
                                                     name="simapp-poll", daemon=True)
                self._poll_thread.start()
                _log("info", "已连接 MuJoCo 仿真应用（%s）" % (client.hello or "HELLO 未知"))
            except Exception as exc:  # noqa: BLE001
                self.client = None
                self.connected = False
                self.error = str(exc) or "%s" % type(exc).__name__
                _log("error", "连接 MuJoCo 仿真应用失败：%s" % self.error)
                return {"ok": False, "error": self.error, "state": self.state()}
        # 读一帧真值，避免前端第一屏是空
        self._refresh_state()
        return {"ok": True, "state": self.state()}

    def _drop_client(self):
        with self._lock:
            client = self.client
            self.client = None
            self.connected = False
            self._poll_stop.set()
            self._poll_thread = None
        try:
            if client is not None:
                client.close()
        except Exception:  # noqa: BLE001
            pass

    def disconnect(self):
        self._play_stop.set()
        with self._lock:
            self.play["state"] = "idle"
            self.play["t"] = 0.0
        self._drop_client()
        self.estop_on = False
        _log("info", "已断开 MuJoCo 仿真应用（应用进程仍在运行）")
        return {"ok": True, "state": self.state()}

    def _poll_loop(self):
        period = 1.0 / POLL_HZ
        while not self._poll_stop.is_set():
            t0 = time.time()
            self._refresh_state()
            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)

    def _app_alive(self):
        """端口上还有没有监听者（不建 TCP 连接，不会和真实 client 抢连接）。"""
        if simapplib is None:
            return False
        try:
            return bool(simapplib._port_listening(port=self.port))
        except Exception:  # noqa: BLE001
            return False

    def _try_reconnect(self):
        """读回连续失败时主动重连一次（socket 可能只是被对端 RST 了）。"""
        if simapplib is None:
            return False
        with self._lock:
            old = self.client
        try:
            if old is not None:
                old.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            cli = simapplib.SimAppClient(port=self.port)
            cli.connect(timeout=2.0)
        except Exception as exc:  # noqa: BLE001
            _log("warn", "重连 MuJoCo 仿真应用失败：%s" % (str(exc) or type(exc).__name__))
            return False
        with self._lock:
            self.client = cli
            self.connected = True
            self.error = ""
            self.fail_streak = 0
        _log("info", "已重新连接 MuJoCo 仿真应用（%s）" % (cli.hello or "HELLO 未知"))
        return True

    def _refresh_state(self):
        client = self.client
        if client is None:
            return
        try:
            raw = client.read_state()
        except Exception as exc:  # noqa: BLE001
            msg = str(exc) or "%s" % type(exc).__name__
            alive = self._app_alive()
            with self._lock:
                self.fail_streak += 1
                streak = self.fail_streak
                self.error = msg
                if streak in (10, 60):
                    _log("warn", "读回仿真应用状态失败（连续 %d 次）：%s" % (streak, msg))
                if not alive:
                    self.connected = False
                    if streak in (3, 30):
                        _log("error", "MuJoCo 仿真应用似乎已退出（端口 %s 没监听）" % self.port)
            # 端口还在 → 应用没死，可能只是这条连接坏了：连续失败就重连一次
            if alive and streak >= 10 and streak % 10 == 0:
                self._try_reconnect()
            return
        with self._lock:
            self.fail_streak = 0
            self.connected = True
            self.error = ""
            self.last = {"deg": [float(v) for v in raw.get("deg") or []],
                         "gripper": float(raw.get("gripper") or 0.0),
                         "ts": time.time()}

    # -- 真值 / FK -------------------------------------------------------
    def _ik_module(self):
        with self._lock:
            if self._ik_tried:
                return self._ik
            self._ik_tried = True
            try:
                from robot.ik import ArmIK  # 纯运动学，允许在 server 里用
                self._ik = ArmIK()
                _log("info", "已加载 robot/ik.py（flange/tcp 由 FK 计算）")
            except Exception as exc:  # noqa: BLE001
                self._ik = None
                _log("warn", "robot/ik.py 不可用（%s）→ flange/tcp 只给 null" % exc)
        return self._ik

    def _fk(self, frame, degs):
        ik = self._ik_module()
        if ik is None or not degs:
            return None, None
        try:
            import numpy as _np
            q = _np.zeros(12, dtype=float)
            for i in range(6):
                q[i] = C.deg_to_dh(i, float(degs[i]))
            pos, R = ik.fk(frame, q)
            from robot.ik import matrix_to_euler_xyz_deg
            return ([round(float(v) * 1000.0, 1) for v in pos],
                    [round(float(v), 2) for v in matrix_to_euler_xyz_deg(R)])
        except Exception as exc:  # noqa: BLE001
            _log("warn", "FK 计算失败（%s）：%s" % (frame, exc))
            return None, None

    def _angles(self):
        with self._lock:
            deg = list(self.last.get("deg") or [])
            grip = self.last.get("gripper")
        if len(deg) < 6:
            deg = list(self.target)
        return deg, grip

    def state(self):
        degs, grip = self._angles()
        f_mm, f_deg = self._fk("flange", degs)
        t_mm, t_deg = self._fk("tcp", degs)
        with self._lock:
            play = {"has_traj": bool(self.play["traj"]),
                    "state": self.play["state"], "t": round(self.play["t"], 3),
                    "duration": round(self.play["duration"], 3),
                    "speed": self.play["speed"], "loop": self.play["loop"]}
            target = list(self.target)
            grip_target = self.grip_target
            connected = self.connected
            estop_on = self.estop_on
            error = self.error
            mode = self.mode
            channel = self.channel
        app = self.app_status()
        st = {
            "ok": True,
            "mode": mode,
            "channel": channel,
            "app": app,
            "connected": bool(connected),
            "ready": bool(connected) and bool(self.client is not None),
            "port": self.port,
            "estop": bool(estop_on),
            "engine": {"name": "MuJoCo", "version": _engine_info().get("version")},
            "joints": {"deg": [round(float(v), 3) for v in degs[:6]],
                       "target": [round(float(v), 3) for v in target[:6]],
                       "range": [list(r) for r in JOINT_RANGE],
                       # limit[i] = 该关节当前是否顶在软限位上（前端用来把数值框描红）
                       # 注意：不是限位数组，限位数组在 range 里
                       "limit": [_at_joint_limit(i, degs[i]) for i in range(6)],
                       "names": list(JOINT_NAMES)},
            "flange": {"mm": f_mm, "deg": f_deg},
            "tcp": {"mm": t_mm, "deg": t_deg, "offset_mm": TCP_MM},
            "gripper": {"deg": None if grip is None else round(float(grip), 2),
                        "target": round(float(grip_target), 2),
                        "range": list(GRIPPER_RANGE)},
            "play": play,
            "ts": round(time.time(), 3),
        }
        if error and not st["connected"]:
            st["error"] = error
        return st

    # -- 下发 ------------------------------------------------------------
    def _refuse(self):
        if self.client is None:
            return "MuJoCo 仿真应用未连接，先点「启动服务器」"
        if self.estop_on:
            return "急停中：目标已冻结在当前位姿，先解除急停再下发"
        return None

    def _send_joints(self, degs):
        self.client.send_joints(degs)
        with self._lock:
            self.target = [float(v) for v in degs]

    def _send_gripper(self, deg):
        self.client.send_gripper(deg)
        with self._lock:
            self.grip_target = float(deg)

    def set_joints(self, degs):
        if not isinstance(degs, (list, tuple)) or len(degs) < 6:
            return {"ok": False, "error": "deg 需要 6 个数字", "state": self.state()}
        try:
            deg6 = [_clamp_joint(i, float(degs[i])) for i in range(6)]
        except (TypeError, ValueError):
            return {"ok": False, "error": "deg 里有非数字", "state": self.state()}
        with self._lock:
            why = self._refuse()
            if why:
                return {"ok": False, "error": why, "state": self.state()}
            try:
                self._send_joints(deg6)
            except Exception as exc:  # noqa: BLE001
                msg = str(exc) or "%s" % type(exc).__name__
                _log("error", "下发关节角度失败：%s" % msg)
                return {"ok": False, "error": msg, "state": self.state()}
        _log("info", "下发关节角度 %s" % " ".join("%.1f" % v for v in deg6))
        # 立刻读一帧：前端拿到的返回体就是"下发后的真值"，不是上一轮缓存
        self._refresh_state()
        return {"ok": True, "state": self.state()}

    def set_gripper(self, deg):
        try:
            value = float(deg)
        except (TypeError, ValueError):
            return {"ok": False, "error": "deg 不是数字", "state": self.state()}
        lo, hi = GRIPPER_RANGE
        value = max(lo, min(hi, value))
        with self._lock:
            why = self._refuse()
            if why:
                return {"ok": False, "error": why, "state": self.state()}
            try:
                self._send_gripper(value)
            except Exception as exc:  # noqa: BLE001
                msg = str(exc) or "%s" % type(exc).__name__
                _log("error", "下发夹爪角度失败：%s" % msg)
                return {"ok": False, "error": msg, "state": self.state()}
        _log("info", "下发夹爪 %.1f°" % value)
        self._refresh_state()
        return {"ok": True, "state": self.state()}

    def estop(self, on=True):
        on = bool(on)
        with self._lock:
            if self.client is None:
                return {"ok": False, "error": "MuJoCo 仿真应用未连接",
                        "state": self.state()}
            try:
                if on:
                    self.client.estop()
                    degs, _g = self._angles()
                    self.target = [float(v) for v in degs[:6]]
                    self.play["state"] = "idle"
                    self._play_stop.set()
                self.estop_on = on
            except Exception as exc:  # noqa: BLE001
                msg = str(exc) or "%s" % type(exc).__name__
                _log("error", "急停指令失败：%s" % msg)
                return {"ok": False, "error": msg, "state": self.state()}
        _log("warn" if on else "info",
             "急停：目标 = 当前实测，机械臂停住" if on else "急停已解除，可以继续下发")
        return {"ok": True, "state": self.state()}

    def home(self):
        with self._lock:
            why = self._refuse()
            if why:
                return {"ok": False, "error": why, "state": self.state()}
            try:
                self._send_joints([_clamp_joint(i, OFFSET[i]) for i in range(6)])
                self._send_gripper(0.0)
            except Exception as exc:  # noqa: BLE001
                msg = str(exc) or "%s" % type(exc).__name__
                _log("error", "回默认位失败：%s" % msg)
                return {"ok": False, "error": msg, "state": self.state()}
        _log("info", "回默认位（convention.OFFSET）")
        return {"ok": True, "state": self.state()}

    def goto_pose(self, frame, mm, degs):
        frame = str(frame or "tcp").strip().lower()
        if frame not in ("tcp", "flange"):
            return {"ok": False, "error": "frame 只能是 tcp 或 flange",
                    "state": self.state()}
        try:
            mm3 = [float(v) for v in (mm or [])][:3]
            deg3 = [float(v) for v in (degs or [])][:3]
        except (TypeError, ValueError):
            return {"ok": False, "error": "mm/deg 里必须都是数字", "state": self.state()}
        if len(mm3) != 3 or len(deg3) != 3:
            return {"ok": False, "error": "mm 与 deg 都要 3 个数", "state": self.state()}
        ik = self._ik_module()
        if ik is None:
            return {"ok": False, "error": "robot/ik.py 不可用，无法解 IK",
                    "state": self.state()}
        with self._lock:
            why = self._refuse()
            if why:
                return {"ok": False, "error": why, "state": self.state()}
            try:
                import numpy as _np
                R = self._euler_xyz_to_R(deg3)
                pos = _np.array(mm3, dtype=float) / 1000.0
                cur, _g = self._angles()
                q_seed = _np.zeros(12, dtype=float)
                for i in range(6):
                    q_seed[i] = C.deg_to_dh(i, float(cur[i]))
                res = ik.solve_pose(frame, pos, R, q_seed=q_seed)
                status = str(res.get("status") or "unreachable")
                info = {"status": status, "text": IK_TEXT.get(status, status),
                        "pos_err_mm": round(float(res.get("pos_err_mm") or 0.0), 2),
                        "ori_err_deg": round(float(res.get("ori_err_deg") or 0.0), 2)}
                if status != "ok":
                    _log("warn", "pose 目标不可达（%s）：%s" % (status, info["text"]))
                    return {"ok": False, "ik": info, "error": info["text"],
                            "state": self.state()}
                q = res.get("q")
                deg6 = [_clamp_joint(i, C.dh_to_deg(i, float(q[i]))) for i in range(6)]
                self._send_joints(deg6)
            except Exception as exc:  # noqa: BLE001
                msg = str(exc) or "%s" % type(exc).__name__
                _log("error", "goto_pose 失败：%s" % msg)
                return {"ok": False, "error": msg, "state": self.state()}
        _log("info", "已按 %s 位姿下发 IK 结果 %s"
             % (frame, " ".join("%.1f" % v for v in deg6)))
        return {"ok": True, "ik": info, "state": self.state()}

    @staticmethod
    def _euler_xyz_to_R(deg):
        try:
            import numpy as np
        except Exception:  # noqa: BLE001
            return None
        rx, ry, rz = [math.radians(float(v)) for v in deg]
        cx, sx = math.cos(rx), math.sin(rx)
        cy, sy = math.cos(ry), math.sin(ry)
        cz, sz = math.cos(rz), math.sin(rz)
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=float)
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=float)
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=float)
        return Rz @ Ry @ Rx

    # -- 规划 / 播放（50Hz 流式 send_joints）------------------------------
    def plan(self, waypoints, speed=1.0):
        try:
            speed = max(0.05, min(5.0, float(speed)))
        except (TypeError, ValueError):
            speed = 1.0
        if not isinstance(waypoints, (list, tuple)) or not waypoints:
            return {"ok": False, "error": "waypoints 必须是非空数组",
                    "state": self.state()}
        planner = _planner_mod()
        if planner is None:
            return {"ok": False, "error": "robot/planner.py 不可用，无法规划",
                    "state": self.state()}
        wps = []
        for i, wp in enumerate(waypoints):
            try:
                wps.append(_wp_fields(wp))
            except (TypeError, ValueError, IndexError, KeyError):
                return {"ok": False, "error": "第 %d 个路点格式不对" % (i + 1),
                        "state": self.state()}
        # robot/planner.py 吃「米」：[x, y, z, dwell, grip]；grip 统一成 0–110 角度
        plans = [[float(w["x"]), float(w["y"]), float(w["z"]), w["hold"], w["grip"]]
                 for w in wps]
        import numpy as _np
        q_start = _np.zeros(12, dtype=float)
        degs, _g = self._angles()
        for i in range(6):
            q_start[i] = C.deg_to_dh(i, float(degs[i]))
        ik = self._ik_module()
        try:
            res = planner.plan_waypoints(plans, speed=speed, q_start=q_start, ik=ik,
                                         dt=1.0 / PLAY_HZ)
        except Exception as exc:  # noqa: BLE001
            msg = "%s: %s" % (type(exc).__name__, exc)
            _log("error", "规划失败：%s" % msg)
            return {"ok": False, "error": "规划失败：%s" % msg, "state": self.state()}
        traj = []
        try:
            for item in res.get("traj") or []:
                t = float(item[0])
                deg6 = [float(v) for v in list(item[1])[:6]]
                grip = float(item[2]) if len(item) > 2 and item[2] is not None else None
                traj.append([t, deg6, grip])
        except Exception:  # noqa: BLE001
            traj = []
        if not res.get("ok") or not traj:
            text = str(res.get("err") or "规划失败")
            _log("warn", "规划未通过：%s" % text)
            return {"ok": False, "error": text, "reachable": bool(res.get("reachable")),
                    "wp": [_wp_fields(w) for w in waypoints], "state": self.state()}
        with self._lock:
            self.play = {"traj": traj, "state": "idle", "t": 0.0,
                         "duration": round(float(res.get("duration") or traj[-1][0]), 3),
                         "speed": speed, "loop": False}
        _log("done", "规划完成：%d 个路点，%.1fs（%d 帧）"
             % (len(wps), self.play["duration"], len(traj)))
        return {"ok": True, "duration": self.play["duration"],
                "reachable": bool(res.get("reachable")),
                "wp": res.get("wp") or [], "err": res.get("err") or "",
                "state": self.state()}

    def _traj_frame(self, t):
        with self._lock:
            traj = self.play["traj"]
        if not traj:
            return None
        if t <= traj[0][0]:
            return traj[0]
        if t >= traj[-1][0]:
            return traj[-1]
        lo, hi = 0, len(traj) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if traj[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, d0, g0 = traj[lo]
        t1, d1, g1 = traj[hi]
        span = max(1e-6, t1 - t0)
        k = (t - t0) / span
        deg6 = [d0[i] + (d1[i] - d0[i]) * k for i in range(6)]
        grip = None
        if g0 is not None and g1 is not None:
            grip = g0 + (g1 - g0) * k
        elif g1 is not None:
            grip = g1
        elif g0 is not None:
            grip = g0
        return [t, deg6, grip]

    def play_action(self, action="play", speed=1.0, loop=False):
        action = str(action or "play").strip().lower()
        if action not in ("play", "pause", "stop"):
            return {"ok": False, "error": "action 只能是 play / pause / stop",
                    "state": self.state()}
        with self._lock:
            why = self._refuse()
            if why and action == "play":
                return {"ok": False, "error": why, "state": self.state()}
            if not self.play["traj"]:
                return {"ok": False, "error": "还没有规划轨迹，先调 /api/v2/plan",
                        "state": self.state()}
            try:
                self.play["speed"] = max(0.05, min(5.0, float(speed or 1.0)))
            except (TypeError, ValueError):
                return {"ok": False, "error": "speed 不是数字", "state": self.state()}
            self.play["loop"] = bool(loop)
            if action == "stop":
                self.play["state"] = "idle"
                self.play["t"] = 0.0
                self._play_stop.set()
            elif action == "pause":
                self.play["state"] = "paused"
                self._play_stop.set()
            else:
                self.play["state"] = "playing"
                if self.play["t"] >= self.play["duration"]:
                    self.play["t"] = 0.0
                self._play_stop.clear()
                if self._play_thread is None or not self._play_thread.is_alive():
                    self._play_thread = threading.Thread(target=self._play_loop,
                                                         name="simapp-play", daemon=True)
                    self._play_thread.start()
        _log("info", "轨迹 %s（%.1fs，%.2f×）"
             % ({'play': '播放', 'pause': '暂停', 'stop': '停止'}[action],
                self.play["duration"], self.play["speed"]))
        return {"ok": True, "action": action, "state": self.state()}

    # 注意：self.play 是状态字典，所以方法名不能叫 play（api_play 调 play_action）


    def _play_loop(self):
        """50Hz 把轨迹流给仿真应用；暂停/停止/播完即退出线程。"""
        while True:
            if self._play_stop.is_set():
                return
            t0 = time.time()
            frame, client, playing = None, None, False
            with self._lock:
                if self.play["state"] == "playing" and self.client is not None:
                    playing = True
                    self.play["t"] += PLAY_TICK * float(self.play["speed"])
                    dur = float(self.play["duration"])
                    if self.play["t"] >= dur:
                        if self.play["loop"]:
                            self.play["t"] = 0.0
                        else:
                            self.play["t"] = dur
                            self.play["state"] = "done"
                    frame = self._traj_frame(self.play["t"])
                    client = self.client
            if playing and frame is not None and client is not None:
                try:
                    degs = [_clamp_joint(i, float(frame[1][i])) for i in range(6)]
                    client.send_joints(degs)
                    with self._lock:
                        self.target = degs
                    if frame[2] is not None:
                        gdeg = max(GRIPPER_RANGE[0], min(GRIPPER_RANGE[1], frame[2]))
                        client.send_gripper(gdeg)
                        with self._lock:
                            self.grip_target = float(gdeg)
                except Exception as exc:  # noqa: BLE001
                    _log("error", "播放下发失败：%s" % exc)
                    with self._lock:
                        self.play["state"] = "paused"
            if self.play_state() in ("done", "idle", "paused"):
                if self.play_state() != "playing":
                    time.sleep(PLAY_TICK)
                    if self._play_stop.is_set():
                        return
                    with self._lock:
                        if self.play["state"] != "playing":
                            return
            dt = PLAY_TICK - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)

    def play_state(self):
        with self._lock:
            return self.play["state"]

    def seek(self, t):
        try:
            t = float(t)
        except (TypeError, ValueError):
            return {"ok": False, "error": "t 不是数字", "state": self.state()}
        with self._lock:
            why = self._refuse()
            if why:
                return {"ok": False, "error": why, "state": self.state()}
            if not self.play["traj"]:
                return {"ok": False, "error": "还没有规划轨迹，先调 /api/v2/plan",
                        "state": self.state()}
            t = max(0.0, min(self.play["duration"], t))
            self.play["t"] = t
            self.play["state"] = "paused"
            self._play_stop.set()
            frame = self._traj_frame(t)
            if frame is not None:
                try:
                    self._send_joints([_clamp_joint(i, frame[1][i]) for i in range(6)])
                    if frame[2] is not None:
                        self._send_gripper(max(GRIPPER_RANGE[0],
                                               min(GRIPPER_RANGE[1], frame[2])))
                except Exception as exc:  # noqa: BLE001
                    return {"ok": False, "error": "跳转下发失败：%s" % exc,
                            "state": self.state()}
        return {"ok": True, "t": t, "state": self.state()}

    # -- 场景 / 相机（预览用，只存在 server 侧）---------------------------
    def load(self, model_xml, scene_xml):
        return {"ok": False, "scene": {"model": model_xml, "scene": scene_xml},
                "error": "换模型/场景需要重启仿真应用（当前版本不支持热加载）",
                "state": self.state()}

    def set_cam(self, az=None, el=None, dist=None, lookat=None):
        for key, val in (("az", az), ("el", el), ("dist", dist)):
            if val is None:
                continue
            try:
                self.cam[key] = float(val)
            except (TypeError, ValueError):
                return {"ok": False, "error": "%s 不是数字" % key, "cam": dict(self.cam)}
        if lookat is not None:
            try:
                look = [float(v) for v in lookat][:3]
            except (TypeError, ValueError):
                return {"ok": False, "error": "lookat 必须全是数字", "cam": dict(self.cam)}
            if len(look) != 3:
                return {"ok": False, "error": "lookat 需要 3 个数", "cam": dict(self.cam)}
            self.cam["lookat"] = look
        self.cam["dist"] = max(0.15, min(8.0, self.cam["dist"]))
        return {"ok": True, "cam": dict(self.cam)}

    def reset_cam(self):
        self.cam = {"az": DEFAULT_CAM["az"], "el": DEFAULT_CAM["el"],
                    "dist": DEFAULT_CAM["dist"], "lookat": list(DEFAULT_CAM["lookat"])}
        return {"ok": True, "cam": dict(self.cam)}


HOLDER = SimController()


def _engine_info():
    """MuJoCo 版本（仿真应用与本进程共用同一个 mujoco 包）。"""
    version = "unavailable"
    try:
        import mujoco
        version = str(getattr(mujoco, "__version__", "unknown"))
    except Exception:  # noqa: BLE001
        pass
    return {"name": "MuJoCo", "version": version}


def _ensure_holder():
    """兼容旧调用：现在没有「惰性启动仿真」这回事，只保证应用状态可用。"""
    return HOLDER.app_status()

# ---------------------------------------------------------------------------
# HTTP 小工具（与 app.py 的 _no_cache 等价：允许 file:// 直接打开页面）
# ---------------------------------------------------------------------------
def _cors(handler, cache=True):
    handler.send_header("Access-Control-Allow-Origin", "*")
    handler.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
    handler.send_header("Access-Control-Allow-Headers", "Content-Type")
    handler.send_header("Access-Control-Max-Age", "600")
    if cache:
        handler.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        handler.send_header("Pragma", "no-cache")
        handler.send_header("Expires", "0")


def _send_json(handler, payload, status=200):
    body = json.dumps(payload, ensure_ascii=False, default=str).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    _cors(handler)
    handler.send_header("X-Content-Type-Options", "nosniff")
    handler.end_headers()
    if getattr(handler, "command", "GET") != "HEAD":
        handler.wfile.write(body)


def _send_html(handler, html, status=200):
    body = html.encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "text/html; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    _cors(handler)
    handler.end_headers()
    if getattr(handler, "command", "GET") != "HEAD":
        handler.wfile.write(body)


def _read_body(handler):
    """POST 的 JSON body，防御式解析：任何异常都返回 (None, 中文错误)。"""
    try:
        raw_len = handler.headers.get("Content-Length")
        length = int(raw_len or 0)
    except (TypeError, ValueError):
        return None, "Content-Length 非法"
    if length < 0:
        return None, "Content-Length 非法"
    if length > MAX_BODY:
        return None, "请求体过大"
    if not length:
        return {}, None
    try:
        raw = handler.rfile.read(length)
    except Exception as exc:  # noqa: BLE001
        return None, "读取请求体失败：%s" % exc
    try:
        body = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        return None, "请求体不是合法 JSON"
    if not isinstance(body, dict):
        return None, "请求体必须是 JSON 对象"
    return body, None


def _as_int(value, default):
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


# ---------------------------------------------------------------------------
# 页面别名（/v2/base、/v2/viz 由另外两个 agent 写 web/v2/*.html）
# ---------------------------------------------------------------------------
PAGE_ALIASES = {
    "/v2/base": "v2/base.html",
    "/v2/viz": "v2/viz.html",
}

_MISSING_PAGE = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>页面还没就位 · 上位机 V2</title>
<style>
 body{background:#0d1117;color:#e6edf3;font:15px/1.7 -apple-system,"PingFang SC",sans-serif;
      display:flex;min-height:100vh;align-items:center;justify-content:center;margin:0}
 .box{max-width:560px;padding:28px 30px;border:1px solid #30363d;border-radius:14px;background:#161b22}
 h1{font-size:18px;margin:0 0 12px} code{color:#79c0ff}
 a{color:#58a6ff;text-decoration:none} p{margin:8px 0;color:#8b949e}
</style></head><body><div class="box">
<h1>404 · 页面还没生成好</h1>
<p>路由 <code>%(path)s</code> 已经接好，但文件 <code>%(file)s</code> 还不存在。</p>
<p>这个页面由另一个 agent 并行创建，稍后刷新即可。</p>
<p>后端接口是好的：<a href="/api/v2/state">/api/v2/state</a> ·
   <a href="/api/v2/selftest">/api/v2/selftest</a> ·
   <a href="/api/v2/logs">/api/v2/logs</a></p>
</div></body></html>
"""


def _serve_page(handler, method, rel, url):
    if method not in ("GET", "HEAD"):
        _send_json(handler, {"ok": False, "error": "页面只支持 GET"}, 405)
        return
    target = (WEB_DIR / rel).resolve()
    if target != WEB_DIR and WEB_DIR not in target.parents:
        _send_json(handler, {"ok": False, "error": "禁止访问"}, 403)
        return
    if not target.is_file():
        _send_html(handler, _MISSING_PAGE % {"path": url, "file": rel}, 404)
        return
    try:
        data = target.read_bytes()
    except OSError as exc:
        _send_html(handler, "<h1>读取失败</h1><p>%s</p>" % exc, 500)
        return
    handler.send_response(200)
    handler.send_header("Content-Type", "text/html; charset=utf-8")
    handler.send_header("Content-Length", str(len(data)))
    _cors(handler)
    handler.end_headers()
    if method != "HEAD":
        handler.wfile.write(data)


# ---------------------------------------------------------------------------
# 流：chunked 手写分块
# ---------------------------------------------------------------------------
def _write_chunk(handler, data):
    handler.wfile.write(b"%x\r\n" % len(data) + data + b"\r\n")
    handler.wfile.flush()


def _end_stream(handler):
    try:
        handler.wfile.write(b"0\r\n\r\n")
        handler.wfile.flush()
    except Exception:  # noqa: BLE001
        pass


def _start_stream(handler, content_type):
    handler.send_response(200)
    handler.send_header("Content-Type", content_type)
    handler.send_header("Transfer-Encoding", "chunked")
    handler.send_header("Connection", "keep-alive")
    handler.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
    handler.send_header("X-Accel-Buffering", "no")
    _cors(handler, cache=False)   # 缓存头上面已写，避免重复
    handler.end_headers()
    handler.wfile.flush()


def _serve_sse(handler, query):
    """契约 §2：10Hz 推 {"seq","state","log"}。"""
    if not HOLDER.acquire_stream():
        _send_json(handler, {"ok": False, "error": "事件流连接数已满"}, 503)
        return
    try:
        since = _as_int((query.get("since") or ["0"])[0], 0)
        _start_stream(handler, "text/event-stream; charset=utf-8")
        seq = since
        cursor = max(0, since)
        idle = 0
        while True:
            t0 = time.time()
            state = HOLDER.state()
            logs = LOGS.since(cursor)
            if logs:
                cursor = logs[-1]["seq"]
            seq += 1
            frame = {"seq": seq, "state": state, "log": logs}
            payload = json.dumps(frame, ensure_ascii=False, default=str)
            _write_chunk(handler, ("data: " + payload + "\n\n").encode("utf-8"))
            idle += 1
            if idle >= 150:            # 15s 一次心跳注释，防中间层超时断开
                _write_chunk(handler, b": ping\n\n")
                idle = 0
            dt = 0.1 - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
        pass
    except OSError:
        pass
    except Exception:  # noqa: BLE001
        traceback.print_exc()
    finally:
        _end_stream(handler)
        HOLDER.release_stream()


def _sniff_image(payload):
    # 无 pillow：Arm 返回什么就发什么，默认按契约写 image/png，若字节其实是
    # JPEG（FFD8）就纠正成 image/jpeg，避免浏览器 nosniff 之后不出图。
    if payload[:3] == b"\xff\xd8\xff":
        return "image/jpeg"
    return "image/png"


def _serve_mjpg(handler, query):
    """契约 §4：multipart/x-mixed-replace，约 30fps。"""
    if not HOLDER.acquire_stream():
        _send_json(handler, {"ok": False, "error": "图流连接数已满"}, 503)
        return
    try:
        w = max(160, min(1920, _as_int((query.get("w") or ["672"])[0], 672)))
        h = max(120, min(1080, _as_int((query.get("h") or ["456"])[0], 456)))
        boundary = "mjpgframe"
        _start_stream(handler, "multipart/x-mixed-replace; boundary=%s" % boundary)
        # 契约 V2.1：页面的 3D 是本地 three.js 预览，这里只回一张静态占位图，
        # 不再从 MuJoCo 窗口抓画面（也不允许抓）。旧页面里的 <img> 不会 404。
        while True:
            t0 = time.time()
            data = mock_png(w, h)
            head = ("--%s\r\nContent-Type: %s\r\nContent-Length: %d\r\n\r\n"
                    % (boundary, _sniff_image(data), len(data))).encode("ascii")
            _write_chunk(handler, head + data + b"\r\n")
            dt = (1.0 / 30.0) - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
        pass
    except OSError:
        pass
    except Exception:  # noqa: BLE001
        traceback.print_exc()
    finally:
        _end_stream(handler)
        HOLDER.release_stream()


# ---------------------------------------------------------------------------
# 接口实现（契约 §1..§5）
# ---------------------------------------------------------------------------
def api_state(_query, _body):
    return HOLDER.state()


def api_connect(_query, body):
    return HOLDER.connect(mode=body.get("mode", "sim"),
                          channel=body.get("channel"),
                          port=body.get("port"))


def api_disconnect(_query, _body):
    return HOLDER.disconnect()


def api_app(_query, body):
    """契约 §3 /app：{action: start|stop|status}。起/停独立 MuJoCo 仿真应用。"""
    action = (body or {}).get("action", "status")
    return HOLDER.app(action)


def api_joints(_query, body):
    return HOLDER.set_joints(body.get("deg"))


def api_gripper(_query, body):
    return HOLDER.set_gripper(body.get("deg"))


def api_estop(_query, body):
    on = True
    if isinstance(body, dict) and "on" in body:
        on = bool(body.get("on"))
    return HOLDER.estop(on)


def api_home(_query, _body):
    return HOLDER.home()


def api_pose(_query, body):
    frame = body.get("frame", "tcp")
    mm = body.get("mm")
    deg = body.get("deg")
    return HOLDER.goto_pose(frame, mm, deg)


def api_plan(_query, body):
    raw = body.get("waypoints")
    if not isinstance(raw, list) or not raw:
        return {"ok": False, "error": "waypoints 必须是非空数组",
                "state": HOLDER.state()}
    # 路点格式在 SimController.plan 里统一校验，非法时返回中文错误而不是 500
    return HOLDER.plan(raw, body.get("speed", 1.0))


def api_play(_query, body):
    return HOLDER.play_action(body.get("action", "play"),
                              body.get("speed", 1.0), body.get("loop", False))


def api_play_seek(_query, body):
    return HOLDER.seek(body.get("t", 0.0))


def api_generate(_query, body):
    """生成独立可跑的路点脚本（robot/scriptgen.py 真正写文件，不用旧 layout 模块）。"""
    raw_wps = body.get("waypoints")
    if not isinstance(raw_wps, list) or not raw_wps:
        return {"ok": False, "error": "waypoints 必须是非空数组", "state": HOLDER.state()}
    name = str(body.get("name") or "pick_and_place.py").strip().replace("\\", "/")
    if not name:
        name = "pick_and_place"
    if not name.endswith(".py"):
        name += ".py"
    stem = name[:-3]
    if scriptgenlib is None:
        return {"ok": False, "error": "robot/scriptgen.py 不可用，无法生成脚本",
                "state": HOLDER.state()}
    if not scriptgenlib.NAME_RE.match(name):
        return {"ok": False, "error": "脚本名只能包含字母、数字、下划线、短横线、点（不能以符号开头）",
                "state": HOLDER.state()}
    try:
        speed = float(body.get("speed", 1.0) or 1.0)
    except (TypeError, ValueError):
        return {"ok": False, "error": "speed 不是数字", "state": HOLDER.state()}
    try:
        result = scriptgenlib.generate(name, raw_wps, speed=speed)
    except Exception as exc:  # noqa: BLE001  契约：绝不 500
        _log("error", "生成脚本异常：%s: %s" % (type(exc).__name__, exc))
        return {"ok": False, "error": "生成脚本失败：%s" % exc, "state": HOLDER.state()}
    if not isinstance(result, dict):
        return {"ok": False, "error": "生成脚本失败：返回格式异常", "state": HOLDER.state()}
    if not result.get("ok"):
        return {"ok": False, "error": str(result.get("error") or "生成脚本失败"),
                "state": HOLDER.state()}
    _log("done", "脚本已生成：%s（%d 个路点）"
         % (result.get("path"), int(result.get("waypoints") or 0)))
    out = {"ok": True, "name": stem, "path": result.get("path"),
           "abs_path": result.get("abs_path"),
           "waypoints": int(result.get("waypoints") or 0),
           "speed": float(result.get("speed") or speed),
           "state": HOLDER.state()}
    # 顺手把时长算好落盘，供脚本库列表直接显示（列表接口不再现场规划）
    try:
        mod = _planner_mod()
        abs_path = result.get("abs_path")
        if mod is not None and abs_path:
            res = mod.plan_waypoints(raw_wps, speed)
            dur = round(float(res.get("duration") or 0.0), 3)
            mt = round(os.path.getmtime(abs_path), 3)
            _DUR_CACHE[duration_key(os.path.basename(str(abs_path)), mt)] = dur
            _save_dur_cache()
            out["duration"] = dur
    except Exception as exc:  # noqa: BLE001
        _log("warn", "算脚本时长失败：%s" % exc)
    return out


def api_scene(_query, body):
    model = _norm_path(body.get("model"), DEFAULT_MODEL_XML)
    scene = _norm_path(body.get("scene"), DEFAULT_SCENE_XML)
    for path, label in ((model, "模型"), (scene, "场景")):
        if _rel_file(path) is None:
            return {"ok": False, "error": "%s不存在：%s" % (label, path),
                    "state": HOLDER.state()}
    if str(HOLDER.mode) != "sim":
        return {"ok": False, "error": "换模型/场景只支持 sim 模式",
                "state": HOLDER.state()}
    return HOLDER.load(model, scene)


def api_cam(_query, body):
    return HOLDER.set_cam(az=body.get("az"), el=body.get("el"),
                          dist=body.get("dist"), lookat=body.get("lookat"))


def api_cam_reset(_query, _body):
    return HOLDER.reset_cam()


def api_logs(query, _body):
    since = _as_int((query.get("since") or ["0"])[0], 0)
    return {"ok": True, "items": LOGS.since(since)}


def api_limits(_query, _body):
    joints = []
    for i, name in enumerate(JOINT_NAMES):
        step = 1.0
        try:
            step = round(float(C.readback_step_deg(i)), 4)
        except Exception:  # noqa: BLE001
            step = 1.0
        joints.append({"name": name, "range": list(JOINT_RANGE[i]), "step": step})
    return {"ok": True, "joints": joints,
            "gripper": {"range": list(GRIPPER_RANGE), "step": 1.0},
            "tcp_offset_mm": TCP_MM}


_PLANNER = None
_PLANNER_TRIED = False
_DUR_CACHE = {}
_DUR_MAX_PER_CALL = 10


def _planner_mod():
    """robot.planner（真正算时长的那个）；不可用返回 None。"""
    global _PLANNER, _PLANNER_TRIED
    if _PLANNER is None and not _PLANNER_TRIED:
        _PLANNER_TRIED = True
        try:
            from robot import planner as plannerlib  # noqa: E402
            _PLANNER = plannerlib
        except Exception:  # noqa: BLE001
            _PLANNER = None
    return _PLANNER


_DUR_FILE = SCRIPTS_DIR / ".durations.json"


def _load_dur_cache():
    """从 scripts/.durations.json 读回已算好的时长（重启后仍可用，避免列表接口现场规划）。"""
    try:
        with open(_DUR_FILE, "r", encoding="utf-8") as fp:
            data = json.load(fp)
    except Exception:  # noqa: BLE001
        return
    if not isinstance(data, dict):
        return
    for name, item in data.items():
        try:
            _DUR_CACHE[(name, round(float(item["mtime"]), 3))] = round(float(item["duration"]), 3)
        except Exception:  # noqa: BLE001
            continue


def _save_dur_cache():
    data = {}
    for (name, mtime), dur in _DUR_CACHE.items():
        data[str(name)] = {"mtime": mtime, "duration": dur}
    try:
        SCRIPTS_DIR.mkdir(parents=True, exist_ok=True)
        with open(_DUR_FILE, "w", encoding="utf-8") as fp:
            json.dump(data, fp, ensure_ascii=False, indent=1)
    except Exception:  # noqa: BLE001
        pass


def duration_key(fname, mtime):
    return (str(fname), round(float(mtime), 3) if mtime is not None else None)


def _script_duration(path, waypoints, mtime, budget):
    """只查缓存：时长由 /api/v2/generate 生成时算好并落盘，列表接口绝不现场规划。

    现场规划一个够不到的路点要几十秒，会让网页一直转圈（实测 test1.py 冷启动 48 s）。
    """
    return _DUR_CACHE.get(duration_key(path.name, mtime), 0.0)


_load_dur_cache()


def _scan_scripts():
    items = []
    if not SCRIPTS_DIR.is_dir():
        return items
    try:
        files = [p for p in SCRIPTS_DIR.glob("*.py")
                 if p.is_file() and not p.name.startswith(".")]
        files.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    except OSError:
        return items
    budget = [_DUR_MAX_PER_CALL]
    for path in files[:60]:
        waypoints, duration = [], 0.0
        if layoutlib is not None:
            try:
                meta = layoutlib.load_script(path)
                waypoints = meta.get("waypoints") or []
            except Exception:  # noqa: BLE001
                waypoints = []
        try:
            mtime = round(path.stat().st_mtime, 3)
        except OSError:
            mtime = None
        # 时长按 robot.planner 真规划算（旧 layout.duration_seconds 是 UR5e 时代的假数）
        duration = _script_duration(path, waypoints, mtime, budget)
        items.append({"name": path.stem, "waypoints": waypoints,
                      "duration": duration, "path": "scripts/%s" % path.name,
                      "mtime": mtime})
    return items


def api_scripts(_query, _body):
    return {"ok": True, "items": _scan_scripts()}

def api_selftest(_query, _body):
    """契约 §3：自检项都必须是「真的跑过」：应用能起、能连、能下发、能读回、急停有效。"""
    checks = []

    def check(name, fn):
        try:
            ok, detail = fn()
        except Exception as exc:  # noqa: BLE001
            ok, detail = False, "%s: %s" % (type(exc).__name__, exc)
        checks.append({"name": name, "ok": bool(ok), "detail": str(detail)})

    check("项目根", lambda: (WORK4_DIR.is_dir(), str(WORK4_DIR)))

    def _mujoco():
        import mujoco
        return True, "mujoco %s" % getattr(mujoco, "__version__", "?")

    check("MuJoCo", _mujoco)
    check("机械臂模型", lambda: (_rel_file(DEFAULT_MODEL_XML) is not None,
                           str(WORK4_DIR / DEFAULT_MODEL_XML)))
    check("场景模型", lambda: (_rel_file(DEFAULT_SCENE_XML) is not None,
                           str(WORK4_DIR / DEFAULT_SCENE_XML)))
    check("仿真应用模块", lambda: (simapplib is not None,
                             "robot/simapp.py 可用" if simapplib else "robot/simapp.py 不可用"))
    check("仿真脚本", lambda: ((WORK4_DIR / "bridge" / "sim_bridge.py").is_file(),
                           str(WORK4_DIR / "bridge" / "sim_bridge.py")))

    def _app():
        if HOLDER.connected and HOLDER.client is not None:
            info = HOLDER.app_status()
            return True, "已在运行 · pid=%s · TCP %s" % (info.get("pid"), HOLDER.port)
        res = HOLDER.connect()
        if not res.get("ok"):
            return False, "启动/连接失败：%s" % (res.get("error") or "未知原因")
        info = HOLDER.app_status()
        return True, "已启动并连接 · %s · TCP %s" % (
            ("pid=%s" % info.get("pid")) if info.get("pid") else "外部进程", HOLDER.port)

    check("仿真应用可启动/可连接", _app)

    def _send_read():
        if HOLDER.client is None:
            return False, "未连接仿真应用"
        # 用默认位（convention.OFFSET）当目标：当前位置可能正好顶在软限位上，
        # 拿它自己当目标会因为阻尼/限幅永远差几度，属误报
        want = [float(v) for v in OFFSET]
        res = HOLDER.estop(False)
        if not res.get("ok"):
            return False, "解除急停失败：%s" % (res.get("error") or "未知原因")
        res = HOLDER.set_joints(want)
        if not res.get("ok"):
            return False, "下发失败：%s" % (res.get("error") or "未知原因")
        deadline = time.time() + 6.0
        worst = None
        while time.time() < deadline:
            time.sleep(0.2)
            got = HOLDER.state()["joints"]["deg"]
            if len(got) < 6:
                continue
            worst = max(abs(float(got[i]) - want[i]) for i in range(6))
            if worst < 1.0:
                break
        if worst is None:
            return False, "没有读到关节回读"
        return worst < 1.0, "下发 %s · 最大回读误差 %.2f°" % (want, worst)

    check("可下发且回读一致（<1°）", _send_read)

    def _estop():
        if HOLDER.client is None:
            return False, "未连接仿真应用"
        res = HOLDER.estop(True)
        if not res.get("ok"):
            return False, "急停指令失败：%s" % res.get("error")
        a = HOLDER.state()["joints"]["deg"]
        time.sleep(0.8)
        b = HOLDER.state()["joints"]["deg"]
        drift = max(abs(float(a[i]) - float(b[i])) for i in range(6))
        HOLDER.estop(False)
        return drift < 2.0, "急停后 0.8s 漂移 %.2f°（阈值 2°），已解除急停" % drift

    check("急停有效", _estop)

    def _plan():
        planner = _planner_mod()
        if planner is None:
            return False, "robot/planner.py 不可用"
        degs, _g = HOLDER._angles()
        import numpy as _np
        q0 = _np.zeros(12, dtype=float)
        for i in range(6):
            q0[i] = C.deg_to_dh(i, float(degs[i]))
        # planner 要「米」的列表路点：[x, y, z, dwell, grip]
        # robot/planner.py 至少需要 2 个路点（1 个路点时内部 phases 为空会 IndexError）
        res = planner.plan_waypoints([[0.250, 0.080, 0.260, 0.0, None],
                                      [0.250, 0.080, 0.210, 0.2, None]], speed=1.0,
                                     q_start=q0, ik=HOLDER._ik_module(), dt=1.0 / PLAY_HZ)
        if not res.get("ok"):
            return False, "规划未通过：%s" % (res.get("err") or "未知原因")
        return True, "2 个路点 · %.2fs · %d 帧" % (float(res.get("duration") or 0.0),
                                                  len(res.get("traj") or []))

    check("规划可用", _plan)

    def _generate():
        if scriptgenlib is None:
            return False, "robot/scriptgen.py 不可用"
        return True, "robot/scriptgen.py 可用（写 %s）" % SCRIPTS_DIR

    check("生成脚本可用", _generate)
    check("脚本目录可写", lambda: (SCRIPTS_DIR.is_dir() and os.access(str(SCRIPTS_DIR), os.W_OK),
                             str(SCRIPTS_DIR)))
    check("页面 /v2/base", lambda: ((WEB_DIR / "v2" / "base.html").is_file(),
                                str(WEB_DIR / "v2" / "base.html")))
    def _version():
        info = HOLDER.app_status()
        who = ("pid=%s" % info.get("pid")) if info.get("pid") else (
            "外部进程" if info.get("running") else "未运行")
        return True, "v%s · 仿真应用 %s" % (VERSION, who)

    check("接口版本", _version)
    return {"ok": all(c["ok"] for c in checks), "checks": checks}

# ---------------------------------------------------------------------------
# 路由
# ---------------------------------------------------------------------------
ROUTES = {
    ("GET", "/api/v2/state"): api_state,
    ("GET", "/api/v2/app"): api_app,
    ("POST", "/api/v2/app"): api_app,
    ("POST", "/api/v2/connect"): api_connect,
    ("POST", "/api/v2/disconnect"): api_disconnect,
    ("POST", "/api/v2/joints"): api_joints,
    ("POST", "/api/v2/gripper"): api_gripper,
    ("POST", "/api/v2/estop"): api_estop,
    ("POST", "/api/v2/home"): api_home,
    ("POST", "/api/v2/pose"): api_pose,
    ("POST", "/api/v2/plan"): api_plan,
    ("POST", "/api/v2/play"): api_play,
    ("POST", "/api/v2/play/seek"): api_play_seek,
    ("POST", "/api/v2/generate"): api_generate,
    ("POST", "/api/v2/scene"): api_scene,
    ("POST", "/api/v2/cam"): api_cam,
    ("POST", "/api/v2/cam/reset"): api_cam_reset,
    ("GET", "/api/v2/logs"): api_logs,
    ("GET", "/api/v2/limits"): api_limits,
    ("GET", "/api/v2/scripts"): api_scripts,
    ("GET", "/api/v2/selftest"): api_selftest,
}

STREAM_ROUTES = {
    "/api/v2/events": _serve_sse,
    "/api/v2/stream.mjpg": _serve_mjpg,
}


def dispatch(handler, method, path, query):
    """V2 的入口：返回 True 表示已经写完响应，False 表示不是 V2 的请求。"""
    p = (path or "/").rstrip("/") or "/"

    alias = PAGE_ALIASES.get(p)
    if alias is not None:
        _serve_page(handler, method, alias, path)
        return True

    if not (p == "/api/v2" or p.startswith("/api/v2/")):
        return False

    if method == "HEAD":
        method = "GET"

    stream = STREAM_ROUTES.get(p)
    if stream is not None:
        if method != "GET":
            _send_json(handler, {"ok": False, "error": "该接口只支持 GET"}, 405)
            return True
        stream(handler, query)          # 内部自己 try/except，不会抛给上层
        return True

    body, err = ({}, None)
    if method == "POST":
        body, err = _read_body(handler)
        if err:
            _send_json(handler, {"ok": False, "error": err}, 400)
            return True

    fn = ROUTES.get((method, p))
    if fn is None:
        allowed = [m for (m, path_) in ROUTES if path_ == p]
        if allowed:
            _send_json(handler, {"ok": False,
                                 "error": "该接口只支持 %s" % "/".join(sorted(allowed))}, 405)
            return True
        _send_json(handler, {"ok": False, "error": "未知接口：%s" % p}, 404)
        return True

    try:
        result = fn(query, body)
        if not isinstance(result, dict):
            result = {"ok": True, "data": result}
    except (BrokenPipeError, ConnectionResetError):
        return True
    except Exception as exc:  # noqa: BLE001
        traceback.print_exc()
        _log("error", "接口 %s 异常：%s" % (p, exc))
        result = {"ok": False, "error": "接口异常：%s: %s" % (type(exc).__name__, exc)}
    try:
        _send_json(handler, result)
    except (BrokenPipeError, ConnectionResetError):
        pass
    return True


def startup_log():
    """服务启动时打一条横幅，方便从 /api/v2/logs 看后端状态。"""
    _log("info", "上位机 V2 后端就绪（v%s，仿真应用端口 %s，项目根 %s）"
         % (VERSION, HOLDER.port, WORK4_DIR))
