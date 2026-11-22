#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robot/simapp.py —— 独立 MuJoCo 仿真应用的客户端与进程管理

上位机（运行器/server/v2.py）只用这个模块跟仿真应用打交道：
  * SimAppClient  —— TCP 127.0.0.1:47999 上的 CAN 语义客户端（写关节 / 写夹爪 / 回读 / 急停）
  * launch_app()   —— 用 mjpython 起一个新进程（cwd = 项目根，stdout 进日志）
  * ensure_app()   —— 没起就起，最多等 timeout 秒到 TCP 就绪
  * app_status()   —— {running, pid, port, reachable}
  * stop_app()     —— 优雅退出（发 QUIT 帧），不行再 SIGTERM/SIGKILL

协议细节见 bridge/sim_bridge.py 顶部注释。线程安全：每个客户端一把锁，
多线程同时 read_state / send_joints 不会互相踩包。
"""

from __future__ import annotations

import os
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

WORK4_DIR = Path(__file__).resolve().parent.parent
SIM_SCRIPT = WORK4_DIR / "bridge" / "sim_bridge.py"
LOG_DIR = WORK4_DIR / "bridge" / "logs"
LAUNCH_LOG = LOG_DIR / "simapp-launch.log"
LAUNCH_LOG_MAX = 2 * 1024 * 1024

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 47999

# 读回超时（秒）：7 个查询帧，正常 < 20 ms
DEFAULT_READ_TIMEOUT = 1.5
# 建立 TCP 连接的超时
DEFAULT_CONNECT_TIMEOUT = 3.0

CAN_ESTOP_ID = 0x7FF
CAN_ESTOP_DATA = b"ESTOP"
CAN_QUIT_DATA = b"QUIT "

_PROC = None          # subprocess.Popen
_PROC_PORT = None
_PROC_GUI = True
_PROC_LOG = None      # 打开的日志文件对象
_PROC_LOCK = threading.RLock()


class SimAppError(RuntimeError):
    """仿真应用相关的可读错误（中文消息，直接可以回给前端）。"""


# ---------------------------------------------------------------------------
# 换算（唯一来源 robot/convention.py）
# ---------------------------------------------------------------------------
try:
    from robot import convention as C
except Exception:  # noqa: BLE001  —— 允许被直接当脚本调试时 cwd 不同
    sys.path.insert(0, str(WORK4_DIR))
    from robot import convention as C


def _venv_python():
    p = WORK4_DIR / ".venv" / "bin" / "python"
    return str(p) if p.is_file() else sys.executable


def _mjpython():
    p = WORK4_DIR / ".venv" / "bin" / "mjpython"
    return str(p) if p.is_file() else ""


def _launch_command(port, gui):
    """macOS 上 MuJoCo 窗口必须由 mjpython 启动；无窗口时用普通 python 更快。"""
    py = _venv_python()
    mjp = _mjpython()
    args = [str(SIM_SCRIPT), "--port", str(port)]
    args.append("--gui" if gui else "--headless")
    if gui and mjp:
        return [py, mjp] + args
    return [py] + args


def _rotate_launch_log():
    try:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        if LAUNCH_LOG.is_file() and LAUNCH_LOG.stat().st_size > LAUNCH_LOG_MAX:
            backup = LAUNCH_LOG.with_suffix(".log.1")
            try:
                if backup.is_file():
                    backup.unlink()
                LAUNCH_LOG.rename(backup)
            except OSError:
                pass
    except OSError:
        pass


# ---------------------------------------------------------------------------
# 客户端
# ---------------------------------------------------------------------------
class SimAppClient(object):
    """TCP 127.0.0.1:47999 上的 CAN 语义客户端。"""

    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_PORT,
                 read_timeout=DEFAULT_READ_TIMEOUT):
        self.host = str(host or DEFAULT_HOST)
        self.port = int(port or DEFAULT_PORT)
        self.read_timeout = float(read_timeout)
        self._sock = None
        self._lock = threading.RLock()
        self._buf = b""
        self.hello = ""

    # ---------- 连接 ----------
    def connect(self, timeout=DEFAULT_CONNECT_TIMEOUT):
        with self._lock:
            if self._sock is not None:
                return self
            try:
                sock = socket.create_connection((self.host, self.port), timeout=timeout)
            except socket.timeout:
                raise SimAppError("连接 MuJoCo 仿真应用超时（%s:%d，%ss）"
                                  % (self.host, self.port, timeout))
            except OSError as exc:
                raise SimAppError("无法连接 MuJoCo 仿真应用（%s:%d）：%s"
                                  % (self.host, self.port, exc))
            sock.settimeout(self.read_timeout)
            self._sock = sock
            self._buf = b""
            self.hello = ""
            # 应用连上后会先发一行 HELLO 版本串；读到就记下来，读不到不影响
            old = sock.gettimeout()
            try:
                sock.settimeout(0.2)
                line = self._read_line_raw()
                if line.startswith("HELLO"):
                    self.hello = line
            except (socket.timeout, OSError):
                pass
            finally:
                if self._sock is not None:
                    self._sock.settimeout(old)
            return self

    def is_connected(self):
        return self._sock is not None

    def close(self):
        with self._lock:
            sock, self._sock = self._sock, None
            self._buf = b""
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.close()

    # ---------- 底层 ----------
    def _require(self):
        if self._sock is None:
            raise SimAppError("MuJoCo 仿真应用未连接（先 connect）")

    def _write(self, lines):
        with self._lock:
            self._require()
            payload = ("\n".join(lines) + "\n").encode()
            try:
                self._sock.sendall(payload)
            except socket.timeout:
                raise SimAppError("向 MuJoCo 仿真应用发送指令超时")
            except OSError as exc:
                self.close()
                raise SimAppError("与 MuJoCo 仿真应用的连接已断开：%s" % exc)

    def _read_line_raw(self):
        """从缓冲区/套接字读一行（不含换行）。"""
        if self._sock is None:
            raise SimAppError("MuJoCo 仿真应用未连接")
        while b"\n" not in self._buf:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise SimAppError("MuJoCo 仿真应用已关闭连接")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return line.decode("utf-8", "ignore").strip()

    # ---------- §2 接口 ----------
    def send_joints(self, deg_list):
        """下发 6 个关节的目标角（电机度数）。"""
        try:
            degs = [float(v) for v in deg_list]
        except (TypeError, ValueError):
            raise SimAppError("关节角度必须是数字数组")
        if len(degs) != 6:
            raise SimAppError("关节角度必须是 6 个（收到 %d 个）" % len(degs))
        lines = []
        for j, deg in enumerate(degs):
            deg = C.clamp_deg(j, deg)
            pulse = C.deg_to_pulse(j, deg)
            hi, lo = (pulse >> 8) & 0xFF, pulse & 0xFF
            # 参数帧：fd|dir|speed_hi|speed_lo|accel|00|00|pulse_hi（8 字节 16 个 hex）
            lines.append("W %03x 8 0000ffff000000%02x" % (0x100 * (j + 1), hi))
            # 同步帧：fd|pulse_lo|01|00|6b
            lines.append("W %03x 5 %02x%02x01006b" % (0x100 * (j + 1) + 1, 0x00, lo))
        self._write(lines)
        return {"ok": True, "deg": degs}

    def send_gripper(self, deg):
        """下发夹爪角度（0..110 度）。"""
        try:
            d = float(deg)
        except (TypeError, ValueError):
            raise SimAppError("夹爪角度必须是数字")
        lo, hi = C.GRIPPER_RANGE_DEG
        d = max(lo, min(hi, d))
        # 0x700 [0xAE, 角度, 0x01, 0x6B]
        line = "W 700 4 ae%02x016b" % int(round(d))
        self._write([line])
        return {"ok": True, "deg": d}

    def read_state(self, timeout=None):
        """一次读回 6 个关节角 + 夹爪角（度）。失败抛 SimAppError。"""
        deadline = time.time() + float(timeout or self.read_timeout)
        # 6 个关节 + 夹爪的查询帧
        queries = ["W %03x 2 3600" % (0x100 * (j + 1)) for j in range(6)]
        queries.append("W 700 2 3600")
        with self._lock:
            self._require()
            degs = [None] * 6
            gripper = None
            try:
                self._sock.sendall(("\n".join(queries) + "\n").encode())
            except OSError as exc:
                self.close()
                raise SimAppError("与 MuJoCo 仿真应用的连接已断开：%s" % exc)
            while time.time() < deadline:
                if all(v is not None for v in degs) and gripper is not None:
                    return {"ok": True, "deg": degs, "gripper": gripper}
                self._sock.settimeout(max(0.05, deadline - time.time()))
                try:
                    line = self._read_line_raw()
                except socket.timeout:
                    break
                except OSError as exc:
                    self.close()
                    raise SimAppError("读取仿真应用状态失败：%s" % exc)
                if not line.startswith("R "):
                    continue
                parts = line.split()
                if len(parts) < 3:
                    continue
                try:
                    can_id = int(parts[1], 16)
                    payload = bytes.fromhex(parts[2])
                except ValueError:
                    continue
                if len(payload) < 5 or payload[0] != 0x36:
                    continue
                raw = int.from_bytes(payload[1:5], "big")
                if can_id == 0x700:
                    gripper = round(raw / 100.0, 2)
                elif can_id % 0x100 == 0:
                    # 关节 j 的读回 ID 是 0x100*(j+1)（j=0 → 0x100）
                    j = can_id // 0x100 - 1
                    if 0 <= j < 6:
                        degs[j] = round(C.readback_raw_to_deg(j, raw), 3)
            missing = [("J%d" % (i + 1)) for i, v in enumerate(degs) if v is None]
            if gripper is None:
                missing.append("夹爪")
            raise SimAppError("读取仿真应用状态超时：%s 没有回读（%.1fs）"
                              % ("、".join(missing), float(timeout or self.read_timeout)))

    def estop(self):
        """急停：仿真应用把目标置为当前实测，机械臂停住。"""
        self._write(["W 7ff 5 %s" % CAN_ESTOP_DATA.hex()])
        return {"ok": True}

    def quit_app(self):
        """让仿真应用关窗退出（优雅）。"""
        try:
            self._write(["W 7ff 5 %s" % CAN_QUIT_DATA.hex()])
        except SimAppError:
            return {"ok": False}
        return {"ok": True}


# ---------------------------------------------------------------------------
# 进程管理
# ---------------------------------------------------------------------------
def _lsof_listeners(port, host=DEFAULT_HOST):
    """谁在监听这个端口：返回 pid 列表（不发起 TCP 连接，避免刷对方日志）。

    macOS/Linux 上用 lsof；lsof 不可用时返回 None。"""
    try:
        out = subprocess.run(["lsof", "-nP", "-iTCP:%d" % int(port), "-sTCP:LISTEN", "-t"],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             timeout=2.0)
    except (OSError, subprocess.SubprocessError):
        return None
    if out.returncode != 0 and not out.stdout.strip():
        return [] if out.returncode == 1 else None
    pids = []
    for line in out.stdout.decode("utf-8", "ignore").split():
        try:
            pids.append(int(line))
        except ValueError:
            pass
    return pids


def _port_listening(host=DEFAULT_HOST, port=DEFAULT_PORT):
    """端口上是否有人在监听（不建 TCP 连接，避免和真实 client 抢连接）。"""
    pids = _lsof_listeners(port, host)
    if pids is None:
        # lsof 不可用：只能真连一下（会有握手，但至少答案可靠）
        try:
            with socket.create_connection((host, int(port)), timeout=0.35):
                return True
        except OSError:
            return False
    return bool(pids)


def _port_open(host=DEFAULT_HOST, port=DEFAULT_PORT, timeout=0.35):
    """兼容旧调用：等价于"端口上有人监听"。"""
    return _port_listening(host, port)


def app_status(port=None, host=DEFAULT_HOST):
    """仿真应用状态：{running, pid, port, reachable, external}。"""
    with _PROC_LOCK:
        proc = _PROC
        pport = int(port or _PROC_PORT or DEFAULT_PORT)
        alive = proc is not None and proc.poll() is None
        listeners = _lsof_listeners(pport, host)
        reachable = bool(listeners) if listeners is not None else _port_open(host, pport)
        if alive:
            pid = proc.pid
            # 端口被别人的进程占着（不是我们的子进程）就不算"我们管的"
            managed = bool((not listeners) or (proc.pid in (listeners or [])))
        else:
            managed = False
            pid = (listeners or [None])[0]
        return {
            "running": bool(alive or reachable),
            "pid": pid if (alive or reachable) else None,
            "port": pport,
            "reachable": bool(reachable),
            "external": bool(reachable and not managed),
            "managed": managed,
        }


def launch_app(port=DEFAULT_PORT, gui=True, host=DEFAULT_HOST):
    """起一个独立仿真应用进程（不等待就绪）。已在跑时直接返回当前状态。"""
    global _PROC, _PROC_PORT, _PROC_GUI, _PROC_LOG
    with _PROC_LOCK:
        if _PROC is not None and _PROC.poll() is None:
            return app_status(port)
        if _port_open(host, port):
            # 端口上已有别人起的应用（双击 .command 启动的），不重复起
            _PROC_PORT = int(port)
            return app_status(port)
        if not SIM_SCRIPT.is_file():
            raise SimAppError("找不到仿真应用脚本：%s" % SIM_SCRIPT)
        _rotate_launch_log()
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        try:
            logf = open(str(LAUNCH_LOG), "ab", buffering=0)
        except OSError as exc:
            raise SimAppError("无法写启动日志 %s：%s" % (LAUNCH_LOG, exc))
        cmd = _launch_command(port, gui)
        try:
            proc = subprocess.Popen(cmd, cwd=str(WORK4_DIR), stdin=subprocess.DEVNULL,
                                    stdout=logf, stderr=subprocess.STDOUT,
                                    start_new_session=True)
        except OSError as exc:
            logf.close()
            raise SimAppError("启动 MuJoCo 仿真应用失败：%s" % exc)
        _PROC, _PROC_PORT, _PROC_GUI, _PROC_LOG = proc, int(port), bool(gui), logf
    return app_status(port)


def ensure_app(port=DEFAULT_PORT, timeout=20.0, gui=True, host=DEFAULT_HOST):
    """确保仿真应用已启动且 TCP 就绪；超时或进程秒退则抛 SimAppError。"""
    with _PROC_LOCK:
        if _port_open(host, port):
            return app_status(port)
        status = launch_app(port, gui=gui, host=host)
        proc = _PROC
    deadline = time.time() + float(timeout)
    while time.time() < deadline:
        if _port_open(host, port):
            log = app_status(port)
            log["ok"] = True
            return log
        if proc is not None and proc.poll() is not None:
            raise SimAppError("MuJoCo 仿真应用启动后立即退出（退出码 %s），详见日志 %s"
                              % (proc.returncode, LAUNCH_LOG))
        time.sleep(0.2)
    raise SimAppError("MuJoCo 仿真应用启动超时（%.0fs 内 TCP %s:%d 未就绪），详见日志 %s"
                      % (float(timeout), host, port, LAUNCH_LOG))


def stop_app(port=None, timeout=6.0, host=DEFAULT_HOST):
    """停掉仿真应用：先发 QUIT 让它自己关窗，再 SIGTERM/SIGKILL 兜底。"""
    global _PROC, _PROC_LOG
    with _PROC_LOCK:
        proc = _PROC
        pport = int(port or _PROC_PORT or DEFAULT_PORT)
        logf = _PROC_LOG
        _PROC = None
        _PROC_LOG = None
    # 1) 优雅：发 QUIT 帧（对双击启动、不归我们管的进程也有效）
    if _port_open(host, pport):
        try:
            cli = SimAppClient(host, pport).connect(timeout=1.5)
            cli.quit_app()
            cli.close()
        except SimAppError:
            pass
    # 2) 我们自己起的进程：等它退，不行就杀
    if proc is not None:
        try:
            proc.wait(timeout=min(4.0, timeout))
        except subprocess.TimeoutExpired:
            try:
                proc.terminate()
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
            except OSError:
                pass
        except OSError:
            pass
    # 3) 等端口释放
    deadline = time.time() + max(0.0, timeout - 2.0)
    while time.time() < deadline and _port_open(host, pport, timeout=0.2):
        time.sleep(0.15)
    if logf is not None:
        try:
            logf.close()
        except OSError:
            pass
    st = app_status(pport, host)
    st["ok"] = not st["running"]
    return st


def launch_log_tail(lines=40):
    """启动日志最后若干行（/app 与 selftest 排障用）。"""
    try:
        with open(str(LAUNCH_LOG), "rb") as fp:
            data = fp.read()
    except OSError:
        return ""
    text = data.decode("utf-8", "ignore").splitlines()
    return "\n".join(text[-lines:])
