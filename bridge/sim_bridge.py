#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sim_bridge.py —— 独立的 MuJoCo 仿真应用（episode1）

这个进程就是「一台仿真机械臂」：它自己开 MuJoCo 窗口、自己算物理，
上位机（运行器/server/v2.py）通过 TCP 127.0.0.1:47999 像连真机一样控制它。
上位机进程里不再创建/步进 MuJoCo，页面 3D 是本地 three.js FK 预览，不推流。

启动（macOS 的 MuJoCo 窗口必须用 mjpython 启动 Python）：
  .venv/bin/python .venv/bin/mjpython bridge/sim_bridge.py            # 开窗口（默认）
  .venv/bin/python .venv/bin/mjpython bridge/sim_bridge.py --headless # 无窗口（自测）
  .venv/bin/python .venv/bin/mjpython bridge/sim_bridge.py --verbose  # 控制台也打日志

TCP 行协议（与 bridge/libPCBUSB.dylib 假 CAN 驱动一致）：
  请求  "W <can_id_hex> <len> <data_hex>"        例：W 101 8 00010000000001f4
  应答  "R <can_id_hex> <payload_hex>"
  连接建立后先回一行 "HELLO simapp/<版本> <端口>"（上位机可忽略）

CAN 语义（换算全部取自 robot/convention.py，本文件不写任何魔数）：
  关节写  参数帧 0x100*j         8 字节 fd|dir|speed_hi|speed_lo|accel|00|00|pulse_hi
          同步帧 0x100*j+1(低 4 位=1) 5 字节 fd|pulse_lo|01|00|6b
          pulse = 电机度数 × PPR / 360（16 位）；电机度数 → DH 用 convention
  关节读  0x100*j 数据 36 00      → 回 "R <id> 36 <8位十六进制 V>"，度 = V × READ_SCALE / PPR
  夹爪写  0x700 [0xAE, 角度, 0x01, 0x6B]     角度 0..110 度（0=张开，110=合拢）
  夹爪读  0x700 [0x36, 0x00]                  → 回 "R 700 36 <8位十六进制 厘度>"（度 = V/100）
  吸盘    0x700 [0xAD, 0x01/0x00, 0x00, 0x6B]（本模型没有吸盘，忽略）
  急停    0x7FF 数据 "ESTOP"（45 53 54 4F 50）  目标 = 当前实测，机械臂停住（之后仍可再下指令）
  退出    0x7FF 数据 "QUIT "（51 55 49 54 20）  关闭窗口、进程退出

日志：bridge/logs/simapp.log（2 MB × 3 滚动）；--verbose 同时打控制台。
窗口：标题栏由 MuJoCo 自己的 viewer 固定为 "MuJoCo :"，所以这里把
      "MuJoCo · episode1" 画在窗口左上角（见 _install_title_overlay）。
"""

from __future__ import annotations

import argparse
import logging
import logging.handlers
import math
import os
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------
APP_NAME = "MuJoCo · episode1"
VERSION = "1.0"
XML_PATH = "model/scene_episode1.xml"
LISTEN_HOST = "127.0.0.1"
LISTEN_PORT = 47999
DEFAULT_TCP_PORT = LISTEN_PORT

MAX_JOINT_SPEED_DEG = 180.0     # 关节斜坡限速（度/秒）
GRIPPER_SPEED_DEG = 150.0       # 夹爪开合限速（度/秒）
GRIPPER_FULL_DEG = 110.0        # 夹爪满量程（度）→ convention.GRIPPER_RANGE_DEG[1]

# 重力/科氏力矩补偿：每个物理步把这 6 个臂关节的「偏置力」（qfrc_bias，含重力）
# 反向加到 qfrc_applied 上。位置伺服就不必靠「位置误差」去顶重力，
# 于是下发角度后机械臂不再被自重拖下去（实测稳态误差从 0.5° 降到 0.02°）。
# 只补偿机械臂 6 个关节：球/盘子的 free joint、夹爪的从动关节都不碰。
GRAVITY_COMP = True

STATE_JSON = "/tmp/sim_bridge_state.json"
CLIENT_IDLE_TIMEOUT_S = 120.0   # 单条连接空闲多久算客户死了（防止残留连接占资源）

CAN_ESTOP_ID = 0x7FF
CAN_ESTOP_DATA = b"ESTOP"
CAN_QUIT_DATA = b"QUIT "

LOG_DIR = Path(__file__).resolve().parent / "logs"
LOG_PATH = LOG_DIR / "simapp.log"

log = logging.getLogger("simapp")


def setup_logging(verbose: bool = False) -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log.setLevel(logging.DEBUG)
    fmt = logging.Formatter("%(asctime)s %(levelname)-5s %(message)s", "%Y-%m-%d %H:%M:%S")
    fh = logging.handlers.RotatingFileHandler(str(LOG_PATH), maxBytes=2 * 1024 * 1024,
                                              backupCount=3, encoding="utf-8")
    fh.setFormatter(fmt)
    log.addHandler(fh)
    if verbose:
        sh = logging.StreamHandler(sys.stdout)
        sh.setFormatter(fmt)
        log.addHandler(sh)
    log.propagate = False
    log.info("=" * 64)
    log.info("sim_bridge 启动 v%s · python %s", VERSION, sys.version.split()[0])


# ---------------------------------------------------------------------------
# 唯一换算源：robot/convention.py
# ---------------------------------------------------------------------------
_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from robot import convention as C  # noqa: E402


def _clamp_gripper_deg(deg: float) -> float:
    lo, hi = C.GRIPPER_RANGE_DEG
    return max(lo, min(hi, float(deg)))


def qpos_to_motor(q_rad) -> list:
    """MuJoCo 关节角（rad）→ 电机读数（度），按 convention 换算。"""
    out = []
    for i in range(6):
        out.append(C.dh_to_deg(i, float(q_rad[i])))
    return out


def motor_to_qpos(motor_deg) -> list:
    """电机读数（度）→ MuJoCo 关节角（rad）。"""
    return [C.deg_to_dh(i, float(motor_deg[i])) for i in range(6)]


class ArmSim:
    """MuJoCo 仿真本体：持「目标关节角」，每个物理步朝目标平滑靠近。"""

    def __init__(self, xml_path, use_gui=False):
        import mujoco
        self.mujoco = mujoco
        self.xml_path = xml_path
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.joint_names = list(C.JOINT_NAMES)
        self.joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                          for n in self.joint_names]
        self.qpos_adr = [int(self.model.jnt_qposadr[j]) for j in self.joint_ids]
        self.act_ids = []
        for name in C.ARM_ACTUATORS:
            self.act_ids.append(int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)))
        self.grip_act = int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, C.GRIPPER_ACTUATOR))
        self.grip_target = 0.0                  # 夹爪目标（度）
        self.grip_now = 0.0                     # 夹爪当前指令（度）
        if self.grip_act >= 0:
            self.data.ctrl[self.grip_act] = C.gripper_deg_to_ctrl(self.grip_now)
        # 6 个臂关节在 qfrc_applied / qfrc_bias 里的下标（重力补偿用）
        self.dof_adr = [int(self.model.jnt_dofadr[j]) for j in self.joint_ids]
        self.lo = np.array([self.model.jnt_range[j][0] for j in self.joint_ids])
        self.hi = np.array([self.model.jnt_range[j][1] for j in self.joint_ids])
        if self.model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        q0 = np.array([self.data.qpos[a] for a in self.qpos_adr], dtype=float)
        self.q_target = q0.copy()
        self.q_cmd = q0.copy()      # 斜坡命令（位置执行器的 ctrl），必须自己积分，不能跟着实测走
        self.q_actual = q0.copy()
        self.lock = threading.Lock()
        self.viewer = None
        self.use_gui = use_gui
        self.stopped = threading.Event()
        self.timestep = float(self.model.opt.timestep)
        self._last_dump = -1.0

    # ---------- 给 socket 线程调用 ----------
    def set_target_qpos(self, q):
        q = np.clip(np.asarray(q, dtype=float), self.lo, self.hi)
        with self.lock:
            self.q_target = q

    def set_gripper(self, deg):
        deg = _clamp_gripper_deg(deg)
        with self.lock:
            self.grip_target = deg

    def get_qpos(self):
        with self.lock:
            return self.q_actual.copy()

    def get_gripper_deg(self):
        with self.lock:
            return float(self.grip_now)

    def estop(self):
        """急停：目标 = 当前实测（关节 + 夹爪），机械臂当场停住。"""
        with self.lock:
            self.q_target = self.q_actual.copy()
            self.grip_target = self.grip_now
        log.warning("急停：目标已置为当前实测，机械臂停住")

    def request_stop(self, reason="quit"):
        log.info("收到退出请求（%s）", reason)
        self.stopped.set()

    # ---------- 标题（MuJoCo viewer 标题栏写死为 "MuJoCo :"，这里画在窗口左上角） ----------
    def _install_title_overlay(self):
        try:
            import mujoco
            # 固定 UI 文本：右上角（左上角是 MuJoCo 自带的版本信息，别去挤）
            self.viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                                   None, APP_NAME + "  "))
        except Exception as exc:  # noqa: BLE001
            log.debug("标题叠加文本设置失败（不影响运行）：%s", exc)

    # ---------- 主循环：推进物理 ----------
    def run(self):
        mujoco = self.mujoco
        if self.use_gui:
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.cam.distance = 1.4
            self.viewer.cam.azimuth = 135
            self.viewer.cam.elevation = -18
            self.viewer.cam.lookat[:] = [0.0, 0.0, 0.25]
            self._install_title_overlay()
            log.info("MuJoCo 窗口已打开（%s）", APP_NAME)
        dt = self.timestep
        max_step = math.radians(MAX_JOINT_SPEED_DEG) * dt
        g_step = GRIPPER_SPEED_DEG * dt
        next_t = time.perf_counter()
        while not self.stopped.is_set():
            if self.viewer is not None and not self.viewer.is_running():
                log.info("MuJoCo 窗口已关闭 → 进程退出")
                break
            with self.lock:
                q_goal = self.q_target.copy()
                g_goal = self.grip_target
            # 斜坡要作用在「命令」上：若把命令写成「实测 + 一步」，重力一压
            # 实测往下掉、命令跟着掉，会把机械臂带跑（实测过 J2 掉 48°）。
            delta = np.clip(q_goal - self.q_cmd, -max_step, max_step)
            self.q_cmd = np.clip(self.q_cmd + delta, self.lo, self.hi)
            for i, aid in enumerate(self.act_ids):
                if aid >= 0:
                    self.data.ctrl[aid] = self.q_cmd[i]
            if GRAVITY_COMP:
                # 重力 + 科氏补偿：qfrc_applied 每步都要重写（mj_step 不会清它）
                for dof in self.dof_adr:
                    self.data.qfrc_applied[dof] = self.data.qfrc_bias[dof]
            if self.grip_act >= 0:
                self.grip_now += float(np.clip(g_goal - self.grip_now, -g_step, g_step))
                self.data.ctrl[self.grip_act] = C.gripper_deg_to_ctrl(self.grip_now)
            mujoco.mj_step(self.model, self.data)
            with self.lock:
                self.q_actual = np.array([self.data.qpos[a] for a in self.qpos_adr], dtype=float)
                if STATE_JSON and (self.data.time - self._last_dump) >= 0.1:
                    self._last_dump = self.data.time
                    try:
                        import json as _json
                        with open(STATE_JSON, "w") as fp:
                            _json.dump({"time": round(float(self.data.time), 3),
                                        "deg": [round(v, 2) for v in qpos_to_motor(self.q_actual)],
                                        "gripper": round(self.grip_now, 2)}, fp)
                    except OSError:
                        pass
            if self.viewer is not None:
                self.viewer.sync()
            next_t += dt
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()
        self.stopped.set()
        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:  # noqa: BLE001
                pass


class Bridge:
    """TCP 服务端：解析上位机发来的 CAN 报文，回送电机回读。"""

    def __init__(self, sim, host=LISTEN_HOST, port=LISTEN_PORT):
        self.sim = sim
        self.host = host
        self.port = port
        self.pending = {}
        self.dh_target = None
        self._local = threading.local()      # 每条连接一个线程，各自记住自己的 socket
        self._clients = set()
        self._clients_lock = threading.Lock()
        self._state_lock = threading.Lock()  # pending / dh_target 的并发保护
        self._srv = None

    # ---------- 回帧 ----------
    def _reply_angle(self, j, conn=None):
        motor_deg = qpos_to_motor(self.sim.get_qpos())[j] % 360.0
        v = C.deg_to_readback_raw(j, motor_deg)
        self._send("R %03x 36%08x" % (0x100 * (j + 1), v), conn)

    def _reply_gripper(self, conn=None):
        v = int(round(self.sim.get_gripper_deg() * 100.0)) & 0xFFFFFFFF
        self._send("R 700 36%08x" % v, conn)

    def _send(self, line, conn=None):
        """回一帧。未指定 conn 时回给本线程正在服务的那条连接。"""
        if conn is None:
            conn = getattr(self._local, "conn", None)
        if conn is None:
            return
        try:
            conn.sendall((line + "\n").encode())
        except OSError:
            pass

    # ---------- 解析一条 "W <id> <len> <hex>" ----------
    def handle_write(self, can_id, data, conn=None):
        n = len(data)
        if can_id == CAN_ESTOP_ID:
            if data == CAN_ESTOP_DATA:
                self.sim.estop()
            elif data == CAN_QUIT_DATA:
                self.sim.request_stop("CAN 0x7FF QUIT")
            else:
                log.warning("0x7FF 未知指令 %s", data.hex())
            return
        if can_id == 0x700:
            if n >= 2 and data[0] == 0xAE:
                deg = float(data[1])
                log.debug("夹爪 → %.0f°", deg)
                self.sim.set_gripper(deg)
            elif n >= 2 and data[0] == 0x36:
                self._reply_gripper(conn)
            elif n >= 2 and data[0] == 0xAD:
                log.debug("吸盘指令（本模型无吸盘，忽略）：%s", data.hex())
            else:
                log.debug("0x700 其它指令：%s", data.hex())
            return
        joint = can_id // 0x100
        if 1 <= joint <= 6:
            j = joint - 1
            if n >= 8:
                with self._state_lock:
                    self.pending[j] = {"pulse_hi": data[7]}
            elif n == 5 and (can_id & 0x0F) == 0x01:
                with self._state_lock:
                    info = self.pending.pop(j, None)
                pulse_hi = info["pulse_hi"] if info else 0
                pulse = (pulse_hi << 8) | data[1]
                motor_deg = C.pulse_to_deg(j, pulse)
                self._apply_joint(j, motor_deg)
            elif n == 2 and data[0] == 0x36:
                self._reply_angle(j, conn)
            elif n == 2 and data[0] == 0x22:
                self._send("R %03x 226b" % can_id, conn)
            else:
                log.debug("关节%d 其它指令 n=%d %s", j + 1, n, data.hex())
            return
        log.debug("未识别报文 id=0x%x n=%d %s", can_id, n, data.hex())

    def _apply_joint(self, j, motor_deg):
        """只改第 j 个关节的目标角，其它关节保持不变。"""
        with self._state_lock:
            if self.dh_target is None:
                self.dh_target = [C.dh_to_deg(i, float(v)) for i, v in
                                  enumerate(self.sim.get_qpos())]
            self.dh_target[j] = motor_deg
            q = [C.deg_to_dh(i, self.dh_target[i]) for i in range(6)]
        self.sim.set_target_qpos(q)

    # ---------- 主循环：监听 + 收行 ----------
    def serve_forever(self):
        try:
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self.host, self.port))
            srv.listen(4)
        except OSError as exc:
            log.error("无法监听 %s:%d → %s", self.host, self.port, exc)
            print("[simapp] 无法监听 %s:%d → %s" % (self.host, self.port, exc), file=sys.stderr)
            self.sim.request_stop("bind failed")
            return
        self._srv = srv
        log.info("已监听 %s:%d，等待上位机连接", self.host, self.port)
        print("[simapp] 已监听 %s:%d" % (self.host, self.port), flush=True)
        # 一条连接一个线程：上位机（Server）探测端口、旧进程残留的连接都不会把
        # 正常的一路读回堵死（之前单连接时会 head-of-line 阻塞，上位机表现为"掉线"）。
        while not self.sim.stopped.is_set():
            try:
                conn, addr = srv.accept()
            except OSError:
                break
            try:
                conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            except OSError:
                pass
            with self._clients_lock:
                self._clients.add(conn)
            log.info("上位机已连接：%s", addr)
            t = threading.Thread(target=self._serve_client, args=(conn, addr),
                                 name="bridge-client-%s" % (addr[1],), daemon=True)
            t.start()
        try:
            srv.close()
        except OSError:
            pass
        with self._clients_lock:
            clients = list(self._clients)
        for conn in clients:
            try:
                conn.close()
            except OSError:
                pass

    def _serve_client(self, conn, addr):
        """单条连接的收行循环；写回全部走本线程的 socket。"""
        idle = 0.0
        self._local.conn = conn        # 本线程的回帧出口
        try:
            conn.sendall(("HELLO simapp/%s %d\n" % (VERSION, self.port)).encode())
            conn.settimeout(0.5)
            buf = b""
            while not self.sim.stopped.is_set():
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    idle += 0.5
                    if idle > CLIENT_IDLE_TIMEOUT_S:
                        log.warning("上位机连接空闲超时（%.0fs），主动断开：%s",
                                    idle, addr)
                        break
                    continue
                if not chunk:
                    break
                idle = 0.0
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self._handle_line(line.decode("utf-8", "ignore").strip(), conn)
        except OSError:
            pass
        except Exception:  # noqa: BLE001
            log.exception("处理上位机连接时出错")
        finally:
            try:
                conn.close()
            except OSError:
                pass
            with self._clients_lock:
                self._clients.discard(conn)
            log.info("上位机断开：%s", addr)

    def _handle_line(self, line, conn=None):
        if not line.startswith("W "):
            return
        parts = line.split()
        if len(parts) < 4:
            return
        try:
            can_id = int(parts[1], 16)
            data = bytes.fromhex(parts[3])
        except ValueError:
            return
        self.handle_write(can_id, data, conn)

    def shutdown(self):
        if self._srv is not None:
            try:
                self._srv.close()
            except OSError:
                pass
        with self._clients_lock:
            clients = list(self._clients)
        for conn in clients:
            try:
                conn.close()
            except OSError:
                pass


def main(argv=None):
    ap = argparse.ArgumentParser(description="%s 独立仿真应用" % APP_NAME)
    ap.add_argument("--gui", dest="gui", action="store_true", default=True,
                    help="打开 MuJoCo 窗口（默认开）")
    ap.add_argument("--headless", dest="gui", action="store_false",
                    help="不开窗口，只跑物理（自测用）")
    ap.add_argument("--xml", default=XML_PATH, help="场景 XML（默认 %s）" % XML_PATH)
    ap.add_argument("--port", type=int, default=LISTEN_PORT, help="监听端口（默认 %d）" % LISTEN_PORT)
    ap.add_argument("--verbose", action="store_true", help="控制台也输出日志")
    args = ap.parse_args(argv)

    setup_logging(args.verbose)
    root = Path(__file__).resolve().parent.parent
    os.chdir(str(root))
    if not Path(args.xml).is_file():
        log.error("找不到场景文件：%s", args.xml)
        print("[simapp] 找不到场景文件：%s" % args.xml, file=sys.stderr)
        return 1

    log.info("场景 %s · 端口 %d · 窗口 %s", args.xml, args.port, "开" if args.gui else "关")
    print("[simapp] %s · 场景 %s · 端口 %d · 窗口 %s"
          % (APP_NAME, args.xml, args.port, "开" if args.gui else "关"), flush=True)

    sim = ArmSim(args.xml, use_gui=args.gui)
    bridge = Bridge(sim, LISTEN_HOST, args.port)
    th = threading.Thread(target=bridge.serve_forever, name="bridge-tcp", daemon=True)
    th.start()
    try:
        sim.run()
    except KeyboardInterrupt:
        log.info("收到 Ctrl+C")
    finally:
        sim.stopped.set()
        bridge.shutdown()
        time.sleep(0.2)
        log.info("simapp 已退出")
        print("[simapp] 已退出", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
