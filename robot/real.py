#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真机（CAN）驱动：ctypes 直接调 PEAK PCAN-Basic 的 libPCBUSB.dylib。

场景
----
Episode1 桌面机械臂的控制盒走 CAN，Mac 上用 PEAK 的 PCAN-USB 适配器，
库是 /Applications/Episode1.app/Contents/Frameworks/libPCBUSB.dylib，
导出 CAN_Initialize / CAN_Write / CAN_Read / CAN_Reset 等 PCAN-Basic 函数。

**没有插适配器时 CAN_Initialize 返回 0x1400（PCAN_ERROR_ILLNET）**。
本模块的所有公开方法都不抛异常：失败只写一条中文日志、返回 False / None，
并且 is_available() 为 False，调用方据此自动退回仿真（MockReal）。

报文协议（与 bridge/sim_bridge.py 的仿真桥完全一致）
------------------------------------------------------------------
关节 j（0=J1..5=J6）：

* 参数帧   ID = 0x100*(j+1)，8 字节
          [0]=0xFD 帧头 [1]=dir 方向 [2..3]=speed 大端 [4]=accel
          [5]=0x00 [6]=0x00 [7]=pulse_hi
* 同步帧   ID = 0x100*(j+1)+1，5 字节
          [0]=0xFD [1]=pulse_lo [2]=0x01 [3]=0x00 [4]=0x6B
* 脉冲     pulse = 角度(度) * PPR[j] / 360，16 位 = (pulse_hi << 8) | pulse_lo
* 回读     往 ID=0x100*(j+1) 发 2 字节 [0x36, 0x00]，
          应答 5 字节 [0x36, V(4 字节大端)]，角度 = V * 4500 / PPR[j]
          （换算全走 robot.convention，别在这里二次定义）

夹爪：ID = 0x700，4 字节 [0xAE, 角度(0..110), 0x01, 0x6B]。

自查
----
    python -m robot.real          # 无适配器时自动跑 MockReal 自测
"""

from __future__ import annotations

import ctypes
import ctypes.util
import os
import time

import robot.convention as C

# ---------------------------------------------------------------- 常量
LIB_PATH = "/Applications/Episode1.app/Contents/Frameworks/libPCBUSB.dylib"
LIB_FALLBACKS = ("libPCBUSB.dylib", "libpcanbasic.dylib")

CAN_HANDLE_USB1 = 0x51          # PCAN_USBBUS1
CAN_HANDLE_USB2 = 0x52
CAN_BAUD_1M = 0x0014           # BTR0BTR1：1 Mbit/s（桥默认）

PCAN_ERROR_OK = 0x0000
PCAN_ERROR_ILLNET = 0x1400      # 没插适配器时 CAN_Initialize 就是这个
PCAN_ERROR_ILLHW = 0x1401
PCAN_ERROR_ILLCLIENT = 0x1403
PCAN_ERROR_QRCVEMPTY = 0x0020   # 接收队列空

ERROR_TEXT = {
    PCAN_ERROR_OK: "正常",
    PCAN_ERROR_ILLNET: "未检测到 CAN 适配器（PCAN_ERROR_ILLNET）",
    PCAN_ERROR_ILLHW: "CAN 适配器句柄无效（PCAN_ERROR_ILLHW）",
    PCAN_ERROR_ILLCLIENT: "CAN 客户端无效（PCAN_ERROR_ILLCLIENT）",
    PCAN_ERROR_QRCVEMPTY: "接收队列为空（PCAN_ERROR_QRCVEMPTY）",
}

JOINT_PARAM_ID = 0x100         # 关节 j 的参数帧 ID = JOINT_PARAM_ID * (j+1)
GRIPPER_ID = 0x700             # 夹爪
FRAME_HEADER = 0xFD
TAIL = 0x6B
SYNC_BYTE_2 = 0x01
READBACK_CMD = 0x36
GRIPPER_CMD = 0xAE

DEFAULT_SPEED = 0x03E8         # 速度档位（大端 16 位）——真机可调
DEFAULT_ACCEL = 0x01           # 加速度档位
MSG_TYPE_STANDARD = 0x00

N_JOINTS = 6


# ---------------------------------------------------------------- ctypes 结构
class TPCANMsg(ctypes.Structure):
    """PCAN-Basic 的标准帧。"""
    _fields_ = [
        ("ID", ctypes.c_uint32),
        ("MSGTYPE", ctypes.c_ubyte),
        ("LEN", ctypes.c_ubyte),
        ("DATA", ctypes.c_ubyte * 8),
    ]


class TPCANTimestamp(ctypes.Structure):
    _fields_ = [
        ("millis", ctypes.c_uint32),
        ("millis_overflow", ctypes.c_uint16),
        ("micros", ctypes.c_uint16),
    ]


def _err_text(code: int) -> str:
    return ERROR_TEXT.get(int(code), "CAN 错误 0x%04X" % int(code))


# ---------------------------------------------------------------- 真机
class Real:
    """真机驱动。所有方法都不抛异常；不可用时 is_available() 为 False。"""

    def __init__(self, lib_path: str | None = None, handle: int = CAN_HANDLE_USB1,
                 baud: int = CAN_BAUD_1M, speed: int = DEFAULT_SPEED,
                 accel: int = DEFAULT_ACCEL, logger=None):
        self.lib_path = lib_path or LIB_PATH
        self.handle = handle
        self.baud = baud
        self.speed = int(speed) & 0xFFFF
        self.accel = int(accel) & 0xFF
        self._log_fn = logger
        self._lib = None
        self._connected = False
        self._lib_error = ""
        self._last_pulse = [None] * N_JOINTS
        self._load()

    # ------------------------------------------------------------ 日志
    def _log(self, msg: str) -> None:
        line = "[真机] " + msg
        if self._log_fn is not None:
            self._log_fn(line)
        else:
            print(line, flush=True)

    # ------------------------------------------------------------ 加载
    def _candidates(self):
        out = [self.lib_path]
        out.extend(LIB_FALLBACKS)
        found = ctypes.util.find_library("PCBUSB")
        if found:
            out.append(found)
        seen, uniq = set(), []
        for p in out:
            if p and p not in seen:
                seen.add(p)
                uniq.append(p)
        return uniq

    def _load(self) -> bool:
        for path in self._candidates():
            if os.sep in path and not os.path.exists(path):
                continue
            try:
                lib = ctypes.CDLL(path)
            except OSError:
                continue
            try:
                lib.CAN_Initialize.argtypes = [ctypes.c_uint16, ctypes.c_uint16,
                                               ctypes.c_uint8, ctypes.c_uint32,
                                               ctypes.c_uint16]
                lib.CAN_Initialize.restype = ctypes.c_uint32
                lib.CAN_Uninitialize.argtypes = [ctypes.c_uint16]
                lib.CAN_Uninitialize.restype = ctypes.c_uint32
                lib.CAN_Write.argtypes = [ctypes.c_uint16, ctypes.POINTER(TPCANMsg)]
                lib.CAN_Write.restype = ctypes.c_uint32
                lib.CAN_Read.argtypes = [ctypes.c_uint16, ctypes.POINTER(TPCANMsg),
                                         ctypes.POINTER(TPCANTimestamp)]
                lib.CAN_Read.restype = ctypes.c_uint32
                lib.CAN_Reset.argtypes = [ctypes.c_uint16]
                lib.CAN_Reset.restype = ctypes.c_uint32
            except AttributeError as exc:
                self._lib_error = "库里缺少 PCAN-Basic 符号：%s" % exc
                continue
            self._lib = lib
            self._lib_path_used = path
            return True
        self._lib_error = "找不到 libPCBUSB.dylib（试过：%s）" % "、".join(self._candidates())
        self._log(self._lib_error + "，真机不可用")
        return False

    # ------------------------------------------------------------ 连接
    def connect(self) -> bool:
        if self._lib is None:
            return False
        if self._connected:
            return True
        code = int(self._lib.CAN_Initialize(
            ctypes.c_uint16(self.handle), ctypes.c_uint16(self.baud),
            ctypes.c_uint8(0), ctypes.c_uint32(0), ctypes.c_uint16(0)))
        if code != PCAN_ERROR_OK:
            # 典型情况：0x1400 —— 没插适配器。绝不抛异常，只降级。
            self._log("CAN_Initialize 失败：%s（0x%04X）；已降级，真机不可用"
                      % (_err_text(code), code))
            self._connected = False
            self._last_error = code
            return False
        self._connected = True
        self._last_error = 0
        self._log("CAN 适配器已连接（handle=0x%02X，1 Mbit/s）" % self.handle)
        return True

    def disconnect(self) -> None:
        if self._lib is not None and self._connected:
            try:
                self._lib.CAN_Reset(ctypes.c_uint16(self.handle))
                self._lib.CAN_Uninitialize(ctypes.c_uint16(self.handle))
                self._log("CAN 适配器已断开")
            except Exception as exc:  # 绝不抛
                self._log("断开时出错：%s" % exc)
        self._connected = False

    def is_available(self) -> bool:
        """能不能真机控制：库加载成功且连接成功。"""
        return bool(self._lib is not None and self._connected)

    @property
    def status(self) -> dict:
        return {"available": self.is_available(), "lib": getattr(self, "_lib_path_used", None),
                "handle": self.handle, "error": getattr(self, "_last_error", 0),
                "text": _err_text(getattr(self, "_last_error", 0)),
                "detail": self._lib_error}

    # ------------------------------------------------------------ 底层读写
    def _write(self, can_id: int, data: bytes) -> bool:
        if not self.is_available():
            return False
        if len(data) > 8:
            self._log("报文过长（%d 字节），丢弃" % len(data))
            return False
        msg = TPCANMsg()
        msg.ID = int(can_id)
        msg.MSGTYPE = MSG_TYPE_STANDARD
        msg.LEN = len(data)
        for i, b in enumerate(data):
            msg.DATA[i] = int(b)
        code = int(self._lib.CAN_Write(ctypes.c_uint16(self.handle), ctypes.byref(msg)))
        if code != PCAN_ERROR_OK:
            self._log("CAN_Write 失败（ID=0x%03X）：%s" % (int(can_id), _err_text(code)))
            return False
        return True

    def _read(self, timeout_ms: int = 50):
        """读一帧，返回 (id, bytes) 或 None。"""
        if not self.is_available():
            return None
        deadline = time.time() + max(int(timeout_ms), 1) / 1000.0
        msg = TPCANMsg()
        ts = TPCANTimestamp()
        while time.time() <= deadline:
            code = int(self._lib.CAN_Read(ctypes.c_uint16(self.handle),
                                          ctypes.byref(msg), ctypes.byref(ts)))
            if code == PCAN_ERROR_OK:
                n = int(msg.LEN)
                return int(msg.ID), bytes(int(msg.DATA[i]) for i in range(min(n, 8)))
            if code != PCAN_ERROR_QRCVEMPTY:
                self._log("CAN_Read 失败：%s" % _err_text(code))
                return None
            time.sleep(0.002)
        return None

    # ------------------------------------------------------------ 关节报文
    @staticmethod
    def joint_param_id(j: int) -> int:
        return JOINT_PARAM_ID * (int(j) + 1)

    @staticmethod
    def joint_sync_id(j: int) -> int:
        return JOINT_PARAM_ID * (int(j) + 1) + 1

    def _param_frame(self, j: int, deg: float) -> bytes:
        pulse = C.deg_to_pulse(j, deg) & 0xFFFF
        prev = self._last_pulse[j]
        direction = 0x01 if (prev is None or pulse >= prev) else 0x00
        self._last_pulse[j] = pulse
        return bytes([
            FRAME_HEADER, direction,
            (self.speed >> 8) & 0xFF, self.speed & 0xFF,
            self.accel & 0xFF, 0x00, 0x00,
            (pulse >> 8) & 0xFF,
        ])

    @staticmethod
    def _sync_frame(j: int, deg: float) -> bytes:
        pulse = C.deg_to_pulse(j, deg) & 0xFFFF
        return bytes([FRAME_HEADER, pulse & 0xFF, SYNC_BYTE_2, 0x00, TAIL])

    def move_joint(self, j: int, deg: float) -> bool:
        """单关节：先发参数帧，再发同步帧（同步帧一发，电机真的动）。"""
        deg = C.clamp_deg(j, deg)
        ok = self._write(self.joint_param_id(j), self._param_frame(j, deg))
        ok = self._write(self.joint_sync_id(j), self._sync_frame(j, deg)) and ok
        return bool(ok)

    def move_joints(self, degs, blocking: bool = False) -> bool:
        """六轴一起下发。blocking=True 时读回角度直到到位（最多 5 s）。"""
        degs = list(degs)
        if len(degs) < N_JOINTS:
            self._log("move_joints 需要 6 个角度，收到 %d 个" % len(degs))
            return False
        ok = True
        for j in range(N_JOINTS):
            ok = self.move_joint(j, float(degs[j])) and ok
        if blocking and ok:
            self.wait_arrived(degs)
        return bool(ok)

    def wait_arrived(self, degs, tol_deg: float = 2.0, timeout_s: float = 5.0) -> bool:
        deadline = time.time() + float(timeout_s)
        target = [C.clamp_deg(j, float(d)) for j, d in enumerate(degs)]
        while time.time() < deadline:
            now = self.read_all()
            if now is not None and all(abs(now[j] - target[j]) <= tol_deg for j in range(N_JOINTS)):
                return True
            time.sleep(0.05)
        self._log("等待到位超时（%.1f s）" % float(timeout_s))
        return False

    # ------------------------------------------------------------ 夹爪
    def set_gripper(self, deg: float) -> bool:
        lo, hi = C.GRIPPER_RANGE_DEG
        d = int(round(max(lo, min(hi, float(deg)))))
        return self._write(GRIPPER_ID, bytes([GRIPPER_CMD, d & 0xFF, 0x01, TAIL]))

    # ------------------------------------------------------------ 回读
    def read_joint(self, j: int, timeout_ms: int = 60):
        """读单关节角度（度）：发 [0x36,0x00]，收 0x36+4 字节大端。"""
        j = int(j)
        if not self._write(self.joint_param_id(j), bytes([READBACK_CMD, 0x00])):
            return None
        deadline = time.time() + max(int(timeout_ms), 1) / 1000.0
        while time.time() <= deadline:
            got = self._read(timeout_ms=max(1, int((deadline - time.time()) * 1000)))
            if got is None:
                break
            can_id, data = got
            if can_id != self.joint_param_id(j) or len(data) < 5 or data[0] != READBACK_CMD:
                continue
            raw = int.from_bytes(data[1:5], "big", signed=False)
            return float(C.readback_raw_to_deg(j, raw))
        return None

    def read_all(self):
        """读六轴；任何一轴失败返回 None。"""
        out = []
        for j in range(N_JOINTS):
            d = self.read_joint(j)
            if d is None:
                return None
            out.append(d)
        return out

    # ------------------------------------------------------------ 组合动作
    def execute_plan(self, plan: dict, scale: float = 1.0) -> bool:
        """按 planner 的结果把关节轨迹播一遍（真机）。

        真机伺服自己做插值，这里按 10 ms 的节奏持续下发目标角度；
        scale 可以整体放慢（例如 2.0 = 半速），方便第一次试跑。
        """
        if not self.is_available():
            self._log("真机不可用，execute_plan 直接返回 False")
            return False
        points = plan.get("points") or []
        if not points:
            self._log("轨迹为空，不执行")
            return False
        t_prev = points[0]["t"]
        for p in points:
            dt = (p["t"] - t_prev) / max(float(scale), 1e-3)
            if dt > 0:
                time.sleep(dt)
            t_prev = p["t"]
            self.move_joints(p["deg"])
            if p.get("grip") is not None:
                self.set_gripper(p["grip"])
        self._log("轨迹执行完成（%d 个采样点，%.2f s）" % (len(points), points[-1]["t"]))
        return True

    def close(self) -> None:
        self.disconnect()


# ---------------------------------------------------------------- 假真机
class MockReal(Real):
    """不碰 CAN 的假驱动：把报文留在内存里，用于自测与演示。

    * 角度直接记在 _deg[] 里（夹爪 _grip），回读立刻返回。
    * frames 里存 (id, bytes)，自测可以逐字节核对协议。
    """

    def __init__(self, **kw):
        kw.setdefault("logger", lambda line: print(line, flush=True))
        super().__init__(**kw)
        self._deg = [C.OFFSET[i] for i in range(N_JOINTS)]
        self._grip = float(C.GRIPPER_RANGE_DEG[0])
        self.frames = []
        self._mock_connected = True

    def connect(self) -> bool:
        self._connected = True
        self._last_error = 0
        self._log("MockReal 已连接（不占 CAN 总线）")
        return True

    def is_available(self) -> bool:
        return True

    def _write(self, can_id: int, data: bytes) -> bool:
        self.frames.append((int(can_id), bytes(data)))
        if int(can_id) == GRIPPER_ID and len(data) >= 2 and data[0] == GRIPPER_CMD:
            self._grip = float(data[1])
        else:
            for j in range(N_JOINTS):
                if int(can_id) == self.joint_sync_id(j) and len(data) >= 2:
                    hi = self._deg[j]
                    hi_pulse = C.deg_to_pulse(j, hi)
                    lo = data[1]
                    # 用最近一次参数帧的 pulse_hi 拼回 16 位
                    for fid, fdata in reversed(self.frames[:-1]):
                        if fid == self.joint_param_id(j) and len(fdata) == 8:
                            hi_pulse = (fdata[7] << 8) | lo
                            break
                    self._deg[j] = C.pulse_to_deg(j, hi_pulse)
        return True

    def _read(self, timeout_ms: int = 50):
        if self._pending_read is None:
            return None
        item = self._pending_read
        self._pending_read = None
        return item

    _pending_read = None

    def read_joint(self, j: int, timeout_ms: int = 60):
        j = int(j)
        raw = C.deg_to_readback_raw(j, self._deg[j])
        self.frames.append((self.joint_param_id(j), bytes([READBACK_CMD, 0x00])))
        self._pending_read = (self.joint_param_id(j),
                              bytes([READBACK_CMD]) + raw.to_bytes(4, "big"))
        return super().read_joint(j, timeout_ms)

    def set_gripper(self, deg: float) -> bool:
        lo, hi = C.GRIPPER_RANGE_DEG
        self._grip = max(lo, min(hi, float(deg)))
        return super().set_gripper(deg)

    @property
    def degs(self):
        return list(self._deg)

    @property
    def grip(self):
        return float(self._grip)


# ---------------------------------------------------------------- 自测
def _selftest() -> int:
    print("=== robot.real 自测 ===", flush=True)
    real = Real()
    print("真机状态：", real.status, flush=True)
    ok = real.connect()
    if ok:
        print("检测到适配器，回读角度：", real.read_all(), flush=True)
        real.disconnect()
    else:
        print("未检测到真实适配器，改用 MockReal 验证协议", flush=True)

    mock = MockReal()
    mock.connect()
    mock.move_joints([180.0, 100.0, 60.0, 30.0, 110.0, 30.0])
    mock.set_gripper(110.0)
    print("下发 6 帧 + 夹爪，共 %d 帧" % len(mock.frames), flush=True)
    for fid, data in mock.frames:
        print("  ID=0x%03X LEN=%d %s" % (fid, len(data), data.hex(" ").upper()), flush=True)
    # 参数帧/同步帧逐字节核对（J1 目标 180° -> pulse = 180*80000/360 = 40000 = 0x9C40）
    j1_param = [d for fid, d in mock.frames if fid == 0x100][0]
    j1_sync = [d for fid, d in mock.frames if fid == 0x101][0]
    print("J1 参数帧:", j1_param.hex(" ").upper(), "（应为 FD 01 03 E8 01 00 00 9C）", flush=True)
    print("J1 同步帧:", j1_sync.hex(" ").upper(), "（应为 FD 40 01 00 6B）", flush=True)
    print("读回：", mock.read_all(), flush=True)
    print("夹爪：", mock.grip, flush=True)
    return 0 if (j1_param == bytes([0xFD, 0x01, 0x03, 0xE8, 0x01, 0, 0, 0x9C])
                 and j1_sync == bytes([0xFD, 0x40, 0x01, 0x00, 0x6B])) else 1


class RealArm(Real):
    """别名：Arm 门面里真机侧就叫 RealArm。"""


if __name__ == "__main__":
    raise SystemExit(_selftest())
