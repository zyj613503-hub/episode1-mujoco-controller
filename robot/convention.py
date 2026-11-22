#!/usr/bin/env python3
"""唯一换算源：电机度数 <-> DH/URDF 关节角 <-> MuJoCo qpos。

上位机界面（以及真机驱动器、仿真桥、脚本生成器）**只允许**从这里取换算，
任何地方都不要自己再写一遍数字。要改，只改这个文件。

约定
----
* 界面上滑块显示的是「电机度数」deg，也叫「关节角度」，范围见 MOTOR_RANGE_DEG。
* DH 关节角（= URDF 关节角 = MuJoCo joint1..joint6 的值，单位 rad）：
      dh = radians(SIGN * (deg - OFFSET))
      deg = OFFSET + SIGN * degrees(dh)
* 全零 DH 姿态（机械臂竖直站立）对应 deg = OFFSET = [180, 90, 83, 30, 110, 30]，
  这正好是上位机「恢复默认」下发的角度，也是厂家说明书里的零位。
* 夹爪：界面 0..110 度开合 -> MuJoCo 执行器 ctrl 0..1.70。
"""

from __future__ import annotations

import math

# ---------------------------------------------------------------- 关节
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
ARM_ACTUATORS = ["joint1_pos", "joint2_pos", "joint3_pos", "joint4_pos", "joint5_pos", "joint6_pos"]
JOINT_LABELS = ["J1", "J2", "J3", "J4", "J5", "J6"]

# 电机每转脉冲数（厂家参数，回读分辨率 = READ_SCALE / PPR 度）
PPR = [80000, 64000, 80000, 32000, 12800, 3200]

# 零位（电机度数）。用户「恢复默认」= 这个值。
OFFSET = [180.0, 90.0, 83.0, 30.0, 110.0, 30.0]

# 电机方向：+1 = 电机度数增大时 DH 角增大。
# 只有 J2/J5 是对称限位，靠限位推不出符号；真机到手后用 tools/verify_signs.py 单关节点动确认。
SIGN = [+1.0, +1.0, +1.0, +1.0, +1.0, +1.0]

# 电机度数可用区间（= URDF 限位换算过来，和厂家上位机滑块一致）
MOTOR_RANGE_DEG = [
    (180.0 - 180.0, 180.0 + 160.0),      # J1  0..340   URDF -180..160
    (90.0 - 90.0, 90.0 + 90.0),          # J2  0..180   URDF  -90.. 90
    (83.0 - 83.0, 83.0 + 80.0),          # J3  0..163   URDF  -83.. 80
    (30.0 + -30.0, 30.0 + 305.0),        # J4  0..335   URDF  -30..305
    (110.0 - 110.0, 110.0 + 110.0),      # J5  0..220   URDF -110..110
    (30.0 + -30.0, 30.0 + 305.0),        # J6  0..335   URDF  -30..305
]

# DH 关节角限位（rad），与上面严格对应，用于 IK 约束与显示
JOINT_LIMIT_RAD = [
    (math.radians(MOTOR_RANGE_DEG[i][0] - OFFSET[i]) * SIGN[i],
     math.radians(MOTOR_RANGE_DEG[i][1] - OFFSET[i]) * SIGN[i])
    for i in range(6)
]

# 回读换算：角度(度) = 原始整数 * READ_SCALE / PPR
READ_SCALE = 4500.0

# ---------------------------------------------------------------- 夹爪
GRIPPER_ACTUATOR = "gripper_servo"
GRIPPER_RANGE_DEG = (0.0, 110.0)     # 界面开合角
GRIPPER_CTRL_MAX = 1.70              # MuJoCo ctrl 上限（全合拢）
GRIPPER_TCP_MM = 120.0               # 法兰 -> 夹爪尖距离
SUCTION_TCP_MM = 50.0
DEFAULT_TCP_MM = GRIPPER_TCP_MM

# ---------------------------------------------------------------- 模型 / 场景
DEFAULT_MODEL_XML = "model/episode1.xml"
DEFAULT_SCENE_XML = "model/scene_episode1.xml"
TCP_SITE = "tcp"
FLANGE_SITE = "flange"
HOME_KEYFRAME = "home"


# ---------------------------------------------------------------- 换算函数
def clamp_deg(i: int, deg: float) -> float:
    lo, hi = MOTOR_RANGE_DEG[i]
    return max(lo, min(hi, float(deg)))


def deg_to_dh(i: int, deg: float) -> float:
    """电机度数 -> DH 关节角（rad）。"""
    return math.radians(SIGN[i] * (float(deg) - OFFSET[i]))


def dh_to_deg(i: int, dh: float) -> float:
    """DH 关节角（rad）-> 电机度数。"""
    return OFFSET[i] + SIGN[i] * math.degrees(float(dh))


def degs_to_dh(degs) -> list:
    return [deg_to_dh(i, d) for i, d in enumerate(degs)]


def dh_to_degs(dhs) -> list:
    return [dh_to_deg(i, q) for i, q in enumerate(dhs)]


def wrap_deg(deg: float) -> float:
    """把任意角度折回 -180..180。"""
    return (float(deg) + 180.0) % 360.0 - 180.0


def deg_to_pulse(i: int, deg: float) -> int:
    """电机度数 -> CAN 报文里的 16 位脉冲（pulse = 度数 * PPR / 360）。"""
    return int(round(float(deg) * PPR[i] / 360.0)) & 0xFFFF


def pulse_to_deg(i: int, pulse: int) -> float:
    return float(pulse) * 360.0 / PPR[i]


def deg_to_readback_raw(i: int, deg: float) -> int:
    """电机度数 -> 回读报文里的 32 位整数（上位机显示 = 原始值 * READ_SCALE / PPR）。"""
    return int(round(float(deg) * PPR[i] / READ_SCALE)) & 0xFFFFFFFF


def readback_raw_to_deg(i: int, raw: int) -> float:
    return float(raw) * READ_SCALE / PPR[i]


def readback_step_deg(i: int) -> float:
    """回读分辨率（度）。J6 最粗，1.4 度一格。"""
    return READ_SCALE / PPR[i]


def gripper_deg_to_ctrl(deg: float) -> float:
    lo, hi = GRIPPER_RANGE_DEG
    d = max(lo, min(hi, float(deg)))
    return GRIPPER_CTRL_MAX * (d - lo) / (hi - lo)


def gripper_ctrl_to_deg(ctrl: float) -> float:
    lo, hi = GRIPPER_RANGE_DEG
    return lo + (hi - lo) * min(1.0, max(0.0, float(ctrl) / GRIPPER_CTRL_MAX))


def motor_frame(i: int, deg: float) -> bytes:
    """CAN 参数帧 + 同步帧（真机写角度用）。"""
    p = deg_to_pulse(i, deg)
    head = bytes([0x00, 0x00, (p >> 8) & 0xFF, p & 0xFF, 0x00, 0x00, 0x00, (p >> 8) & 0xFF])
    return head


__all__ = [n for n in dir() if not n.startswith("_")]
