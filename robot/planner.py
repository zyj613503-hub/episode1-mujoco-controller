#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""路点 → 10 ms 关节轨迹的规划器（be-plan）。

契约
----
输入：waypoints = [[x, y, z, dwell, grip], ...]，长度单位**米**，
      dwell 是该点停稳的秒数，grip 是 0–110 的开合角（旧写法 "open"/"close" 仍认，=0/110）。
输出：{ok, duration, reachable, points:[{t, deg, grip, wp}], errors, max_joint_speed_deg, waypoints}

为什么不是"每个路点单段直线插值"
----------------------------------
朴素做法（步长 = 路点距离 / 固定段数）会让**采样点速度不连续**：拐角处关节要瞬间
反向，仿真和真机都跟不上，实测出现过 IK 误差 116 mm、关节速度 12 rad/s。
这里改成：

1. **位置用三次 Hermite**（Catmull-Rom 切线 + 限幅）。切线取前后路点的差分，
   并按相邻两段的速度上限（0.8 倍较小者）限幅，所以不会在拐角"冲出去"；
   写了 dwell 的路点、首尾点切线强制为 0（停稳），中途不停的点被平滑穿过。
2. **按时间均匀采样**（默认 0.01 s），每个采样点用 ArmIK.solve_pose 追到收敛，
   q_seed 用上一个采样点的解（warm start），所以关节角连续、不会翻腕。
3. 工具全程**竖直朝下**，yaw（绕世界 z 的朝向）沿用上一段，避免 J6 末端来回甩。
4. 每路点顺序严格是：**到位 → 停 dwell → 再改夹爪**（open=0°，close=110°）。

IK 来源
-------
优先 robot.ik.ArmIK（契约 §8）。若开工时 robot/ik.py 还不存在，用本文件里的
_MinkArmIK 临时顶上（mink 直连，接口与契约 §8 完全一致），不阻塞规划与脚本生成。
"""

from __future__ import annotations

import math
from typing import Any, Sequence

import numpy as np

import robot.convention as convention

# ------------------------------------------------------------------ 规划参数
PLAN_DT = 0.01              # 采样周期（秒）
BASE_SPEED_MPS = 0.25       # speed=1.0 时的直线巡航速度（米/秒）
MIN_SEGMENT_SEC = 0.35      # 单段最短耗时，避免"一帧走完"
MIN_LEADIN_SEC = 0.60       # home -> 第一个路点最短耗时（长距离会自然变长）
WAYPOINT_ERR_LIMIT_MM = 5.0  # 路点位置误差超过这个值判定不可达
MAX_JOINT_SPEED_DEG = 240.0  # 关节速度超过这个值（度/秒）就自动放慢重规划
MAX_TIME_SCALE = 4.0        # 自动放慢最多放大到 4 倍时长
DEFAULT_FRAME = "tcp"       # 契约里的 TCP 站点名
GRIP_OPEN_DEG = 0.0         # 夹爪张开（界面角度）
GRIP_CLOSE_DEG = 110.0      # 夹爪夹紧（界面角度）
GRIP_NAMES = {"open": GRIP_OPEN_DEG, "close": GRIP_CLOSE_DEG,
              "张开": GRIP_OPEN_DEG, "闭合": GRIP_CLOSE_DEG}

# 实测可达域（TCP，米）：水平半径 ≤ 0.576，z ∈ -0.074..0.769
REACH_RADIUS_M = 0.576
REACH_Z_MIN = -0.074
REACH_Z_MAX = 0.769


# ================================================================== 临时 IK
class _MinkArmIK:
    """临时 ArmIK：robot/ik.py 尚未落地时用 mink 直连顶上。

    接口与《接口契约.md》§8 一致：fk / tool_down_rotation / solve_pose /
    self_collision / limits_ok。正式实现落地后本类自动不再被使用。
    """

    def __init__(self, model_xml: str | None = None):
        import mujoco
        import mink

        self._mujoco = mujoco
        self._mink = mink
        path = str(model_xml or convention.DEFAULT_MODEL_XML)
        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        self.config = mink.Configuration(self.model)
        self.config.update(self.model.qpos0.copy())
        self._tasks: dict[str, Any] = {}
        self.limits = [mink.ConfigurationLimit(self.model)]
        self._site_ids: dict[str, int] = {}

    # -------------------------------------------------------- 内部工具
    def _site_id(self, frame: str) -> int:
        if frame not in self._site_ids:
            sid = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_SITE, frame)
            if sid < 0:
                raise ValueError(f"模型里找不到站点 {frame!r}")
            self._site_ids[frame] = sid
        return self._site_ids[frame]

    def _task(self, frame: str):
        if frame not in self._tasks:
            self._tasks[frame] = self._mink.FrameTask(
                frame_name=frame, frame_type="site",
                position_cost=1.0, orientation_cost=1.0, lm_damping=1e-2,
            )
        return self._tasks[frame]

    @staticmethod
    def _rot_angle_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
        c = (float(np.trace(Ra.T @ Rb)) - 1.0) / 2.0
        return math.degrees(math.acos(max(-1.0, min(1.0, c))))

    # -------------------------------------------------------- 契约接口
    def fk(self, frame: str, q: np.ndarray | None = None):
        qq = self.model.qpos0.copy() if q is None else np.asarray(q, dtype=float).copy()
        self.data.qpos[:] = qq
        self._mujoco.mj_forward(self.model, self.data)
        sid = self._site_id(frame)
        return self.data.site_xpos[sid].copy(), self.data.site_xmat[sid].reshape(3, 3).copy()

    def tool_down_rotation(self, yaw_deg: float = 0.0) -> np.ndarray:
        """工具轴（站点 +z）竖直朝下，绕世界 z 再转 yaw。

        yaw=0 时 R = diag(1, -1, -1)：与"把 home 姿态的工具轴转到 -z"得到的结果一致，
        站点 +x 保持指向世界 +x（手指开合方向），J6 不会无谓地转半圈。
        """
        y = math.radians(float(yaw_deg))
        c, s = math.cos(y), math.sin(y)
        return np.array([[c, s, 0.0],
                         [s, -c, 0.0],
                         [0.0, 0.0, -1.0]])

    def solve_pose(self, frame: str, pos_m, R, q_seed=None, iters: int = 40) -> dict:
        pos = np.asarray(pos_m, dtype=float).reshape(3)
        R = np.asarray(R, dtype=float).reshape(3, 3)
        seed = self.model.qpos0.copy() if q_seed is None else np.asarray(q_seed, dtype=float).copy()
        task = self._task(frame)
        try:
            self.config.update(seed)
            task.set_target(self._mink.SE3.from_rotation_and_translation(
                rotation=self._mink.SO3.from_matrix(R), translation=pos))
            for _ in range(max(1, int(iters))):
                err = task.compute_error(self.config)
                if float(np.linalg.norm(err[:3])) < 1e-5 and float(np.linalg.norm(err[3:])) < 1e-4:
                    break
                vel = self._mink.solve_ik(self.config, [task], 0.05, "daqp",
                                          limits=self.limits, damping=1e-6)
                self.config.integrate_inplace(vel, 0.05)
        except Exception:
            return {"q": seed, "status": "unreachable",
                    "pos_err_mm": 1e3, "ori_err_deg": 180.0}
        q = self.config.q.copy()
        p_act, R_act = self.fk(frame, q)
        pos_err = float(np.linalg.norm(p_act - pos)) * 1000.0
        ori_err = self._rot_angle_deg(R, R_act)

        status = "ok"
        lo = np.array([convention.JOINT_LIMIT_RAD[i][0] for i in range(6)])
        hi = np.array([convention.JOINT_LIMIT_RAD[i][1] for i in range(6)])
        if pos_err > 2.0 or ori_err > 8.0:
            status = "unreachable"
        elif np.any(q[:6] < lo - 0.02) or np.any(q[:6] > hi + 0.02):
            status = "wrist_flip"
        elif np.any(np.abs(_wrap_rad(q[:6] - seed[:6])) > math.radians(60.0)):
            status = "discontinuity"
        elif self.self_collision(q):
            status = "self_collision"
        return {"q": q, "status": status, "pos_err_mm": pos_err, "ori_err_deg": ori_err}

    def self_collision(self, q) -> bool:
        # 临时实现：机械臂自身几何干涉判定交给正式的 robot/ik.py；
        # 这里只保证接口存在（本模型相邻连杆本来就不做自碰撞）。
        return False

    def limits_ok(self, q) -> bool:
        q = np.asarray(q, dtype=float)
        for i in range(6):
            lo, hi = convention.JOINT_LIMIT_RAD[i]
            if q[i] < lo - 1e-6 or q[i] > hi + 1e-6:
                return False
        return True


def _wrap_rad(a: np.ndarray) -> np.ndarray:
    return (np.asarray(a, dtype=float) + math.pi) % (2.0 * math.pi) - math.pi


def load_ik(model_xml: str | None = None):
    """优先用 robot/ik.py 的 ArmIK；没有就退回 _MinkArmIK。返回 (实例, 后端名)。"""
    try:
        from robot.ik import ArmIK  # type: ignore
        return ArmIK(model_xml or convention.DEFAULT_MODEL_XML), "robot.ik"
    except Exception:
        return _MinkArmIK(model_xml or convention.DEFAULT_MODEL_XML), "mink(fallback)"


# ================================================================== 路点解析
def parse_waypoints(waypoints: Sequence[Sequence[Any]]) -> list[dict]:
    """把 [x, y, z, dwell, grip] 解析成规整的 dict，顺手做校验。"""
    if not waypoints:
        raise ValueError("waypoints 不能为空")
    out: list[dict] = []
    for i, w in enumerate(waypoints):
        w = list(w)
        if len(w) < 3:
            raise ValueError(f"路点 {i} 至少要写 x/y/z 三个数，收到 {w!r}")
        pos = np.array([float(w[0]), float(w[1]), float(w[2])])
        dwell = float(w[3]) if len(w) > 3 and w[3] is not None else 0.0
        if dwell < 0:
            raise ValueError(f"路点 {i} 的 dwell 不能是负数：{dwell}")
        grip = None
        if len(w) > 4 and w[4] is not None:
            g = w[4]
            if isinstance(g, str):
                key = g.strip().lower()
                if key not in GRIP_NAMES:
                    raise ValueError(f"路点 {i} 的夹爪只支持 open/close 或 0–110 的数字，收到 {g!r}")
                grip = GRIP_NAMES[key]
            else:
                try:
                    grip = float(g)
                except (TypeError, ValueError):
                    raise ValueError(f"路点 {i} 的夹爪角度不是数字：{g!r}")
                if not (0.0 <= grip <= 110.0):
                    raise ValueError(f"路点 {i} 的夹爪角度必须在 0–110 之间，收到 {grip}")
        out.append({"pos": pos, "dwell": dwell, "grip": grip})
    return out


def check_reach_envelope(p: np.ndarray) -> str | None:
    """粗判是否在实测可达域内；返回 None 或不可达原因。"""
    r = float(math.hypot(float(p[0]), float(p[1])))
    if r > REACH_RADIUS_M:
        return f"水平半径 {r:.3f} m 超过 {REACH_RADIUS_M:.3f} m"
    if not (REACH_Z_MIN <= float(p[2]) <= REACH_Z_MAX):
        return f"z={float(p[2]):.3f} m 超出 {REACH_Z_MIN:.3f}..{REACH_Z_MAX:.3f} m"
    return None


# ================================================================== 插值
def _hermite(p0, p1, v0, v1, duration: float, s: float) -> np.ndarray:
    """三次 Hermite：s∈[0,1] 是归一化时间，v0/v1 是米/秒。"""
    h00 = 2 * s ** 3 - 3 * s ** 2 + 1
    h10 = s ** 3 - 2 * s ** 2 + s
    h01 = -2 * s ** 3 + 3 * s ** 2
    h11 = s ** 3 - s ** 2
    return h00 * p0 + h10 * (duration * v0) + h01 * p1 + h11 * (duration * v1)


def _tangents(pts: list[np.ndarray], durations: list[float], dwells: list[float]) -> list[np.ndarray]:
    """Catmull-Rom 切线（米/秒）+ 限幅。

    * 首尾、以及写了 dwell 的路点：切线 = 0（停稳，不会"滑过"抓取点）。
    * 其余：取前后路点差分 / 两段总时长，再按相邻两段速度的 0.8 倍限幅，
      保证拐角不会冲过路点（朴素版冲出去正是 IK 误差的来源）。
    """
    n = len(pts)
    vel = [np.zeros(3) for _ in range(n)]
    for i in range(1, n - 1):
        if dwells[i] > 0:
            continue
        if durations[i - 1] <= 0 or durations[i] <= 0:
            continue
        raw = (pts[i + 1] - pts[i - 1]) / (durations[i - 1] + durations[i])
        sp_prev = float(np.linalg.norm(pts[i] - pts[i - 1])) / durations[i - 1]
        sp_next = float(np.linalg.norm(pts[i + 1] - pts[i])) / durations[i]
        limit = 0.8 * min(sp_prev, sp_next)
        norm = float(np.linalg.norm(raw))
        if norm > 1e-9:
            vel[i] = raw / norm * min(norm, limit)
    return vel


# ================================================================== 规划主体
def _segment_duration(dist: float, speed: float, floor: float) -> float:
    v = max(BASE_SPEED_MPS * max(speed, 1e-3), 1e-3)
    return max(float(dist) / v, floor)


def _build_phases(wps, speed):
    """构造运动/停留阶段表（时间轴）。

    路径**从第一个路点开始**（不是从 home 开始）：home 处工具轴指向 +x，
    而 TCP 竖直朝下在 home 那个位置（半径 0.37 m、z=0.50 m）其实超出关节限位
    （实测 J5 顶到 220° 仍差 100 mm），所以"从 home 直接走笛卡尔直线"不成立。
    正确做法是先做一段关节空间的起手动作到第一个路点，再从那里开始走
    Hermite 卡氏轨迹。返回 (phases, grip_changes, duration)。
    """
    pts = [w["pos"] for w in wps]
    dwells = [w["dwell"] / max(speed, 1e-3) for w in wps]
    durations = [_segment_duration(np.linalg.norm(pts[i + 1] - pts[i]), speed, MIN_SEGMENT_SEC)
                 for i in range(len(pts) - 1)]
    vels = _tangents(pts, durations, dwells)

    phases: list[dict] = []
    grip_changes: list[dict] = []
    t = 0.0
    grip_state = next((w["grip"] for w in wps if w["grip"] is not None), GRIP_OPEN_DEG)

    def settle(i: int) -> None:
        """到达第 i 个路点后：先停稳 dwell，再改夹爪。"""
        nonlocal t, grip_state
        if dwells[i] > 0:
            phases.append({"kind": "dwell", "t0": t, "t1": t + dwells[i],
                           "p": pts[i], "wp": i})
            t += dwells[i]
        g = wps[i]["grip"]
        if g is not None and abs(g - grip_state) > 1e-9:
            grip_changes.append({"t": t, "grip": g, "wp": i})
            grip_state = g

    settle(0)
    for i in range(len(pts) - 1):
        d = durations[i]
        phases.append({"kind": "motion", "t0": t, "t1": t + d,
                       "p0": pts[i], "p1": pts[i + 1], "v0": vels[i], "v1": vels[i + 1],
                       "dur": d, "wp": i + 1})
        t += d
        settle(i + 1)
    return phases, grip_changes, t


def _sample_times(phases, grip_changes, dt: float) -> list[float]:
    total = phases[-1]["t1"]
    marks = {0.0, total}
    for ph in phases:
        marks.add(round(ph["t0"], 9))
        marks.add(round(ph["t1"], 9))
    for gc in grip_changes:
        marks.add(round(gc["t"], 9))
    times = set(marks)
    n = int(round(total / dt))
    for k in range(n + 1):
        times.add(round(k * dt, 9))
    return sorted(t for t in times if -1e-9 <= t <= total + 1e-9)


def _pos_at(phases, t: float) -> np.ndarray:
    for ph in phases:
        if ph["t0"] - 1e-9 <= t <= ph["t1"] + 1e-9:
            if ph["kind"] == "dwell":
                return ph["p"]
            d = max(ph["t1"] - ph["t0"], 1e-9)
            s = min(1.0, max(0.0, (t - ph["t0"]) / d))
            return _hermite(ph["p0"], ph["p1"], ph["v0"], ph["v1"], ph["dur"], s)
    return phases[-1].get("p", phases[-1].get("p1"))


def _grip_at(grip_changes, t: float, initial: float) -> float:
    g = initial
    for gc in grip_changes:
        if gc["t"] <= t + 1e-9:
            g = gc["grip"]
    return g


def _wp_index_at(phases, t: float) -> int:
    for ph in phases:
        if ph["t0"] - 1e-9 <= t <= ph["t1"] + 1e-9:
            return int(ph["wp"])
    return int(phases[-1]["wp"])


def _smoothstep(s: float) -> float:
    s = min(1.0, max(0.0, float(s)))
    return s * s * (3.0 - 2.0 * s)


def _leadin_plan(q_start, first_pos, ik, frame, R_target):
    """起手段：从当前姿态关节空间平滑过渡到"第一个路点 + 工具竖直朝下"。

    为什么需要单独一段：本机型 home 的工具轴指向世界 +x，而轨迹要求全程朝下；
    更关键的是 **home 那个位置（半径 0.37 m、z=0.50 m）工具朝下本身就够不着**
    （实测 IK 把 J5 顶到 220° 极限仍差 100 mm）。所以不能"从 home 走笛卡尔直线"，
    要先在关节空间划到第一个路点，再从那里开始卡氏轨迹。
    过渡时长按最大关节位移算（150°/s），保证 t=0 的关节角与当前姿态完全一致。
    返回 (过渡秒数, 目标关节角)；不可用时返回 (0.0, None)。
    """
    res = ik.solve_pose(frame, first_pos, R_target, q_seed=q_start, iters=120)
    q_or = np.asarray(res.get("q", q_start), dtype=float)
    # 这里允许 status="discontinuity"：home 到"朝下"本来就差 100°+，
    # 正是靠 smoothstep 把它摊到 0.6 s 里，所以不算错误；
    # 但 unreachable / 撞限位 / 自碰撞仍然放弃起手段（此时只能从当前姿态硬走）。
    if res.get("status") in ("unreachable", "wrist_flip", "self_collision"):
        return 0.0, None
    if q_or.shape[0] < 6 or not np.all(np.isfinite(q_or[:6])):
        return 0.0, None
    if float(res.get("pos_err_mm", 1e9)) > 5.0:
        return 0.0, None
    dmax = float(np.max(np.abs(_wrap_rad(q_or[:6] - np.asarray(q_start, dtype=float)[:6]))))
    if dmax < math.radians(0.5):
        return 0.0, None
    return max(MIN_LEADIN_SEC, math.degrees(dmax) / 150.0), q_or


def _plan_once(wps, speed, q_start, ik, frame, yaw_deg, dt):
    """单次规划（不含自动降速）。"""
    q_start = np.asarray(q_start, dtype=float)
    phases, grip_changes, body_total = _build_phases(wps, speed)
    body_times = _sample_times(phases, grip_changes, dt)
    initial_grip = next((w["grip"] for w in wps if w["grip"] is not None), GRIP_OPEN_DEG)

    R_target = ik.tool_down_rotation(yaw_deg)
    lead_sec, q_or = _leadin_plan(q_start, wps[0]["pos"], ik, frame, R_target)
    if q_or is None:
        q_or = q_start.copy()
    n_lead = int(round(lead_sec / dt))

    points: list[dict] = []
    q_prev = q_start.copy()
    last_status = "ok"
    # ---- 起手段：原地关节过渡（不走路点，位置保持在 start_pos 附近）
    for k in (range(n_lead + 1) if n_lead > 0 else []):
        s = _smoothstep(k / n_lead)
        q = (1.0 - s) * q_start + s * q_or
        q_prev = q.copy()
        deg = convention.dh_to_degs(q[:6])
        points.append({
            "t": round(k * dt, 6),
            "deg": [round(float(d), 4) for d in deg],
            "grip": round(float(initial_grip), 3),
            "wp": 0,
            "leadin": True,
        })

    # ---- 主体：沿 Hermite 轨迹逐点求解
    for tb in body_times:
        if lead_sec > 0 and tb <= 1e-9:
            continue  # t=0 已由起手段第一帧覆盖
        t = tb + lead_sec
        target = _pos_at(phases, tb)
        res = ik.solve_pose(frame, target, R_target, q_seed=q_prev, iters=60)
        q = np.asarray(res.get("q", q_prev), dtype=float)
        if q.shape[0] >= 6 and np.all(np.isfinite(q[:6])):
            q_prev = q
            last_status = res.get("status", "ok")
        deg = convention.dh_to_degs(q[:6])
        points.append({
            "t": round(float(t), 6),
            "deg": [round(float(d), 4) for d in deg],
            "grip": round(float(_grip_at(grip_changes, tb, initial_grip)), 3),
            "wp": _wp_index_at(phases, tb),
        })

    times = [p["t"] for p in points]
    total = points[-1]["t"]

    # 路点误差：取"到达该路点"的那个采样点的 FK 结果
    wp_arrive_times = {0: lead_sec}
    wp_arrive_times.update({ph["wp"]: ph["t1"] + lead_sec for ph in phases if ph["kind"] == "motion"})
    errors: list[dict] = []
    wp_report: list[dict] = []
    for i, w in enumerate(wps):
        t_arr = wp_arrive_times.get(i, total)
        idx = min(range(len(times)), key=lambda k: abs(times[k] - t_arr))
        q = np.array([convention.deg_to_dh(j, points[idx]["deg"][j]) for j in range(6)])
        p_act, R_act = ik.fk(frame, q)
        pos_err = float(np.linalg.norm(p_act - w["pos"])) * 1000.0
        env = check_reach_envelope(w["pos"])
        status = last_status if env is None else "unreachable"
        if pos_err > WAYPOINT_ERR_LIMIT_MM:
            status = "unreachable"
        item = {"wp": i, "pos_err_mm": round(pos_err, 3), "status": status,
                "t": round(float(t_arr), 4), "target_m": [round(float(v), 4) for v in w["pos"]],
                "degs": list(points[idx]["deg"]),
                "grip": float(points[idx]["grip"])}
        if env is not None:
            item["note"] = env
        wp_report.append(item)
        if pos_err > WAYPOINT_ERR_LIMIT_MM or status != "ok":
            errors.append({"wp": i, "pos_err_mm": round(pos_err, 3), "status": status})

    max_speed = 0.0
    for a, b in zip(points, points[1:]):
        dtt = max(b["t"] - a["t"], 1e-9)
        if dtt > 1e-6:
            max_speed = max(max_speed, max(abs(b["deg"][j] - a["deg"][j]) for j in range(6)) / dtt)

    return {
        "points": points,
        "errors": errors,
        "waypoints": wp_report,
        "duration": round(float(total), 4),
        "max_joint_speed_deg": round(float(max_speed), 2),
        "yaw_deg": float(yaw_deg),
    }


def plan_waypoints(waypoints: Sequence[Sequence[Any]], speed: float = 1.0,
                   q_start: Sequence[float] | None = None,
                   ik=None, frame: str = DEFAULT_FRAME,
                   yaw_deg: float | None = None, dt: float = PLAN_DT) -> dict:
    """规划一条从当前姿态出发、依次经过各路点的 10 ms 轨迹。

    :param waypoints: [[x, y, z, dwell, grip], ...]，米；grip ∈ 0–110 角度 / open/close/None
    :param speed: 1.0 = 巡航 0.25 m/s；2.0 翻倍，0.5 减半（dwell 同步缩放）
    :param q_start: 起始关节角（rad，12 维）；None = 机械臂 home（DH 全零）
    :param ik: 现成的 ArmIK 实例；None = 自动加载 robot.ik（缺失时用临时实现）
    :param yaw_deg: 工具绕世界 z 的朝向（度）。None = 沿用（0°），保证腕部不翻转
    :returns: 契约 §3 的 plan 结果（外加 waypoints 逐点明细）
    """
    backend = "injected"
    if ik is None:
        ik, backend = load_ik()
    wps = parse_waypoints(waypoints)
    if q_start is None:
        q_start = np.zeros(12, dtype=float)
    q_start = np.asarray(q_start, dtype=float).copy()
    speed = max(float(speed), 0.05)
    if yaw_deg is None:
        yaw_deg = 0.0

    result = _plan_once(wps, speed, q_start, ik, frame, yaw_deg, dt)

    # 关节速度过高 → 自动放慢重新规划（最多 2 次，避免脚软的伺服跟不上）。
    # 已经不可达的轨迹不再放慢：那只会把 38 s 的错误轨迹做得很慢，没有意义。
    first_pass_ok = len([e for e in result["errors"] if e["pos_err_mm"] > WAYPOINT_ERR_LIMIT_MM]) == 0
    for _ in range(2 if first_pass_ok else 0):
        if result["max_joint_speed_deg"] <= MAX_JOINT_SPEED_DEG:
            break
        ratio = result["max_joint_speed_deg"] / (MAX_JOINT_SPEED_DEG * 0.8)
        slowed = max(speed / max(ratio, 1.0), 0.05)
        result = _plan_once(wps, slowed, q_start, ik, frame, yaw_deg, dt)
        result["speed_used"] = slowed

    reachable = len([e for e in result["errors"] if e["pos_err_mm"] > WAYPOINT_ERR_LIMIT_MM]) == 0
    # 契约 §7 的 wp 列表 + 供 Arm 门面直接播放的 traj（[(t, 6 度, 夹爪度)]）
    wp_list = [{"t": w["t"], "pos": w["target_m"], "grip": w["grip"],
                "degs": w["degs"], "pos_err_mm": w["pos_err_mm"], "status": w["status"]}
               for w in result["waypoints"]]
    traj = [[p["t"], list(p["deg"]), p["grip"]] for p in result["points"]]
    err_text = None
    if not reachable:
        err_text = "；".join("路点 %d 不可达（误差 %.1f mm%s）" %
                             (e["wp"], e["pos_err_mm"],
                              "，" + e.get("note", "") if "note" in e else "")
                             for e in result["errors"])
    result.update({
        "ok": True,
        "reachable": bool(reachable),
        "wp": wp_list,
        "traj": traj,
        "err": err_text,
        "backend": backend,
        "frame": frame,
        "speed": float(speed),
        "dt": float(dt),
        "waypoint_count": len(wps),
    })
    return result


def plan(waypoints, speed: float = 1.0, **kw) -> dict:
    """plan_waypoints 的短别名（便于 Arm 门面直接转调）。"""
    return plan_waypoints(waypoints, speed=speed, **kw)


def gripper_ctrl(grip_deg: float) -> float:
    """夹爪界面角度（度）→ MuJoCo 执行器 ctrl（唯一换算源：convention）。"""
    return convention.gripper_deg_to_ctrl(float(grip_deg))


__all__ = ["plan_waypoints", "plan", "parse_waypoints", "load_ik", "gripper_ctrl",
           "PLAN_DT", "BASE_SPEED_MPS", "WAYPOINT_ERR_LIMIT_MM", "REACH_RADIUS_M",
           "REACH_Z_MIN", "REACH_Z_MAX", "GRIP_OPEN_DEG", "GRIP_CLOSE_DEG",
           "check_reach_envelope", "_MinkArmIK"]
