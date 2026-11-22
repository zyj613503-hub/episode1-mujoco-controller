#!/usr/bin/env python3
"""逆运动学与自干涉检测（接口契约 §8）。

对外只暴露 ArmIK：
    fk(frame, q=None)                -> (pos_m(3), R(3,3))
    tool_down_rotation(yaw_deg=0)     -> R(3,3)  工具轴竖直朝下
    solve_pose(frame, pos_m, R, ...)  -> {q, status, pos_err_mm, ori_err_deg}
    self_collision(q)                -> bool
    limits_ok(q)                     -> bool

实现要点（mink 1.4 / MuJoCo 3.17）：
* 只用「空场景」episode1.xml（nq=12：6 个臂关节 + 6 个夹爪关节）。
* 夹爪关节在 IK 里速度上限设为 0（真机是闭环机构，不能自由解算），
  由 PostureTask 保持成 q_seed 的值。
* status 五条判据严格按契约顺序：unreachable -> wrist_flip -> discontinuity
  -> self_collision -> ok。
* 自干涉：MuJoCo 场景里 episode1.xml 的几何体默认 conaffinity=0（臂自己不和
  自己碰），这里用 MjSpec 另编译一份「只用于碰撞检测」的模型，给每个连杆一个
  contype 位、conaffinity 里排除自己与相邻连杆，于是 mj_forward 后
  d.ncon 里剩下的就是真正的自我干涉。
"""

from __future__ import annotations

import math
import os
import threading

import numpy as np

_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_PKG_DIR)

import mujoco  # noqa: E402
import mink  # noqa: E402

from . import convention as C  # noqa: E402


# ---------------------------------------------------------------- 小工具
def resolve_path(path):
    """把契约里的相对路径（相对项目根）解析成绝对路径。"""
    if path is None:
        return None
    p = os.path.expanduser(str(path))
    if os.path.isabs(p):
        return p
    return os.path.normpath(os.path.join(ROOT, p))


def rotation_x(deg):
    a = math.radians(float(deg))
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def rotation_y(deg):
    a = math.radians(float(deg))
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def rotation_z(deg):
    a = math.radians(float(deg))
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def euler_xyz_deg_to_matrix(deg):
    """欧拉角 XYZ（度）-> 旋转矩阵，R = Rz(rz) @ Ry(ry) @ Rx(rx)。"""
    rx, ry, rz = [float(v) for v in deg]
    return rotation_z(rz) @ rotation_y(ry) @ rotation_x(rx)


def matrix_to_euler_xyz_deg(R):
    """旋转矩阵 -> 欧拉角 XYZ（度），与 euler_xyz_deg_to_matrix 互逆。"""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))
    ry = math.asin(sy)
    if abs(sy) < 0.999999:
        rx = math.atan2(R[2, 1], R[2, 2])
        rz = math.atan2(R[1, 0], R[0, 0])
    else:  # 万向锁
        rx = math.atan2(-R[1, 2], R[1, 1])
        rz = 0.0
    return np.array([math.degrees(rx), math.degrees(ry), math.degrees(rz)])


# ---------------------------------------------------------------- 契约文字
IK_TEXT = {
    "ok": "可达",
    "self_collision": "自我干涉",
    "wrist_flip": "不可达（需翻腕）",
    "discontinuity": "不可达（关节突跳）",
    "unreachable": "不可达（够不到）",
}

POS_ERR_LIMIT_MM = 2.0
ORI_ERR_LIMIT_DEG = 8.0
LIMIT_TOL_RAD = 0.02
JUMP_LIMIT_DEG = 60.0
# 只有目标离种子位姿小于这个距离（mm）时才用 60° 突跳判据；
# 目标本身很远时腕关节转得多是正常运动，不该判成 IK 换分支。
JUMP_NEAR_MM = 60.0


class ArmIK:
    """机械臂逆运动学（mink）与自干涉检测。线程安全。"""

    def __init__(self, model_xml=None):
        model_xml = model_xml or C.DEFAULT_MODEL_XML
        self.model_path = resolve_path(model_xml)
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError("找不到模型文件: %s" % self.model_path)

        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.nq = int(self.model.nq)
        self.home_q = np.zeros(self.nq, dtype=float)

        self._lock = threading.RLock()
        # 夹爪关节：IK 时冻结（速度 0）
        self._freeze = {}
        name_by_adr = {}
        for j in range(self.model.njnt):
            name_by_adr[int(self.model.jnt_dofadr[j])] = self.model.joint(j).name
        for dof in range(6, self.model.nv):
            nm = name_by_adr.get(dof)
            if nm:
                self._freeze[nm] = 0.0

        self._collision_model = None
        self._collision_data = None
        self._build_collision_model()

    # ------------------------------------------------------------ 模型
    def _build_collision_model(self):
        """另外编译一份开了自碰撞的模型：每根连杆一个 contype 位，相邻连杆互不检测。"""
        groups = {
            "world": 0,
            "link1": 1, "link2": 2, "link3": 3,
            "link4": 4, "link5": 5, "link6": 6,
            "gripper_mount": 7, "link_gear1": 7, "link_gear2": 7,
            "link_finger1": 7, "link_finger2": 7,
            "link_f1": 7, "link_f2": 7,
        }
        adjacent = {0: (1,), 1: (0, 2), 2: (1, 3), 3: (2, 4), 4: (3, 5),
                    5: (4, 6), 6: (5, 7), 7: (6,)}
        nbits = 8
        full = (1 << nbits) - 1
        try:
            spec = mujoco.MjSpec.from_file(self.model_path)
            for geom in spec.geoms:
                body = geom.parent.name if geom.parent is not None else "world"
                bit = groups.get(body, 0)
                mask = full & ~(1 << bit)
                for other in adjacent.get(bit, ()):
                    mask &= ~(1 << other)
                geom.contype = 1 << bit
                geom.conaffinity = mask
            model = spec.compile()
            data = mujoco.MjData(model)
            self._collision_model = model
            self._collision_data = data
        except Exception:  # pragma: no cover - 兜底：没有自碰撞也别让 IK 挂
            self._collision_model = None
            self._collision_data = None

    # ------------------------------------------------------------ FK
    @staticmethod
    def _as_q(q, nq):
        if q is None:
            return np.zeros(nq, dtype=float)
        arr = np.asarray(q, dtype=float).reshape(-1)
        out = np.zeros(nq, dtype=float)
        n = min(nq, arr.size)
        out[:n] = arr[:n]
        return out

    def fk(self, frame=None, q=None):
        """frame 可以是 site/body 名（默认 tcp），返回 (位置米, 旋转矩阵)。"""
        frame = frame or C.TCP_SITE
        qq = self._as_q(q, self.nq)
        with self._lock:
            self.data.qpos[:] = qq
            mujoco.mj_forward(self.model, self.data)
            return self._frame_pose_locked(frame)

    def _frame_pose_locked(self, frame):
        try:
            sid = int(self.model.site(frame).id) if isinstance(frame, str) else int(frame)
            pos = np.array(self.data.site_xpos[sid], dtype=float, copy=True)
            mat = np.array(self.data.site_xmat[sid], dtype=float, copy=True).reshape(3, 3)
            return pos, mat
        except Exception:
            pass
        try:
            bid = int(self.model.body(frame).id) if isinstance(frame, str) else int(frame)
            pos = np.array(self.data.xpos[bid], dtype=float, copy=True)
            mat = np.array(self.data.xmat[bid], dtype=float, copy=True).reshape(3, 3)
            return pos, mat
        except Exception:
            raise KeyError("模型里没有 frame: %r" % (frame,))

    # ------------------------------------------------------------ 姿态
    def tool_down_rotation(self, yaw_deg=0.0):
        """工具轴（tcp site 的 +z）竖直朝下、yaw 绕世界 z 轴。

        实测 home 处 tcp 的 R = [x->世界+z, y->世界-y, z->世界+x]，
        把 z 转到世界 -z、x 转到世界 +x，得到基准姿态 diag(1,-1,-1)，
        再左乘绕世界 z 的 yaw 旋转。
        """
        base = np.diag([1.0, -1.0, -1.0])
        return rotation_z(yaw_deg) @ base

    # ------------------------------------------------------------ IK
    def _make_cfg(self, q_seed):
        cfg = mink.Configuration(self.model)
        cfg.update(q=q_seed)
        return cfg

    def _limits(self):
        # 臂关节 3 rad/s；夹爪关节锁死（真机是闭环机构，不参与 IK）
        velocities = {C.JOINT_NAMES[i]: 3.0 for i in range(len(C.JOINT_NAMES))}
        velocities.update(self._freeze)
        return [mink.ConfigurationLimit(self.model),
                mink.VelocityLimit(self.model, velocities)]

    # 重启用的腕部扰动（度）：直接平移解会被关节限位挡住时换个盆地再解
    RESTART_OFFSETS = (
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 92.0, 0.0, 92.0),
        (0.0, 0.0, 0.0, -30.0, 0.0, -30.0),
        (0.0, 0.0, 0.0, 180.0, 0.0, 0.0),
    )

    def _solve_once(self, frame, target_pos, target_R, rot, seed, iters, dt):
        seed = self._as_q(seed, self.nq)      # 允许只给 6 个关节角
        cfg = mink.Configuration(self.model)
        cfg.update(q=seed)
        task = mink.FrameTask(frame, "site", position_cost=1.0, orientation_cost=1.0,
                              gain=0.6, lm_damping=0.5)
        posture = mink.PostureTask(self.model, cost=1e-4)
        posture.set_target(seed)
        limits = self._limits()
        se3_target = mink.SE3.from_rotation_and_translation(rotation=rot, translation=target_pos)

        with self._lock:
            for _ in range(iters):
                task.set_target(se3_target)
                try:
                    vel = mink.solve_ik(cfg, [task, posture], dt, solver="daqp",
                                        damping=1e-12, limits=limits)
                except Exception:
                    # 求解器在退化位形下可能抛 NoSolutionFound：停下，按残差判定
                    break
                cfg.integrate_inplace(vel, dt)
                T = cfg.get_transform_frame_to_world(frame, "site")
                if float(np.linalg.norm(T.translation() - target_pos)) < 5e-5:
                    break
            q = np.array(cfg.q, dtype=float, copy=True)
            T = cfg.get_transform_frame_to_world(frame, "site")
            pos = np.array(T.translation(), dtype=float, copy=True)
            Rgot = np.array(T.rotation().as_matrix(), dtype=float, copy=True)

        pos_err_mm = float(np.linalg.norm(pos - target_pos) * 1000.0)
        Rrel = target_R.T @ Rgot
        cos = max(-1.0, min(1.0, (float(np.trace(Rrel)) - 1.0) / 2.0))
        ori_err_deg = float(math.degrees(math.acos(cos)))
        return q, pos_err_mm, ori_err_deg

    def solve_pose(self, frame=None, pos_m=None, R=None, q_seed=None, iters=40):
        """解 IK。pos_m 米、R 3x3 世界系姿态。返回契约 §8 的 dict。"""
        frame = frame or C.TCP_SITE
        target_pos = np.asarray(pos_m, dtype=float).reshape(3)
        target_R = np.asarray(R, dtype=float).reshape(3, 3)
        seed = self._as_q(q_seed, self.nq)
        iters = max(1, int(iters))
        dt = 0.02

        try:
            rot = mink.SO3.from_matrix(target_R)
        except Exception:
            # 非正交矩阵：正交化后再用
            u, _, vt = np.linalg.svd(target_R)
            d = np.sign(np.linalg.det(u @ vt))
            rot = mink.SO3.from_matrix(u @ np.diag([1.0, 1.0, d]) @ vt)

        # 种子组：先用调用方给的 q_seed，不行再换 home 与几个常见工作位姿。
        # 只解一次很容易落到「位姿对但关节反向」的另一支，导致 status 误报 discontinuity。
        seed_list = [seed]
        home = np.zeros_like(seed)
        seed_list.append(home)
        for a4 in (90.0, -90.0):
            s = home.copy()
            s[3] = math.radians(a4)
            seed_list.append(s)

        best = None   # (rank, err, q, e_pos, e_ori, status)
        for base in seed_list:
            for k, off in enumerate(self.RESTART_OFFSETS):
                if k == 0:
                    s = base
                else:
                    s = base.copy()
                    for i in range(6):
                        if off[i]:
                            s[i] = s[i] + math.radians(off[i])
                    # 扰动后超限位就先夹回限位内，避免一开始就被挡住
                    for i in range(6):
                        lo, hi = C.JOINT_LIMIT_RAD[i]
                        s[i] = min(hi, max(lo, s[i]))
                q, e_pos, e_ori = self._solve_once(
                    frame, target_pos, target_R, rot, s, iters, dt)
                status = self._classify(q, e_pos, e_ori, seed, frame, target_pos)
                # 状态优先，其次才是误差：宁可差一点点也要连续、不翻腕的解
                rank = 0 if status == "ok" else 1
                err = e_pos + 10.0 * e_ori
                if best is None or (rank, err) < (best[0], best[1]):
                    best = (rank, err, q, e_pos, e_ori, status)
                if rank == 0 and e_pos < 1.0 and e_ori < 0.5:
                    best = (rank, err, q, e_pos, e_ori, status)
                    break
            if best is not None and best[0] == 0:
                break

        _, _, q, pos_err_mm, ori_err_deg, status = best
        return {"q": q, "status": status, "text": IK_TEXT[status],
                "pos_err_mm": pos_err_mm, "ori_err_deg": ori_err_deg}

    def _classify(self, q, pos_err_mm, ori_err_deg, seed, frame=None, target_pos=None):
        """契约 §8 的五条判据，顺序严格不可换。

        第 3 条（关节突跳）只在「目标离种子位姿很近」时才判：
        目标本来就远（比如横跨半个工作台）时，腕关节转 100°+ 是正常运动，
        不是 IK 换分支造成的突跳，否则 /api/v2/pose 几乎全部会误报。
        """
        if pos_err_mm > POS_ERR_LIMIT_MM or ori_err_deg > ORI_ERR_LIMIT_DEG:
            return "unreachable"
        if self._limits_violated(q, LIMIT_TOL_RAD):
            return "wrist_flip"
        if (target_pos is not None
                and self._jump_deg(q, seed) > JUMP_LIMIT_DEG
                and self._seed_is_near(target_pos, seed, frame)):
            return "discontinuity"
        if self.self_collision(q):
            return "self_collision"
        return "ok"

    def _seed_is_near(self, target, seed, frame=None):
        """种子的 FK 位姿是否已经贴近目标（近才允许用 60° 突跳判据）。"""
        try:
            frame = frame or C.TCP_SITE
            pos_seed, R_seed = self.fk(frame, seed)
            gap_mm = float(np.linalg.norm(np.asarray(target, dtype=float) - pos_seed) * 1000.0)
            if gap_mm > JUMP_NEAR_MM:
                return False
            return True
        except Exception:
            return True

    # ------------------------------------------------------------ 限位 / 干涉
    def _limits_violated(self, q, tol):
        qq = np.asarray(q, dtype=float).reshape(-1)
        for i in range(min(6, qq.size)):
            lo, hi = C.JOINT_LIMIT_RAD[i]
            if qq[i] < lo - tol or qq[i] > hi + tol:
                return True
        return False

    def limits_ok(self, q):
        return not self._limits_violated(q, 0.0)

    @staticmethod
    def _jump_deg(q, seed):
        qq = np.asarray(q, dtype=float).reshape(-1)
        ss = np.asarray(seed, dtype=float).reshape(-1)
        n = min(6, qq.size, ss.size)
        if n == 0:
            return 0.0
        # 逐关节判，且角度差折回到 (-180, 180] 再取绝对值：359° 要算成 1°
        diff = np.degrees(qq[:n] - ss[:n])
        diff = (diff + 180.0) % 360.0 - 180.0
        return float(np.max(np.abs(diff)))

    def self_collision(self, q):
        """机械臂自身几何干涉（不含场景物体）。"""
        model = self._collision_model
        data = self._collision_data
        if model is None or data is None:
            return False
        qq = self._as_q(q, self.nq)
        with self._lock:
            data.qpos[:] = qq
            mujoco.mj_forward(model, data)
            ncon = int(data.ncon)
            for i in range(ncon):
                if float(data.contact[i].dist) < 0.0:
                    return True
        return False

    # ------------------------------------------------------------ 便利方法
    def fk_tcp(self, q=None):
        return self.fk(C.TCP_SITE, q)

    def fk_flange(self, q=None):
        return self.fk(C.FLANGE_SITE, q)

    def eig_check(self):
        """(nq, nu, neq) 供 selftest 用。"""
        return int(self.model.nq), int(self.model.nu), int(self.model.neq)


__all__ = [
    "ROOT", "ArmIK", "IK_TEXT", "resolve_path",
    "rotation_x", "rotation_y", "rotation_z",
    "euler_xyz_deg_to_matrix", "matrix_to_euler_xyz_deg",
]
