#!/usr/bin/env python3
"""MuJoCo 仿真驱动（接口契约 §1、§4、§7 的仿真侧）。

职责：
* 加载 model/scene_episode1.xml（nq=26，含桌子/球/盘子的 free joint）。
* 500 Hz 步进线程：关节目标按 180°/s 斜坡逼近，夹爪同样按度斜坡。
* 同一线程里以 30 fps 调 mujoco.Renderer 出图，PNG 编码（纯标准库，无 pillow）。
* 相机默认 az=138 el=-19 dist=1.22 lookat=(0.20,0.10,0.13)。

线程模型：
    sim 线程  —— 独占 MjData：mj_step + 渲染（渲染降频到 30 fps）
    其它线程  —— 只拿 self._lock 读状态 / 写目标 / 拷贝最近一帧 PNG

注意：永远不要 qpos[:] = 0 —— 球和盘子的 free joint 会被甩飞。只有在 load()
时用 mj_resetData（回到模型自带初始位）才是安全的。
"""

from __future__ import annotations

import math
import os
import struct
import threading
import time
import zlib

import numpy as np

import mujoco

from . import convention as C
from .ik import matrix_to_euler_xyz_deg, resolve_path

# ---------------------------------------------------------------- 常量
STEP_HZ = 500.0
RENDER_HZ = 30.0
JOINT_RAMP_DPS = 180.0
GRIPPER_RAMP_DPS = 180.0

DEFAULT_WIDTH = 672
DEFAULT_HEIGHT = 456

DEFAULT_CAM = {
    "az": 138.0,
    "el": -19.0,
    "dist": 1.22,
    "lookat": [0.20, 0.10, 0.13],
}

ARM_DEG_RANGE = [tuple(float(v) for v in r) for r in C.MOTOR_RANGE_DEG]
GRIPPER_DEG_RANGE = tuple(float(v) for v in C.GRIPPER_RANGE_DEG)


def encode_png(rgb, level=3):
    """numpy uint8 (h,w,3) -> PNG 字节（无 pillow，纯 zlib）。

    level=3 是速度/体积折中：级别 6 要多花 ~13 ms 才省 20 KB，30 fps 不划算。
    """
    arr = np.ascontiguousarray(rgb, dtype=np.uint8)
    if arr.ndim == 2:
        arr = np.repeat(arr[:, :, None], 3, axis=2)
    h, w = int(arr.shape[0]), int(arr.shape[1])
    raw = b"".join(b"\x00" + arr[i].tobytes() for i in range(h))

    def chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(raw, int(level)))
            + chunk(b"IEND", b""))


def _clamp(v, lo, hi):
    return max(lo, min(hi, float(v)))


class SimDriver:
    """MuJoCo 仿真：500 Hz 步进 + 30 fps 渲染。所有公开方法线程安全。"""

    def __init__(self, scene_xml=None, model_xml=None,
                 width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT):
        self.scene_xml = resolve_path(scene_xml or C.DEFAULT_SCENE_XML)
        self.model_xml = resolve_path(model_xml or C.DEFAULT_MODEL_XML)
        self.width = int(width)
        self.height = int(height)

        self._lock = threading.RLock()
        self._thread = None
        self._running = False
        self._closed = False

        self.model = None
        self.data = None
        self.renderer = None
        self.png = b""
        self.png_version = 0
        self.frames = 0
        self.fps = 0.0
        self.render_error = ""
        self.render_fail_count = 0
        self.loop_error = ""
        self.loop_error_count = 0

        self._cam = dict(DEFAULT_CAM)
        self._cam_defaults = dict(DEFAULT_CAM)

        # 关节目标（度）与斜坡当前值（度）
        self._target_deg = [float(v) for v in C.OFFSET]
        self._ramp_deg = [float(v) for v in C.OFFSET]
        self._target_grip = 0.0
        self._ramp_grip = 0.0

        self._estop = False
        self._step_count = 0
        self._frame_times = []

        # 轨迹播放（Arm 装载，sim 线程推进）
        self._traj = None        # [(t, degs[6], grip)]
        self._play_t = 0.0
        self._play_state = "stop"  # stop | play | pause
        self._play_speed = 1.0
        self._play_loop = False

        self._load_locked()

    # ------------------------------------------------------------ 加载
    def _load_locked(self):
        if not os.path.isfile(self.scene_xml):
            raise FileNotFoundError("找不到场景文件: %s" % self.scene_xml)
        model = mujoco.MjModel.from_xml_path(self.scene_xml)
        # 渲染缓冲尺寸必须在创建 Renderer 之前定好
        model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), self.width)
        model.vis.global_.offheight = max(int(model.vis.global_.offheight), self.height)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        self.model = model
        self.data = data
        self._sync_from_state_locked()
        self.png = b""
        self.png_version = 0
        self.frames = 0
        self.fps = 0.0
        self.render_error = ""
        self.render_fail_count = 0
        self.loop_error = ""
        self.loop_error_count = 0
        # 渲染器必须由「创建它的那个线程」使用（macOS 的 CGL 上下文不可跨线程），
        # 所以渲染器一律在步进线程里创建/使用/销毁，别的线程只读 png 缓存。
        self._render_size = (self.width, self.height)

    # ---- 以下两个只在步进线程里调用 ----
    def _open_renderer_in_thread(self):
        self._close_renderer_in_thread()
        if self.model is None:
            return False
        self.model.vis.global_.offwidth = max(int(self.model.vis.global_.offwidth), self.width)
        self.model.vis.global_.offheight = max(int(self.model.vis.global_.offheight), self.height)
        try:
            self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
            self._render_size = (self.width, self.height)
            self.render_error = ""
            return True
        except Exception as exc:
            self.renderer = None
            self.render_error = str(exc)
            return False

    def _close_renderer_in_thread(self):
        renderer = self.renderer
        self.renderer = None
        if renderer is not None:
            try:
                renderer.close()
            except Exception:
                pass

    def _sync_from_state_locked(self):
        degs = [C.dh_to_deg(i, self.data.qpos[i]) for i in range(6)]
        self._target_deg = list(degs)
        self._ramp_deg = list(degs)
        grip = C.gripper_ctrl_to_deg(self.data.ctrl[6] if self.model.nu > 6 else 0.0)
        self._target_grip = grip
        self._ramp_grip = grip
        # ctrl 初值跟关节/夹爪对齐，避免上电第一帧猛跳
        for i in range(6):
            self.data.ctrl[i] = self.data.qpos[i]
        if self.model.nu > 6:
            self.data.ctrl[6] = C.gripper_deg_to_ctrl(grip)

    # ------------------------------------------------------------ 线程
    def start(self):
        with self._lock:
            if self._running or self._closed:
                return self._running
            self._running = True
            self._thread = threading.Thread(target=self._run, name="sim-500hz", daemon=True)
            self._thread.start()
        return True

    def close(self):
        with self._lock:
            self._closed = True
            self._running = False
            thread = self._thread
            self._thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=3.0)
        # 渲染器由步进线程自己关闭（_run 的 finally）

    @property
    def running(self):
        return bool(self._running)

    def _run(self):
        dt = float(self.model.opt.timestep) or 0.002
        last = time.perf_counter()
        next_render = 0.0
        with self._lock:
            self._open_renderer_in_thread()
        try:
            # 步进线程绝不允许因为一次渲染/步进异常就死掉：捕获后记录、稍息、继续跑
            while True:
                try:
                    self._loop(dt, last, next_render)
                    break
                except Exception as exc:      # noqa: BLE001 - 兜底必须最宽
                    with self._lock:
                        self.loop_error = "步进线程异常：%s" % exc
                        self.loop_error_count += 1
                        running = self._running
                    if not running:
                        break
                    last = time.perf_counter()
                    next_render = 0.0
                    time.sleep(0.01)
        finally:
            with self._lock:
                self._close_renderer_in_thread()

    def _loop(self, dt, last, next_render):
        while True:
            with self._lock:
                if not self._running:
                    break
            now = time.perf_counter()
            elapsed = now - last
            last = now
            if elapsed > 0.25:      # 卡顿（比如关窗口）后别追帧
                elapsed = 0.25
            n = int(round(elapsed / dt))
            if n < 1:
                time.sleep(0.0005)
                continue
            n = min(n, 40)
            with self._lock:
                if not self._running:
                    break
                for _ in range(n):
                    self._step_locked(dt)
                self._step_count += n
                rebuild = (self.width, self.height) != self._render_size or self.renderer is None
            # 渲染器只属于这个线程，重建也在锁外做（重建要几百毫秒，别挡住 state()）
            if rebuild:
                with self._lock:
                    size = (self.width, self.height)
                if self._open_renderer_in_thread():
                    next_render = 0.0
                elif (self.width, self.height) == size:
                    next_render = time.perf_counter() + 1.0  # 建不起来就慢点重试
            started = time.perf_counter()
            if next_render <= started:
                # 渲染绝不在持锁状态下做：其它线程调 state()/set_joints() 绝不被卡
                # 下一拍按「本拍开始 + 周期」排，避免渲染耗时被重复计入间隔
                if self._render_frame():
                    next_render = started + 1.0 / RENDER_HZ
                    if next_render < started:  # 落后太多就不要雪崩式补帧
                        next_render = started
                else:
                    next_render = started + 1.0
            sleep = dt - (time.perf_counter() - now)
            if sleep > 0:
                time.sleep(sleep)

    # ------------------------------------------------------------ 每步
    def _step_locked(self, dt):
        self._advance_playback_locked(dt)

        max_step = JOINT_RAMP_DPS * dt
        for i in range(6):
            target = self._target_deg[i]
            cur = self._ramp_deg[i]
            delta = target - cur
            if delta != 0.0:
                delta = _clamp(delta, -max_step, max_step)
                self._ramp_deg[i] = cur + delta
            self.data.ctrl[i] = C.deg_to_dh(i, self._ramp_deg[i])

        gmax = GRIPPER_RAMP_DPS * dt
        g_delta = self._target_grip - self._ramp_grip
        if g_delta != 0.0:
            g_delta = _clamp(g_delta, -gmax, gmax)
            self._ramp_grip += g_delta
        if self.model.nu > 6:
            self.data.ctrl[6] = C.gripper_deg_to_ctrl(self._ramp_grip)

        mujoco.mj_step(self.model, self.data)

    def _advance_playback_locked(self, dt):
        if self._play_state != "play" or not self._traj:
            return
        self._play_t += dt * self._play_speed
        traj = self._traj
        duration = traj[-1][0]
        if self._play_t >= duration:
            if self._play_loop and duration > 1e-6:
                self._play_t = self._play_t % duration
            else:
                self._play_t = duration
                self._play_state = "pause"
        degs, grip = _sample_traj(traj, self._play_t)
        for i in range(6):
            self._target_deg[i] = _clamp(degs[i], *ARM_DEG_RANGE[i])
            self._ramp_deg[i] = self._target_deg[i]
        self._target_grip = _clamp(grip, *GRIPPER_DEG_RANGE)
        self._ramp_grip = self._target_grip

    # ------------------------------------------------------------ 渲染
    def _render_frame(self):
        """渲染一帧并发布成 self.png。

        只在步进线程里调用，且**不持锁**：GLFW/CGL 上下文与 mj_step 同线程，
        其它线程永远不碰 renderer，只读 self.png 缓存。
        """
        renderer = self.renderer
        if renderer is None:
            return False
        with self._lock:
            cam_state = dict(self._cam)
        try:
            cam = mujoco.MjvCamera()
            cam.azimuth = float(cam_state["az"])
            cam.elevation = float(cam_state["el"])
            cam.distance = float(cam_state["dist"])
            cam.lookat[:] = cam_state["lookat"]
            renderer.update_scene(self.data, camera=cam)
            img = renderer.render()
            png = encode_png(img)
        except Exception as exc:
            with self._lock:
                self.render_error = "渲染失败：%s" % exc
                self.render_fail_count += 1
            # 上下文坏了就丢掉，下一拍重建
            try:
                self._close_renderer_in_thread()
            except Exception:
                self.renderer = None
            return False
        now = time.perf_counter()
        with self._lock:
            self.png = png
            self.png_version += 1
            self.frames += 1
            self.render_error = ""
            self._frame_times.append(now)
            # 只保留最近 1 秒的出帧时间戳
            self._frame_times = [t for t in self._frame_times if now - t <= 1.0]
            times = self._frame_times
            if len(times) >= 2:
                span = times[-1] - times[0]
                if span > 1e-6:
                    self.fps = (len(times) - 1) / span
            elif len(times) == 1:
                self.fps = 1.0
        return True

    def _wait_frame(self, timeout=3.0, version=None):
        """等步进线程产出一帧（version=None 表示「只要有一帧就行」）。"""
        end = time.time() + timeout
        while time.time() < end:
            with self._lock:
                if version is None:
                    if self.png:
                        return True
                elif self.png_version > version:
                    return True
                if not self._running:
                    return bool(self.png)
            time.sleep(0.01)
        return False

    def render_image(self):
        """返回最近一帧 PNG 字节；还没有就等一帧（不跨线程渲染）。"""
        with self._lock:
            if self.png and self.png_version > 0:
                return bytes(self.png)
        self._wait_frame(timeout=3.0)
        with self._lock:
            return bytes(self.png)

    def resize(self, width, height):
        width = int(_clamp(width, 160, 3840))
        height = int(_clamp(height, 120, 2160))
        with self._lock:
            if width == self.width and height == self.height:
                return
            self.width, self.height = width, height
            version = self.png_version
        # 等步进线程按新尺寸重建并出帧
        self._wait_frame(timeout=5.0, version=version)

    def render_at(self, width=None, height=None):
        if width or height:
            self.resize(width or self.width, height or self.height)
        return self.render_image()

    # ------------------------------------------------------------ 相机
    def set_cam(self, az=None, el=None, dist=None, lookat=None):
        with self._lock:
            if az is not None:
                self._cam["az"] = float(az) % 360.0
            if el is not None:
                self._cam["el"] = _clamp(el, -89.0, 89.0)
            if dist is not None:
                self._cam["dist"] = _clamp(dist, 0.2, 6.0)
            if lookat is not None:
                vals = np.asarray(lookat, dtype=float).reshape(-1)
                if vals.size >= 3:
                    self._cam["lookat"] = [float(v) for v in vals[:3]]
            return dict(self._cam)

    def reset_cam(self):
        with self._lock:
            self._cam = dict(self._cam_defaults)
            return dict(self._cam)

    def camera(self):
        with self._lock:
            return dict(self._cam)

    # ------------------------------------------------------------ 目标
    def set_joints(self, degs):
        vals = np.asarray(degs, dtype=float).reshape(-1)
        if vals.size < 6:
            raise ValueError("需要 6 个关节角")
        with self._lock:
            if self._estop:
                self._estop = False
            for i in range(6):
                self._target_deg[i] = _clamp(vals[i], *ARM_DEG_RANGE[i])

    def set_gripper(self, deg):
        with self._lock:
            self._target_grip = _clamp(float(deg), *GRIPPER_DEG_RANGE)

    def estop(self):
        with self._lock:
            self._estop = True
            self._play_state = "stop"
            self._play_t = 0.0
            self._target_deg = list(self._ramp_deg)
            self._target_grip = float(self._ramp_grip)

    def home(self):
        with self._lock:
            self._estop = False
            self._play_state = "stop"
            self._traj = None
            self._target_deg = [float(v) for v in C.OFFSET]
            self._target_grip = 0.0

    def clear_estop(self):
        with self._lock:
            self._estop = False

    # ------------------------------------------------------------ 轨迹
    def set_trajectory(self, traj):
        """traj: [(t, [6 个电机度数], 夹爪度数)]，t 单调递增。"""
        clean = []
        for item in traj or []:
            t, degs, grip = item
            clean.append((float(t), [float(v) for v in degs], float(grip)))
        clean.sort(key=lambda x: x[0])
        with self._lock:
            self._traj = clean
            self._play_t = 0.0
            self._play_state = "stop"

    def play(self, action="play", speed=1.0, loop=False):
        with self._lock:
            if not self._traj:
                return False
            action = (action or "play").lower()
            if action == "play":
                self._play_state = "play"
                self._play_speed = max(0.05, float(speed))
                self._play_loop = bool(loop)
                if self._traj and self._play_t >= self._traj[-1][0] - 1e-9:
                    self._play_t = 0.0
            elif action == "pause":
                self._play_state = "pause"
            elif action == "stop":
                self._play_state = "stop"
                self._play_t = 0.0
            else:
                return False
            return True

    def seek(self, t):
        with self._lock:
            if not self._traj:
                return False
            duration = self._traj[-1][0]
            self._play_state = "pause"
            self._play_t = _clamp(t, 0.0, duration)
            degs, grip = _sample_traj(self._traj, self._play_t)
            for i in range(6):
                self._target_deg[i] = _clamp(degs[i], *ARM_DEG_RANGE[i])
                self._ramp_deg[i] = self._target_deg[i]
            self._target_grip = _clamp(grip, *GRIPPER_DEG_RANGE)
            self._ramp_grip = self._target_grip
            return True

    def playback(self):
        with self._lock:
            return {
                "has_traj": bool(self._traj),
                "state": self._play_state,
                "t": float(self._play_t),
                "duration": float(self._traj[-1][0]) if self._traj else 0.0,
                "speed": float(self._play_speed),
                "loop": bool(self._play_loop),
            }

    # ------------------------------------------------------------ 状态
    def _site_pose_locked(self, name):
        sid = int(self.model.site(name).id)
        pos = np.array(self.data.site_xpos[sid], dtype=float, copy=True) * 1000.0
        R = np.array(self.data.site_xmat[sid], dtype=float, copy=True).reshape(3, 3)
        return [round(float(v), 3) for v in pos], [round(float(v), 3) for v in matrix_to_euler_xyz_deg(R)]

    def state(self):
        with self._lock:
            degs = [C.dh_to_deg(i, self.data.qpos[i]) for i in range(6)]
            limit = []
            for i in range(6):
                lo, hi = ARM_DEG_RANGE[i]
                d = degs[i]
                limit.append(bool(d <= lo + 0.5 or d >= hi - 0.5))
            flange_mm, flange_deg = self._site_pose_locked(C.FLANGE_SITE)
            tcp_mm, tcp_deg = self._site_pose_locked(C.TCP_SITE)
            grip = C.gripper_ctrl_to_deg(self.data.ctrl[6]) if self.model.nu > 6 else 0.0
            moving = any(abs(self._target_deg[i] - degs[i]) > 0.25 for i in range(6)) \
                or abs(self._target_grip - grip) > 0.5
            return {
                "ok": True,
                "mode": "sim",
                "connected": bool(self._running),
                "busy": bool(self._play_state == "play" or moving),
                "estop": bool(self._estop),
                "engine": {"name": "MuJoCo", "version": mujoco.__version__},
                "joints": {
                    "deg": [round(float(v), 3) for v in degs],
                    "target": [round(float(v), 3) for v in self._target_deg],
                    "range": [list(r) for r in ARM_DEG_RANGE],
                    "limit": limit,
                },
                "gripper": {
                    "deg": round(float(grip), 3),
                    "target": round(float(self._target_grip), 3),
                    "range": [GRIPPER_DEG_RANGE[0], GRIPPER_DEG_RANGE[1]],
                },
                "flange": {"mm": flange_mm, "deg": flange_deg},
                "tcp": {"mm": tcp_mm, "deg": tcp_deg, "offset_mm": float(C.DEFAULT_TCP_MM)},
                "ik": {"status": "ok", "text": "可达", "pos_err_mm": 0.0, "ori_err_deg": 0.0},
                "play": self.playback(),
                "fps": round(float(self.fps), 1),
                "frames": int(self.frames),
                "render_error": self.render_error,
                "loop_error": self.loop_error,
                "ts": time.time(),
            }

    def qpos(self):
        with self._lock:
            return np.array(self.data.qpos, dtype=float, copy=True)

    def arm_q(self):
        with self._lock:
            return np.array(self.data.qpos[:6], dtype=float, copy=True)

    # ------------------------------------------------------------ 重载
    def load(self, scene_xml=None, model_xml=None):
        was_running = self._running
        if was_running:
            self.close()
        with self._lock:
            if scene_xml:
                self.scene_xml = resolve_path(scene_xml)
            if model_xml:
                self.model_xml = resolve_path(model_xml)
            self._load_locked()
            self._running = False
            self._closed = False
        if was_running:
            self.start()


def _sample_traj(traj, t):
    """线性插值取 (degs, grip)。traj 已按 t 排序。"""
    if not traj:
        return [0.0] * 6, 0.0
    if t <= traj[0][0]:
        return list(traj[0][1]), traj[0][2]
    if t >= traj[-1][0]:
        return list(traj[-1][1]), traj[-1][2]
    lo, hi = 0, len(traj) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if traj[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    t0, d0, g0 = traj[lo]
    t1, d1, g1 = traj[hi]
    span = t1 - t0
    a = 0.0 if span <= 1e-9 else (t - t0) / span
    degs = [d0[i] + (d1[i] - d0[i]) * a for i in range(6)]
    return degs, g0 + (g1 - g0) * a


__all__ = ["SimDriver", "DEFAULT_CAM", "DEFAULT_WIDTH", "DEFAULT_HEIGHT",
           "STEP_HZ", "RENDER_HZ", "JOINT_RAMP_DPS", "encode_png"]
