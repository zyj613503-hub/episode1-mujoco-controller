#!/usr/bin/env python3
"""Arm 门面（接口契约 §7）：组合 SimDriver + ArmIK，供 HTTP 层直接调用。

约定：
* 所有公开方法线程安全、**绝不抛异常**；出错返回 {"ok": False, "error": "中文"}
  并写一条 error 日志。
* 日志是环形缓冲，drain_logs(since) 按 seq 增量取。
* plan / play / seek / generate 先接 robot.planner（另一个 agent 并行实现）：
  没装就返回 {"ok": False, "error": "planner 未就绪"}，不抛异常。
"""

from __future__ import annotations

import importlib
import os
import threading
import time
from collections import deque

import numpy as np

from . import convention as C
from .ik import ArmIK, IK_TEXT, euler_xyz_deg_to_matrix
from .sim import SimDriver

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(ROOT, "scripts")

LOG_MAX = 2000
PLAYER_MISSING = "planner 未就绪"


def _is_num(v):
    try:
        float(v)
        return True
    except (TypeError, ValueError):
        return False


class Arm:
    """上位机后端唯一入口。"""

    def __init__(self, model_xml=None, scene_xml=None, mode="sim"):
        self.model_xml = model_xml or C.DEFAULT_MODEL_XML
        self.scene_xml = scene_xml or C.DEFAULT_SCENE_XML
        self.mode = mode or "sim"

        self._lock = threading.RLock()
        self._log_lock = threading.Lock()
        self._logs = deque(maxlen=LOG_MAX)
        self._seq = 0

        self._sim = None
        self._ik = None
        self._ik_state = {"status": "ok", "text": IK_TEXT["ok"],
                          "pos_err_mm": 0.0, "ori_err_deg": 0.0}
        self._last_render_error = ""
        self._traj = None            # [(t, [6 度], grip)]，与 sim 同步
        self._plan_meta = {"duration": 0.0, "reachable": True, "wp": []}

        self._log("info", "Arm 初始化：模式=%s 模型=%s 场景=%s" % (self.mode, self.model_xml, self.scene_xml))

    # ------------------------------------------------------------ 日志
    def _log(self, level, text):
        with self._log_lock:
            self._seq += 1
            item = {"seq": self._seq, "t": time.time(), "level": str(level), "text": str(text)}
            self._logs.append(item)
            return item

    def drain_logs(self, since=0):
        try:
            since = int(since)
        except (TypeError, ValueError):
            since = 0
        with self._log_lock:
            return [dict(x) for x in self._logs if x["seq"] > since]

    def logs(self, since=0):
        return self.drain_logs(since)

    # ------------------------------------------------------------ 装饰器
    @staticmethod
    def _safe(func):
        def wrapper(self, *args, **kwargs):
            try:
                return func(self, *args, **kwargs)
            except Exception as exc:  # 契约：绝不把异常抛给 HTTP 层
                msg = "%s 失败：%s" % (func.__name__, exc)
                try:
                    self._log("error", msg)
                except Exception:
                    pass
                return {"ok": False, "error": msg}
        wrapper.__name__ = func.__name__
        wrapper.__doc__ = func.__doc__
        return wrapper

    # ------------------------------------------------------------ 生命周期
    @property
    def sim(self):
        with self._lock:
            if self._sim is None:
                self._sim = SimDriver(scene_xml=self.scene_xml, model_xml=self.model_xml)
            return self._sim

    @property
    def ik(self):
        with self._lock:
            if self._ik is None:
                self._ik = ArmIK(self.model_xml)
            return self._ik

    @_safe
    def start(self):
        if self.mode != "sim":
            self._log("error", "真机模式尚未实现，只有 mode='sim'")
            return {"ok": False, "error": "真机模式尚未实现"}
        with self._lock:
            sim = self.sim
            sim.start()
            self._log("info", "仿真已启动：nq=%d nu=%d neq=%d" % (sim.model.nq, sim.model.nu, sim.model.neq))
        return {"ok": True}

    @_safe
    def close(self):
        with self._lock:
            sim = self._sim
            self._sim = None
        if sim is not None:
            sim.close()
        self._log("info", "仿真已关闭")
        return {"ok": True}

    @_safe
    def load(self, model_xml=None, scene_xml=None):
        if model_xml:
            self.model_xml = model_xml
        if scene_xml:
            self.scene_xml = scene_xml
        with self._lock:
            self._ik = None
            sim = self.sim
            sim.load(model_xml=self.model_xml, scene_xml=self.scene_xml)
            if not sim.running:
                sim.start()
            self._traj = None
            self._plan_meta = {"duration": 0.0, "reachable": True, "wp": []}
            self._ik_state = {"status": "ok", "text": IK_TEXT["ok"],
                              "pos_err_mm": 0.0, "ori_err_deg": 0.0}
            info = (sim.model.nq, sim.model.nu, sim.model.neq)
        self._log("info", "模型已重载：%s / %s（nq=%d nu=%d neq=%d）" % (self.model_xml, self.scene_xml, *info))
        return {"ok": True, "state": self.state()}

    # ------------------------------------------------------------ 状态
    @_safe
    def state(self):
        with self._lock:
            sim = self.sim
            st = sim.state()
            st["ik"] = dict(self._ik_state)
            st["mode"] = self.mode
            st["play"] = sim.playback()
            st["ts"] = time.time()
            # 渲染/步进出问题也不影响 state()：只记一条 warn，别刷屏
            err = st.get("render_error") or st.get("loop_error") or ""
            if err and err != self._last_render_error:
                self._log("warn", err)
            self._last_render_error = err
        return st

    # ------------------------------------------------------------ 运动
    @_safe
    def set_joints(self, degs):
        vals = np.asarray(degs, dtype=float).reshape(-1)
        if vals.size < 6:
            return {"ok": False, "error": "需要 6 个关节角"}
        clamped = [C.clamp_deg(i, vals[i]) for i in range(6)]
        if any(abs(clamped[i] - float(vals[i])) > 1e-6 for i in range(6)):
            self._log("warn", "关节目标超出软限位，已夹到边界：%s" % (clamped,))
        with self._lock:
            self.sim.set_joints(clamped)
        return {"ok": True, "state": self.state()}

    @_safe
    def set_gripper(self, deg):
        if not _is_num(deg):
            return {"ok": False, "error": "夹爪角度无效"}
        with self._lock:
            self.sim.set_gripper(float(deg))
        return {"ok": True, "state": self.state()}

    @_safe
    def estop(self):
        with self._lock:
            self.sim.estop()
            self._traj = None
        self._log("warn", "急停：目标 = 当前位置，播放已取消")
        return {"ok": True, "state": self.state()}

    @_safe
    def clear_estop(self):
        """解除急停锁定（不移动机械臂，不改目标）。"""
        with self._lock:
            self.sim.clear_estop()
        self._log("info", "急停已解除")
        return {"ok": True, "state": self.state()}

    @_safe
    def home(self):
        with self._lock:
            self.sim.home()
            self._traj = None
        self._log("info", "回默认位（OFFSET）")
        return {"ok": True, "state": self.state()}

    @_safe
    def goto_pose(self, frame, mm, degs):
        frame = (frame or "tcp").strip() or "tcp"
        if frame not in (C.TCP_SITE, C.FLANGE_SITE, "tcp", "flange"):
            return {"ok": False, "error": "frame 只能是 tcp 或 flange"}
        try:
            pos_m = np.asarray(mm, dtype=float).reshape(3) / 1000.0
            R = euler_xyz_deg_to_matrix(np.asarray(degs, dtype=float).reshape(3))
        except Exception:
            return {"ok": False, "error": "位姿数值无效"}

        with self._lock:
            sim = self.sim
            seed = np.zeros(sim.model.nq, dtype=float)
            seed[:6] = sim.arm_q()
        res = self.ik.solve_pose(frame, pos_m, R, q_seed=seed)
        info = {"status": res["status"], "text": res["text"],
                "pos_err_mm": round(float(res["pos_err_mm"]), 3),
                "ori_err_deg": round(float(res["ori_err_deg"]), 3)}
        with self._lock:
            self._ik_state = info
        if res["status"] != "ok":
            self._log("warn", "IK 失败（%s）：%s" % (info["status"], info["text"]))
            return {"ok": False, "ik": info, "error": info["text"]}

        degs_out = C.dh_to_degs(res["q"][:6])
        with self._lock:
            self.sim.set_joints(degs_out)
            self._traj = None
        self._log("info", "运动到 %s 位姿：误差 %.3f mm / %.3f°" % (frame, info["pos_err_mm"], info["ori_err_deg"]))
        return {"ok": True, "ik": info, "state": self.state()}

    # ------------------------------------------------------------ planner 桥
    def _planner(self):
        """返回 planner 模块；没有就返回 None。"""
        try:
            module = importlib.import_module("robot.planner")
        except Exception:
            return None
        return module

    @staticmethod
    def _call(fn, *args, **kwargs):
        """容错调用：先按关键字，再按位置，最后不带可选参数。"""
        try:
            return fn(*args, **kwargs)
        except TypeError:
            pass
        if kwargs:
            try:
                return fn(*args, *kwargs.values())
            except TypeError:
                pass
        try:
            return fn(*args)
        except TypeError:
            pass
        return fn(args[0])

    def _planner_fn(self, module, names):
        for name in names:
            fn = getattr(module, name, None)
            if callable(fn):
                return fn
        return None

    def _traj_from_plan(self, result):
        """把 planner 结果转成 [(t, 6 度, 夹爪度)]；能转就转，转不了返回 None。"""
        if not isinstance(result, dict):
            return None
        entries = result.get("traj") or result.get("wp") or result.get("waypoints") or []
        if not entries:
            return None
        seed_q = None
        with self._lock:
            sim = self.sim
            seed_q = np.zeros(sim.model.nq, dtype=float)
            seed_q[:6] = sim.arm_q()
        traj = []
        for item in entries:
            # planner 原生格式：[t, [6 个关节角], grip]
            if isinstance(item, (list, tuple)) and len(item) >= 2:
                try:
                    t = float(item[0])
                    degs = [float(v) for v in np.asarray(item[1], dtype=float).reshape(-1)[:6]]
                except Exception:
                    continue
                grip = item[2] if len(item) > 2 else 0.0
                if isinstance(grip, str):
                    grip = 0.0 if grip.lower() == "open" else C.GRIPPER_RANGE_DEG[1]
                grip = float(grip) if _is_num(grip) else 0.0
                if len(degs) >= 6:
                    traj.append((t, degs, grip))
                continue
            if not isinstance(item, dict):
                continue
            t = float(item.get("t", len(traj) * 0.5))
            grip = item.get("grip", item.get("gripper", item.get("open_close", 0.0)))
            if isinstance(grip, str):
                grip = 0.0 if grip.lower() == "open" else 110.0
            grip = float(grip) if _is_num(grip) else 0.0
            degs = None
            if item.get("degs") is not None:
                degs = [float(v) for v in np.asarray(item["degs"], dtype=float).reshape(-1)[:6]]
            elif item.get("q") is not None and len(np.asarray(item["q"]).reshape(-1)) >= 6:
                degs = C.dh_to_degs(np.asarray(item["q"], dtype=float).reshape(-1)[:6])
            elif item.get("pos") is not None:
                pos = np.asarray(item["pos"], dtype=float).reshape(-1)
                scale = 0.001 if (pos.size >= 3 and np.max(np.abs(pos)) > 10.0) else 1.0
                R = self.ik.tool_down_rotation(item.get("yaw", 0.0) or 0.0)
                res = self.ik.solve_pose("tcp", pos[:3] * scale, R, q_seed=seed_q, iters=40)
                if res["status"] != "ok":
                    return None
                seed_q = res["q"]
                degs = C.dh_to_degs(res["q"][:6])
            if degs is None or len(degs) < 6:
                continue
            traj.append((t, degs, grip))
        return traj or None

    @_safe
    def plan(self, waypoints, speed=1.0):
        if self.mode != "sim":
            return {"ok": False, "error": "真机模式尚未实现"}
        module = self._planner()
        if module is None:
            self._log("error", PLAYER_MISSING)
            return {"ok": False, "error": PLAYER_MISSING}
        fn = self._planner_fn(module, ("plan", "plan_waypoints", "build_plan"))
        if fn is None:
            self._log("error", PLAYER_MISSING)
            return {"ok": False, "error": PLAYER_MISSING}
        try:
            result = self._call(fn, waypoints, speed=float(speed))
        except Exception as exc:
            self._log("error", "规划失败：%s" % exc)
            return {"ok": False, "error": "规划失败：%s" % exc}
        if not isinstance(result, dict):
            result = {"ok": True, "duration": 0.0, "reachable": True, "wp": []}

        duration = float(result.get("duration", 0.0) or 0.0)
        reachable = bool(result.get("reachable", True))
        wp = result.get("wp") or []
        err = result.get("err") or result.get("error")

        traj = self._traj_from_plan(result)
        if traj is not None:
            if not duration:
                duration = float(traj[-1][0])
            with self._lock:
                self._traj = traj
                self._plan_meta = {"duration": duration, "reachable": reachable, "wp": wp}
                self.sim.set_trajectory(traj)
        if not err and reachable:
            # 规划成功意味着当前位姿是可达的：把 IK 状态复位成 ok
            with self._lock:
                self._ik_state = {"status": "ok", "text": "可达",
                                  "pos_err_mm": 0.0, "ori_err_deg": 0.0}
        if err:
            self._log("warn", "规划提示：%s" % err)
        else:
            self._log("info", "规划完成：时长 %.2fs，%d 个路点，%s" %
                      (duration, len(wp), "可达" if reachable else "不可达"))
        return {"ok": bool(result.get("ok", True)), "duration": round(duration, 3),
                "reachable": reachable, "wp": wp, "err": err}

    @_safe
    def play(self, action="play", speed=1.0, loop=False):
        if self.mode != "sim":
            return {"ok": False, "error": "真机模式尚未实现"}
        module = self._planner()
        if module is None:
            self._log("error", PLAYER_MISSING)
            return {"ok": False, "error": PLAYER_MISSING}
        fn = self._planner_fn(module, ("play",))
        if fn is not None:
            try:
                result = self._call(fn, action, speed=float(speed), loop=bool(loop))
                if isinstance(result, dict):
                    return dict(result, ok=bool(result.get("ok", True)), state=self.state())
            except Exception:
                pass
        with self._lock:
            if self._traj is None:
                if not getattr(module, "is_ready", lambda: True)():
                    return {"ok": False, "error": PLAYER_MISSING}
                return {"ok": False, "error": "还没有规划好的轨迹"}
            ok = self.sim.play(action, speed=float(speed), loop=bool(loop))
            state = self.state()
        if not ok:
            return {"ok": False, "error": "未知的播放动作：%s" % action}
        self._log("info", "播放 %s（speed=%.2f loop=%s）" % (action, float(speed), bool(loop)))
        return {"ok": True, "state": state}

    @_safe
    def seek(self, t):
        if self.mode != "sim":
            return {"ok": False, "error": "真机模式尚未实现"}
        module = self._planner()
        if module is None:
            self._log("error", PLAYER_MISSING)
            return {"ok": False, "error": PLAYER_MISSING}
        fn = self._planner_fn(module, ("seek",))
        if fn is not None:
            try:
                result = self._call(fn, float(t))
                if isinstance(result, dict):
                    return dict(result, ok=bool(result.get("ok", True)), state=self.state())
            except Exception:
                pass
        with self._lock:
            if self._traj is None:
                return {"ok": False, "error": "还没有规划好的轨迹"}
            ok = self.sim.seek(float(t))
            state = self.state()
        if not ok:
            return {"ok": False, "error": "还没有规划好的轨迹"}
        return {"ok": True, "state": state}

    @_safe
    def generate(self, name, waypoints):
        if self.mode != "sim":
            return {"ok": False, "error": "真机模式尚未实现"}
        module = self._planner()
        if module is None:
            self._log("error", PLAYER_MISSING)
            return {"ok": False, "error": PLAYER_MISSING}
        fn = self._planner_fn(module, ("generate", "generate_script", "write_script"))
        if fn is None:
            self._log("error", PLAYER_MISSING)
            return {"ok": False, "error": PLAYER_MISSING}
        try:
            result = self._call(fn, name, waypoints)
        except Exception as exc:
            self._log("error", "生成脚本失败：%s" % exc)
            return {"ok": False, "error": "生成脚本失败：%s" % exc}
        if not isinstance(result, dict):
            result = {"ok": True, "path": str(result)}
        if result.get("ok", True):
            self._log("info", "已生成脚本：%s" % result.get("path"))
        else:
            self._log("error", "生成脚本失败：%s" % result.get("error"))
        return result

    @_safe
    def set_trajectory(self, traj):
        """给 planner/服务端直接塞关节轨迹：[(t, 6 度, 夹爪度)]。"""
        with self._lock:
            self._traj = [(float(t), [float(v) for v in d], float(g)) for t, d, g in traj]
            self.sim.set_trajectory(self._traj)
        return {"ok": True}

    # ------------------------------------------------------------ 渲染 / 相机
    @_safe
    def render_image(self, w=None, h=None):
        with self._lock:
            if w or h:
                sim = self.sim
                sim.resize(w or sim.width, h or sim.height)
            return self.sim.render_image()

    @_safe
    def render_jpeg(self, w=None, h=None):
        """契约 §7 的名字；实际返回 PNG（不装 pillow，浏览器/前端都认）。"""
        return self.render_image(w, h)

    @_safe
    def set_cam(self, az=None, el=None, dist=None, lookat=None):
        with self._lock:
            cam = self.sim.set_cam(az=az, el=el, dist=dist, lookat=lookat)
        return {"ok": True, "cam": cam}

    @_safe
    def reset_cam(self):
        with self._lock:
            cam = self.sim.reset_cam()
        return {"ok": True, "cam": cam}

    # ------------------------------------------------------------ 自检
    @_safe
    def selftest(self):
        checks = []

        def add(name, ok, detail=""):
            checks.append({"name": name, "ok": bool(ok), "detail": str(detail)})

        with self._lock:
            sim = self.sim
            add("模型加载", sim.model is not None,
                "nq=%d nu=%d neq=%d" % (sim.model.nq, sim.model.nu, sim.model.neq))
            add("步进线程", sim.running, "running=%s fps=%.1f" % (sim.running, sim.fps))
            st = sim.state()
        add("引擎", st["engine"]["name"] == "MuJoCo", "MuJoCo %s" % st["engine"]["version"])
        add("关节数", len(st["joints"]["deg"]) == 6, "deg=%s" % (st["joints"]["deg"],))

        try:
            p_tcp, R_tcp = self.ik.fk(C.TCP_SITE)
            p_fl, _ = self.ik.fk(C.FLANGE_SITE)
            add("FK", True, "tcp=(%.3f, %.3f, %.3f) m" % tuple(p_tcp))
        except Exception as exc:
            add("FK", False, str(exc))

        try:
            res = self.ik.solve_pose(C.TCP_SITE, np.array([0.30, 0.08, 0.14]),
                                     self.ik.tool_down_rotation(0.0), q_seed=self.ik.home_q)
            add("IK", res["status"] == "ok",
                "%s 误差 %.3f mm / %.3f°" % (res["text"], res["pos_err_mm"], res["ori_err_deg"]))
        except Exception as exc:
            add("IK", False, str(exc))

        add("自干涉检测", not self.ik.self_collision(self.ik.home_q), "home 无干涉")

        try:
            png = self.render_image()
            add("渲染", len(png) > 1000, "%d 字节 PNG" % len(png))
        except Exception as exc:
            add("渲染", False, str(exc))

        ok = all(c["ok"] for c in checks)
        self._log("done" if ok else "error", "自检%s" % ("通过" if ok else "未通过"))
        return {"ok": ok, "checks": checks}


__all__ = ["Arm"]
