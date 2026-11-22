#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pick_and_place —— 路点脚本

由 robot/scriptgen.py 生成，可独立运行（不需要启动 Web 服务）：

    python scripts/pick_and_place.py                 在 MuJoCo 里规划并执行一遍
    python scripts/pick_and_place.py --gui           带查看器窗口
    python scripts/pick_and_place.py --csv out.csv   顺便导出 10 ms 轨迹

路点格式：[x, y, z, dwell, grip]，位置单位米，dwell 秒，grip 取 "open"/"close"/None。
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time

# ---- 让脚本在任意目录下都能 import robot.*：逐级往上找包含 robot/ 的目录
_HERE = os.path.dirname(os.path.abspath(__file__))
_root = _HERE
for _ in range(6):
    if os.path.isdir(os.path.join(_root, "robot")):
        break
    _root = os.path.dirname(_root)
if _root not in sys.path:
    sys.path.insert(0, _root)

import robot.convention as C        # noqa: E402
import robot.planner as planner     # noqa: E402
from robot.arm import Arm           # noqa: E402

# ---------------------------------------------------------------- 路点常量
WAYPOINTS = [
    [0.3000, 0.0800, 0.1500, 0.500, 'open'],  # 第 0 点：(0.300, 0.080, 0.150) m，停 0.50 s，张开夹爪
    [0.3000, 0.0800, 0.0620, 0.300, 'close'],  # 第 1 点：(0.300, 0.080, 0.062) m，停 0.30 s，夹紧夹爪
    [0.3400, 0.2800, 0.1400, 0.800, 'open'],  # 第 2 点：(0.340, 0.280, 0.140) m，停 0.80 s，张开夹爪
]

SPEED = 1.00          # 1.0 = 巡航 0.25 m/s；2.0 翻倍，0.5 减半


def write_csv(path: str, points) -> int:
    """把规划出的 10 ms 采样点写成 CSV。"""
    folder = os.path.dirname(os.path.abspath(path))
    if folder:
        os.makedirs(folder, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fp:
        w = csv.writer(fp)
        w.writerow(["t", "j1", "j2", "j3", "j4", "j5", "j6", "gripper_deg", "wp"])
        for p in points:
            degs = [round(float(v), 4) for v in p["deg"]]
            w.writerow([round(float(p["t"]), 4)] + degs + [p["grip"], p["wp"]])
    return len(points)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='pick_and_place —— 路点脚本')
    ap.add_argument("--gui", action="store_true", help="打开 MuJoCo 查看器")
    ap.add_argument("--csv", default=None, help="把轨迹导出成 CSV（路径）")
    ap.add_argument("--speed", type=float, default=SPEED, help="速度倍率")
    ap.add_argument("--dry-run", action="store_true", help="只规划、不执行")
    args = ap.parse_args(argv)

    print("规划 %d 个路点（speed=%.2f）..." % (len(WAYPOINTS), args.speed), flush=True)
    plan = planner.plan_waypoints(WAYPOINTS, speed=args.speed)
    print("  时长 %.3f s，进度 %d 点，最大关节速度 %.1f °/s，%s"
          % (plan["duration"], len(plan["waypoints"]), plan["max_joint_speed_deg"],
             "可达" if plan["reachable"] else "不可达"), flush=True)
    for w in plan["waypoints"]:
        print("  路点 %d @ %.2f s  误差 %.3f mm  %s"
              % (w["wp"], w["t"], w["pos_err_mm"], w["status"]), flush=True)
    if not plan["reachable"]:
        print("有路点不可达，不执行：%s" % (plan.get("err") or ""), flush=True)
        return 2

    if args.csv:
        n = write_csv(args.csv, plan["points"])
        print("已导出 %d 个采样点到 %s" % (n, args.csv), flush=True)

    if args.dry_run:
        print("--dry-run：只规划不执行", flush=True)
        return 0

    arm = Arm()
    started = arm.start()
    if not started.get("ok"):
        print("仿真启动失败：%s" % started.get("error"), flush=True)
        return 3
    viewer = None
    try:
        arm.set_trajectory(plan["traj"])
        arm.play("play", speed=1.0)
        if args.gui:
            try:
                import mujoco.viewer
                viewer = mujoco.viewer.launch_passive(arm.sim.model, arm.sim.data)
            except Exception as exc:      # 没显示器 / 无 GL 时退回 headless
                print("查看器打不开（%s），继续 headless" % exc, flush=True)
                viewer = None
        t0 = time.time()
        while True:
            pb = arm.sim.playback()
            if pb["t"] >= pb["duration"] - 1e-6:
                break
            if time.time() - t0 > pb["duration"] + 30.0:
                print("播放超时", flush=True)
                break
            if viewer is not None:
                if not viewer.is_running():
                    break
                viewer.sync()
            time.sleep(1.0 / 60.0)
        st = arm.state()
        print("执行完成：TCP=(%.1f, %.1f, %.1f) mm，夹爪 %.1f°"
              % tuple(st["tcp"]["mm"] + [st["gripper"]["deg"]]), flush=True)
    finally:
        if viewer is not None:
            try:
                viewer.close()
            except Exception:
                pass
        arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
