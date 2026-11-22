#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""路点 → 独立可跑脚本（scripts/<name>.py）的生成器。

生成出来的脚本特点：
* 路点写成模块常量，每个路点带中文注释（坐标、停留、夹爪角度）。
* 自己把仓库根加进 sys.path，所以在任何目录下都能直接
  python scripts/v2/demo_pick.py 跑。
* 用 robot.planner 规划、用 robot.arm 的 Arm 门面（SimDriver）在 MuJoCo 里执行，
  和上位机 Web 端走的是同一条代码路径。
* 支持 --gui（MuJoCo 查看器）和 --csv（导出 10 ms 轨迹）。

生成器自身也补上了 Arm.generate 需要的宽松接口：
    generate(name, waypoints, speed=1.0, ...) -> {"ok": bool, "path": 相对路径, ...}
不抛异常：出问题返回 {"ok": False, "error": 中文原因}。
"""

from __future__ import annotations

import os
import re

import robot.convention as C
import robot.planner as planner

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(ROOT, "scripts")
NAME_RE = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_./-]*$")


def default_notes(waypoints) -> list[str]:
    """没有给注释时，按路点内容自动写一句中文说明。"""
    notes = []
    for i, w in enumerate(waypoints):
        x, y, z = (float(v) for v in list(w)[:3])
        dwell = float(w[3]) if len(w) > 3 and w[3] is not None else 0.0
        grip = w[4] if len(w) > 4 else None
        if isinstance(grip, str):
            action = {"open": "张开夹爪（0°）", "close": "夹紧夹爪（110°）"}.get(
                grip.strip().lower(), "夹爪保持")
        elif grip is None:
            action = "夹爪保持"
        else:
            action = "夹爪开到 %.1f°" % float(grip)
        notes.append("第 %d 点：(%.3f, %.3f, %.3f) m%s，%s" % (
            i, x, y, z, "，停 %.2f s" % dwell if dwell > 0 else "", action))
    return notes


def _fmt_waypoints(waypoints, notes) -> str:
    lines = ["WAYPOINTS = ["]
    for i, w in enumerate(waypoints):
        w = list(w)
        while len(w) < 5:
            w.append(None)
        x, y, z, dwell, grip = w[0], w[1], w[2], w[3], w[4]
        dwell_s = "None" if dwell is None else ("%.3f" % float(dwell))
        if grip is None:
            grip_s = "None"
        elif isinstance(grip, str):
            grip_s = repr(str(grip))
        else:
            grip_s = "%.1f" % float(grip)
        note = notes[i] if i < len(notes) else ""
        lines.append("    [%.4f, %.4f, %.4f, %s, %s],  # %s"
                     % (float(x), float(y), float(z), dwell_s, grip_s, note))
    lines.append("]")
    return "\n".join(lines)


TEMPLATE = '''#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""@@TITLE@@

由 robot/scriptgen.py 生成，可独立运行（不需要启动 Web 服务）：

    python scripts/@@REL@@                 在 MuJoCo 里规划并执行一遍
    python scripts/@@REL@@ --gui           带查看器窗口
    python scripts/@@REL@@ --csv out.csv   顺便导出 10 ms 轨迹

路点格式：[x, y, z, dwell, grip]，位置单位米，dwell 秒，
grip 是夹爪开合角（数字，0–110°；旧写法 "open"/"close" 也仍兼容 = 0/110）。
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
@@WAYPOINTS@@

SPEED = @@SPEED@@          # 1.0 = 巡航 0.25 m/s；2.0 翻倍，0.5 减半


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
    ap = argparse.ArgumentParser(description=@@TITLE_SHORT@@)
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
'''


def _safe_rel_path(name: str) -> str:
    """把 name 规整成 scripts/ 下的相对路径，拒绝越界。"""
    name = str(name or "").strip().replace("\\", "/")
    if not name:
        raise ValueError("脚本名不能为空")
    if not NAME_RE.match(name):
        raise ValueError("脚本名只能包含字母/数字/_/-/./，收到 %r" % name)
    if not name.endswith(".py"):
        name += ".py"
    rel = os.path.normpath(name).replace(os.sep, "/")
    if rel.startswith("../") or rel.startswith("/") or ".." in rel.split("/"):
        raise ValueError("脚本名不能跑出 scripts/ 目录：%r" % name)
    return rel


def generate(name: str, waypoints, speed: float = 1.0, title: str | None = None,
             notes=None, out_dir: str | None = None, overwrite: bool = True) -> dict:
    """生成独立脚本，返回 {"ok", "path", "abs_path", "waypoints", "speed"}。

    name 可以是 "demo_pick" 或 "v2/demo_pick"（落到 scripts/v2/demo_pick.py）。
    """
    try:
        rel = _safe_rel_path(name)
        wps = planner.parse_waypoints(waypoints)
        spd = float(speed if speed else 1.0)
        if spd <= 0:
            raise ValueError("speed 必须为正数")
        base = os.path.join(out_dir or SCRIPTS_DIR, rel)
        if os.path.exists(base) and not overwrite:
            raise ValueError("脚本已存在：%s" % rel)

        title_text = title or ("%s —— 路点脚本" % os.path.splitext(os.path.basename(rel))[0])
        if notes is None:
            notes = default_notes([list(w) for w in waypoints])
        body = (TEMPLATE
                .replace("@@TITLE_SHORT@@", repr(title_text.splitlines()[0] if title_text else "路点脚本"))
                .replace("@@TITLE@@", title_text)
                .replace("@@REL@@", rel)
                .replace("@@WAYPOINTS@@", _fmt_waypoints([list(w) for w in waypoints], notes))
                .replace("@@SPEED@@", "%.2f" % spd))
        os.makedirs(os.path.dirname(base) or SCRIPTS_DIR, exist_ok=True)
        with open(base, "w", encoding="utf-8") as fp:
            fp.write(body)
        os.chmod(base, 0o755)
        print("[scriptgen] 已生成 scripts/%s（%d 个路点）" % (rel, len(wps)), flush=True)
        return {"ok": True, "path": "scripts/" + rel, "abs_path": base,
                "waypoints": len(wps), "speed": spd}
    except Exception as exc:
        print("[scriptgen] 生成失败：%s" % exc, flush=True)
        return {"ok": False, "error": "生成脚本失败：%s" % exc}


def generate_script(name: str, waypoints, speed: float = 1.0, **kw) -> dict:
    """Arm.generate 的另一个候选名。"""
    return generate(name, waypoints, speed=speed, **kw)


write_script = generate_script


def main(argv=None) -> int:
    import argparse
    import json

    ap = argparse.ArgumentParser(description="生成独立可跑的路点脚本")
    ap.add_argument("name", help="脚本名，如 demo_pick 或 v2/demo_pick")
    ap.add_argument("--waypoints",
                    default="[[0.30,0.08,0.15,0.6,'open'],[0.30,0.08,0.062,0.5,'close'],[0.34,0.28,0.14,0.0,None]]",
                    help="路点 JSON：[[x,y,z,dwell,grip],...]")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--title", default=None)
    args = ap.parse_args(argv)
    text = args.waypoints.strip()
    try:
        wps = json.loads(text)
    except Exception:
        import ast
        wps = ast.literal_eval(text)   # 允许 Python 字面量写法（单引号）
    res = generate(args.name, wps, speed=args.speed, title=args.title)
    print(json.dumps({"ok": res.get("ok"), "path": res.get("path"), "error": res.get("error")},
                     ensure_ascii=False))
    return 0 if res.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
