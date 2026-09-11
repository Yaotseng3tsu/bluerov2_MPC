#!/usr/bin/env python3
"""Phase 1 — 引导式方向/中位标定 (真机,出水台架用)。

逐个自由度发一个小脉冲 (默认 u=+0.2, 0.8s),由你现场观察推进器/艇体运动方向,
回答"是否与期望一致",脚本据此把 sign_x/y/z/r 写回 config/vehicle.yaml。
用于确定 MANUAL_CONTROL 各轴正负号(仿真无法替代的真机步骤)。

期望的正方向:
  x = +前进(surge forward)   y = +右移(sway right)
  z = +下潜(heave down)      r = +右转(yaw clockwise)

安全:
  - 出水/台架、远离桨叶后再运行。
  - 需要解锁(推进器会转);脚本退出/中断自动上锁。
  - 每个脉冲后自动回中位;可随时 Ctrl+C。

用法:
  python -m src.calibrate                 # 全部四轴
  python -m src.calibrate --axes z        # 只标定 z
  python -m src.calibrate --pulse 0.15 --dur 0.6
"""
from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

from src.pseudo_stick import CONFIG_PATH, PseudoStick, load_config  # noqa: E402

AXIS_DESC = {
    "x": "前进 surge(+ 应向前)",
    "y": "右移 sway(+ 应向右)",
    "z": "下潜 heave(+ 应下潜)",
    "r": "右转 yaw(+ 应顺时针/向右转)",
}


def update_config_signs(path: Path, signs: dict, z_neutral: int | None = None) -> None:
    """行级替换 manual_control 下的 sign_* / z_neutral,保留注释。"""
    text = path.read_text(encoding="utf-8")
    for axis, val in signs.items():
        text = re.sub(rf"(sign_{axis}:\s*)-?\d+", rf"\g<1>{val}", text, count=1)
    if z_neutral is not None:
        text = re.sub(r"(z_neutral:\s*)\d+", rf"\g<1>{z_neutral}", text, count=1)
    path.write_text(text, encoding="utf-8")


def ask(prompt: str) -> str:
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return "s"


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="P1 方向/中位标定")
    p.add_argument("--endpoint", default=None)
    p.add_argument("--axes", default="xyzr", help="要标定的轴,如 z 或 xyzr")
    p.add_argument("--pulse", type=float, default=0.2, help="脉冲归一化幅值")
    p.add_argument("--dur", type=float, default=0.8, help="脉冲时长 (s)")
    p.add_argument("--umax", type=float, default=None,
                   help="临时覆盖 U_MAX(干测顶过 ESC 死区,如 0.6)")
    p.add_argument("--force-arm", action="store_true")
    p.add_argument("--yes", action="store_true", help="跳过解锁确认(慎用)")
    args = p.parse_args(argv)

    cfg = load_config()
    hz = float(cfg.get("control", {}).get("CTRL_HZ", 10))
    stick = PseudoStick(cfg, endpoint=args.endpoint)
    if args.umax is not None:
        stick.u_max = args.umax
        print(f"[calibrate] 临时 U_MAX = {args.umax}")
    signs: dict[str, int] = {}
    try:
        if not stick.wait_heartbeat(float(cfg["connection"].get("heartbeat_timeout_s", 10))):
            return 1
        print("\n⚠ 安全确认:ROV 是否已【出水/固定、人员远离桨叶】?")
        if not args.yes and ask("   确认安全请输入 yes: ") != "yes":
            print("已取消。")
            return 0
        stick.set_mode("MANUAL")
        time.sleep(0.3)
        if not stick.arm(force=args.force_arm):
            return 2
        # 后台持续发中位+心跳,保证等你按回车/观察时飞控不因失联 disarm
        stick.start_keepalive(hz)

        for axis in args.axes:
            if axis not in AXIS_DESC:
                continue
            print(f"\n=== 标定 {axis} 轴:{AXIS_DESC[axis]} ===")
            ans = ask(f"回车发 {axis}=+{args.pulse} 脉冲 {args.dur}s(观察方向),或输入 s 跳过: ")
            if ans == "s":
                print("  跳过。")
                continue
            stick.set_cmd(**{axis: args.pulse})   # keepalive 线程持续发脉冲
            time.sleep(args.dur)
            stick.set_cmd()                        # 回中位
            obs = ask("  观察到的运动方向与【期望正方向】一致吗? y=一致 / n=反了 / s=跳过: ")
            if obs == "y":
                signs[axis] = 1
                print(f"  记录 sign_{axis} = +1")
            elif obs == "n":
                signs[axis] = -1
                print(f"  记录 sign_{axis} = -1(反向)")
            else:
                print("  跳过记录。")

        if signs:
            update_config_signs(CONFIG_PATH, signs)
            print(f"\n✅ 已写入 config/vehicle.yaml: {signs}")
        else:
            print("\n未记录任何符号。")
        return 0
    except KeyboardInterrupt:
        print("\n[calibrate] 用户中断 → 回中位并上锁")
        return 130
    finally:
        stick.close()
        print("[calibrate] 已安全退出(中位 + 自动上锁)")


if __name__ == "__main__":
    raise SystemExit(main())
