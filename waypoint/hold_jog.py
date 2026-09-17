#!/usr/bin/env python3
"""ALT_HOLD 友好的水平点动 — 不 arm / 不换模式 / 不 disarm。

给负浮力机器人用:假设你已在 Cockpit/QGC 解锁并设 ALT_HOLD(飞控定深),
本脚本只叠加水平 MANUAL_CONTROL(surge x / sway y / yaw r),z 恒中位(不干预深度),
结束时只回中位、**不 disarm**(避免掉出 ALT_HOLD 沉底)。

用于:W1 vx 符号验证等——发一个前进点动,配合 dvl_dashboard 读 vx。

⚠ 前提:机器人已 armed + ALT_HOLD;跑本脚本时请暂停 Cockpit 手柄输入(避免两路抢)。
   结束后 pilot 失效计时(建议已设 30s)内用 Cockpit 接管。

用法:
  python -m waypoint.hold_jog --x 0.4 --seconds 5 --umax 0.8
"""
from __future__ import annotations

import argparse
import sys
import time

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

from src.pseudo_stick import PseudoStick, load_config, MAV_MODE_FLAG_SAFETY_ARMED  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="ALT_HOLD 友好水平点动 (不 arm/不换模式/不 disarm)")
    ap.add_argument("--x", type=float, default=0.0, help="前进 surge")
    ap.add_argument("--y", type=float, default=0.0, help="右移 sway")
    ap.add_argument("--r", type=float, default=0.0, help="偏航 yaw")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--umax", type=float, default=0.8, help="覆盖 U_MAX 顶过 ESC 死区")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--endpoint", default=None)
    args = ap.parse_args(argv)

    cfg = load_config()
    stick = PseudoStick(cfg, endpoint=args.endpoint)
    stick.u_max = float(args.umax)
    try:
        if not stick.wait_heartbeat(float(cfg["connection"].get("heartbeat_timeout_s", 10))):
            return 1
        armed = stick.is_armed(timeout=2.0)
        print(f"[jog] U_MAX={stick.u_max}  当前 armed={armed}  "
              f"(本脚本不 arm/不换模式/不 disarm; z 恒中位交给 ALT_HOLD 定深)")
        if armed is False:
            print("[jog] ⚠ 机器人未解锁 → 指令会被忽略、不会动。请先在 Cockpit 解锁+ALT_HOLD。")
            return 2
        print(f"[jog] 发送 x={args.x} y={args.y} r={args.r} 共 {args.seconds}s ...")
        dt = 1.0 / args.hz
        n = max(1, int(args.seconds * args.hz))
        for i in range(n):
            stick.send(x=args.x, y=args.y, z=0.0, r=args.r)   # z=0 → 中位, 不干预深度
            if i % max(1, int(args.hz)) == 0:
                stick.send_gcs_heartbeat()
            time.sleep(dt)
        # 回中位几帧, 但不 disarm
        for _ in range(5):
            stick.send(x=0.0, y=0.0, z=0.0, r=0.0)
            time.sleep(0.02)
        print("[jog] 点动结束, 已回中位。**未 disarm** — 机器人仍 armed+ALT_HOLD。"
              "请用 Cockpit 接管定深。")
        return 0
    except KeyboardInterrupt:
        for _ in range(5):
            try:
                stick.send(0, 0, 0, 0)
            except Exception:
                pass
            time.sleep(0.02)
        print("\n[jog] 中断, 已回中位 (未 disarm)。")
        return 130
    finally:
        # 只关连接, 不调用 close() 的自动 disarm 逻辑 (armed_by_us=False 本就不会 disarm,
        # 但这里显式只 close socket, 更保险)
        try:
            stick.conn.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
