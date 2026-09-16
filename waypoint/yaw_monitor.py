#!/usr/bin/env python3
"""只读航向监视 — 实时打印 ATTITUDE 的 yaw/roll/pitch (不解锁, 不动桨)。

干测用: 用手转动 ROV, 核对 yaw 是否跟随、方向、wrap。也报观测到的 yaw 范围。

用法:
  python -m waypoint.yaw_monitor              # 默认 30s
  python -m waypoint.yaw_monitor --seconds 60
"""
from __future__ import annotations

import argparse
import math
import sys
import time

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="只读航向监视 (ATTITUDE yaw)")
    ap.add_argument("--endpoint", default="udpin:0.0.0.0:14550")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--hz", type=float, default=2.0, help="打印频率")
    args = ap.parse_args(argv)

    from pymavlink import mavutil
    c = mavutil.mavlink_connection(args.endpoint, dialect="ardupilotmega")
    print("[yaw] 等待 heartbeat ...")
    c.wait_heartbeat()
    mid = mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE
    c.mav.command_long_send(c.target_system, c.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            float(mid), 50000, 0, 0, 0, 0, 0)
    print(f"[yaw] 现在开始转动 ROV ({args.seconds:.0f}s) ...  (Ctrl+C 提前结束)")
    t_end = time.monotonic() + args.seconds
    last = 0.0
    ymin, ymax = 999.0, -999.0
    n = 0
    try:
        while time.monotonic() < t_end:
            m = c.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
            if not m:
                continue
            n += 1
            y = math.degrees(m.yaw); r = math.degrees(m.roll); p = math.degrees(m.pitch)
            ymin, ymax = min(ymin, y), max(ymax, y)
            now = time.monotonic()
            if now - last >= 1.0 / args.hz:
                last = now
                print(f"  yaw={y:+7.1f}  roll={r:+6.1f}  pitch={p:+6.1f}   (帧{n})")
    except KeyboardInterrupt:
        print("\n[yaw] 中断。")
    if n == 0:
        print("[yaw] ❌ 未收到 ATTITUDE 帧。")
        return 1
    print(f"[yaw] 结束。收到 {n} 帧, yaw 观测范围 = [{ymin:+.1f}, {ymax:+.1f}]° "
          f"(跨度 {ymax-ymin:.0f}°)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
