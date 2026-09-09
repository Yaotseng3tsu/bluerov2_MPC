#!/usr/bin/env python3
"""离线 MAVLink 模拟器 —— 假 ArduSub,用于在无真机时验证 src/link.py。

它向 udpout 目标 (默认 127.0.0.1:14550) 发送:
  - HEARTBEAT  (1 Hz, type=SUBMARINE, autopilot=ARDUPILOTMEGA, DISARMED, MANUAL 模式)
  - GLOBAL_POSITION_INT (10 Hz),深度按正弦在 0~1 m 之间变化 (relative_alt 水下为负)

用法 (两个终端):
  终端A:  python tests/sim_vehicle.py
  终端B:  python -m src.link --check --seconds 12

说明:link.py 用 udpin:0.0.0.0:14550 监听;本模拟器用 udpout 主动推流。
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

from pymavlink import mavutil


def build_base_mode(mavutil) -> int:
    # 使用 custom_mode + DISARMED (不含 SAFETY_ARMED 位)
    return mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED


def main() -> int:
    p = argparse.ArgumentParser(description="假 ArduSub MAVLink 模拟器")
    p.add_argument("--target", default="udpout:127.0.0.1:14550",
                   help="发送目标 (link.py 监听的地址)")
    p.add_argument("--seconds", type=float, default=0.0, help="运行时长,0=直到 Ctrl+C")
    p.add_argument("--period", type=float, default=8.0, help="深度正弦周期 (s)")
    p.add_argument("--depth-amp", type=float, default=0.5, help="深度振幅 (m),中心=振幅")
    args = p.parse_args()

    mav = mavutil.mavlink_connection(
        args.target, source_system=1, source_component=1, dialect="ardupilotmega",
    )
    # ArduSub MANUAL 模式号
    manual_mode = mavutil.mode_mapping_sub.get("MANUAL", 19)
    base_mode = build_base_mode(mavutil)

    print(f"[sim] 向 {args.target} 推流 (Ctrl+C 停止)。DISARMED, MANUAL, 深度 0~{2*args.depth_amp:.1f}m")

    t0 = time.monotonic()
    last_hb = 0.0
    hz = 10.0
    dt = 1.0 / hz
    while True:
        now = time.monotonic()
        el = now - t0
        if args.seconds > 0 and el >= args.seconds:
            break

        # 1 Hz heartbeat
        if now - last_hb >= 1.0:
            last_hb = now
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_SUBMARINE,
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode, manual_mode,
                mavutil.mavlink.MAV_STATE_STANDBY,
            )

        # 10 Hz GLOBAL_POSITION_INT:深度正弦 (0 ~ 2*amp),relative_alt 水下为负(mm)
        depth_m = args.depth_amp * (1 - math.cos(2 * math.pi * el / args.period))
        rel_alt_mm = int(-depth_m * 1000)
        mav.mav.global_position_int_send(
            int(el * 1000),      # time_boot_ms
            356800000, 1396000000,  # lat, lon (占位)
            0, rel_alt_mm,       # alt(mm), relative_alt(mm)
            0, 0, 0,             # vx, vy, vz
            0,                   # hdg
        )

        time.sleep(dt)

    print("[sim] 结束。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
