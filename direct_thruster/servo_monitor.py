#!/usr/bin/env python3
"""只读输出监视 — 被动读 SERVO_OUTPUT_RAW 1-8 + 解锁状态(不解锁, 不发任何控制信号)。

用于 direct_thruster 的 M2 装机验证: 刷机**前后**各跑一遍做对比。
判据: 未解锁时 8 路应全为中位 1500 —— 证明固件在驱动输出通道且处于安全中位。

本脚本**只接收**, 除了请求遥测流之外不发送任何控制指令, 也不会解锁。

用法:
  .venv\\Scripts\\python direct_thruster\\servo_monitor.py
  .venv\\Scripts\\python direct_thruster\\servo_monitor.py --seconds 15
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

from pymavlink import mavutil

ENDPOINT = "udpin:0.0.0.0:14550"
MAV_MODE_FLAG_SAFETY_ARMED = 128


def main() -> int:
    ap = argparse.ArgumentParser(description="只读 SERVO_OUTPUT_RAW 监视 (不解锁/不发控制)")
    ap.add_argument("--endpoint", default=ENDPOINT)
    ap.add_argument("--seconds", type=float, default=12.0)
    args = ap.parse_args()

    from fc_link import connect_fc
    conn = connect_fc(args.endpoint)
    if conn is None:
        return 2

    # 只请求遥测流, 不发任何控制指令
    sid = mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                               float(sid), 2e5, 0, 0, 0, 0, 0)  # 5 Hz

    armed = None
    n = 0
    seen = {i: set() for i in range(1, 9)}
    last_print = 0.0
    t_end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < t_end:
            m = conn.recv_match(type=["SERVO_OUTPUT_RAW", "HEARTBEAT"],
                                blocking=True, timeout=1.0)
            if m is None:
                continue
            if m.get_type() == "HEARTBEAT":
                if m.get_srcSystem() == conn.target_system:
                    armed = bool(m.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
                continue
            n += 1
            vals = [getattr(m, f"servo{i}_raw") for i in range(1, 9)]
            for i, v in enumerate(vals, start=1):
                seen[i].add(v)
            now = time.monotonic()
            if now - last_print >= 1.0:
                last_print = now
                state = "ARMED" if armed else ("DISARMED" if armed is not None else "?")
                print(f"  [{state:8s}] SERVO1-8 = {vals}")
    except KeyboardInterrupt:
        print("\n[servo] 中断。")

    print("-" * 60)
    if n == 0:
        print("[FAIL] 未收到 SERVO_OUTPUT_RAW —— 固件可能未正常输出。")
        return 2
    print(f"[*] 收到 {n} 帧。各通道出现过的值:")
    all_neutral = True
    for i in range(1, 9):
        vs = sorted(seen[i])
        flag = "" if vs == [1500] else "  ← 非恒定 1500"
        if vs != [1500]:
            all_neutral = False
        print(f"    SERVO{i}: {vs}{flag}")
    if armed:
        print("\n[WARN] 当前处于 ARMED —— 本验证应在 DISARMED 下做。")
        return 1
    if all_neutral:
        print("\n[PASS] 未解锁且 8 路全程恒为 1500(安全中位) —— 输出通道正常且安全。")
        return 0
    print("\n[WARN] 未解锁但有通道不等于 1500 —— 请检查(是否有其它源在发指令/Cockpit 手柄?)。")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
