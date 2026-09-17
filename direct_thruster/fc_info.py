#!/usr/bin/env python3
"""M0 只读工具 — 读取飞控 ArduSub 版本 + 推进器相关参数 (不改任何东西)。

用于 direct_thruster 研究的 M0:
  - AUTOPILOT_VERSION: ArduSub 版本号 + git hash (决定 Phase1 要 checkout 的 tag)
  - 推进器映射备份: FRAME_CONFIG/FRAME_CLASS/FRAME_TYPE, SERVO1-8 的 FUNCTION/REVERSED/MIN/MAX/TRIM

运行 (在 C:\\bluerov2_mpc 下, 机器连着):
  .venv\\Scripts\\python direct_thruster\\fc_info.py
"""
from __future__ import annotations

import sys
import time

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

from pymavlink import mavutil

ENDPOINT = "udpin:0.0.0.0:14550"

PARAMS = ["FRAME_CONFIG", "FRAME_CLASS", "FRAME_TYPE"]
for i in range(1, 9):
    for suf in ("FUNCTION", "REVERSED", "MIN", "MAX", "TRIM"):
        PARAMS.append(f"SERVO{i}_{suf}")

# ArduSub SERVOn_FUNCTION: 33..40 = Motor1..8 (BLHeli/常规电机功能号)
SERVO_FUNC = {0: "Disabled", 1: "RCPassThru", 33: "Motor1", 34: "Motor2",
              35: "Motor3", 36: "Motor4", 37: "Motor5", 38: "Motor6",
              39: "Motor7", 40: "Motor8", 94: "Script1", 95: "Script2",
              96: "Script3", 97: "Script4", 98: "Script5", 99: "Script6",
              100: "Script7", 101: "Script8"}


def get_version(conn) -> None:
    m = mavutil.mavlink
    for _ in range(3):
        conn.mav.command_long_send(conn.target_system, conn.target_component,
                                   m.MAV_CMD_REQUEST_MESSAGE, 0,
                                   148, 0, 0, 0, 0, 0, 0)  # 148 = AUTOPILOT_VERSION
        msg = conn.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=2.0)
        if msg:
            v = msg.flight_sw_version
            major, minor, patch = (v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF
            fw_type = {0: "dev", 64: "alpha", 128: "beta", 192: "rc", 255: "official"}.get(v & 0xFF, v & 0xFF)
            cust = getattr(msg, "flight_custom_version", None)
            git = ""
            if cust:
                try:
                    git = bytes(cust).split(b"\x00")[0].decode("ascii", "ignore")
                except Exception:
                    git = str(cust)
            print(f"  ArduSub 版本 = {major}.{minor}.{patch}  ({fw_type})   git={git}")
            print(f"  → Phase1 checkout 建议: ArduSub-{major}.{minor}.{patch}")
            return
    print("  ⚠ 未取到 AUTOPILOT_VERSION (可在 BlueOS Autopilot Firmware 页看版本)")


def read_param(conn, name: str, timeout: float = 1.5):
    conn.mav.param_request_read_send(conn.target_system, conn.target_component,
                                     name.encode("ascii"), -1)
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
        if msg is None:
            continue
        pid = msg.param_id
        if isinstance(pid, bytes):
            pid = pid.split(b"\x00")[0].decode(errors="ignore")
        if pid == name:
            return msg.param_value
    return None


def main() -> int:
    from fc_link import connect_fc
    conn = connect_fc(ENDPOINT)
    if conn is None:
        return 2
    print("\n=== ArduSub 版本 ===")
    get_version(conn)

    print("\n=== 推进器相关参数 (备份用) ===")
    vals = {}
    for name in PARAMS:
        v = read_param(conn, name)
        vals[name] = v
    for k in ("FRAME_CONFIG", "FRAME_CLASS", "FRAME_TYPE"):
        print(f"  {k:14s} = {vals.get(k)}")
    print("  ---- SERVO1-8 (通常 Motor1-8 = 输出通道1-8) ----")
    print(f"  {'ch':>3} {'FUNCTION':>18} {'REV':>4} {'MIN':>6} {'MAX':>6} {'TRIM':>6}")
    for i in range(1, 9):
        f = vals.get(f"SERVO{i}_FUNCTION")
        fname = SERVO_FUNC.get(int(f), int(f)) if f is not None else "?"
        rev = vals.get(f"SERVO{i}_REVERSED")
        mn, mx, tr = vals.get(f"SERVO{i}_MIN"), vals.get(f"SERVO{i}_MAX"), vals.get(f"SERVO{i}_TRIM")
        print(f"  {i:>3} {str(fname):>18} {str(int(rev)) if rev is not None else '?':>4} "
              f"{str(int(mn)) if mn is not None else '?':>6} "
              f"{str(int(mx)) if mx is not None else '?':>6} "
              f"{str(int(tr)) if tr is not None else '?':>6}")
    print("\n[fc_info] 完成 (只读, 未改动任何参数)。完整参数请在 BlueOS Autopilot Parameters 导出备份。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
