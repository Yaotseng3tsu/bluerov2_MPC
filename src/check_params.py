#!/usr/bin/env python3
"""失效保护(failsafe)参数只读查询 —— 真机现场第一步用。

通过 MAVLink 逐个读取 ArduSub 的 failsafe 相关参数并打印,带简要释义。
【严格只读】:只发 PARAM_REQUEST_READ,绝不发 PARAM_SET,不改任何参数、不解锁。

用法:
  python -m src.check_params                 # 读默认 failsafe 清单
  python -m src.check_params --param FS_PILOT_INPUT FS_GCS_ENABLE
  python -m src.check_params --all           # 拉全部参数(量大,慎用)
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "vehicle.yaml"

# 关注的 failsafe 参数 + 简要释义(实际枚举以 BlueOS/ArduSub 文档为准)
FAILSAFE_PARAMS = {
    "FS_PILOT_INPUT":   "手柄/MANUAL_CONTROL 失联动作 (常见 0=禁用 1=仅警告 2=disarm)",
    "FS_PILOT_TIMEOUT": "手柄失联判定超时 (秒)",
    "FS_GCS_ENABLE":    "地面站失联保护 (0=禁用 / 非0=启用)",
    "FS_LEAK_ENABLE":   "漏水检测保护开关",
    "FS_LEAK_ACTION":   "漏水时动作",
    "FS_CRASH_CHECK":   "碰撞/异常检测",
    "FS_EKF_ACTION":    "EKF 失效动作",
    "FS_EKF_THRESH":    "EKF 失效阈值",
    "FS_BATT_ENABLE":   "电池失效保护 (旧命名)",
    "BATT_LOW_VOLT":    "低电压阈值 (V,新命名体系)",
}


def load_conn_cfg():
    import yaml
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)["connection"]


def read_one(conn, mavutil, name: str, timeout: float = 2.0):
    """请求单个参数,返回 float 值或 None。只读。"""
    conn.mav.param_request_read_send(
        conn.target_system, conn.target_component, name.encode(), -1)
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        m = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if m is None:
            continue
        pid = m.param_id
        if isinstance(pid, bytes):
            pid = pid.split(b"\x00")[0].decode(errors="ignore")
        if pid == name:
            return m.param_value
    return None


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="失效保护参数只读查询")
    p.add_argument("--endpoint", default=None)
    p.add_argument("--param", nargs="+", default=None, help="指定要读的参数名")
    p.add_argument("--all", action="store_true", help="拉取全部参数 (量大)")
    args = p.parse_args(argv)

    from pymavlink import mavutil
    c = load_conn_cfg()
    endpoint = args.endpoint or c["endpoint"]
    conn = mavutil.mavlink_connection(endpoint, dialect=c.get("dialect", "ardupilotmega"),
                                      source_system=255)
    print(f"[params] 连接 {endpoint},等待 heartbeat ...(只读,不会修改任何参数)")
    if conn.wait_heartbeat(timeout=float(c.get("heartbeat_timeout_s", 10))) is None:
        print("[params] ❌ 无 heartbeat")
        return 1
    print(f"[params] 已连接 system={conn.target_system}\n")

    if args.all:
        print("[params] 拉取全部参数(PARAM_REQUEST_LIST)...")
        conn.mav.param_request_list_send(conn.target_system, conn.target_component)
        seen = {}
        t_end = time.monotonic() + 20
        while time.monotonic() < t_end:
            m = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if m is None:
                continue
            pid = m.param_id
            if isinstance(pid, bytes):
                pid = pid.split(b"\x00")[0].decode(errors="ignore")
            seen[pid] = m.param_value
            if m.param_count and len(seen) >= m.param_count:
                break
        for k in sorted(seen):
            print(f"  {k} = {seen[k]}")
        print(f"\n[params] 共 {len(seen)} 个参数。")
        conn.close()
        return 0

    names = args.param or list(FAILSAFE_PARAMS)
    print("=== 失效保护参数(只读)===")
    for name in names:
        val = read_one(conn, mavutil, name)
        desc = FAILSAFE_PARAMS.get(name, "")
        if val is None:
            print(f"  {name:18s} = (未获取/可能不存在)   {desc}")
        else:
            v = int(val) if float(val).is_integer() else round(val, 4)
            print(f"  {name:18s} = {v!s:8s}  {desc}")

    print("\n提示:这些只是读数。是否需要调整以 BlueOS 参数页 / ArduSub 文档为准;")
    print("     本工具不修改任何参数。硬崩溃兜底主要看 FS_PILOT_INPUT / FS_PILOT_TIMEOUT / FS_GCS_ENABLE。")
    conn.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
