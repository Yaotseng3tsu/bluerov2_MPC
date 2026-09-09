#!/usr/bin/env python3
"""Phase 0 — BlueROV2 R4 连接自检 (只读)。

功能:
  - 读取 config/vehicle.yaml
  - 通过 MAVLink 连接 (默认 udpin:0.0.0.0:14550)
  - 等待 heartbeat,打印飞控 system/component、autopilot 类型、arm 状态、飞行模式
  - 请求并持续打印深度 (GLOBAL_POSITION_INT.relative_alt,回退 SCALED_PRESSURE2)
  - 给出深度符号自检提示

安全:本脚本【只读】,不发送任何运动指令,绝不解锁 (arm)。
      唯一的上行报文是 SET_MESSAGE_INTERVAL(请求遥测频率),不影响载具运动。

用法:
  python -m src.link --check                 # 连真机/模拟器,自检 ~30s
  python -m src.link --check --seconds 10
  python -m src.link --endpoint udpin:0.0.0.0:14550
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Windows 控制台可能是 cp932/GBK,强制 utf-8 输出以免中文报错
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

try:
    import yaml
except ImportError:
    sys.exit("缺少 pyyaml,请先 pip install -r requirements.txt")

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "vehicle.yaml"

MAV_MODE_FLAG_SAFETY_ARMED = 128


def load_config(path: Path = CONFIG_PATH) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def import_mavutil():
    try:
        from pymavlink import mavutil
    except ImportError as exc:
        raise RuntimeError(
            "pymavlink 未安装。运行: python -m pip install -r requirements.txt"
        ) from exc
    return mavutil


def connect(mavutil, endpoint: str, dialect: str):
    print(f"[link] 连接 {endpoint} (dialect={dialect}) ...")
    conn = mavutil.mavlink_connection(
        endpoint, dialect=dialect, source_system=255, source_component=0,
        autoreconnect=True,
    )
    return conn


def wait_heartbeat(conn, timeout_s: float):
    print(f"[link] 等待 heartbeat (最多 {timeout_s:.0f}s) ...")
    hb = conn.wait_heartbeat(timeout=timeout_s)
    if hb is None:
        raise RuntimeError(
            "未收到 heartbeat。检查 BlueOS UDP endpoint、上位机 IP/端口、"
            "Windows 防火墙是否放行 UDP。"
        )
    armed = bool(hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
    print("[link] 收到 heartbeat:")
    print(f"       system={conn.target_system} component={conn.target_component}")
    print(f"       autopilot={hb.autopilot} type={hb.type}")
    print(f"       arm 状态 = {'ARMED ⚠' if armed else 'DISARMED (安全)'}")
    try:
        print(f"       飞行模式 = {conn.flightmode}")
    except Exception:
        print(f"       custom_mode = {hb.custom_mode}")
    return hb


def request_message_interval(mavutil, conn, message_name: str, hz: float) -> None:
    msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{message_name}", None)
    if msg_id is None:
        print(f"[link] 警告: 未知消息名 {message_name},跳过频率请求")
        return
    interval_us = int(1e6 / hz) if hz > 0 else 0
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        float(msg_id), float(interval_us), 0, 0, 0, 0, 0,
    )
    print(f"[link] 已请求 {message_name} @ {hz:.0f} Hz")


def parse_depth(msg):
    """返回 (depth_m, source_field) —— 向下为正。None 表示该消息不含深度。"""
    t = msg.get_type()
    if t == "GLOBAL_POSITION_INT":
        # relative_alt: mm, 水下为负 -> 深度取负号后为正
        return -msg.relative_alt / 1000.0, "relative_alt"
    if t == "SCALED_PRESSURE2":
        # press_abs: hPa; 粗略换算 (淡水 rho=1000, g=9.81, 海面≈1013.25 hPa)
        depth = (msg.press_abs - 1013.25) * 100.0 / (1000.0 * 9.81)
        return depth, "press_abs"
    if t == "VFR_HUD":
        return -msg.alt, "vfr_alt"
    return None


def run_check(conn, mavutil, depth_msg: str, seconds: float) -> int:
    request_message_interval(mavutil, conn, depth_msg, 10.0)

    print(f"\n[link] 开始 {seconds:.0f}s 深度自检。请手动上下移动 ROV,观察深度变化 & 符号。")
    print("       约定: 向下潜 -> 深度值应【增大】。若相反,请在 P2 记录并修正符号。\n")

    t_end = time.monotonic() + seconds
    hb_count = depth_count = 0
    last_print = 0.0
    depth_min = float("inf")
    depth_max = float("-inf")

    while time.monotonic() < t_end:
        msg = conn.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            print("[link] ...1s 无消息 (检查链路)")
            continue
        t = msg.get_type()
        if t == "HEARTBEAT":
            hb_count += 1
        d = parse_depth(msg)
        if d is not None:
            depth_m, field = d
            depth_count += 1
            depth_min = min(depth_min, depth_m)
            depth_max = max(depth_max, depth_m)
            now = time.monotonic()
            if now - last_print >= 0.5:
                last_print = now
                print(f"  depth = {depth_m:+.3f} m  ({t}.{field})")

    span = depth_max - depth_min if depth_count else 0.0
    depth_hz = depth_count / seconds if seconds > 0 else 0.0
    hb_hz = hb_count / seconds if seconds > 0 else 0.0
    print("\n[link] === 自检小结 ===")
    print(f"  heartbeat {hb_count} 帧 ({hb_hz:.1f} Hz)")
    print(f"  深度 {depth_count} 帧 ({depth_hz:.1f} Hz),源={depth_msg},"
          f"范围 [{depth_min:+.3f}, {depth_max:+.3f}] m,跨度 {span:.3f} m")
    ok = hb_count > 0 and depth_count > 0
    if not ok:
        print("  ❌ 未同时收到 heartbeat 与深度,链路/配置需排查。")
        return 1
    if depth_hz < 3.0:
        print(f"  ⚠ 深度到达率偏低 ({depth_hz:.1f} Hz) —— 闭环建议 ≥ CTRL_HZ;"
              f"可提高 SET_MESSAGE_INTERVAL 或换深度源。")
    if span < 0.02:
        print("  ⚠ 深度几乎无变化 —— 若你确实移动了 ROV,检查深度源是否正确。")
    print("  ✅ 链路 OK:heartbeat + 深度均可读。")
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="BlueROV2 R4 连接自检 (只读)")
    p.add_argument("--check", action="store_true", help="运行深度自检循环")
    p.add_argument("--endpoint", default=None, help="覆盖 config 里的 endpoint")
    p.add_argument("--seconds", type=float, default=30.0, help="自检时长")
    p.add_argument("--config", default=str(CONFIG_PATH))
    args = p.parse_args(argv)

    cfg = load_config(Path(args.config))
    conn_cfg = cfg["connection"]
    endpoint = args.endpoint or conn_cfg["endpoint"]
    dialect = conn_cfg.get("dialect", "ardupilotmega")
    hb_timeout = float(conn_cfg.get("heartbeat_timeout_s", 10.0))
    depth_msg = conn_cfg.get("depth_message", "GLOBAL_POSITION_INT")

    mavutil = import_mavutil()
    conn = connect(mavutil, endpoint, dialect)
    try:
        wait_heartbeat(conn, hb_timeout)
        if args.check:
            return run_check(conn, mavutil, depth_msg, args.seconds)
        print("[link] 未加 --check,仅验证 heartbeat。完成。")
        return 0
    except KeyboardInterrupt:
        print("\n[link] 用户中断。")
        return 130
    finally:
        try:
            conn.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
