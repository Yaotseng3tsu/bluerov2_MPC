#!/usr/bin/env python3
"""Phase 3 — 系统辨识数据采集(开环阶跃)。

依次发一串【恒定归一化 heave 指令 u】(默认 ±0.1/±0.2/±0.3),每个保持数秒,
其间回中位让深度重新稳定,全程记录 (t, u_cmd, depth_meas, depth_kf, rate_kf) 到 CSV,
供 src/sysid_fit.py 拟合深度动力学模型。

安全:复用 pseudo_stick 的限幅/解锁/退出自动上锁 + GCS 心跳 + 深度软限位。
      **需入水**(出水台架无深度变化,拟合无意义)。真机运行务必现场监管。

用法(SITL 验证 / 真机采集):
  python -m src.sysid_collect --hold 4 --settle 4 --arm --yes --label pool1
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

from src.pseudo_stick import PseudoStick, load_config  # noqa: E402
from src.state import DepthEstimator  # noqa: E402
from src.link import parse_depth  # noqa: E402
from pymavlink import mavutil  # noqa: E402

DATA_DIR = Path(__file__).resolve().parent.parent / "data"


def run(args) -> int:
    cfg = load_config()
    hz = float(cfg.get("control", {}).get("CTRL_HZ", 10))
    dt_nom = 1.0 / hz
    dmax = float(cfg.get("safety", {}).get("DEPTH_MAX", 3.0))
    depth_msg = cfg["connection"].get("depth_message", "GLOBAL_POSITION_INT")
    steps = [float(x) for x in args.steps.split(",")]

    stick = PseudoStick(cfg, endpoint=args.endpoint)
    conn = stick.conn
    est = DepthEstimator.from_config()
    DATA_DIR.mkdir(exist_ok=True)
    csv_path = DATA_DIR / f"sysid_{args.label}.csv"
    rows = []

    def drain_and_feed():
        while True:
            m = conn.recv_match(blocking=False)
            if m is None:
                break
            d = parse_depth(m, depth_msg)
            if d is not None:
                est.feed(d[0], time.monotonic())
                return d[0]
        return None

    try:
        if not stick.wait_heartbeat(float(cfg["connection"].get("heartbeat_timeout_s", 10))):
            return 1
        msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{depth_msg}", None)
        if msg_id is not None:
            conn.mav.command_long_send(conn.target_system, conn.target_component,
                                       mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                       float(msg_id), 1e6 / max(hz, 10), 0, 0, 0, 0, 0)
        # 预热
        t_warm = time.monotonic() + 1.5
        n = 0
        while time.monotonic() < t_warm:
            if drain_and_feed() is not None:
                n += 1
            if est.kf.initialized and n >= 5:
                break
        if not est.kf.initialized:
            print("[sysid] ❌ 未收到深度"); return 1

        if args.arm:
            if not args.yes and input("[sysid] ⚠ 即将解锁并开环发指令,确认安全? yes: ").strip().lower() != "yes":
                return 0
            stick.set_mode(args.mode); time.sleep(0.3)
            if not stick.arm(force=args.force_arm):
                return 2

        t0 = time.monotonic()
        seq = []
        for u in steps:
            seq.append((u, args.hold)); seq.append((0.0, args.settle))
        print(f"[sysid] 采集 {len(steps)} 个阶跃 {steps},每个保持{args.hold}s/回中{args.settle}s → {csv_path.name}")

        for (u_cmd, dur) in seq:
            t_end = time.monotonic() + dur
            while time.monotonic() < t_end:
                cyc = time.monotonic()
                raw = drain_and_feed()
                depth, rate = est.depth, est.depth_rate
                # 软限位保护
                if depth > dmax or depth < -0.5:
                    print(f"[sysid] ⚠ 深度越界 {depth:.2f} → 中止")
                    stick.send_neutral()
                    raise KeyboardInterrupt
                stick.send(z=u_cmd)
                if int((cyc - t0) * hz) % int(hz) == 0:
                    stick.send_gcs_heartbeat()
                rows.append([round(cyc - t0, 3), u_cmd,
                             round(raw, 4) if raw is not None else "",
                             round(depth, 4), round(rate, 4)])
                sl = dt_nom - (time.monotonic() - cyc)
                if sl > 0:
                    time.sleep(sl)
            print(f"  u={u_cmd:+.2f} 段完成, depth={est.depth:+.3f}")
        print("[sysid] 采集完成。")
        return 0
    except KeyboardInterrupt:
        print("\n[sysid] 中断 → 中位+上锁")
        return 130
    finally:
        stick.close()
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "u_cmd", "depth_meas", "depth_kf", "rate_kf"])
            w.writerows(rows)
        print(f"[sysid] 已保存 {csv_path} ({len(rows)} 行)")


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="P3 系统辨识数据采集(开环阶跃)")
    p.add_argument("--endpoint", default=None)
    p.add_argument("--steps", default="0.1,-0.1,0.2,-0.2,0.3,-0.3", help="逗号分隔的 u 序列")
    p.add_argument("--hold", type=float, default=4.0, help="每个 u 保持秒数")
    p.add_argument("--settle", type=float, default=4.0, help="回中位稳定秒数")
    p.add_argument("--arm", action="store_true")
    p.add_argument("--force-arm", action="store_true")
    p.add_argument("--mode", default="MANUAL")
    p.add_argument("--yes", action="store_true")
    p.add_argument("--label", default="run")
    return run(p.parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
