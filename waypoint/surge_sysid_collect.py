#!/usr/bin/env python3
"""W2 — surge 系统辨识数据采集 (开环阶跃 + DVL 测速)。

依次发一串【恒定归一化前进指令 u_x】(默认 ±0.35/±0.45/±0.55, 顶过 ESC 死区~0.3),
每个保持数秒, 其间回中位让速度衰减; 全程用 DVL 记录 (t, u_cmd, vx) 到 CSV,
供 surge_sysid_fit.py 拟合 surge 动力学。

安全:
  - 复用 pseudo_stick 限幅/解锁/退出自动上锁 + GCS 心跳。
  - **距离护栏**: DVL 航位推算 |s|>--max-dist 立即回中位并跳到下一段, 防撞池壁。
  - DVL 超龄/丢底锁 → 回中位跳过该段 (无速度数据无意义)。
  - **需入水且有底锁**; 出水/无底锁采不到 vx。深度靠操作者维持 (本脚本只发 x)。

用法 (SITL 验证 / 真机采集):
  # SITL: 先起 python -m waypoint.sim.sim_waypoint
  python -m waypoint.surge_sysid_collect --dvl-ip 127.0.0.1 --umax 0.6 --arm --yes --label sitl
  # 真机: python -m waypoint.surge_sysid_collect --umax 0.6 --max-dist 2.0 --arm --yes --label pool1
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
from waypoint.dvl_stream import DvlStream  # noqa: E402

DATA_DIR = Path(__file__).resolve().parent / "data"


def run(args) -> int:
    cfg = load_config()
    hz = float(cfg.get("control", {}).get("CTRL_HZ", 10))
    dt_nom = 1.0 / hz
    max_age = float(cfg.get("state", {}).get("max_age_s", 0.5))
    steps = [float(x) for x in args.steps.split(",")]

    stick = PseudoStick(cfg, endpoint=args.endpoint)
    if args.umax is not None:
        stick.u_max = float(args.umax)
    dvl = DvlStream(ip=args.dvl_ip, port=args.dvl_port).start()

    DATA_DIR.mkdir(exist_ok=True)
    csv_path = DATA_DIR / f"surge_sysid_{args.label}.csv"
    rows = []

    try:
        if not stick.wait_heartbeat(float(cfg["connection"].get("heartbeat_timeout_s", 10))):
            return 1
        print(f"[sysid] U_MAX={stick.u_max}  等待 DVL 底锁 ...")
        t_warm = time.monotonic() + 5.0
        while time.monotonic() < t_warm and not dvl.is_fresh(max(max_age, 0.6)):
            time.sleep(0.1)
        if not dvl.is_fresh(max(max_age, 0.6)):
            print("[sysid] ❌ 无 DVL 有效速度 (气中/无底锁?)。入水贴底后再采。")
            return 1
        print("[sysid] DVL 底锁 OK。")

        if args.arm:
            if not args.yes and input("[sysid] ⚠ 即将解锁并开环前进发指令, 确认现场安全(远离池壁)? yes: ").strip().lower() != "yes":
                return 0
            stick.set_mode(args.mode); time.sleep(0.3)
            if not stick.arm(force=args.force_arm):
                return 2

        t0 = time.monotonic()
        seq = []
        for u in steps:
            seq.append((u, args.hold)); seq.append((0.0, args.settle))
        print(f"[sysid] {len(steps)} 阶跃 {steps} 各保持{args.hold}s/回中{args.settle}s "
              f"距离护栏±{args.max_dist}m → {csv_path.name}")

        for (u_cmd, dur) in seq:
            s = 0.0  # 每段航位推算(段内积分, 判撞墙)
            t_end = time.monotonic() + dur
            last = time.monotonic()
            aborted = ""
            while time.monotonic() < t_end:
                cyc = time.monotonic()
                dt = max(1e-3, cyc - last); last = cyc
                fresh = dvl.is_fresh(max(max_age, 0.6))
                vx = dvl.latest().vx * args.vx_sign if fresh else 0.0
                s += vx * dt
                # 距离护栏
                if u_cmd != 0.0 and abs(s) > args.max_dist:
                    aborted = f"距离护栏 |s|={abs(s):.2f}>±{args.max_dist}"
                    break
                if u_cmd != 0.0 and not fresh:
                    aborted = "DVL 丢失"
                    break
                stick.send(x=u_cmd)
                if int((cyc - t0) * hz) % int(hz) == 0:
                    stick.send_gcs_heartbeat()
                rows.append([round(cyc - t0, 3), u_cmd,
                             round(vx, 4), round(s, 3), int(fresh)])
                sl = dt_nom - (time.monotonic() - cyc)
                if sl > 0:
                    time.sleep(sl)
            if aborted:  # 护栏/丢失 → 立即回中位稳定
                stick.send_neutral()
                print(f"  u={u_cmd:+.2f} 段中止({aborted}) → 回中位稳定{args.settle}s")
                t_s = time.monotonic() + args.settle
                while time.monotonic() < t_s:
                    stick.send_neutral(); time.sleep(dt_nom)
            else:
                print(f"  u={u_cmd:+.2f} 段完成, 末 vx={dvl.latest().vx:+.3f} s={s:+.2f}")
        print("[sysid] 采集完成。")
        return 0
    except KeyboardInterrupt:
        print("\n[sysid] 中断 → 中位+上锁")
        return 130
    finally:
        dvl.stop()
        stick.close()
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "u_cmd", "vx", "s_seg", "dvl_valid"])
            w.writerows(rows)
        print(f"[sysid] 已保存 {csv_path} ({len(rows)} 行)")


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="W2 surge 系统辨识采集 (开环阶跃+DVL)")
    p.add_argument("--endpoint", default=None)
    p.add_argument("--dvl-ip", default=None)
    p.add_argument("--dvl-port", type=int, default=None)
    p.add_argument("--steps", default="0.35,-0.35,0.45,-0.45,0.55,-0.55",
                   help="逗号分隔的 u_x 序列 (顶过 ESC 死区)")
    p.add_argument("--hold", type=float, default=3.0, help="每个 u 保持秒数")
    p.add_argument("--settle", type=float, default=4.0, help="回中位稳定秒数")
    p.add_argument("--max-dist", type=float, default=2.0, help="段内距离护栏 (m, 防撞墙)")
    p.add_argument("--vx-sign", type=float, default=1.0, help="DVL vx 前进符号 (W1 确认)")
    p.add_argument("--umax", type=float, default=0.6, help="覆盖 U_MAX 顶过 ESC 死区")
    p.add_argument("--arm", action="store_true")
    p.add_argument("--force-arm", action="store_true")
    p.add_argument("--mode", default="MANUAL")
    p.add_argument("--yes", action="store_true")
    p.add_argument("--label", default="run")
    return run(p.parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
