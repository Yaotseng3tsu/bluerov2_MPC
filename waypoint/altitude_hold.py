#!/usr/bin/env python3
"""高度控制 (altitude hold) —— PID + DVL altitude + 伪手柄。

与 depth_control 的区别:反馈用 **DVL 离底高度 altitude**(不是压力深度),
适合池内作业:直接保持"离池底多高",不受水面基准漂移影响。

控制约定:
  - 高度 alt 向上为正;深度 z 向下为正 —— 两者方向相反。
  - 令 **伪深度 m = -alt、设定值 sp = -target**,即可直接复用已验证的深度 PID
    (PID 的 +u = 下潜)。数学上等价于:
        u_z = -Kp*(target-alt) + Kd*d(alt)/dt - Ki*∫(target-alt)
  - 负浮力机器人需要**恒定上推力**才能悬停:可用 `--u-bias`(负值=上推)
    前馈补掉大部分,PID 只修残差,收敛更快、积分不易饱和。

安全:
  - DVL 无底锁 / 数据超龄 → 立即回中位并停(没有高度就不许控)。
  - alt 越出 [--alt-min, --alt-max] → 停(防撞底 / 防冲出水面)。
  - 退出:回中位 → 交回 --exit-mode(默认 ALT_HOLD) → **默认不 disarm**(负浮力不沉底)。

用法:
  # 先看当前高度:  python -m waypoint.dvl_stream --seconds 5
  python -m waypoint.altitude_hold --target 0.8 --seconds 30 --u-bias -0.5
  python -m waypoint.altitude_hold --target 0.8 --seconds 30 --u-bias -0.5 --kp 1.5 --ki 0.4 --kd 0.8
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
from src.pid import PID  # noqa: E402
from waypoint.dvl_stream import DvlStream  # noqa: E402

DATA_DIR = Path(__file__).resolve().parent / "data"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="高度控制 (PID + DVL altitude + 伪手柄)")
    ap.add_argument("--target", type=float, required=True, help="目标离底高度 (m)")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--kp", type=float, default=1.5)
    ap.add_argument("--ki", type=float, default=0.4)
    ap.add_argument("--kd", type=float, default=0.8)
    ap.add_argument("--u-limit", type=float, default=0.9, dest="u_limit",
                    help="PID 输出/积分限幅")
    ap.add_argument("--u-bias", type=float, default=0.0, dest="u_bias",
                    help="前馈偏置(负=恒定上推力,补负浮力);建议先试 -0.5")
    ap.add_argument("--slew", type=float, default=0.2,
                    help="设定值爬升限速 (m/s, 0=关闭)。大阶跃时把目标做成斜坡,"
                         "避免推力饱和导致超调")
    ap.add_argument("--umax", type=float, default=1.0, help="覆盖 U_MAX")
    ap.add_argument("--alt-min", type=float, default=0.25, dest="alt_min",
                    help="低于此高度即停(防撞底)")
    ap.add_argument("--alt-max", type=float, default=3.0, dest="alt_max",
                    help="高于此高度即停(防冲出水面)")
    ap.add_argument("--max-age", type=float, default=1.0, dest="max_age",
                    help="DVL 数据超龄阈值 (s)")
    ap.add_argument("--exit-mode", default="ALT_HOLD", help="退出时交回的模式(空=不切)")
    ap.add_argument("--disarm", action="store_true", help="退出时上锁(默认不上锁)")
    ap.add_argument("--mode", default="MANUAL")
    ap.add_argument("--dvl-ip", default=None)
    ap.add_argument("--dvl-port", type=int, default=None)
    ap.add_argument("--endpoint", default=None)
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--label", default="run")
    args = ap.parse_args(argv)

    cfg = load_config()
    hz = float(cfg.get("control", {}).get("CTRL_HZ", 10))
    dt_nom = 1.0 / hz

    pid = PID(Kp=args.kp, Ki=args.ki, Kd=args.kd,
              i_limit=args.u_limit, u_limit=args.u_limit)
    pid.reset()

    stick = PseudoStick(cfg, endpoint=args.endpoint)
    stick.u_max = float(args.umax)
    dvl = DvlStream(ip=args.dvl_ip, port=args.dvl_port).start()

    DATA_DIR.mkdir(exist_ok=True)
    csv_path = DATA_DIR / f"altitude_hold_{args.label}.csv"
    rows = []

    alt_prev = None
    t_prev = None
    alt_rate = 0.0        # d(alt)/dt, 低通滤波

    try:
        if not stick.wait_heartbeat(float(cfg["connection"].get("heartbeat_timeout_s", 10))):
            return 1
        print(f"[alt] 等待 DVL 底锁 ...")
        t_w = time.monotonic() + 8.0
        while time.monotonic() < t_w and not dvl.is_fresh(args.max_age):
            time.sleep(0.1)
        if not dvl.is_fresh(args.max_age):
            print("[alt] ❌ 无 DVL 有效高度(无底锁),不能做高度控制。")
            return 1
        alt0 = dvl.latest_valid().altitude
        print(f"[alt] 底锁 OK  当前高度={alt0:.3f}m  目标={args.target:.3f}m  "
              f"Kp={args.kp} Ki={args.ki} Kd={args.kd} u_limit={args.u_limit} "
              f"u_bias={args.u_bias}  U_MAX={stick.u_max}")

        if stick.warn_if_rival() and not args.yes:
            if input("[alt] 仍要继续? 输入 yes: ").strip().lower() != "yes":
                return 0
        if not args.yes:
            ans = input(f"[alt] ⚠ 即将解锁并在 {args.mode} 下做高度闭环,确认安全? 输入 yes: ")
            if ans.strip().lower() != "yes":
                print("[alt] 已取消。")
                return 0

        stick.set_mode(args.mode)
        time.sleep(0.3)
        if not stick.arm():
            return 2

        t0 = time.monotonic()
        sp = dvl.latest_valid().altitude      # 斜坡起点 = 当前高度
        alt_guard = (sp >= args.alt_min + 0.05)   # 起步就在安全高度以上则立即武装
        last_print = 0.0
        stop_reason = "完成"
        while True:
            cyc = time.monotonic()
            el = cyc - t0
            if el >= args.seconds:
                break

            # --- DVL 高度 ---
            if not dvl.is_fresh(args.max_age):
                stick.send_neutral()
                stop_reason = "DVL 丢底锁/超龄"
                break
            s = dvl.latest_valid()   # 用最近有效帧, 容忍瞬时丢帧
            alt = s.altitude

            # 高度变化率(数值微分 + 低通)
            if alt_prev is not None and t_prev is not None:
                dt = max(1e-3, cyc - t_prev)
                raw = (alt - alt_prev) / dt
                alt_rate += 0.3 * (raw - alt_rate)     # 一阶低通
            alt_prev, t_prev = alt, cyc

            # --- 安全边界 ---
            # alt_min 保护"先武装再生效": 允许从池底起浮(起始 alt 可能本就低于 alt_min),
            # 一旦升到安全高度以上才开始防撞底, 否则起浮瞬间就会被误判中止。
            if not alt_guard and alt >= args.alt_min + 0.05:
                alt_guard = True
            if alt_guard and alt < args.alt_min:
                stick.send_neutral()
                stop_reason = f"低于 alt_min ({alt:.2f}<{args.alt_min})"
                break
            if alt > args.alt_max:
                stick.send_neutral()
                stop_reason = f"高于 alt_max ({alt:.2f}>{args.alt_max})"
                break

            # --- 设定值限速: 大阶跃做成斜坡, 避免推力饱和 → 超调 ---
            if args.slew > 0:
                step = args.slew * dt_nom
                sp += max(-step, min(step, args.target - sp))
            else:
                sp = args.target

            # --- PID: 把"高度"取负当作"深度"用, 复用已验证的深度 PID (+u=下潜) ---
            u_pid = pid.compute(-sp, -alt, -alt_rate, dt_nom)
            u_raw = args.u_bias + u_pid
            u_z = max(-1.0, min(1.0, u_raw))
            # --- 抗饱和(back-calculation): 钳位发生在 PID 之外, 需把多余量退回积分 ---
            if u_z != u_raw and pid.Ki > 0:
                pid.integ -= (u_raw - u_z) / pid.Ki
            stick.send(z=u_z)
            if int(el * hz) % int(hz) == 0:
                stick.send_gcs_heartbeat()

            err = args.target - alt
            rows.append([round(el, 3), round(args.target, 3), round(sp, 4), round(alt, 4),
                         round(alt_rate, 4), round(u_pid, 4), round(u_z, 4)])
            if el - last_print >= 0.5:
                last_print = el
                print(f"  t={el:5.1f} alt={alt:+.3f} sp={sp:.3f}/{args.target:.2f} "
                      f"err={err:+.3f} rate={alt_rate:+.3f} u_z={u_z:+.3f}")

            sl = dt_nom - (time.monotonic() - cyc)
            if sl > 0:
                time.sleep(sl)

        print(f"[alt] 结束 ({stop_reason})。")
        if rows:
            last = rows[-1]
            print(f"[alt] 末高度={last[3]:.3f}m (目标 {args.target:.2f}, "
                  f"误差 {args.target - last[3]:+.3f}m)  末 u_z={last[6]:+.3f}")
        return 0
    except KeyboardInterrupt:
        print("\n[alt] 用户中断 → 回中位")
        return 130
    finally:
        try:
            for _ in range(5):
                stick.send_neutral(); time.sleep(0.02)
        except Exception:
            pass
        if args.exit_mode:
            stick.set_mode(args.exit_mode)
            time.sleep(0.3)
            for _ in range(5):
                try:
                    stick.send_neutral()
                except Exception:
                    pass
                time.sleep(0.02)
            print(f"[alt] 已交回 {args.exit_mode}")
        if not args.disarm:
            stick.armed_by_us = False
            print("[alt] 保持 armed (未上锁)")
        dvl.stop()
        stick.close()
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "target", "sp", "alt", "alt_rate", "u_pid", "u_z"])
            w.writerows(rows)
        print(f"[alt] 已保存 {csv_path} ({len(rows)} 行)")


if __name__ == "__main__":
    raise SystemExit(main())
