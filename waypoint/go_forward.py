#!/usr/bin/env python3
"""定高前进 —— 高度 PID(z) + 距离 PID(x) + 航向 PID(r) 三闭环,叠加在同一条伪手柄指令上。

目标:先稳定在指定离底高度,再沿艇艏方向前进指定距离。

两路 PID(都复用 src/pid.py,已验证):
  1) 高度 z:反馈 DVL altitude。把"高度"取负当深度用 → PID 的 +u = 下潜。
     `--u-bias` 前馈补负浮力;`--alt-slew` 设定值限速;外部钳位做 back-calculation 抗饱和。
  2) 距离 x:反馈 DVL 航位推算 s = ∫vx·dt。
     u_x = Kp·(sp−s) + Ki·∫e − Kd·vx   —— D 项直接用 DVL 实测 vx,无需数值微分。
     `--v-cruise` 把距离设定值做成匀速斜坡 → 机器人匀速前进,不会冲过头。

  3) 航向 r:反馈飞控 ATTITUDE.yaw(不用 DVL 的 yaw,后者无罗盘会漂)。
     锁定解锁时的航向, HeadingHold(PD) 输出 r, 抑制上升/前进过程中的偏航。

设定值节流(关键): 高度和距离的设定值只有在"实测跟得上"时才继续推进。
  实测教训: v_cruise=0.15 但机器人只有 ~0.06m/s → 设定值跑到 2.0 而实际才 0.90,
  滞后 1.18m、u_x 74% 时间顶死限幅, PID 退化成开关控制。节流后滞后 <0.1m、不再饱和。

阶段: HOLD(只控高度,等稳) → ADVANCE(高度+距离+航向) → DONE(反推刹车并稳住)

安全:
  - DVL 丢底锁/超龄 → 立即回中位并停(高度和距离都来自 DVL,没它什么都不能做)。
  - 高度越出 [--alt-min, --alt-max] 即停(alt_min 保护"先武装再生效",允许从池底起浮)。
  - 距离硬护栏 --max-dist(防撞池壁);|u_x| 限幅 --x-limit。
  - 退出:回中位 → 交回 --exit-mode(默认 ALT_HOLD) → 默认不 disarm(负浮力不沉底)。

用法:
  python -m waypoint.go_forward --alt 0.8 --dist 2.0 --u-bias -0.6 --v-cruise 0.15
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
from waypoint.heading_hold import HeadingHold, wrap_deg  # noqa: E402

DATA_DIR = Path(__file__).resolve().parent / "data"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="定高前进 (高度 PID + 距离 PID)")
    # --- 目标 ---
    ap.add_argument("--alt", type=float, default=0.8, help="目标离底高度 (m)")
    ap.add_argument("--dist", type=float, default=2.0, help="前进距离 (m)")
    ap.add_argument("--v-cruise", type=float, default=0.08, dest="v_cruise",
                    help="前进速度 = 距离设定值爬升限速 (m/s)")
    # --- 高度 PID (沿用 altitude_hold 验证过的一组) ---
    ap.add_argument("--alt-kp", type=float, default=1.2, dest="alt_kp")
    ap.add_argument("--alt-ki", type=float, default=0.15, dest="alt_ki")
    ap.add_argument("--alt-kd", type=float, default=1.5, dest="alt_kd")
    ap.add_argument("--u-bias", type=float, default=-0.6, dest="u_bias",
                    help="垂直前馈偏置(负=上推力,补负浮力);实测悬停约 -0.6")
    ap.add_argument("--alt-slew", type=float, default=0.15, dest="alt_slew")
    ap.add_argument("--z-limit", type=float, default=0.9, dest="z_limit")
    # --- 距离 PID ---
    ap.add_argument("--x-kp", type=float, default=1.0, dest="x_kp")
    ap.add_argument("--x-ki", type=float, default=0.05, dest="x_ki")
    ap.add_argument("--x-kd", type=float, default=1.2, dest="x_kd",
                    help="D 项作用于 DVL 实测 vx(阻尼)")
    ap.add_argument("--x-limit", type=float, default=0.8, dest="x_limit",
                    help="前进指令 |u_x| 限幅")
    ap.add_argument("--max-lag", type=float, default=0.25, dest="max_lag",
                    help="距离设定值最大允许超前实测多少米(节流);越小越跟脚")
    ap.add_argument("--alt-lag", type=float, default=0.20, dest="alt_lag",
                    help="高度设定值最大允许超前实测多少米(节流)")
    # --- 航向 PID (第三路: 抑制上升/前进时的 yaw 偏移) ---
    ap.add_argument("--yaw-kp", type=float, default=1.0, dest="yaw_kp")
    ap.add_argument("--yaw-kd", type=float, default=0.3, dest="yaw_kd")
    ap.add_argument("--r-limit", type=float, default=0.5, dest="r_limit",
                    help="偏航指令 |u_r| 限幅")
    ap.add_argument("--heading", type=float, default=None,
                    help="要保持的绝对航向(度);默认=解锁时的当前航向")
    ap.add_argument("--no-yaw", action="store_true", help="关闭航向控制")
    # --- 阶段/容差 ---
    ap.add_argument("--alt-tol", type=float, default=0.06, dest="alt_tol",
                    help="高度到位容差 (m)")
    ap.add_argument("--hold-settle", type=float, default=2.0, dest="hold_settle",
                    help="高度连续在容差内多少秒才开始前进")
    ap.add_argument("--hold-timeout", type=float, default=25.0, dest="hold_timeout",
                    help="等高度稳定的最长时间 (s)")
    ap.add_argument("--dist-tol", type=float, default=0.05, dest="dist_tol")
    ap.add_argument("--done-hold", type=float, default=4.0, dest="done_hold",
                    help="到距后继续定高多少秒再退出")
    # --- 安全 ---
    ap.add_argument("--alt-min", type=float, default=0.3, dest="alt_min")
    ap.add_argument("--alt-max", type=float, default=1.5, dest="alt_max")
    ap.add_argument("--max-dist", type=float, default=None, dest="max_dist",
                    help="距离硬护栏 (m);默认 dist+0.4")
    ap.add_argument("--vx-sign", type=float, default=1.0, dest="vx_sign",
                    help="DVL vx 前进符号 (+1/-1)")
    ap.add_argument("--max-age", type=float, default=1.0, dest="max_age")
    ap.add_argument("--umax", type=float, default=1.0)
    ap.add_argument("--seconds", type=float, default=180.0, help="总超时")
    # --- 运行 ---
    ap.add_argument("--exit-mode", default="ALT_HOLD")
    ap.add_argument("--disarm", action="store_true")
    ap.add_argument("--mode", default="MANUAL")
    ap.add_argument("--dvl-ip", default=None)
    ap.add_argument("--dvl-port", type=int, default=None)
    ap.add_argument("--endpoint", default=None)
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--label", default="run")
    args = ap.parse_args(argv)

    max_dist = args.max_dist if args.max_dist is not None else args.dist + 0.4
    cfg = load_config()
    hz = float(cfg.get("control", {}).get("CTRL_HZ", 10))
    dt_nom = 1.0 / hz

    pid_z = PID(Kp=args.alt_kp, Ki=args.alt_ki, Kd=args.alt_kd,
                i_limit=args.z_limit, u_limit=args.z_limit)
    pid_x = PID(Kp=args.x_kp, Ki=args.x_ki, Kd=args.x_kd,
                i_limit=args.x_limit, u_limit=args.x_limit)
    pid_z.reset(); pid_x.reset()

    hh = HeadingHold(kp=args.yaw_kp, kd=args.yaw_kd,
                     r_limit=args.r_limit, tol_deg=3.0)

    stick = PseudoStick(cfg, endpoint=args.endpoint)
    stick.u_max = float(args.umax)
    conn = stick.conn
    dvl = DvlStream(ip=args.dvl_ip, port=args.dvl_port).start()

    yaw_deg = None

    def drain_attitude():
        """抽干 MAVLink, 取最新 ATTITUDE.yaw(飞控融合航向, 比 DVL 的 yaw 不漂)。"""
        nonlocal yaw_deg
        import math as _m
        while True:
            m = conn.recv_match(type="ATTITUDE", blocking=False)
            if m is None:
                break
            yaw_deg = wrap_deg(_m.degrees(m.yaw))

    DATA_DIR.mkdir(exist_ok=True)
    csv_path = DATA_DIR / f"go_forward_{args.label}.csv"
    rows = []

    alt_prev = t_prev = None
    alt_rate = 0.0
    s = 0.0                 # 已走距离 (航位推算)
    phase = "HOLD"
    hold_cnt = 0
    wrong_dir = 0          # 推前进却后退的连续计数 (vx_sign 自检)
    u_x_prev = 0.0
    done_t0 = None
    stop_reason = "完成"

    try:
        if not stick.wait_heartbeat(float(cfg["connection"].get("heartbeat_timeout_s", 10))):
            return 1
        print("[fwd] 等待 DVL 底锁 ...")
        t_w = time.monotonic() + 8.0
        while time.monotonic() < t_w and not dvl.is_fresh(args.max_age):
            time.sleep(0.1)
        if not dvl.is_fresh(args.max_age):
            print("[fwd] ❌ 无 DVL 有效数据(无底锁),高度与距离都不可用。")
            return 1
        alt0 = dvl.latest().altitude
        print(f"[fwd] 底锁 OK  当前高度={alt0:.3f}m → 目标 {args.alt:.2f}m;"
              f" 然后前进 {args.dist:.2f}m @ {args.v_cruise:.2f}m/s")
        print(f"[fwd] 高度PID kp{args.alt_kp}/ki{args.alt_ki}/kd{args.alt_kd} bias{args.u_bias} | "
              f"距离PID kp{args.x_kp}/ki{args.x_ki}/kd{args.x_kd} 限幅{args.x_limit} | "
              f"护栏 max_dist={max_dist:.2f}m vx_sign={args.vx_sign:+.0f}")

        if not args.yes:
            ans = input("[fwd] ⚠ 即将解锁并做定高前进,确认前方无障碍/池壁余量足够? 输入 yes: ")
            if ans.strip().lower() != "yes":
                print("[fwd] 已取消。")
                return 0

        stick.set_mode(args.mode)
        time.sleep(0.3)
        if not stick.arm():
            return 2

        # 请求 ATTITUDE 并锁定要保持的航向
        if not args.no_yaw:
            from pymavlink import mavutil as _mv
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                _mv.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                float(_mv.mavlink.MAVLINK_MSG_ID_ATTITUDE), 5e4, 0, 0, 0, 0, 0)
            t_y = time.monotonic() + 3.0
            while time.monotonic() < t_y and yaw_deg is None:
                drain_attitude(); time.sleep(0.05)
            if yaw_deg is None:
                print("[fwd] ⚠ 收不到 ATTITUDE, 本次关闭航向控制")
                args.no_yaw = True
            else:
                hh.reset(args.heading if args.heading is not None else yaw_deg)
                print(f"[fwd] 航向锁定 = {hh.target_deg:.1f}° (当前 {yaw_deg:.1f}°)")

        t0 = time.monotonic()
        sp_alt = dvl.latest().altitude
        sp_dist = 0.0
        alt_guard = (sp_alt >= args.alt_min + 0.05)
        last_print = 0.0

        while True:
            cyc = time.monotonic()
            el = cyc - t0
            if el >= args.seconds:
                stop_reason = "总超时"; stick.send_neutral(); break

            # --- DVL ---
            if not dvl.is_fresh(args.max_age):
                stick.send_neutral(); stop_reason = "DVL 丢底锁/超龄"; break
            d = dvl.latest()
            alt = d.altitude
            vx = d.vx * args.vx_sign

            # 高度变化率
            if alt_prev is not None and t_prev is not None:
                dtm = max(1e-3, cyc - t_prev)
                alt_rate += 0.3 * ((alt - alt_prev) / dtm - alt_rate)
            alt_prev, t_prev = alt, cyc

            # --- 安全 ---
            if not alt_guard and alt >= args.alt_min + 0.05:
                alt_guard = True
            if alt_guard and alt < args.alt_min:
                stick.send_neutral(); stop_reason = f"低于 alt_min ({alt:.2f})"; break
            if alt > args.alt_max:
                stick.send_neutral(); stop_reason = f"高于 alt_max ({alt:.2f})"; break
            if s > max_dist:
                stick.send_neutral(); stop_reason = f"距离护栏 ({s:.2f}>{max_dist:.2f})"; break
            # vx 符号自检: 命令前进却测到持续后退 → --vx-sign 多半反了。
            # 若不拦, s 会越走越负、误差越来越大, u_x 一直顶满冲向池壁 (max_dist 护栏永不触发)。
            if s < -0.3:
                stick.send_neutral()
                stop_reason = f"反向位移 s={s:.2f} → --vx-sign 可能反了"; break

            # --- 高度 PID (始终运行) ---
            if args.alt_slew > 0:
                # 设定值节流: 实测跟不上时就不再推进设定值, 避免 sp 跑到机器人前面
                # (实测从池底起浮时 sp 已到 0.80 而 alt 才 0.50 → 超调到 0.958)
                if abs(sp_alt - alt) <= args.alt_lag:
                    st = args.alt_slew * dt_nom
                    sp_alt += max(-st, min(st, args.alt - sp_alt))
            else:
                sp_alt = args.alt
            u_pid_z = pid_z.compute(-sp_alt, -alt, -alt_rate, dt_nom)
            u_raw = args.u_bias + u_pid_z
            u_z = max(-1.0, min(1.0, u_raw))
            if u_z != u_raw and pid_z.Ki > 0:
                pid_z.integ -= (u_raw - u_z) / pid_z.Ki

            # --- 阶段逻辑 ---
            u_x = 0.0
            if phase == "HOLD":
                if abs(args.alt - alt) <= args.alt_tol:
                    hold_cnt += 1
                    if hold_cnt >= int(args.hold_settle * hz):
                        phase = "ADVANCE"
                        pid_x.reset()
                        print(f"[fwd] ▶ 高度已稳({alt:.3f}m),开始前进 {args.dist:.2f}m")
                else:
                    hold_cnt = 0
                if el > args.hold_timeout and phase == "HOLD":
                    stop_reason = f"高度未在 {args.hold_timeout:.0f}s 内稳定"
                    stick.send_neutral(); break

            elif phase == "ADVANCE":
                # 推进方向自检: 明确在推前进, 却持续测到后退 → 符号/接线有问题, 及早停
                if u_x_prev > 0.15 and vx < -0.03:
                    wrong_dir += 1
                    if wrong_dir >= int(2.0 * hz):
                        stick.send_neutral()
                        stop_reason = ("推前进却持续后退 → --vx-sign 反了 或 推进方向不对")
                        break
                else:
                    wrong_dir = 0
                s += vx * dt_nom                       # 航位推算
                # 设定值节流: 只有机器人跟得上(滞后 < max_lag)才继续推进设定值。
                # 实测 v_cruise=0.15 但机器人只有 ~0.06m/s → sp 跑到 2.0 而 s 才 0.90,
                # 误差被拉到 1.1m、u_x 常年顶死限幅。节流后自动适配机器人真实速度。
                if sp_dist - s <= args.max_lag:
                    st = args.v_cruise * dt_nom
                    sp_dist = min(args.dist, sp_dist + st)
                u_x = pid_x.compute(sp_dist, s, vx, dt_nom)
                if s >= args.dist - args.dist_tol and sp_dist >= args.dist - 1e-6:
                    phase = "DONE"; done_t0 = cyc; u_x = 0.0
                    print(f"[fwd] ✔ 已到距 s={s:.3f}m,停前进,继续定高 {args.done_hold:.0f}s")

            else:  # DONE:距离 PID 继续以 dist 为目标 → 滑行超出即反推刹车并稳住位置
                s += vx * dt_nom
                u_x = pid_x.compute(args.dist, s, vx, dt_nom)
                if cyc - done_t0 >= args.done_hold:
                    break

            u_x = max(-args.x_limit, min(args.x_limit, u_x))
            u_x_prev = u_x

            # --- 航向 PID (第三路) ---
            u_r = 0.0
            if not args.no_yaw:
                drain_attitude()
                if yaw_deg is not None:
                    u_r = hh.update(yaw_deg, dt_nom)

            stick.send(x=u_x, z=u_z, r=u_r)
            if int(el * hz) % int(hz) == 0:
                stick.send_gcs_heartbeat()

            rows.append([round(el, 3), phase, round(sp_alt, 4), round(alt, 4),
                         round(u_z, 4), round(sp_dist, 4), round(s, 4),
                         round(vx, 4), round(u_x, 4),
                         round(yaw_deg, 2) if yaw_deg is not None else "",
                         round(hh.error_deg(yaw_deg), 2) if (yaw_deg is not None and not args.no_yaw) else "",
                         round(u_r, 4)])
            if el - last_print >= 0.5:
                last_print = el
                ystr = (f" | yaw={yaw_deg:+.1f} err={hh.error_deg(yaw_deg):+.1f} u_r={u_r:+.2f}"
                        if (yaw_deg is not None and not args.no_yaw) else "")
                print(f"  t={el:5.1f} [{phase:7s}] alt={alt:.3f}/{args.alt:.2f} u_z={u_z:+.3f} | "
                      f"s={s:.3f}/{args.dist:.2f} (sp{sp_dist:.2f}) vx={vx:+.3f} u_x={u_x:+.3f}{ystr}")

            sl = dt_nom - (time.monotonic() - cyc)
            if sl > 0:
                time.sleep(sl)

        print(f"[fwd] 结束 ({stop_reason})。")
        if rows:
            last = rows[-1]
            print(f"[fwd] 末态: 高度={last[3]:.3f}m (目标{args.alt:.2f}, 误差{args.alt-last[3]:+.3f}) | "
                  f"已走 s={last[6]:.3f}m (目标{args.dist:.2f}, 误差{args.dist-last[6]:+.3f})")
        return 0
    except KeyboardInterrupt:
        print("\n[fwd] 用户中断 → 回中位")
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
            print(f"[fwd] 已交回 {args.exit_mode}")
        if not args.disarm:
            stick.armed_by_us = False
            print("[fwd] 保持 armed (未上锁)")
        dvl.stop()
        stick.close()
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "phase", "sp_alt", "alt", "u_z", "sp_dist", "s", "vx", "u_x",
                        "yaw", "yaw_err", "u_r"])
            w.writerows(rows)
        print(f"[fwd] 已保存 {csv_path} ({len(rows)} 行)")


if __name__ == "__main__":
    raise SystemExit(main())
