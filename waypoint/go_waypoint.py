#!/usr/bin/env python3
"""W5 — 相对航点定向航行主脚本 (Point-and-go 4DOF)。

状态机: DESCEND(调深度) → TURN(转航向) → CRUISE(沿航向前进到距离) → STOP → DONE。
  - 深度: 绝对深度闭环 (复用 src/pid + src/state), 全程保持目标深度。
  - 航向: heading_hold (ATTITUDE.yaw), TURN 阶段转向, CRUISE 阶段保持走直线。
  - 前进: motion_model 前馈 u_x; 距离用 DVL 航位推算 s=∫vx·dt 判定 (到距即停)。
  - 发送/安全: 复用 pseudo_stick (限幅/看门狗/退出回中位+自动上锁)。

安全降级:
  - DVL 超龄/丢底锁 (CRUISE 阶段) → 立即停车。
  - 深度软限位越界、roll/pitch 超阈 → 停车/告警。
  - 总时长超时 → 停。

用法:
  # 离线: 先起 SITL   python -m waypoint.sim.sim_waypoint
  python -m waypoint.go_waypoint --heading 90 --dist 3 --depth 0.5 --arm --yes
  # 真机: 加 --umax 0.6 顶过 ESC 死区, --deadzone 0.3 死区补偿 (下水前确认)
"""
from __future__ import annotations

import argparse
import csv
import math
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
from src.pid import PID  # noqa: E402
from src.link import parse_depth  # noqa: E402
from waypoint.heading_hold import HeadingHold, wrap_deg  # noqa: E402
from waypoint.motion_model import SurgeParams, plan_feedforward  # noqa: E402
from waypoint.dvl_stream import DvlStream  # noqa: E402

DATA_DIR = Path(__file__).resolve().parent / "data"


def deadzone_comp(u: float, db: float, eps: float = 1e-3) -> float:
    """死区补偿: 把小的非零 u 顶到至少 ±db, 使 ESC 越过死区。db=0 时原样返回。"""
    if db <= 0 or abs(u) < eps:
        return u
    return math.copysign(db + (1.0 - db) * abs(u), u)


def run(args) -> int:
    cfg = load_config()
    hz = float(cfg.get("control", {}).get("CTRL_HZ", 10))
    dt_nom = 1.0 / hz
    safety = cfg.get("safety", {})
    dmax = float(safety.get("DEPTH_MAX", 3.0))
    depth_msg = cfg["connection"].get("depth_message", "GLOBAL_POSITION_INT")
    max_age = float(cfg.get("state", {}).get("max_age_s", 0.5))

    # 控制器
    pid = PID.from_config(cfg)
    pid.reset()
    est = DepthEstimator.from_config()
    hh = HeadingHold(kp=args.yaw_kp, kd=args.yaw_kd,
                     r_limit=args.r_limit, tol_deg=args.heading_tol)
    surge_p = SurgeParams.from_yaml()
    traj, u_ff = plan_feedforward(surge_p, args.dist, args.v_cruise, args.a_max, dt_nom)
    print(f"[wp] surge 参数={'identified' if surge_p.using_identified() else 'sim_truth占位'}"
          f"  前馈总时长={traj.total_time:.1f}s |u_ff|峰值={max(abs(u) for u in u_ff):.3f}")

    stick = PseudoStick(cfg, endpoint=args.endpoint)
    if args.umax is not None:
        stick.u_max = float(args.umax)
        print(f"[wp] U_MAX 覆盖为 {stick.u_max} (顶过 ESC 死区)")
    conn = stick.conn

    dvl = DvlStream(ip=args.dvl_ip, port=args.dvl_port).start()

    DATA_DIR.mkdir(exist_ok=True)
    csv_path = DATA_DIR / f"go_waypoint_{args.label}.csv"
    rows = []

    # 运行时状态
    yaw_deg = None           # 最新 ATTITUDE 航向
    roll_deg = pitch_deg = 0.0
    s = 0.0                  # 航位推算已走距离
    phase = "DESCEND"
    hold_cnt = 0            # 阶段到位的连续计数
    cruise_t0 = None
    stop_t0 = None         # 进入 STOP 的时刻
    emergency = False      # True=硬安全停(全中位); False=正常到达停(保持深度/航向)
    reason = ""

    def drain_mavlink():
        nonlocal yaw_deg, roll_deg, pitch_deg
        while True:
            m = conn.recv_match(blocking=False)
            if m is None:
                break
            t = m.get_type()
            if t == "ATTITUDE":
                yaw_deg = wrap_deg(math.degrees(m.yaw))
                roll_deg = math.degrees(m.roll)
                pitch_deg = math.degrees(m.pitch)
            else:
                d = parse_depth(m)
                if d is not None:
                    est.feed(d[0], time.monotonic())

    try:
        if not stick.wait_heartbeat(float(cfg["connection"].get("heartbeat_timeout_s", 10))):
            return 1
        from pymavlink import mavutil
        for mid_name in (depth_msg, "ATTITUDE"):
            mid = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{mid_name}", None)
            if mid is not None:
                conn.mav.command_long_send(conn.target_system, conn.target_component,
                                           mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                           float(mid), 1e6 / max(hz, 20), 0, 0, 0, 0, 0)

        # 预热: 收深度+航向
        t_warm = time.monotonic() + 2.0
        while time.monotonic() < t_warm:
            drain_mavlink()
            if est.kf.initialized and yaw_deg is not None:
                break
            time.sleep(0.02)
        if not est.kf.initialized or yaw_deg is None:
            print(f"[wp] ❌ 预热未就绪 (depth_init={est.kf.initialized} yaw={yaw_deg})")
            return 1

        # 目标航向: 绝对, 或 --rel 相对起始
        target_heading = wrap_deg((yaw_deg + args.heading) if args.rel else args.heading)
        hh.reset(target_heading)
        print(f"[wp] 起始 yaw={yaw_deg:.1f}° depth={est.depth:+.2f}m → "
              f"目标 heading={target_heading:.1f}° dist={args.dist}m depth={args.depth}m")

        if args.arm:
            if not args.yes:
                ans = input("[wp] ⚠ 即将解锁并自主航行, 确认现场安全? 输入 yes: ")
                if ans.strip().lower() != "yes":
                    return 0
            stick.set_mode(args.mode)
            time.sleep(0.3)
            if not stick.arm(force=args.force_arm):
                return 2

        t0 = time.monotonic()
        last = t0
        while True:
            now = time.monotonic()
            el = now - t0
            if el >= args.seconds and phase != "STOP":
                reason = "TIMEOUT"; phase = "STOP"; emergency = True
            drain_mavlink()
            dt = max(1e-3, now - last)
            last = now

            depth, drate = est.depth, est.depth_rate
            ux = uz = ur = 0.0

            # ---- 全局安全 ----
            if not est.valid(now):
                stick.send_neutral(); reason = "DEPTH_STALE"
                _log(rows, el, phase, s, depth, yaw_deg, ux, uz, ur, reason)
                _pace(now, dt_nom); continue
            if phase != "STOP" and (depth > dmax or depth < -0.5):
                reason = "DEPTH_LIMIT"; phase = "STOP"; emergency = True
            if phase != "STOP" and (abs(roll_deg) > args.tilt_abort or abs(pitch_deg) > args.tilt_abort):
                reason = f"TILT({roll_deg:.0f},{pitch_deg:.0f})"; phase = "STOP"; emergency = True

            # ---- 深度与航向: 除"紧急停"外全程保持 (含正常到达后的稳停) ----
            if not (phase == "STOP" and emergency):
                uz = pid.compute(args.depth, depth, drate, dt)
                ur = hh.update(yaw_deg, dt)

            # ---- 阶段逻辑 ----
            if phase == "DESCEND":
                if abs(args.depth - depth) <= args.depth_tol:
                    hold_cnt += 1
                    if hold_cnt >= int(0.7 * hz):
                        phase = "TURN"; hold_cnt = 0
                else:
                    hold_cnt = 0
                reason = "DESCEND"

            elif phase == "TURN":
                if hh.at_target(yaw_deg):
                    hold_cnt += 1
                    if hold_cnt >= int(0.7 * hz):
                        phase = "CRUISE"; hold_cnt = 0; cruise_t0 = now
                else:
                    hold_cnt = 0
                reason = "TURN"

            elif phase == "CRUISE":
                if not dvl.is_fresh(max_age_s=max(max_age, 0.6)):
                    reason = "DVL_LOST"; phase = "STOP"; emergency = True
                else:
                    s += dvl.latest().vx * args.vx_sign * dt   # 航位推算
                    if s >= args.dist:
                        phase = "STOP"; reason = "REACHED"
                    else:
                        # 前馈: 按 cruise 时刻查 u_ff; 表尾则用巡航段前馈保持
                        idx = min(int((now - cruise_t0) / dt_nom), len(u_ff) - 1)
                        ux = u_ff[idx]
                        reason = "CRUISE"

            # STOP: 前进=0。emergency→全中位; 正常到达→保持深度/航向, 直到速度停或稳停超时
            if phase == "STOP":
                if stop_t0 is None:
                    stop_t0 = now
                if emergency:
                    stick.send_neutral()
                else:  # 正常到达: 继续保持深度+航向, 只切断前进
                    stick.send(x=0.0, z=deadzone_comp(uz, args.deadzone),
                               r=deadzone_comp(ur, args.deadzone))
                v = dvl.latest().vx * args.vx_sign if dvl.is_fresh(0.6, require_valid=False) else 0.0
                _log(rows, el, phase, s, depth, yaw_deg, 0, uz if not emergency else 0,
                     ur if not emergency else 0, reason)
                settled = abs(v) < args.v_eps
                timed_out = (now - stop_t0) > args.stop_settle
                if settled or timed_out:
                    tag = "" if settled else f" (稳停超时, 残余v={v:+.2f}m/s, 多因水流/无位置控)"
                    print(f"[wp] DONE  reason={reason}{tag}  已走 s={s:.2f}m/{args.dist}m  "
                          f"depth={depth:+.2f}m yaw={yaw_deg:.1f}°/{target_heading:.1f}°")
                    return 0 if reason == "REACHED" else 3
                _pace(now, dt_nom); continue

            # 死区补偿 + 发送
            ux = deadzone_comp(ux, args.deadzone)
            ur = deadzone_comp(ur, args.deadzone)
            uz = deadzone_comp(uz, args.deadzone)
            stick.send(x=ux, z=uz, r=ur)
            if int(el * hz) % int(hz) == 0:
                stick.send_gcs_heartbeat()

            _log(rows, el, phase, s, depth, yaw_deg, ux, uz, ur, reason)
            if int(el * 2) != int((el - dt_nom) * 2):
                print(f"  t={el:5.1f} [{phase:7s}] s={s:5.2f} depth={depth:+.2f} "
                      f"yaw={yaw_deg:+.1f}/{target_heading:.0f} "
                      f"ux={ux:+.2f} uz={uz:+.2f} ur={ur:+.2f} {reason}")
            _pace(now, dt_nom)

    except KeyboardInterrupt:
        print("\n[wp] 用户中断 → 中位+上锁")
        return 130
    finally:
        dvl.stop()
        stick.close()
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "phase", "s", "depth", "yaw", "ux", "uz", "ur", "reason"])
            w.writerows(rows)
        print(f"[wp] 已保存 {csv_path} ({len(rows)} 行); 已安全退出 (中位+自动上锁)")


def _log(rows, el, phase, s, depth, yaw, ux, uz, ur, reason):
    rows.append([round(el, 3), phase, round(s, 3), round(depth, 4),
                 round(yaw or 0, 2), round(ux, 4), round(uz, 4), round(ur, 4), reason])


def _pace(now, dt_nom):
    sleep = dt_nom - (time.monotonic() - now)
    if sleep > 0:
        time.sleep(sleep)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="W5 相对航点 4DOF 航行 (Point-and-go)")
    p.add_argument("--heading", type=float, default=0.0, help="目标航向 (度, 绝对; --rel 则相对起始)")
    p.add_argument("--rel", action="store_true", help="heading 按相对起始航向解释")
    p.add_argument("--dist", type=float, default=3.0, help="前进距离 (m)")
    p.add_argument("--depth", type=float, default=0.5, help="目标深度 (m, 向下为正)")
    p.add_argument("--v-cruise", type=float, default=0.4)
    p.add_argument("--a-max", type=float, default=0.15)
    p.add_argument("--depth-tol", type=float, default=0.1)
    p.add_argument("--heading-tol", type=float, default=3.0)
    p.add_argument("--v-eps", type=float, default=0.03, help="判停的速度阈值 (m/s)")
    p.add_argument("--stop-settle", type=float, default=3.0, help="稳停最长等待 (s, 超时也 DONE)")
    p.add_argument("--yaw-kp", type=float, default=1.0)
    p.add_argument("--yaw-kd", type=float, default=0.2)
    p.add_argument("--r-limit", type=float, default=0.3)
    p.add_argument("--tilt-abort", type=float, default=30.0, help="roll/pitch 超此角度即停 (度)")
    p.add_argument("--deadzone", type=float, default=0.0, help="ESC 死区补偿 (真机 ~0.3, 仿真 0)")
    p.add_argument("--vx-sign", type=float, default=1.0, help="DVL vx 前进符号 (W1 确认, 反则 -1)")
    p.add_argument("--dvl-ip", default=None)
    p.add_argument("--dvl-port", type=int, default=None)
    p.add_argument("--endpoint", default=None)
    p.add_argument("--seconds", type=float, default=90.0, help="总超时 (s)")
    p.add_argument("--arm", action="store_true")
    p.add_argument("--force-arm", action="store_true")
    p.add_argument("--mode", default="MANUAL")
    p.add_argument("--yes", action="store_true")
    p.add_argument("--umax", type=float, default=None, help="覆盖 U_MAX (真机顶死区用)")
    p.add_argument("--label", default="run")
    return run(p.parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
