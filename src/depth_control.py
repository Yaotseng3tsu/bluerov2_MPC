#!/usr/bin/env python3
"""深度闭环运行器(P4/P5 通用框架)。

把连接/解锁(pseudo_stick)+ 状态估计(state)+ 控制器(PID/MPC)+ 看门狗 + CSV 记录
串成一个闭环。控制器只需实现 reset() 和 compute(setpoint, depth, depth_rate, dt)->u。

安全:
  - 控制器侧看门狗:深度遥测超龄(> state.max_age_s)→ 发中位,不跑控制器。
  - 复用 pseudo_stick 的限幅 / 解锁 / 退出自动上锁 / GCS 心跳。
  - 深度软限位(DEPTH_MIN/MAX)越界 → 中位并停。

用法(对 SITL 或真机):
  python -m src.depth_control --controller pid --d1 0.5 --d2 1.0 --t-step 12 --seconds 30 --arm --yes
  # 真机加 --arm(会解锁);仿真也需 --arm(SITL 未解锁不动)。
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

from src.pseudo_stick import CONFIG_PATH, PseudoStick, load_config  # noqa: E402
from src.state import DepthEstimator  # noqa: E402
from src.link import parse_depth  # noqa: E402

DATA_DIR = Path(__file__).resolve().parent.parent / "data"


def build_controller(name: str, cfg: dict):
    if name == "pid":
        from src.pid import PID
        return PID.from_config(cfg)
    if name == "mpc":
        from src.mpc import MPCDepth  # P5
        return MPCDepth.from_config(cfg)
    raise ValueError(f"未知控制器 {name}")


def make_setpoint(args):
    def sp(t):
        return args.d1 if t < args.t_step else args.d2
    return sp


def run(args) -> int:
    cfg = load_config()
    hz = float(cfg.get("control", {}).get("CTRL_HZ", 10))
    dt_nom = 1.0 / hz
    safety = cfg.get("safety", {})
    dmin = float(safety.get("DEPTH_MIN", 0.0))
    dmax = float(safety.get("DEPTH_MAX", 3.0))
    depth_msg = cfg["connection"].get("depth_message", "GLOBAL_POSITION_INT")

    ctrl = build_controller(args.controller, cfg)
    ctrl.reset()
    est = DepthEstimator.from_config()
    setpoint = make_setpoint(args)

    stick = PseudoStick(cfg, endpoint=args.endpoint)
    from pymavlink import mavutil
    conn = stick.conn

    DATA_DIR.mkdir(exist_ok=True)
    stamp = args.tag or args.controller
    csv_path = DATA_DIR / f"depth_{stamp}_{args.label}.csv"

    rows = []
    try:
        if not stick.wait_heartbeat(float(cfg["connection"].get("heartbeat_timeout_s", 10))):
            return 1
        # 请求深度遥测
        msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{depth_msg}", None)
        if msg_id is not None:
            conn.mav.command_long_send(conn.target_system, conn.target_component,
                                       mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                       float(msg_id), 1e6 / max(hz, 10), 0, 0, 0, 0, 0)

        # 短预热:收到足够深度帧初始化 KF 即停(尽量减少未控上浮)
        t_warm = time.monotonic() + 1.5
        n_fed = 0
        while time.monotonic() < t_warm:
            m = conn.recv_match(blocking=True, timeout=0.3)
            if m is not None:
                d = parse_depth(m)
                if d is not None:
                    est.feed(d[0], time.monotonic())
                    n_fed += 1
            if est.kf.initialized and n_fed >= 5:
                break
        if not est.kf.initialized:
            print("[ctrl] ❌ 预热阶段未收到深度,退出。")
            return 1

        if args.arm:
            if not args.yes:
                ans = input("[ctrl] ⚠ 即将解锁并闭环控制深度,确认现场安全? 输入 yes: ")
                if ans.strip().lower() != "yes":
                    return 0
            stick.set_mode(args.mode)
            time.sleep(0.3)
            if not stick.arm(force=args.force_arm):
                return 2

        print(f"[ctrl] 控制器={args.controller}  hz={hz}  时长={args.seconds}s  "
              f"目标 {args.d1}→{args.d2}@{args.t_step}s  → {csv_path.name}")

        t0 = time.monotonic()
        last = t0
        wd_events = 0
        while True:
            now = time.monotonic()
            el = now - t0
            if el >= args.seconds:
                break

            # 吸收所有深度帧
            while True:
                m = conn.recv_match(blocking=False)
                if m is None:
                    break
                d = parse_depth(m)
                if d is not None:
                    est.feed(d[0], time.monotonic())

            now = time.monotonic()
            sp = setpoint(el)
            depth, rate = est.depth, est.depth_rate

            # --- 控制器侧看门狗:深度超龄 → 中位 ---
            if not est.valid(now):
                stick.send_neutral()
                wd_events += 1
                u = 0.0
                reason = "STALE"
            # --- 深度软限位(安全界:冲出水面 或 过深)---
            elif depth < -0.5 or depth > dmax:
                stick.send_neutral()
                u = 0.0
                reason = "LIMIT"
            else:
                dt = max(1e-3, now - last)
                u = ctrl.compute(sp, depth, rate, dt)
                stick.send(z=u)
                reason = "OK"
            last = now

            if int(el * hz) % int(hz) == 0:
                stick.send_gcs_heartbeat()

            rows.append([round(el, 3), round(sp, 3), round(depth, 4),
                         round(rate, 4), round(u, 4), reason])
            if int(el * 2) != int((el - dt_nom) * 2):
                print(f"  t={el:5.1f} sp={sp:.2f} z={depth:+.3f} rate={rate:+.3f} "
                      f"u={u:+.3f} {reason}")

            # 维持节拍
            sleep = dt_nom - (time.monotonic() - now)
            if sleep > 0:
                time.sleep(sleep)

        print(f"[ctrl] 结束。看门狗触发 {wd_events} 次。")
        return 0
    except KeyboardInterrupt:
        print("\n[ctrl] 用户中断 → 中位+上锁")
        return 130
    finally:
        stick.close()
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "setpoint", "depth", "depth_rate", "u", "status"])
            w.writerows(rows)
        print(f"[ctrl] 已保存 {csv_path} ({len(rows)} 行);已安全退出(中位+自动上锁)")


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="深度闭环运行器 (PID/MPC)")
    p.add_argument("--controller", default="pid", choices=["pid", "mpc"])
    p.add_argument("--endpoint", default=None)
    p.add_argument("--d1", type=float, default=0.5, help="阶跃前目标深度")
    p.add_argument("--d2", type=float, default=1.0, help="阶跃后目标深度")
    p.add_argument("--t-step", type=float, default=12.0, help="阶跃时刻 (s)")
    p.add_argument("--seconds", type=float, default=30.0)
    p.add_argument("--arm", action="store_true")
    p.add_argument("--force-arm", action="store_true")
    p.add_argument("--mode", default="MANUAL")
    p.add_argument("--yes", action="store_true")
    p.add_argument("--tag", default=None, help="CSV 前缀(默认=控制器名)")
    p.add_argument("--label", default="run", help="CSV 标签")
    return run(p.parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
