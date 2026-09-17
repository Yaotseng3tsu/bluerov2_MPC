#!/usr/bin/env python3
"""z 轴垂直运动(MANUAL 直接推力)—— 上浮 / 下潜,并实时打印深度。

用途:
  1) 负浮力机器人需要主动上浮时的最小可用指令;
  2) **一次定掉 sign_z**:命令"上浮"后看深度是变小(真上浮)还是变大(约定反了)。

约定(config/vehicle.yaml 的 manual_control.sign_z):
  内部 u_z: **+ = 下潜, - = 上浮**;经 _norm_to_ch 换算成 MANUAL_CONTROL 的 z 通道
  (0~1000, 500=中位): z_ch = z_neutral + u_z * 500。

流程: 切 MANUAL → arm(应用当前 JS_GAIN_DEFAULT) → 按 hz 连发 z 指令 → 回中位
      → 交回 --exit-mode(默认 ALT_HOLD) → **默认不 disarm**(负浮力避免沉底)。

安全:
  - 上浮到 --min-depth 以上(更浅)即停,避免冲出水面;
  - 下潜超过 safety.DEPTH_MAX 即停;
  - 深度遥测超龄即停;Ctrl+C 随时中断(同样回中位+交回 ALT_HOLD)。

用法:
  python -m waypoint.z_move --dir up   --u 0.5 --seconds 3
  python -m waypoint.z_move --dir down --u 0.4 --seconds 3
  python -m waypoint.z_move --dir up   --u 0.6 --seconds 3 --umax 1.0 --yes
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

from src.pseudo_stick import PseudoStick, load_config  # noqa: E402
from src.link import parse_depth  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="z 轴上浮/下潜 (MANUAL 直接推力)")
    ap.add_argument("--dir", choices=["up", "down"], default="up", help="上浮 / 下潜")
    ap.add_argument("--u", type=float, default=0.5, help="指令幅度 0~1")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--umax", type=float, default=1.0, help="覆盖 U_MAX")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--min-depth", type=float, default=0.15,
                    help="上浮保护:深度浅于此值即停 (m)")
    ap.add_argument("--exit-mode", default="ALT_HOLD",
                    help="结束时交回的模式 (空字符串=不切)")
    ap.add_argument("--disarm", action="store_true",
                    help="结束时上锁 (默认不上锁,负浮力会沉底)")
    ap.add_argument("--mode", default="MANUAL", help="运行时模式")
    ap.add_argument("--endpoint", default=None)
    ap.add_argument("--yes", action="store_true", help="跳过解锁确认")
    args = ap.parse_args(argv)

    cfg = load_config()
    dmax = float(cfg.get("safety", {}).get("DEPTH_MAX", 3.0))
    depth_msg = cfg["connection"].get("depth_message", "GLOBAL_POSITION_INT")

    # 内部约定: + 下潜 / - 上浮
    u_z = -abs(args.u) if args.dir == "up" else abs(args.u)

    stick = PseudoStick(cfg, endpoint=args.endpoint)
    stick.u_max = float(args.umax)
    conn = stick.conn
    from pymavlink import mavutil

    depth = None
    d_start = None

    def drain():
        nonlocal depth
        while True:
            m = conn.recv_match(blocking=False)
            if m is None:
                break
            d = parse_depth(m, depth_msg)
            if d is not None:
                depth = d[0]

    try:
        if not stick.wait_heartbeat(float(cfg["connection"].get("heartbeat_timeout_s", 10))):
            return 1
        mid = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{depth_msg}", None)
        if mid is not None:
            conn.mav.command_long_send(conn.target_system, conn.target_component,
                                       mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                       float(mid), 1e5, 0, 0, 0, 0, 0)
        # 预热取深度
        t_w = time.monotonic() + 3.0
        while time.monotonic() < t_w and depth is None:
            drain(); time.sleep(0.05)
        if depth is None:
            print("[z] ❌ 收不到深度遥测,退出。")
            return 1
        z_ch = stick._norm_to_ch(u_z, "z")
        print(f"[z] 起始深度={depth:+.3f}m  方向={args.dir}  u_z={u_z:+.2f} "
              f"→ MANUAL_CONTROL z={z_ch} (500=中位)  U_MAX={stick.u_max}")

        if stick.warn_if_rival() and not args.yes:
            if input("[z] 仍要继续? 输入 yes: ").strip().lower() != "yes":
                return 0
        if not args.yes:
            ans = input(f"[z] ⚠ 即将解锁并在 {args.mode} 下{'上浮' if args.dir=='up' else '下潜'}"
                        f" {args.seconds}s,确认安全? 输入 yes: ")
            if ans.strip().lower() != "yes":
                print("[z] 已取消。")
                return 0

        stick.set_mode(args.mode)
        time.sleep(0.3)
        if not stick.arm():
            return 2

        d_start = depth
        t0 = time.monotonic()
        dt = 1.0 / args.hz
        last_print = 0.0
        stop_reason = "完成"
        while time.monotonic() - t0 < args.seconds:
            cyc = time.monotonic()
            drain()
            el = cyc - t0
            # 安全保护
            if depth is not None:
                if args.dir == "up" and depth < args.min_depth:
                    stop_reason = f"上浮保护 (depth={depth:.2f} < {args.min_depth})"
                    break
                if args.dir == "down" and depth > dmax:
                    stop_reason = f"深度上限 (depth={depth:.2f} > {dmax})"
                    break
            stick.send(z=u_z)
            if int(el * args.hz) % int(args.hz) == 0:
                stick.send_gcs_heartbeat()
            if el - last_print >= 0.5:
                last_print = el
                dd = (depth - d_start) if depth is not None else float("nan")
                print(f"  t={el:4.1f}s  depth={depth:+.3f}m  Δ={dd:+.3f}m  z_ch={z_ch}")
            sl = dt - (time.monotonic() - cyc)
            if sl > 0:
                time.sleep(sl)

        drain()
        d_end = depth
        print(f"[z] 结束 ({stop_reason})。深度 {d_start:+.3f} → {d_end:+.3f} m "
              f"(Δ={d_end - d_start:+.3f})")
        # --- sign_z 判读 ---
        delta = d_end - d_start
        if abs(delta) < 0.03:
            print("[z] ⚠ 深度几乎没变 → 推力不足/被卡住,先查增益或加大 --u。")
        elif args.dir == "up":
            if delta < 0:
                print("[z] ✅ 命令上浮 → 深度变小 = 真上浮。**sign_z 正确(保持 +1)**")
            else:
                print("[z] ❌ 命令上浮 → 深度反而变大(下沉)。**sign_z 反了 → config 改 sign_z: -1**")
        else:
            if delta > 0:
                print("[z] ✅ 命令下潜 → 深度变大 = 真下潜。**sign_z 正确(保持 +1)**")
            else:
                print("[z] ❌ 命令下潜 → 深度反而变小(上浮)。**sign_z 反了 → config 改 sign_z: -1**")
        return 0
    except KeyboardInterrupt:
        print("\n[z] 用户中断。")
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
            print(f"[z] 已交回 {args.exit_mode}")
        if not args.disarm:
            stick.armed_by_us = False   # 保持 armed,负浮力不沉底
            print("[z] 保持 armed (未上锁)")
        stick.close()


if __name__ == "__main__":
    raise SystemExit(main())
