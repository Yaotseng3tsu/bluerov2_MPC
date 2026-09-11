#!/usr/bin/env python3
"""电机不转诊断:解锁并持续发 z 指令,同时读回 armed/mode、SERVO_OUTPUT_RAW、STATUSTEXT。

判读:
  - SERVO_OUTPUT_RAW 的垂直通道若从 ~1500 变化 → 飞控【确实在输出 PWM】,
    电机不转 = 下游(ESC/电源/接线/推力方向)问题。
  - SERVO 一直 1500 不变 → 飞控【没在输出】= 模式/解锁/指令未被接受问题;
    看 HEARTBEAT 是否持续 ARMED、mode 是否 MANUAL,以及 STATUSTEXT 报什么。

用法:
  python -m src.diag_motor --u 0.3 --seconds 4          # 发 z=0.3 观察
  python -m src.diag_motor --u 0.3 --axis x --seconds 4 # 换前进轴
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
from pymavlink import mavutil  # noqa: E402

MODE = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
        7: "CIRCLE", 9: "SURFACE", 16: "POSHOLD", 19: "MANUAL"}


def main(argv=None) -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--u", type=float, default=0.3)
    p.add_argument("--axis", default="z", choices=list("xyzr"))
    p.add_argument("--seconds", type=float, default=4.0)
    p.add_argument("--force-arm", action="store_true")
    p.add_argument("--yes", action="store_true")
    args = p.parse_args(argv)

    cfg = load_config()
    hz = float(cfg.get("control", {}).get("CTRL_HZ", 10))
    stick = PseudoStick(cfg)
    conn = stick.conn
    try:
        if not stick.wait_heartbeat(10):
            return 1
        # 请求 SERVO_OUTPUT_RAW @ 5Hz
        sid = mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW
        conn.mav.command_long_send(conn.target_system, conn.target_component,
                                   mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                   float(sid), 2e5, 0, 0, 0, 0, 0)
        if not args.yes and input("⚠ 出水/远离桨叶?输入 yes: ").strip().lower() != "yes":
            return 0
        stick.set_mode("MANUAL"); time.sleep(0.3)
        if not stick.arm(force=args.force_arm):
            return 2
        stick.start_keepalive(hz)
        stick.set_cmd(**{args.axis: args.u})
        print(f"\n持续发 {args.axis}={args.u},观察 {args.seconds}s ...\n")

        hb_sources = {}
        t_end = time.monotonic() + args.seconds
        while time.monotonic() < t_end:
            m = conn.recv_match(blocking=True, timeout=0.5)
            if m is None:
                continue
            t = m.get_type()
            if t == "HEARTBEAT":
                key = (m.get_srcSystem(), m.get_srcComponent())
                hb_sources.setdefault(key, {"ap": m.autopilot, "type": m.type})
                if m.get_srcSystem() == conn.target_system:
                    armed = bool(m.base_mode & 128)
                    print(f"  HB sys{m.get_srcSystem()}/comp{m.get_srcComponent()} "
                          f"armed={armed} mode={MODE.get(m.custom_mode, m.custom_mode)}")
            elif t == "SERVO_OUTPUT_RAW":
                s = [getattr(m, f"servo{i}_raw") for i in range(1, 9)]
                print(f"  SERVO {s}")
            elif t == "STATUSTEXT":
                txt = m.text.decode() if isinstance(m.text, bytes) else m.text
                print(f"  >>> STATUSTEXT: {txt}")
        print("\n=== 心跳源汇总(autopilot=3 是飞控;其它=GCS/路由/扩展)===")
        for (s, c), info in sorted(hb_sources.items()):
            tag = "← 飞控" if info["ap"] == 3 else "← 非飞控(可能在抢控制)"
            print(f"  sys{s}/comp{c}  autopilot={info['ap']} type={info['type']}  {tag}")
        return 0
    finally:
        stick.close()
        print("[diag] 已中位+上锁退出")


if __name__ == "__main__":
    raise SystemExit(main())
