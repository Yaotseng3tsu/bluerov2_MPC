#!/usr/bin/env python3
"""离线仿真件 — Waypoint 4DOF SITL (MAVLink + 假 DVL 一体)。

在 tests/sim_vehicle.py (深度 SITL) 基础上扩出水平面, 模拟一台"假 ArduSub":
  - 接收 MANUAL_CONTROL, 解出 u_x(前进)/u_z(深度)/u_r(偏航)
  - surge 用 waypoint/motion_model.SurgePlant, depth 用 src/plant.DepthPlant,
    yaw 用 waypoint/heading_hold.YawPlant 积分
  - MAVLink 回传: HEARTBEAT(1Hz) + GLOBAL_POSITION_INT(深度,10Hz) + ATTITUDE(yaw,20Hz)
  - 同时起 FakeDvl(127.0.0.1:16171), vx=SurgePlant 机体速度 → go_waypoint 能同时收 MAVLink+DVL

网络约定与 tests/sim_vehicle.py 一致 (双绑定 UDP, Windows 友好)。
sway 未建模 (本期 4DOF), vy≈0; roll/pitch 恒 0 (被动稳定)。

用法 (离线):
  python -m waypoint.sim.sim_waypoint
  # 另开终端跑 W5: python -m waypoint.go_waypoint --heading 90 --dist 3 ...
"""
from __future__ import annotations

import argparse
import errno
import math
import os
import random as _random
import socket
import sys
import time

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from pymavlink.dialects.v20 import ardupilotmega as mav2  # noqa: E402
from src.plant import DepthParams, DepthPlant  # noqa: E402
from waypoint.motion_model import SurgeParams, SurgePlant  # noqa: E402
from waypoint.heading_hold import YawParams, YawPlant, wrap_deg  # noqa: E402
from waypoint.sim.fake_dvl import FakeDvl  # noqa: E402

MAV_TYPE_SUBMARINE = 12
MAV_AUTOPILOT_ARDUPILOTMEGA = 3
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_STATE_STANDBY = 3
MAV_STATE_ACTIVE = 4
MANUAL_MODE = 19
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_DO_SET_MODE = 176
MAV_RESULT_ACCEPTED = 0


class _Writer:
    def __init__(self, sock: socket.socket, dest):
        self.sock = sock
        self.dest = dest

    def write(self, buf):
        try:
            self.sock.sendto(buf, self.dest)
        except OSError:
            pass


def main() -> int:
    p = argparse.ArgumentParser(description="Waypoint 4DOF SITL (MAVLink + 假 DVL)")
    p.add_argument("--bind-port", type=int, default=14551)
    p.add_argument("--ctrl-addr", default="127.0.0.1:14550")
    p.add_argument("--dvl-port", type=int, default=16171)
    p.add_argument("--seconds", type=float, default=0.0)
    p.add_argument("--cmd-timeout", type=float, default=1.5)
    p.add_argument("--z0", type=float, default=0.0, help="初始深度 (m)")
    p.add_argument("--yaw0", type=float, default=0.0, help="初始航向 (度)")
    p.add_argument("--current-vx", type=float, default=0.0,
                   help="水流对 surge 的恒定扰动力 (N, 前向为正), 测鲁棒性")
    p.add_argument("--dvl-bias", type=float, default=1.0,
                   help="DVL vx 相对真值的比例偏差 (1.0=无偏; 测模型失配)")
    p.add_argument("--no-dvl", action="store_true", help="不启假 DVL (测 DVL 失效降级)")
    p.add_argument("--alt-glitch", type=float, default=0.0, dest="alt_glitch",
                   help="注入 DVL 高度跳变的比例 (跳到 2.5m), 复现实测外点")
    p.add_argument("--yaw-shift-at", type=float, default=0.0, dest="yaw_shift_at",
                   help="在该时刻(s)起, 给回传航向叠加**持续**偏移(模拟 EKF 重对准)")
    p.add_argument("--yaw-shift-deg", type=float, default=105.0, dest="yaw_shift_deg")
    p.add_argument("--yaw-glitch", type=float, default=0.0, dest="yaw_glitch",
                   help="注入 ATTITUDE 航向跳变的比例 (瞬时 +58°), 复现实测 EKF 跳变")
    p.add_argument("--dvl-dropout", type=float, default=0.0, dest="dvl_dropout",
                   help="注入 DVL 无解帧的比例 (真机实测 ~0.025), 用于验证容错")
    p.add_argument("--bottom", type=float, default=2.0,
                   help="池底所在深度 (m); DVL 高度 = bottom - depth")
    p.add_argument("--net-buoy", type=float, default=None,
                   help="覆盖净浮力 N (负浮力用正值, 下潜为正系);模拟重力>浮力")
    args = p.parse_args()

    host, port = args.ctrl_addr.split(":")
    dest = (host, int(port))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.bind_port))
    sock.setblocking(False)
    mav = mav2.MAVLink(_Writer(sock, dest), srcSystem=1, srcComponent=1)

    _rnd = _random.Random(1234)
    surge = SurgePlant(SurgeParams.from_yaml())
    _dp = DepthParams.from_yaml()
    if args.net_buoy is not None:
        _dp.net_buoy_N = float(args.net_buoy)   # 正=下沉(重力>浮力)
    depth = DepthPlant(_dp, z0=args.z0)
    yaw = YawPlant(YawParams(), yaw_deg=args.yaw0)

    state = {"armed": False, "mode": MANUAL_MODE}

    # 假 DVL: vx = surge 机体速度 (可加偏差模拟失配); 仿真恒有底锁
    dvl = None
    if not args.no_dvl:
        def vel():
            # 真机 DVL 底锁与是否解锁无关: 仿真恒有效 (贴底)
            # 高度 = 池底深度 - 当前深度 (随垂直运动变化, 供 altitude_hold 离线验证)
            alt = max(0.05, args.bottom - depth.z)
            if args.alt_glitch > 0 and _rnd.random() < args.alt_glitch:
                return {"vx": surge.v * args.dvl_bias, "vy": 0.0, "vz": depth.w,
                        "valid": True, "altitude": 2.5}     # 外点
            if args.dvl_dropout > 0 and _rnd.random() < args.dvl_dropout:
                # 模拟 A50 间歇解算失败: valid=false, altitude=-1
                return {"vx": 0.0, "vy": 0.0, "vz": 0.0, "valid": False, "altitude": -1.0}
            return {"vx": surge.v * args.dvl_bias, "vy": 0.0, "vz": depth.w,
                    "valid": True, "altitude": alt}
        dvl = FakeDvl(vel, port=args.dvl_port, rate_hz=10.0).start()

    def send_heartbeat():
        base = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if state["armed"]:
            base |= MAV_MODE_FLAG_SAFETY_ARMED
        st = MAV_STATE_ACTIVE if state["armed"] else MAV_STATE_STANDBY
        mav.heartbeat_send(MAV_TYPE_SUBMARINE, MAV_AUTOPILOT_ARDUPILOTMEGA,
                           base, state["mode"], st)

    print(f"[sitl] bind :{args.bind_port} → {dest}  DVL={'off' if args.no_dvl else args.dvl_port}"
          f"  DISARMED/MANUAL  yaw0={args.yaw0}°")
    print(f"[sitl] surge: eff_mass={surge.p.eff_mass} K={surge.p.K_thrust_N}N"
          f"  current={args.current_vx}N  dvl_bias={args.dvl_bias}")

    sim_hz = 50.0
    dt = 1.0 / sim_hz
    t0 = time.monotonic()
    last_hb = last_pos = last_att = last_log = 0.0
    last_cmd_t = -1e9
    ux = uz = ur = 0.0
    send_heartbeat()

    try:
        while True:
            now = time.monotonic()
            el = now - t0
            if args.seconds > 0 and el >= args.seconds:
                break

            while True:
                try:
                    data, _ = sock.recvfrom(4096)
                except OSError as e:
                    if e.errno in (errno.EWOULDBLOCK, errno.EAGAIN, 10035):
                        break
                    break
                if not data:
                    break
                try:
                    msgs = mav.parse_buffer(data) or []
                except Exception:
                    msgs = []
                for m in msgs:
                    t = m.get_type()
                    if t == "MANUAL_CONTROL":
                        ux = max(-1.0, min(1.0, m.x / 1000.0))
                        # 真机实测极性(2026-09-17): MANUAL_CONTROL z 通道 >500 = 上浮。
                        # 配置里 sign_z=-1 负责补偿, 故仿真此处同样取负, 端到端才与真机一致。
                        uz = max(-1.0, min(1.0, -(m.z - 500) / 500.0))
                        ur = max(-1.0, min(1.0, m.r / 1000.0))
                        last_cmd_t = now
                    elif t == "COMMAND_LONG" and m.command == MAV_CMD_COMPONENT_ARM_DISARM:
                        state["armed"] = (m.param1 >= 0.5)
                        print(f"[sitl] {'ARM' if state['armed'] else 'DISARM'} → ACK")
                        mav.command_ack_send(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED)
                        send_heartbeat()
                    elif t == "COMMAND_LONG" and m.command == MAV_CMD_DO_SET_MODE:
                        state["mode"] = int(m.param2) if m.param2 else MANUAL_MODE
                        mav.command_ack_send(MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED)
                        send_heartbeat()
                    elif t == "SET_MODE":
                        state["mode"] = m.custom_mode
                        send_heartbeat()

            if now - last_cmd_t > args.cmd_timeout:
                ux = uz = ur = 0.0  # 仿真 failsafe
            armed = state["armed"]
            surge.ext_force = args.current_vx if armed else 0.0
            surge.step(ux if armed else 0.0, dt)
            depth.step(uz if armed else 0.0, dt)
            yaw.step(ur if armed else 0.0, dt)

            if now - last_hb >= 1.0:
                last_hb = now
                send_heartbeat()
            if now - last_pos >= 0.1:
                last_pos = now
                mav.global_position_int_send(int(el * 1000), 356800000, 1396000000,
                                             0, int(-depth.z * 1000), 0, 0, 0,
                                             int(wrap_deg(math.degrees(yaw.yaw)) * 100) % 36000)
            if now - last_att >= 0.05:
                last_att = now
                yw = math.atan2(math.sin(yaw.yaw), math.cos(yaw.yaw))  # wrap ±pi
                if args.yaw_shift_at > 0 and el >= args.yaw_shift_at:
                    # 持续参考系平移: 陀螺 yawspeed 不变(物理没转), 只有角度整体偏
                    yw = math.atan2(math.sin(yaw.yaw + math.radians(args.yaw_shift_deg)),
                                    math.cos(yaw.yaw + math.radians(args.yaw_shift_deg)))
                if args.yaw_glitch > 0 and _rnd.random() < args.yaw_glitch:
                    yw = math.atan2(math.sin(yaw.yaw + math.radians(58)),
                                    math.cos(yaw.yaw + math.radians(58)))  # 假跳变, 陀螺不变
                mav.attitude_send(int(el * 1000), 0.0, 0.0, yw, 0.0, 0.0, yaw.rate)
            if now - last_log >= 0.5:
                last_log = now
                a = "ARMED" if armed else "DISARM"
                print(f"[sitl] t={el:5.1f}s {a} ux={ux if armed else 0:+.2f} "
                      f"uz={uz if armed else 0:+.2f} ur={ur if armed else 0:+.2f} | "
                      f"v={surge.v:+.3f}m/s depth={depth.z:+.2f}m yaw={wrap_deg(math.degrees(yaw.yaw)):+.1f}°")

            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        if dvl:
            dvl.stop()
        print("[sitl] 结束。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
