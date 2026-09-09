#!/usr/bin/env python3
"""BlueROV2 深度 SITL —— 闭环 MAVLink 仿真 (软件在环)。

模拟一台"假 ArduSub":
  - 接收上位机的 MANUAL_CONTROL,取 z 通道 → 归一化 u∈[-1,1] (z∈[0,1000], 500=中位, +u 下潜)
  - 用 src/plant.py 的深度动力学积分
  - 以 MAVLink 回传 HEARTBEAT (1Hz) + GLOBAL_POSITION_INT (深度, 10Hz)

网络 (Windows 友好):
  SITL 自绑定 UDP 端口 (默认 14551),向控制器 (默认 127.0.0.1:14550) 主动推遥测;
  控制器用 udpin:0.0.0.0:14550 (与真机一致),收到遥测后把指令回发到 14551。
  两端都 bind,规避 pymavlink udpout 在 Windows 上 recvfrom 的 WSAEINVAL 问题。

模式:
  默认 (闭环):  python tests/sim_vehicle.py
  P0 演示 (正弦, 忽略指令):  python tests/sim_vehicle.py --demo

约定:z 深度向下为正。
"""
from __future__ import annotations

import argparse
import errno
import math
import os
import socket
import sys
import time

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pymavlink.dialects.v20 import ardupilotmega as mav2  # noqa: E402
from src.plant import DepthParams, DepthPlant  # noqa: E402

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

# 仿真用的失效保护参数(仅供 check_params.py 离线联调;数值为示意)
SIM_PARAMS = {
    "FS_PILOT_INPUT": 2,      # 0禁用 1警告 2 disarm
    "FS_PILOT_TIMEOUT": 3.0,  # s
    "FS_GCS_ENABLE": 1,
    "FS_LEAK_ENABLE": 1,
    "FS_LEAK_ACTION": 1,
    "FS_CRASH_CHECK": 1,
    "FS_EKF_ACTION": 1,
    "FS_EKF_THRESH": 0.8,
    "FS_BATT_ENABLE": 0,
    "BATT_LOW_VOLT": 0.0,
}


def z_to_u(z_channel: int) -> float:
    """MANUAL_CONTROL z (0..1000, 500 中位) -> u∈[-1,1], +u 下潜。"""
    return max(-1.0, min(1.0, (z_channel - 500) / 500.0))


class _Writer:
    """给 mavlink2.MAVLink 用的 file-like 写出口,固定发往控制器地址。"""

    def __init__(self, sock: socket.socket, dest):
        self.sock = sock
        self.dest = dest

    def write(self, buf):
        try:
            self.sock.sendto(buf, self.dest)
        except OSError:
            pass


def main() -> int:
    p = argparse.ArgumentParser(description="BlueROV2 深度 SITL")
    p.add_argument("--bind-port", type=int, default=14551, help="SITL 自身绑定端口")
    p.add_argument("--ctrl-addr", default="127.0.0.1:14550",
                   help="控制器监听地址 (遥测发往此处)")
    p.add_argument("--seconds", type=float, default=0.0, help="运行时长,0=直到 Ctrl+C")
    p.add_argument("--demo", action="store_true",
                   help="P0 演示:忽略指令,深度按正弦变化")
    p.add_argument("--period", type=float, default=8.0, help="demo 正弦周期 (s)")
    p.add_argument("--depth-amp", type=float, default=0.5, help="demo 深度振幅 (m)")
    p.add_argument("--cmd-timeout", type=float, default=1.5,
                   help="超过该秒数未收到指令则 u=0 (仿真 failsafe)")
    p.add_argument("--z0", type=float, default=0.0, help="初始深度 (m)")
    p.add_argument("--depth-noise", type=float, default=0.0,
                   help="回传深度上叠加的高斯噪声 std (m),模拟压力计噪声")
    p.add_argument("--noise-seed", type=int, default=12345, help="噪声随机种子")
    p.add_argument("--disturb-force", type=float, default=0.0,
                   help="扰动力 N (下潜为正,恒定),模拟缆线拉力/水流")
    p.add_argument("--disturb-at", type=float, default=1e9, help="扰动起始时刻 (s)")
    p.add_argument("--disturb-dur", type=float, default=1e9, help="扰动持续 (s)")
    args = p.parse_args()

    import random as _random
    rng = _random.Random(args.noise_seed)

    host, port = args.ctrl_addr.split(":")
    dest = (host, int(port))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.bind_port))
    sock.setblocking(False)

    mav = mav2.MAVLink(_Writer(sock, dest), srcSystem=1, srcComponent=1)

    params = DepthParams.from_yaml()
    plant = DepthPlant(params, z0=args.z0)

    mode = "DEMO(正弦,忽略指令)" if args.demo else "闭环(响应 MANUAL_CONTROL)"
    print(f"[sitl] bind :{args.bind_port} → 遥测发往 {dest}。模式={mode}  DISARMED/MANUAL")
    print(f"[sitl] plant: eff_mass={params.eff_mass} c_lin={params.c_lin} "
          f"c_quad={params.c_quad} K={params.K_thrust_N}N net_buoy={params.net_buoy_N}N")

    state = {"armed": False, "mode": MANUAL_MODE}

    def send_heartbeat():
        base = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if state["armed"]:
            base |= MAV_MODE_FLAG_SAFETY_ARMED
        st = MAV_STATE_ACTIVE if state["armed"] else MAV_STATE_STANDBY
        mav.heartbeat_send(MAV_TYPE_SUBMARINE, MAV_AUTOPILOT_ARDUPILOTMEGA,
                           base, state["mode"], st)

    def send_ack(cmd, result=MAV_RESULT_ACCEPTED):
        try:
            mav.command_ack_send(cmd, result)
        except Exception:
            pass

    sim_hz = 50.0
    dt = 1.0 / sim_hz
    t0 = time.monotonic()
    last_hb = last_pos = last_log = 0.0
    last_cmd_t = -1e9
    u = 0.0

    send_heartbeat()  # 先喊一声,让控制器学到本 SITL 地址

    while True:
        now = time.monotonic()
        el = now - t0
        if args.seconds > 0 and el >= args.seconds:
            break

        # --- 接收指令 (吸收所有待处理报文) ---
        while True:
            try:
                data, _addr = sock.recvfrom(4096)
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
                    u = z_to_u(m.z)
                    last_cmd_t = now
                elif t == "COMMAND_LONG" and m.command == MAV_CMD_COMPONENT_ARM_DISARM:
                    state["armed"] = (m.param1 >= 0.5)
                    forced = " (force)" if abs(m.param2 - 21196) < 1 else ""
                    print(f"[sitl] {'ARM' if state['armed'] else 'DISARM'} 指令{forced} → ACK")
                    send_ack(MAV_CMD_COMPONENT_ARM_DISARM)
                    send_heartbeat()
                elif t == "COMMAND_LONG" and m.command == MAV_CMD_DO_SET_MODE:
                    state["mode"] = int(m.param2) if m.param2 else MANUAL_MODE
                    send_ack(MAV_CMD_DO_SET_MODE)
                    send_heartbeat()
                elif t == "SET_MODE":
                    state["mode"] = m.custom_mode
                    send_heartbeat()
                elif t == "PARAM_REQUEST_READ":
                    pid = m.param_id
                    if isinstance(pid, bytes):
                        pid = pid.split(b"\x00")[0].decode(errors="ignore")
                    if pid in SIM_PARAMS:
                        names = list(SIM_PARAMS)
                        mav.param_value_send(pid.encode(), float(SIM_PARAMS[pid]),
                                             9, len(names), names.index(pid))  # type=REAL32
                elif t == "PARAM_REQUEST_LIST":
                    names = list(SIM_PARAMS)
                    for i, k in enumerate(names):
                        mav.param_value_send(k.encode(), float(SIM_PARAMS[k]),
                                             9, len(names), i)

        # --- 积分动力学 ---
        if args.demo:
            depth = args.depth_amp * (1 - math.cos(2 * math.pi * el / args.period))
            plant.z = depth
        else:
            if now - last_cmd_t > args.cmd_timeout:
                u = 0.0  # 仿真 failsafe
            # 真机行为:未 arm 时推进器不转
            u_eff = u if state["armed"] else 0.0
            # 扰动力窗口
            if args.disturb_at <= el < args.disturb_at + args.disturb_dur:
                plant.ext_force = args.disturb_force
            else:
                plant.ext_force = 0.0
            plant.step(u_eff, dt)
            depth = plant.z

        # --- 遥测 ---
        if now - last_hb >= 1.0:
            last_hb = now
            send_heartbeat()
        if now - last_pos >= 0.1:
            last_pos = now
            depth_report = depth
            if args.depth_noise > 0:
                depth_report += rng.gauss(0.0, args.depth_noise)
            rel_alt_mm = int(-depth_report * 1000)  # 水下为负
            mav.global_position_int_send(int(el * 1000), 356800000, 1396000000,
                                         0, rel_alt_mm, 0, 0, 0, 0)
        if now - last_log >= 0.5:
            last_log = now
            a = "ARMED" if (not args.demo and state["armed"]) else ("DEMO" if args.demo else "DISARM")
            u_show = (u if state["armed"] else 0.0) if not args.demo else 0.0
            print(f"[sitl] t={el:5.1f}s  {a}  u={u_show:+.2f}  depth={depth:+.3f}m  w={plant.w:+.3f}m/s")

        time.sleep(dt)

    print("[sitl] 结束。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
