#!/usr/bin/env python3
"""共用:连接并**锁定真正的飞控心跳**(忽略 BlueOS 服务组件 / GCS)。

网络上除飞控外还有 BlueOS 的板载服务(常见 comp191/194, autopilot=8=INVALID)以及
GCS 心跳。朴素的 wait_heartbeat() 会抓到第一个到达的心跳, 可能把 target 锁到
sys=0 或某个非飞控组件 → 之后读参数/发指令全部失败。

判据(与 src/pseudo_stick.py 一致): autopilot != MAV_AUTOPILOT_INVALID 且 type != MAV_TYPE_GCS。
"""
from __future__ import annotations

import time

from pymavlink import mavutil


def connect_fc(endpoint: str = "udpin:0.0.0.0:14550", timeout_s: float = 12.0,
               verbose: bool = True):
    """返回已把 target_system/component 锁定到真飞控的连接; 失败返回 None。"""
    m = mavutil.mavlink
    conn = mavutil.mavlink_connection(endpoint, dialect="ardupilotmega")
    if verbose:
        print(f"[link] 等待飞控 heartbeat ({endpoint}) ...")
    t_end = time.monotonic() + timeout_s
    seen = {}
    while time.monotonic() < t_end:
        hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if hb is None:
            continue
        src = (hb.get_srcSystem(), hb.get_srcComponent())
        seen.setdefault(src, (hb.autopilot, hb.type))
        if hb.autopilot != m.MAV_AUTOPILOT_INVALID and hb.type != m.MAV_TYPE_GCS:
            conn.target_system = hb.get_srcSystem()
            conn.target_component = hb.get_srcComponent()
            if verbose:
                armed = bool(hb.base_mode & 128)
                print(f"[link] 飞控 OK  sys={conn.target_system} comp={conn.target_component} "
                      f"autopilot={hb.autopilot} type={hb.type} "
                      f"{'ARMED' if armed else 'DISARMED'}")
            return conn
    if verbose:
        print("[link] ❌ 超时:未收到真飞控 heartbeat。收到过的心跳源:")
        for (s, c), (ap, ty) in sorted(seen.items()):
            print(f"        sys{s}/comp{c}  autopilot={ap} type={ty}"
                  f"{'  ← 非飞控' if ap == m.MAV_AUTOPILOT_INVALID else ''}")
    return None
