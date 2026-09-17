#!/usr/bin/env python3
"""Phase 2 — 深度状态估计 (2 阶卡尔曼滤波)。

从含噪深度测量估计观测 (depth, depth_rate),供 P4 PID / P5 MPC 使用。
模型:恒速 (constant-velocity),状态 x=[z, v] (z 深度向下为正,v=dz/dt)。

  预测:  x' = F x,  P' = F P Fᵀ + Q
         F = [[1, dt],[0, 1]]
         Q = q * [[dt³/3, dt²/2],[dt²/2, dt]]   (白噪声加速度模型, q=process_accel_sigma²)
  更新:  z_meas = H x + r,  H=[1,0],  R = meas_sigma²

附带:深度龄 (age) 与 valid 标志 —— P4 控制器看门狗据此在深度超龄时回中位。

可独立运行 (对着 SITL 打印估计):
  python tests/sim_vehicle.py --depth-noise 0.03
  python -m src.state --seconds 15
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "vehicle.yaml"


class KalmanDepth:
    """2 状态 (深度, 速率) 恒速卡尔曼滤波,支持变步长 dt。"""

    def __init__(self, meas_sigma: float, process_accel_sigma: float):
        self.R = float(meas_sigma) ** 2
        self.q = float(process_accel_sigma) ** 2
        # 状态与协方差 (未初始化)
        self.z = 0.0
        self.v = 0.0
        self.P = [[1e3, 0.0], [0.0, 1e3]]
        self.initialized = False

    def reset(self, z0: float = 0.0, v0: float = 0.0):
        self.z, self.v = z0, v0
        self.P = [[1e3, 0.0], [0.0, 1e3]]
        self.initialized = False

    def predict(self, dt: float) -> None:
        if dt <= 0:
            return
        dt = min(dt, 1.0)  # 防止长间隔导致协方差爆炸
        # x' = F x
        self.z = self.z + dt * self.v
        # P' = F P Fᵀ + Q
        p00, p01 = self.P[0]
        p10, p11 = self.P[1]
        # F P
        a00 = p00 + dt * p10
        a01 = p01 + dt * p11
        a10 = p10
        a11 = p11
        # (F P) Fᵀ
        fp00 = a00 + dt * a01
        fp01 = a01
        fp10 = a10 + dt * a11
        fp11 = a11
        q = self.q
        dt2, dt3 = dt * dt, dt * dt * dt
        Q00 = q * dt3 / 3.0
        Q01 = q * dt2 / 2.0
        Q11 = q * dt
        self.P = [[fp00 + Q00, fp01 + Q01],
                  [fp10 + Q01, fp11 + Q11]]

    def update(self, z_meas: float) -> None:
        if not self.initialized:
            self.z, self.v = z_meas, 0.0
            self.P = [[self.R, 0.0], [0.0, 1.0]]
            self.initialized = True
            return
        # y = z_meas - H x ; S = H P Hᵀ + R = P00 + R
        y = z_meas - self.z
        S = self.P[0][0] + self.R
        # K = P Hᵀ / S = [P00, P10] / S
        k0 = self.P[0][0] / S
        k1 = self.P[1][0] / S
        # x = x + K y
        self.z += k0 * y
        self.v += k1 * y
        # P = (I - K H) P
        p00, p01 = self.P[0]
        p10, p11 = self.P[1]
        self.P = [[(1 - k0) * p00, (1 - k0) * p01],
                  [p10 - k1 * p00, p11 - k1 * p01]]


class DepthEstimator:
    """带时间戳与超龄检测的深度估计器封装。"""

    def __init__(self, meas_sigma: float, process_accel_sigma: float,
                 max_age_s: float = 0.5):
        self.kf = KalmanDepth(meas_sigma, process_accel_sigma)
        self.max_age_s = float(max_age_s)
        self.last_t = None

    @classmethod
    def from_config(cls, path: Path = CONFIG_PATH) -> "DepthEstimator":
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            s = yaml.safe_load(f).get("state", {})
        return cls(
            meas_sigma=float(s.get("meas_sigma", 0.03)),
            process_accel_sigma=float(s.get("process_accel_sigma", 0.5)),
            max_age_s=float(s.get("max_age_s", 0.5)),
        )

    def feed(self, z_meas: float, t: float) -> None:
        """输入一帧深度测量 (t = 单调时钟秒)。"""
        if self.last_t is not None:
            self.kf.predict(t - self.last_t)
        self.kf.update(z_meas)
        self.last_t = t

    @property
    def depth(self) -> float:
        return self.kf.z

    @property
    def depth_rate(self) -> float:
        return self.kf.v

    def age(self, now: float) -> float:
        return float("inf") if self.last_t is None else now - self.last_t

    def valid(self, now: float) -> bool:
        return self.kf.initialized and self.age(now) <= self.max_age_s


# --------- 独立运行:对着 SITL 打印估计 ---------
def _main(argv=None) -> int:
    import time
    import yaml
    from src.link import import_mavutil, parse_depth  # 复用连接与深度解析

    p = argparse.ArgumentParser(description="深度状态估计 (对 SITL/真机)")
    p.add_argument("--endpoint", default=None)
    p.add_argument("--seconds", type=float, default=15.0)
    p.add_argument("--config", default=str(CONFIG_PATH))
    args = p.parse_args(argv)

    cfg = yaml.safe_load(open(args.config, encoding="utf-8"))
    c = cfg["connection"]
    endpoint = args.endpoint or c["endpoint"]
    depth_msg = c.get("depth_message", "GLOBAL_POSITION_INT")

    est = DepthEstimator.from_config(Path(args.config))
    mavutil = import_mavutil()
    conn = mavutil.mavlink_connection(endpoint, dialect=c.get("dialect", "ardupilotmega"),
                                      source_system=255)
    print(f"[state] 连接 {endpoint},等待 heartbeat ...")
    if conn.wait_heartbeat(timeout=float(c.get("heartbeat_timeout_s", 10))) is None:
        print("[state] ❌ 无 heartbeat")
        return 1
    msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{depth_msg}", None)
    if msg_id is not None:
        conn.mav.command_long_send(conn.target_system, conn.target_component,
                                   mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                   float(msg_id), 1e5, 0, 0, 0, 0, 0)  # 10 Hz

    t0 = time.monotonic()
    last_print = 0.0
    while time.monotonic() - t0 < args.seconds:
        m = conn.recv_match(blocking=True, timeout=1.0)
        now = time.monotonic()
        if m is None:
            continue
        d = parse_depth(m, depth_msg)
        if d is not None:
            est.feed(d[0], now)
            if now - last_print >= 0.5:
                last_print = now
                print(f"  meas={d[0]:+.3f}  z_kf={est.depth:+.3f}  "
                      f"rate_kf={est.depth_rate:+.3f} m/s  "
                      f"age={est.age(now)*1000:4.0f}ms  valid={est.valid(now)}")
    conn.close()
    print("[state] 完成。")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
