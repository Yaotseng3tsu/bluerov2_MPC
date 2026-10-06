#!/usr/bin/env python3
"""导航状态 —— 把 DVL + 深度计 + 陀螺汇成一组可直接喂控制器的量。

把原先散在 go_forward.py / altitude_hold.py 里的估计逻辑收到一处, 两个脚本吃同一份
实现, 也便于离线回放验证。发布:

  离底高度 alt / 上升率 alt_rate   ← DVL 高度(慢变绝对基准) + 飞控 EKF 垂向速度(快变)
  航向 heading_deg / 角速率        ← 陀螺积分为主, 绝对 yaw 只做慢速对齐 (HeadingTracker)
  前向位移 fwd_m / 速度 vx         ← DVL 逐帧按自带 dt 航位推算 (DvlStream.displacement)

三条链路各自的依据(均出自 2026-10-01 的 ArduSub 4.7.1 源码阅读):

  **高度** —— §6 SURFTRAK 的结构: terrain offset = EKF 深度 − 0.25Hz 低通的 DVL 距离,
  "DVL 距离只作为慢变偏置, 快变部分由 EKF 深度负责"。我们原先快慢都压在 4.5–7.5Hz 的
  DVL 上; 现在把快通道交给 ~26Hz 的 GLOBAL_POSITION_INT。平底水池里
  d(alt)/dt = −d(depth)/dt, 所以 −vz 是 rate 状态的直接量测。
  只用**速率**不用绝对深度 —— §1.2: 水面基准会被 update_calibration() 重新归零,
  绝对深度会阶跃, 速率不受影响(也正好绕开 9/17 那批被 VFR_HUD 污染过的深度)。

  **航向** —— §2.3: ArduSub 自己的角速率 PID 吃 get_gyro_latest()(滤波陀螺 + EKF 零偏修正),
  而 EKF3 的 yaw 源默认是罗盘, 水池磁场不稳会整体跳。见 heading_tracker.py。

  **位移** —— §3.1: BlueOS 的 DVL 扩展算 VISION_POSITION_DELTA 用的是报文自带的
  Δp = v·dt。我们原先在 10Hz 控制环里按标称 dt 重积分一个 4.5–7.5Hz 的零阶保持量,
  同一帧会被数 1.3–2.2 次。

坐标约定: alt 向上为正; depth 向下为正; fwd_m / vx 已乘 vx_sign(前进为正)。
"""
from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

from waypoint.altitude_estimator import AltitudeEstimator  # noqa: E402
from waypoint.heading_tracker import HeadingTracker  # noqa: E402


@dataclass
class NavFix:
    """一个控制周期的导航快照。"""
    t: float = 0.0
    dt: float = 0.1                  # 本周期实测步长 (已钳位)
    # --- 离底高度 ---
    alt: float = 0.0
    alt_rate: float = 0.0            # m/s, + = 远离池底
    alt_ok: bool = False
    alt_reset_delta: float = 0.0     # 本周期高度基准跳变量 (m)
    alt_age_s: float = 0.0           # 距最近一次被接受的高度量测
    # --- 航向 ---
    heading_deg: float | None = None
    heading_rate_dps: float = 0.0
    heading_ok: bool = False
    heading_realign_deg: float = 0.0  # 本周期检测到的 EKF 重对准量
    heading_realign_n: int = 0
    heading_offset_deg: float = 0.0
    # --- 水平航位推算 ---
    fwd_m: float = 0.0               # 累计前向位移 (已乘 vx_sign)
    vx: float = 0.0                  # 最近一帧前向速度 (已乘 vx_sign)
    dvl_age_s: float = 0.0
    dvl_gap_s: float = 0.0           # 累计无解时长
    # --- 深度 (向下为正) ---
    depth: float | None = None
    depth_rate: float | None = None  # m/s, + = 下潜
    depth_age_s: float = float("inf")
    depth_used: bool = False         # 本周期高度 rate 是否由深度计提供


class NavState:
    """汇聚 MAVLink(ATTITUDE / GLOBAL_POSITION_INT) 与 DVL 的导航估计。"""

    def __init__(self, conn, dvl, *,
                 vx_sign: float = 1.0,
                 hz: float = 10.0,
                 use_depth: bool = True,
                 depth_stale_s: float = 1.0,
                 yaw_tau: float = 120.0,
                 yaw_gate_deg: float = 12.0,
                 yaw_accept_n: int = 5,
                 yaw_stale: float = 1.5,
                 est: AltitudeEstimator | None = None,
                 tracker: HeadingTracker | None = None) -> None:
        self.conn = conn
        self.dvl = dvl
        self.vx_sign = float(vx_sign)
        self.dt_nom = 1.0 / float(hz)
        self.use_depth = use_depth
        self.depth_stale_s = depth_stale_s
        self.est = est or AltitudeEstimator()
        self.tracker = tracker or HeadingTracker(
            tau_s=yaw_tau, gate_deg=yaw_gate_deg,
            accept_n=yaw_accept_n, stale_s=yaw_stale)

        self._t_prev = None          # 上一次 update() 时刻
        self._t_alt = None           # 高度滤波已推进到的时刻
        self._alt_seq = -1           # 上次见到的 DVL 高度帧号
        self._boot_off = None        # time_boot_ms → 本地单调钟 的偏移
        self.depth = None
        self.depth_rate = None
        self._t_depth = None
        self.realign_total = 0

    # ---------------- 数据流 ----------------
    def request_streams(self, att_hz: float = 20.0, pos_hz: float = 20.0) -> None:
        """向飞控申请 ATTITUDE 与 GLOBAL_POSITION_INT 的推送频率。"""
        from pymavlink import mavutil as _mv
        for msg_id, rate in ((_mv.mavlink.MAVLINK_MSG_ID_ATTITUDE, att_hz),
                             (_mv.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, pos_hz)):
            if rate <= 0:
                continue
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                _mv.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                float(msg_id), 1e6 / rate, 0, 0, 0, 0, 0)

    def _msg_time(self, msg, now: float) -> float:
        """把报文的 time_boot_ms 映射到本地单调钟。

        同一个控制周期里会一次抽干好几条报文, 它们的到达时刻几乎相同 ——
        若用墙钟当时间戳, 陀螺积分的步长会被压成 0, 航向就不走了。
        用飞控自己的 time_boot_ms 才拿得到正确的帧间隔。
        偏移漂了(重连/飞控重启/时钟跳变)就重新对齐。
        """
        tb = getattr(msg, "time_boot_ms", None)
        if not tb:
            return now
        boot_s = float(tb) / 1000.0
        if self._boot_off is None or abs(boot_s + self._boot_off - now) > 1.0:
            self._boot_off = now - boot_s
        return boot_s + self._boot_off

    def _drain(self, now: float) -> tuple[float, list]:
        """抽干 MAVLink, 喂航向跟踪器, 收集深度样本。

        注意必须**一个循环同时收两种报文** —— pymavlink 的 recv_match 会丢弃不匹配的,
        分两次抽会把对方的报文吃掉。
        """
        realign = 0.0
        depth_samples = []
        while True:
            m = self.conn.recv_match(type=["ATTITUDE", "GLOBAL_POSITION_INT"],
                                     blocking=False)
            if m is None:
                break
            t_m = self._msg_time(m, now)
            if m.get_type() == "ATTITUDE":
                rl = self.tracker.feed(
                    yaw_deg=math.degrees(m.yaw),
                    yawspeed_dps=math.degrees(m.yawspeed),
                    roll_deg=math.degrees(m.roll),
                    pitch_deg=math.degrees(m.pitch),
                    pitchspeed_dps=math.degrees(m.pitchspeed),
                    t=t_m)
                if rl and abs(rl) > abs(realign):
                    realign = rl
            else:  # GLOBAL_POSITION_INT
                depth = -m.relative_alt / 1000.0      # mm, 水下为负 → 深度向下为正
                vz_down = m.vz / 100.0                # cm/s, NED 向下为正
                depth_samples.append((t_m, depth, vz_down))
                self.depth = depth
                self.depth_rate = vz_down
                self._t_depth = now
        return realign, depth_samples

    # ---------------- 主入口 ----------------
    def update(self) -> NavFix:
        now = time.monotonic()
        if self._t_prev is None:
            dt = self.dt_nom
            self._t_alt = now
        else:
            dt = min(max(now - self._t_prev, 0.2 * self.dt_nom), 5.0 * self.dt_nom)
        self._t_prev = now

        realign, depth_samples = self._drain(now)
        if realign:
            self.realign_total += 1

        depth_fresh = (self._t_depth is not None
                       and (now - self._t_depth) <= self.depth_stale_s)
        depth_used = False

        # --- 高度滤波: 按深度样本的真实时序推进, 中间用 -vz 校正 rate ---
        if self.use_depth and depth_samples:
            t_cur = self._t_alt if self._t_alt is not None else now
            for t_m, _d, vz_down in sorted(depth_samples, key=lambda r: r[0]):
                step = min(max(t_m - t_cur, 0.0), 5.0 * self.dt_nom)
                if step > 0:
                    self.est.predict(step)
                if self.est.update_rate(-vz_down, r=self.est.r_vz_ekf):
                    depth_used = True
                t_cur = max(t_cur, t_m)
            tail = min(max(now - t_cur, 0.0), 5.0 * self.dt_nom)
            if tail > 0:
                self.est.predict(tail)
        else:
            self.est.predict(dt)
        self._t_alt = now

        # --- DVL: 新的高度帧 → 位置量测; 深度不可用时退回 DVL vz 当 rate ---
        reset_delta = 0.0
        seq = self.dvl.alt_seq
        sample = self.dvl.latest_valid()
        if seq != self._alt_seq:
            self._alt_seq = seq
            if not depth_used and not (self.use_depth and depth_fresh):
                self.est.update_rate(-sample.vz)      # DVL vz 向下为正 → 取负
            reset_delta = self.est.update_alt(sample.altitude)

        e = self.est.estimate(reset_delta)
        h = self.tracker.estimate(realign, t=now)
        disp = self.dvl.displacement()

        return NavFix(
            t=now, dt=dt,
            alt=e.alt, alt_rate=e.rate, alt_ok=e.initialized,
            alt_reset_delta=reset_delta, alt_age_s=e.age_s,
            heading_deg=h.heading_deg if h.ok else None,
            heading_rate_dps=h.rate_dps, heading_ok=h.ok,
            heading_realign_deg=realign, heading_realign_n=h.realign_n,
            heading_offset_deg=h.offset_deg,
            fwd_m=disp.x * self.vx_sign, vx=sample.vx * self.vx_sign,
            dvl_age_s=sample.age_s, dvl_gap_s=disp.gap_s,
            depth=self.depth, depth_rate=self.depth_rate,
            depth_age_s=(now - self._t_depth) if self._t_depth else float("inf"),
            depth_used=depth_used)

    # ---------------- 解锁前预热 ----------------
    def prime(self, seconds: float = 2.0, need_heading: bool = True) -> NavFix:
        """解锁前先把估计器喂热。

        不预热的话控制头一两个周期会对着未初始化的 0 算出满推(9/17 踩过)。
        顺便等到航向跟踪器也初始化, 这样解锁时锁定的航向是真的。
        """
        t_end = time.monotonic() + seconds
        fix = NavFix()
        while time.monotonic() < t_end:
            fix = self.update()
            if fix.alt_ok and (fix.heading_ok or not need_heading):
                # 再多喂 0.5s 让 KF 的 P 收下来
                t_extra = time.monotonic() + 0.5
                while time.monotonic() < t_extra:
                    fix = self.update()
                    time.sleep(0.02)
                break
            time.sleep(0.02)
        return fix
