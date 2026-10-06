#!/usr/bin/env python3
"""航向跟踪器 —— 陀螺积分为主,绝对航向只做慢速对齐。

动机(2026-09-17 实测 + 2026-10-07 源码对照):
  ArduSub 的 EKF3 默认以**罗盘**为 yaw 源。水池里磁场不稳,会出现 "Yaw realigned"
  事件 —— 角度整体跳一大截,而机器人根本没转。实测到过两种:
    · 瞬时尖刺: t=15.8→16.2s 内 yaw 由 -0.7° 跳到 +57.7°(145°/s),而当时 u_r 只有 0.25
    · 永久平移: t≈20s 起整体偏 105°,再也不回来

  旧做法(go_forward 内联的 drain_attitude)是**建在绝对 yaw 之上再打补丁**:
  用陀螺交叉校验剔除、连续 N 帧就把目标同量平移、再坏下去就干脆停用航向控制。
  能防住事故,但每次重对准都要付出"5 帧检测 + 目标平移",坏到超时还会彻底失去航向权限。

本模块把因果关系反过来 —— **陀螺是基准,罗盘是参考**:
  1) 用 ATTITUDE 的角速率积分维持航向 ψ。陀螺不受 yaw 重置影响(ArduSub 的
     ATTITUDE.yawspeed = 滤波后陀螺 + EKF 零偏修正,见 AP_AHRS_Backend::get_gyro_latest)。
  2) 绝对 yaw 以时间常数 `tau_s` 做**慢速**互补修正,只负责压住陀螺的长期漂移。
  3) 偏差超过 `gate_deg` 且连续 `accept_n` 帧 → 判为 EKF 重对准:
     **只平移"EKF 系→我们系"的偏置 offset,ψ 一动不动**。
     于是 105° 重对准对控制器而言是一次 no-op,航向目标不用动、也不用停用控制。

与旧实现的一个关键简化:门限是**固定角度**,不随 dt 膨胀。
旧的 `allow = (yawspeed + jump_dps) * dt` 两次踩同一个坑 —— 被拒期间 dt 一直变大、
门限跟着涨,约 0.9s 后 108° 的门限就放过了 105° 的跳变(见 PROGRESS "航向跳变剔除第二版")。
这里因为 ψ 本身已由陀螺推进,新息 y 天然扣掉了真实转动,**固定门限才是正确的判据**。

自检(离线,不需要硬件):
  python -m waypoint.heading_tracker
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

from waypoint.heading_hold import wrap_deg  # noqa: E402


@dataclass
class HeadingFix:
    heading_deg: float    # 我们的航向估计(陀螺积分 + 慢速对齐),度
    rate_dps: float       # 欧拉偏航率 dψ/dt,度/s —— 直接给控制器当 D 项
    ok: bool              # 已初始化且 ATTITUDE 不超龄
    realign_deg: float    # 本次检测到的 EKF 重对准量(度);0 = 无
    offset_deg: float     # 当前 "EKF yaw → 我们系" 的偏置
    age_s: float          # 距最近一条 ATTITUDE 多久
    realign_n: int        # 累计重对准次数(诊断用)
    rejected: int         # 当前连续被门控拒绝的绝对航向量测数


class HeadingTracker:
    """陀螺积分 + 绝对航向慢速对齐 + 重对准偏置再基准。"""

    def __init__(self,
                 tau_s: float = 120.0,       # 绝对航向修正时间常数(s);<=0 = 纯陀螺
                 gate_deg: float = 12.0,     # 新息门限(度),固定值,不随 dt 变
                 accept_n: int = 5,          # 连续超门限这么多帧 → 判为重对准
                 stale_s: float = 1.5,       # 多久收不到 ATTITUDE 判为不可用
                 max_rate_dps: float = 360.0,  # 角速率合理上限,超过视为坏帧
                 max_dt: float = 0.2         # 单步积分时长钳位(s)
                 ) -> None:
        self.tau_s = tau_s
        self.gate_deg = gate_deg
        self.accept_n = max(1, accept_n)
        self.stale_s = stale_s
        self.max_rate_dps = max_rate_dps
        self.max_dt = max_dt
        self.reset()

    def reset(self) -> None:
        self.psi = 0.0          # 我们的航向估计(度)
        self.offset = 0.0       # wrap(yaw_ekf + offset) 应当 ≈ psi
        self.rate_dps = 0.0
        self.initialized = False
        self._t_last = None     # 上一条 ATTITUDE 的时刻
        self._bad_n = 0
        self.realign_n = 0

    # ---------------- 喂数据 ----------------
    def feed(self, yaw_deg: float, yawspeed_dps: float,
             roll_deg: float = 0.0, pitch_deg: float = 0.0,
             pitchspeed_dps: float = 0.0, t: float | None = None) -> float:
        """喂一条 ATTITUDE。返回本次检测到的重对准量(度),0 = 无。

        角度单位全部用度;yawspeed/pitchspeed 是**体轴**角速率(ATTITUDE 报文原样)。
        """
        now = time.monotonic() if t is None else t
        if not self.initialized:
            self.psi = wrap_deg(yaw_deg)
            self.offset = 0.0
            self.rate_dps = 0.0
            self.initialized = True
            self._t_last = now
            self._bad_n = 0
            return 0.0

        dt = min(max(now - self._t_last, 1e-4), self.max_dt)
        self._t_last = now

        # --- 1) 陀螺推进 ---
        # 体轴角速率 → 欧拉偏航率: dψ/dt = (q·sinφ + r·cosφ) / cosθ
        # BlueROV2 被动稳定、roll/pitch 接近 0 时 ≈ r;推进时会有几度俯仰,顺手修掉。
        phi = math.radians(roll_deg)
        cth = math.cos(math.radians(pitch_deg))
        if abs(cth) < 0.25:                      # 防除零(接近 ±90° 俯仰,本机不会发生)
            cth = 0.25 if cth >= 0 else -0.25
        psi_dot = (pitchspeed_dps * math.sin(phi) + yawspeed_dps * math.cos(phi)) / cth
        if abs(psi_dot) > self.max_rate_dps:     # 坏帧保护
            psi_dot = math.copysign(self.max_rate_dps, psi_dot)
        self.psi = wrap_deg(self.psi + psi_dot * dt)
        self.rate_dps = psi_dot

        # --- 2) 绝对航向:门控 + 慢速对齐 ---
        z = wrap_deg(yaw_deg + self.offset)      # 把 EKF 的 yaw 换算到我们系
        y = wrap_deg(z - self.psi)               # 新息

        if abs(y) > self.gate_deg:
            self._bad_n += 1
            if self._bad_n < self.accept_n:
                return 0.0                       # 瞬时尖刺: 丢弃, ψ 继续靠陀螺走
            # 持续偏离 = EKF 把航向参考系整体挪了。物理指向没变 →
            # 只把 offset 同量平移,让 z 重新对上 ψ;ψ / 控制目标一概不动。
            self.offset = wrap_deg(self.offset - y)
            self._bad_n = 0
            self.realign_n += 1
            return y

        self._bad_n = 0
        if self.tau_s > 0:
            k = min(1.0, dt / self.tau_s)        # 一阶互补,压住陀螺长期漂移
            self.psi = wrap_deg(self.psi + k * y)
        return 0.0

    # ---------------- 取值 ----------------
    def estimate(self, realign_deg: float = 0.0, t: float | None = None) -> HeadingFix:
        now = time.monotonic() if t is None else t
        age = float("inf") if self._t_last is None else (now - self._t_last)
        return HeadingFix(heading_deg=self.psi, rate_dps=self.rate_dps,
                          ok=self.initialized and age <= self.stale_s,
                          realign_deg=realign_deg, offset_deg=self.offset,
                          age_s=age, realign_n=self.realign_n, rejected=self._bad_n)


# ------------------------- 离线自检 -------------------------
def _selfcheck() -> int:
    """合成数据自检:真实转动 / 尖刺 / 永久平移 / 反复重对准 四种工况。"""
    dt = 0.05                                   # ATTITUDE 20Hz
    fails = []

    def run(name, n, true_rate_fn, corrupt_fn, tau=120.0):
        tr = HeadingTracker(tau_s=tau)
        psi_true = 0.0
        t = 0.0
        err_max = 0.0
        realigns = 0
        for i in range(n):
            r = true_rate_fn(t)                 # 真实偏航角速率 (deg/s)
            psi_true = wrap_deg(psi_true + r * dt)
            yaw_rep = corrupt_fn(t, psi_true)   # 飞控回报的(可能被污染的) yaw
            rl = tr.feed(yaw_rep, r, t=t)
            if rl:
                realigns += 1
            if i > 10:                          # 跳过初始化
                err_max = max(err_max, abs(wrap_deg(tr.psi - psi_true)))
            t += dt
        return tr, err_max, realigns

    # ① 干净: 以 20°/s 转 3s 再停
    tr, e, _ = run("clean", 400, lambda t: 20.0 if t < 3.0 else 0.0, lambda t, p: p)
    ok = e < 0.5
    print(f"{'✅' if ok else '❌'} 干净工况: 最大航向误差 {e:.3f}° (<0.5)")
    if not ok:
        fails.append("clean")

    # ② 瞬时尖刺 5%: +58°,陀螺不变(复现实测)
    import random
    rnd = random.Random(7)
    tr, e, rl = run("spike", 600, lambda t: 0.0,
                    lambda t, p: wrap_deg(p + 58.0) if rnd.random() < 0.05 else p)
    ok = e < 1.0 and rl == 0
    print(f"{'✅' if ok else '❌'} 5% 尖刺(+58°): 最大误差 {e:.3f}° (<1.0), "
          f"误判重对准 {rl} 次 (应为 0)")
    if not ok:
        fails.append("spike")

    # ③ 永久平移 +105° @ t=10s (复现实测 9/17)
    tr, e, rl = run("shift", 600, lambda t: 0.0,
                    lambda t, p: wrap_deg(p + 105.0) if t >= 10.0 else p)
    ok = e < 1.0 and rl == 1
    print(f"{'✅' if ok else '❌'} 永久平移 +105°: 最大误差 {e:.3f}° (<1.0), "
          f"重对准 {rl} 次 (应为 1), 末偏置 {tr.offset:+.1f}°")
    if not ok:
        fails.append("shift")

    # ④ 反复重对准: 每 10s 再跳 +40°(模拟"每下潜 0.5m 用罗盘重置一次"),同时有真实转动
    def rep(t, p):
        return wrap_deg(p + 40.0 * int(t // 10.0))

    tr, e, rl = run("repeat", 1200, lambda t: 5.0 * math.sin(t / 3.0), rep)
    ok = e < 1.0 and rl == 5
    print(f"{'✅' if ok else '❌'} 每 10s 罗盘重置 +40°(叠加真实转动): "
          f"最大误差 {e:.3f}° (<1.0), 重对准 {rl} 次 (应为 5)")
    if not ok:
        fails.append("repeat")

    # ⑤ 纯陀螺(tau<=0)也要能跑, 且 105° 平移完全不影响
    tr, e, rl = run("gyro-only", 600, lambda t: 10.0 if t < 2 else 0.0,
                    lambda t, p: wrap_deg(p + 105.0) if t >= 10.0 else p, tau=0.0)
    ok = e < 0.2
    print(f"{'✅' if ok else '❌'} 纯陀螺 (--yaw-tau 0): 最大误差 {e:.3f}° (<0.2)")
    if not ok:
        fails.append("gyro-only")

    print(f"\n=== {5 - len(fails)}/5 通过 ===")

    # ---- 参考数据(不判定): 罗盘**慢漂**时 tau 的取舍 ----
    # 门限只拦得住跳变; 低于门限的缓慢漂移会被互补滤波跟进去。
    # 这正是 --yaw-tau 的意义: 大 tau = 更信陀螺, 小 tau = 更信罗盘。
    print("\n[参考] 罗盘以 0.5°/s 缓慢漂移(低于门限, 拦不住)时, 60s 末航向误差:")
    for tau in (0.0, 300.0, 120.0, 60.0, 30.0):
        _tr, _e, _ = run("drift", 1200, lambda t: 0.0,
                         lambda t, p: wrap_deg(p + 0.5 * t), tau=tau)
        tag = "纯陀螺" if tau <= 0 else f"tau={tau:.0f}s"
        print(f"       {tag:>9s}: 最大误差 {_e:5.1f}°  "
              f"(罗盘自身此时已漂 30.0°)")
    print("       → 默认 tau=120s: 60s 航程内罗盘漂移只带进约 3°, 同时仍压得住陀螺长期漂移。")
    print("         池内若确认罗盘极差, 可 --yaw-tau 0 走纯陀螺。")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(_selfcheck())
