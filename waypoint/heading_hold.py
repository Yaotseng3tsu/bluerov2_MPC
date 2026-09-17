#!/usr/bin/env python3
"""W4 — 航向保持 (heading hold)。

锁定目标航向 yaw*, 用 P/PD 输出归一化 yaw 指令 r∈[-r_limit, r_limit]。
用于 go_waypoint 的"转向"阶段, 以及"前进"阶段抑制侧偏 (走直线)。
航向来自 ArduSub 融合的 ATTITUDE.yaw (不用磁力计, rl_logger 报告已证其不可靠)。

关键: 角度误差 wrap 到 [-180,180], 走最短转向 (例 170°→-170° 只转 +20°, 不转 -340°)。

本文件含一个**仅用于离线自检**的轻量 yaw 动力学 (YawPlant, 占位参数),
真机/SITL 不用它。控制器本身 (HeadingHold) 不依赖任何 plant。

自检 (离线):
  python -m waypoint.heading_hold --start 0 --target 90
  python -m waypoint.heading_hold --start 170 --target -170   # 验证最短转向 wrap
  python -m waypoint.heading_hold --target 90 --plot          # 出图到 waypoint/data/
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass


def wrap_deg(a: float) -> float:
    """把角度 wrap 到 (-180, 180]。"""
    a = (a + 180.0) % 360.0 - 180.0
    return a + 360.0 if a <= -180.0 else a


# ------------------------- 控制器 (真机/SITL/离线通用) -------------------------
class HeadingHold:
    """航向 PD 控制器。误差与微分均在 wrap 后的角度上计算, 输出归一化 r。"""

    def __init__(self, kp: float = 1.0, kd: float = 0.2,
                 r_limit: float = 0.3, tol_deg: float = 3.0,
                 ki: float = 0.0, i_limit: float = 0.3):
        self.kp = kp                 # per rad
        self.kd = kd                 # per (rad/s)
        self.ki = ki                 # per (rad·s);对抗恒定扰动力矩(如垂直推力的反扭矩)
        self.i_limit = i_limit       # |ki*∫e| 上限
        self.r_limit = r_limit
        self.tol_deg = tol_deg
        self.target_deg = 0.0
        self._prev_err_rad: float | None = None
        self._integ = 0.0

    def reset(self, target_deg: float) -> None:
        self.target_deg = wrap_deg(target_deg)
        self._prev_err_rad = None
        self._integ = 0.0

    def error_deg(self, yaw_deg: float) -> float:
        return wrap_deg(self.target_deg - yaw_deg)

    def at_target(self, yaw_deg: float) -> bool:
        return abs(self.error_deg(yaw_deg)) <= self.tol_deg

    def update(self, yaw_deg: float, dt: float) -> float:
        """给当前 yaw(度) 与步长, 返回归一化 r 指令 (限幅后)。"""
        err = math.radians(self.error_deg(yaw_deg))
        derr = 0.0 if (self._prev_err_rad is None or dt <= 0) \
            else (err - self._prev_err_rad) / dt
        self._prev_err_rad = err
        # 试探性积分(抗恒定扰动: 负浮力下垂直推力常年产生偏航反扭矩, 纯 PD 必留稳态误差)
        integ_try = self._integ + err * dt
        if self.ki > 0:
            cap = self.i_limit / self.ki
            integ_try = max(-cap, min(cap, integ_try))
        r_unsat = self.kp * err + self.ki * integ_try + self.kd * derr
        r = max(-self.r_limit, min(self.r_limit, r_unsat))
        # 条件积分抗饱和: 饱和且继续朝同向积分会加剧饱和 → 本步冻结
        if not (r != r_unsat and err * r_unsat > 0):
            self._integ = integ_try
        return r


# ------------------------- 仅离线自检用的 yaw 动力学 -------------------------
@dataclass
class YawParams:
    inertia: float = 0.5      # kg·m^2, 转动惯量(含附加), 占位
    c_lin: float = 1.0        # 线性阻尼
    c_quad: float = 2.0       # 二次阻尼
    K_r_Nm: float = 5.0       # r=1 时的偏航力矩 (Nm), 占位


class YawPlant:
    """RK4 一维偏航动力学 (仅离线演示 HeadingHold 收敛)。状态 (yaw_deg, rate_rad_s)。"""

    def __init__(self, p: YawParams, yaw_deg: float = 0.0):
        self.p = p
        self.yaw = math.radians(yaw_deg)
        self.rate = 0.0

    def _deriv(self, yaw: float, rate: float, r: float):
        p = self.p
        r = max(-1.0, min(1.0, r))
        M = p.K_r_Nm * r - p.c_lin * rate - p.c_quad * rate * abs(rate)
        return rate, M / p.inertia

    def step(self, r: float, dt: float) -> float:
        yaw, rate = self.yaw, self.rate
        k1a, k1w = self._deriv(yaw, rate, r)
        k2a, k2w = self._deriv(yaw + 0.5 * dt * k1a, rate + 0.5 * dt * k1w, r)
        k3a, k3w = self._deriv(yaw + 0.5 * dt * k2a, rate + 0.5 * dt * k2w, r)
        k4a, k4w = self._deriv(yaw + dt * k3a, rate + dt * k3w, r)
        self.yaw = yaw + dt / 6.0 * (k1a + 2 * k2a + 2 * k3a + k4a)
        self.rate = rate + dt / 6.0 * (k1w + 2 * k2w + 2 * k3w + k4w)
        return math.degrees(self.yaw)


# ------------------------- 离线自检 -------------------------
def selfcheck(start: float, target: float, dt: float,
              kp: float, kd: float, r_limit: float, tol_deg: float,
              plot: bool) -> int:
    ctrl = HeadingHold(kp=kp, kd=kd, r_limit=r_limit, tol_deg=tol_deg)
    ctrl.reset(target)
    plant = YawPlant(YawParams(), yaw_deg=start)
    print(f"[hh] start={wrap_deg(start):.1f}° target={ctrl.target_deg:.1f}° "
          f"初始误差(最短)={ctrl.error_deg(start):+.1f}°  "
          f"kp={kp} kd={kd} r_lim={r_limit} tol=±{tol_deg}°")

    T = 30.0
    n = int(T / dt)
    ts, yaws, rs, errs = [], [], [], []
    settle_t = None
    hold = 0
    yaw = wrap_deg(start)
    for i in range(n):
        r = ctrl.update(yaw, dt)
        yaw = wrap_deg(plant.step(r, dt))
        t = i * dt
        ts.append(t); yaws.append(yaw); rs.append(r); errs.append(ctrl.error_deg(yaw))
        # 连续 1s 在容差内视为到位
        hold = hold + 1 if ctrl.at_target(yaw) else 0
        if settle_t is None and hold >= int(1.0 / dt):
            settle_t = t
    final_err = ctrl.error_deg(yaw)
    print(f"[hh] 终值 yaw={yaw:.2f}°  终误差={final_err:+.2f}°  "
          f"到位时间(连续1s内容差)={'%.2fs' % settle_t if settle_t else '未到位'}")
    ok = abs(final_err) <= tol_deg
    print(f"[hh] {'PASS' if ok else 'FAIL'} 收敛到目标航向 (容差±{tol_deg}°)")

    if plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("[plot] 未装 matplotlib, 跳过"); return 0 if ok else 1
        out = Path(__file__).resolve().parent / "data"; out.mkdir(exist_ok=True)
        fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        ax[0].plot(ts, yaws, "b-", label="yaw")
        ax[0].axhline(ctrl.target_deg, color="k", ls=":", label="target")
        ax[0].fill_between(ts, ctrl.target_deg - tol_deg, ctrl.target_deg + tol_deg,
                           color="g", alpha=0.15, label=f"±{tol_deg}°")
        ax[0].set_ylabel("yaw (deg)"); ax[0].legend(); ax[0].grid(True)
        ax[1].plot(ts, rs, "m-", label="r cmd")
        ax[1].axhline(r_limit, color="gray", ls=":"); ax[1].axhline(-r_limit, color="gray", ls=":")
        ax[1].set_ylabel("r"); ax[1].set_xlabel("t (s)"); ax[1].legend(); ax[1].grid(True)
        fig.suptitle(f"heading_hold selfcheck  {wrap_deg(start):.0f}°→{ctrl.target_deg:.0f}°")
        png = out / "heading_hold_selftest.png"
        fig.savefig(png, dpi=110, bbox_inches="tight")
        print(f"[plot] 已保存 {png}")
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="W4 航向保持 (离线自检)")
    ap.add_argument("--start", type=float, default=0.0, help="初始 yaw (度)")
    ap.add_argument("--target", type=float, default=90.0, help="目标 yaw (度)")
    ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--kp", type=float, default=1.0)
    ap.add_argument("--kd", type=float, default=0.2)
    ap.add_argument("--r-limit", type=float, default=0.3)
    ap.add_argument("--tol-deg", type=float, default=3.0)
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args(argv)
    return selfcheck(args.start, args.target, args.dt, args.kp, args.kd,
                     args.r_limit, args.tol_deg, args.plot)


if __name__ == "__main__":
    raise SystemExit(main())
