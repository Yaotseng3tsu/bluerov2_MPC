#!/usr/bin/env python3
"""W3 — surge 运动模型 + 前馈梯形速度轨迹 (全离线)。

两块内容:
  1) SurgePlant : 水平前向 (surge) 一维 RK4 动力学 (照搬 src/plant.py 结构,去掉浮力项)。
       (m - X_udot)·v_dot = K_x·u - c_lin·v - c_quad·v·|v|
  2) 轨迹生成 : 给定目标距离 → 梯形速度剖面 (加速—匀速—减速,末端零速),
       再用逆动力学求前馈指令 u_ff(t)。轨迹逻辑写成 DOF 无关,将来可复用于 sway。

参数默认读 config/surge_model.yaml (identified 优先, 否则 sim_truth 占位)。
W2 辨识回填 identified 后, 本模块自动切到真机参数。

自检 (离线, 不连硬件):
  python -m waypoint.motion_model --dist 3.0 --v-cruise 0.4 --a-max 0.15
  python -m waypoint.motion_model --dist 3.0 --plot   # 需 matplotlib, 出图到 waypoint/data/
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

CONFIG_PATH = Path(__file__).resolve().parent / "config" / "surge_model.yaml"


# ------------------------- 动力学 -------------------------
@dataclass
class SurgeParams:
    eff_mass: float        # kg, 有效质量 (m - X_udot)
    c_lin: float           # N/(m/s), 线性阻尼 (正)
    c_quad: float          # N/(m/s)^2, 二次阻尼 (正)
    K_thrust_N: float      # N, u=1 时的前向推力
    u_max: float = 1.0

    @staticmethod
    def from_yaml(path: Path = CONFIG_PATH, section: str = "surge") -> "SurgeParams":
        """section='surge' → 优先 identified, 缺则回退 sim_truth; 'sway' → sway 段。"""
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f)
        if section == "sway":
            src = d.get("sway") or {}
            if src.get("eff_mass") is None:
                raise ValueError("sway 段未辨识 (本期预留),不能用于仿真")
        else:
            ident = d.get("identified") or {}
            src = ident if ident.get("eff_mass") is not None else d["sim_truth"]
        return SurgeParams(
            eff_mass=float(src["eff_mass"]),
            c_lin=float(src["c_lin"]),
            c_quad=float(src["c_quad"]),
            K_thrust_N=float(src["K_thrust_N"]),
            u_max=float(src.get("u_max", 1.0)),
        )

    def using_identified(self, path: Path = CONFIG_PATH) -> bool:
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            ident = (yaml.safe_load(f).get("identified") or {})
        return ident.get("eff_mass") is not None


class SurgePlant:
    """RK4 积分的一维 surge 动力学。状态 (s, v)。"""

    def __init__(self, params: SurgeParams, s0: float = 0.0, v0: float = 0.0):
        self.p = params
        self.s = float(s0)
        self.v = float(v0)
        self.ext_force = 0.0   # 外部扰动力 (N, 前向为正), 如水流

    def _deriv(self, s: float, v: float, u: float):
        p = self.p
        u = max(-p.u_max, min(p.u_max, u))
        F = p.K_thrust_N * u + self.ext_force - p.c_lin * v - p.c_quad * v * abs(v)
        return v, F / p.eff_mass

    def step(self, u: float, dt: float) -> tuple[float, float]:
        s, v = self.s, self.v
        k1s, k1v = self._deriv(s, v, u)
        k2s, k2v = self._deriv(s + 0.5 * dt * k1s, v + 0.5 * dt * k1v, u)
        k3s, k3v = self._deriv(s + 0.5 * dt * k2s, v + 0.5 * dt * k2v, u)
        k4s, k4v = self._deriv(s + dt * k3s, v + dt * k3v, u)
        self.s = s + dt / 6.0 * (k1s + 2 * k2s + 2 * k3s + k4s)
        self.v = v + dt / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v)
        return self.s, self.v


# ------------------------- 梯形速度轨迹 (DOF 无关) -------------------------
@dataclass
class TrapezoidTraj:
    """梯形速度剖面: 加速(a_max)—匀速(v_cruise)—减速(-a_max), 末端零速。

    dist>0 沿正方向。t/v_ref/a_ref 为等间隔 dt 的参考序列, 供逆动力学求前馈。
    """
    dt: float
    t: list[float]
    v_ref: list[float]
    a_ref: list[float]
    total_time: float
    peak_v: float
    triangular: bool   # True=距离太短未达 v_cruise (退化为三角形)

    @staticmethod
    def build(dist: float, v_cruise: float, a_max: float, dt: float = 0.1) -> "TrapezoidTraj":
        assert dist > 0 and v_cruise > 0 and a_max > 0 and dt > 0
        t_acc = v_cruise / a_max
        d_acc = 0.5 * a_max * t_acc ** 2            # 单侧加速段位移
        triangular = (2 * d_acc >= dist)
        if triangular:
            peak_v = (dist * a_max) ** 0.5          # 三角形峰值速度
            t_acc = peak_v / a_max
            t_cruise = 0.0
            total = 2 * t_acc
        else:
            peak_v = v_cruise
            t_cruise = (dist - 2 * d_acc) / v_cruise
            total = 2 * t_acc + t_cruise

        t_list, v_list, a_list = [], [], []
        n = int(round(total / dt)) + 1
        for i in range(n):
            tau = i * dt
            if tau < t_acc:                          # 加速
                a, v = a_max, a_max * tau
            elif tau < t_acc + t_cruise:             # 匀速
                a, v = 0.0, peak_v
            elif tau <= total + 1e-9:                # 减速
                td = tau - (t_acc + t_cruise)
                a, v = -a_max, max(0.0, peak_v - a_max * td)
            else:
                a, v = 0.0, 0.0
            t_list.append(tau)
            v_list.append(v)
            a_list.append(a)
        return TrapezoidTraj(dt, t_list, v_list, a_list, total, peak_v, triangular)

    def planned_distance(self) -> float:
        """对参考速度做梯形积分, 校验剖面本身覆盖的距离。"""
        s = 0.0
        for i in range(1, len(self.v_ref)):
            s += 0.5 * (self.v_ref[i] + self.v_ref[i - 1]) * self.dt
        return s


def feedforward_cmd(p: SurgeParams, v_ref: float, a_ref: float) -> float:
    """逆动力学前馈: 让 surge 跟踪 (v_ref, a_ref) 所需的归一化指令 u。"""
    F = p.eff_mass * a_ref + p.c_lin * v_ref + p.c_quad * v_ref * abs(v_ref)
    return max(-p.u_max, min(p.u_max, F / p.K_thrust_N))


def plan_feedforward(p: SurgeParams, dist: float, v_cruise: float,
                     a_max: float, dt: float = 0.1):
    """给定距离与限制 → (轨迹, 前馈指令序列 u_ff)。"""
    traj = TrapezoidTraj.build(dist, v_cruise, a_max, dt)
    u_ff = [feedforward_cmd(p, v, a) for v, a in zip(traj.v_ref, traj.a_ref)]
    return traj, u_ff


# ------------------------- 离线自检 -------------------------
def selfcheck(dist: float, v_cruise: float, a_max: float, dt: float,
              plot: bool) -> int:
    p = SurgeParams.from_yaml()
    ident = SurgeParams.from_yaml().using_identified()
    print(f"[model] 参数来源 = {'identified(真机)' if ident else 'sim_truth(占位)'}  "
          f"eff_mass={p.eff_mass} c_lin={p.c_lin} c_quad={p.c_quad} K={p.K_thrust_N}")

    traj, u_ff = plan_feedforward(p, dist, v_cruise, a_max, dt)
    kind = "三角形(距离短未达巡航)" if traj.triangular else "梯形"
    print(f"[traj] {kind}  目标={dist:.2f}m  峰值v={traj.peak_v:.3f}m/s  "
          f"总时长={traj.total_time:.2f}s  剖面积分距离={traj.planned_distance():.3f}m")
    umax_ff = max(abs(u) for u in u_ff)
    print(f"[ff]   前馈指令 |u| 峰值={umax_ff:.3f}  (须 ≤ {p.u_max} 且顶过 ESC 死区~0.3)")

    # 用真值模型跑前馈, 看开环能否走到目标 (零反馈开环验证)
    plant = SurgePlant(p)
    ss, vv = [], []
    for u in u_ff:
        s, v = plant.step(u, dt)
        ss.append(s); vv.append(v)
    print(f"[sim]  开环终点位移={ss[-1]:.3f}m (目标{dist:.2f})  末端速度={vv[-1]:+.3f}m/s")
    err = ss[-1] - dist
    print(f"[sim]  距离误差={err:+.3f}m ({err/dist*100:+.1f}%)  "
          f"→ 前馈参数=真值时误差应≈0; 真机模型失配/水流会放大, 故 W5 用 DVL 航位推算兜底")

    if plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("[plot] 未装 matplotlib, 跳过出图")
            return 0
        out = Path(__file__).resolve().parent / "data"
        out.mkdir(exist_ok=True)
        fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
        ax[0].plot(traj.t, traj.v_ref, "b-", label="v_ref")
        ax[0].plot(traj.t, vv, "r--", label="v_sim")
        ax[0].set_ylabel("v (m/s)"); ax[0].legend(); ax[0].grid(True)
        ax[1].plot(traj.t, ss, "g-", label="s_sim")
        ax[1].axhline(dist, color="k", ls=":", label="target")
        ax[1].set_ylabel("s (m)"); ax[1].legend(); ax[1].grid(True)
        ax[2].plot(traj.t, u_ff, "m-", label="u_ff")
        ax[2].axhline(0.3, color="gray", ls=":"); ax[2].axhline(-0.3, color="gray", ls=":")
        ax[2].set_ylabel("u_x"); ax[2].set_xlabel("t (s)"); ax[2].legend(); ax[2].grid(True)
        fig.suptitle(f"surge feedforward selfcheck  dist={dist}m v={v_cruise} a={a_max}")
        png = out / "motion_model_selftest.png"
        fig.savefig(png, dpi=110, bbox_inches="tight")
        print(f"[plot] 已保存 {png}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="W3 surge 运动模型 + 前馈轨迹 (离线自检)")
    ap.add_argument("--dist", type=float, default=3.0, help="目标距离 (m)")
    ap.add_argument("--v-cruise", type=float, default=0.4, help="巡航速度 (m/s)")
    ap.add_argument("--a-max", type=float, default=0.15, help="加/减速度 (m/s^2)")
    ap.add_argument("--dt", type=float, default=0.1, help="轨迹步长 (s)")
    ap.add_argument("--plot", action="store_true", help="出图到 waypoint/data/")
    args = ap.parse_args(argv)
    return selfcheck(args.dist, args.v_cruise, args.a_max, args.dt, args.plot)


if __name__ == "__main__":
    raise SystemExit(main())
