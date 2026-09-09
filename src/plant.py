#!/usr/bin/env python3
"""BlueROV2 深度 (heave) 动力学模型。

坐标约定:z = 深度,【向下为正】;w = dz/dt (下潜速度为正)。
一维纵向 (heave) 简化模型 (Fossen 形式的单自由度):

    (m - Z_wdot) * w_dot = F_thrust + F_net_buoy - c_lin*w - c_quad*w*|w|

其中:
  - m - Z_wdot   : 有效质量 (含附加质量),eff_mass
  - F_thrust     : 垂直推力,= K_thrust_N * u,u∈[-1,1] 归一化指令 (+u 下潜)
  - F_net_buoy   : 剩余浮力在【下潜为正】系下的投影 (W-B);略正浮时为负(趋于上浮)
  - c_lin, c_quad: 线性 / 二次阻尼系数 (正,耗散)

该类既作 SITL 的"真值"对象,也作 MPC 的内部预测模型 (P5 复用)。
参数默认从 config/depth_model.yaml 读取,可被显式覆盖 (MPC 用辨识后的参数)。
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "depth_model.yaml"


@dataclass
class DepthParams:
    eff_mass: float        # kg, 有效质量 (m - Z_wdot)
    c_lin: float           # N/(m/s), 线性阻尼 (正)
    c_quad: float          # N/(m/s)^2, 二次阻尼 (正)
    K_thrust_N: float      # N,  u=1 时的垂直推力
    net_buoy_N: float      # N,  剩余浮力 (下潜为正系;略正浮为负)
    u_max: float = 1.0     # 归一化指令饱和

    @staticmethod
    def from_yaml(path: Path = CONFIG_PATH) -> "DepthParams":
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f)["sim_truth"]
        return DepthParams(
            eff_mass=float(d["eff_mass"]),
            c_lin=float(d["c_lin"]),
            c_quad=float(d["c_quad"]),
            K_thrust_N=float(d["K_thrust_N"]),
            net_buoy_N=float(d["net_buoy_N"]),
            u_max=float(d.get("u_max", 1.0)),
        )


class DepthPlant:
    """RK4 积分的一维深度动力学。状态 (z, w)。"""

    def __init__(self, params: DepthParams, z0: float = 0.0, w0: float = 0.0):
        self.p = params
        self.z = float(z0)
        self.w = float(w0)
        self.ext_force = 0.0   # 外部扰动力 (N,下潜为正),如缆线拉力/水流

    def _deriv(self, z: float, w: float, u: float):
        p = self.p
        u = max(-p.u_max, min(p.u_max, u))
        F = (p.K_thrust_N * u + p.net_buoy_N + self.ext_force
             - p.c_lin * w - p.c_quad * w * abs(w))
        return w, F / p.eff_mass

    def step(self, u: float, dt: float) -> tuple[float, float]:
        """用零阶保持的 u 积分 dt 秒,返回 (z, w)。"""
        z, w = self.z, self.w
        k1z, k1w = self._deriv(z, w, u)
        k2z, k2w = self._deriv(z + 0.5 * dt * k1z, w + 0.5 * dt * k1w, u)
        k3z, k3w = self._deriv(z + 0.5 * dt * k2z, w + 0.5 * dt * k2w, u)
        k4z, k4w = self._deriv(z + dt * k3z, w + dt * k3w, u)
        self.z = z + dt / 6.0 * (k1z + 2 * k2z + 2 * k3z + k4z)
        self.w = w + dt / 6.0 * (k1w + 2 * k2w + 2 * k3w + k4w)
        return self.z, self.w


if __name__ == "__main__":
    # 快速自检:施加恒定下潜指令,观察是否稳定下潜并趋于终速
    p = DepthParams.from_yaml()
    plant = DepthPlant(p)
    dt = 0.02
    print("t(s)  z(m)   w(m/s)   u")
    for i in range(int(6 / dt) + 1):
        u = 0.3
        z, w = plant.step(u, dt)
        if i % 25 == 0:
            print(f"{i*dt:4.1f}  {z:6.3f}  {w:6.3f}   {u}")
