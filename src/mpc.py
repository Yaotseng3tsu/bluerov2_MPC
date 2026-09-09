#!/usr/bin/env python3
"""深度 MPC 控制器(P5,CasADi/IPOPT)。

滚动时域非线性 MPC,预测模型 = src/plant.py 的深度动力学(下潜为正):
  z_dot = w
  w_dot = (K*u + net_buoy - c_lin*w - c_quad*w*|w|) / m_eff

代价:Σ_k Q_pos*(z-z_ref)² + Q_vel*w² + R*u² + R_du*Δu²  + 终端 Q_pos*(z-z_ref)²
约束:|u| ≤ u_limit(+可选深度约束)。每步用当前 (depth, depth_rate) 作初值,解一次,取首个 u。

接口与 PID 一致:reset() 与 compute(setpoint, depth, depth_rate, dt) -> u。

模型参数:优先用 config/depth_model.yaml 的 `identified`(P3 真机辨识后回填);
          为空时退回 `sim_truth`(当前=SITL 真值,仿真里即"完美模型",真机需 P3 后重估)。
"""
from __future__ import annotations

from pathlib import Path

import casadi as ca

DEPTH_MODEL_PATH = Path(__file__).resolve().parent.parent / "config" / "depth_model.yaml"


def _load_model(path: Path = DEPTH_MODEL_PATH) -> dict:
    import yaml
    d = yaml.safe_load(open(path, encoding="utf-8"))
    ident = d.get("identified") or {}
    truth = d.get("sim_truth", {})
    # 若 identified 任一关键项为空,则整体退回 sim_truth
    keys = ("eff_mass", "c_lin", "c_quad", "K_thrust_N", "net_buoy_N")
    if all(ident.get(k) is not None for k in keys):
        src = ident
        used = "identified"
    else:
        src = truth
        used = "sim_truth"
    return {
        "eff_mass": float(src["eff_mass"]), "c_lin": float(src["c_lin"]),
        "c_quad": float(src["c_quad"]), "K_thrust_N": float(src["K_thrust_N"]),
        "net_buoy_N": float(src["net_buoy_N"]), "_used": used,
    }


class MPCDepth:
    def __init__(self, model: dict, dt=0.1, N=20, Q_pos=10.0, Q_vel=1.0,
                 R=0.1, R_du=0.5, u_limit=0.3, dist_gain=0.2, delay_comp=0.15):
        self.m = model
        self.dt, self.N, self.u_limit = float(dt), int(N), float(u_limit)
        self.dist_gain = float(dist_gain)   # 扰动观测器增益 (0=关闭)
        self.delay_comp = float(delay_comp) # 状态前推补偿的延迟 (s;含估计滞后+传输)
        self._build(Q_pos, Q_vel, R, R_du)
        self.reset()

    @classmethod
    def from_config(cls, cfg: dict) -> "MPCDepth":
        mp = cfg.get("mpc", {})
        model = _load_model()
        print(f"[mpc] 预测模型来源 = {model['_used']}  "
              f"(eff_mass={model['eff_mass']}, K={model['K_thrust_N']}N)")
        return cls(model, dt=mp.get("dt", 0.1), N=mp.get("N", 20),
                   Q_pos=mp.get("Q_pos", 10.0), Q_vel=mp.get("Q_vel", 1.0),
                   R=mp.get("R", 0.1), R_du=mp.get("R_du", 0.5),
                   u_limit=mp.get("u_limit", 0.3), dist_gain=mp.get("dist_gain", 0.2),
                   delay_comp=mp.get("delay_comp", 0.15))

    def _dyn(self, x, u, fhat):
        m = self.m
        z, w = x[0], x[1]
        wdot = (m["K_thrust_N"] * u + m["net_buoy_N"] + fhat
                - m["c_lin"] * w - m["c_quad"] * w * ca.fabs(w)) / m["eff_mass"]
        return ca.vertcat(w, wdot)

    def _rk4(self, x, u, dt, fhat):
        k1 = self._dyn(x, u, fhat)
        k2 = self._dyn(x + 0.5 * dt * k1, u, fhat)
        k3 = self._dyn(x + 0.5 * dt * k2, u, fhat)
        k4 = self._dyn(x + dt * k3, u, fhat)
        return x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

    def _rk4_np(self, x, u, dt, fhat):
        """纯 numpy 的 RK4(用于延迟补偿前推,不进优化图)。"""
        import numpy as np
        m = self.m

        def f(xx):
            z, w = xx[0], xx[1]
            wdot = (m["K_thrust_N"] * u + m["net_buoy_N"] + fhat
                    - m["c_lin"] * w - m["c_quad"] * w * abs(w)) / m["eff_mass"]
            return np.array([w, wdot])
        k1 = f(x); k2 = f(x + 0.5 * dt * k1)
        k3 = f(x + 0.5 * dt * k2); k4 = f(x + dt * k3)
        return x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

    def _build(self, Q_pos, Q_vel, R, R_du):
        N, dt = self.N, self.dt
        opti = ca.Opti()
        X = opti.variable(2, N + 1)
        U = opti.variable(1, N)
        x0 = opti.parameter(2)
        zref = opti.parameter(1)
        uprev = opti.parameter(1)
        fhat = opti.parameter(1)   # 在线估计的恒定扰动力 (offset-free)

        opti.subject_to(X[:, 0] == x0)
        cost = 0
        for k in range(N):
            opti.subject_to(X[:, k + 1] == self._rk4(X[:, k], U[0, k], dt, fhat))
            opti.subject_to(opti.bounded(-self.u_limit, U[0, k], self.u_limit))
            du = U[0, k] - (U[0, k - 1] if k > 0 else uprev)
            cost += (Q_pos * (X[0, k] - zref) ** 2 + Q_vel * X[1, k] ** 2
                     + R * U[0, k] ** 2 + R_du * du ** 2)
        cost += Q_pos * (X[0, N] - zref) ** 2 + Q_vel * X[1, N] ** 2
        opti.minimize(cost)
        opti.solver("ipopt", {"print_time": False, "ipopt.print_level": 0,
                              "ipopt.max_iter": 60, "ipopt.sb": "yes"})
        self.opti, self.X, self.U = opti, X, U
        self.p_x0, self.p_zref, self.p_uprev, self.p_fhat = x0, zref, uprev, fhat

    def reset(self):
        self.u_prev = 0.0
        self.f_hat = 0.0          # 扰动力估计 (N,下潜为正)
        self.w_prev = None
        self._warm_X = None
        self._warm_U = None

    def _update_disturbance(self, w_now, dt):
        """扰动观测器:用上一步预测速度与实测的残差,慢速更新 f_hat。"""
        if self.w_prev is None or dt <= 0:
            return
        m = self.m
        wp = self.w_prev
        w_pred = wp + dt * ((m["K_thrust_N"] * self.u_prev + m["net_buoy_N"] + self.f_hat
                             - m["c_lin"] * wp - m["c_quad"] * wp * abs(wp)) / m["eff_mass"])
        resid = w_now - w_pred                      # 实测比预测多出的速度增量
        L = self.dist_gain                          # 观测器增益
        self.f_hat += L * m["eff_mass"] * resid / dt
        self.f_hat = max(-60.0, min(60.0, self.f_hat))

    def compute(self, setpoint: float, depth: float, depth_rate: float, dt: float) -> float:
        self._update_disturbance(depth_rate, dt)
        # 延迟补偿:用模型把当前状态前推 delay_comp 秒(抵消估计滞后+传输延迟)
        x_now = [depth, depth_rate]
        if self.delay_comp > 0:
            import numpy as _np
            xf = self._rk4_np(_np.array(x_now), self.u_prev, self.delay_comp, self.f_hat)
            x_now = [float(xf[0]), float(xf[1])]
        opti = self.opti
        opti.set_value(self.p_x0, x_now)
        opti.set_value(self.p_zref, setpoint)
        opti.set_value(self.p_uprev, self.u_prev)
        opti.set_value(self.p_fhat, self.f_hat)
        if self._warm_X is not None:
            opti.set_initial(self.X, self._warm_X)
            opti.set_initial(self.U, self._warm_U)
        try:
            sol = opti.solve()
            u = float(sol.value(self.U[0, 0]))
            self._warm_X = sol.value(self.X)
            self._warm_U = sol.value(self.U)
        except Exception as e:  # 求解失败 → 安全回退
            print(f"[mpc] 求解失败({type(e).__name__}),回退 u_prev")
            u = self.u_prev
        u = max(-self.u_limit, min(self.u_limit, u))
        self.u_prev = u
        self.w_prev = depth_rate
        return u


if __name__ == "__main__":
    # 离线自检:用 plant 当"真值"闭环,看 MPC 能否定深
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from src.plant import DepthParams, DepthPlant
    mpc = MPCDepth(_load_model())
    plant = DepthPlant(DepthParams.from_yaml(), z0=0.5)
    dt = 0.1
    sp = 1.0
    print("t   z      w      u")
    for i in range(int(15 / dt)):
        u = mpc.compute(sp, plant.z, plant.w, dt)
        plant.step(u, dt)
        if i % 10 == 0:
            print(f"{i*dt:4.1f} {plant.z:+.3f} {plant.w:+.3f} {u:+.3f}")
