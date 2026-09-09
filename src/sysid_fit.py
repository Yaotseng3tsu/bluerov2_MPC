#!/usr/bin/env python3
"""Phase 3 — 从采集数据拟合深度动力学模型。

模型(下潜为正):
  w_dot = (K*u + net_buoy - c_lin*w - c_quad*w*|w|) / eff_mass

只有比值可辨识(eff_mass 与 K/c/net_buoy 同尺度耦合),故:
  1) 线性最小二乘拟合  a ≈ b_u*u + b0 - a_lin*w - a_quad*w*|w|
     (a=dw/dt, w 用 rate_kf,a 用其数值差分并低通)
  2) 固定 eff_mass = 标称值(--eff-mass,默认取 sim_truth),反算
     K=b_u*m, net_buoy=b0*m, c_lin=a_lin*m, c_quad=a_quad*m。
     —— 动力学只依赖比值,故此约定不影响 MPC/plant 预测。

给出拟合优度 + 用拟合模型仿真回放对比(验证图),可选写回 config/depth_model.yaml。

用法:
  python -m src.sysid_fit data/sysid_pool1.csv                 # 拟合+出图(不写)
  python -m src.sysid_fit data/sysid_pool1.csv --write         # 写回 identified 段
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

import numpy as np

CONFIG = Path(__file__).resolve().parent.parent / "config" / "depth_model.yaml"


def load(path):
    t, u, zk, wk = [], [], [], []
    for r in csv.DictReader(open(path, encoding="utf-8")):
        t.append(float(r["t"])); u.append(float(r["u_cmd"]))
        zk.append(float(r["depth_kf"])); wk.append(float(r["rate_kf"]))
    return np.array(t), np.array(u), np.array(zk), np.array(wk)


def _sim_depth(theta, t, u, z0, w0):
    """用比值参数 theta=[b_u,b0,a_lin,a_quad] 仿真深度轨迹(RK4),只输出 z。"""
    b_u, b0, a_lin, a_quad = theta
    z, w = z0, w0
    zs = np.empty(len(t)); zs[0] = z0
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt <= 0 or dt > 0.5:
            dt = 0.1
        uu = u[i - 1]

        def f(zw):
            zz, ww = zw
            return np.array([ww, b_u * uu + b0 - a_lin * ww - a_quad * ww * abs(ww)])
        x = np.array([z, w])
        k1 = f(x); k2 = f(x + 0.5 * dt * k1); k3 = f(x + 0.5 * dt * k2); k4 = f(x + dt * k3)
        x = x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        z, w = x[0], x[1]
        zs[i] = z
    return zs


def fit(t, u, w, z):
    """仿真误差最小化:拟合 z(t),避免对噪声微分。返回比值参数与深度拟合优度。"""
    from scipy.optimize import least_squares
    z0, w0 = z[0], w[0]
    # 初值:合理量级(b_u~3, 阻尼~0.2/1.4, 浮力~0)
    x0 = [3.0, 0.0, 0.2, 1.0]

    def resid(theta):
        return _sim_depth(theta, t, u, z0, w0) - z
    sol = least_squares(resid, x0, method="trf",
                        bounds=([0.1, -2, 0, 0], [20, 2, 20, 50]), max_nfev=2000)
    b_u, b0, a_lin, a_quad = sol.x
    z_sim = _sim_depth(sol.x, t, u, z0, w0)
    rmse = float(np.sqrt(np.mean((z_sim - z) ** 2)))
    ss_tot = np.sum((z - z.mean()) ** 2)
    r2 = 1 - np.sum((z - z_sim) ** 2) / ss_tot if ss_tot > 0 else float("nan")
    return dict(b_u=b_u, b0=b0, a_lin=a_lin, a_quad=a_quad, r2=float(r2), rmse=rmse)


def simulate(t, u, z0, w0, params):
    """用拟合模型回放,返回预测深度轨迹(验证用)。"""
    from src.plant import DepthParams, DepthPlant
    p = DepthParams(eff_mass=params["eff_mass"], c_lin=params["c_lin"],
                    c_quad=params["c_quad"], K_thrust_N=params["K_thrust_N"],
                    net_buoy_N=params["net_buoy_N"], u_max=1.0)
    plant = DepthPlant(p, z0=z0, w0=w0)
    zs = [z0]
    for i in range(1, len(t)):
        dt = max(1e-3, min(0.5, t[i] - t[i - 1]))
        plant.step(u[i - 1], dt)
        zs.append(plant.z)
    return np.array(zs)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="P3 深度模型拟合")
    p.add_argument("csv")
    p.add_argument("--eff-mass", type=float, default=None,
                   help="标称有效质量;默认取 depth_model.yaml 的 sim_truth.eff_mass")
    p.add_argument("--write", action="store_true", help="把结果写回 identified 段")
    args = p.parse_args(argv)

    import yaml
    dm = yaml.safe_load(open(CONFIG, encoding="utf-8"))
    eff_mass = args.eff_mass or float(dm["sim_truth"]["eff_mass"])

    t, u, zk, wk = load(args.csv)
    f = fit(t, u, wk, zk)
    params = dict(eff_mass=eff_mass,
                  K_thrust_N=f["b_u"] * eff_mass,
                  net_buoy_N=f["b0"] * eff_mass,
                  c_lin=f["a_lin"] * eff_mass,
                  c_quad=f["a_quad"] * eff_mass)

    print("=== 拟合结果 (仿真误差最小化;wdot=b_u*u+b0-a_lin*w-a_quad*w|w|) ===")
    print(f"  b_u={f['b_u']:.4f}  b0={f['b0']:.4f}  a_lin={f['a_lin']:.4f}  a_quad={f['a_quad']:.4f}")
    print(f"  深度拟合 R²={f['r2']:.3f}  RMSE={f['rmse']:.4f} m")
    print(f"\n=== 反算参数 (eff_mass 固定 = {eff_mass}) ===")
    for k in ("eff_mass", "K_thrust_N", "net_buoy_N", "c_lin", "c_quad"):
        print(f"  {k:12s} = {params[k]:.4f}")

    # 回放验证图
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        z_sim = simulate(t, u, zk[0], wk[0], params)
        png = str(Path(args.csv).with_suffix("")) + "_fit.png"
        fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        ax[0].plot(t, zk, label="measured (kf)", lw=1.3)
        ax[0].plot(t, z_sim, "--", label="fitted model", lw=1.3)
        ax[0].invert_yaxis(); ax[0].legend(); ax[0].grid(alpha=.3); ax[0].set_ylabel("depth (m,+down)")
        ax[1].plot(t, u, color="tab:orange"); ax[1].set_ylabel("u"); ax[1].set_xlabel("t(s)"); ax[1].grid(alpha=.3)
        fit_rmse = float(np.sqrt(np.mean((z_sim - zk) ** 2)))
        fig.suptitle(f"sysid fit validation  (depth RMSE={fit_rmse:.3f} m)")
        fig.tight_layout(); fig.savefig(png, dpi=110)
        print(f"\n回放验证: 深度 RMSE={fit_rmse:.3f} m  图={png}")
    except Exception as e:
        print(f"(出图跳过: {e})")

    if args.write:
        # 行级替换 identified: 段下的键(定位真正的段起始行,忽略注释里的 "identified:")
        keys = ("eff_mass", "c_lin", "c_quad", "K_thrust_N", "net_buoy_N")
        lines = CONFIG.read_text(encoding="utf-8").split("\n")
        idx = next((i for i, l in enumerate(lines)
                    if l.startswith("identified:")), None)
        if idx is None:
            print("❌ 未找到 identified: 段,未写入。")
            return 1
        for j in range(idx + 1, len(lines)):
            stripped = lines[j].lstrip()
            if lines[j] and not lines[j][0].isspace():
                break  # 到下一个顶格段,停止
            for k in keys:
                if stripped.startswith(k + ":"):
                    lines[j] = f"  {k}: {params[k]:.4f}"
        CONFIG.write_text("\n".join(lines), encoding="utf-8")
        print(f"\n✅ 已写回 {CONFIG} 的 identified 段(MPC 将优先使用)。")
    else:
        print("\n(未写回;确认无误后加 --write 写入 identified 段)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
