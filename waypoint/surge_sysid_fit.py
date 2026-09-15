#!/usr/bin/env python3
"""W2 — 从采集数据拟合 surge 动力学模型。

模型 (前进为正, 无浮力项):
  v_dot = (K*u - c_lin*v - c_quad*v*|v|) / eff_mass + b0

拟合 (仿真误差最小化, 直接拟合 DVL 测得的 vx, 无需微分):
  1) 拟合 a ≈ b_u*u + b0 - a_lin*v - a_quad*v*|v|  (v 用 DVL vx)
     b0 = 残余恒定加速度 (推力不对称/残余水流), 仅报告不入模型。
  2) 固定 eff_mass = 标称 (--eff-mass, 默认 surge_model.yaml 的 sim_truth), 反算
     K=b_u*m, c_lin=a_lin*m, c_quad=a_quad*m。(动力学只依赖比值。)

给出拟合优度 + 回放对比图, 可选写回 config/surge_model.yaml 的 identified 段。

用法:
  python -m waypoint.surge_sysid_fit data/surge_sysid_sitl.csv           # 拟合+出图(不写)
  python -m waypoint.surge_sysid_fit data/surge_sysid_pool1.csv --write  # 写回 identified
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

CONFIG = Path(__file__).resolve().parent / "config" / "surge_model.yaml"


def load(path):
    t, u, v, valid = [], [], [], []
    for r in csv.DictReader(open(path, encoding="utf-8")):
        t.append(float(r["t"])); u.append(float(r["u_cmd"]))
        v.append(float(r["vx"])); valid.append(int(r.get("dvl_valid", 1)))
    return np.array(t), np.array(u), np.array(v), np.array(valid)


def _sim_v(theta, t, u, v0):
    """用比值参数 theta=[b_u,b0,a_lin,a_quad] 仿真速度轨迹 (RK4), 输出 v。"""
    b_u, b0, a_lin, a_quad = theta
    v = v0
    vs = np.empty(len(t)); vs[0] = v0
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt <= 0 or dt > 0.5:
            dt = 0.1
        uu = u[i - 1]

        def f(vv):
            return b_u * uu + b0 - a_lin * vv - a_quad * vv * abs(vv)
        k1 = f(v); k2 = f(v + 0.5 * dt * k1); k3 = f(v + 0.5 * dt * k2); k4 = f(v + dt * k3)
        v = v + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        vs[i] = v
    return vs


def fit(t, u, v):
    from scipy.optimize import least_squares
    v0 = v[0]
    x0 = [3.0, 0.0, 0.3, 1.0]

    def resid(theta):
        return _sim_v(theta, t, u, v0) - v
    sol = least_squares(resid, x0, method="trf",
                        bounds=([0.1, -1, 0, 0], [40, 1, 40, 100]), max_nfev=3000)
    b_u, b0, a_lin, a_quad = sol.x
    v_sim = _sim_v(sol.x, t, u, v0)
    rmse = float(np.sqrt(np.mean((v_sim - v) ** 2)))
    ss_tot = np.sum((v - v.mean()) ** 2)
    r2 = 1 - np.sum((v - v_sim) ** 2) / ss_tot if ss_tot > 0 else float("nan")
    return dict(b_u=b_u, b0=b0, a_lin=a_lin, a_quad=a_quad, r2=float(r2), rmse=rmse)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="W2 surge 模型拟合")
    p.add_argument("csv")
    p.add_argument("--eff-mass", type=float, default=None,
                   help="标称有效质量;默认取 surge_model.yaml 的 sim_truth.eff_mass")
    p.add_argument("--write", action="store_true", help="把结果写回 identified 段")
    args = p.parse_args(argv)

    import yaml
    sm = yaml.safe_load(open(CONFIG, encoding="utf-8"))
    eff_mass = args.eff_mass or float(sm["sim_truth"]["eff_mass"])

    t, u, v, valid = load(args.csv)
    mask = valid > 0
    if mask.sum() < len(v):
        print(f"[fit] 丢弃 {int((~mask).sum())} 个无效(无底锁)样本")
        t, u, v = t[mask], u[mask], v[mask]
    f = fit(t, u, v)
    params = dict(eff_mass=eff_mass,
                  K_thrust_N=f["b_u"] * eff_mass,
                  c_lin=f["a_lin"] * eff_mass,
                  c_quad=f["a_quad"] * eff_mass)

    print("=== 拟合结果 (vdot=b_u*u+b0-a_lin*v-a_quad*v|v|) ===")
    print(f"  b_u={f['b_u']:.4f}  b0={f['b0']:.4f}(残余加速度)  "
          f"a_lin={f['a_lin']:.4f}  a_quad={f['a_quad']:.4f}")
    print(f"  速度拟合 R²={f['r2']:.3f}  RMSE={f['rmse']:.4f} m/s")
    print(f"\n=== 反算参数 (eff_mass 固定 = {eff_mass}) ===")
    for k in ("eff_mass", "K_thrust_N", "c_lin", "c_quad"):
        print(f"  {k:12s} = {params[k]:.4f}")
    # 终速参考 (u=0.5)
    b_u, _, a_lin, a_quad = f["b_u"], f["b0"], f["a_lin"], f["a_quad"]
    import numpy as _np
    vt = _np.roots([a_quad, a_lin, -b_u * 0.5])
    vt = [x.real for x in vt if abs(x.imag) < 1e-6 and x.real > 0]
    if vt:
        print(f"  → u=0.5 稳态终速 ≈ {vt[0]:.3f} m/s")

    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        v_sim = _sim_v([f["b_u"], f["b0"], f["a_lin"], f["a_quad"]], t, u, v[0])
        png = str(Path(args.csv).with_suffix("")) + "_fit.png"
        fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        ax[0].plot(t, v, label="DVL vx (measured)", lw=1.2)
        ax[0].plot(t, v_sim, "--", label="fitted model", lw=1.2)
        ax[0].legend(); ax[0].grid(alpha=.3); ax[0].set_ylabel("vx (m/s)")
        ax[1].plot(t, u, color="tab:orange"); ax[1].set_ylabel("u_x"); ax[1].set_xlabel("t(s)"); ax[1].grid(alpha=.3)
        fig.suptitle(f"surge sysid fit  (R2={f['r2']:.3f}, RMSE={f['rmse']:.3f} m/s)")
        fig.tight_layout(); fig.savefig(png, dpi=110)
        print(f"\n回放验证图: {png}")
    except Exception as e:
        print(f"(出图跳过: {e})")

    if args.write:
        keys = ("eff_mass", "c_lin", "c_quad", "K_thrust_N")
        lines = CONFIG.read_text(encoding="utf-8").split("\n")
        idx = next((i for i, l in enumerate(lines) if l.startswith("identified:")), None)
        if idx is None:
            print("❌ 未找到 identified: 段, 未写入。"); return 1
        for j in range(idx + 1, len(lines)):
            if lines[j] and not lines[j][0].isspace():
                break
            stripped = lines[j].lstrip()
            for k in keys:
                if stripped.startswith(k + ":"):
                    lines[j] = f"  {k}: {params[k]:.4f}"
        CONFIG.write_text("\n".join(lines), encoding="utf-8")
        print(f"\n✅ 已写回 {CONFIG} 的 identified 段 (go_waypoint 将优先使用)。")
    else:
        print("\n(未写回;确认无误后加 --write 写入 identified 段)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
