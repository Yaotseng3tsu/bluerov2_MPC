#!/usr/bin/env python3
"""Phase 2 验证 —— 深度状态估计器质量 (确定性数值实验)。

用 src/plant.py 生成"真值"轨迹 → 叠加已知高斯噪声 → 喂给 KF →
对比 KF 输出与真值,给出并判定:
  1. 噪声标定:从 (measured - truth) 反算 σ,应 ≈ 注入 σ
  2. 深度平滑:RMSE(z_kf, z_true) 应 < 注入 σ (滤波确实降噪)
  3. 速率跟踪:RMSE(rate_kf, w_true) 有界且合理 (不发散)
  4. 时延:rate_kf 相对 w_true 的互相关滞后,应 < 若干控制周期

不依赖网络。用固定随机种子,结果可复现。
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402

from src.plant import DepthParams, DepthPlant  # noqa: E402
from src.state import DepthEstimator  # noqa: E402


def generate_truth(dt: float, T: float):
    """用 plant 生成真值 (t, z_true, w_true),指令方波激励产生变化的速率。"""
    p = DepthParams.from_yaml()
    plant = DepthPlant(p)
    n = int(T / dt)
    ts, zs, ws, us = [], [], [], []
    for i in range(n):
        t = i * dt
        # 方波指令:每 4s 切换 +0.3 / -0.3,制造正负速率
        u = 0.3 if (int(t // 4) % 2 == 0) else -0.3
        plant.step(u, dt)
        ts.append(t); zs.append(plant.z); ws.append(plant.w); us.append(u)
    return (np.array(ts), np.array(zs), np.array(ws), np.array(us))


def xcorr_lag(a: np.ndarray, b: np.ndarray, dt: float, max_lag: int = 30) -> float:
    """返回使 a(t) 最匹配 b(t-lag) 的滞后秒数 (a 相对 b 落后为正)。"""
    a = a - a.mean()
    b = b - b.mean()
    best_lag, best = 0, -1e18
    for lag in range(0, max_lag + 1):
        if lag == 0:
            c = float(np.dot(a, b))
        else:
            c = float(np.dot(a[lag:], b[:-lag]))
        if c > best:
            best, best_lag = c, lag
    return best_lag * dt


def main() -> int:
    for _s in (sys.stdout, sys.stderr):
        try:
            _s.reconfigure(encoding="utf-8")
        except Exception:
            pass

    dt = 0.1                    # 10 Hz 深度
    T = 40.0
    inj_sigma = 0.03            # 注入噪声 std (m)
    rng = np.random.default_rng(12345)

    ts, z_true, w_true, us = generate_truth(dt, T)
    noise = rng.normal(0.0, inj_sigma, size=z_true.shape)
    z_meas = z_true + noise

    est = DepthEstimator.from_config()
    z_kf, r_kf = [], []
    for i in range(len(ts)):
        est.feed(float(z_meas[i]), float(ts[i]))
        z_kf.append(est.depth)
        r_kf.append(est.depth_rate)
    z_kf = np.array(z_kf); r_kf = np.array(r_kf)

    # 丢弃前 2s 收敛段
    k = int(2.0 / dt)
    zt, wt = z_true[k:], w_true[k:]
    zm = z_meas[k:]
    zk, rk = z_kf[k:], r_kf[k:]

    rec_sigma = float(np.std(zm - zt))
    rmse_z = float(np.sqrt(np.mean((zk - zt) ** 2)))
    rmse_meas = float(np.sqrt(np.mean((zm - zt) ** 2)))
    rmse_rate = float(np.sqrt(np.mean((rk - wt) ** 2)))
    rate_max = float(np.max(np.abs(rk)))
    w_max = float(np.max(np.abs(wt)))
    lag_s = xcorr_lag(rk, wt, dt)

    print("=== P2 状态估计验证 ===")
    print(f"注入噪声 σ = {inj_sigma:.3f} m,反算 σ = {rec_sigma:.3f} m")
    print(f"深度 RMSE:  原始测量 {rmse_meas:.4f} m  →  KF {rmse_z:.4f} m")
    print(f"速率 RMSE(rate_kf vs w_true) = {rmse_rate:.4f} m/s  (|w|max={w_max:.3f})")
    print(f"速率幅值:  |rate_kf|max={rate_max:.3f}  (真值 |w|max={w_max:.3f}) —— 不发散检查")
    print(f"速率估计时延 ≈ {lag_s*1000:.0f} ms  ({lag_s/dt:.1f} 个深度周期)")

    checks = {
        "噪声反算 σ ≈ 注入 σ (±40%)": abs(rec_sigma - inj_sigma) <= 0.4 * inj_sigma,
        "KF 降噪 (RMSE_z < 注入 σ)": rmse_z < inj_sigma,
        "速率 RMSE < 0.15 m/s": rmse_rate < 0.15,
        "速率不发散 (|rate_kf|max < 2*|w|max)": rate_max < 2.0 * w_max,
        "速率时延 < 0.6 s": lag_s < 0.6,
    }
    print("\n--- 判定 ---")
    ok = True
    for name, passed in checks.items():
        print(f"  {'✅' if passed else '❌'} {name}")
        ok = ok and passed
    print(f"\n=== {'全部通过' if ok else '存在未通过项'} ===")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
