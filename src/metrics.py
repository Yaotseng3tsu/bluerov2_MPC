#!/usr/bin/env python3
"""控制性能指标(P4/P5 统一)。

从时间序列计算:上升时间、超调、调节时间、稳态误差 RMSE、控制能量,
以及(可选)阶跃后的抗扰恢复。所有控制器用同一套指标对比。
"""
from __future__ import annotations

import numpy as np


def step_response_metrics(t, depth, target, t0, t_end, settle_band=0.05):
    """分析 [t0, t_end] 内对 target 的阶跃响应。

    t, depth: 数组;target: 该段目标深度;t0: 阶跃时刻;settle_band: 调节带(m)。
    返回 dict。假设 t0 时刻已切换到 target。
    """
    t = np.asarray(t); depth = np.asarray(depth)
    m = (t >= t0) & (t <= t_end)
    tt, zz = t[m], depth[m]
    if len(tt) < 3:
        return {}
    z0 = zz[0]
    dz = target - z0
    out = {"target": target, "z_start": float(z0)}

    if abs(dz) < 1e-6:
        # 定点保持:只算稳态
        out["rmse_ss"] = float(np.sqrt(np.mean((zz - target) ** 2)))
        out["max_dev"] = float(np.max(np.abs(zz - target)))
        return out

    # 上升时间 10%→90%
    lvl10, lvl90 = z0 + 0.1 * dz, z0 + 0.9 * dz
    def crossing(level):
        for i in range(1, len(zz)):
            if (zz[i - 1] - level) * (zz[i] - level) <= 0:
                return tt[i]
        return None
    t10, t90 = crossing(lvl10), crossing(lvl90)
    out["rise_time"] = (t90 - t10) if (t10 is not None and t90 is not None) else None

    # 超调 %
    peak = np.max(zz) if dz > 0 else np.min(zz)
    out["overshoot_pct"] = float(max(0.0, (peak - target) / dz * 100.0))

    # 调节时间:最后一次离开 ±band 的时刻
    band = settle_band
    outside = np.abs(zz - target) > band
    idx = np.where(outside)[0]
    out["settle_time"] = float(tt[idx[-1]] - t0) if len(idx) else 0.0

    # 稳态误差 RMSE:末段 30%
    n_ss = max(3, int(0.3 * len(zz)))
    out["rmse_ss"] = float(np.sqrt(np.mean((zz[-n_ss:] - target) ** 2)))
    return out


def control_energy(t, u):
    """∑ u² dt(近似积分)。"""
    t = np.asarray(t); u = np.asarray(u)
    if len(t) < 2:
        return 0.0
    dt = np.diff(t)
    u2 = u[:-1] ** 2
    return float(np.sum(u2 * dt))


def disturbance_recovery(t, depth, target, t_dist, band=0.05):
    """扰动注入 t_dist 后,深度重新回到 ±band 内所需时间;及最大偏离。"""
    t = np.asarray(t); depth = np.asarray(depth)
    m = t >= t_dist
    tt, zz = t[m], depth[m]
    if len(tt) < 3:
        return {}
    max_dev = float(np.max(np.abs(zz - target)))
    rec = None
    for i in range(len(zz)):
        if np.all(np.abs(zz[i:] - target) <= band):
            rec = float(tt[i] - t_dist)
            break
    return {"max_dev": max_dev, "recovery_time": rec}


def summarize(rows: dict) -> str:
    lines = []
    for k, v in rows.items():
        if isinstance(v, float):
            lines.append(f"  {k:16s} = {v:.4f}")
        else:
            lines.append(f"  {k:16s} = {v}")
    return "\n".join(lines)
