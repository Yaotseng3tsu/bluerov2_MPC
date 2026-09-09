#!/usr/bin/env python3
"""深度闭环 CSV 分析 —— 指标 + 出图(P4/P5 通用)。

用法:
  python tests/analyze_depth.py data/depth_pid_step.csv --t-step 12 --target 1.0
  python tests/analyze_depth.py data/depth_pid_disturb.csv --detect-disturb --target 0.8
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from src import metrics as M  # noqa: E402


def load(path):
    t, sp, z, rate, u, st = [], [], [], [], [], []
    with open(path, encoding="utf-8") as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"])); sp.append(float(row["setpoint"]))
            z.append(float(row["depth"])); rate.append(float(row["depth_rate"]))
            u.append(float(row["u"])); st.append(row["status"])
    return (np.array(t), np.array(sp), np.array(z), np.array(rate),
            np.array(u), np.array(st))


def detect_disturbance(t, z, target, band, t_min=5.0, k=2.0):
    """跳过启动瞬态(t<t_min)后,首次超出 k*band 的时刻,作为扰动起点。"""
    for i in range(len(t)):
        if t[i] >= t_min and abs(z[i] - target) > k * band:
            return float(t[i])
    return None


def main() -> int:
    for _s in (sys.stdout, sys.stderr):
        try:
            _s.reconfigure(encoding="utf-8")
        except Exception:
            pass
    p = argparse.ArgumentParser()
    p.add_argument("csv")
    p.add_argument("--t-step", type=float, default=None, help="阶跃时刻")
    p.add_argument("--target", type=float, default=None, help="阶跃后目标(默认取末段 setpoint)")
    p.add_argument("--band", type=float, default=0.05, help="调节带/稳态带 (m)")
    p.add_argument("--detect-disturb", action="store_true")
    args = p.parse_args()

    t, sp, z, rate, u, st = load(args.csv)
    target = args.target if args.target is not None else float(sp[-1])
    print(f"=== 分析 {os.path.basename(args.csv)} ===")
    print(f"样本 {len(t)},时长 {t[-1]:.1f}s,目标(末)={target}")
    print(f"看门狗/限位 非OK 帧: {int(np.sum(st!='OK'))}")

    result = {}
    if args.t_step is not None:
        sm = M.step_response_metrics(t, z, target, args.t_step, t[-1], args.band)
        print("\n--- 阶跃响应指标 ---")
        for k in ("rise_time", "overshoot_pct", "settle_time", "rmse_ss"):
            if k in sm and sm[k] is not None:
                print(f"  {k:14s} = {sm[k]:.4f}")
                result[k] = sm[k]

    result["control_energy"] = M.control_energy(t, u)
    print(f"  control_energy = {result['control_energy']:.4f}")

    t_dist = None
    if args.detect_disturb:
        t_dist = detect_disturbance(t, z, target, args.band)
        if t_dist is not None:
            dr = M.disturbance_recovery(t, z, target, t_dist, args.band)
            print("\n--- 抗扰 ---")
            print(f"  扰动起点 ≈ {t_dist:.1f}s")
            print(f"  max_dev        = {dr.get('max_dev'):.4f}")
            rec = dr.get("recovery_time")
            print(f"  recovery_time  = {rec if rec is None else round(rec,2)}")
        else:
            print("\n(未检测到明显扰动)")

    # --- 出图 ---
    png = os.path.splitext(args.csv)[0] + ".png"
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    ax[0].plot(t, z, label="depth", lw=1.4)
    ax[0].plot(t, sp, "--", label="setpoint", color="k", lw=1)
    ax[0].fill_between(t, sp - args.band, sp + args.band, color="gray", alpha=0.15,
                       label=f"±{args.band}m")
    if t_dist:
        ax[0].axvline(t_dist, color="r", ls=":", label="disturbance")
    ax[0].set_ylabel("depth (m, +down)"); ax[0].legend(loc="best"); ax[0].grid(alpha=0.3)
    ax[0].invert_yaxis()
    ax[1].plot(t, u, color="tab:orange", lw=1.2, label="u (norm)")
    ax[1].set_ylabel("u"); ax[1].set_xlabel("t (s)"); ax[1].grid(alpha=0.3); ax[1].legend()
    fig.suptitle(os.path.basename(args.csv))
    fig.tight_layout()
    fig.savefig(png, dpi=110)
    print(f"\n图已保存: {png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
