#!/usr/bin/env python3
"""DVL A50 轨迹绘图 + 数据质量检查。

读取 dvl_traj_log.py 生成的 vel/pos 两个 CSV，输出：
- 数据质量摘要（有效率、fom、altitude、频率、时长）
- 一张 2x2 图：XY 轨迹、速度时序、高度时序、yaw 时序
  XY 轨迹同时画两条：position_local（DVL 航位推算）与 velocity 世界系积分（对照）。

用法：
    python plot_dvl_traj.py --tag 02_fwd            # 自动找 data/dvl_data 里最新匹配
    python plot_dvl_traj.py --vel <vel.csv> --pos <pos.csv>
    python plot_dvl_traj.py                          # 用最新一对
"""
import argparse
import csv
import glob
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "dvl_data")


def load_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return rows


def col(rows, key, cast=float):
    out = []
    for r in rows:
        v = r.get(key, "")
        try:
            out.append(cast(v))
        except (ValueError, TypeError):
            out.append(np.nan)
    return np.array(out, dtype=float)


def find_pair(args):
    if args.vel and args.pos:
        return args.vel, args.pos
    pat_v = os.path.join(DATA_DIR, f"dvl_vel_{args.tag}_*.csv" if args.tag else "dvl_vel_*.csv")
    pat_p = os.path.join(DATA_DIR, f"dvl_pos_{args.tag}_*.csv" if args.tag else "dvl_pos_*.csv")
    vs = sorted(glob.glob(pat_v))
    ps = sorted(glob.glob(pat_p))
    if not vs or not ps:
        sys.exit(f"[FAIL] 找不到 CSV：{pat_v} / {pat_p}")
    return vs[-1], ps[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="")
    ap.add_argument("--vel", default="")
    ap.add_argument("--pos", default="")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    vel_path, pos_path = find_pair(args)
    print(f"[*] vel: {vel_path}")
    print(f"[*] pos: {pos_path}")
    vel = load_csv(vel_path)
    pos = load_csv(pos_path)

    # ---- velocity ----
    tov = col(vel, "time_of_validity")          # μs
    tv = (tov - tov[0]) / 1e6 if len(tov) else np.array([])
    vx, vy, vz = col(vel, "vx"), col(vel, "vy"), col(vel, "vz")
    alt = col(vel, "altitude")
    fom = col(vel, "fom")
    valid = np.array([str(r.get("velocity_valid")).lower() == "true" for r in vel])

    # ---- position_local ----
    px, py, pz = col(pos, "x"), col(pos, "y"), col(pos, "z")
    yaw = col(pos, "yaw")
    pts = col(pos, "ts")
    tp = pts - pts[0] if len(pts) else np.array([])

    # ---- 数据质量摘要 ----
    n = len(vel)
    nvalid = int(valid.sum())
    dur = tv[-1] if len(tv) else 0.0
    rate = n / dur if dur > 0 else 0.0
    print("\n=== 数据质量摘要 ===")
    print(f"  velocity 报文 : {n}，有效 {nvalid} ({100*nvalid/n:.0f}%)" if n else "  无速度数据")
    print(f"  时长 / 频率   : {dur:.1f}s / {rate:.1f}Hz")
    if nvalid:
        print(f"  fom (有效)    : 均值 {np.nanmean(fom[valid]):.4f}，最大 {np.nanmax(fom[valid]):.4f}")
        print(f"  altitude      : {np.nanmin(alt[valid]):.2f} ~ {np.nanmax(alt[valid]):.2f} m")
        sp = np.sqrt(vx**2 + vy**2)[valid]
        print(f"  水平速度      : 均值 {np.nanmean(sp):.3f}，峰值 {np.nanmax(sp):.3f} m/s")
    print(f"  position_local: {len(pos)} 条")
    if len(px):
        disp = math.hypot(px[-1] - px[0], py[-1] - py[0])
        print(f"  末位置 x/y/z  : {px[-1]:.2f}, {py[-1]:.2f}, {pz[-1]:.2f} m（净位移 {disp:.2f} m）")

    # ---- 由 velocity 世界系积分（对照轨迹）----
    # 用 pos 的 yaw 按 vel 时间插值，body(vx,vy) 旋转到世界系再积分
    ix = iy = None
    if len(tv) > 1 and len(tp) > 1:
        yaw_i = np.interp(tv, tp, np.radians(yaw))
        wvx = vx * np.cos(yaw_i) - vy * np.sin(yaw_i)
        wvy = vx * np.sin(yaw_i) + vy * np.cos(yaw_i)
        dt = np.diff(tv, prepend=tv[0])
        m = valid.copy()
        wvx[~m] = 0
        wvy[~m] = 0
        ix = np.cumsum(wvx * dt)
        iy = np.cumsum(wvy * dt)

    # ---- 绘图 ----
    fig, ax = plt.subplots(2, 2, figsize=(12, 9))
    fig.suptitle(f"DVL A50 trajectory — {os.path.basename(vel_path)}", fontsize=12)

    a = ax[0, 0]
    if len(px):
        a.plot(px, py, "-", color="tab:blue", lw=1.5, label="position_local (DVL DR)")
        a.plot(px[0], py[0], "go", ms=9, label="start")
        a.plot(px[-1], py[-1], "rs", ms=9, label="end")
    if ix is not None:
        a.plot(ix, iy, "--", color="tab:orange", lw=1.2, label="velocity-integrated")
    a.set_xlabel("x [m]"); a.set_ylabel("y [m]"); a.set_title("XY trajectory")
    a.axis("equal"); a.grid(True, alpha=0.3); a.legend(fontsize=8)

    a = ax[0, 1]
    a.plot(tv, vx, label="vx (fwd)", lw=1)
    a.plot(tv, vy, label="vy (right)", lw=1)
    a.plot(tv, vz, label="vz (down)", lw=1)
    a.set_xlabel("t [s]"); a.set_ylabel("velocity [m/s]"); a.set_title("Body velocity")
    a.grid(True, alpha=0.3); a.legend(fontsize=8); a.axhline(0, color="k", lw=0.5)

    a = ax[1, 0]
    a.plot(tv[valid], alt[valid], ".", ms=3, color="tab:green")
    a.set_xlabel("t [s]"); a.set_ylabel("altitude [m]"); a.set_title("Altitude to bottom")
    a.grid(True, alpha=0.3)

    a = ax[1, 1]
    a.plot(tp, yaw, color="tab:purple", lw=1)
    a.set_xlabel("t [s]"); a.set_ylabel("yaw [deg]"); a.set_title("Yaw (DVL DR)")
    a.grid(True, alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out = args.out or os.path.join(os.path.dirname(vel_path),
                                   os.path.basename(vel_path).replace("dvl_vel_", "plot_").replace(".csv", ".png"))
    fig.savefig(out, dpi=130)
    print(f"\n[OK] 图已保存：{out}")


if __name__ == "__main__":
    main()
