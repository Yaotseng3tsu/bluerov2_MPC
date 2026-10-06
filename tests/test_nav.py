#!/usr/bin/env python3
"""导航重构的离线对照测试(合成数据, 无需网络/真机)。

对照的是 2026-10-07 重构前后的三条链路。每项都跑**旧做法 vs 新做法**, 打印定量差距:

  A. 前向位移   —— 10Hz 控制环按标称 dt 重积分 ZOH  vs  DVL 逐帧按自带 dt 累加
  B. 离底高度   —— 只有 DVL(高度 + vz)             vs  DVL 高度 + 飞控 EKF 垂向速度
  C. 航向       —— 绝对 yaw + 角度数值微分          vs  陀螺积分 + 实测角速率当 D 项
  C2. 闭环收敛  —— 换了 D 项之后控制器本身仍要能把航向转到位

C 项复现的是 2026-09-17 实测到的两种污染: 瞬时尖刺 +58° 与永久平移 +105°,
外加"每下潜一段就用罗盘重置一次 yaw"的反复重对准。

用法:  python tests/test_nav.py
"""
from __future__ import annotations

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

from waypoint.altitude_estimator import AltitudeEstimator  # noqa: E402
from waypoint.heading_hold import HeadingHold, YawParams, YawPlant, wrap_deg  # noqa: E402
from waypoint.heading_tracker import HeadingTracker  # noqa: E402

PASS, FAIL = [], []


def check(name: str, ok: bool, detail: str) -> None:
    (PASS if ok else FAIL).append(name)
    print(f"{'✅' if ok else '❌'} {name}: {detail}")


# =============================== A. 前向位移 ===============================
def test_distance() -> None:
    """DVL 4.5-7.5Hz 而控制环 10Hz —— 旧做法同一帧会被重复积分 1.3-2.2 次。"""
    print("\n--- A. 前向位移: 10Hz 重积分 ZOH  vs  DVL 逐帧 dt ---")
    rnd = random.Random(20261007)
    T = 35.0

    def v_true(t):                      # 真实前向速度: 起步 + 巡航 + 轻微波动
        ramp = min(1.0, t / 4.0)
        return ramp * (0.060 + 0.012 * math.sin(2 * math.pi * t / 9.0))

    # --- DVL 帧序列: 名义 4.5-7.5Hz 随机间隔; 2.5% 无解 ---
    frames = []                          # (t_frame, dt, vx_measured, valid)
    t, t_prev = 0.0, 0.0
    while t < T:
        t += 1.0 / rnd.uniform(4.5, 7.5)
        if t >= T:
            break
        # DVL 报的是整个 ping 区间的平均速度
        v_avg = 0.5 * (v_true(t_prev) + v_true(t)) + rnd.gauss(0, 0.004)
        frames.append((t, t - t_prev, v_avg, rnd.random() > 0.025))
        t_prev = t

    truth = sum(0.5 * (v_true(a) + v_true(b)) * (b - a)
                for a, b in zip([0.0] + [f[0] for f in frames[:-1]],
                                [f[0] for f in frames]))

    # --- 旧做法: 10Hz 环, s += v_latest * dt_nom (标称 0.1, 实测 sleep 有溢出) ---
    s_old, i, v_hold, t_loop = 0.0, 0, 0.0, 0.0
    while t_loop < frames[-1][0]:
        t_loop += 0.1 * rnd.uniform(1.00, 1.08)    # Windows sleep 溢出
        while i < len(frames) and frames[i][0] <= t_loop:
            if frames[i][3]:
                v_hold = frames[i][2]
            i += 1
        s_old += v_hold * 0.1                      # 用的是**标称** dt

    # --- 新做法: 逐帧按自带 dt 累加, 无解帧用上一帧零阶保持 ---
    s_new, v_last = 0.0, None
    for _t, dt_f, v, valid in frames:
        if valid:
            v_last = v
            s_new += v * dt_f
        elif v_last is not None:
            s_new += v_last * dt_f

    e_old, e_new = abs(s_old - truth), abs(s_new - truth)
    print(f"    真值 {truth:.4f} m | 旧 {s_old:.4f} m (误差 {s_old - truth:+.4f}, "
          f"{100 * e_old / truth:.1f}%) | 新 {s_new:.4f} m (误差 {s_new - truth:+.4f}, "
          f"{100 * e_new / truth:.1f}%)")
    check("A 位移精度", e_new < e_old * 0.5 and e_new / truth < 0.01,
          f"误差从 {100 * e_old / truth:.1f}% 降到 {100 * e_new / truth:.1f}% "
          f"(要求 <1% 且至少好一半)")


# =============================== B. 离底高度 ===============================
def _alt_scenario(seed: int):
    """生成一段真实高度轨迹 + 两路量测。"""
    rnd = random.Random(seed)
    T, bottom = 40.0, 2.5

    def depth_true(t):                   # 下潜到位后定高, 含波动
        base = 1.70 - 0.45 * min(1.0, t / 8.0)
        return (base + 0.03 * math.sin(2 * math.pi * t / 6.0)
                + 0.012 * math.sin(2 * math.pi * t / 1.7))

    def alt_true(t):
        return bottom - depth_true(t)

    def rate_true(t, h=1e-3):
        return (alt_true(t + h) - alt_true(t - h)) / (2 * h)

    dvl = []                             # (t, altitude, vz_down, valid)
    t = 0.0
    while t < T:
        t += 1.0 / rnd.uniform(4.5, 7.5)
        if t >= T:
            break
        valid = rnd.random() > 0.025                      # 2.5% 无解(实测)
        a = alt_true(t) + rnd.gauss(0, 0.012)
        if rnd.random() < 0.03:                           # 3% 外点(实测跳到 2.5m)
            a = 2.5
        vz = -rate_true(t) + rnd.gauss(0, 0.10)           # DVL vz: sigma>=0.1 m/s
        dvl.append((t, a, vz, valid))

    depth = []                           # (t, depth, vz_down) @20Hz, 质量高得多
    t = 0.0
    while t < T:
        t += 0.05
        depth.append((t, depth_true(t) + rnd.gauss(0, 0.005),
                      -rate_true(t) + rnd.gauss(0, 0.02)))
    return alt_true, rate_true, dvl, depth, T


def _run_old(dvl, T, dt_ctl=0.1):
    """旧做法: 10Hz 控制环, rate 量测只有 DVL vz。"""
    est = AltitudeEstimator()
    out, i, t = [], 0, 0.0
    while t < T:
        t += dt_ctl
        est.predict(dt_ctl)
        new = None
        while i < len(dvl) and dvl[i][0] <= t:
            if dvl[i][3]:
                new = dvl[i]
            i += 1
        if new is not None:
            est.update_rate(-new[2])
            est.update_alt(new[1])
        e = est.estimate()
        out.append((t, e.alt, e.rate))
    return out


def _run_new(dvl, depth, T, dt_ctl=0.1):
    """新做法: 深度帧按真实时序推进 + -vz 校正 rate; DVL 只做高度位置量测。"""
    est = AltitudeEstimator()
    out, i, j, t, t_f = [], 0, 0, 0.0, 0.0
    while t < T:
        t += dt_ctl
        while j < len(depth) and depth[j][0] <= t:
            step = depth[j][0] - t_f
            if step > 0:
                est.predict(step)
            est.update_rate(-depth[j][2], r=est.r_vz_ekf)
            t_f = depth[j][0]
            j += 1
        if t > t_f:
            est.predict(t - t_f)
            t_f = t
        new = None
        while i < len(dvl) and dvl[i][0] <= t:
            if dvl[i][3]:
                new = dvl[i]
            i += 1
        if new is not None:
            est.update_alt(new[1])
        e = est.estimate()
        out.append((t, e.alt, e.rate))
    return out


def test_altitude() -> None:
    print("\n--- B. 离底高度: 仅 DVL  vs  DVL + 深度计快通道 (5 种子) ---")
    agg = {"old": [0.0, 0.0, 0.0], "new": [0.0, 0.0, 0.0]}
    for seed in range(1, 6):
        alt_true, rate_true, dvl, depth, T = _alt_scenario(seed)
        for tag, series in (("old", _run_old(dvl, T)), ("new", _run_new(dvl, depth, T))):
            use = [r for r in series if r[0] > 3.0]       # 跳过初始化段
            n = len(use)
            rmse = math.sqrt(sum((a - alt_true(t)) ** 2 for t, a, _ in use) / n)
            emax = max(abs(a - alt_true(t)) for t, a, _ in use)
            rrms = math.sqrt(sum((r - rate_true(t)) ** 2 for t, _, r in use) / n)
            agg[tag][0] += rmse / 5
            agg[tag][1] = max(agg[tag][1], emax)
            agg[tag][2] += rrms / 5
    o, n_ = agg["old"], agg["new"]
    print(f"    {'':14s}{'高度 RMSE':>11s}{'高度最大误差':>13s}{'速率 RMSE':>11s}")
    print(f"    {'旧(仅DVL)':14s}{o[0]:11.4f}{o[1]:13.4f}{o[2]:11.4f}")
    print(f"    {'新(+深度计)':14s}{n_[0]:11.4f}{n_[1]:13.4f}{n_[2]:11.4f}")
    print(f"    {'改善':14s}{o[0] / n_[0]:10.2f}x{o[1] / n_[1]:12.2f}x{o[2] / n_[2]:10.2f}x")
    check("B 高度 RMSE", n_[0] < o[0], f"{o[0]:.4f} → {n_[0]:.4f} ({o[0] / n_[0]:.2f}x)")
    check("B 高度最大误差", n_[1] < o[1], f"{o[1]:.4f} → {n_[1]:.4f} ({o[1] / n_[1]:.2f}x)")
    check("B 速率 RMSE", n_[2] < o[2] * 0.5,
          f"{o[2]:.4f} → {n_[2]:.4f} ({o[2] / n_[2]:.2f}x, 要求至少 2x)")


# =============================== C. 航向 ===============================
def test_heading() -> None:
    """同一段污染过的 yaw, 分别喂旧控制器与新控制器。

    关键设计: **机器人物理上完全不转**。于是任何非零的 u_r 都是被传感器污染骗出来的
    虚假打舵 —— 这正是 2026-09-17 最危险的那个行为: 控制器被假跳变骗到 u_r=-0.90
    满舵, 把机器人真的抡了起来。
    (若让真实航向也动, 这个开环测试里"追不上"的误差会混进来, 测的就不是抗污染能力了;
     控制器本身的收敛能力交给下面的 C2 闭环测。)
    """
    print("\n--- C. 航向: 绝对yaw+角度微分  vs  陀螺积分+实测角速率 ---")
    dt = 0.05
    rnd = random.Random(99)
    T = 60.0

    # 污染: 5% 瞬时 +58° 尖刺; t=20s 起永久 +105°; 每 12s 再叠加一次罗盘重置 ±35°
    creset = {"k": -1, "off": 0.0}

    def corrupt(t, psi):
        y = psi
        if t >= 20.0:
            y += 105.0
        k = int(t // 12.0)
        if k != creset["k"]:
            creset["k"] = k
            if k > 0:
                creset["off"] += rnd.uniform(-35, 35)
        y += creset["off"]
        if rnd.random() < 0.05:
            y += 58.0
        return wrap_deg(y)

    hh_old = HeadingHold(kp=1.0, kd=0.3, ki=0.2, r_limit=0.9, i_limit=0.9)
    hh_new = HeadingHold(kp=1.0, kd=0.3, ki=0.2, r_limit=0.9, i_limit=0.9)
    tr = HeadingTracker()

    psi_true, t = 0.0, 0.0          # 真实航向恒为 0 —— 机器人不转
    ur_old_max = ur_new_max = 0.0
    est_old_max = est_new_max = 0.0
    started = False
    for _i in range(int(T / dt)):
        y_rep = corrupt(t, psi_true)
        tr.feed(y_rep, 0.0, t=t)             # 陀螺如实报告"没转"
        fix = tr.estimate(t=t)
        if not started and fix.ok and t > 1.0:
            started = True
            hh_old.reset(y_rep)
            hh_new.reset(fix.heading_deg)
        if not started:
            t += dt
            continue
        u_old = hh_old.update(y_rep, dt)                       # 旧: 角度数值微分
        u_new = hh_new.update(fix.heading_deg, dt, rate_dps=fix.rate_dps)
        ur_old_max = max(ur_old_max, abs(u_old))
        ur_new_max = max(ur_new_max, abs(u_new))
        # 航向**估计**误差: 估计值 vs 真实物理指向
        est_old_max = max(est_old_max, abs(wrap_deg(y_rep - psi_true)))
        est_new_max = max(est_new_max, abs(wrap_deg(fix.heading_deg - psi_true)))
        t += dt

    print(f"    {'':8s}{'|u_r| 峰值':>12s}{'航向估计误差峰值':>18s}")
    print(f"    {'旧':8s}{ur_old_max:12.2f}{est_old_max:17.1f}°")
    print(f"    {'新':8s}{ur_new_max:12.2f}{est_new_max:17.1f}°")
    leak = 100.0 * est_new_max / max(abs(tr.offset), 1e-9)
    print(f"    (机器人全程未转; 跟踪器吸收重对准 {tr.realign_n} 次, 末偏置 {tr.offset:+.1f}°;"
          f" 累计污染只漏进 {leak:.1f}%)")
    # 残余的那几度来自**低于门限**的小幅罗盘重置 —— 门限拦不住, 会被互补滤波
    # 以 1/tau 的速率慢慢吸收进来。这是 --yaw-tau 的固有取舍, 不是缺陷:
    # 调大 tau 或调小 --yaw-gate-deg 都能再压, 代价是更不信/更敏感于罗盘。
    # 判据按**物理后果**定: 不能打出会把机器人抡起来的舵, 航向估计不能跑偏到影响走直线。
    check("C 虚假打舵 |u_r|", ur_new_max < 0.2 and ur_new_max < ur_old_max * 0.25,
          f"{ur_old_max:.2f} → {ur_new_max:.2f} (要求 <0.2 且至少好 4x)")
    check("C 航向估计误差", est_new_max < 3.0,
          f"{est_old_max:.1f}° → {est_new_max:.1f}° (要求 <3°; 污染漏入率 {leak:.1f}%)")


def test_heading_closed_loop() -> None:
    """闭环收敛自检: D 项换成实测角速率后, 控制器本身仍要能把航向转到位。

    抗污染改好了但控制变差就是白改。用 heading_hold 自带的 YawPlant 闭环对照。
    """
    print("\n--- C2. 闭环收敛 (无污染): D 项两种算法对照 ---")
    dt = 0.1
    res = {}
    for tag, use_rate in (("角度数值微分(旧)", False), ("实测角速率(新)", True)):
        ctrl = HeadingHold(kp=1.0, kd=0.2, r_limit=0.3, tol_deg=3.0)
        ctrl.reset(90.0)
        plant = YawPlant(YawParams(), yaw_deg=0.0)
        yaw, settle, hold, over = 0.0, None, 0, 0.0
        for i in range(int(30.0 / dt)):
            rate_dps = math.degrees(plant.rate) if use_rate else None
            r = ctrl.update(yaw, dt, rate_dps=rate_dps)
            yaw = wrap_deg(plant.step(r, dt))
            over = max(over, yaw - 90.0)
            hold = hold + 1 if ctrl.at_target(yaw) else 0
            if settle is None and hold >= int(1.0 / dt):
                settle = i * dt
        res[tag] = (settle, ctrl.error_deg(yaw), over)
        st = f"{settle:.2f}s" if settle is not None else "未到位"
        print(f"    {tag:18s} 到位 {st:>8s}  末误差 {ctrl.error_deg(yaw):+.2f}°  "
              f"超调 {over:+.2f}°")
    s_new, e_new, _ = res["实测角速率(新)"]
    s_old = res["角度数值微分(旧)"][0]
    check("C2 闭环收敛", s_new is not None and abs(e_new) < 1.0,
          f"到位 {s_new:.2f}s (旧 {s_old:.2f}s), 末误差 {e_new:+.2f}° (要求收敛且 <1°)")


def main() -> int:
    test_distance()
    test_altitude()
    test_heading()
    test_heading_closed_loop()
    total = len(PASS) + len(FAIL)
    print(f"\n=== {len(PASS)}/{total} 通过 ===")
    if FAIL:
        print("失败项: " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
