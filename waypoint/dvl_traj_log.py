#!/usr/bin/env python3
"""DVL A50 轨迹分析数据记录（velocity + position_local 两路同时记录）。

面向后续轨迹分析 / 航位推算 / EKF：
- velocity 路：body 系速度 vx/vy/vz、协方差 cov_*、altitude、fom、绝对时间戳。
- position_local 路：DVL 内部航位推算的 x/y/z + roll/pitch/yaw + std（世界/局部系）。

两路各存一个 CSV，均带 PC 墙钟时间 wall_time 便于对齐。

用法：
    python dvl_traj_log.py --tag run1
    python dvl_traj_log.py --tag run1 --duration 60
    python dvl_traj_log.py --ip 192.168.2.95 --outdir data

坐标系提示（分析时注意）：
- velocity 的 vx/vy/vz 是 DVL/载体系。要积分成世界系轨迹需按 yaw 旋转。
- position_local 已是 DVL 航位推算后的局部系位置，可直接画轨迹；yaw 由 DVL 内部
  陀螺积分（无罗盘时会缓慢漂移，若 BlueOS 扩展把飞控航向喂给 DVL 则更准）。
"""
import argparse
import csv
import json
import os
import socket
import sys
import time

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

VEL_FIELDS = ["wall_time", "time_of_validity", "time_of_transmission", "time",
              "velocity_valid", "vx", "vy", "vz", "altitude", "fom", "status",
              "cov_xx", "cov_xy", "cov_xz", "cov_yy", "cov_yz", "cov_zz"]
POS_FIELDS = ["wall_time", "ts", "x", "y", "z",
              "roll", "pitch", "yaw", "std", "status"]


def parse_args():
    ap = argparse.ArgumentParser(description="DVL A50 轨迹数据记录")
    ap.add_argument("--ip", default="192.168.2.95")
    ap.add_argument("--port", type=int, default=16171)
    ap.add_argument("--duration", type=float, default=0.0, help="秒，0=Ctrl+C 停止")
    ap.add_argument("--tag", default="run")
    ap.add_argument("--outdir", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "data", "dvl_data"))
    ap.add_argument("--reset", action="store_true",
                    help="开始前发送 reset_dead_reckoning，使 position_local 从 0 起算")
    return ap.parse_args()


def reset_dead_reckoning(sock):
    """向 DVL 发送航位推算归零指令。"""
    try:
        sock.sendall(b'{"command":"reset_dead_reckoning"}\n')
        return True
    except OSError as exc:
        print(f"[warn] 归零指令发送失败：{exc}")
        return False


def cov_flat(cov):
    """3x3 协方差取上三角 6 个: xx,xy,xz,yy,yz,zz。缺失填空。"""
    try:
        return [cov[0][0], cov[0][1], cov[0][2],
                cov[1][1], cov[1][2], cov[2][2]]
    except Exception:
        return [None] * 6


def main() -> int:
    args = parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    vel_path = os.path.join(args.outdir, f"dvl_vel_{args.tag}_{stamp}.csv")
    pos_path = os.path.join(args.outdir, f"dvl_pos_{args.tag}_{stamp}.csv")

    print(f"[*] 连接 DVL {args.ip}:{args.port}")
    print(f"[*] 速度 CSV : {vel_path}")
    print(f"[*] 位置 CSV : {pos_path}")
    print("[*] Ctrl+C 停止" if not args.duration else f"[*] 采样 {args.duration:.0f}s")

    try:
        sock = socket.create_connection((args.ip, args.port), timeout=5)
    except OSError as exc:
        print(f"[FAIL] 无法连接 DVL：{exc}")
        return 2
    sock.settimeout(2.0)

    if args.reset:
        if reset_dead_reckoning(sock):
            print("[*] 已发送 reset_dead_reckoning，position_local 从 0 起算")
        time.sleep(0.5)

    n_vel = n_valid = n_pos = 0
    t0 = time.time()
    deadline = t0 + args.duration if args.duration else None
    last_pos = None

    fv = open(vel_path, "w", newline="", encoding="utf-8")
    fp = open(pos_path, "w", newline="", encoding="utf-8")
    wv = csv.DictWriter(fv, fieldnames=VEL_FIELDS); wv.writeheader()
    wp = csv.DictWriter(fp, fieldnames=POS_FIELDS); wp.writeheader()
    buf = b""
    try:
        while deadline is None or time.time() < deadline:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                print("\n[warn] 2s 无数据…")
                continue
            if not chunk:
                print("\n[warn] 连接被关闭。")
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    m = json.loads(line.decode("utf-8", "ignore"))
                except json.JSONDecodeError:
                    continue
                wt = f"{time.time():.3f}"
                ty = m.get("type")

                if ty == "velocity" or "velocity_valid" in m:
                    n_vel += 1
                    valid = bool(m.get("velocity_valid"))
                    if valid:
                        n_valid += 1
                    cxx, cxy, cxz, cyy, cyz, czz = cov_flat(m.get("covariance"))
                    wv.writerow({
                        "wall_time": wt,
                        "time_of_validity": m.get("time_of_validity"),
                        "time_of_transmission": m.get("time_of_transmission"),
                        "time": m.get("time"),
                        "velocity_valid": valid,
                        "vx": m.get("vx"), "vy": m.get("vy"), "vz": m.get("vz"),
                        "altitude": m.get("altitude"), "fom": m.get("fom"),
                        "status": m.get("status"),
                        "cov_xx": cxx, "cov_xy": cxy, "cov_xz": cxz,
                        "cov_yy": cyy, "cov_yz": cyz, "cov_zz": czz,
                    })
                    fv.flush()

                elif ty == "position_local":
                    n_pos += 1
                    last_pos = m
                    wp.writerow({
                        "wall_time": wt, "ts": m.get("ts"),
                        "x": m.get("x"), "y": m.get("y"), "z": m.get("z"),
                        "roll": m.get("roll"), "pitch": m.get("pitch"),
                        "yaw": m.get("yaw"), "std": m.get("std"),
                        "status": m.get("status"),
                    })
                    fp.flush()

                elapsed = time.time() - t0
                pv = (100 * n_valid / n_vel) if n_vel else 0
                pos_str = ""
                if last_pos:
                    pos_str = (f"pos=({_f(last_pos.get('x'))},{_f(last_pos.get('y'))},"
                               f"{_f(last_pos.get('z'))}) yaw={_f(last_pos.get('yaw'))}")
                sys.stdout.write(
                    f"\r[{elapsed:6.1f}s] vel {n_valid}/{n_vel}({pv:3.0f}%)  "
                    f"pos#{n_pos}  {pos_str}   ")
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n[*] 手动停止。")
    finally:
        sock.close(); fv.close(); fp.close()

    print("\n" + "=" * 60)
    print("记录汇总")
    print("=" * 60)
    print(f"  时长              : {time.time() - t0:.1f} s")
    print(f"  velocity 报文     : {n_vel}（有效 {n_valid}）")
    print(f"  position_local 报文: {n_pos}")
    if last_pos:
        print(f"  末位置 x/y/z      : "
              f"{_f(last_pos.get('x'))}, {_f(last_pos.get('y'))}, {_f(last_pos.get('z'))} m")
        print(f"  末 yaw            : {_f(last_pos.get('yaw'))} deg")
    print(f"  速度 CSV          : {vel_path}")
    print(f"  位置 CSV          : {pos_path}")
    return 0


def _f(v):
    return f"{v:.2f}" if isinstance(v, (int, float)) else str(v)


if __name__ == "__main__":
    sys.exit(main())
