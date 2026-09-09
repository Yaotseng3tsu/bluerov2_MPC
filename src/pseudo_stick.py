#!/usr/bin/env python3
"""Phase 1 — 伪手柄:通过代码发归一化 MANUAL_CONTROL 指令 (最低目标)。

把归一化指令 u∈[-1,1] 映射为 MAVLink MANUAL_CONTROL 各通道并按 CTRL_HZ 发送。
这是"让机器人动起来"的最小实现,等价于 motors.set_forward / set_vertical。

内置安全 (来自 config/vehicle.yaml):
  - 限幅 U_MAX:所有轴 |u| 被裁剪
  - 看门狗:每个周期都发指令;set_zero()/退出时强制回中位
  - 退出保护:正常结束、Ctrl+C、异常 —— 都先发中位再退出
  - 默认【不解锁】(allow_arm=false);arm 需显式开启

对仿真 (tests/sim_vehicle.py) 与真机使用完全相同的接口。

用法 (先跑 SITL,再跑本脚本):
  python -m src.pseudo_stick --demo-heave        # 演示:下潜 2s → 上浮 2s → 停
  python -m src.pseudo_stick --z 0.2 --seconds 3 # 恒定下潜指令 3s
  python -m src.pseudo_stick --x 0.2 --seconds 2 # 前进 (真机验证方向用)
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

try:
    import yaml
except ImportError:
    sys.exit("缺少 pyyaml,请先 pip install -r requirements.txt")

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "vehicle.yaml"
MAV_MODE_FLAG_SAFETY_ARMED = 128


def load_config(path: Path = CONFIG_PATH) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


class PseudoStick:
    """归一化 MANUAL_CONTROL 发送器,带限幅与回中位保护。"""

    def __init__(self, cfg: dict, endpoint: str | None = None):
        from pymavlink import mavutil
        self.mavutil = mavutil
        c = cfg["connection"]
        self.u_max = float(cfg["safety"]["U_MAX"])
        self.mc = cfg.get("manual_control", {})
        self.z_neutral = int(self.mc.get("z_neutral", 500))
        self.sign = {k: int(self.mc.get(f"sign_{k}", 1)) for k in "xyzr"}
        self.endpoint = endpoint or c["endpoint"]
        self.conn = mavutil.mavlink_connection(
            self.endpoint, dialect=c.get("dialect", "ardupilotmega"),
            source_system=255, source_component=0, autoreconnect=True,
        )

    def wait_heartbeat(self, timeout_s: float = 10.0) -> bool:
        print(f"[stick] 等待 heartbeat ({self.endpoint}) ...")
        hb = self.conn.wait_heartbeat(timeout=timeout_s)
        if hb is None:
            print("[stick] ❌ 未收到 heartbeat")
            return False
        armed = bool(hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"[stick] heartbeat OK  system={self.conn.target_system} "
              f"arm={'ARMED' if armed else 'DISARMED'}")
        return True

    def _norm_to_ch(self, u: float, axis: str) -> int:
        """u∈[-1,1] -> 整型通道值。x/y/r: -1000..1000,0 中位; z: 以 z_neutral 为中位。"""
        u = max(-self.u_max, min(self.u_max, u)) * self.sign[axis]
        if axis == "z":
            span = min(self.z_neutral, 1000 - self.z_neutral)
            return int(self.z_neutral + u * span)
        return int(u * 1000)

    def send(self, x=0.0, y=0.0, z=0.0, r=0.0, buttons=0) -> None:
        self.conn.mav.manual_control_send(
            self.conn.target_system,
            self._norm_to_ch(x, "x"),
            self._norm_to_ch(y, "y"),
            self._norm_to_ch(z, "z"),
            self._norm_to_ch(r, "r"),
            buttons,
        )

    def send_neutral(self) -> None:
        self.send(0.0, 0.0, 0.0, 0.0)

    def hold(self, x=0.0, y=0.0, z=0.0, r=0.0, seconds=1.0, hz=10.0) -> None:
        """按 hz 持续发送同一指令 seconds 秒 (满足飞控的连续指令/看门狗要求)。"""
        dt = 1.0 / hz
        n = max(1, int(seconds * hz))
        for _ in range(n):
            self.send(x, y, z, r)
            time.sleep(dt)

    def close(self) -> None:
        # 退出保护:多发几帧中位,确保载具停止
        try:
            for _ in range(5):
                self.send_neutral()
                time.sleep(0.02)
        except Exception:
            pass
        try:
            self.conn.close()
        except Exception:
            pass


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="伪手柄:发归一化 MANUAL_CONTROL")
    p.add_argument("--endpoint", default=None)
    p.add_argument("--x", type=float, default=0.0, help="前进 surge u")
    p.add_argument("--y", type=float, default=0.0, help="右移 sway u")
    p.add_argument("--z", type=float, default=0.0, help="下潜 heave u (+ 下潜)")
    p.add_argument("--r", type=float, default=0.0, help="偏航 yaw u")
    p.add_argument("--seconds", type=float, default=2.0)
    p.add_argument("--hz", type=float, default=10.0)
    p.add_argument("--demo-heave", action="store_true",
                   help="演示:下潜 2s → 上浮 2s → 停")
    args = p.parse_args(argv)

    cfg = load_config()
    hz = float(cfg.get("control", {}).get("CTRL_HZ", args.hz))
    stick = PseudoStick(cfg, endpoint=args.endpoint)
    try:
        if not stick.wait_heartbeat(float(cfg["connection"].get("heartbeat_timeout_s", 10))):
            return 1
        print(f"[stick] U_MAX={stick.u_max}  z_neutral={stick.z_neutral}  hz={hz}")
        if args.demo_heave:
            print("[stick] 演示:下潜 (z=+0.3) 2s")
            stick.hold(z=0.3, seconds=2.0, hz=hz)
            print("[stick] 上浮 (z=-0.3) 2s")
            stick.hold(z=-0.3, seconds=2.0, hz=hz)
            print("[stick] 停")
        else:
            print(f"[stick] 发送 x={args.x} y={args.y} z={args.z} r={args.r},"
                  f"共 {args.seconds}s")
            stick.hold(x=args.x, y=args.y, z=args.z, r=args.r,
                       seconds=args.seconds, hz=hz)
        return 0
    except KeyboardInterrupt:
        print("\n[stick] 用户中断 → 回中位")
        return 130
    finally:
        stick.close()
        print("[stick] 已发送中位并关闭连接 (安全)")


if __name__ == "__main__":
    raise SystemExit(main())
