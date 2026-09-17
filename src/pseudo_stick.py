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
        self.armed_by_us = False  # 只对"本进程解锁的"负责自动上锁

    def wait_heartbeat(self, timeout_s: float = 10.0) -> bool:
        """只锁定真正的飞控心跳(autopilot≠INVALID 且非 GCS),把 target 固定到飞控。

        网络上除飞控外还有 BlueOS/路由器/GCS 的心跳(常为 system 0 或 GCS 类型);
        若锁错 system,MANUAL_CONTROL 会被 ArduSub 忽略 → 电机不转。
        """
        m = self.mavutil.mavlink
        print(f"[stick] 等待飞控 heartbeat ({self.endpoint}) ...")
        t_end = time.monotonic() + timeout_s
        hb = None
        while time.monotonic() < t_end:
            msg = self.conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.autopilot != m.MAV_AUTOPILOT_INVALID and msg.type != m.MAV_TYPE_GCS:
                hb = msg
                self.conn.target_system = msg.get_srcSystem()
                self.conn.target_component = msg.get_srcComponent()
                break
        if hb is None:
            print("[stick] ❌ 未收到飞控 heartbeat(只收到 GCS/路由器心跳?)")
            return False
        armed = bool(hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"[stick] heartbeat OK  system={self.conn.target_system} "
              f"comp={self.conn.target_component} arm={'ARMED' if armed else 'DISARMED'}")
        return True

    def detect_rival_manual_control(self, seconds: float = 2.0) -> dict:
        """监听是否有**其它来源**也在发 MANUAL_CONTROL (典型: Cockpit/QGC 的手柄)。

        飞控只认最后到达的那条 MANUAL_CONTROL。若手柄以 25Hz 刷中位、
        而我们以 10Hz 发指令, 约 70% 的周期会被手柄的中位覆盖 → 指令被稀释、推不动。
        (2026-09-17 水中实测: sys255/comp240 以 25Hz 发 z≈490, 导致满推也上不去。)

        返回 {(sys, comp): {"n": 条数, "hz": 频率, "z": [z 样例]}}, 空 dict = 没有竞争源。
        """
        mine = (self.conn.source_system, self.conn.source_component)
        found: dict = {}
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            m = self.conn.recv_match(type="MANUAL_CONTROL", blocking=True, timeout=0.3)
            if m is None:
                continue
            key = (m.get_srcSystem(), m.get_srcComponent())
            if key == mine:
                continue
            d = found.setdefault(key, {"n": 0, "z": []})
            d["n"] += 1
            if len(d["z"]) < 6:
                d["z"].append(m.z)
        for d in found.values():
            d["hz"] = d["n"] / max(seconds, 1e-6)
        return found

    def warn_if_rival(self, seconds: float = 2.0) -> bool:
        """检测并打印竞争源警告。返回 True = 存在竞争源。"""
        rivals = self.detect_rival_manual_control(seconds)
        if not rivals:
            return False
        print("=" * 66)
        print("⚠️  检测到**其它来源**也在发 MANUAL_CONTROL —— 指令会互相覆盖!")
        for (s, c), d in sorted(rivals.items()):
            print(f"    sys{s}/comp{c}: {d['n']} 条 ≈ {d['hz']:.0f} Hz, z 样例={d['z']}")
        print("    飞控只认最后到达的一条; 对方频率更高时我们的指令大部分会被冲掉,")
        print("    表现为「满推也推不动 / 感觉在打架」。")
        print("    → 请先在 Cockpit/QGC 里**断开或停用手柄**, 再跑本脚本。")
        print("=" * 66)
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

    # -------- keepalive:后台持续发指令+心跳,避免 input() 阻塞期间失联失效 --------
    def start_keepalive(self, hz: float = 10.0) -> None:
        import threading
        self._ka_cmd = [0.0, 0.0, 0.0, 0.0]
        self._ka_run = True
        self._ka_hz = hz
        self._ka_thread = threading.Thread(target=self._ka_loop, daemon=True)
        self._ka_thread.start()

    def _ka_loop(self) -> None:
        period = 1.0 / self._ka_hz
        i = 0
        while getattr(self, "_ka_run", False):
            c = self._ka_cmd
            try:
                self.send(x=c[0], y=c[1], z=c[2], r=c[3])
                if i % max(1, int(self._ka_hz)) == 0:
                    self.send_gcs_heartbeat()
            except Exception:
                pass
            i += 1
            time.sleep(period)

    def set_cmd(self, x=0.0, y=0.0, z=0.0, r=0.0) -> None:
        """设置 keepalive 线程持续发送的指令值。"""
        self._ka_cmd = [x, y, z, r]

    def stop_keepalive(self) -> None:
        self._ka_run = False
        th = getattr(self, "_ka_thread", None)
        if th is not None:
            th.join(timeout=1.0)

    # ---------------- 解锁 / 模式 (真机需要) ----------------
    def send_gcs_heartbeat(self) -> None:
        """以 GCS 身份发心跳,避免 ArduSub 的 GCS 失联失效。"""
        m = self.mavutil.mavlink
        self.conn.mav.heartbeat_send(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def set_mode(self, mode_name: str = "MANUAL") -> bool:
        # mode_mapping_sub 是 id->名称,需反向查名称->id
        name_to_id = {v: k for k, v in self.mavutil.mode_mapping_sub.items()}
        if mode_name not in name_to_id:
            print(f"[stick] 未知模式 {mode_name}(可选: {sorted(name_to_id)})")
            return False
        mode_id = name_to_id[mode_name]
        m = self.mavutil.mavlink
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            m.MAV_CMD_DO_SET_MODE, 0,
            m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
        print(f"[stick] 请求模式 = {mode_name}")
        return True

    def _wait_ack(self, cmd_id: int, timeout: float = 3.0):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            ack = self.conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
            if ack and ack.command == cmd_id:
                return ack.result
        return None

    def is_armed(self, timeout: float = 2.0) -> bool | None:
        """只认飞控(锁定的 target)心跳;BlueOS 组件心跳的 armed 字段无意义,须过滤。"""
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            hb = self.conn.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
            if hb is None:
                continue
            if (hb.get_srcSystem() == self.conn.target_system
                    and hb.get_srcComponent() == self.conn.target_component):
                return bool(hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
        return None

    def arm(self, force: bool = False, timeout: float = 5.0) -> bool:
        """解锁。返回是否成功。会持续发 GCS 心跳。"""
        m = self.mavutil.mavlink
        # 解锁前先喂几帧心跳 + 中位,建立 GCS 存在
        for _ in range(5):
            self.send_gcs_heartbeat()
            self.send_neutral()
            time.sleep(0.05)
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            m.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 21196 if force else 0, 0, 0, 0, 0, 0)
        res = self._wait_ack(m.MAV_CMD_COMPONENT_ARM_DISARM, timeout)
        ok = (res == 0)  # MAV_RESULT_ACCEPTED
        if ok:
            self.armed_by_us = True
            print("[stick] ✅ 已解锁 (ARMED)  —— 推进器现在会转,注意安全!")
        else:
            print(f"[stick] ❌ 解锁失败 (ACK result={res})。"
                  f"可能预检未过;可尝试 --force 或先在 Cockpit 解锁。")
        return ok

    def _send_disarm_cmd(self) -> None:
        """只发上锁命令,不等 ACK (fire-and-forget,退出保护用)。"""
        m = self.mavutil.mavlink
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            m.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

    def disarm(self, timeout: float = 3.0) -> bool:
        m = self.mavutil.mavlink
        self.send_neutral()
        self._send_disarm_cmd()
        res = self._wait_ack(m.MAV_CMD_COMPONENT_ARM_DISARM, timeout)
        ok = (res == 0)
        print("[stick] 已上锁 (DISARMED)" if ok else f"[stick] ⚠ 上锁未确认 (ACK={res}),已重发命令")
        self.armed_by_us = not ok
        return ok

    def hold(self, x=0.0, y=0.0, z=0.0, r=0.0, seconds=1.0, hz=10.0) -> None:
        """按 hz 持续发送同一指令 seconds 秒 (满足飞控的连续指令/看门狗要求)。"""
        dt = 1.0 / hz
        n = max(1, int(seconds * hz))
        for i in range(n):
            self.send(x, y, z, r)
            if i % max(1, int(hz)) == 0:   # 约 1 Hz 补 GCS 心跳
                self.send_gcs_heartbeat()
            time.sleep(dt)

    def close(self) -> None:
        self.stop_keepalive()
        # 退出保护:先回中位,若本进程解锁过则自动上锁 (多次 fire-and-forget,不依赖收包)
        try:
            for _ in range(5):
                self.send_neutral()
                time.sleep(0.02)
        except Exception:
            pass
        if self.armed_by_us:
            print("[stick] 退出保护:自动上锁 (disarm)")
            for _ in range(5):   # 反复发上锁命令,确保至少一条到达
                try:
                    self.send_neutral()
                    self._send_disarm_cmd()
                except Exception:
                    pass
                time.sleep(0.05)
            self.armed_by_us = False
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
    p.add_argument("--arm", action="store_true",
                   help="真机:设 MANUAL 模式并解锁 (推进器会转!);退出自动上锁")
    p.add_argument("--force-arm", action="store_true", help="强制解锁 (绕过部分预检)")
    p.add_argument("--mode", default="MANUAL", help="解锁前设置的飞行模式")
    p.add_argument("--yes", action="store_true", help="跳过解锁前的交互确认")
    args = p.parse_args(argv)

    cfg = load_config()
    hz = float(cfg.get("control", {}).get("CTRL_HZ", args.hz))
    stick = PseudoStick(cfg, endpoint=args.endpoint)
    try:
        if not stick.wait_heartbeat(float(cfg["connection"].get("heartbeat_timeout_s", 10))):
            return 1
        print(f"[stick] U_MAX={stick.u_max}  z_neutral={stick.z_neutral}  hz={hz}")
        if args.arm:
            if not args.yes:
                ans = input("[stick] ⚠ 即将解锁,推进器会转。确认现场安全(出水/远离桨叶)? 输入 yes: ")
                if ans.strip().lower() != "yes":
                    print("[stick] 已取消解锁。")
                    return 0
            stick.set_mode(args.mode)
            time.sleep(0.3)
            if not stick.arm(force=args.force_arm):
                return 2
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
