#!/usr/bin/env python3
"""M3 SITL 验收 — 外部 8 路直控的安全/功能验收 (跑在官方 SITL, 不碰真机)。

验收项(对应 README §7 的四条):
  A. 未解锁时发非零外部命令      → 8 路输出必须保持 1500
  B. 只改一路                    → 只有对应电机输出变化, 其余保持 1500
  C. 停发命令(心跳仍在)          → 外部看门狗: 归中 + disarm + 锁存故障
  D. 非法/越权输入拒收           → 错误 group / NaN / 超范围 一律不生效;
                                   锁存后拒绝接受, 清 MOT_EXT_ENABLE 才解锁存

用法(WSL 内, 两个终端):
  # 终端1 起 SITL
  cd ~/rov-dev/ardupilot-external
  build/sitl/bin/ardusub -S -I0 --model vectored_6dof \
      --defaults Tools/autotest/default_params/sub.parm
  # 终端2 跑验收
  python3 /mnt/c/bluerov2_mpc/direct_thruster/sitl_accept.py
"""
from __future__ import annotations

import math
import sys
import time

from pymavlink import mavutil

CONN = "tcp:127.0.0.1:5760"
NMOT = 8
NEUTRAL = 1500

results: list[tuple[str, bool, str]] = []


def record(name: str, ok: bool, detail: str = "") -> None:
    results.append((name, ok, detail))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  — {detail}" if detail else ""))


class Sitl:
    def __init__(self, conn):
        self.c = conn
        self.statustexts: list[str] = []
        self.armed = False

    # ---------- 基础 ----------
    def pump(self) -> None:
        """吸收待处理报文, 顺带记录 STATUSTEXT 与 armed 状态。"""
        while True:
            m = self.c.recv_match(blocking=False)
            if m is None:
                return
            t = m.get_type()
            if t == "STATUSTEXT":
                txt = m.text.decode() if isinstance(m.text, bytes) else m.text
                self.statustexts.append(txt)
            elif t == "HEARTBEAT" and m.get_srcSystem() == self.c.target_system:
                self.armed = bool(m.base_mode & 128)

    def heartbeat(self) -> None:
        self.c.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                  mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def set_param(self, name: str, value: float, ptype=mavutil.mavlink.MAV_PARAM_TYPE_REAL32) -> bool:
        for _ in range(10):
            self.c.mav.param_set_send(self.c.target_system, self.c.target_component,
                                      name.encode(), float(value), ptype)
            t_end = time.time() + 1.0
            while time.time() < t_end:
                m = self.c.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
                if m is None:
                    continue
                pid = m.param_id
                if isinstance(pid, bytes):
                    pid = pid.split(b"\x00")[0].decode(errors="ignore")
                if pid == name and abs(m.param_value - value) < 1e-3:
                    return True
        return False

    def send_ext(self, controls, group: int = 1) -> None:
        vals = list(controls) + [0.0] * (NMOT - len(controls))
        self.c.mav.set_actuator_control_target_send(
            int(time.time() * 1e6) & 0xFFFFFFFF, group,
            self.c.target_system, self.c.target_component, vals)

    def stream_ext(self, controls, seconds: float, group: int = 1, hz: float = 25.0) -> None:
        """按 hz 持续发外部命令 + 心跳。"""
        dt = 1.0 / hz
        t_end = time.time() + seconds
        i = 0
        while time.time() < t_end:
            self.send_ext(controls, group=group)
            if i % int(hz) == 0:
                self.heartbeat()
            self.pump()
            i += 1
            time.sleep(dt)

    def idle(self, seconds: float, hz: float = 10.0) -> None:
        """只发心跳, 不发外部命令(用于触发看门狗)。"""
        dt = 1.0 / hz
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.heartbeat()
            self.pump()
            time.sleep(dt)

    def servos(self, seconds: float = 1.0):
        """采样 SERVO_OUTPUT_RAW, 返回每通道出现过的值集合。"""
        seen = {i: set() for i in range(1, NMOT + 1)}
        t_end = time.time() + seconds
        while time.time() < t_end:
            m = self.c.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.5)
            if m is None:
                continue
            for i in range(1, NMOT + 1):
                seen[i].add(getattr(m, f"servo{i}_raw"))
        return seen

    def servos_while_streaming(self, controls, seconds: float, group: int = 1):
        """边发命令边采样 —— 命令必须持续发, 否则会超时。"""
        seen = {i: set() for i in range(1, NMOT + 1)}
        t_end = time.time() + seconds
        i = 0
        while time.time() < t_end:
            self.send_ext(controls, group=group)
            if i % 25 == 0:
                self.heartbeat()
            m = self.c.recv_match(type="SERVO_OUTPUT_RAW", blocking=False)
            while m is not None:
                for k in range(1, NMOT + 1):
                    seen[k].add(getattr(m, f"servo{k}_raw"))
                m = self.c.recv_match(type="SERVO_OUTPUT_RAW", blocking=False)
            self.pump()
            i += 1
            time.sleep(0.04)
        return seen

    def reset_ext(self) -> None:
        """回到干净起点: 上锁 + 清故障锁存(MOT_EXT_ENABLE 0→1) + 丢弃旧命令。

        必须这么做 —— 两项测试之间只要有 >MOT_EXT_TMOUT 的空档, 看门狗就会触发并锁存,
        后续测试全部会被"正确地"拒绝, 看起来像功能坏了。
        """
        self.arm(False)
        self.set_param("MOT_EXT_ENABLE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        self.idle(0.4)
        self.set_param("MOT_EXT_ENABLE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        self.statustexts.clear()

    def arm_and_settle(self) -> bool:
        """解锁并等 spool up, 期间持续发零命令以免刚解锁就超时锁存。"""
        if not self.arm(True):
            return False
        self.stream_ext([0.0] * NMOT, 1.2)
        return True

    def set_mode_manual(self) -> None:
        self.c.mav.command_long_send(self.c.target_system, self.c.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                     19, 0, 0, 0, 0, 0)  # 19 = MANUAL
        time.sleep(0.5)

    def arm(self, want: bool = True) -> bool:
        for _ in range(5):
            self.c.mav.command_long_send(self.c.target_system, self.c.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                         1 if want else 0, 0, 0, 0, 0, 0, 0)
            t_end = time.time() + 2.0
            while time.time() < t_end:
                self.heartbeat()
                self.pump()
                if self.armed == want:
                    return True
                time.sleep(0.05)
        return self.armed == want


# 实际启用的电机通道 (启动后自动探测: 未解锁时读数为 1500 的才是被驱动的电机输出)
MOTOR_CH: list[int] = list(range(1, NMOT + 1))


def only(seen, ch: int) -> bool:
    """只有 ch 通道离开过中位, 其余启用通道恒 1500。"""
    for i in MOTOR_CH:
        vals = seen.get(i, set())
        if i == ch:
            if vals == {NEUTRAL} or not vals:
                return False
        elif vals != {NEUTRAL}:
            return False
    return True


def all_neutral(seen) -> bool:
    return all(seen.get(i, set()) == {NEUTRAL} for i in MOTOR_CH)


def fmt(seen) -> str:
    return str({i: sorted(seen.get(i, set())) for i in MOTOR_CH})


def main() -> int:
    print(f"[sitl] 连接 {CONN} ...")
    conn = mavutil.mavlink_connection(CONN)
    conn.wait_heartbeat()
    print(f"[sitl] heartbeat OK sys={conn.target_system} comp={conn.target_component}")
    s = Sitl(conn)
    s.pump()

    # ---- 准备 ----
    print("\n[准备] 设置参数 ...")
    ok = (s.set_param("ARMING_CHECK", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
          and s.set_param("SYSID_MYGCS", 255, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
          and s.set_param("MOT_EXT_TMOUT", 500, mavutil.mavlink.MAV_PARAM_TYPE_INT16)
          and s.set_param("MOT_EXT_ENABLE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8))
    if not ok:
        print("  ❌ 参数设置失败 —— MOT_EXT_* 是否存在? 固件是否为改版?")
        return 2
    print("  参数 OK: MOT_EXT_ENABLE=1, MOT_EXT_TMOUT=500")
    boot = [t for t in s.statustexts if "EXT-THRUSTER" in t]
    record("开机标记 STATUSTEXT 存在", bool(boot), boot[0] if boot else "未捕获(可能在连接前已发出)")
    s.set_mode_manual()

    # ---- A: 未解锁时发非零命令 ----
    print("\n[A] 未解锁时发非零外部命令 (期望: 8 路恒 1500)")
    s.arm(False)
    seen = s.servos_while_streaming([0.6] * NMOT, 2.5)
    record("A 未解锁时输出保持安全中位", all_neutral(seen),
           f"各通道观测值 { {k: sorted(v) for k, v in seen.items()} }" if not all_neutral(seen) else "")

    # ---- B: 只改一路 ----
    print("\n[B] 解锁后只驱动 Motor3 (期望: 只有 SERVO3 变化)")
    if not s.arm(True):
        record("B 解锁", False, "无法解锁")
    else:
        time.sleep(1.0)  # 等 spool up
        ctl = [0.0] * NMOT
        ctl[2] = 0.6     # Motor3
        seen = s.servos_while_streaming(ctl, 2.5)
        record("B 只有 Motor3 输出变化", only(seen, 3),
               f"SERVO3={sorted(seen[3])} 其余={ {k: sorted(v) for k, v in seen.items() if k != 3} }")

    # ---- C: 停发命令 → 看门狗 ----
    print("\n[C] 停发外部命令(心跳仍在) (期望: 归中 + disarm + 锁存)")
    s.statustexts.clear()
    s.idle(2.0)
    seen = s.servos(1.0)
    got_text = any("external thrust" in t.lower() for t in s.statustexts)
    record("C 看门狗归中位", all_neutral(seen), f"{ {k: sorted(v) for k, v in seen.items()} }" if not all_neutral(seen) else "")
    record("C 看门狗 disarm", not s.armed, f"armed={s.armed}")
    record("C 看门狗告警 STATUSTEXT", got_text, str(s.statustexts[-3:]))

    # ---- D1: 锁存后拒绝 ----
    print("\n[D1] 故障锁存后, 合法命令也应被拒绝")
    s.arm(True)
    time.sleep(0.8)
    ctl = [0.0] * NMOT
    ctl[2] = 0.6
    seen = s.servos_while_streaming(ctl, 2.0)
    record("D1 锁存期间合法命令不生效", all_neutral(seen),
           f"SERVO3={sorted(seen[3])}")

    # ---- D2: 清 MOT_EXT_ENABLE 解锁存 ----
    print("\n[D2] 清 MOT_EXT_ENABLE 再打开 → 应恢复可用")
    s.set_param("MOT_EXT_ENABLE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    s.idle(0.5)
    s.set_param("MOT_EXT_ENABLE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    s.arm(True)
    time.sleep(0.8)
    seen = s.servos_while_streaming(ctl, 2.0)
    record("D2 解锁存后恢复可控", only(seen, 3), f"SERVO3={sorted(seen[3])}")

    # ---- D3: 错误 group 拒收 ----
    print("\n[D3] group_mlx=0 (标准组) 应被拒收")
    s.idle(1.5)                      # 让上一条命令过期 + 触发锁存
    s.set_param("MOT_EXT_ENABLE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    s.set_param("MOT_EXT_ENABLE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    s.arm(True)
    time.sleep(0.8)
    seen = s.servos_while_streaming(ctl, 2.0, group=0)
    record("D3 错误 group 不生效", all_neutral(seen), f"SERVO3={sorted(seen[3])}")

    # ---- D4: 非法数值拒收 ----
    print("\n[D4] NaN / 超范围 应整帧拒收")
    s.set_param("MOT_EXT_ENABLE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    s.set_param("MOT_EXT_ENABLE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    s.arm(True)
    time.sleep(0.8)
    bad = [0.0] * NMOT
    bad[2] = 0.6
    bad[5] = float("nan")
    seen_nan = s.servos_while_streaming(bad, 1.5)
    bad2 = [0.0] * NMOT
    bad2[2] = 0.6
    bad2[5] = 5.0
    seen_big = s.servos_while_streaming(bad2, 1.5)
    record("D4 含 NaN 整帧拒收", all_neutral(seen_nan), f"SERVO3={sorted(seen_nan[3])}")
    record("D4 含超范围值整帧拒收", all_neutral(seen_big), f"SERVO3={sorted(seen_big[3])}")

    # ---- 收尾 ----
    s.arm(False)
    s.set_param("MOT_EXT_ENABLE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)

    print("\n" + "=" * 62)
    npass = sum(1 for _, ok, _ in results if ok)
    for name, ok, detail in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    print(f"\n总计 {npass}/{len(results)} 通过")
    return 0 if npass == len(results) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n中断")
        sys.exit(130)
