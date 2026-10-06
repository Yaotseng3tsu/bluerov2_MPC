#!/usr/bin/env python3
"""M4 SITL 验收 — 外部 8 路直控的功能/安全验收 (跑在官方 SITL, 不碰真机)。

用法 (WSL 内, 一条命令起 SITL + 跑验收):
    bash /mnt/c/bluerov2_mpc/direct_thruster/sitl_run.sh

也可以手动分两步, 但 SITL 必须用 FRAME_CONFIG=2 起 (见 sitl_ext.parm), 否则
Motor7/8 不被使能、SERVO7/8 恒为 0, 所有"全中位"判据会整体报红。

计分项 (14):
  A        未解锁时发非零外部命令    -> 8 路恒 1500
  B1..B4   解锁后的单路独立性        -> 只有目标电机动 / 推力标度 / 负向 / 八路逐一扫描
  C1..C4   看门狗                    -> 归中 / disarm / 告警 / 故障锁存
  D1..D5   拒收与恢复                -> 错 group / NaN / 越范围 / 错 target / 按流程恢复

前置条件 (不计分, 不满足直接 abort —— 首轮 0/11 的教训是: 前提错了还照常跑完,
会产出一堆看不懂的 FAIL):
  * MOT_EXT_ENABLE / MOT_EXT_TMOUT 存在 (证明跑的是改版固件)
  * FRAME_CONFIG == 2
  * 未解锁时探测到的电机通道恰好是 {1..8}
  * 基线: 关掉外部控制 + 解锁 + 中位手柄 -> 8 路全 1500
"""
from __future__ import annotations

import sys
import time

from pymavlink import mavutil

# Windows 控制台是 cp932, 打印中文会直接崩 -> 强制 utf-8
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

CONN = "tcp:127.0.0.1:5760"
NMOT = 8
NEUTRAL = 1500
PWM_SPAN = 400.0        # calc_thrust_to_pwm(): 1500 + thrust * 400
PWM_TOL = 30            # 推力标度判据的容差
TMOUT_MS = 500          # MOT_EXT_TMOUT
MODE_MANUAL = 19        # ArduSub custom mode

INT8 = mavutil.mavlink.MAV_PARAM_TYPE_INT8
INT16 = mavutil.mavlink.MAV_PARAM_TYPE_INT16
INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32

# 实际被使能的电机输出通道, 启动后由 detect_motor_channels() 覆写
MOTOR_CH: list[int] = list(range(1, NMOT + 1))

results: list[tuple[str, bool, str]] = []
infos: list[str] = []


def record(name: str, ok: bool, detail: str = "") -> None:
    results.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"   {detail}" if detail else ""))


def info(text: str) -> None:
    infos.append(text)
    print(f"  [INFO] {text}")


def die(msg: str) -> int:
    print(f"\n  [ABORT] {msg}")
    print("\n前置条件不满足, 不再往下跑 —— 继续跑只会产生一堆与本功能无关的 FAIL。")
    return 2


class Obs:
    """一次采样窗口里每个通道出现过的 PWM 值。"""

    def __init__(self) -> None:
        self.seen: dict[int, set[int]] = {i: set() for i in range(1, NMOT + 1)}
        self.n = 0

    def add(self, msg) -> None:
        for i in range(1, NMOT + 1):
            self.seen[i].add(getattr(msg, f"servo{i}_raw"))
        self.n += 1

    def vals(self, ch: int) -> list[int]:
        return sorted(self.seen.get(ch, set()))

    def enough(self) -> bool:
        """采样太少则任何判据都不可信 —— 宁可报 FAIL 也不要假 PASS。"""
        return self.n >= 5

    def moved(self, ch: int) -> bool:
        v = self.seen.get(ch, set())
        return bool(v) and v != {NEUTRAL}

    def peak(self, ch: int) -> int:
        """离中位最远的那个读数 (没采到就返回 1500)。"""
        v = self.seen.get(ch, set())
        if not v:
            return NEUTRAL
        return max(v, key=lambda x: abs(x - NEUTRAL))

    def all_neutral(self) -> bool:
        return self.enough() and all(self.seen.get(i) == {NEUTRAL} for i in MOTOR_CH)

    def only(self, ch: int) -> bool:
        """只有 ch 离开过中位, 其余使能通道恒 1500。"""
        if not self.enough() or not self.moved(ch):
            return False
        return all(self.seen.get(i) == {NEUTRAL} for i in MOTOR_CH if i != ch)

    def dump(self, skip_neutral: bool = False) -> str:
        parts = []
        for i in MOTOR_CH:
            v = self.vals(i)
            if skip_neutral and v == [NEUTRAL]:
                continue
            parts.append(f"{i}:{v}")
        body = " ".join(parts) if parts else "(其余全 1500)"
        return f"n={self.n} {body}"


class Sitl:
    def __init__(self, conn) -> None:
        self.c = conn
        self.statustexts: list[str] = []
        self.armed: bool | None = None

    # ---------- 收发基础 ----------
    @staticmethod
    def _pid(m) -> str:
        pid = m.param_id
        if isinstance(pid, bytes):
            pid = pid.split(b"\x00")[0].decode(errors="ignore")
        return pid

    def pump(self, obs: Obs | None = None) -> None:
        """吸收所有待处理报文; obs 非空时顺带把 SERVO_OUTPUT_RAW 记进去。"""
        while True:
            m = self.c.recv_match(blocking=False)
            if m is None:
                return
            t = m.get_type()
            if t == "SERVO_OUTPUT_RAW":
                if obs is not None:
                    obs.add(m)
            elif t == "STATUSTEXT":
                txt = m.text.decode(errors="replace") if isinstance(m.text, bytes) else m.text
                self.statustexts.append(txt)
            elif t == "HEARTBEAT" and m.get_srcSystem() == self.c.target_system:
                self.armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def heartbeat(self) -> None:
        self.c.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                  mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def manual_neutral(self) -> None:
        """中位伪手柄。

        被拒收的测试项 (错 group / NaN / 锁存期间...) 会落回原混控, 而 SITL 里没人喂
        RC, norm_input() 会钳到 -1 把桨推离中位 —— 那样"全 1500"会因为与本功能无关的
        原因报红。发中位 MANUAL_CONTROL 把混控钉在中位, 判据才干净。
        z=500 是油门中位 (固件里 channel[2] = 1500 + (z-500)*0.8)。
        """
        self.c.mav.manual_control_send(self.c.target_system, 0, 0, 500, 0, 0)

    def send_ext(self, controls, group: int = 1, target: int | None = None) -> None:
        vals = [float(v) for v in controls]
        vals += [0.0] * (NMOT - len(vals))
        self.c.mav.set_actuator_control_target_send(
            int(time.time() * 1e6) & 0xFFFFFFFF, group,
            self.c.target_system if target is None else target,
            self.c.target_component, vals)

    def run(self, seconds: float, controls=None, *, group: int = 1,
            target: int | None = None, settle: float = 0.0, hz: float = 25.0) -> Obs:
        """统一收发循环, 返回采样结果。

        controls=None 表示只发心跳/手柄、不发外部命令 (用来触发看门狗)。
        settle 这段时间照常发命令但**丢弃采样** —— 换命令的瞬间, 上一轮窗口残留在
        接收队列里的报文会被算到新窗口头上, 让"其余通道恒中位"假性失败。
        """
        obs = Obs()
        now = time.time()
        t_sample = now + settle
        t_end = t_sample + seconds
        next_cmd = next_manual = next_hb = now
        dt = 1.0 / hz
        while True:
            now = time.time()
            if now >= t_end:
                break
            if controls is not None and now >= next_cmd:
                self.send_ext(controls, group=group, target=target)
                next_cmd = now + dt
            if now >= next_manual:
                self.manual_neutral()
                next_manual = now + 0.1
            if now >= next_hb:
                self.heartbeat()
                next_hb = now + 0.5
            self.pump(obs if now >= t_sample else None)
            time.sleep(0.005)
        return obs

    # ---------- 参数 ----------
    def set_param(self, name: str, value: float, ptype=INT32) -> bool:
        for _ in range(8):
            self.c.mav.param_set_send(self.c.target_system, self.c.target_component,
                                      name.encode(), float(value), ptype)
            t_end = time.time() + 1.0
            while time.time() < t_end:
                self.heartbeat()
                m = self.c.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
                if m is not None and self._pid(m) == name and abs(m.param_value - value) < 1e-3:
                    return True
        return False

    def get_param(self, name: str) -> float | None:
        for _ in range(4):
            self.c.mav.param_request_read_send(self.c.target_system, self.c.target_component,
                                               name.encode(), -1)
            t_end = time.time() + 1.5
            while time.time() < t_end:
                self.heartbeat()
                m = self.c.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
                if m is not None and self._pid(m) == name:
                    return float(m.param_value)
        return None

    # ---------- 状态 ----------
    def arm(self, want: bool = True, timeout: float = 8.0) -> bool:
        t_end = time.time() + timeout
        last_send = 0.0
        while time.time() < t_end:
            if time.time() - last_send > 1.2:
                self.c.mav.command_long_send(
                    self.c.target_system, self.c.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    1 if want else 0, 0, 0, 0, 0, 0, 0)
                last_send = time.time()
            self.heartbeat()
            self.manual_neutral()
            self.pump()
            if self.armed == want:
                return True
            time.sleep(0.05)
        return self.armed == want

    def set_mode_manual(self) -> bool:
        t_end = time.time() + 5.0
        last_send = 0.0
        while time.time() < t_end:
            if time.time() - last_send > 1.0:
                self.c.mav.command_long_send(
                    self.c.target_system, self.c.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    MODE_MANUAL, 0, 0, 0, 0, 0)
                last_send = time.time()
            m = self.c.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)
            if m is not None and m.get_srcSystem() == self.c.target_system:
                self.armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if m.custom_mode == MODE_MANUAL:
                    return True
        return False

    def reset_ext(self) -> None:
        """回到干净起点: 无锁存 / 无残留命令 / MOT_EXT_ENABLE=1 / 未解锁。

        顺序是关键。固件里"解锁存"和"清残留命令"是两条独立路径:
            clear_external_fault()  只在 MOT_EXT_ENABLE==0 时被 50Hz 检查调用
            clear_external_thrust() 只在看门狗真正触发时调用 (它清 _ext_have_cmd)
        所以不能直接 ENABLE 0->1: 那样 _ext_have_cmd 仍是 true、_ext_last_ms 早已过期,
        重新使能的那一瞬 external_thrust_timed_out() 立刻为真 -> 20ms 内当场重新锁存,
        后续测试全部被"正确地"拒绝, 看起来像功能坏了。

        正确做法: 先让看门狗在 ENABLE 还开着的时候打一次 (它会清掉 _ext_have_cmd),
        再 ENABLE 0 -> 1。
        """
        self.set_param("MOT_EXT_ENABLE", 1, INT8)
        self.run(1.0)                                   # 静默 > TMOUT, 逼看门狗清残留
        self.set_param("MOT_EXT_ENABLE", 0, INT8)
        self.run(0.3)                                   # >1 个 50Hz 周期, 解锁存
        self.set_param("MOT_EXT_ENABLE", 1, INT8)
        self.arm(False)
        self.statustexts.clear()

    def arm_and_settle(self) -> bool:
        """解锁并等 spool up, 期间持续发零命令, 以免刚解锁就超时锁存。"""
        if not self.arm(True):
            return False
        self.run(1.2, [0.0] * NMOT)
        return True


def expected_pwm(thrust: float) -> int:
    return int(round(NEUTRAL + thrust * PWM_SPAN))


def one_hot(ch: int, value: float) -> list[float]:
    ctl = [0.0] * NMOT
    ctl[ch - 1] = value
    return ctl


def detect_motor_channels(s: Sitl) -> list[int]:
    """未解锁时被写成 1500 的通道才是真正使能的电机输出。

    未使能的通道 (比如 6 推进器帧下的 SERVO7/8) 从头到尾没人写, 读数恒 0。
    """
    window = 1.5
    obs = s.run(window)
    found = [i for i in range(1, NMOT + 1) if obs.seen[i] == {NEUTRAL}]
    # 采样率要一并报出来: 整套判据都靠 SERVO_OUTPUT_RAW 采样, 速率太低会让短窗口
    # (B4 每路只有 0.8s) 采不到几帧, 失败原因看起来会很莫名其妙
    info(f"SERVO_OUTPUT_RAW 采样率 ~{obs.n / window:.0f} Hz ({obs.n} 帧 / {window}s)")
    info(f"通道探测: {obs.dump()}")
    return found


def main() -> int:
    global MOTOR_CH

    print(f"[sitl] 连接 {CONN} ...")
    conn = mavutil.mavlink_connection(CONN, source_system=255)
    conn.wait_heartbeat()
    print(f"[sitl] heartbeat OK  sys={conn.target_system} comp={conn.target_component}")
    s = Sitl(conn)

    # SERVO_OUTPUT_RAW 属于 RC_CHANNELS 流, 默认速率不保证 -> 显式要 25Hz
    conn.mav.request_data_stream_send(conn.target_system, conn.target_component,
                                      mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 25, 1)
    conn.mav.request_data_stream_send(conn.target_system, conn.target_component,
                                      mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5, 1)

    # ================= 前置条件 =================
    print("\n========== 前置条件 (不计分, 不满足即中止) ==========")

    v_en = s.get_param("MOT_EXT_ENABLE")
    v_tm = s.get_param("MOT_EXT_TMOUT")
    if v_en is None or v_tm is None:
        return die("读不到 MOT_EXT_ENABLE / MOT_EXT_TMOUT —— SITL 跑的不是改版固件。"
                   "\n        重编: cd ~/rov-dev/ardupilot-external && "
                   "python3 ./waf configure --board sitl && python3 ./waf sub")
    info(f"改版固件参数在位: MOT_EXT_ENABLE={v_en:.0f} MOT_EXT_TMOUT={v_tm:.0f}")

    frame = s.get_param("FRAME_CONFIG")
    if frame is None or int(frame) != 2:
        return die(f"FRAME_CONFIG={frame} (需要 2 = Vectored_6DOF / 8 推进器)。"
                   "\n        SITL 默认是 6 推进器帧, Motor7/8 不被使能、SERVO7/8 恒 0,"
                   "\n        会把所有'全中位'判据带崩。请用 sitl_run.sh 启动。")
    info("FRAME_CONFIG=2 (Vectored_6DOF, 8 推进器) —— 与实机 M0 基线一致")

    ok = (s.set_param("ARMING_CHECK", 0, INT32)
          and s.set_param("SYSID_MYGCS", 255, INT32)
          and s.set_param("MOT_EXT_TMOUT", TMOUT_MS, INT16)
          and s.set_param("MOT_EXT_ENABLE", 0, INT8))
    if not ok:
        return die("参数写入失败")
    info(f"参数就绪: ARMING_CHECK=0 SYSID_MYGCS=255 MOT_EXT_TMOUT={TMOUT_MS}")

    if not s.set_mode_manual():
        return die("切 MANUAL 模式失败")
    info("模式 = MANUAL")

    boot = [t for t in s.statustexts if "EXT-THRUSTER" in t]
    info("开机标记 STATUSTEXT: " + (boot[0] if boot else
         "未捕获 —— 我们是 SITL 启动之后才连上的, 抓不到属正常; 真正验证留到刷机后"))

    print("\n[settle] 跳过启动瞬态 ...")
    s.arm(False)
    s.run(3.0)

    MOTOR_CH = detect_motor_channels(s)
    if MOTOR_CH != list(range(1, NMOT + 1)):
        return die(f"使能的电机通道 = {MOTOR_CH}, 期望 {list(range(1, NMOT + 1))}")
    info(f"使能电机通道 = {MOTOR_CH}")

    print("\n[基线] 关闭外部控制 + 解锁 + 中位手柄 -> 混控应输出全中位")
    if not s.arm(True):
        return die("基线解锁失败")
    base = s.run(2.0, settle=0.8)
    if not base.all_neutral():
        return die(f"基线不是全中位: {base.dump()}"
                   "\n        后面每一条判据都建立在'没有外部命令时 8 路恒 1500'之上,"
                   "\n        这一条不成立就无法区分是本功能的问题还是混控/手柄的问题。")
    info(f"基线全中位 OK (n={base.n})")
    s.arm(False)

    # ================= A =================
    print("\n========== A  未解锁时发非零外部命令 ==========")
    s.reset_ext()
    s.arm(False)
    obs = s.run(2.5, [0.6] * NMOT, settle=0.5)
    record("A  未解锁时 8 路保持安全中位", obs.all_neutral(), obs.dump(skip_neutral=True))

    # ================= B =================
    print("\n========== B  解锁后的单路独立性 ==========")
    s.reset_ext()
    if not s.arm_and_settle():
        record("B  解锁", False, "无法解锁, B1-B4 跳过")
    else:
        obs = s.run(2.0, one_hot(3, 0.6), settle=0.5)
        record("B1 只有 Motor3 输出变化", obs.only(3), obs.dump())

        peak = obs.peak(3)
        want = expected_pwm(0.6)
        record(f"B2 推力标度 SERVO3 ~= {want}", abs(peak - want) <= PWM_TOL,
               f"实测峰值 {peak} (容差 +-{PWM_TOL})")

        obs = s.run(2.0, one_hot(3, -0.6), settle=0.5)
        peak_n = obs.peak(3)
        want_n = expected_pwm(-0.6)
        record(f"B3 负向命令 SERVO3 ~= {want_n}",
               obs.only(3) and abs(peak_n - want_n) <= PWM_TOL,
               f"实测峰值 {peak_n}, 其余={obs.dump(skip_neutral=True)}")

        print("  -- B4 八路逐一扫描 (每路 0.5, 其余 0) --")
        sweep_ok, sweep_detail = True, []
        for ch in range(1, NMOT + 1):
            o = s.run(0.8, one_hot(ch, 0.5), settle=0.5)
            good = o.only(ch) and abs(o.peak(ch) - expected_pwm(0.5)) <= PWM_TOL
            sweep_ok = sweep_ok and good
            sweep_detail.append(f"M{ch}->{o.peak(ch)}" + ("" if good else "!"))
            print(f"     M{ch}: SERVO{ch} 峰值 {o.peak(ch)}  其余 "
                  + ("全中位" if o.only(ch) else o.dump(skip_neutral=True))
                  + ("  OK" if good else "  NG"))
        record("B4 八路逐一独立驱动", sweep_ok, " ".join(sweep_detail))

    # ================= C =================
    print("\n========== C  看门狗 (停发命令, 心跳仍在) ==========")
    s.reset_ext()
    if not s.arm_and_settle():
        record("C  解锁", False, "无法解锁, C1-C4 跳过")
    else:
        s.run(1.0, one_hot(3, 0.6))          # 先确认在控
        s.statustexts.clear()
        s.run(1.2)                           # 停发 > TMOUT, 等看门狗动作
        obs = s.run(1.0)
        texts = list(s.statustexts)
        armed_now = s.armed
        record("C1 看门狗归中位", obs.all_neutral(), obs.dump(skip_neutral=True))
        record("C2 看门狗自动 disarm", armed_now is False, f"armed={armed_now}")
        hit = [t for t in texts if "lost external thrust" in t.lower()]
        record("C3 看门狗告警 STATUSTEXT", bool(hit),
               hit[0] if hit else f"收到的是 {texts[-3:]}")

        # 锁存: 重新解锁后发合法命令也必须被拒。
        # 必须把"解锁成功"算进判据 —— 否则解锁失败时输出也是全 1500,
        # 这一项会以错误的理由 PASS。
        rearmed = s.arm(True)
        obs = s.run(2.0, one_hot(3, 0.6), settle=0.5)
        record("C4 故障锁存期间合法命令被拒", rearmed and obs.all_neutral(),
               f"重新解锁={rearmed} {obs.dump(skip_neutral=True)}")

    # ================= D =================
    print("\n========== D  非法/越权输入拒收 ==========")

    def reject_case(tag: str, name: str, controls, **kw) -> None:
        s.reset_ext()
        if not s.arm_and_settle():
            record(f"{tag} {name}", False, "无法解锁")
            return
        o = s.run(2.0, controls, settle=0.5, **kw)
        record(f"{tag} {name}", o.all_neutral(),
               f"SERVO3={o.vals(3)} {o.dump(skip_neutral=True)}")

    reject_case("D1", "group_mlx=0 (标准组) 拒收", one_hot(3, 0.6), group=0)
    bad_nan = one_hot(3, 0.6)
    bad_nan[5] = float("nan")
    reject_case("D2", "含 NaN 整帧拒收", bad_nan)
    bad_big = one_hot(3, 0.6)
    bad_big[5] = 5.0
    reject_case("D3", "含越范围值整帧拒收", bad_big)
    reject_case("D4", "错误 target_system 拒收", one_hot(3, 0.6), target=99)

    print("\n[D5] 先制造一次锁存, 再按文档流程 (清 MOT_EXT_ENABLE 再开) 恢复")
    s.reset_ext()
    if not s.arm_and_settle():
        record("D5 锁存后按流程恢复可控", False, "无法解锁")
    else:
        s.run(1.0, one_hot(3, 0.6))
        s.run(1.5)                           # 触发并锁存
        latched = s.run(1.5, one_hot(3, 0.6), settle=0.3)
        s.reset_ext()                        # 文档规定的恢复动作
        if not s.arm_and_settle():
            record("D5 锁存后按流程恢复可控", False, "恢复后无法解锁")
        else:
            obs = s.run(2.0, one_hot(3, 0.6), settle=0.5)
            record("D5 锁存后按流程恢复可控", obs.only(3),
                   f"锁存期间 SERVO3={latched.vals(3)} -> 恢复后 SERVO3={obs.vals(3)}")

    # ================= 收尾 =================
    s.arm(False)
    s.set_param("MOT_EXT_ENABLE", 0, INT8)

    print("\n" + "=" * 64)
    npass = sum(1 for _, ok_, _ in results if ok_)
    for name, ok_, _detail in results:
        print(f"  [{'PASS' if ok_ else 'FAIL'}] {name}")
    print(f"\n总计 {npass}/{len(results)} 通过")
    if npass != len(results):
        print("\n失败项明细:")
        for name, ok_, detail in results:
            if not ok_:
                print(f"  - {name}: {detail}")
    return 0 if npass == len(results) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n中断")
        sys.exit(130)
