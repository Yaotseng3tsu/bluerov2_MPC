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

前置条件 (不计分, 不满足直接 abort —— 教训是前提错了还照常跑完, 会产出一堆
看不懂的 FAIL):
  * MOT_EXT_ENABLE / MOT_EXT_TMOUT 存在 (证明跑的是改版固件)
  * FRAME_CONFIG == 2
  * 未解锁时探测到的电机通道恰好是 {1..8}
  * 基线: 关掉外部控制 + 解锁 + 中位手柄 -> 8 路全 1500

### 关于"链路空洞"

2026-10-06 第二轮跑出 11/14, 三条失败全部是同一个原因: 某些采样窗口一帧
SERVO_OUTPUT_RAW 都没收到(2.5s 窗口 n=0), 而正常窗口稳定在满速率。这不是采样率
不够, 是链路整段停了, 而且是**双向**的 —— B4 后四路的现象(数据恢复了但命令不
生效、全 1500)说明我们的 SET_ACTUATOR_CONTROL_TARGET 也有 >MOT_EXT_TMOUT 没送达,
看门狗**正确地**触发并锁存了。也就是说那三条失败都不是固件问题。

所以本脚本:
  * 每个窗口统计帧数 / 实际速率 / 最大帧间隔 / 发出的命令数, 一并打进判据详情
  * 窗口若出现空洞(无样本, 或最大帧间隔 > GAP_LIMIT)就判为**不可信**, 自动
    reset_ext() 后重跑该窗口; 有状态的 C / D5 整段重跑
  * 判据函数一律要求窗口可信 —— 宁可报 FAIL 也不要在有空洞的数据上给 PASS
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

# 所有计时一律用单调时钟。墙钟(time.time())在 WSL2 下会前后跳数秒,
# 用它量间隔会把窗口逻辑整个带偏 —— 2026-10-07 实测单次跳变约 7.9s。
mono = time.monotonic

CONN = "tcp:127.0.0.1:5760"
NMOT = 8
NEUTRAL = 1500
PWM_SPAN = 400.0        # calc_thrust_to_pwm(): 1500 + thrust * 400
PWM_TOL = 30            # 推力标度判据的容差
TMOUT_MS = 500          # MOT_EXT_TMOUT
MODE_MANUAL = 19        # ArduSub custom mode

# 发包速率。第二轮用 ext 25Hz + 手柄 10Hz + 心跳 2Hz + 循环 200Hz 时出现过数秒的
# 链路空洞; 首轮(无手柄、心跳 1Hz、循环 25Hz)从未出现。先把密度降回那个量级。
EXT_HZ = 20.0
MANUAL_PERIOD = 0.2     # 5 Hz
HB_PERIOD = 1.0         # 1 Hz
LOOP_SLEEP = 0.01

# 帧间隔超过这个值就认为链路有空洞。正常间隔约 1/EXT_HZ = 50ms;
# 阈值取得比 MOT_EXT_TMOUT(500ms) 略大 —— 空洞一旦超过它, 外发的命令多半也断了,
# 看门狗会锁存, 这一窗口的数据无论如何都不能用来判功能对错。
GAP_LIMIT = 0.6

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


def warn(text: str) -> None:
    print(f"  [WARN] {time.strftime('%H:%M:%S')} {text}")


def die(msg: str) -> int:
    print(f"\n  [ABORT] {msg}")
    print("\n前置条件不满足, 不再往下跑 —— 继续跑只会产生一堆与本功能无关的 FAIL。")
    return 2


class Obs:
    """一次采样窗口: 每个通道出现过的 PWM 值, 外加链路健康度统计。"""

    def __init__(self) -> None:
        self.seen: dict[int, set[int]] = {i: set() for i in range(1, NMOT + 1)}
        self.n = 0
        self.sent = 0            # 本窗口发出的外部命令帧数
        self.dur = 0.0
        self.max_gap = 0.0       # 相邻两帧 SERVO_OUTPUT_RAW 的最大间隔
        self.gap_at = 0.0        # 最大间隔出现时的墙钟时间
        self.max_loop = 0.0      # 本进程两次循环迭代之间的最大间隔
        self.clock_jump = 0.0    # 窗口内墙钟相对单调时钟多走/少走了多少
        self.attempt = 1
        self._t_prev = 0.0
        self._t_begin = 0.0

    # ---- 采样窗口的生命周期 ----
    def begin(self, now: float) -> None:
        self._t_begin = now
        self._t_prev = now

    def add(self, msg, now: float) -> None:
        gap = now - self._t_prev
        if gap > self.max_gap:
            self.max_gap = gap
            self.gap_at = now
        self._t_prev = now
        for i in range(1, NMOT + 1):
            self.seen[i].add(getattr(msg, f"servo{i}_raw"))
        self.n += 1

    def end(self, now: float) -> None:
        # 窗口尾部的空档同样算空洞 (n=0 时这一步让 max_gap = 整个窗口长度)
        gap = now - self._t_prev
        if gap > self.max_gap:
            self.max_gap = gap
            self.gap_at = now
        self.dur = now - self._t_begin

    # ---- 可信度 ----
    def valid(self) -> bool:
        return self.n >= 5 and self.max_gap <= GAP_LIMIT

    def frozen(self) -> bool:
        """空洞期间我们自己的循环是不是也停了。

        这是区分两类故障的关键。循环照常跑、只是没收到数据 = 链路/SITL 的问题；
        循环本身也卡了同样久 = 本进程(多半是整个 WSL2 VM)被宿主冻结了 —— 那么
        SITL 在同一个 VM 里一起停, 恢复时它的 millis() 同样跳过了 MOT_EXT_TMOUT,
        看门狗会**正确地**锁存。这种窗口与固件对错无关, 只能重采。
        """
        return self.max_gap > GAP_LIMIT and self.max_loop >= 0.5 * self.max_gap

    def stats(self) -> str:
        hz = self.n / self.dur if self.dur > 0 else 0.0
        retry = "" if self.attempt == 1 else f" 重试{self.attempt - 1}次"
        # 墙钟跳变单独报出来: 计时已全部改用单调时钟, 所以跳变不再影响判据,
        # 但它是 WSL 环境有问题的直接证据, 要让它留在日志里。
        jump = "" if abs(self.clock_jump) < 0.5 else f" 墙钟跳变={self.clock_jump:+.2f}s"
        if self.valid():
            tag = ""
        elif self.frozen():
            tag = "  <<进程被冻结(WSL VM 挂起), 判据不可信"
        else:
            tag = "  <<链路无数据, 判据不可信"
        return (f"[n={self.n} {hz:.1f}Hz 最大帧间隔={self.max_gap:.2f}s "
                f"循环卡顿<={self.max_loop:.2f}s tx={self.sent}{jump}{retry}]{tag}")

    # ---- 判据 (一律要求窗口可信) ----
    def vals(self, ch: int) -> list[int]:
        return sorted(self.seen.get(ch, set()))

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
        return self.valid() and all(self.seen.get(i) == {NEUTRAL} for i in MOTOR_CH)

    def only(self, ch: int) -> bool:
        """只有 ch 离开过中位, 其余使能通道恒 1500。"""
        if not self.valid() or not self.moved(ch):
            return False
        return all(self.seen.get(i) == {NEUTRAL} for i in MOTOR_CH if i != ch)

    def dump(self, skip_neutral: bool = False) -> str:
        parts = []
        for i in MOTOR_CH:
            v = self.vals(i)
            if skip_neutral and v == [NEUTRAL]:
                continue
            parts.append(f"{i}:{v}")
        return " ".join(parts) if parts else "(其余全 1500)"


class Sitl:
    def __init__(self, conn) -> None:
        self.c = conn
        self.statustexts: list[str] = []
        # statustexts 会被 reset_ext() 清掉, 但排查解锁失败时需要看之前的告警,
        # 所以另存一份从不清空的
        self.all_texts: list[str] = []
        self.last_ack = None
        self.armed: bool | None = None
        self._next_hb = 0.0
        self._next_manual = 0.0

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
                    obs.add(m, mono())
            elif t == "STATUSTEXT":
                txt = m.text.decode(errors="replace") if isinstance(m.text, bytes) else m.text
                self.statustexts.append(txt)
                self.all_texts.append(txt)
            elif t == "COMMAND_ACK":
                self.last_ack = m
            elif t == "HEARTBEAT" and m.get_srcSystem() == self.c.target_system:
                self.armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def tick_hb(self) -> None:
        """限速心跳。各处循环都走这里, 免得 arm()/set_param() 里按循环频率狂发。"""
        now = mono()
        if now >= self._next_hb:
            self.c.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._next_hb = now + HB_PERIOD

    def tick_manual(self) -> None:
        """限速中位伪手柄。

        被拒收的测试项 (错 group / NaN / 锁存期间...) 会落回原混控, 而 SITL 里没人喂
        RC, norm_input() 会钳到 -1 把桨推离中位 —— 那样"全 1500"会因为与本功能无关的
        原因报红。发中位 MANUAL_CONTROL 把混控钉在中位, 判据才干净。
        z=500 是油门中位 (固件里 channel[2] = 1500 + (z-500)*throttleScale)。
        """
        now = mono()
        if now >= self._next_manual:
            self.c.mav.manual_control_send(self.c.target_system, 0, 0, 500, 0, 0)
            self._next_manual = now + MANUAL_PERIOD

    def send_ext(self, controls, group: int = 1, target: int | None = None) -> None:
        vals = [float(v) for v in controls]
        vals += [0.0] * (NMOT - len(vals))
        self.c.mav.set_actuator_control_target_send(
            int(time.time() * 1e6) & 0xFFFFFFFF, group,
            self.c.target_system if target is None else target,
            self.c.target_component, vals)

    def run(self, seconds: float, controls=None, *, group: int = 1,
            target: int | None = None, settle: float = 0.0) -> Obs:
        """统一收发循环, 返回采样结果。

        controls=None 表示只发心跳/手柄、不发外部命令 (用来触发看门狗)。
        settle 这段时间照常发命令但**丢弃采样** —— 换命令的瞬间, 上一窗口残留在
        接收队列里的报文会被算到新窗口头上, 让"其余通道恒中位"假性失败。
        """
        obs = Obs()
        now = mono()
        wall0 = time.time()                   # 只为统计墙钟跳变, 不参与任何判断
        t_sample = now + settle
        t_end = t_sample + seconds
        next_cmd = now
        dt = 1.0 / EXT_HZ
        sampling = False
        prev_iter = now
        while True:
            now = mono()
            # 本进程的调度卡顿。和 max_gap 一起看才能分清"数据没来"和"我们停了"
            if now - prev_iter > obs.max_loop:
                obs.max_loop = now - prev_iter
            prev_iter = now
            if now >= t_end:
                break
            if not sampling and now >= t_sample:
                obs.begin(now)
                sampling = True
            if controls is not None and now >= next_cmd:
                self.send_ext(controls, group=group, target=target)
                obs.sent += 1
                next_cmd = now + dt
            self.tick_manual()
            self.tick_hb()
            self.pump(obs if sampling else None)
            time.sleep(LOOP_SLEEP)
        if not sampling:                      # 窗口太短没进采样段, 不该发生
            obs.begin(t_sample)
        end_mono = mono()
        obs.clock_jump = (time.time() - wall0) - (end_mono - (t_sample - settle))
        obs.end(end_mono)
        return obs

    def window(self, seconds: float, controls=None, *, settle: float = 0.5,
               rearm: bool = True, tries: int = 3, **kw) -> Obs:
        """跑一个采样窗口, 对链路空洞自愈。

        空洞期间我们发出的命令多半也丢了 -> 看门狗会锁存 -> 这一窗口既测不出功能,
        又会把锁存状态带给下一项。所以空洞要当基础设施故障处理: reset 后重跑,
        而不是当成功能失败记一条红。
        """
        obs = Obs()
        for attempt in range(1, tries + 1):
            obs = self.run(seconds, controls, settle=settle, **kw)
            obs.attempt = attempt
            if obs.valid():
                return obs
            warn(f"链路空洞 {obs.stats()} —— reset 后重跑 (第 {attempt}/{tries} 次)")
            if attempt < tries:
                self.reset_ext()
                if rearm and not self.arm_and_settle():
                    warn(f"重跑前解锁失败 —— {self.arm_detail()}")
                    break
        return obs

    # ---------- 参数 ----------
    def set_param(self, name: str, value: float, ptype=INT32) -> bool:
        for _ in range(8):
            self.c.mav.param_set_send(self.c.target_system, self.c.target_component,
                                      name.encode(), float(value), ptype)
            t_end = mono() + 1.0
            while mono() < t_end:
                self.tick_hb()
                m = self.c.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
                if m is not None and self._pid(m) == name and abs(m.param_value - value) < 1e-3:
                    return True
        return False

    def get_param(self, name: str) -> float | None:
        for _ in range(4):
            self.c.mav.param_request_read_send(self.c.target_system, self.c.target_component,
                                               name.encode(), -1)
            t_end = mono() + 1.5
            while mono() < t_end:
                self.tick_hb()
                m = self.c.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
                if m is not None and self._pid(m) == name:
                    return float(m.param_value)
        return None

    # ---------- 状态 ----------
    def arm(self, want: bool = True, timeout: float = 8.0) -> bool:
        t_end = mono() + timeout
        last_send = 0.0
        while mono() < t_end:
            if mono() - last_send > 1.2:
                self.c.mav.command_long_send(
                    self.c.target_system, self.c.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    1 if want else 0, 0, 0, 0, 0, 0, 0)
                last_send = mono()
            self.tick_hb()
            self.tick_manual()
            self.pump()
            if self.armed == want:
                return True
            time.sleep(0.05)
        return self.armed == want

    def set_mode_manual(self) -> bool:
        t_end = mono() + 5.0
        last_send = 0.0
        while mono() < t_end:
            if mono() - last_send > 1.0:
                self.c.mav.command_long_send(
                    self.c.target_system, self.c.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    MODE_MANUAL, 0, 0, 0, 0, 0)
                last_send = mono()
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

    def latched_text(self) -> bool:
        return any("lost external thrust" in t.lower() for t in self.statustexts)

    def arm_detail(self) -> str:
        """解锁失败时把飞控给的理由带出来。

        之前"无法解锁"只有四个字, 看不出是预解锁检查拦的、还是处在某个 failsafe、
        还是命令根本没被受理 —— 排查全靠猜。
        """
        ack = "无 COMMAND_ACK"
        m = self.last_ack
        if m is not None:
            name = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
                    3: "UNSUPPORTED", 4: "FAILED"}.get(m.result, str(m.result))
            ack = f"COMMAND_ACK(cmd={m.command}) = {name}"
        return f"armed={self.armed} {ack} 最近告警={self.all_texts[-5:]}"


def expected_pwm(thrust: float) -> int:
    return int(round(NEUTRAL + thrust * PWM_SPAN))


def one_hot(ch: int, value: float) -> list[float]:
    ctl = [0.0] * NMOT
    ctl[ch - 1] = value
    return ctl


def stable(s: Sitl, label: str, seconds: float, controls=None, *,
           settle: float = 0.0, tries: int = 4) -> Obs:
    """前置条件用的重采窗口。

    前置条件是 abort 级的, 所以绝不能让一次 VM 冻结把整轮验收否掉 —— 冻结是
    环境问题, 不是"基线不成立"。注意这里不做 reset_ext: 前置阶段外部控制还没开,
    没有锁存可清。
    """
    obs = Obs()
    for attempt in range(1, tries + 1):
        obs = s.run(seconds, controls, settle=settle)
        obs.attempt = attempt
        if obs.valid():
            return obs
        warn(f"{label} {obs.stats()} —— 重采 ({attempt}/{tries})")
    return obs


def detect_motor_channels(s: Sitl) -> list[int]:
    """未解锁时被写成 1500 的通道才是真正使能的电机输出。

    未使能的通道 (比如 6 推进器帧下的 SERVO7/8) 从头到尾没人写, 读数恒 0。
    """
    obs = stable(s, "通道探测", 1.5)
    found = [i for i in range(1, NMOT + 1) if obs.seen[i] == {NEUTRAL}]
    info(f"SERVO_OUTPUT_RAW {obs.stats()}")
    info(f"通道探测: {obs.dump()}")
    return found


def main() -> int:
    global MOTOR_CH

    print(f"[sitl] 连接 {CONN} ...")
    conn = mavutil.mavlink_connection(CONN, source_system=255)
    conn.wait_heartbeat()
    print(f"[sitl] heartbeat OK  sys={conn.target_system} comp={conn.target_component}")
    s = Sitl(conn)

    # SERVO_OUTPUT_RAW 属于 RC_CHANNELS 流, 默认速率不保证 -> 显式要
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

    # SERVOn_FUNCTION 必须恰好是 Motor1..8 (k_motor1..k_motor8 = 33..40)。
    # 2026-10-07 B4 里只有 M8 不响应、其余七路全部准确命中, 窗口统计还干干净净,
    # 查下来怀疑是这里: add_motor_num() 只用 set_aux_channel_default() 装"默认值",
    # 被显式设过的 SERVOn_FUNCTION 会赢 -> 那个电机根本没有输出通道, 而对应的
    # SERVO 显示的是另一个功能(停在 1500), 看起来就像"这一路驱动不了"。
    # 实机 M0 基线就是 SERVO1-8 = Motor1-8, 所以这也是在强制 SITL 对齐实机。
    funcs = [s.get_param(f"SERVO{ch}_FUNCTION") for ch in range(1, NMOT + 1)]
    funcs = [None if v is None else int(v) for v in funcs]
    want = list(range(33, 33 + NMOT))
    info(f"SERVO1-8_FUNCTION = {funcs}  (期望 {want} = Motor1..Motor8)")
    if funcs != want:
        bad = ", ".join(f"SERVO{i + 1}={funcs[i]}" for i in range(NMOT) if funcs[i] != want[i])
        return die(f"SERVOn_FUNCTION 没有全部指向 Motor1..8: {bad}"
                   "\n        这一路的电机没有输出通道, 外部命令发了也驱动不了,"
                   "\n        而该 SERVO 读数会停在别的功能的中位, 看起来像功能坏了。"
                   "\n        sitl_ext.parm 里已钉死 33..40; 若仍不符, 说明 defaults 没生效。")

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
        return die(f"使能的电机通道 = {MOTOR_CH}, 期望 {list(range(1, NMOT + 1))}"
                   "\n        (若全为空, 说明一帧 SERVO_OUTPUT_RAW 都没收到 —— 查链路)")
    info(f"使能电机通道 = {MOTOR_CH}")

    print("\n[基线] 关闭外部控制 + 解锁 + 中位手柄 -> 混控应输出全中位")
    if not s.arm(True):
        return die("基线解锁失败")
    base = stable(s, "基线", 2.0, settle=0.8)
    if not base.valid():
        return die(f"基线窗口重采 4 次仍不可信: {base.stats()}"
                   "\n        这是环境问题不是功能问题: 本进程(多半是整个 WSL2 VM)被宿主"
                   "\n        冻结了数秒。SITL 在同一个 VM 里一起停, 没法做有意义的计时验收。"
                   "\n        建议: 关掉占满 CPU/磁盘的程序(含杀毒实时扫描)后重跑;"
                   "\n        若反复出现, 考虑给 WSL 固定内存与 CPU 配额(.wslconfig)。")
    if not base.all_neutral():
        return die(f"基线不是全中位: {base.stats()} {base.dump()}"
                   "\n        后面每一条判据都建立在'没有外部命令时 8 路恒 1500'之上,"
                   "\n        这一条不成立就无法区分是本功能的问题还是混控/手柄的问题。")
    info(f"基线全中位 OK {base.stats()}")
    s.arm(False)

    # ================= A =================
    print("\n========== A  未解锁时发非零外部命令 ==========")
    s.reset_ext()
    s.arm(False)
    obs = s.window(2.5, [0.6] * NMOT, rearm=False)
    record("A  未解锁时 8 路保持安全中位", obs.all_neutral(),
           f"{obs.stats()} {obs.dump(skip_neutral=True)}")

    # ================= B =================
    print("\n========== B  解锁后的单路独立性 ==========")
    s.reset_ext()
    if not s.arm_and_settle():
        record("B  解锁", False, f"无法解锁, B1-B4 跳过 —— {s.arm_detail()}")
    else:
        obs = s.window(2.0, one_hot(3, 0.6))
        record("B1 只有 Motor3 输出变化", obs.only(3), f"{obs.stats()} {obs.dump()}")

        peak, want = obs.peak(3), expected_pwm(0.6)
        record(f"B2 推力标度 SERVO3 ~= {want}",
               obs.valid() and abs(peak - want) <= PWM_TOL,
               f"实测峰值 {peak} (容差 +-{PWM_TOL}) {obs.stats()}")

        obs = s.window(2.0, one_hot(3, -0.6))
        peak_n, want_n = obs.peak(3), expected_pwm(-0.6)
        record(f"B3 负向命令 SERVO3 ~= {want_n}",
               obs.only(3) and abs(peak_n - want_n) <= PWM_TOL,
               f"实测峰值 {peak_n} {obs.stats()} 其余={obs.dump(skip_neutral=True)}")

        print("  -- B4 八路逐一扫描 (每路 0.5, 其余 0) --")
        sweep_ok, sweep_detail = True, []
        for ch in range(1, NMOT + 1):
            o = s.window(0.8, one_hot(ch, 0.5))
            good = o.only(ch) and abs(o.peak(ch) - expected_pwm(0.5)) <= PWM_TOL
            sweep_ok = sweep_ok and good
            sweep_detail.append(f"M{ch}->{o.peak(ch)}" + ("" if good else "!"))
            print(f"     M{ch}: SERVO{ch} 峰值 {o.peak(ch)}  其余 "
                  + ("全中位" if o.only(ch) else o.dump(skip_neutral=True))
                  + ("  OK" if good else "  NG") + f"  {o.stats()}")
            if not good and s.latched_text():
                # 空洞期间命令断流 -> 看门狗锁存 -> 后续各路都会被"正确地"拒绝。
                # 必须在这里清掉, 否则一次抖动会连累剩下所有通道。
                warn(f"M{ch} 期间出现锁存告警, reset 后继续扫描")
                s.reset_ext()
                s.arm_and_settle()
        record("B4 八路逐一独立驱动", sweep_ok, " ".join(sweep_detail))

    # ================= C =================
    print("\n========== C  看门狗 (停发命令, 心跳仍在) ==========")

    def run_c():
        """C 是有状态的序列, 中途有空洞只能整段重跑。"""
        s.reset_ext()
        if not s.arm_and_settle():
            return None
        s.run(1.0, one_hot(3, 0.6))      # 先确认在控
        s.statustexts.clear()
        s.run(1.2)                       # 停发 > TMOUT, 等看门狗动作
        after = s.run(1.0)               # 采归中位
        texts = list(s.statustexts)
        armed_now = s.armed
        # 锁存: 重新解锁后发合法命令也必须被拒。
        # 必须把"解锁成功"算进判据 —— 否则解锁失败时输出也是全 1500, 会假 PASS。
        rearmed = s.arm(True)
        held = s.run(2.0, one_hot(3, 0.6), settle=0.5)
        return after, texts, armed_now, rearmed, held

    c = None
    for attempt in range(1, 4):
        c = run_c()
        if c is None:
            # 解锁失败同样重试: 上一轮就是在一次冻结之后解锁不上, 整个 C 段被跳过
            warn(f"C 段解锁失败 ({s.arm_detail()}) —— 整段重跑 (第 {attempt}/3 次)")
            continue
        if c[0].valid() and c[4].valid():
            break
        warn(f"C 段窗口不可信 (归中 {c[0].stats()} / 锁存 {c[4].stats()})"
             f" —— 整段重跑 (第 {attempt}/3 次)")
    if c is None:
        record("C  解锁", False, f"重试 3 次仍无法解锁 —— {s.arm_detail()}")
    else:
        after, texts, armed_now, rearmed, held = c
        record("C1 看门狗归中位", after.all_neutral(),
               f"{after.stats()} {after.dump(skip_neutral=True)}")
        record("C2 看门狗自动 disarm", armed_now is False, f"armed={armed_now}")
        hit = [t for t in texts if "lost external thrust" in t.lower()]
        record("C3 看门狗告警 STATUSTEXT", bool(hit),
               hit[0] if hit else f"收到的是 {texts[-3:]}")
        record("C4 故障锁存期间合法命令被拒", rearmed and held.all_neutral(),
               f"重新解锁={rearmed} {held.stats()} {held.dump(skip_neutral=True)}")

    # ================= D =================
    print("\n========== D  非法/越权输入拒收 ==========")

    def reject_case(tag: str, name: str, controls, **kw) -> None:
        s.reset_ext()
        if not s.arm_and_settle():
            record(f"{tag} {name}", False, f"无法解锁 —— {s.arm_detail()}")
            return
        o = s.window(2.0, controls, **kw)
        record(f"{tag} {name}", o.all_neutral(),
               f"SERVO3={o.vals(3)} {o.stats()} {o.dump(skip_neutral=True)}")

    reject_case("D1", "group_mlx=0 (标准组) 拒收", one_hot(3, 0.6), group=0)
    bad_nan = one_hot(3, 0.6)
    bad_nan[5] = float("nan")
    reject_case("D2", "含 NaN 整帧拒收", bad_nan)
    bad_big = one_hot(3, 0.6)
    bad_big[5] = 5.0
    reject_case("D3", "含越范围值整帧拒收", bad_big)
    reject_case("D4", "错误 target_system 拒收", one_hot(3, 0.6), target=99)

    print("\n[D5] 先制造一次锁存, 再按文档流程 (清 MOT_EXT_ENABLE 再开) 恢复")

    def run_d5():
        s.reset_ext()
        if not s.arm_and_settle():
            return None
        s.run(1.0, one_hot(3, 0.6))
        s.run(1.5)                                       # 触发并锁存
        latched = s.run(1.5, one_hot(3, 0.6), settle=0.3)
        s.reset_ext()                                    # 文档规定的恢复动作
        if not s.arm_and_settle():
            return None
        back = s.run(2.0, one_hot(3, 0.6), settle=0.5)
        return latched, back

    d5 = None
    for attempt in range(1, 4):
        d5 = run_d5()
        if d5 is None:
            break
        if d5[1].valid():
            break
        warn(f"D5 出现链路空洞 (恢复窗口 {d5[1].stats()}) —— 整段重跑 (第 {attempt}/3 次)")
    if d5 is None:
        record("D5 锁存后按流程恢复可控", False, f"无法解锁 —— {s.arm_detail()}")
    else:
        latched, back = d5
        record("D5 锁存后按流程恢复可控", back.only(3),
               f"锁存期间 SERVO3={latched.vals(3)} -> 恢复后 SERVO3={back.vals(3)} "
               f"{back.stats()}")

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
        print("\n注: 详情里带 '<<' 标记的, 是重试后窗口仍不可信。")
        print("    进程被冻结 = WSL VM 被宿主挂起, 固件侧无从验证, 不构成固件失败的证据;")
        print("    链路无数据 = 我们还在跑但收不到报文, 那才需要查 SITL/链路。")
    return 0 if npass == len(results) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n中断")
        sys.exit(130)
