#!/usr/bin/env python3
"""上位机 — 对改版 ArduSub 直接下发 8 路归一化推力 (direct_thruster 线的发送端)。

这是 direct_thruster 的"水面那一半"。改版固件负责**接收、校验、执行、保护**(M3 实现,
M4 在官方 SITL 上 14/14 验收通过); 本文件负责**发送与上位机侧的安全外壳**。
它不是 MPC 也不是推力分配器 —— 那些在更上层, 改它们不需要重编固件。

    MPC → 推力分配器 → 8 路归一化命令
                        │  ← 本文件
                        ▼  SET_ACTUATOR_CONTROL_TARGET, group_mlx=1
            改版 ArduSub: 校验 → 替换混控结果 → 原有电机输出层 → ESC → 8×T200

接口语义: controls[i] ∈ [-1, 1] 对应 Motor(i+1), 是**归一化执行器命令, 不是牛顿**。
分配器算出的 fᵢ 需经推力标定才能转到这一层(尚未做)。

与 waypoint 线 `src/pseudo_stick.py` 的关系: 同一个位置、低一层。那边发 MANUAL_CONTROL
给官方固件的混控分配矩阵, 这边直接给每个桨。安全范式(限幅 / 看门狗 / 退出先回中位再
自动上锁 / 默认不解锁)照搬, 但有三处**必须**不同:

  1. **持续发送是硬要求, 不是好习惯。** MANUAL_CONTROL 停发只是不动; 外部直控停发超过
     MOT_EXT_TMOUT(默认 500ms), 固件会归中 + 自动 disarm + **锁存故障**。所以 keepalive
     线程是强制的。
  2. **要管 MOT_EXT_ENABLE。** 进入时置 1、退出时置 0 —— 置 0 同时也是官方的解锁存动作。
  3. **清锁存有顺序陷阱。** 不能直接 ENABLE 0→1: 固件里清 _ext_have_cmd 的
     clear_external_thrust() **只在看门狗真正触发时**才调用, 于是重新使能那一瞬
     external_thrust_timed_out() 立刻为真 → 当场重新锁存。正确顺序见 reset_latch()。

⚠️ 默认指向 **SITL** (与本目录其它只读工具不同 —— 它们默认连真机, 但本文件会驱动推进器)。
   连真机要显式写: --endpoint udpin:0.0.0.0:14550

用法:
  # SITL (先 bash direct_thruster/sitl_run.sh 起 SITL, 或自己起一个)
  python direct_thruster/external_thruster.py --zero --seconds 5
  python direct_thruster/external_thruster.py --arm --motor 3 --thrust 0.3 --seconds 3
  python direct_thruster/external_thruster.py --arm --sweep --thrust 0.3

  # 真机干测 (需先刷改版固件; 建议首轮 ±0.1、每次 1 秒)
  python direct_thruster/external_thruster.py --endpoint udpin:0.0.0.0:14550 \\
      --arm --u-max 0.1 --sweep --thrust 0.1 --dwell 1.0

作为库:
  from external_thruster import ExternalThruster, connect
  with ExternalThruster(connect("tcp:127.0.0.1:5760"), u_max=0.3) as ext:
      ext.arm()
      ext.set_thrust([0, 0, 0.3, 0, 0, 0, 0, 0])
      time.sleep(2)
  # 退出时自动: 归零 → disarm → MOT_EXT_ENABLE=0
"""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

from pymavlink import mavutil

# 计时一律用单调时钟。墙钟在 WSL2 下实测会前后跳 ±8 秒, 用它量间隔会把看门狗的
# 节奏算错 —— M4 为这个坑折腾了一整轮, 详见 PROGRESS.md。
mono = time.monotonic

NMOT = 8
SITL_ENDPOINT = "tcp:127.0.0.1:5760"
ROV_ENDPOINT = "udpin:0.0.0.0:14550"
GROUP_MOTORS = 1          # 本项目私有约定: group_mlx=1 表示 Motor1..8
MODE_MANUAL = 19          # ArduSub custom mode
ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

INT8 = mavutil.mavlink.MAV_PARAM_TYPE_INT8
INT16 = mavutil.mavlink.MAV_PARAM_TYPE_INT16

LATCH_TEXT = "lost external thrust"


def connect(endpoint: str = SITL_ENDPOINT, timeout_s: float = 12.0, verbose: bool = True):
    """连接并锁定真飞控心跳。

    SITL 上只有一个系统, 直接连即可; 真机上网络里还有 BlueOS 的板载服务(autopilot=INVALID)
    和 GCS 心跳, 朴素的 wait_heartbeat() 会锁到 sys=0 导致之后参数一个都读不出来, 所以
    走 fc_link.connect_fc() 的判据。
    """
    if endpoint.startswith("tcp:"):
        try:
            conn = mavutil.mavlink_connection(endpoint, dialect="ardupilotmega",
                                              source_system=255)
        except Exception as e:
            print(f"[link] ❌ 连不上 {endpoint}: {e}")
            print("[link]    SITL 没在跑? 先: bash direct_thruster/sitl_run.sh --sitl-only")
            return None
        if verbose:
            print(f"[link] 等待 heartbeat ({endpoint}) ...")
        # 必须带 timeout。SITL 的 5760 一次只接一个客户端, 多接的那个会被当场关掉,
        # 而 pymavlink 会无限重连并刷屏 "EOF on TCP socket" —— 不限时就永远卡在这。
        if conn.wait_heartbeat(timeout=timeout_s) is None:
            print(f"\n[link] ❌ {timeout_s:.0f}s 内没收到 heartbeat。")
            print("[link]    若刚才一直刷 'EOF on TCP socket': SITL 的 5760 一次只接一个")
            print("[link]    客户端, 多半是已经有别的程序连着(上一次没退干净的脚本)。")
            print("[link]    排查:  pgrep -af bin/ardusub   和   ss -tn | grep 5760")
            return None
        if verbose:
            print(f"[link] OK  sys={conn.target_system} comp={conn.target_component}")
        return conn
    from fc_link import connect_fc
    return connect_fc(endpoint, timeout_s=timeout_s, verbose=verbose)


class ThrustRejected(ValueError):
    """上位机侧就拦下来的非法命令。

    固件对非法帧是**整帧拒收**(M4 的 D2/D3 验过), 但上位机不该把这种帧发出去 ——
    发出去只会白白浪费一个控制周期, 而那个周期里固件沿用的是上一条命令。
    """


class ExternalThruster:
    """8 路直控的发送端 + 安全外壳。

    线程模型: 一个后台 keepalive 线程按 hz 发命令与心跳; 主线程只改 _cmd。
    所有对 MAVLink 连接的收发都走同一把锁 —— pymavlink 的 mav 对象(序列号、解析缓冲)
    不是线程安全的。
    """

    def __init__(self, conn, *, u_max: float = 0.3, hz: float = 25.0,
                 timeout_ms: int = 500, verbose: bool = True):
        if not 0.0 < u_max <= 1.0:
            raise ValueError("u_max 必须在 (0, 1]")
        # 控制周期必须远快于固件看门狗, 否则抖一下就被判失联
        if hz < 4.0 / (timeout_ms / 1000.0):
            raise ValueError(f"hz={hz} 相对 MOT_EXT_TMOUT={timeout_ms}ms 太慢, "
                             f"至少要 {4.0 / (timeout_ms / 1000.0):.0f} Hz")
        self.c = conn
        self.u_max = float(u_max)
        self.hz = float(hz)
        self.timeout_ms = int(timeout_ms)
        self.verbose = verbose

        self._lock = threading.Lock()
        self._cmd = [0.0] * NMOT
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._next_hb = 0.0
        self._last_send = 0.0
        self._worst_interval = 0.0     # keepalive 实际间隔的最差值, 退出时报出来

        self.armed: bool | None = None
        self.statustexts: list[str] = []
        self.latched = False           # 收到过固件的看门狗告警
        self.servo: list[int] | None = None
        self._clamped_warned = False
        self._last_hb_rx = mono()      # 最后一次收到飞控心跳的时刻

    def link_lost(self, limit: float = 5.0) -> bool:
        """飞控心跳断了。继续发命令没有意义, 而且会掩盖掉线这个真正的问题。"""
        return mono() - self._last_hb_rx > limit

    # ---------------- 收发底层 ----------------
    def _tx_ext(self, vals) -> None:
        with self._lock:
            self.c.mav.set_actuator_control_target_send(
                int(time.time() * 1e6) & 0xFFFFFFFF, GROUP_MOTORS,
                self.c.target_system, self.c.target_component, list(vals))

    def _tx_heartbeat(self) -> None:
        with self._lock:
            self.c.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def pump(self) -> None:
        """吸收待处理报文: 解锁状态 / 告警 / 输出读数。"""
        while True:
            with self._lock:
                m = self.c.recv_match(blocking=False)
            if m is None:
                return
            t = m.get_type()
            if t == "HEARTBEAT" and m.get_srcSystem() == self.c.target_system:
                self.armed = bool(m.base_mode & ARMED_FLAG)
                self._last_hb_rx = mono()
            elif t == "STATUSTEXT":
                txt = m.text.decode(errors="replace") if isinstance(m.text, bytes) else m.text
                self.statustexts.append(txt)
                if LATCH_TEXT in txt.lower():
                    self.latched = True
                    print(f"\n[ext] ⚠ 固件看门狗触发并锁存: {txt}")
                    print("[ext]   在 reset_latch() 之前, 后续合法命令也会被拒绝。")
            elif t == "SERVO_OUTPUT_RAW":
                self.servo = [getattr(m, f"servo{i}_raw") for i in range(1, NMOT + 1)]

    def request_servo_stream(self, hz: int = 10) -> None:
        with self._lock:
            self.c.mav.request_data_stream_send(
                self.c.target_system, self.c.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, hz, 1)

    # ---------------- 参数 ----------------
    def set_param(self, name: str, value: float, ptype=INT8, tries: int = 6) -> bool:
        for _ in range(tries):
            with self._lock:
                self.c.mav.param_set_send(self.c.target_system, self.c.target_component,
                                          name.encode(), float(value), ptype)
            t_end = mono() + 1.0
            while mono() < t_end:
                with self._lock:
                    m = self.c.recv_match(type="PARAM_VALUE", blocking=False)
                if m is not None and _pid(m) == name and abs(m.param_value - value) < 1e-3:
                    return True
                time.sleep(0.01)
        return False

    def get_param(self, name: str, tries: int = 4) -> float | None:
        for _ in range(tries):
            with self._lock:
                self.c.mav.param_request_read_send(
                    self.c.target_system, self.c.target_component, name.encode(), -1)
            t_end = mono() + 1.5
            while mono() < t_end:
                with self._lock:
                    m = self.c.recv_match(type="PARAM_VALUE", blocking=False)
                if m is not None and _pid(m) == name:
                    return float(m.param_value)
                time.sleep(0.01)
        return None

    def check_firmware(self) -> bool:
        """确认对面是改版固件 —— 官方 ArduSub 没有 MOT_EXT_*。"""
        en = self.get_param("MOT_EXT_ENABLE")
        tm = self.get_param("MOT_EXT_TMOUT")
        if en is None or tm is None:
            print("[ext] ❌ 读不到 MOT_EXT_ENABLE / MOT_EXT_TMOUT —— 对面不是改版固件。")
            return False
        if self.verbose:
            print(f"[ext] 改版固件 OK  MOT_EXT_ENABLE={en:.0f} MOT_EXT_TMOUT={tm:.0f}ms")
        if int(tm) != self.timeout_ms:
            print(f"[ext] 注意: 固件 MOT_EXT_TMOUT={tm:.0f}ms, 本地按 {self.timeout_ms}ms 配速")
            self.timeout_ms = int(tm)
        return True

    # ---------------- 使能 / 锁存 ----------------
    def enable(self) -> bool:
        return self.set_param("MOT_EXT_ENABLE", 1, INT8)

    def disable(self) -> bool:
        """关掉外部直控。顺带就是官方的解锁存动作(固件在 ENABLE==0 时清 latch)。"""
        ok = self.set_param("MOT_EXT_ENABLE", 0, INT8)
        if ok:
            self.latched = False
        return ok

    def reset_latch(self) -> bool:
        """清掉固件的故障锁存, 回到可控状态。

        顺序是关键, 不能直接 ENABLE 0→1。固件里"解锁存"和"清残留命令"是两条独立路径:
            clear_external_fault()  只在 MOT_EXT_ENABLE==0 时被 50Hz 检查调用
            clear_external_thrust() (清 _ext_have_cmd) 只在看门狗真正触发时调用
        所以如果直接 0→1, _ext_have_cmd 仍是 true 而 _ext_last_ms 早已过期, 重新使能的
        那一瞬 external_thrust_timed_out() 立刻为真 → 20ms 内当场重新锁存。
        正确做法: 先让看门狗在 ENABLE 还开着时打一次(它会清掉 _ext_have_cmd), 再 0→1。
        """
        if self.verbose:
            print("[ext] 清锁存 ...")
        was_running = self._thread is not None and self._thread.is_alive()
        if was_running:
            self.stop_keepalive(neutral_hold=0.0)
        self.set_param("MOT_EXT_ENABLE", 1, INT8)
        self._idle(self.timeout_ms / 1000.0 + 0.5)   # 静默 > TMOUT, 逼看门狗清残留
        self.set_param("MOT_EXT_ENABLE", 0, INT8)    # 解锁存
        self._idle(0.3)                              # >1 个 50Hz 周期
        ok = self.set_param("MOT_EXT_ENABLE", 1, INT8)
        self.latched = False
        self.statustexts.clear()
        if was_running:
            self.start_keepalive()
        return ok

    def _idle(self, seconds: float) -> None:
        """只收不发外部命令 (心跳照常, 否则会撞上 GCS failsafe)。"""
        t_end = mono() + seconds
        while mono() < t_end:
            now = mono()
            if now >= self._next_hb:
                self._tx_heartbeat()
                self._next_hb = now + 1.0
            self.pump()
            time.sleep(0.01)

    # ---------------- 命令 ----------------
    def set_thrust(self, values) -> list[float]:
        """设置 8 路归一化推力; 返回实际生效(限幅后)的值。

        固件对非法帧整帧拒收, 所以这里直接拒绝而不是发出去碰运气。
        u_max 是**上位机侧**的额外限幅, 比固件的 [-1,1] 更保守 —— 干测时用它压住幅度。
        """
        vals = [float(v) for v in values]
        if len(vals) != NMOT:
            raise ThrustRejected(f"必须给 {NMOT} 个值, 收到 {len(vals)}")
        for i, v in enumerate(vals):
            if not math.isfinite(v):
                raise ThrustRejected(f"Motor{i + 1} 是 NaN/Inf")
            if abs(v) > 1.0:
                raise ThrustRejected(f"Motor{i + 1}={v} 超出 [-1, 1]")
        out = [max(-self.u_max, min(self.u_max, v)) for v in vals]
        if out != vals and not self._clamped_warned:
            print(f"[ext] 注意: 命令被 u_max={self.u_max} 限幅 (后续不再提示)")
            self._clamped_warned = True
        self._cmd = out
        return out

    def set_motor(self, n: int, value: float) -> list[float]:
        """只驱动 Motor n (1..8), 其余归零。"""
        if not 1 <= n <= NMOT:
            raise ThrustRejected(f"电机号必须是 1..{NMOT}, 收到 {n}")
        cmd = [0.0] * NMOT
        cmd[n - 1] = value
        return self.set_thrust(cmd)

    def zero(self) -> None:
        self._cmd = [0.0] * NMOT

    # ---------------- keepalive ----------------
    def start_keepalive(self) -> None:
        """启动后台发送线程。

        **这不是可选项。** 停发超过 MOT_EXT_TMOUT, 固件就会归中 + disarm + 锁存。
        """
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._worst_interval = 0.0
        self._thread = threading.Thread(target=self._ka_loop, name="ext-keepalive", daemon=True)
        self._thread.start()

    def _ka_loop(self) -> None:
        dt = 1.0 / self.hz
        self._last_send = mono()
        while not self._stop.is_set():
            now = mono()
            gap = now - self._last_send
            if gap > self._worst_interval:
                self._worst_interval = gap
            self._tx_ext(self._cmd)
            self._last_send = now
            if now >= self._next_hb:
                self._tx_heartbeat()
                self._next_hb = now + 1.0
            self.pump()
            time.sleep(dt)

    def stop_keepalive(self, neutral_hold: float = 0.3) -> None:
        """停发之前先把命令归零并保持一会, 让桨先停在中位再撒手。"""
        if self._thread is None:
            return
        if neutral_hold > 0:
            self.zero()
            t_end = mono() + neutral_hold
            while mono() < t_end:
                time.sleep(0.01)
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._thread = None

    # ---------------- 解锁 ----------------
    def set_mode_manual(self, timeout: float = 5.0) -> bool:
        t_end = mono() + timeout
        last = 0.0
        while mono() < t_end:
            if mono() - last > 1.0:
                with self._lock:
                    self.c.mav.command_long_send(
                        self.c.target_system, self.c.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        MODE_MANUAL, 0, 0, 0, 0, 0)
                last = mono()
            with self._lock:
                m = self.c.recv_match(type="HEARTBEAT", blocking=False)
            if m is not None and m.get_srcSystem() == self.c.target_system:
                self.armed = bool(m.base_mode & ARMED_FLAG)
                if m.custom_mode == MODE_MANUAL:
                    return True
            time.sleep(0.02)
        return False

    def _arm_cmd(self, want: bool) -> None:
        with self._lock:
            self.c.mav.command_long_send(
                self.c.target_system, self.c.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1 if want else 0, 0, 0, 0, 0, 0, 0)

    def arm(self, timeout: float = 8.0) -> bool:
        """解锁。调用前 keepalive 必须已经在跑 —— 否则刚解锁就会超时锁存。"""
        if self._thread is None or not self._thread.is_alive():
            raise RuntimeError("解锁前必须先 start_keepalive(), 否则会被看门狗立刻判失联")
        t_end = mono() + timeout
        last = 0.0
        while mono() < t_end:
            if mono() - last > 1.2:
                self._arm_cmd(True)
                last = mono()
            if self.armed:
                return True
            time.sleep(0.05)
        return bool(self.armed)

    def disarm(self, timeout: float = 5.0) -> bool:
        self.zero()
        t_end = mono() + timeout
        last = 0.0
        while mono() < t_end:
            if mono() - last > 1.2:
                self._arm_cmd(False)
                last = mono()
            self.pump()
            if self.armed is False:
                return True
            time.sleep(0.05)
        return self.armed is False

    # ---------------- 生命周期 ----------------
    def open(self) -> bool:
        if not self.check_firmware():
            return False
        self.request_servo_stream()
        if not self.set_mode_manual():
            print("[ext] ⚠ 切 MANUAL 模式失败, 继续 (直控不依赖模式, 但建议查一下)")
        if not self.enable():
            print("[ext] ❌ MOT_EXT_ENABLE 置 1 失败")
            return False
        self.start_keepalive()
        if self.verbose:
            print(f"[ext] 已就绪  u_max={self.u_max}  {self.hz:.0f}Hz  "
                  f"看门狗 {self.timeout_ms}ms")
        return True

    def close(self) -> None:
        """退出保护: 归零 → disarm → MOT_EXT_ENABLE=0。每一步都尽力做完。

        顺序不能换: 必须在还能发命令的时候先把推力归零, 再上锁, 最后才关外部控制。
        """
        print("[ext] 退出保护: 归零 → 上锁 → 关闭外部直控")
        try:
            self.zero()
            time.sleep(0.3)
        except Exception as e:
            print(f"[ext]   归零失败: {e}")
        try:
            if self.armed:
                self.disarm()
        except Exception as e:
            print(f"[ext]   上锁失败: {e}")
        try:
            self.stop_keepalive(neutral_hold=0.0)
        except Exception as e:
            print(f"[ext]   停 keepalive 失败: {e}")
        try:
            self.disable()
        except Exception as e:
            print(f"[ext]   关 MOT_EXT_ENABLE 失败: {e}")
        if self._worst_interval > 2.0 / self.hz:
            print(f"[ext] 注意: keepalive 最差间隔 {self._worst_interval * 1000:.0f}ms "
                  f"(目标 {1000 / self.hz:.0f}ms) —— 本机调度顶不住这个控制频率")
        print(f"[ext] 收尾完成  armed={self.armed}")

    def __enter__(self) -> "ExternalThruster":
        if not self.open():
            raise RuntimeError("ExternalThruster 初始化失败")
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def _pid(m) -> str:
    pid = m.param_id
    if isinstance(pid, bytes):
        pid = pid.split(b"\x00")[0].decode(errors="ignore")
    return pid


def _fmt_servo(ext: ExternalThruster) -> str:
    if ext.servo is None:
        return "SERVO: (无读数)"
    return "SERVO: " + " ".join(f"{v}" for v in ext.servo)


def _hold(ext: ExternalThruster, seconds: float, label: str) -> None:
    """保持当前命令 seconds 秒, 期间按 2Hz 打印输出读数。

    进来先把缓存的读数丢掉: 命令刚换时还没收到新的 SERVO_OUTPUT_RAW(流只有 10Hz),
    直接打印会显示上一段的值 —— 比如归零那一行显示上一路的 1620, 看起来像归零没生效。
    """
    t_end = mono() + seconds
    # 丢掉"命令生效之前就已经在队列里"的旧报文。只把 servo 置 None 不够 ——
    # 队列里那几帧 pump 出来照样会填回去, 于是每段第一行显示的是**上一段**的值。
    # t_end 在 flush 之前就算好, 所以这 0.2s 不会缩短本段时长(推力已经发出去了)。
    t_flush = mono() + 0.2
    while mono() < t_flush:
        ext.pump()
        time.sleep(0.01)
    ext.servo = None
    next_print = 0.0
    while mono() < t_end:
        ext.pump()
        if ext.servo is not None and mono() >= next_print:
            print(f"  {label}  {_fmt_servo(ext)}")
            next_print = mono() + 0.5
        if ext.latched:
            print("  [中止] 固件已锁存, 停止本段")
            return
        if ext.link_lost():
            print("  [中止] 超过 5s 没收到飞控心跳, 链路已断")
            return
        time.sleep(0.02)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="对改版 ArduSub 直接下发 8 路归一化推力",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="默认连 SITL。连真机要显式 --endpoint udpin:0.0.0.0:14550")
    ap.add_argument("--endpoint", default=SITL_ENDPOINT,
                    help=f"默认 {SITL_ENDPOINT} (SITL); 真机用 {ROV_ENDPOINT}")
    ap.add_argument("--u-max", type=float, default=0.3,
                    help="上位机侧限幅, 默认 0.3。干测首轮建议 0.1")
    ap.add_argument("--hz", type=float, default=25.0, help="控制频率, 默认 25")
    ap.add_argument("--arm", action="store_true",
                    help="允许解锁。不加这个就只验证链路 —— 未解锁时固件硬写 1500")
    ap.add_argument("--seconds", type=float, default=3.0, help="单段保持时长")

    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--zero", action="store_true", help="只发零命令(验链路与看门狗不触发)")
    g.add_argument("--motor", type=int, metavar="N", help="只驱动 Motor N (1..8)")
    g.add_argument("--all", type=float, metavar="V", help="八路同时给 V")
    g.add_argument("--sweep", action="store_true", help="八路逐一扫描 (对应 M4 的 B4)")

    ap.add_argument("--thrust", type=float, default=0.2, help="--motor/--sweep 用的推力值")
    ap.add_argument("--dwell", type=float, default=1.5, help="--sweep 每路保持时长")
    args = ap.parse_args(argv)

    if args.endpoint != SITL_ENDPOINT and args.arm:
        print("=" * 64)
        print("⚠  正在对**真机**解锁并驱动推进器。")
        print(f"   endpoint={args.endpoint}  u_max={args.u_max}  thrust={args.thrust}")
        print("   直控层没有混控兜底, 单桨满推时整机会在台面上移动。")
        print("=" * 64)

    conn = connect(args.endpoint)
    if conn is None:
        return 2

    ext = ExternalThruster(conn, u_max=args.u_max, hz=args.hz)
    try:
        if not ext.open():
            return 2
        if ext.latched:
            ext.reset_latch()

        if args.arm:
            if not ext.arm():
                print("[ext] ❌ 解锁失败")
                return 1
            print("[ext] 已解锁")
            time.sleep(1.0)       # 等 spool up
        else:
            print("[ext] 未加 --arm: 不解锁。固件会把 8 路硬写 1500, 这本身就是 A 项验收。")

        if args.zero:
            ext.zero()
            _hold(ext, args.seconds, "零命令")
        elif args.motor is not None:
            ext.set_motor(args.motor, args.thrust)
            _hold(ext, args.seconds, f"Motor{args.motor}={args.thrust}")
        elif args.all is not None:
            ext.set_thrust([args.all] * NMOT)
            _hold(ext, args.seconds, f"八路={args.all}")
        elif args.sweep:
            for n in range(1, NMOT + 1):
                ext.set_motor(n, args.thrust)
                _hold(ext, args.dwell, f"Motor{n}={args.thrust}")
                ext.zero()
                _hold(ext, 0.5, "间隔归零")
                if ext.latched:
                    break
        return 0
    except ThrustRejected as e:
        print(f"[ext] ❌ 命令被上位机拒收: {e}")
        return 1
    except KeyboardInterrupt:
        print("\n[ext] 中断")
        return 130
    finally:
        ext.close()


if __name__ == "__main__":
    sys.exit(main())
