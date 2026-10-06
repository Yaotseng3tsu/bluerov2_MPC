#!/usr/bin/env python3
"""W1 — DVL A50 速度读取器 (后台线程 + 线程安全最新值 + 超龄检测)。

Water Linked DVL A50 通过 JSON/TCP (默认 192.168.2.95:16171) 持续推送 velocity 报文。
本模块起一个后台线程持续解析,主控制循环随时用 latest()/is_fresh() 取"最新速度",
供 W5 的航位推算 (s = ∫vx dt) 与安全降级 (底锁丢失/数据超龄 → 立即停) 使用。

设计要点 (对齐主项目安全约定):
  - 只读旁路:不影响 BlueOS DVL 扩展 (扩展保持启用);A50 JSON 服务通常允许多客户端读取。
    若 16171 被占用/连不上,W1 阶段退回 MAVLink 读扩展转发速度 (见 README;此处先做直连)。
  - 线程安全:latest() 返回最近一帧的拷贝,带 age_s (单调钟龄期)。
  - 断线自愈:socket 异常时自动重连,不阻塞主循环。
  - 超龄检测:is_fresh(max_age) 让上层在数据变旧时安全停车。

用法 (自检):
  python -m waypoint.dvl_stream                 # 默认 IP,流式打印 8s,报更新率
  python -m waypoint.dvl_stream --ip 192.168.2.95 --seconds 12
  python -m waypoint.dvl_stream --forward-hint  # 打印"如何用前进动作确认 vx 符号"
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

DEFAULT_IP = "192.168.2.95"
DEFAULT_PORT = 16171
CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "vehicle.yaml"


@dataclass
class DvlSample:
    """一帧 DVL 速度 (DVL 坐标系;vx 前向须 W1 连机确认符号)。"""
    vx: float = 0.0          # m/s
    vy: float = 0.0
    vz: float = 0.0
    valid: bool = False      # velocity_valid (气中/丢底锁时 false)
    altitude: float = -1.0   # m, 到底距离 (>0 表示有底锁)
    fom: float = -1.0        # figure of merit (越小越可信)
    t_mono: float = 0.0      # 收到该帧时的 monotonic 时刻
    age_s: float = 0.0       # latest() 计算时填充
    dt_s: float = 0.0        # 本帧的积分时长 (报文 `time` 字段, ms -> s)


@dataclass
class DvlDisplacement:
    """DVL 坐标系下的累计位移 (未乘 vx_sign, 由上层决定符号约定)。"""
    x: float = 0.0           # m, 累计前向位移
    y: float = 0.0
    z: float = 0.0
    dt_s: float = 0.0        # 累计积分时长
    seq: int = 0             # 已累加的帧数 (上层据此判断"有没有新帧")
    gap_s: float = 0.0       # 累计无解时长 (零阶保持或完全缺口)
    n_valid: int = 0
    n_total: int = 0


def _load_dvl_cfg(path: Path = CONFIG_PATH) -> tuple[str, int]:
    """从 vehicle.yaml 读可选 dvl.ip/port;缺省用默认值 (不强制改主配置)。"""
    try:
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            d = (yaml.safe_load(f) or {}).get("dvl", {}) or {}
        return str(d.get("ip", DEFAULT_IP)), int(d.get("port", DEFAULT_PORT))
    except Exception:
        return DEFAULT_IP, DEFAULT_PORT


class DvlStream:
    """后台线程读取 DVL 速度。start() 启动,latest()/is_fresh() 取值,stop() 关闭。"""

    def __init__(self, ip: str | None = None, port: int | None = None,
                 connect_timeout: float = 5.0, max_alt_rate: float = 1.5,
                 alt_outlier_accept: int = 5, zoh_max_s: float = 0.5):
        cip, cport = _load_dvl_cfg()
        self.ip = ip or cip
        self.port = port or cport
        self.connect_timeout = connect_timeout
        # 高度跳变剔除: 机器人不可能在 0.1s 内上下窜 1.7m。实测 A50 偶发把 altitude
        # 跳到 2.5m/3.3m, 会直接撞上 alt_max 护栏令任务中止。
        self.max_alt_rate = max_alt_rate            # m/s, 超过即视为跳变
        self.alt_outlier_accept = alt_outlier_accept  # 连续这么多帧仍偏离 → 认账(真换地形)
        self._alt_outliers = 0
        # 航位推算: 在**后台线程**里逐帧累加, 用报文自带的积分时长。
        # 旧做法是在 10Hz 控制环里 `s += vx * dt_nom` —— DVL 只有 4.5-7.5Hz,
        # 同一帧会被重复积分 1.3-2.2 次, 且用的是标称 dt 而不是实测步长。
        # BlueOS 的 DVL 扩展算 VISION_POSITION_DELTA 时用的也是报文里的 dt (Dp = v*dt)。
        self.zoh_max_s = zoh_max_s      # 无解帧最多按上一帧速度保持多久
        self._disp = DvlDisplacement()
        self._last_v = None             # 最近一帧好速度 (零阶保持用)
        self._gap_run = 0.0             # 当前这段连续无解已持续多久
        self._t_prev_frame = 0.0        # 上一帧到达时刻 (dt 兜底用)
        self._alt_seq = 0               # 被接受的高度帧计数
        self._lock = threading.Lock()
        self._latest = DvlSample()
        # 最近一帧"有效"样本单独保留: A50 会间歇性解算失败(实测 ~2.5% 帧 valid=false/alt=-1,
        # 贴底 + 满推搅起气泡泥沙时更频繁)。只看最新帧会被单帧坏数据误判为丢底锁。
        self._latest_valid = DvlSample()
        self._run = False
        self._thread: threading.Thread | None = None
        self._n_frames = 0          # 累计解析到的 velocity 帧数
        self._connected = False

    # ---------------- 后台线程 ----------------
    def start(self) -> "DvlStream":
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return self

    def _loop(self) -> None:
        buf = b""
        while self._run:
            try:
                with socket.create_connection((self.ip, self.port),
                                              timeout=self.connect_timeout) as sock:
                    sock.settimeout(2.0)
                    self._connected = True
                    buf = b""
                    while self._run:
                        try:
                            chunk = sock.recv(4096)
                        except socket.timeout:
                            continue
                        if not chunk:
                            break  # 对端关闭 → 跳出去重连
                        buf += chunk
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            self._handle_line(line.strip())
            except OSError:
                self._connected = False
                # 连接失败/断开:短暂退避后重连,不抛给主循环
                for _ in range(10):
                    if not self._run:
                        break
                    time.sleep(0.1)
        self._connected = False

    def _handle_line(self, line: bytes) -> None:
        if not line:
            return
        try:
            msg = json.loads(line.decode("utf-8", "ignore"))
        except json.JSONDecodeError:
            return
        if msg.get("type") != "velocity" and "velocity_valid" not in msg:
            return  # 只关心 velocity 报文 (忽略 dead_reckoning/transducer 等)
        now = time.monotonic()
        # A50 的 `time` 字段 = **距上一帧的毫秒数**(即这帧速度对应的积分时长)。
        # 取值不在合理区间(固件差异/累计时间戳/丢报文)就退回墙钟差, 并钳位。
        dt_s = None
        raw = msg.get("time")
        if raw is not None:
            try:
                v = float(raw) / 1000.0
                if 0.005 <= v <= 1.0:
                    dt_s = v
            except (TypeError, ValueError):
                dt_s = None
        if dt_s is None:
            dt_s = (now - self._t_prev_frame) if self._t_prev_frame else (1.0 / 7.0)
            dt_s = min(max(dt_s, 0.005), 1.0)
        self._t_prev_frame = now

        s = DvlSample(
            vx=float(msg.get("vx", 0.0)),
            vy=float(msg.get("vy", 0.0)),
            vz=float(msg.get("vz", 0.0)),
            valid=bool(msg.get("velocity_valid", False)),
            altitude=float(msg.get("altitude", -1.0)),
            fom=float(msg.get("fom", -1.0)),
            t_mono=now,
            dt_s=dt_s,
        )
        with self._lock:
            self._latest = s
            self._accumulate(s)
            if s.valid and s.altitude > 0:
                prev = self._latest_valid
                jump_ok = True
                if prev.t_mono > 0:
                    dt = max(1e-3, s.t_mono - prev.t_mono)
                    if abs(s.altitude - prev.altitude) > self.max_alt_rate * dt + 0.10:
                        jump_ok = False
                if jump_ok:
                    self._alt_outliers = 0
                    self._latest_valid = s
                    self._alt_seq += 1
                else:
                    # 跳变: 先丢弃; 若连续多帧都在新值附近, 说明是真的(例如越过台阶)
                    self._alt_outliers += 1
                    if self._alt_outliers >= self.alt_outlier_accept:
                        self._alt_outliers = 0
                        self._latest_valid = s
                        self._alt_seq += 1
            self._n_frames += 1

    def _accumulate(self, s: DvlSample) -> None:
        """逐帧累加机体系位移 (调用方已持锁)。

        无解帧 (~2.5%) 用上一帧好速度做零阶保持, 上限 `zoh_max_s`;
        超过就只记缺口不再积分 —— 真丢底锁时继续按旧速度推会越积越错,
        而控制环的 is_fresh(max_age) 本来就会先一步停车。
        """
        d = self._disp
        d.n_total += 1
        dt = s.dt_s
        if s.valid:
            d.x += s.vx * dt
            d.y += s.vy * dt
            d.z += s.vz * dt
            d.dt_s += dt
            d.seq += 1
            d.n_valid += 1
            self._last_v = (s.vx, s.vy, s.vz)
            self._gap_run = 0.0
            return
        if self._last_v is not None and self._gap_run < self.zoh_max_s:
            vx, vy, vz = self._last_v
            d.x += vx * dt
            d.y += vy * dt
            d.z += vz * dt
            d.dt_s += dt
            d.seq += 1
        self._gap_run += dt
        d.gap_s += dt

    # ---------------- 取值接口 ----------------
    def displacement(self) -> DvlDisplacement:
        """累计位移的快照 (DVL 坐标系, 未乘 vx_sign)。上层取差分即为本段位移。"""
        with self._lock:
            d = self._disp
            return DvlDisplacement(x=d.x, y=d.y, z=d.z, dt_s=d.dt_s, seq=d.seq,
                                   gap_s=d.gap_s, n_valid=d.n_valid, n_total=d.n_total)

    @property
    def alt_seq(self) -> int:
        """被接受的高度帧计数 —— 上层据此判断"本周期有没有新高度量测"。"""
        with self._lock:
            return self._alt_seq

    def latest(self) -> DvlSample:
        """返回最近一帧的拷贝,age_s 填为当前龄期 (秒)。"""
        with self._lock:
            s = self._latest
        return replace(s, age_s=(time.monotonic() - s.t_mono) if s.t_mono else float("inf"))

    def latest_valid(self) -> DvlSample:
        """最近一帧**有效**样本 (valid 且 altitude>0)。age_s = 距今多久。

        控制环应当用它而不是 latest():这样单帧解算失败时沿用上一帧好数据,
        不会因为 ~2.5% 的坏帧而抖动或误停。
        """
        with self._lock:
            s = self._latest_valid
        return replace(s, age_s=(time.monotonic() - s.t_mono) if s.t_mono else float("inf"))

    def is_fresh(self, max_age_s: float = 0.5, require_valid: bool = True) -> bool:
        """数据是否可用。

        require_valid 时判据 = "最近一次**有效**帧的龄期 <= max_age_s",
        而不是"最新那一帧恰好有效" —— 后者会被单帧坏数据误判成丢底锁
        (实测 2.5% 帧无解, 10Hz 控制环几秒内必然撞上, 导致任务被误中止)。
        """
        s = self.latest_valid() if require_valid else self.latest()
        return s.age_s <= max_age_s

    @property
    def alt_outliers(self) -> int:
        """当前连续被剔除的高度跳变帧数 (诊断用)。"""
        with self._lock:
            return self._alt_outliers

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def frame_count(self) -> int:
        with self._lock:
            return self._n_frames

    def stop(self) -> None:
        self._run = False
        th = self._thread
        if th is not None:
            th.join(timeout=2.0)


# ------------------------- 自检 CLI -------------------------
FORWARD_HINT = """\
[提示] 如何确认 vx 符号 (明天连机时):
  1. 让 ROV 稳定悬停/贴底有底锁 (altitude>0, valid=true)。
  2. 用伪手柄给一个明确【前进】点动:
       python -m src.pseudo_stick --x 0.4 --seconds 2 --arm   (--umax 顶过死区)
  3. 同时看本脚本打印的 vx:
       - 前进时 vx 稳定为【正】  → 约定一致,surge_model/航位推算直接用。
       - 前进时 vx 为【负】      → 在 W5 里对 vx 取反 (记入 config, 勿改 DVL 固件)。
  注意:vx 是 DVL 坐标系速度,气中恒为 valid=false 属正常。
"""


def selfcheck(ip: str, port: int, seconds: float) -> int:
    print(f"[dvl] 连接 {ip}:{port},流式自检 {seconds:.0f}s (Ctrl+C 提前结束) ...")
    stream = DvlStream(ip=ip, port=port).start()
    t_end = time.monotonic() + seconds
    n_printed = 0
    last_frame = -1
    try:
        while time.monotonic() < t_end:
            s = stream.latest()
            fc = stream.frame_count
            if fc > 0 and fc != last_frame:  # 只在有新帧时打印,避免刷屏
                last_frame = fc
                flag = "OK " if (s.valid and s.altitude > 0) else "----"
                print(f"[{flag}] vx={s.vx:+.3f} vy={s.vy:+.3f} vz={s.vz:+.3f} "
                      f"alt={s.altitude:5.2f} fom={s.fom:5.2f} "
                      f"valid={s.valid} age={s.age_s*1000:4.0f}ms  frames={fc}")
                n_printed += 1
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[dvl] 用户中断。")
    finally:
        fc = stream.frame_count
        stream.stop()

    print("-" * 60)
    if not stream.connected and fc == 0:
        print(f"[FAIL] 未连上或未收到 velocity 帧 (frames={fc})。")
        print("       → 检查 ping {} ; 是否被 BlueOS DVL 扩展独占 16171 ;".format(ip))
        print("         若扩展独占,W1 退回从 MAVLink 读扩展转发速度 (见 README)。")
        return 2
    rate = fc / seconds if seconds > 0 else 0.0
    print(f"[*] 共收到 {fc} 帧,平均 ≈ {rate:.1f} Hz。")
    s = stream.latest()
    if s.valid and s.altitude > 0:
        print("[PASS] 有底锁、速度有效,DvlStream 读取正常。")
        return 0
    print("[WARN] 已连上但速度无效 (valid=false)。气中属正常;下水贴底再测。")
    return 1


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="W1 DVL 速度读取器自检")
    cip, cport = _load_dvl_cfg()
    p.add_argument("--ip", default=cip)
    p.add_argument("--port", type=int, default=cport)
    p.add_argument("--seconds", type=float, default=8.0)
    p.add_argument("--forward-hint", action="store_true",
                   help="只打印'如何用前进动作确认 vx 符号'的说明并退出")
    args = p.parse_args(argv)
    if args.forward_hint:
        print(FORWARD_HINT)
        return 0
    return selfcheck(args.ip, args.port, args.seconds)


if __name__ == "__main__":
    raise SystemExit(main())
