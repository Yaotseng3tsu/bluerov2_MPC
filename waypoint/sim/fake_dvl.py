#!/usr/bin/env python3
"""离线仿真件 — 假 DVL A50 服务器 (吐 velocity JSON/TCP)。

在 127.0.0.1:16171 起一个 TCP 服务, 按固定频率向已连接客户端推送与真 A50 同格式的
velocity 报文 (含 vx/vy/vz/velocity_valid/altitude/fom/time), 让 dvl_stream.py 与
go_waypoint.py 在没有硬件时也能端到端跑通。

速度来源由回调提供 (sim_waypoint 把 SurgePlant 的机体速度喂进来);
独立运行时可发恒定/正弦速度, 单测 dvl_stream。

用法 (独立自检):
  python -m waypoint.sim.fake_dvl --vx 0.3 --seconds 10        # 恒定 vx
  python -m waypoint.sim.fake_dvl --sine --amp 0.4 --period 8  # 正弦 vx
  # 另开一个终端: python -m waypoint.dvl_stream --ip 127.0.0.1
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import threading
import time

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 16171


class FakeDvl:
    """TCP 服务, 按 rate_hz 向所有客户端推 velocity JSON。

    velocity_fn() -> dict(vx,vy,vz,valid,altitude[,fom]);缺省字段自动补。
    """

    def __init__(self, velocity_fn, host: str = DEFAULT_HOST,
                 port: int = DEFAULT_PORT, rate_hz: float = 10.0):
        self.velocity_fn = velocity_fn
        self.host = host
        self.port = port
        self.rate_hz = rate_hz
        self._run = False
        self._srv: socket.socket | None = None
        self._clients: list[socket.socket] = []
        self._lock = threading.Lock()
        self._threads: list[threading.Thread] = []

    def start(self) -> "FakeDvl":
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.host, self.port))
        self._srv.listen(4)
        self._srv.settimeout(0.5)
        self._run = True
        for target in (self._accept_loop, self._push_loop):
            th = threading.Thread(target=target, daemon=True)
            th.start()
            self._threads.append(th)
        return self

    def _accept_loop(self) -> None:
        while self._run:
            try:
                cli, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with self._lock:
                self._clients.append(cli)

    def _push_loop(self) -> None:
        period = 1.0 / self.rate_hz
        t0 = time.monotonic()
        while self._run:
            v = self.velocity_fn() or {}
            report = {
                "type": "velocity",
                "time": round((time.monotonic() - t0) * 1000, 1),
                "vx": float(v.get("vx", 0.0)),
                "vy": float(v.get("vy", 0.0)),
                "vz": float(v.get("vz", 0.0)),
                "fom": float(v.get("fom", 0.01)),
                "altitude": float(v.get("altitude", 2.0)),
                "velocity_valid": bool(v.get("valid", True)),
                "status": 0,
            }
            line = (json.dumps(report) + "\n").encode("utf-8")
            with self._lock:
                dead = []
                for c in self._clients:
                    try:
                        c.sendall(line)
                    except OSError:
                        dead.append(c)
                for c in dead:
                    self._clients.remove(c)
                    try:
                        c.close()
                    except OSError:
                        pass
            time.sleep(period)

    def stop(self) -> None:
        self._run = False
        with self._lock:
            for c in self._clients:
                try:
                    c.close()
                except OSError:
                    pass
            self._clients.clear()
        if self._srv:
            try:
                self._srv.close()
            except OSError:
                pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="假 DVL 服务器 (离线自检)")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--vx", type=float, default=0.3, help="恒定 vx (m/s)")
    ap.add_argument("--sine", action="store_true", help="vx 用正弦")
    ap.add_argument("--amp", type=float, default=0.4)
    ap.add_argument("--period", type=float, default=8.0)
    ap.add_argument("--invalid", action="store_true", help="模拟气中 valid=false")
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args(argv)

    t0 = time.monotonic()

    def vel():
        if args.sine:
            vx = args.amp * math.sin(2 * math.pi * (time.monotonic() - t0) / args.period)
        else:
            vx = args.vx
        return {"vx": vx, "vy": 0.0, "vz": 0.0,
                "valid": not args.invalid, "altitude": -1.0 if args.invalid else 2.0}

    dvl = FakeDvl(vel, host=args.host, port=args.port, rate_hz=args.rate).start()
    print(f"[fake_dvl] 服务 {args.host}:{args.port} @ {args.rate}Hz, "
          f"{'正弦' if args.sine else f'恒定vx={args.vx}'}, {args.seconds}s")
    try:
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        dvl.stop()
        print("[fake_dvl] 停止。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
