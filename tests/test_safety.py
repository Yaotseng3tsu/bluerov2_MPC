#!/usr/bin/env python3
"""Phase 1 安全逻辑单元测试 (无需网络/真机)。

用 FakeConn 记录实际发出的 MANUAL_CONTROL 通道值,逐项验证:
  1. U_MAX 限幅:|u|>U_MAX 被裁剪
  2. z 通道映射:中位 = z_neutral,+u 下潜,限幅后对称
  3. 符号 sign_* 生效
  4. send_neutral / close 确实发出中位
  5. 中断(KeyboardInterrupt)路径也会回中位 (finally→close)
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src import pseudo_stick as ps  # noqa: E402


class FakeMav:
    def __init__(self):
        self.sends = []  # list of (x,y,z,r)

    def manual_control_send(self, target, x, y, z, r, buttons):
        self.sends.append((x, y, z, r))


class FakeConn:
    target_system = 1
    target_component = 1

    def __init__(self):
        self.mav = FakeMav()
        self.closed = False

    def wait_heartbeat(self, timeout=0):
        # 伪 heartbeat:base_mode=0 → DISARMED
        class HB:
            base_mode = 0
        return HB()

    def close(self):
        self.closed = True


def make_stick(u_max=0.3, z_neutral=500, signs=None):
    signs = signs or {}
    cfg = {
        "connection": {"endpoint": "udpin:0.0.0.0:0", "dialect": "ardupilotmega",
                       "heartbeat_timeout_s": 1},
        "safety": {"U_MAX": u_max},
        "manual_control": {"z_neutral": z_neutral,
                           **{f"sign_{k}": signs.get(k, 1) for k in "xyzr"}},
        "control": {"CTRL_HZ": 10},
    }
    stick = ps.PseudoStick(cfg, endpoint="udpin:0.0.0.0:0")
    stick.conn = FakeConn()  # 换成记录用 mock
    return stick


def approx(a, b, tol=1):
    return abs(a - b) <= tol


def test_clamp_horizontal():
    s = make_stick(u_max=0.3)
    assert s._norm_to_ch(0.9, "x") == 300, "x 超限应裁剪到 0.3*1000=300"
    assert s._norm_to_ch(-0.9, "x") == -300
    assert s._norm_to_ch(0.1, "x") == 100
    print("✅ 水平轴限幅 OK")


def test_clamp_and_map_z():
    s = make_stick(u_max=0.3, z_neutral=500)
    assert s._norm_to_ch(0.0, "z") == 500, "z 中位"
    assert s._norm_to_ch(0.9, "z") == 650, "u=+0.9 限幅到 0.3 → 500+0.3*500=650"
    assert s._norm_to_ch(-0.9, "z") == 350, "u=-0.9 → 350"
    print("✅ z 限幅+映射 OK (中位500, +下潜)")


def test_z_neutral_nonstandard():
    # 若真机 z 中位不是 500,span 取到边界的较小值,保证不越界 [0,1000]
    s = make_stick(u_max=1.0, z_neutral=300)
    assert s._norm_to_ch(1.0, "z") == 600, "span=min(300,700)=300 → 300+300=600"
    assert s._norm_to_ch(-1.0, "z") == 0, "300-300=0,不越界"
    print("✅ 非标准 z 中位不越界 OK")


def test_sign_flip():
    s = make_stick(u_max=1.0, signs={"z": -1})
    assert s._norm_to_ch(0.5, "z") == 250, "sign_z=-1 翻转 → 500-0.5*500=250"
    print("✅ 符号翻转 OK")


def test_send_applies_clamp():
    s = make_stick(u_max=0.3)
    s.send(x=0.9, y=-0.9, z=0.0, r=0.9)
    x, y, z, r = s.conn.mav.sends[-1]
    assert (x, y, z, r) == (300, -300, 500, 300), f"实际={x,y,z,r}"
    print("✅ send() 端到端限幅 OK")


def test_neutral_on_close():
    s = make_stick()
    s.send(z=0.3)               # 先发个非中位
    s.close()
    # close 应追加若干中位帧
    neutrals = [t for t in s.conn.mav.sends if t == (0, 0, 500, 0)]
    assert len(neutrals) >= 3, f"close 应发多帧中位,实际中位帧={len(neutrals)}"
    assert s.conn.mav.sends[-1] == (0, 0, 500, 0), "最后一帧必须是中位"
    assert s.conn.closed, "连接应被关闭"
    print(f"✅ 退出回中位 OK (发出 {len(neutrals)} 帧中位,末帧=中位)")


def test_interrupt_path_sends_neutral():
    """对 main() 打桩:hold() 抛 KeyboardInterrupt,验证 finally→close 仍回中位。"""
    import src.pseudo_stick as psmod

    built = {}
    orig_cfg, orig_ctor = psmod.load_config, psmod.PseudoStick

    def fake_load_config():
        return {
            "connection": {"endpoint": "udpin:0.0.0.0:0", "heartbeat_timeout_s": 1},
            "safety": {"U_MAX": 0.3},
            "manual_control": {"z_neutral": 500},
            "control": {"CTRL_HZ": 10},
        }

    def fake_ctor(cfg, endpoint=None):
        psmod.PseudoStick = orig_ctor          # 临时还原,避免 make_stick 递归
        try:
            s = make_stick()
        finally:
            psmod.PseudoStick = fake_ctor
        def boom(*a, **k):
            raise KeyboardInterrupt            # 模拟运行中 Ctrl+C
        s.hold = boom
        s.wait_heartbeat = lambda *a, **k: True
        built["s"] = s
        return s

    psmod.load_config, psmod.PseudoStick = fake_load_config, fake_ctor
    try:
        rc = psmod.main(["--z", "0.3", "--seconds", "5"])
    finally:
        psmod.load_config, psmod.PseudoStick = orig_cfg, orig_ctor

    s = built["s"]
    assert rc == 130, f"KeyboardInterrupt 应返回 130,实际 {rc}"
    assert s.conn.mav.sends[-1] == (0, 0, 500, 0), "中断后末帧必须是中位"
    assert s.conn.closed, "连接应被关闭"
    print("✅ 中断(Ctrl+C)路径回中位 OK (rc=130, 末帧=中位)")


def main():
    tests = [
        test_clamp_horizontal,
        test_clamp_and_map_z,
        test_z_neutral_nonstandard,
        test_sign_flip,
        test_send_applies_clamp,
        test_neutral_on_close,
        test_interrupt_path_sends_neutral,
    ]
    failed = 0
    for t in tests:
        try:
            t()
        except AssertionError as e:
            failed += 1
            print(f"❌ {t.__name__}: {e}")
        except Exception as e:  # noqa
            failed += 1
            print(f"💥 {t.__name__}: {type(e).__name__}: {e}")
    print(f"\n=== {len(tests)-failed}/{len(tests)} 通过 ===")
    return 1 if failed else 0


if __name__ == "__main__":
    for _s in (sys.stdout, sys.stderr):
        try:
            _s.reconfigure(encoding="utf-8")
        except Exception:
            pass
    raise SystemExit(main())
