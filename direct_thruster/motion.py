#!/usr/bin/env python3
"""开环运动原语 — 上升 / 定高 / 前进 / 横滚, 直接输出 8 路推力。

    控制量 τ (机体系广义力)  →  allocation.Allocator  →  8 路归一化推力
                                                       →  external_thruster (安全外壳)

**没有伪手柄这一层。** 控制量不经过 MANUAL_CONTROL 的 x/y/z/r 单位系(z 中位 500、
throttle_gain、gain、sign_z 都是那个接口的产物), 而是直接是机体系的力与力矩,
再由 direct_thruster 自己标定的分配矩阵摊到 8 个桨上。两条线各用各的 mapping。

本文件是**开环**的: 不读任何传感器, 不做反馈。按 README 的阶段划分, 这一步只要求
"动起来且方向对", 不要求精准控制距离/位置。闭环版本(复用 waypoint 线的 NavState +
PID + 设定值节流 + 安全判据)是下一步。

"定高"在开环下 = **恒定上推力抵消负浮力**(`--hover-bias`), 会缓慢漂, 这是开环的固有
结果不是 bug。真正的定高要等接上 DVL altitude 反馈。

⚠ 符号未标定。`allocation.py` 里 sign 全是 +1, 正 τ 到底对应哪个方向**必须干测逐路
确认** —— 直控绕过了混控, MOT_n_DIRECTION 的生效链路和官方固件不同。所以:
**第一次上机请先 --dry-run 看分配, 再用小幅度短时间实测确认方向, 然后回填 sign。**

用法:
  # 离线看分配, 不连任何东西
  python direct_thruster/motion.py --dry-run --demo sequence
  python direct_thruster/motion.py --dry-run --heave 0.3 --seconds 3

  # SITL
  python direct_thruster/motion.py --arm --demo sequence

  # 真机 (先干测确认方向!)
  python direct_thruster/motion.py --endpoint udpin:0.0.0.0:14550 \\
      --arm --u-max 0.15 --demo ascend --seconds 2
"""
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

from allocation import DOFS, Allocator, AllocatorConfig, parse_tau  # noqa: E402

mono = time.monotonic

SITL_ENDPOINT = "tcp:127.0.0.1:5760"
ROV_ENDPOINT = "udpin:0.0.0.0:14550"
NMOT = 8


@dataclass
class Step:
    """一段恒定 τ 的动作。"""
    label: str
    tau: dict[str, float] = field(default_factory=dict)
    seconds: float = 2.0
    hover: bool = True        # 是否叠加 hover_bias (收尾的归零段不叠加)


def build_demo(name: str, v: float, seconds: float, hover_bias: float) -> list[Step]:
    """四个基本动作, 以及把它们串起来的 sequence。

    每段之间插一段只有 hover_bias 的"稳一下", 让上一段的动量先消掉再看下一段 ——
    开环没有反馈, 不留间隔的话两段会叠在一起, 看不出单个动作的效果。
    """
    settle = Step("稳住(只有 hover_bias)", {}, 1.5)
    table: dict[str, list[Step]] = {
        "ascend":  [Step(f"上升 heave={v:+.2f}", {"heave": v}, seconds, hover=False)],
        "descend": [Step(f"下潜 heave={-v:+.2f}", {"heave": -v}, seconds, hover=False)],
        "hover":   [Step(f"定高(开环) hover_bias={hover_bias:+.2f}", {}, seconds)],
        "forward": [Step(f"前进 surge={v:+.2f}", {"surge": v}, seconds)],
        "roll":    [Step(f"横滚 roll={v:+.2f}", {"roll": v}, seconds)],
        "yaw":     [Step(f"转向 yaw={v:+.2f}", {"yaw": v}, seconds)],
    }
    if name != "sequence":
        return table[name]
    return [
        Step(f"① 上升 heave={v:+.2f}", {"heave": v}, seconds, hover=False),
        Step(f"② 定高(开环) hover_bias={hover_bias:+.2f}", {}, max(seconds, 3.0)),
        Step(f"③ 定高前进 surge={v:+.2f}", {"surge": v}, seconds),
        settle,
        Step(f"④ 定高横滚 roll={v:+.2f}", {"roll": v}, seconds),
        settle,
    ]


def step_split(step: Step, hover_bias: float) -> tuple[dict, dict]:
    """拆成 (保持项, 动作项)。

    hover_bias 是浮力补偿, 饱和时不该被牺牲 —— 该让步的是动作幅度。
    见 Allocator.allocate_priority()。
    """
    hold = {"heave": hover_bias} if (step.hover and hover_bias) else {}
    return hold, dict(step.tau)


def step_desc(hold: dict, move: dict) -> str:
    parts = [f"{k}={v:+.2f}" for k, v in move.items()]
    parts += [f"{k}={v:+.2f}(保持)" for k, v in hold.items()]
    return " ".join(parts) or "(全零)"


# ---------------- 离线预览 ----------------
def dry_run(steps: list[Step], alloc: Allocator, hover_bias: float) -> int:
    total = 0.0
    print(f"分配模式={alloc.mode}  单桨上限={alloc.limit}  hover_bias={hover_bias:+.2f}\n")
    any_sat = False
    any_hold_lost = False
    for i, st in enumerate(steps, 1):
        hold_tau, move_tau = step_split(st, hover_bias)
        r = alloc.allocate_priority(hold=hold_tau, move=move_tau)
        print(f"[{i}] {st.label}   {st.seconds:.1f}s")
        print(f"     τ: {step_desc(hold_tau, move_tau)}")
        print(f"     {r.fmt()}")
        if r.hold_scaled:
            any_hold_lost = True
        elif r.saturated:
            any_sat = True
        total += st.seconds
    print(f"\n总时长 {total:.1f}s")
    if any_hold_lost:
        print("❌ 有动作连 hover_bias 都放不下 —— u_max 的余量撑不住悬停,")
        print("   那几段一定会掉深度。提高 --u-max 或降低 --hover-bias。")
    elif any_sat:
        print("⚠ 有动作触发了限幅: 动作幅度被缩, 但 hover_bias 保住了。")
        print("   想要足幅动作就降 τ 或提高 --u-max。")
    print("\n这只是离线预览, 没有连接任何设备。")
    return 0


# ---------------- 实跑 ----------------
def hold(ext, alloc: Allocator, step: Step, hover_bias: float) -> bool:
    """保持一段恒定 τ; 返回 False 表示需要中止。"""
    hold_tau, move_tau = step_split(step, hover_bias)
    r = alloc.allocate_priority(hold=hold_tau, move=move_tau)
    print(f"\n▶ {step.label}   {step.seconds:.1f}s")
    print(f"   τ: {step_desc(hold_tau, move_tau)}")
    print(f"   {r.fmt()}")
    if r.hold_scaled:
        print("   ⚠ u_max 的余量连悬停都撑不住 —— 这一段一定会掉深度")
    elif r.saturated:
        print("   ⚠ 动作幅度被缩, hover_bias 未受影响")

    ext.set_thrust(r.thrust)
    t_end = mono() + step.seconds
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
            print("   SERVO: " + " ".join(str(v) for v in ext.servo))
            next_print = mono() + 0.5
        if ext.latched:
            print("   [中止] 固件看门狗已锁存")
            return False
        if ext.link_lost():
            print("   [中止] 超过 5s 没收到飞控心跳")
            return False
        time.sleep(0.02)
    return True


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="开环运动原语: τ → 8 路推力 (无伪手柄中间层)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="默认连 SITL。真机用 --endpoint udpin:0.0.0.0:14550")
    ap.add_argument("--endpoint", default=SITL_ENDPOINT)
    ap.add_argument("--dry-run", action="store_true",
                    help="只算分配并打印, 不连接任何设备")
    ap.add_argument("--arm", action="store_true",
                    help="允许解锁。不加就只验链路(固件会把 8 路硬写 1500)")
    ap.add_argument("--u-max", type=float, default=0.2,
                    help="单桨幅值上限, 默认 0.2。真机首轮建议 0.1")
    ap.add_argument("--hz", type=float, default=25.0)
    ap.add_argument("--mode", choices=("scale", "clamp"), default="scale",
                    help="饱和处理: scale 保方向(默认) / clamp 同固件混控但方向失真")

    ap.add_argument("--demo", choices=("sequence", "ascend", "descend", "hover",
                                       "forward", "roll", "yaw"),
                    help="预设动作; sequence = 上升→定高→前进→横滚")
    ap.add_argument("--value", type=float, default=0.15,
                    help="预设动作的 τ 幅值, 默认 0.15")
    ap.add_argument("--hover-bias", type=float, default=0.0, dest="hover_bias",
                    help="恒定上推力(开环定高), 叠加到 heave 上。负浮力机器人需要它悬停")
    ap.add_argument("--seconds", type=float, default=2.0, help="每段时长")

    for d in DOFS:
        ap.add_argument(f"--{d}", type=float, default=None, help=f"直接给 {d} 的 τ")
    ap.add_argument("--tau", help="例: heave=0.2,surge=0.3")
    args = ap.parse_args(argv)

    # --- 组装动作序列 ---
    manual: dict[str, float] = {}
    if args.tau:
        manual.update(parse_tau(args.tau))
    for d in DOFS:
        v = getattr(args, d)
        if v is not None:
            manual[d] = v
    if args.demo and manual:
        print("[motion] --demo 和手动指定 τ 只能二选一")
        return 2
    if args.demo:
        steps = build_demo(args.demo, args.value, args.seconds, args.hover_bias)
    elif manual:
        desc = " ".join(f"{k}={v:+.2f}" for k, v in manual.items())
        steps = [Step(desc, manual, args.seconds)]
    else:
        ap.print_help()
        return 2

    # 限幅放在分配器里做, 而不是交给 ExternalThruster 逐电机钳 ——
    # 那样会把按方向缩放好的结果重新扭歪
    alloc = Allocator(AllocatorConfig(), mode=args.mode, limit=args.u_max)

    if args.dry_run:
        return dry_run(steps, alloc, args.hover_bias)

    from external_thruster import ExternalThruster, connect  # 延迟导入: dry-run 不需要

    if args.endpoint != SITL_ENDPOINT and args.arm:
        print("=" * 66)
        print("⚠  正在对**真机**解锁并驱动推进器。")
        print(f"   endpoint={args.endpoint}  u_max={args.u_max}")
        print("   直控层没有混控兜底; 若 sign 尚未标定, 动作方向可能与预期相反。")
        print("=" * 66)

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
                print(f"[motion] ❌ 解锁失败 —— {ext.arm_detail()}")
                return 1
            print("[motion] 已解锁")
            time.sleep(1.0)          # 等 spool up
        else:
            print("[motion] 未加 --arm: 不解锁, 固件会把 8 路硬写 1500 (这本身就是安全验收)")

        for st in steps:
            if not hold(ext, alloc, st, args.hover_bias):
                return 1
        return 0
    except KeyboardInterrupt:
        print("\n[motion] 中断")
        return 130
    finally:
        ext.close()


if __name__ == "__main__":
    sys.exit(main())
