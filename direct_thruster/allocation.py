#!/usr/bin/env python3
"""推力分配 — 机体系广义力 τ → 8 路归一化推力。

direct_thruster 线的执行器模型。**不是伪手柄的翻译层**:

    伪手柄 (waypoint 线)  x/y/z/r, 带着协议包袱 —— z 中位 500、throttle_gain、
                          gain、sign_z 这些是 MANUAL_CONTROL 这个接口的产物
    广义力 τ (本线)        机体系的力与力矩, 纯物理量, 没有协议包袱

控制器输出 τ, 分配器把 τ 摊到 8 个桨上。这样控制器**不需要知道底下有几个桨** ——
将来换布局、或者某个桨坏了做容错分配, 控制器一行不用动; MPC 的输出天然也是 τ。

### 矩阵从哪来, 以及为什么只当初值

列向量取自 ArduSub 4.1.2 `SUB_FRAME_VECTORED_6DOF` 的几何
(`libraries/AP_Motors/AP_Motors6DOF.cpp`, `add_motor_raw_6dof` 那 8 行,
FRAME_CONFIG=2 就是实机那套)。**只当初值** —— 标度和符号由 direct_thruster 用自己的
实验数据标, 不继承固件那套单位: 固件的 throttle 列整列是 −1、各列也没做归一化,
那是为它自己的混控设计的。

几何上 BlueROV2 Heavy:
  Motor1-4  45° 矢量布置的水平桨  → surge / sway / yaw
  Motor5-8  垂直桨                → heave / roll / pitch

六个列向量两两正交(`--check` 会验), 所以各自由度在**几何上**解耦: 纯 heave 不产生
roll/pitch 力矩。但解耦是几何意义上的, 实际还有水动力耦合, 那要靠闭环去压。

### 饱和的两种处理

- `scale`(默认): 整体等比缩放, **保持方向**。想往前就只是慢一点, 不会歪。
- `clamp`: 逐电机钳位, 和固件混控一致, 但会**扭曲合成方向**。只在和固件对拍时用。

用法:
  python direct_thruster/allocation.py --show
  python direct_thruster/allocation.py --check
  python direct_thruster/allocation.py --tau heave=0.3
  python direct_thruster/allocation.py --tau surge=0.4,yaw=0.2 --limit 0.3
  python direct_thruster/allocation.py --explain roll
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

NMOT = 8
DOFS = ("surge", "sway", "heave", "roll", "pitch", "yaw")

DOF_CN = {
    "surge": "前进(+前)", "sway": "横移(+右)", "heave": "升沉(+待实测)",
    "roll": "横滚(+待实测)", "pitch": "俯仰(+待实测)", "yaw": "偏航(+待实测)",
}

# 列向量: 每个自由度在 8 个桨上的分布。来自 ArduSub SUB_FRAME_VECTORED_6DOF。
#                     M1    M2    M3    M4    M5    M6    M7    M8
GEOMETRY: dict[str, tuple[float, ...]] = {
    "surge": (-1.0, -1.0, +1.0, +1.0,  0.0,  0.0,  0.0,  0.0),
    "sway":  (+1.0, -1.0, +1.0, -1.0,  0.0,  0.0,  0.0,  0.0),
    "yaw":   (+1.0, -1.0, -1.0, +1.0,  0.0,  0.0,  0.0,  0.0),
    "heave": ( 0.0,  0.0,  0.0,  0.0, -1.0, -1.0, -1.0, -1.0),
    "roll":  ( 0.0,  0.0,  0.0,  0.0, +1.0, -1.0, +1.0, -1.0),
    "pitch": ( 0.0,  0.0,  0.0,  0.0, -1.0, -1.0, +1.0, +1.0),
}


@dataclass
class AllocatorConfig:
    """direct_thruster 自己的标定量。不读 config/vehicle.yaml —— 那是 waypoint 线的。

    gain: 每个自由度的标度。τ=1 时该自由度吃掉多少归一化推力。
          默认全 1.0, 即"τ 直接就是列向量的系数"。实测后按各轴的实际效率回填 ——
          水平桨是 45° 布置的, 同样的 τ 在 surge 和 yaw 上产生的效果不一样。
    sign: 实测方向修正, 默认全 +1。
          **必须干测逐路确认**: 直控绕过了混控, MOT_n_DIRECTION 的生效链路和原来不同,
          加上螺旋桨的物理安装, "正值=哪个方向"推不出来只能试出来。
          waypoint 线那个 sign_z=-1 是针对混控链路标的, 不能搬过来。
    """
    gain: dict[str, float] = field(default_factory=lambda: {d: 1.0 for d in DOFS})
    sign: dict[str, int] = field(default_factory=lambda: {d: 1 for d in DOFS})

    def __post_init__(self) -> None:
        for d in DOFS:
            self.gain.setdefault(d, 1.0)
            self.sign.setdefault(d, 1)
            if self.sign[d] not in (1, -1):
                raise ValueError(f"sign[{d}] 必须是 ±1, 收到 {self.sign[d]}")


@dataclass
class Allocation:
    """一次分配的结果, 连同它是怎么被限幅的。"""
    thrust: list[float]
    scale: float = 1.0          # <1 表示整体缩放过
    clamped: bool = False       # clamp 模式下是否真的钳到了
    hold_scaled: bool = False   # 优先级分配时连"必须保住"的那部分都没放下
    priority: bool = False      # 是否出自 allocate_priority (决定 fmt 怎么描述缩放)
    tau_in: dict[str, float] = field(default_factory=dict)

    @property
    def saturated(self) -> bool:
        return self.scale < 1.0 - 1e-9 or self.clamped

    def fmt(self) -> str:
        body = " ".join(f"{v:+.3f}" for v in self.thrust)
        tag = ""
        if self.hold_scaled:
            tag = f"   [⚠ 连保持项都放不下, 整体缩到 ×{self.scale:.3f}]"
        elif self.scale < 1.0 - 1e-9:
            tag = (f"   [动作项缩放 ×{self.scale:.3f}, 保持项未动]" if self.priority
                   else f"   [整体缩放 ×{self.scale:.3f}]")
        elif self.clamped:
            tag = "   [逐电机钳位, 方向已失真]"
        return f"M1..M8: {body}{tag}"


class Allocator:
    def __init__(self, cfg: AllocatorConfig | None = None, *,
                 mode: str = "scale", limit: float = 1.0) -> None:
        if mode not in ("scale", "clamp"):
            raise ValueError("mode 只能是 scale 或 clamp")
        if not 0.0 < limit <= 1.0:
            raise ValueError("limit 必须在 (0, 1]")
        self.cfg = cfg or AllocatorConfig()
        self.mode = mode
        self.limit = float(limit)

    # ---------------- 核心 ----------------
    def allocate(self, **tau: float) -> Allocation:
        """τ → 8 路归一化推力。

        limit 在**这一层**生效, 而不是交给 ExternalThruster 的 u_max 去逐电机钳 ——
        那样会把刚刚按方向缩放好的结果重新扭歪。
        """
        for d in tau:
            if d not in GEOMETRY:
                raise ValueError(f"未知自由度 {d!r}; 可用: {', '.join(DOFS)}")

        raw = [0.0] * NMOT
        for d, v in tau.items():
            if not v:
                continue
            k = self.cfg.sign[d] * self.cfg.gain[d] * float(v)
            col = GEOMETRY[d]
            for i in range(NMOT):
                raw[i] += k * col[i]

        peak = max((abs(v) for v in raw), default=0.0)
        if peak <= self.limit + 1e-12:
            return Allocation(raw, tau_in=dict(tau))

        if self.mode == "scale":
            s = self.limit / peak
            return Allocation([v * s for v in raw], scale=s, tau_in=dict(tau))
        out = [max(-self.limit, min(self.limit, v)) for v in raw]
        return Allocation(out, clamped=True, tau_in=dict(tau))

    def allocate_priority(self, *, hold: dict[str, float],
                          move: dict[str, float]) -> Allocation:
        """优先级分配: hold 先占住, move 缩放去填剩下的余量。

        为什么需要这个 —— "定高横滚"这种叠加动作一旦饱和, 均匀缩放会把
        hover_bias 和 roll **一起**缩。hover_bias 是浮力补偿, 缩了就下沉;
        该让步的是横滚幅度, 不是垂向保持力。

        做法: 先算 hold 的分配 b, 再求最大的 s∈[0,1] 使得逐电机
        |b_i + s·c_i| ≤ limit, 结果取 b + s·c。
        """
        b = self._raw(hold)
        c = self._raw(move)

        peak_b = max((abs(v) for v in b), default=0.0)
        if peak_b > self.limit + 1e-12:
            # 连 hold 自己都放不下 —— 这时只能缩 hold, 并且必须让调用方知道:
            # 它意味着 u_max 给的余量撑不住悬停, 动作一定会掉深度。
            k = self.limit / peak_b
            out = [v * k for v in b]
            r = Allocation(out, scale=k, priority=True, tau_in={**hold, **move})
            r.hold_scaled = True
            return r

        s = 1.0
        for bi, ci in zip(b, c):
            if abs(ci) < 1e-12:
                continue
            lim = self.limit if ci > 0 else -self.limit
            s = min(s, (lim - bi) / ci)
        s = max(0.0, min(1.0, s))
        out = [bi + s * ci for bi, ci in zip(b, c)]
        return Allocation(out, scale=s, priority=True, tau_in={**hold, **move})

    def _raw(self, tau: dict[str, float]) -> list[float]:
        """不做任何限幅的线性叠加。"""
        raw = [0.0] * NMOT
        for d, v in tau.items():
            if d not in GEOMETRY:
                raise ValueError(f"未知自由度 {d!r}; 可用: {', '.join(DOFS)}")
            if not v:
                continue
            k = self.cfg.sign[d] * self.cfg.gain[d] * float(v)
            col = GEOMETRY[d]
            for i in range(NMOT):
                raw[i] += k * col[i]
        return raw

    # ---------------- 给人看的 ----------------
    def explain(self, dof: str) -> str:
        """这个自由度靠哪几个桨、各自什么符号 —— 用来凭逻辑判断各桨该怎么动。"""
        if dof not in GEOMETRY:
            raise ValueError(f"未知自由度 {dof!r}")
        col = GEOMETRY[dof]
        sgn = self.cfg.sign[dof]
        plus = [f"M{i + 1}" for i in range(NMOT) if sgn * col[i] > 0]
        minus = [f"M{i + 1}" for i in range(NMOT) if sgn * col[i] < 0]
        idle = [f"M{i + 1}" for i in range(NMOT) if col[i] == 0]
        lines = [f"{dof} {DOF_CN[dof]}   gain={self.cfg.gain[dof]} sign={sgn:+d}"]
        lines.append(f"  正 τ 时推正向: {' '.join(plus) if plus else '(无)'}")
        lines.append(f"  正 τ 时推反向: {' '.join(minus) if minus else '(无)'}")
        lines.append(f"  不参与:        {' '.join(idle) if idle else '(无)'}")
        return "\n".join(lines)

    def table(self) -> str:
        head = "        " + "".join(f"  M{i + 1}  " for i in range(NMOT))
        rows = [head, "        " + "-" * (6 * NMOT)]
        for d in DOFS:
            cells = "".join(f"{v:+5.1f} " for v in GEOMETRY[d])
            rows.append(f"{d:<7} {cells}  gain={self.cfg.gain[d]:<5} sign={self.cfg.sign[d]:+d}")
        return "\n".join(rows)


def check_orthogonal(verbose: bool = True) -> bool:
    """六个列向量应两两正交 —— 即各自由度在几何上解耦。

    不正交意味着"纯 heave 指令会顺带产生 roll/pitch 力矩", 开环动作会歪。
    这是矩阵抄对没抄对的一个客观判据, 不用连机就能验。
    """
    ok = True
    for i, a in enumerate(DOFS):
        for b in DOFS[i + 1:]:
            dot = sum(x * y for x, y in zip(GEOMETRY[a], GEOMETRY[b]))
            good = abs(dot) < 1e-9
            ok = ok and good
            if verbose and not good:
                print(f"  [FAIL] {a} · {b} = {dot:+.1f}  (应为 0)")
    if verbose and ok:
        print("  [PASS] 六个自由度两两正交 —— 几何上解耦")
    return ok


def parse_tau(text: str) -> dict[str, float]:
    """'heave=0.3,surge=0.2' → {'heave': 0.3, 'surge': 0.2}"""
    out: dict[str, float] = {}
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "=" not in part:
            raise ValueError(f"看不懂 {part!r}; 格式是 自由度=数值, 逗号分隔")
        k, v = part.split("=", 1)
        k = k.strip()
        if k not in GEOMETRY:
            raise ValueError(f"未知自由度 {k!r}; 可用: {', '.join(DOFS)}")
        out[k] = float(v)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="推力分配: 机体系广义力 τ → 8 路归一化推力")
    ap.add_argument("--show", action="store_true", help="打印分配矩阵")
    ap.add_argument("--check", action="store_true", help="验证列向量两两正交")
    ap.add_argument("--tau", help="例: heave=0.3  或  surge=0.4,yaw=0.2")
    ap.add_argument("--explain", metavar="DOF", help="某个自由度靠哪几个桨")
    ap.add_argument("--mode", choices=("scale", "clamp"), default="scale")
    ap.add_argument("--limit", type=float, default=1.0, help="单桨幅值上限")
    args = ap.parse_args(argv)

    if not any((args.show, args.check, args.tau, args.explain)):
        ap.print_help()
        return 2

    alloc = Allocator(mode=args.mode, limit=args.limit)

    if args.show:
        print("分配矩阵 (机体系广义力 → 8 路归一化推力)")
        print(f"  来源: ArduSub SUB_FRAME_VECTORED_6DOF 几何, 只当初值")
        print(f"  模式: {args.mode}   单桨上限: {args.limit}\n")
        print(alloc.table())
        print("\n  Motor1-4 = 45° 矢量布置的水平桨 (surge/sway/yaw)")
        print("  Motor5-8 = 垂直桨 (heave/roll/pitch)")
        print("\n  ⚠ sign 全为 +1 是**未标定**状态。正值到底对应哪个方向,")
        print("    必须干测逐路确认 —— 直控绕过混控, 符号链路和官方固件不同。")

    if args.check:
        print("\n正交性检查:")
        if not check_orthogonal():
            return 1

    if args.explain:
        print()
        print(alloc.explain(args.explain))

    if args.tau:
        tau = parse_tau(args.tau)
        r = alloc.allocate(**tau)
        desc = " ".join(f"{k}={v:+.3f}" for k, v in tau.items())
        print(f"\nτ: {desc}")
        print(f"   {r.fmt()}")
        moving = [f"M{i + 1}" for i, v in enumerate(r.thrust) if abs(v) > 1e-9]
        print(f"   参与的桨: {' '.join(moving) if moving else '(无)'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
