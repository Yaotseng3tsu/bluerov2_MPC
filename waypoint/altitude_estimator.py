#!/usr/bin/env python3
"""离底高度估计器 —— 预测-校正 + 新息门控 + 跳变再基准。

动机(2026-09-17 实测教训):原先把**原始 DVL 高度**直接喂 PID,并用"固定变化率阈值"
剔除外点、丢帧时冻结上一帧。结果是:高度外点(跳到 2.5m)会撞护栏中止任务,
丢帧时估计停滞,微分出来的 alt_rate 还很脏。

参考 ArduSub / ArduPilot 的做法改成三条:
  1) **控制器吃估计值,不吃原始传感器**
     ArduSub `mode_althold.cpp` 用的是 `position_control->get_pos_estimate_U_m()`
     (EKF 融合后的位置),而不是某个传感器的瞬时读数。
  2) **丢帧用速度继续预测,而不是冻结**
     我们有 DVL 的 `vz`,正好做惯性桥接:没有新量测时 alt 仍按 rate 外推。
  3) **跳变 → 重置基准并发布 Δ,让控制器平移目标而不是去追**
     对应 `surface_tracking.cpp` 里 glitch 清除后调
     `init_pos_terrain_D_m(-rf_state.terrain_u_m)` —— 目标被重设到当前量测,
     而不是让控制器去消掉那个阶跃。ArduSub 贴底时也是
     `set_pos_desired_U_cm(MAX(get_pos_estimate_U_m()*100 + 10, ...))`,
     即目标锚在**当前估计**上。

门控用新息(innovation)而非固定阈值:|y| > gate·sqrt(S),S = P + R。
这样门限随估计不确定度自适应 —— 刚复位/长时间丢帧后 P 大、门限自动放宽,
稳态时 P 小、门限收紧。(这是标准 EKF 量测校验写法;ArduPilot 的
rangefinder glitch 具体阈值在 AP_SurfaceDistance 里,本次未取到源码,故此处按
通用 EKF 门控实现,不照抄其数值。)

状态 x = [alt, rate]^T,alt 向上为正(离底越远越大),rate = d(alt)/dt。
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class AltEstimate:
    alt: float           # 融合后的离底高度 (m)
    rate: float          # 上升率 (m/s, + = 远离池底)
    initialized: bool    # 是否已有有效初值
    reset_delta: float   # 本次发生基准跳变的 Δ (m);控制器应把目标同量平移。无跳变时 0
    rejected: int        # 当前连续被门控拒绝的量测数
    age_s: float         # 距最近一次被接受的量测多久 (s)


class AltitudeEstimator:
    """2 状态卡尔曼 (alt, rate) + 新息门控 + 跳变再基准。"""

    def __init__(self,
                 q_alt: float = 0.01,      # 高度过程噪声 (m^2/s)
                 q_rate: float = 0.15,     # 速率过程噪声 (m^2/s^3) —— 越大越信量测
                 r_alt: float = 0.0025,    # 量测噪声 (m^2);DVL 高度 std≈5cm → 0.05^2
                 r_vz: float = 0.01,       # DVL 垂向速度量测噪声 (m/s)^2
                 gate_sigma: float = 3.0,  # 新息门控 (倍 sigma)
                 reset_after: int = 6,     # 连续被拒这么多次 → 判为真实跳变, 重置基准
                 max_coast_s: float = 2.0, # 无有效量测时最多外推多久 (超过则判失效)
                 init_n: int = 3           # 初始化取前 N 帧的**中位数**, 避免拿外点开局
                 ) -> None:
        self.init_n = max(1, init_n)
        self.q_alt, self.q_rate = q_alt, q_rate
        self.r_alt, self.r_vz = r_alt, r_vz
        self.gate_sigma = gate_sigma
        self.reset_after = reset_after
        self.max_coast_s = max_coast_s
        self.reset()

    def reset(self) -> None:
        self.alt = 0.0
        self.rate = 0.0
        # 协方差 [[P00,P01],[P10,P11]]
        self.P = [[1.0, 0.0], [0.0, 1.0]]
        self.initialized = False
        self._rejected = 0
        self._since_accept = 0.0
        self._pending = None     # 连续被拒时记住的候选值(用于再基准)
        self._gate_frozen = None # 首次拒绝时冻结门限(见 update_alt)
        self._init_buf = []      # 初始化缓冲(取中位数)

    # ---------------- 预测 ----------------
    def predict(self, dt: float) -> None:
        """按当前速率外推。丢帧期间持续调用 → 用速度桥接,而不是冻结。"""
        if not self.initialized or dt <= 0:
            return
        self.alt += self.rate * dt
        self._since_accept += dt
        # F = [[1, dt], [0, 1]]
        P = self.P
        p00 = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + self.q_alt * dt
        p01 = P[0][1] + dt * P[1][1]
        p10 = P[1][0] + dt * P[1][1]
        p11 = P[1][1] + self.q_rate * dt
        self.P = [[p00, p01], [p10, p11]]

    # ---------------- 量测:DVL 垂向速度 ----------------
    def update_rate(self, vz_up: float) -> None:
        """用 DVL 垂向速度直接校正 rate(H = [0, 1])。vz_up 需已转成"向上为正"。"""
        if not self.initialized:
            return
        S = self.P[1][1] + self.r_vz
        if S <= 0:
            return
        k0 = self.P[0][1] / S
        k1 = self.P[1][1] / S
        y = vz_up - self.rate
        self.alt += k0 * y
        self.rate += k1 * y
        p00 = self.P[0][0] - k0 * self.P[1][0]
        p01 = self.P[0][1] - k0 * self.P[1][1]
        p10 = (1 - k1) * self.P[1][0]
        p11 = (1 - k1) * self.P[1][1]
        self.P = [[p00, p01], [p10, p11]]

    # ---------------- 量测:DVL 高度(带门控与再基准) ----------------
    def update_alt(self, z: float) -> float:
        """用高度量测校正。返回 reset_delta:非 0 表示发生了基准跳变,
        控制器应把目标**同量平移**(对应 ArduPilot 的 init_pos_terrain_*)。"""
        if not self.initialized:
            # 稳健初始化: 取前 init_n 帧的中位数。单帧开局时若恰好撞上外点,
            # 整个估计会以错误基准起步(实测 seed3: 以 2.5m 外点开局)。
            self._init_buf.append(z)
            if len(self._init_buf) < self.init_n:
                return 0.0
            med = sorted(self._init_buf)[len(self._init_buf) // 2]
            self.alt, self.rate = med, 0.0
            self.P = [[self.r_alt, 0.0], [0.0, 1.0]]
            self.initialized = True
            self._rejected = 0
            self._since_accept = 0.0
            self._init_buf = []
            return 0.0

        S = self.P[0][0] + self.r_alt          # 新息方差
        y = z - self.alt                        # 新息
        # 门限必须在**首次拒绝时冻结**:否则被拒期间没有量测校正, P 持续膨胀 →
        # 门限 3σ 跟着变宽 → 还没数到 reset_after 就把跳变"合法"接受了,
        # 再基准那条路永远不触发。(与 go_forward 第一版 yaw 滤波同一个坑。)
        if self._gate_frozen is None:
            gate = self.gate_sigma * (S ** 0.5)
        else:
            gate = self._gate_frozen

        if abs(y) > gate:
            # 门控拒绝。连续多次都指向同一新值 → 不是尖刺,是真实跳变/地形变化
            self._rejected += 1
            self._pending = z
            if self._gate_frozen is None:
                self._gate_frozen = gate        # 冻结,直到接受或完成再基准
            if self._rejected >= self.reset_after:
                delta = z - self.alt
                self.alt = z                    # 重置基准到当前量测
                self.rate = 0.0
                self.P = [[self.r_alt, 0.0], [0.0, 1.0]]
                self._rejected = 0
                self._since_accept = 0.0
                self._pending = None
                self._gate_frozen = None
                return delta                    # ← 发布跳变量, 让控制器平移目标
            return 0.0

        # 正常校正
        k0 = self.P[0][0] / S
        k1 = self.P[1][0] / S
        self.alt += k0 * y
        self.rate += k1 * y
        p00 = (1 - k0) * self.P[0][0]
        p01 = (1 - k0) * self.P[0][1]
        p10 = self.P[1][0] - k1 * self.P[0][0]
        p11 = self.P[1][1] - k1 * self.P[0][1]
        self.P = [[p00, p01], [p10, p11]]
        self._rejected = 0
        self._since_accept = 0.0
        self._gate_frozen = None
        return 0.0

    # ---------------- 取值 ----------------
    def estimate(self, reset_delta: float = 0.0) -> AltEstimate:
        return AltEstimate(alt=self.alt, rate=self.rate,
                           initialized=self.initialized and self._since_accept <= self.max_coast_s,
                           reset_delta=reset_delta, rejected=self._rejected,
                           age_s=self._since_accept)
