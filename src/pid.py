#!/usr/bin/env python3
"""深度 PID 控制器(P4 基线)。

约定:深度 z 向下为正;输出 u 为归一化 heave 指令(+u 下潜)。
  误差 e = setpoint - depth
  u = Kp*e + Ki*∫e - Kd*depth_rate      (微分作用于测量,避免设定值突跳)
含:积分限幅 + 饱和条件积分(anti-windup) + 输出限幅。

控制器接口(P4/P5 通用):reset() 与 compute(setpoint, depth, depth_rate, dt) -> u。
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PID:
    Kp: float
    Ki: float
    Kd: float
    i_limit: float = 0.3      # 积分项对输出的贡献上限 |Ki*∫e|
    u_limit: float = 0.3      # 输出限幅 |u|

    def __post_init__(self):
        self.reset()

    def reset(self) -> None:
        self.integ = 0.0
        self.last_u = 0.0

    def compute(self, setpoint: float, depth: float, depth_rate: float, dt: float) -> float:
        e = setpoint - depth
        # 先算未含本步积分增量的比例+微分
        p = self.Kp * e
        d = -self.Kd * depth_rate
        # 试探性积分
        integ_try = self.integ + e * dt
        i_term = self.Ki * integ_try
        # 积分限幅
        if self.Ki > 0:
            i_cap = self.i_limit / self.Ki
            integ_try = max(-i_cap, min(i_cap, integ_try))
            i_term = self.Ki * integ_try
        u_unsat = p + i_term + d
        u = max(-self.u_limit, min(self.u_limit, u_unsat))
        # 条件积分:输出饱和且继续朝同向积分会加剧饱和 → 冻结本步积分
        saturated = (u != u_unsat)
        if not (saturated and (e * u_unsat > 0)):
            self.integ = integ_try
        self.last_u = u
        return u

    @classmethod
    def from_config(cls, cfg: dict) -> "PID":
        p = cfg.get("pid", {})
        return cls(Kp=float(p.get("Kp", 3.0)), Ki=float(p.get("Ki", 0.5)),
                   Kd=float(p.get("Kd", 1.0)), i_limit=float(p.get("i_limit", 0.3)),
                   u_limit=float(p.get("u_limit", 0.3)))
