"""键盘映射 -> 运动指令 解算引擎。

支持：
  * 多键同时按下（W+A 斜走、W+E 边走边转）
  * 斜向归一化（避免斜走比直走快 1.41 倍）
  * 加/减速斜坡，手感更线性，也避免电流冲击
  * Shift 加速档 / Ctrl 慢速档
  * 空格制动（最高优先级）
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Set

from .config import MotionConfig


@dataclass
class MotionCommand:
    vx: int = 0
    vy: int = 0
    omega: int = 0
    flags: int = 0

    FLAG_BRAKE = 0x01
    FLAG_BOOST = 0x02
    FLAG_SLOW = 0x04
    FLAG_FIELD = 0x08

    def as_dict(self):
        return {"vx": self.vx, "vy": self.vy, "omega": self.omega,
                "flags": self.flags, "seq_echo": 0}

    def is_zero(self) -> bool:
        return self.vx == 0 and self.vy == 0 and self.omega == 0 and self.flags == 0


class MotionEngine:
    """把「当前按下的动作集合」解算成速度指令。"""

    def __init__(self, cfg: MotionConfig):
        self.cfg = cfg
        self.actions: Set[str] = set()
        self._cur = [0.0, 0.0, 0.0]     # 归一化后的当前输出 -1..1
        self._last_t = time.monotonic()
        self.field_frame = False

    # ---------------- 按键事件 ----------------
    def press(self, action: str):
        self.actions.add(action)

    def release(self, action: str):
        self.actions.discard(action)

    def clear(self):
        self.actions.clear()
        self._cur = [0.0, 0.0, 0.0]

    # ---------------- 解算 ----------------
    def _target_axes(self):
        ax = ay = az = 0.0
        if "forward" in self.actions:
            ax += 1.0
        if "back" in self.actions:
            ax -= 1.0
        if "left" in self.actions:
            ay += 1.0
        if "right" in self.actions:
            ay -= 1.0
        if "ccw" in self.actions:
            az += 1.0
        if "cw" in self.actions:
            az -= 1.0
        # 斜向归一化：只对平移分量做，旋转独立
        if self.cfg.normalize_diagonal:
            mag = math.hypot(ax, ay)
            if mag > 1.0:
                ax, ay = ax / mag, ay / mag
        return ax, ay, az

    def update(self) -> MotionCommand:
        now = time.monotonic()
        dt = max(0.0, min(now - self._last_t, 0.2))
        self._last_t = now

        braking = "brake" in self.actions
        tgt = (0.0, 0.0, 0.0) if braking else self._target_axes()

        acc_t = max(self.cfg.accel_ms, 1) / 1000.0
        dec_t = max(self.cfg.decel_ms, 1) / 1000.0
        for i in range(3):
            t = tgt[i]
            c = self._cur[i]
            if self.cfg.accel_ms <= 0 and self.cfg.decel_ms <= 0:
                self._cur[i] = t
                continue
            rising = abs(t) > abs(c) and (t == 0 or c * t >= 0)
            step = dt / (acc_t if rising else dec_t)
            if braking:
                step = dt / dec_t
            if t > c:
                self._cur[i] = min(t, c + step)
            else:
                self._cur[i] = max(t, c - step)

        scale = 1.0
        flags = 0
        if "boost" in self.actions:
            scale *= self.cfg.boost_scale
            flags |= MotionCommand.FLAG_BOOST
        if "slow" in self.actions:
            scale *= self.cfg.slow_scale
            flags |= MotionCommand.FLAG_SLOW
        if braking:
            flags |= MotionCommand.FLAG_BRAKE
        if self.field_frame:
            flags |= MotionCommand.FLAG_FIELD

        def clip16(v):
            return max(-32768, min(32767, int(round(v))))

        cmd = MotionCommand(
            vx=clip16(self._cur[0] * self.cfg.max_vx * scale),
            vy=clip16(self._cur[1] * self.cfg.max_vy * scale),
            omega=clip16(self._cur[2] * self.cfg.max_omega * scale),
            flags=flags,
        )
        if braking:
            cmd.vx = cmd.vy = cmd.omega = 0
        return cmd
