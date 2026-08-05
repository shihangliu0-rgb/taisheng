"""配置持久化：连接参数、速度限幅、键位映射、PID 表等。"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, asdict, field
from typing import Any, Dict, List

CONFIG_DIR = os.path.join(os.path.expanduser("~"), ".upper_computer")
CONFIG_PATH = os.path.join(CONFIG_DIR, "config.json")

DEFAULT_KEYMAP = {
    "forward": "W",
    "back": "S",
    "left": "A",
    "right": "D",
    "ccw": "Q",
    "cw": "E",
    "brake": "Space",
    "boost": "Shift",
    "slow": "Ctrl",
}


@dataclass
class LinkConfig:
    kind: str = "serial"          # serial | tcp
    port: str = ""
    baud: int = 115200
    rtscts: bool = False
    host: str = "192.168.4.1"
    tcp_port: int = 8888
    auto_reconnect: bool = True
    frame_format: str = "binary"  # binary | text


@dataclass
class MotionConfig:
    max_vx: int = 1000            # mm/s
    max_vy: int = 1000            # mm/s
    max_omega: int = 3000         # mrad/s
    boost_scale: float = 1.6
    slow_scale: float = 0.35
    accel_ms: int = 200           # 0->100% 斜坡时间(ms)，0 = 无斜坡
    decel_ms: int = 150
    normalize_diagonal: bool = True
    deadman: bool = True          # 窗口失焦立即归零


@dataclass
class SendConfig:
    mode: str = "realtime"        # realtime | manual
    period_ms: int = 20           # 实时发送周期
    heartbeat_ms: int = 500
    send_zero_on_idle: bool = True  # 无按键时仍周期发 0，防止下位机看门狗误触发


@dataclass
class AppConfig:
    link: LinkConfig = field(default_factory=LinkConfig)
    motion: MotionConfig = field(default_factory=MotionConfig)
    send: SendConfig = field(default_factory=SendConfig)
    keymap: Dict[str, str] = field(default_factory=lambda: dict(DEFAULT_KEYMAP))
    pid_ids: List[int] = field(default_factory=lambda: [0, 1, 2, 3, 0x10, 0x11, 0x20, 0x21])

    # ------------------------------------------------------------------
    def save(self, path: str = CONFIG_PATH) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(asdict(self), f, indent=2, ensure_ascii=False)

    @classmethod
    def load(cls, path: str = CONFIG_PATH) -> "AppConfig":
        cfg = cls()
        if not os.path.exists(path):
            return cfg
        try:
            with open(path, encoding="utf-8") as f:
                d: Dict[str, Any] = json.load(f)
        except Exception:
            return cfg
        for sub, klass in (("link", LinkConfig), ("motion", MotionConfig), ("send", SendConfig)):
            if isinstance(d.get(sub), dict):
                obj = klass()
                for k, v in d[sub].items():
                    if hasattr(obj, k):
                        setattr(obj, k, v)
                setattr(cfg, sub, obj)
        if isinstance(d.get("keymap"), dict):
            cfg.keymap.update(d["keymap"])
        if isinstance(d.get("pid_ids"), list):
            cfg.pid_ids = d["pid_ids"]
        return cfg
