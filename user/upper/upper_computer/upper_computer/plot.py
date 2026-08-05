"""实时曲线控件（纯 QPainter，无需 pyqtgraph/matplotlib）。

特性：
  * 大容量环形缓冲（默认每通道 20 万点，约合 50Hz 下 1 小时），可回溯查看历史
  * 鼠标拖动平移 / 滚轮缩放 / 双击回到最新（跟随模式）
  * 十字光标读数，显示光标处各通道数值
  * 每通道独立颜色、可单独显示隐藏
  * 支持导出 CSV
"""

from __future__ import annotations

import csv
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

from PyQt5.QtCore import Qt, QPointF, pyqtSignal
from PyQt5.QtGui import QPainter, QPen, QColor, QFont, QBrush
from PyQt5.QtWidgets import QWidget

PALETTE = [
    "#4FC3F7", "#FF7043", "#AED581", "#FFD54F", "#BA68C8",
    "#4DB6AC", "#F06292", "#9575CD", "#DCE775", "#64B5F6",
    "#FFB74D", "#81C784", "#E57373", "#7986CB", "#4DD0E1",
    "#FFF176", "#A1887F", "#90A4AE", "#F48FB1", "#80CBC4",
]

# 默认每通道保留点数。50Hz 下 200000 点 ≈ 66 分钟
DEFAULT_CAPACITY = 200_000


class Series:
    __slots__ = ("name", "color", "visible", "t", "v")

    def __init__(self, name: str, color: str, capacity: int):
        self.name = name
        self.color = color
        self.visible = True
        self.t: Deque[float] = deque(maxlen=capacity)
        self.v: Deque[float] = deque(maxlen=capacity)

    def append(self, t: float, v: float):
        self.t.append(t)
        self.v.append(v)

    def __len__(self):
        return len(self.t)


class ScopeWidget(QWidget):
    """可回溯的多通道示波器。"""

    cursorMoved = pyqtSignal(float)

    def __init__(self, parent=None, capacity: int = DEFAULT_CAPACITY):
        super().__init__(parent)
        self.capacity = capacity
        self.series: Dict[str, Series] = {}
        self._color_i = 0
        self.setMinimumHeight(220)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.NoFocus)   # 不抢键盘映射的焦点

        self.auto_scale = True
        self.y_min, self.y_max = -1.0, 1.0
        self.window_s = 10.0          # 可视时间窗宽度
        self.follow = True            # 跟随最新数据
        self.view_end: Optional[float] = None   # 非跟随时的右边界时间
        self.paused = False           # 暂停 = 停止采集新点

        self._drag_x: Optional[int] = None
        self._drag_end: float = 0.0
        self._cursor_x: Optional[int] = None
        self._t_latest = 0.0

    # ------------------------------------------------------------------
    # 数据
    # ------------------------------------------------------------------
    def ensure_series(self, name: str) -> Series:
        s = self.series.get(name)
        if s is None:
            s = Series(name, PALETTE[self._color_i % len(PALETTE)], self.capacity)
            self._color_i += 1
            self.series[name] = s
        return s

    def add_point(self, name: str, t: float, v: float):
        if self.paused:
            return
        self.ensure_series(name).append(t, v)
        if t > self._t_latest:
            self._t_latest = t

    def set_visible(self, name: str, vis: bool):
        if name in self.series:
            self.series[name].visible = vis
            self.update()

    def remove_series(self, name: str):
        self.series.pop(name, None)
        self.update()

    def clear(self):
        self.series.clear()
        self._color_i = 0
        self._t_latest = 0.0
        self.follow = True
        self.view_end = None
        self.update()

    def set_capacity(self, capacity: int):
        """调整缓冲深度，保留已有数据。"""
        self.capacity = capacity
        for s in self.series.values():
            t = deque(s.t, maxlen=capacity)
            v = deque(s.v, maxlen=capacity)
            s.t, s.v = t, v
        self.update()

    def point_count(self) -> int:
        return sum(len(s) for s in self.series.values())

    def time_span(self) -> float:
        lo, hi = None, None
        for s in self.series.values():
            if not s.t:
                continue
            lo = s.t[0] if lo is None else min(lo, s.t[0])
            hi = s.t[-1] if hi is None else max(hi, s.t[-1])
        return (hi - lo) if (lo is not None and hi is not None) else 0.0

    # ------------------------------------------------------------------
    def export_csv(self, path: str) -> int:
        """导出为 CSV（按时间戳并集对齐，缺失留空）。返回行数。"""
        names = [n for n, s in self.series.items() if len(s)]
        if not names:
            return 0
        stamps = sorted({t for n in names for t in self.series[n].t})
        idx = {n: 0 for n in names}
        rows = 0
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["time_s"] + names)
            for ts in stamps:
                row = [f"{ts:.4f}"]
                for n in names:
                    s = self.series[n]
                    i = idx[n]
                    if i < len(s.t) and abs(s.t[i] - ts) < 1e-9:
                        row.append(f"{s.v[i]:.6g}")
                        idx[n] = i + 1
                    else:
                        row.append("")
                w.writerow(row)
                rows += 1
        return rows

    # ------------------------------------------------------------------
    # 视图控制
    # ------------------------------------------------------------------
    def _view_range(self) -> Tuple[float, float]:
        end = self._t_latest if self.follow else (self.view_end or self._t_latest)
        return end - self.window_s, end

    def go_live(self):
        self.follow = True
        self.view_end = None
        self.update()

    def wheelEvent(self, e):
        # 滚轮缩放时间窗，以光标位置为锚点
        d = e.angleDelta().y()
        if d == 0:
            return
        factor = 0.8 if d > 0 else 1.25
        t_min, t_max = self._view_range()
        left, pw = self._plot_geom()[0], self._plot_geom()[2]
        frac = min(max((e.x() - left) / max(pw, 1), 0.0), 1.0)
        anchor = t_min + (t_max - t_min) * frac
        new_win = min(max(self.window_s * factor, 0.05), 36000.0)
        new_end = anchor + new_win * (1 - frac)
        self.window_s = new_win
        if new_end < self._t_latest - 1e-6:
            self.follow = False
            self.view_end = new_end
        else:
            self.go_live()
        self.update()

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._drag_x = e.x()
            self._drag_end = self._view_range()[1]

    def mouseMoveEvent(self, e):
        self._cursor_x = e.x()
        if self._drag_x is not None:
            pw = max(self._plot_geom()[2], 1)
            dt = (self._drag_x - e.x()) / pw * self.window_s
            new_end = self._drag_end + dt
            if new_end >= self._t_latest:
                self.go_live()
            else:
                self.follow = False
                self.view_end = new_end
        self.update()

    def mouseReleaseEvent(self, e):
        self._drag_x = None

    def mouseDoubleClickEvent(self, e):
        self.go_live()

    def leaveEvent(self, e):
        self._cursor_x = None
        self.update()

    # ------------------------------------------------------------------
    def _plot_geom(self):
        left, top, right, bottom = 62, 24, 12, 26
        return left, top, max(1, self.width() - left - right), max(1, self.height() - top - bottom)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#1A1A1A"))
        left, top, pw, ph = self._plot_geom()

        vis = [s for s in self.series.values() if s.visible and len(s)]
        if not vis:
            p.setPen(QColor("#666"))
            p.drawText(self.rect(), Qt.AlignCenter,
                       "无数据 —— 请在下方勾选要显示的通道并开启数据流")
            return

        t_min, t_max = self._view_range()
        span = max(t_max - t_min, 1e-6)

        # Y 轴自适应（只统计可视窗口内的点）
        vals: List[float] = []
        for s in vis:
            for t, v in zip(s.t, s.v):
                if t_min <= t <= t_max:
                    vals.append(v)
        if self.auto_scale and vals:
            lo, hi = min(vals), max(vals)
            if hi - lo < 1e-9:
                lo, hi = lo - 1.0, hi + 1.0
            pad = (hi - lo) * 0.12
            self.y_min, self.y_max = lo - pad, hi + pad
        yr = max(self.y_max - self.y_min, 1e-9)

        def X(t):
            return left + pw * (t - t_min) / span

        def Y(v):
            return top + ph * (self.y_max - v) / yr

        # 网格 + 刻度
        p.setFont(QFont("Consolas", 8))
        for i in range(5):
            y = top + ph * i / 4
            p.setPen(QPen(QColor("#2E2E2E"), 1))
            p.drawLine(left, int(y), left + pw, int(y))
            p.setPen(QColor("#888"))
            p.drawText(2, int(y) + 4, f"{self.y_max - yr * i / 4:10.4g}")
        for i in range(6):
            x = left + pw * i / 5
            p.setPen(QPen(QColor("#2E2E2E"), 1))
            p.drawLine(int(x), top, int(x), top + ph)
            p.setPen(QColor("#888"))
            p.drawText(int(x) - 18, top + ph + 16, f"{t_min + span * i / 5:.2f}s")

        # 曲线（按像素抽稀，10 万点也不卡）
        p.setClipRect(left, top, pw, ph)
        for s in vis:
            pen = QPen(QColor(s.color), 1.6)
            p.setPen(pen)
            pts: List[QPointF] = []
            last_px = -1e9
            step_t = span / max(pw * 2, 1)
            prev_t = -1e18
            for t, v in zip(s.t, s.v):
                if t < t_min or t > t_max:
                    continue
                if t - prev_t < step_t:
                    continue
                prev_t = t
                pts.append(QPointF(X(t), Y(v)))
            if len(pts) > 1:
                p.drawPolyline(*pts)
            elif len(pts) == 1:
                p.drawEllipse(pts[0], 2, 2)
        p.setClipping(False)

        # 十字光标 + 读数
        if self._cursor_x is not None and left <= self._cursor_x <= left + pw:
            ct = t_min + span * (self._cursor_x - left) / pw
            p.setPen(QPen(QColor("#777"), 1, Qt.DashLine))
            p.drawLine(self._cursor_x, top, self._cursor_x, top + ph)
            box: List[Tuple[str, str, float]] = []
            for s in vis:
                v = _nearest(s, ct)
                if v is not None:
                    box.append((s.name, s.color, v))
            if box:
                p.setFont(QFont("Consolas", 9))
                w = max(p.fontMetrics().width(f"{n} {v:.4g}") for n, _, v in box) + 18
                h = len(box) * 15 + 20
                bx = self._cursor_x + 12
                if bx + w > left + pw:
                    bx = self._cursor_x - w - 12
                by = top + 6
                p.setBrush(QBrush(QColor(20, 20, 20, 225)))
                p.setPen(QColor("#555"))
                p.drawRoundedRect(bx, by, w, h, 4, 4)
                p.setPen(QColor("#AAA"))
                p.drawText(bx + 8, by + 14, f"t = {ct:.3f}s")
                for i, (n, c, v) in enumerate(box):
                    p.setPen(QColor(c))
                    p.drawText(bx + 8, by + 30 + i * 15, f"{n} {v:.4g}")

        # 状态角标
        p.setFont(QFont("Arial", 9))
        if not self.follow:
            p.setPen(QColor("#FFB74D"))
            p.drawText(left + 6, top - 8, "⏸ 历史回溯中（双击/点『回到最新』返回实时）")
        elif self.paused:
            p.setPen(QColor("#FF7043"))
            p.drawText(left + 6, top - 8, "⏸ 已暂停采集")
        else:
            p.setPen(QColor("#7CD992"))
            p.drawText(left + 6, top - 8, "● 实时")


def _nearest(s: Series, t: float) -> Optional[float]:
    """二分找最接近 t 的采样值。"""
    n = len(s.t)
    if n == 0:
        return None
    lo, hi = 0, n - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if s.t[mid] < t:
            lo = mid + 1
        else:
            hi = mid
    cands = [i for i in (lo - 1, lo) if 0 <= i < n]
    best = min(cands, key=lambda i: abs(s.t[i] - t))
    return s.v[best]
