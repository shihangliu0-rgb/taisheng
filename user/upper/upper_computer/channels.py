"""通道选择控件：勾选要显示/订阅的数据通道。

勾选结果 -> STREAM_CFG(0x31) 下发给下位机，下位机之后只上报这几个通道。
"""

from __future__ import annotations

from typing import Callable, Dict, List, Optional

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtWidgets import (
    QHBoxLayout, QLabel, QLineEdit, QPushButton, QTreeWidget, QTreeWidgetItem,
    QVBoxLayout, QWidget,
)

from . import protocol as P
from .plot import PALETTE


class ChannelSelector(QWidget):
    """按分组显示所有可订阅通道，支持勾选、搜索、快捷预设。"""

    selectionChanged = pyqtSignal(list)   # list[int] 通道 ID

    def __init__(self, parent=None):
        super().__init__(parent)
        self._items: Dict[int, QTreeWidgetItem] = {}
        self._emitting = True     # 构建期间不触发信号
        self._build()
        self._emitting = False
        self._on_item_changed(None, 0)

    def _build(self):
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)

        top = QHBoxLayout()
        self.ed_search = QLineEdit()
        self.ed_search.setPlaceholderText("搜索通道，如 yaw / gyro / odom …")
        self.ed_search.textChanged.connect(self._filter)
        self.ed_search.setFocusPolicy(Qt.ClickFocus)
        top.addWidget(self.ed_search)
        lay.addLayout(top)

        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["通道", "单位", "ID"])
        self.tree.setColumnWidth(0, 210)
        self.tree.setColumnWidth(1, 60)
        self.tree.setRootIsDecorated(True)
        self.tree.setFocusPolicy(Qt.ClickFocus)
        self.tree.itemChanged.connect(self._on_item_changed)
        lay.addWidget(self.tree, 1)

        for group, chans in P.channel_groups().items():
            gi = QTreeWidgetItem(self.tree, [group])
            gi.setFlags(gi.flags() | Qt.ItemIsUserCheckable | Qt.ItemIsAutoTristate)
            gi.setCheckState(0, Qt.Unchecked)
            gi.setForeground(0, QColor("#9AA0A6"))
            f = QFont()
            f.setBold(True)
            gi.setFont(0, f)
            for c in chans:
                it = QTreeWidgetItem(gi, [f"{c.name}  ({c.desc})", c.unit, f"0x{c.id:02X}"])
                it.setFlags(it.flags() | Qt.ItemIsUserCheckable)
                it.setCheckState(0, Qt.Unchecked)
                it.setData(0, Qt.UserRole, c.id)
                self._items[c.id] = it
            gi.setExpanded(group.startswith("IMU 姿态") or group == "里程计")

        presets = QHBoxLayout()
        for label, ids in (
            ("姿态角", [0x01, 0x02, 0x03]),
            ("陀螺仪", [0x10, 0x11, 0x12]),
            ("加速度", [0x20, 0x21, 0x22]),
            ("位移", [0x40, 0x41, 0x43]),
            ("实际速度", [0x44, 0x45, 0x46]),
            ("轮速", [0x50, 0x51, 0x52, 0x53]),
            ("DT35", [0x80, 0x81, 0x82, 0x83]),
        ):
            b = QPushButton(label)
            b.setFocusPolicy(Qt.NoFocus)
            b.clicked.connect(lambda _, x=ids: self.set_selected(x))
            presets.addWidget(b)
        b_clear = QPushButton("清空")
        b_clear.setFocusPolicy(Qt.NoFocus)
        b_clear.clicked.connect(lambda: self.set_selected([]))
        presets.addWidget(b_clear)
        presets.addStretch(1)
        self.lbl_count = QLabel("已选 0 个通道")
        self.lbl_count.setStyleSheet("color:#9AA0A6")
        presets.addWidget(self.lbl_count)
        lay.addLayout(presets)

    # ------------------------------------------------------------------
    def _filter(self, text: str):
        text = text.strip().lower()
        for i in range(self.tree.topLevelItemCount()):
            gi = self.tree.topLevelItem(i)
            any_vis = False
            for j in range(gi.childCount()):
                it = gi.child(j)
                vis = (not text) or text in it.text(0).lower() or text in it.text(2).lower()
                it.setHidden(not vis)
                any_vis = any_vis or vis
            gi.setHidden(not any_vis)
            if text and any_vis:
                gi.setExpanded(True)

    def _on_item_changed(self, item, col):
        if self._emitting or col != 0:
            return
        sel = self.selected()
        self.lbl_count.setText(f"已选 {len(sel)} 个通道")
        # 给已选通道标注颜色，和曲线颜色对应
        for idx, cid in enumerate(sel):
            it = self._items.get(cid)
            if it:
                it.setForeground(0, QColor(PALETTE[idx % len(PALETTE)]))
        for cid, it in self._items.items():
            if cid not in sel:
                it.setForeground(0, QColor("#E0E0E0"))
        self.selectionChanged.emit(sel)

    # ------------------------------------------------------------------
    def selected(self) -> List[int]:
        """返回已勾选通道 ID，按通道 ID 升序（与下发顺序一致）。"""
        return sorted(cid for cid, it in self._items.items()
                      if it.checkState(0) == Qt.Checked)

    def set_selected(self, ids: List[int]):
        self._emitting = True
        try:
            s = set(ids)
            for cid, it in self._items.items():
                it.setCheckState(0, Qt.Checked if cid in s else Qt.Unchecked)
        finally:
            self._emitting = False
        self._on_item_changed(None, 0)
