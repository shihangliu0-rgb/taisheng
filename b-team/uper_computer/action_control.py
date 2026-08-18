from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QStyle,
    QVBoxLayout,
)

from robot_protocol import (
    ACTION_FRONT_DOWN,
    ACTION_FRONT_FLAT,
    ACTION_LIFT,
    ACTION_LOWER,
    ACTION_M2006_COAST,
    ACTION_M2006_FORWARD,
    ACTION_REAR_DOWN,
    ACTION_REAR_FLAT,
)


class ActionControlPanel(QFrame):
    action_requested = pyqtSignal(int, str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("CardPanel")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 16, 20, 18)
        layout.setSpacing(10)

        title = QLabel("机构动作")
        title.setObjectName("CardTitle")
        layout.addWidget(title)

        button_groups = (
            (
                "整机",
                (
                    ("抬升", ACTION_LIFT, "抬升"),
                    ("下沉", ACTION_LOWER, "下沉"),
                ),
            ),
            (
                "M2006",
                (
                    ("前进", ACTION_M2006_FORWARD, "2006前进"),
                    ("滑行", ACTION_M2006_COAST, "2006滑行"),
                ),
            ),
            (
                "前腿",
                (
                    ("放平", ACTION_FRONT_FLAT, "前腿放平"),
                    ("放下", ACTION_FRONT_DOWN, "前腿放下"),
                ),
            ),
            (
                "后腿",
                (
                    ("放平", ACTION_REAR_FLAT, "后腿放平"),
                    ("放下", ACTION_REAR_DOWN, "后腿放下"),
                ),
            ),
        )

        self.action_buttons = []
        for group_name, button_specs in button_groups:
            row_layout = QHBoxLayout()
            row_layout.setSpacing(8)

            group_label = QLabel(group_name)
            group_label.setObjectName("FieldLabel")
            group_label.setMinimumWidth(58)
            row_layout.addWidget(group_label)

            for text, action, action_name in button_specs:
                button = QPushButton(text)
                button.setObjectName(
                    "PrimaryButton" if not self.action_buttons else "GhostButton"
                )
                button.setMinimumHeight(38)
                button.setSizePolicy(
                    QSizePolicy.Policy.Expanding,
                    QSizePolicy.Policy.Fixed,
                )
                button.clicked.connect(
                    lambda checked=False, cmd=action, name=action_name: (
                        self.action_requested.emit(cmd, name)
                    )
                )
                row_layout.addWidget(button, 1)
                self.action_buttons.append(button)

            layout.addLayout(row_layout)

        self.action_buttons[0].setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_ArrowUp)
        )
        self.action_buttons[1].setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_ArrowDown)
        )
        self.action_buttons[2].setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_MediaSeekForward)
        )
        self.action_buttons[3].setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_MediaPause)
        )

        self.set_connected(False)

    def set_connected(self, connected: bool) -> None:
        for button in self.action_buttons:
            button.setEnabled(connected)
