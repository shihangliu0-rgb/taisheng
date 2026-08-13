from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QFrame,
    QGridLayout,
    QLabel,
    QPushButton,
    QStyle,
    QVBoxLayout,
)

from robot_protocol import (
    ACTION_FRONT_DOWN,
    ACTION_FRONT_FLAT,
    ACTION_FRONT_FOLD,
    ACTION_LIFT,
    ACTION_LOWER,
    ACTION_M2006_FORWARD,
    ACTION_REAR_DOWN,
    ACTION_REAR_FLAT,
    ACTION_REAR_FOLD,
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

        button_layout = QGridLayout()
        button_layout.setHorizontalSpacing(8)
        button_layout.setVerticalSpacing(8)

        button_specs = (
            ("抬升", ACTION_LIFT),
            ("下沉", ACTION_LOWER),
            ("2006前进", ACTION_M2006_FORWARD),
            ("前腿收平", ACTION_FRONT_FOLD),
            ("前腿放平", ACTION_FRONT_FLAT),
            ("前腿放下", ACTION_FRONT_DOWN),
            ("后退收平", ACTION_REAR_FOLD),
            ("后腿放平", ACTION_REAR_FLAT),
            ("后腿放下", ACTION_REAR_DOWN),
        )

        self.action_buttons = []
        for index, (text, action) in enumerate(button_specs):
            button = QPushButton(text)
            button.setObjectName("PrimaryButton" if index == 0 else "GhostButton")
            button.setMinimumHeight(36)
            button.clicked.connect(
                lambda checked=False, cmd=action, name=text: (
                    self.action_requested.emit(cmd, name)
                )
            )
            button_layout.addWidget(button, index // 3, index % 3)
            self.action_buttons.append(button)

        self.action_buttons[0].setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_ArrowUp)
        )
        self.action_buttons[1].setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_ArrowDown)
        )
        self.action_buttons[2].setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_MediaSeekForward)
        )

        layout.addLayout(button_layout)
        self.set_connected(False)

    def set_connected(self, connected: bool) -> None:
        for button in self.action_buttons:
            button.setEnabled(connected)
