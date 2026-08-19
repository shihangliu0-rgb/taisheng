from __future__ import annotations

from datetime import datetime

from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication,
    QDialog,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
)


class ExportCDialog(QDialog):
    """导出 C 数组对话框"""

    def __init__(self, points: list[tuple[float, float]], parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("导出 C 语言数组")
        self.setMinimumSize(560, 420)
        self.resize(620, 480)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 18, 20, 18)
        layout.setSpacing(12)

        title = QLabel("B 样条轨迹控制点")
        title.setStyleSheet("font-size: 16px; font-weight: 600; color: #1d1d1f;")
        layout.addWidget(title)

        info = QLabel(f"共 {len(points)} 个控制点，单位：米")
        info.setStyleSheet("color: #86868b; font-size: 12px;")
        layout.addWidget(info)

        self.text_edit = QPlainTextEdit()
        self.text_edit.setReadOnly(False)
        self.text_edit.setFont(QFont("Menlo", 11))
        self.text_edit.setStyleSheet(
            """
            QPlainTextEdit {
                background: #1d1d1f;
                color: #f5f5f7;
                border: 1px solid #e5e5ea;
                border-radius: 8px;
                padding: 12px;
                font-family: "SF Mono", Menlo, Consolas, monospace;
                selection-background-color: #007aff;
            }
            """
        )
        self.text_edit.setPlainText(self._generate_c_array(points))
        layout.addWidget(self.text_edit, 1)

        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(8)

        copy_btn = QPushButton("复制到剪贴板")
        copy_btn.setObjectName("PrimaryButton")
        copy_btn.setStyleSheet(
            """
            QPushButton {
                color: white;
                background: #007aff;
                border: 1px solid #007aff;
                border-radius: 8px;
                padding: 8px 20px;
                font-weight: 600;
                font-size: 13px;
            }
            QPushButton:hover { background: #0a84ff; }
            """
        )
        copy_btn.clicked.connect(self._copy_to_clipboard)
        btn_layout.addWidget(copy_btn)
        btn_layout.addStretch(1)

        close_btn = QPushButton("关闭")
        close_btn.setStyleSheet(
            """
            QPushButton {
                background: #f2f2f7;
                border: 1px solid #e5e5ea;
                border-radius: 8px;
                padding: 8px 20px;
                font-weight: 500;
                font-size: 13px;
                color: #1d1d1f;
            }
            QPushButton:hover { background: #e8e8ed; }
            """
        )
        close_btn.clicked.connect(self.accept)
        btn_layout.addWidget(close_btn)
        layout.addLayout(btn_layout)

    def _generate_c_array(self, points: list[tuple[float, float]]) -> str:
        lines = [
            "/* ============================================================",
            " * B 样条轨迹控制点",
            f" * 点数: {len(points)}",
            " * 单位: 米 (m)",
            " * 坐标系: X 向右为正, Y 向上为正",
            " * 生成时间: " + datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            " * ============================================================ */",
            "",
            "#ifndef TRAJECTORY_POINTS_H",
            "#define TRAJECTORY_POINTS_H",
            "",
            "#include <stdint.h>",
            "",
            f"/* 控制点数量 */",
            f"#define TRAJECTORY_POINT_COUNT  {len(points)}",
            "",
            "/* B 样条轨迹控制点数组 [x, y] */",
            "static const float trajectory_points[TRAJECTORY_POINT_COUNT][2] = {",
        ]
        for i, (x, y) in enumerate(points):
            comma = "," if i < len(points) - 1 else ""
            lines.append(f"    {{ {x:+.4f}f, {y:+.4f}f }}{comma}  /* P{i + 1} */")
        lines.extend([
            "};",
            "",
            "/* B 样条次数 (3 = 三次 B 样条) */",
            "#define BSPLINE_DEGREE  3",
            "",
            "#endif /* TRAJECTORY_POINTS_H */",
        ])
        return "\n".join(lines)

    def _copy_to_clipboard(self) -> None:
        QApplication.clipboard().setText(self.text_edit.toPlainText())
        btn = self.sender()
        if btn:
            original_text = btn.text()
            btn.setText("✓ 已复制")
            QTimer.singleShot(1500, lambda: btn.setText(original_text))
