from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import dataclass
from datetime import datetime

from action_control import ActionControlPanel
from app_config import (
    ANGULAR_SPEED_STEP_RAD_S,
    COMMON_BAUDRATES,
    CONFIG_FILE,
    DEFAULT_ANGULAR_SPEED_RAD_S,
    DEFAULT_BAUDRATE,
    DEFAULT_LINEAR_SPEED_M_S,
    LINEAR_SPEED_STEP_M_S,
    MAX_ANGULAR_SPEED_RAD_S,
    MAX_LINEAR_SPEED_M_S,
    MAX_LOG_LINES,
    MIN_ANGULAR_SPEED_RAD_S,
    MIN_LINEAR_SPEED_M_S,
    SEND_PERIOD_MS,
    STOP_REPEAT_COUNT,
    ZERO_FRAME,
)
from export_dialog import ExportCDialog as SharedExportCDialog
from field_map import FieldMapView as SharedFieldMapView
from serial_worker import SerialWorker as SharedSerialWorker
from robot_protocol import (
    DT35_ADDR_F,
    DT35_ADDR_L,
    PNP_ADDR_F,
    PNP_ADDR_B,
    Dt35FrameParser,
    PnpFrameParser,
    VelocityCommand,
    YawFrameParser,
    build_action_frame,
    build_velocity_frame,
)

from PyQt6.QtCore import (
    QEvent,
    QObject,
    QPointF,
    Qt,
    QThread,
    QTimer,
    pyqtSignal,
    pyqtSlot,
)
from PyQt6.QtGui import (
    QCloseEvent,
    QColor,
    QFont,
    QPainter,
    QPen,
    QPixmap,
    QPolygonF,
    QTextCursor,
)
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QStackedWidget,
    QStyle,
    QVBoxLayout,
    QWidget,
)

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


COMMON_BAUDRATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
DEFAULT_BAUDRATE = 115200
SEND_PERIOD_MS = 100
STOP_REPEAT_COUNT = 3
MIN_LINEAR_SPEED_M_S = 0.05
MAX_LINEAR_SPEED_M_S = 5.0
DEFAULT_LINEAR_SPEED_M_S = 0.05
LINEAR_SPEED_STEP_M_S = 0.05
MIN_ANGULAR_SPEED_RAD_S = 0.01
MAX_ANGULAR_SPEED_RAD_S = 10.0
DEFAULT_ANGULAR_SPEED_RAD_S = 0.01
ANGULAR_SPEED_STEP_RAD_S = 0.01

MAX_LOG_LINES = 500

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_config.json")


@dataclass(frozen=True)
class MotionState:
    forward: bool = False
    backward: bool = False
    left: bool = False
    right: bool = False
    counterclockwise: bool = False
    clockwise: bool = False


def calculate_velocity(
    state: MotionState,
    linear_speed_m_s: float,
    angular_speed_rad_s: float,
) -> VelocityCommand:
    vx_axis = int(state.right) - int(state.left)
    vy_axis = int(state.forward) - int(state.backward)
    wz_axis = int(state.clockwise) - int(state.counterclockwise)
    linear_speed_mm_s = linear_speed_m_s * 1000.0

    if vx_axis != 0 and vy_axis != 0:
        linear_speed_mm_s /= math.sqrt(2.0)

    return VelocityCommand(
        vx_mm_s=round(vx_axis * linear_speed_mm_s),
        vy_mm_s=round(vy_axis * linear_speed_mm_s),
        wz_mrad_s=round(wz_axis * angular_speed_rad_s * 1000.0),
    )


ZERO_FRAME = build_velocity_frame(VelocityCommand())


# ─────────────────────────────────────────────
# B 样条曲线工具函数
# ─────────────────────────────────────────────

def bspline_basis(i: int, k: int, t: float, knots: list[float]) -> float:
    if k == 0:
        return 1.0 if knots[i] <= t < knots[i + 1] else 0.0

    left = 0.0
    right = 0.0

    denom_left = knots[i + k] - knots[i]
    if denom_left != 0:
        left = (t - knots[i]) / denom_left * bspline_basis(i, k - 1, t, knots)

    denom_right = knots[i + k + 1] - knots[i + 1]
    if denom_right != 0:
        right = (knots[i + k + 1] - t) / denom_right * bspline_basis(i + 1, k - 1, t, knots)

    return left + right


def generate_bspline_curve(
    control_points: list[QPointF],
    degree: int = 3,
    samples: int = 200,
) -> list[QPointF]:
    n = len(control_points)
    if n < 2:
        return list(control_points)

    actual_degree = min(degree, n - 1)
    m = n + actual_degree + 1
    knots = [0.0] * m
    for i in range(actual_degree + 1):
        knots[i] = 0.0
        knots[m - 1 - i] = 1.0
    inner_count = m - 2 * (actual_degree + 1)
    if inner_count > 0:
        for i in range(inner_count):
            knots[actual_degree + 1 + i] = (i + 1) / (inner_count + 1)

    curve_points = []
    for s in range(samples + 1):
        t = s / samples
        if t >= 1.0:
            t = 1.0 - 1e-9

        px = 0.0
        py = 0.0
        for i in range(n):
            basis = bspline_basis(i, actual_degree, t, knots)
            px += control_points[i].x() * basis
            py += control_points[i].y() * basis

        curve_points.append(QPointF(px, py))

    return curve_points


class SerialWorker(QObject):
    status_changed = pyqtSignal(bool, str)
    error_occurred = pyqtSignal(str)
    rx_bytes = pyqtSignal(bytes)
    tx_bytes = pyqtSignal(bytes)

    def __init__(self) -> None:
        super().__init__()
        self._serial = None
        self._poll_timer: QTimer | None = None

    @pyqtSlot()
    def start(self) -> None:
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(10)
        self._poll_timer.timeout.connect(self._poll_rx)
        self._poll_timer.start()

    @pyqtSlot(str, int)
    def open_port(self, port: str, baudrate: int) -> None:
        self._close_port()

        if serial is None:
            self.error_occurred.emit("缺少 pyserial，请先安装项目依赖。")
            self.status_changed.emit(False, "未连接")
            return

        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
                write_timeout=0.2,
            )
        except Exception as exc:
            self._serial = None
            self.error_occurred.emit(f"打开串口失败：{exc}")
            self.status_changed.emit(False, "未连接")
            return

        self.status_changed.emit(True, f"{port} @ {baudrate}")

    @pyqtSlot(bytes)
    def send_bytes(self, payload: bytes) -> None:
        if not payload or self._serial is None or not self._serial.is_open:
            return

        try:
            self._serial.write(payload)
            self.tx_bytes.emit(payload)
        except Exception as exc:
            self.error_occurred.emit(f"串口发送失败：{exc}")
            self._close_port()
            self.status_changed.emit(False, "连接已断开")

    @pyqtSlot(bytes)
    def disconnect_port(self, stop_frame: bytes) -> None:
        self._send_stop_frames(stop_frame)
        self._close_port()
        self.status_changed.emit(False, "未连接")

    @pyqtSlot(bytes)
    def shutdown(self, stop_frame: bytes) -> None:
        if self._poll_timer is not None:
            self._poll_timer.stop()
        self._send_stop_frames(stop_frame)
        self._close_port()

    def _send_stop_frames(self, stop_frame: bytes) -> None:
        if not stop_frame or self._serial is None or not self._serial.is_open:
            return

        try:
            for _ in range(STOP_REPEAT_COUNT):
                self._serial.write(stop_frame)
            self._serial.flush()
        except Exception:
            pass

    def _close_port(self) -> None:
        if self._serial is None:
            return

        try:
            if self._serial.is_open:
                self._serial.close()
        finally:
            self._serial = None

    def _poll_rx(self) -> None:
        if self._serial is None or not self._serial.is_open:
            return

        try:
            waiting = self._serial.in_waiting
            if waiting > 0:
                payload = self._serial.read(waiting)
                if payload:
                    self.rx_bytes.emit(payload)
        except Exception as exc:
            self.error_occurred.emit(f"串口接收失败：{exc}")
            self._close_port()
            self.status_changed.emit(False, "连接已断开")


class TabButton(QFrame):
    clicked = pyqtSignal()

    def __init__(self, text: str, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("TabButton")
        self.setProperty("active", False)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFixedHeight(36)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(16, 0, 18, 0)
        layout.setSpacing(8)

        self._dot = QFrame()
        self._dot.setObjectName("TabDot")
        self._dot.setFixedSize(6, 6)
        layout.addWidget(self._dot)

        self._label = QLabel(text)
        self._label.setObjectName("TabButtonText")
        layout.addWidget(self._label)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit()
        super().mousePressEvent(event)

    def set_active(self, active: bool) -> None:
        if self.property("active") == active:
            return
        self.setProperty("active", active)
        self.style().unpolish(self)
        self.style().polish(self)
        self._dot.style().unpolish(self._dot)
        self._dot.style().polish(self._dot)
        self._label.style().unpolish(self._label)
        self._label.style().polish(self._label)

    def text(self) -> str:
        return self._label.text()


class FieldMapView(QFrame):
    """场地地图视图 - 支持原点标记、B 样条轨迹"""

    coordinate_changed = pyqtSignal(float, float)
    origin_set = pyqtSignal(float, float)
    trajectory_changed = pyqtSignal(int)

    MODE_VIEW = 0
    MODE_SET_ORIGIN = 1
    MODE_ADD_POINT = 2

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("FieldMapView")

        self._label = QLabel(self)
        self._label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._label.setMouseTracking(True)
        self._label.installEventFilter(self)
        # =========修复关键点=========
        self._label.setScaledContents(False)
        self._label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)   # 大幅缩小内部边距！原来0,0,0,0也可以，2防止贴边框
        layout.setSpacing(0)
        layout.addWidget(self._label)

        self._original_pixmap: QPixmap | None = None
        self._display_pixmap: QPixmap | None = None

        self._field_width_m = 3.0
        self._field_height_m = 2.0
        self._origin_px = QPointF(0, 0)

        self._mode = self.MODE_VIEW

        # B 样条轨迹控制点（像素坐标）
        self._control_points: list[QPointF] = []
        self._curve_points: list[QPointF] = []
        self._bspline_degree = 3

        self.setMouseTracking(True)
        self._label.setCursor(Qt.CursorShape.CrossCursor)

    def set_field_image(self, image_path: str) -> bool:
        pixmap = QPixmap(image_path)
        if pixmap.isNull():
            return False

        self._original_pixmap = pixmap
        if self._origin_px == QPointF(0, 0):
            self._origin_px = QPointF(0, pixmap.height())
        self._update_display()
        return True

    def set_field_size(self, width_m: float, height_m: float) -> None:
        self._field_width_m = max(width_m, 0.1)
        self._field_height_m = max(height_m, 0.1)

    def start_set_origin(self) -> None:
        self._mode = self.MODE_SET_ORIGIN
        self._label.setCursor(Qt.CursorShape.CrossCursor)

    def start_add_point(self) -> None:
        self._mode = self.MODE_ADD_POINT
        self._label.setCursor(Qt.CursorShape.CrossCursor)

    def set_view_mode(self) -> None:
        self._mode = self.MODE_VIEW
        self._label.setCursor(Qt.CursorShape.ArrowCursor)

    def is_setting_origin(self) -> bool:
        return self._mode == self.MODE_SET_ORIGIN

    def is_adding_point(self) -> bool:
        return self._mode == self.MODE_ADD_POINT

    def set_origin_px(self, pos: QPointF) -> None:
        self._origin_px = pos
        self._update_display()

    def get_origin_m(self) -> tuple[float, float]:
        return self._pixel_to_meter(self._origin_px)

    def get_control_points_m(self) -> list[tuple[float, float]]:
        """获取控制点的实际坐标（米）"""
        return [self._pixel_to_meter(p) for p in self._control_points]

    def set_control_points_px(self, points: list[QPointF]) -> None:
        self._control_points = list(points)
        self._update_curve()
        self._update_display()

    def clear_trajectory(self) -> None:
        self._control_points.clear()
        self._curve_points.clear()
        self.trajectory_changed.emit(0)
        self._update_display()

    def _pixel_to_meter(self, px_pos: QPointF) -> tuple[float, float]:
        if self._original_pixmap is None:
            return (0.0, 0.0)

        img_w = self._original_pixmap.width()
        img_h = self._original_pixmap.height()
        if img_w == 0 or img_h == 0:
            return (0.0, 0.0)

        scale_x = self._field_width_m / img_w
        scale_y = self._field_height_m / img_h
        scale = min(scale_x, scale_y)

        dx = (px_pos.x() - self._origin_px.x()) * scale
        dy = (self._origin_px.y() - px_pos.y()) * scale

        return (dx, dy)

    def _update_curve(self) -> None:
        if len(self._control_points) >= 2:
            self._curve_points = generate_bspline_curve(
                self._control_points,
                degree=self._bspline_degree,
                samples=300,
            )
        else:
            self._curve_points = []

    def _update_display(self) -> None:
        if self._original_pixmap is None:
            self._label.setPixmap(QPixmap())
            return

        pixmap = QPixmap(self._original_pixmap)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)

        # 绘制 B 样条曲线
        if len(self._curve_points) >= 2:
            pen = QPen(QColor("#007aff"))
            pen.setWidth(3)
            painter.setPen(pen)
            poly = QPolygonF(self._curve_points)
            painter.drawPolyline(poly)

        # 绘制控制点连线（虚线）
        if len(self._control_points) >= 2:
            pen = QPen(QColor("#007aff"))
            pen.setWidth(1)
            pen.setStyle(Qt.PenStyle.DashLine)
            painter.setPen(pen)
            poly = QPolygonF(self._control_points)
            painter.drawPolyline(poly)

        # 绘制控制点
        for i, pt in enumerate(self._control_points):
            # 外圈白边
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor("#ffffff"))
            painter.drawEllipse(pt, 8, 8)
            # 内圈蓝色
            painter.setBrush(QColor("#007aff"))
            painter.drawEllipse(pt, 6, 6)
            # 序号
            painter.setPen(QColor("#ffffff"))
            font = painter.font()
            font.setBold(True)
            font.setPointSize(8)
            painter.setFont(font)
            painter.drawText(
                int(pt.x() - 4),
                int(pt.y() + 3),
                str(i + 1),
            )

        # 绘制原点标记
        pen = QPen(QColor("#ff3b30"))
        pen.setWidth(2)
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        ox = self._origin_px.x()
        oy = self._origin_px.y()
        size = 12

        painter.drawLine(int(ox - size), int(oy), int(ox + size), int(oy))
        painter.drawLine(int(ox), int(oy - size), int(ox), int(oy + size))
        painter.drawEllipse(QPointF(ox, oy), 6, 6)

        painter.setPen(QColor("#ff3b30"))
        font = painter.font()
        font.setBold(True)
        font.setPointSize(10)
        painter.setFont(font)
        painter.drawText(int(ox + 10), int(oy - 8), "O (0,0)")

        painter.end()

        self._display_pixmap = pixmap
        self._label.setPixmap(pixmap)
        self._fit_image()

    def _fit_image(self) -> None:
        if self._display_pixmap is None:
            return
        scaled = self._display_pixmap.scaled(
            self._label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self._label.setPixmap(scaled)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._fit_image()

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        if watched != self._label:
            return super().eventFilter(watched, event)

        if event.type() == QEvent.Type.MouseMove:
            self._handle_mouse_move(event.pos())
            return False
        elif event.type() == QEvent.Type.MouseButtonPress:
            if event.button() == Qt.MouseButton.LeftButton:
                self._handle_mouse_press(event.pos())
            return False

        return super().eventFilter(watched, event)

    def _label_to_image_pos(self, label_pos: QPointF) -> QPointF:
        if self._display_pixmap is None:
            return QPointF(0, 0)

        label_w = self._label.width()
        label_h = self._label.height()
        img_w = self._display_pixmap.width()
        img_h = self._display_pixmap.height()

        scale_w = label_w / img_w
        scale_h = label_h / img_h
        scale = min(scale_w, scale_h)

        actual_w = img_w * scale
        actual_h = img_h * scale

        offset_x = (label_w - actual_w) / 2
        offset_y = (label_h - actual_h) / 2

        img_x = (label_pos.x() - offset_x) / scale
        img_y = (label_pos.y() - offset_y) / scale

        return QPointF(img_x, img_y)

    def _handle_mouse_move(self, pos) -> None:
        if self._original_pixmap is None:
            return

        img_pos = self._label_to_image_pos(pos)
        x_m, y_m = self._pixel_to_meter(img_pos)
        self.coordinate_changed.emit(x_m, y_m)

    def _handle_mouse_press(self, pos) -> None:
        if self._original_pixmap is None:
            return

        img_pos = self._label_to_image_pos(pos)

        rect = self._original_pixmap.rect().toRectF()
        if not rect.contains(img_pos):
            return

        if self._mode == self.MODE_SET_ORIGIN:
            self._origin_px = img_pos
            self._mode = self.MODE_VIEW
            self._label.setCursor(Qt.CursorShape.ArrowCursor)
            x_m, y_m = self._pixel_to_meter(img_pos)
            self.origin_set.emit(x_m, y_m)
            self._update_display()

        elif self._mode == self.MODE_ADD_POINT:
            self._control_points.append(img_pos)
            self._update_curve()
            self.trajectory_changed.emit(len(self._control_points))
            self._update_display()


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

        # 标题
        title = QLabel("B 样条轨迹控制点")
        title.setStyleSheet(
            "font-size: 16px; font-weight: 600; color: #1d1d1f;"
        )
        layout.addWidget(title)

        # 信息行
        info = QLabel(f"共 {len(points)} 个控制点，单位：米")
        info.setStyleSheet("color: #86868b; font-size: 12px;")
        layout.addWidget(info)

        # 代码文本框
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

        # 生成 C 代码
        code = self._generate_c_array(points)
        self.text_edit.setPlainText(code)
        layout.addWidget(self.text_edit, 1)

        # 按钮
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
            QPushButton:hover {
                background: #0a84ff;
            }
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
            QPushButton:hover {
                background: #e8e8ed;
            }
            """
        )
        close_btn.clicked.connect(self.accept)
        btn_layout.addWidget(close_btn)

        layout.addLayout(btn_layout)

    def _generate_c_array(self, points: list[tuple[float, float]]) -> str:
        lines = []
        lines.append("/* ============================================================")
        lines.append(" * B 样条轨迹控制点")
        lines.append(f" * 点数: {len(points)}")
        lines.append(" * 单位: 米 (m)")
        lines.append(" * 坐标系: X 向右为正, Y 向上为正")
        lines.append(" * 生成时间: " + datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        lines.append(" * ============================================================ */")
        lines.append("")
        lines.append("#ifndef TRAJECTORY_POINTS_H")
        lines.append("#define TRAJECTORY_POINTS_H")
        lines.append("")
        lines.append("#include <stdint.h>")
        lines.append("")
        lines.append(f"/* 控制点数量 */")
        lines.append(f"#define TRAJECTORY_POINT_COUNT  {len(points)}")
        lines.append("")
        lines.append("/* B 样条轨迹控制点数组 [x, y] */")
        lines.append("static const float trajectory_points[TRAJECTORY_POINT_COUNT][2] = {")

        for i, (x, y) in enumerate(points):
            comma = "," if i < len(points) - 1 else ""
            lines.append(f"    {{ {x:+.4f}f, {y:+.4f}f }}{comma}  /* P{i + 1} */")

        lines.append("};")
        lines.append("")
        lines.append("/* B 样条次数 (3 = 三次 B 样条) */")
        lines.append("#define BSPLINE_DEGREE  3")
        lines.append("")
        lines.append("#endif /* TRAJECTORY_POINTS_H */")

        return "\n".join(lines)

    def _copy_to_clipboard(self) -> None:
        clipboard = QApplication.clipboard()
        clipboard.setText(self.text_edit.toPlainText())

        # 短暂提示
        btn = self.sender()
        if btn:
            original_text = btn.text()
            btn.setText("✓ 已复制")
            QTimer.singleShot(1500, lambda: btn.setText(original_text))


SerialWorker = SharedSerialWorker
FieldMapView = SharedFieldMapView
ExportCDialog = SharedExportCDialog


class MainWindow(QMainWindow):
    connect_requested = pyqtSignal(str, int)
    disconnect_requested = pyqtSignal(bytes)
    send_requested = pyqtSignal(bytes)
    shutdown_requested = pyqtSignal(bytes)

    KEY_FORWARD = Qt.Key.Key_W.value
    KEY_BACKWARD = Qt.Key.Key_S.value
    KEY_LEFT = Qt.Key.Key_A.value
    KEY_RIGHT = Qt.Key.Key_D.value
    KEY_CCW = Qt.Key.Key_Q.value
    KEY_CW = Qt.Key.Key_E.value
    MOTION_KEYS = {
        KEY_FORWARD,
        KEY_BACKWARD,
        KEY_LEFT,
        KEY_RIGHT,
        KEY_CCW,
        KEY_CW,
    }

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("机器人运动控制")
        self.setMinimumSize(900, 680)
        self.resize(1020, 720)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self.setFont(QFont(".AppleSystemUIFont", 13))

        self._connected = False
        self._current_baudrate = DEFAULT_BAUDRATE
        self._pressed_keys: set[int] = set()
        self._command = VelocityCommand()
        self._rx_count = 0
        self._tx_count = 0
        self._auto_scroll = True
        self._show_hex = True
        self._yaw_parser = YawFrameParser()
        self._dt35_parser = Dt35FrameParser()
        self._pnp_parser = PnpFrameParser()

        self._filter_tx = True
        self._filter_rx = True
        self._log_buffer: list[tuple[str, str, bytes, str]] = []

        # 定位页配置
        self._field_image_path = ""
        self._field_width_m = 3.0
        self._field_height_m = 2.0
        self._origin_px_x = 0.0
        self._origin_px_y = 0.0
        self._trajectory_points: list[dict] = []  # [{x, y}] 像素坐标

        self._load_config()

        self._serial_thread = QThread(self)
        self._serial_worker = SerialWorker()
        self._serial_worker.moveToThread(self._serial_thread)
        self._serial_thread.started.connect(self._serial_worker.start)
        self.connect_requested.connect(self._serial_worker.open_port)
        self.disconnect_requested.connect(self._serial_worker.disconnect_port)
        self.send_requested.connect(self._serial_worker.send_bytes)
        self.shutdown_requested.connect(
            self._serial_worker.shutdown,
            type=Qt.ConnectionType.BlockingQueuedConnection,
        )
        self._serial_worker.status_changed.connect(self._on_serial_status)
        self._serial_worker.error_occurred.connect(self._on_serial_error)
        self._serial_worker.rx_bytes.connect(self._on_rx_bytes)
        self._serial_worker.tx_bytes.connect(self._on_tx_bytes)
        self._serial_thread.start()

        self._build_ui()
        self._apply_style()

        self._port_timer = QTimer(self)
        self._port_timer.setInterval(1500)
        self._port_timer.timeout.connect(self.scan_ports)
        self._port_timer.start()

        self._send_timer = QTimer(self)
        self._send_timer.setInterval(SEND_PERIOD_MS)
        self._send_timer.timeout.connect(self._send_active_command)
        self._send_timer.start()

        QApplication.instance().installEventFilter(self)
        self.scan_ports()
        self._update_command(send_now=False)

        # 加载保存的场地照片和轨迹
        if self._field_image_path and os.path.exists(self._field_image_path):
            self.field_map.set_field_image(self._field_image_path)
            self.field_map.set_field_size(self._field_width_m, self._field_height_m)
            self.field_map.set_origin_px(QPointF(self._origin_px_x, self._origin_px_y))

            if self._trajectory_points:
                points = [QPointF(p["x"], p["y"]) for p in self._trajectory_points]
                self.field_map.set_control_points_px(points)
                self.traj_count_label.setText(f"{len(points)} 个点")

    def _build_ui(self) -> None:
        central = QWidget(self)
        central.setObjectName("Central")
        root = QVBoxLayout(central)
        root.setContentsMargins(28, 20, 28, 24)
        root.setSpacing(16)
        self.setCentralWidget(central)

        # ── Header ──
        header = QHBoxLayout()
        header.setSpacing(14)
        title_box = QVBoxLayout()
        title_box.setSpacing(2)
        title = QLabel("机器人运动控制")
        title.setObjectName("Title")
        serial_spec = QLabel("UART4   ·   8N1   ·   10 Hz")
        serial_spec.setObjectName("Subtle")
        title_box.addWidget(title)
        title_box.addWidget(serial_spec)
        header.addLayout(title_box)
        header.addStretch(1)

        status_pill = QFrame()
        status_pill.setObjectName("StatusPill")
        status_pill.setProperty("connected", False)
        pill_layout = QHBoxLayout(status_pill)
        pill_layout.setContentsMargins(12, 6, 14, 6)
        pill_layout.setSpacing(8)

        self.status_dot = QFrame()
        self.status_dot.setObjectName("StatusDot")
        self.status_dot.setProperty("connected", False)
        self.status_dot.setFixedSize(8, 8)
        self.status_label = QLabel("未连接")
        self.status_label.setObjectName("StatusText")
        pill_layout.addWidget(self.status_dot)
        pill_layout.addWidget(self.status_label)

        header.addWidget(status_pill)
        root.addLayout(header)

        # ── 顶部标签栏 ──
        tab_bar = QFrame()
        tab_bar.setObjectName("TabBar")
        tab_layout = QHBoxLayout(tab_bar)
        tab_layout.setContentsMargins(3, 3, 3, 3)
        tab_layout.setSpacing(0)

        self._tab_buttons = []
        tab_names = ["调试", "串口", "定位"]
        for i, name in enumerate(tab_names):
            btn = TabButton(name)
            btn.clicked.connect(lambda idx=i: self._switch_tab(idx))
            tab_layout.addWidget(btn)
            self._tab_buttons.append(btn)

        root.addWidget(tab_bar)

        # ── 页面堆栈 ──
        self._stack = QStackedWidget()
        root.addWidget(self._stack, 1)

        self._build_debug_page()
        self._build_serial_page()
        self._build_location_page()

        self._switch_tab(0)

    def _build_debug_page(self) -> None:
        page = QWidget()
        page_layout = QHBoxLayout(page)
        page_layout.setContentsMargins(0, 0, 0, 0)
        page_layout.setSpacing(18)

        # 左侧：控制参数
        settings_group = QFrame()
        settings_group.setObjectName("CardPanel")
        settings_group.setMinimumWidth(280)
        settings_group.setMaximumWidth(340)
        settings_layout = QVBoxLayout(settings_group)
        settings_layout.setContentsMargins(20, 18, 20, 20)
        settings_layout.setSpacing(16)

        card_title = QLabel("控制参数")
        card_title.setObjectName("CardTitle")
        settings_layout.addWidget(card_title)

        linear_row = QVBoxLayout()
        linear_row.setSpacing(6)
        linear_label = QLabel("平移速度")
        linear_label.setObjectName("FieldLabel")
        self.linear_speed_spin = QDoubleSpinBox()
        self.linear_speed_spin.setRange(MIN_LINEAR_SPEED_M_S, MAX_LINEAR_SPEED_M_S)
        self.linear_speed_spin.setDecimals(3)
        self.linear_speed_spin.setSingleStep(LINEAR_SPEED_STEP_M_S)
        self.linear_speed_spin.setValue(DEFAULT_LINEAR_SPEED_M_S)
        self.linear_speed_spin.setSuffix(" m/s")
        self.linear_speed_spin.valueChanged.connect(self._speed_changed)
        self.linear_speed_spin.editingFinished.connect(self.setFocus)
        linear_row.addWidget(linear_label)
        linear_row.addWidget(self.linear_speed_spin)
        settings_layout.addLayout(linear_row)

        angular_row = QVBoxLayout()
        angular_row.setSpacing(6)
        angular_label = QLabel("旋转速度")
        angular_label.setObjectName("FieldLabel")
        self.angular_speed_spin = QDoubleSpinBox()
        self.angular_speed_spin.setRange(MIN_ANGULAR_SPEED_RAD_S, MAX_ANGULAR_SPEED_RAD_S)
        self.angular_speed_spin.setDecimals(3)
        self.angular_speed_spin.setSingleStep(ANGULAR_SPEED_STEP_RAD_S)
        self.angular_speed_spin.setValue(DEFAULT_ANGULAR_SPEED_RAD_S)
        self.angular_speed_spin.setSuffix(" rad/s")
        self.angular_speed_spin.valueChanged.connect(self._speed_changed)
        self.angular_speed_spin.editingFinished.connect(self.setFocus)
        angular_row.addWidget(angular_label)
        angular_row.addWidget(self.angular_speed_spin)
        settings_layout.addLayout(angular_row)

        self.keyboard_check = QCheckBox("键盘控制")
        self.keyboard_check.setObjectName("SwitchCheck")
        self.keyboard_check.setChecked(True)
        self.keyboard_check.toggled.connect(self._keyboard_toggled)
        settings_layout.addWidget(self.keyboard_check)

        settings_layout.addWidget(self._divider())

        conn_section = QVBoxLayout()
        conn_section.setSpacing(8)
        conn_label = QLabel("连接状态")
        conn_label.setObjectName("FieldLabel")
        conn_section.addWidget(conn_label)

        conn_status_row = QHBoxLayout()
        conn_status_row.setSpacing(10)
        self.debug_status_dot = QFrame()
        self.debug_status_dot.setObjectName("DebugStatusDot")
        self.debug_status_dot.setProperty("connected", False)
        self.debug_status_dot.setFixedSize(10, 10)
        self.debug_status_text = QLabel("未连接")
        self.debug_status_text.setObjectName("DebugStatusText")
        conn_status_row.addWidget(self.debug_status_dot)
        conn_status_row.addWidget(self.debug_status_text)
        conn_status_row.addStretch(1)
        conn_section.addLayout(conn_status_row)

        self.debug_connect_btn = QPushButton("连接")
        self.debug_connect_btn.setObjectName("PrimaryButton")
        self.debug_connect_btn.setMinimumHeight(42)
        self.debug_connect_btn.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_DialogApplyButton)
        )
        self.debug_connect_btn.clicked.connect(self._toggle_connection)
        conn_section.addWidget(self.debug_connect_btn)

        settings_layout.addLayout(conn_section)

        settings_layout.addWidget(self._divider())

        key_label = QLabel("键盘控制")
        key_label.setObjectName("FieldLabel")
        settings_layout.addWidget(key_label)

        key_grid = QGridLayout()
        key_grid.setHorizontalSpacing(12)
        key_grid.setVerticalSpacing(8)
        key_rows = [
            ("W / S", "前进 / 后退"),
            ("A / D", "左移 / 右移"),
            ("Q / E", "逆时针 / 顺时针"),
        ]
        for row_index, (key, desc) in enumerate(key_rows):
            key_badge = QLabel(key)
            key_badge.setObjectName("KeyBadge")
            key_badge.setMinimumWidth(70)
            key_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
            desc_label = QLabel(desc)
            desc_label.setObjectName("Subtle")
            key_grid.addWidget(key_badge, row_index, 0)
            key_grid.addWidget(desc_label, row_index, 1)
        key_grid.setColumnStretch(1, 1)
        settings_layout.addLayout(key_grid)
        settings_layout.addStretch(1)

        self.stop_button = QPushButton("停止")
        self.stop_button.setObjectName("StopButton")
        self.stop_button.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_MediaStop)
        )
        self.stop_button.setMinimumHeight(46)
        self.stop_button.clicked.connect(self._stop_motion)
        settings_layout.addWidget(self.stop_button)

        page_layout.addWidget(settings_group)

        # 右侧：遥测数据 + 机构动作
        right_panel = QVBoxLayout()
        right_panel.setSpacing(16)

        cmd_card = QFrame()
        cmd_card.setObjectName("CardPanel")
        cmd_layout = QVBoxLayout(cmd_card)
        cmd_layout.setContentsMargins(20, 18, 20, 20)
        cmd_layout.setSpacing(14)

        cmd_title = QLabel("遥测数据")
        cmd_title.setObjectName("CardTitle")
        cmd_layout.addWidget(cmd_title)

        self.vx_label = QLabel()
        self.vy_label = QLabel()
        self.wz_label = QLabel()
        self.yaw_label = QLabel("Yaw  --.-- deg")
        self.dt35_41_label = QLabel("DT35_L  -- cm")
        self.dt35_40_label = QLabel("DT35_F  -- cm")
        self.pnp_f_label = QLabel("PNP_F  0")
        self.pnp_b_label = QLabel("PNP_B  0")
        telemetry_layout = QGridLayout()
        telemetry_layout.setHorizontalSpacing(18)
        telemetry_layout.setVerticalSpacing(8)
        for index, label in enumerate((
            self.vx_label,
            self.vy_label,
            self.wz_label,
            self.yaw_label,
            self.dt35_41_label,
            self.dt35_40_label,
            self.pnp_f_label,
            self.pnp_b_label,
        )):
            label.setObjectName("CommandValueLarge")
            label.setMinimumHeight(36)
            label.setMinimumWidth(150)
            telemetry_layout.addWidget(label, index // 3, index % 3)

        cmd_layout.addLayout(telemetry_layout)

        right_panel.addWidget(cmd_card)

        self.action_control = ActionControlPanel()
        self.action_control.action_requested.connect(self._send_action)
        right_panel.addWidget(self.action_control)
        right_panel.addStretch(1)

        page_layout.addLayout(right_panel, 1)
        self._stack.addWidget(page)

    def _build_serial_page(self) -> None:
        page = QWidget()
        page_layout = QVBoxLayout(page)
        page_layout.setContentsMargins(0, 0, 0, 0)
        page_layout.setSpacing(16)

        config_row = QHBoxLayout()
        config_row.setSpacing(12)

        port_label = QLabel("串口")
        port_label.setObjectName("SectionLabel")
        port_label.setMinimumWidth(30)
        config_row.addWidget(port_label)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(200)
        self.port_combo.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Fixed,
        )
        config_row.addWidget(self.port_combo, 2)

        baud_label = QLabel("波特率")
        baud_label.setObjectName("SectionLabel")
        baud_label.setMinimumWidth(40)
        config_row.addWidget(baud_label)
        self.baud_combo = QComboBox()
        self.baud_combo.setObjectName("BaudCombo")
        self.baud_combo.setMinimumWidth(120)
        self.baud_combo.setMaximumWidth(160)
        for baud in COMMON_BAUDRATES:
            self.baud_combo.addItem(f"{baud:,}", baud)
        default_idx = self.baud_combo.findData(DEFAULT_BAUDRATE)
        if default_idx >= 0:
            self.baud_combo.setCurrentIndex(default_idx)
        self.baud_combo.currentIndexChanged.connect(self._on_baudrate_changed)
        config_row.addWidget(self.baud_combo)

        config_row.addStretch(1)

        self.refresh_button = QPushButton("刷新")
        self.refresh_button.setObjectName("GhostButton")
        self.refresh_button.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_BrowserReload)
        )
        self.refresh_button.clicked.connect(self.scan_ports)
        config_row.addWidget(self.refresh_button)

        self.connect_button = QPushButton("连接")
        self.connect_button.setObjectName("PrimaryButton")
        self.connect_button.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_DialogApplyButton)
        )
        self.connect_button.clicked.connect(self._toggle_connection)
        config_row.addWidget(self.connect_button)

        page_layout.addLayout(config_row)

        log_group = QFrame()
        log_group.setObjectName("CardPanel")
        log_layout = QVBoxLayout(log_group)
        log_layout.setContentsMargins(20, 18, 20, 20)
        log_layout.setSpacing(12)

        log_header = QHBoxLayout()
        log_header.setSpacing(8)

        log_title = QLabel("串口日志")
        log_title.setObjectName("CardTitle")
        log_header.addWidget(log_title)
        log_header.addStretch(1)

        self.tx_count_label = QLabel("TX 0")
        self.rx_count_label = QLabel("RX 0")
        self.tx_count_label.setObjectName("MetricBadge")
        self.rx_count_label.setObjectName("MetricBadge")
        log_header.addWidget(self.tx_count_label)
        log_header.addWidget(self.rx_count_label)

        self.format_btn = QPushButton("HEX")
        self.format_btn.setObjectName("FormatChip")
        self.format_btn.setCheckable(True)
        self.format_btn.setChecked(True)
        self.format_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.format_btn.clicked.connect(self._on_format_toggled)
        log_header.addWidget(self.format_btn)

        self.filter_tx_btn = QPushButton("TX")
        self.filter_tx_btn.setObjectName("FilterChipTX")
        self.filter_tx_btn.setCheckable(True)
        self.filter_tx_btn.setChecked(True)
        self.filter_tx_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.filter_tx_btn.clicked.connect(self._on_filter_tx_toggled)
        log_header.addWidget(self.filter_tx_btn)

        self.filter_rx_btn = QPushButton("RX")
        self.filter_rx_btn.setObjectName("FilterChipRX")
        self.filter_rx_btn.setCheckable(True)
        self.filter_rx_btn.setChecked(True)
        self.filter_rx_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.filter_rx_btn.clicked.connect(self._on_filter_rx_toggled)
        log_header.addWidget(self.filter_rx_btn)

        self.auto_scroll_check = QCheckBox("自动滚动")
        self.auto_scroll_check.setObjectName("SwitchCheck")
        self.auto_scroll_check.setChecked(True)
        self.auto_scroll_check.toggled.connect(self._on_auto_scroll_toggled)
        log_header.addWidget(self.auto_scroll_check)

        self.clear_log_button = QPushButton("清空")
        self.clear_log_button.setObjectName("GhostButton")
        self.clear_log_button.clicked.connect(self._clear_log)
        log_header.addWidget(self.clear_log_button)

        log_layout.addLayout(log_header)

        self.log_text = QPlainTextEdit()
        self.log_text.setObjectName("LogConsole")
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumBlockCount(MAX_LOG_LINES)
        self.log_text.setFont(QFont("Menlo", 11))
        self.log_text.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        log_layout.addWidget(self.log_text, 1)

        page_layout.addWidget(log_group, 1)
        self._stack.addWidget(page)

    def _build_location_page(self) -> None:
        page = QWidget()
        page_layout = QVBoxLayout(page)
        page_layout.setContentsMargins(0, 0, 0, 0)
        page_layout.setSpacing(8)   # 原来16，减小间距

        # 工具栏第一行：场地尺寸 + 上传 + 原点
        toolbar1 = QFrame()
        toolbar1.setObjectName("CardPanel")
        tool1_layout = QHBoxLayout(toolbar1)
        tool1_layout.setContentsMargins(12,8,12,8)  # 缩小上下边距，原来16,12,16,12
        tool1_layout.setSpacing(12)

        size_label = QLabel("场地尺寸")
        size_label.setObjectName("FieldLabel")
        tool1_layout.addWidget(size_label)

        self.field_width_spin = QDoubleSpinBox()
        self.field_width_spin.setRange(0.1, 100.0)
        self.field_width_spin.setDecimals(2)
        self.field_width_spin.setSingleStep(0.1)
        self.field_width_spin.setValue(self._field_width_m)
        self.field_width_spin.setSuffix(" m 宽")
        self.field_width_spin.valueChanged.connect(self._on_field_size_changed)
        tool1_layout.addWidget(self.field_width_spin)

        self.field_height_spin = QDoubleSpinBox()
        self.field_height_spin.setRange(0.1, 100.0)
        self.field_height_spin.setDecimals(2)
        self.field_height_spin.setSingleStep(0.1)
        self.field_height_spin.setValue(self._field_height_m)
        self.field_height_spin.setSuffix(" m 高")
        self.field_height_spin.valueChanged.connect(self._on_field_size_changed)
        tool1_layout.addWidget(self.field_height_spin)

        tool1_layout.addStretch(1)

        self.upload_btn = QPushButton("上传场地照片")
        self.upload_btn.setObjectName("GhostButton")
        self.upload_btn.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_DirOpenIcon)
        )
        self.upload_btn.clicked.connect(self._upload_field_image)
        tool1_layout.addWidget(self.upload_btn)

        self.origin_btn = QPushButton("标记原点")
        self.origin_btn.setObjectName("PrimaryButton")
        self.origin_btn.clicked.connect(self._toggle_set_origin)
        tool1_layout.addWidget(self.origin_btn)

        page_layout.addWidget(toolbar1)

        # 工具栏第二行：B 样条轨迹
        toolbar2 = QFrame()
        toolbar2.setObjectName("CardPanel")
        tool2_layout = QHBoxLayout(toolbar2)
        tool2_layout.setContentsMargins(12,8,12,8) # 缩小上下边距
        tool2_layout.setSpacing(12)

        traj_label = QLabel("B 样条轨迹")
        traj_label.setObjectName("FieldLabel")
        tool2_layout.addWidget(traj_label)

        self.traj_count_label = QLabel("0 个点")
        self.traj_count_label.setObjectName("MetricBadge")
        tool2_layout.addWidget(self.traj_count_label)

        tool2_layout.addStretch(1)

        self.export_btn = QPushButton("导出 C 数组")
        self.export_btn.setObjectName("GhostButton")
        self.export_btn.setStyleSheet(
            "color: #34c759; border-color: #b8e6c5;"
        )
        self.export_btn.clicked.connect(self._export_c_array)
        tool2_layout.addWidget(self.export_btn)

        self.add_point_btn = QPushButton("添加轨迹点")
        self.add_point_btn.setObjectName("PrimaryButton")
        self.add_point_btn.clicked.connect(self._toggle_add_point)
        tool2_layout.addWidget(self.add_point_btn)

        self.clear_traj_btn = QPushButton("清除轨迹")
        self.clear_traj_btn.setObjectName("GhostButton")
        self.clear_traj_btn.clicked.connect(self._clear_trajectory)
        tool2_layout.addWidget(self.clear_traj_btn)

        page_layout.addWidget(toolbar2)

        # 场地地图 【关键：stretch=10，地图优先抢占剩余窗口高度】
        map_container = QFrame()
        map_container.setObjectName("CardPanel")
        map_layout = QVBoxLayout(map_container)
        map_layout.setContentsMargins(4,4,4,4) # 大幅缩小容器内边距，防止遮挡图片
        map_layout.setSpacing(0)

        self.field_map = FieldMapView()
        self.field_map.setMinimumHeight(300) # 降低最小高度，原来420，窗口小也不会卡死
        self.field_map.coordinate_changed.connect(self._on_mouse_coordinate)
        self.field_map.origin_set.connect(self._on_origin_set)
        self.field_map.trajectory_changed.connect(self._on_trajectory_changed)
        map_layout.addWidget(self.field_map, stretch=1)

        page_layout.addWidget(map_container, stretch=10) # 最重要！地图最大权重

        # 底部坐标栏
        coord_bar = QFrame()
        coord_bar.setObjectName("CommandBar")
        coord_layout = QHBoxLayout(coord_bar)
        coord_layout.setContentsMargins(16,8,16,8) # 缩小上下padding
        coord_layout.setSpacing(24)

        coord_title = QLabel("鼠标坐标")
        coord_title.setObjectName("SectionLabel")
        coord_layout.addWidget(coord_title)

        self.mouse_x_label = QLabel("X: 0.000 m")
        self.mouse_y_label = QLabel("Y: 0.000 m")
        for label in (self.mouse_x_label, self.mouse_y_label):
            label.setObjectName("CoordValue")
            label.setMinimumWidth(130)
            coord_layout.addWidget(label)

        coord_layout.addStretch(1)

        origin_title = QLabel("原点")
        origin_title.setObjectName("SectionLabel")
        coord_layout.addWidget(origin_title)

        self.origin_x_label = QLabel("X: 0.000 m")
        self.origin_y_label = QLabel("Y: 0.000 m")
        for label in (self.origin_x_label, self.origin_y_label):
            label.setObjectName("OriginValue")
            label.setMinimumWidth(130)
            coord_layout.addWidget(label)

        page_layout.addWidget(coord_bar)

        self._stack.addWidget(page)

    @staticmethod
    def _divider() -> QFrame:
        divider = QFrame()
        divider.setFrameShape(QFrame.Shape.HLine)
        divider.setObjectName("Divider")
        return divider

    def _switch_tab(self, index: int) -> None:
        for i, btn in enumerate(self._tab_buttons):
            btn.set_active(i == index)
        self._stack.setCurrentIndex(index)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            /* ========== 基础重置 ========== */
            QWidget {
                color: #1d1d1f;
                font-family: -apple-system, BlinkMacSystemFont, "SF Pro Text",
                             "Helvetica Neue", "PingFang SC", "Microsoft YaHei UI",
                             sans-serif;
                font-size: 13px;
                selection-background-color: #007aff;
                selection-color: #ffffff;
            }

            QWidget#Central {
                background: #f5f5f7;
            }

            QLabel#Title {
                font-size: 26px;
                font-weight: 700;
                color: #1d1d1f;
                letter-spacing: -0.5px;
            }
            QLabel#Subtle {
                color: #86868b;
                font-size: 12px;
            }
            QLabel#CardTitle {
                font-size: 15px;
                font-weight: 600;
                color: #1d1d1f;
                letter-spacing: -0.2px;
            }
            QLabel#FieldLabel {
                font-size: 12px;
                font-weight: 500;
                color: #86868b;
                padding-left: 2px;
            }
            QLabel#SectionLabel {
                font-size: 12px;
                font-weight: 600;
                color: #86868b;
                text-transform: uppercase;
                letter-spacing: 0.5px;
            }
            QLabel#DebugStatusText {
                font-size: 14px;
                font-weight: 600;
                color: #86868b;
            }
            QLabel#KeyBadge {
                background: #f2f2f7;
                color: #1d1d1f;
                border-radius: 6px;
                padding: 6px 10px;
                font-size: 12px;
                font-weight: 600;
                font-family: "SF Mono", Menlo, Consolas, monospace;
            }

            /* ========== 顶部标签栏 ========== */
            QFrame#TabBar {
                background: #e8e8ed;
                border-radius: 9px;
                padding: 0;
            }
            QFrame#TabButton {
                background: transparent;
                border-radius: 6px;
                margin: 0;
            }
            QFrame#TabButton:hover {
                background: rgba(0, 0, 0, 0.05);
            }
            QFrame#TabButton[active="true"] {
                background: #ffffff;
            }
            QFrame#TabDot {
                background: #aeaeb2;
                border-radius: 3px;
            }
            QFrame#TabButton[active="true"] QFrame#TabDot {
                background: #007aff;
            }
            QFrame#TabButton:hover QFrame#TabDot {
                background: #86868b;
            }
            QLabel#TabButtonText {
                color: #86868b;
                font-size: 13px;
                font-weight: 500;
            }
            QFrame#TabButton[active="true"] QLabel#TabButtonText {
                color: #1d1d1f;
                font-weight: 600;
            }
            QFrame#TabButton:hover QLabel#TabButtonText {
                color: #1d1d1f;
            }

            /* ========== 状态胶囊 ========== */
            QFrame#StatusPill {
                background: #ffffff;
                border: 1px solid #e5e5ea;
                border-radius: 16px;
                padding: 0px;
            }
            QFrame#StatusPill[connected="true"] {
                background: #e8f8ed;
                border: 1px solid #b8e6c5;
            }
            QLabel#StatusText {
                font-weight: 600;
                font-size: 12px;
                color: #86868b;
            }
            QFrame#StatusDot {
                background: #aeaeb2;
                border-radius: 4px;
            }
            QFrame#StatusDot[connected="true"] {
                background: #34c759;
            }
            QFrame#DebugStatusDot {
                background: #aeaeb2;
                border-radius: 5px;
            }
            QFrame#DebugStatusDot[connected="true"] {
                background: #34c759;
            }

            /* ========== 卡片面板 ========== */
            QFrame#CardPanel, QFrame#CommandBar {
                background: #ffffff;
                border: 1px solid #e5e5ea;
                border-radius: 12px;
            }

            /* ========== 下拉框 & 数字输入 ========== */
            QComboBox, QDoubleSpinBox, QSpinBox {
                min-height: 36px;
                padding: 0 12px;
                background: #f2f2f7;
                border: 1px solid #e5e5ea;
                border-radius: 8px;
                color: #1d1d1f;
                font-size: 13px;
            }
            QComboBox:hover, QDoubleSpinBox:hover, QSpinBox:hover {
                background: #e8e8ed;
            }
            QComboBox:focus, QDoubleSpinBox:focus, QSpinBox:focus {
                border: 2px solid #007aff;
                background: #ffffff;
                padding: 0 11px;
            }
            QComboBox::drop-down {
                border: none;
                width: 24px;
            }
            QComboBox::down-arrow {
                image: none;
                border: none;
                width: 0;
                height: 0;
            }
            QComboBox QAbstractItemView {
                background: #ffffff;
                border: 1px solid #e5e5ea;
                border-radius: 8px;
                padding: 4px;
                selection-background-color: #f2f2f7;
                selection-color: #1d1d1f;
                outline: none;
            }
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button,
            QSpinBox::up-button, QSpinBox::down-button {
                width: 20px;
                background: transparent;
                border: none;
            }
            QDoubleSpinBox::up-arrow, QDoubleSpinBox::down-arrow,
            QSpinBox::up-arrow, QSpinBox::down-arrow {
                width: 0;
                height: 0;
            }

            QComboBox#BaudCombo {
                font-weight: 600;
            }

            /* ========== 按钮 ========== */
            QPushButton {
                min-height: 36px;
                padding: 0 16px;
                background: #f2f2f7;
                border: 1px solid #e5e5ea;
                border-radius: 8px;
                font-weight: 500;
                font-size: 13px;
                color: #1d1d1f;
            }
            QPushButton:hover {
                background: #e8e8ed;
            }
            QPushButton:pressed {
                background: #dddde3;
            }
            QPushButton:disabled {
                color: #aeaeb2;
                background: #f2f2f7;
                border-color: #e5e5ea;
            }

            QPushButton#GhostButton {
                background: transparent;
                border: 1px solid #d1d1d6;
                color: #007aff;
                min-height: 30px;
                padding: 0 12px;
                font-size: 12px;
            }
            QPushButton#GhostButton:hover {
                background: #f0f7ff;
                border-color: #007aff;
            }
            QPushButton#GhostButton:pressed {
                background: #e0efff;
            }

            QPushButton#PrimaryButton {
                color: #ffffff;
                background: #007aff;
                border: 1px solid #007aff;
                font-weight: 600;
                font-size: 14px;
            }
            QPushButton#PrimaryButton:hover {
                background: #0a84ff;
                border-color: #0a84ff;
            }
            QPushButton#PrimaryButton:pressed {
                background: #0066cc;
                border-color: #0066cc;
            }
            QPushButton#PrimaryButton:disabled {
                background: #a8c8f0;
                border-color: #a8c8f0;
                color: #ffffff;
            }

            QPushButton#StopButton {
                color: #ffffff;
                background: #ff3b30;
                border: 1px solid #ff3b30;
                font-weight: 600;
                font-size: 14px;
                border-radius: 10px;
            }
            QPushButton#StopButton:hover {
                background: #ff453a;
                border-color: #ff453a;
            }
            QPushButton#StopButton:pressed {
                background: #d70015;
                border-color: #d70015;
            }

            /* ========== 格式切换芯片按钮 ========== */
            QPushButton#FormatChip {
                min-height: 30px;
                min-width: 56px;
                padding: 0 14px;
                background: #e5e5ea;
                border: 1px solid #d1d1d6;
                border-radius: 15px;
                font-size: 12px;
                font-weight: 700;
                color: #86868b;
            }
            QPushButton#FormatChip:checked {
                background: #1d1d1f;
                border: 1px solid #1d1d1f;
                color: #ffffff;
            }
            QPushButton#FormatChip:checked:hover {
                background: #2c2c2e;
                border-color: #2c2c2e;
            }
            QPushButton#FormatChip:hover:!checked {
                background: #d1d1d6;
                color: #1d1d1f;
            }

            /* ========== TX/RX 过滤芯片按钮 ========== */
            QPushButton#FilterChipTX {
                min-height: 30px;
                min-width: 52px;
                padding: 0 16px;
                background: #e5e5ea;
                border: 1px solid #d1d1d6;
                border-radius: 15px;
                font-size: 12px;
                font-weight: 700;
                color: #86868b;
            }
            QPushButton#FilterChipTX:checked {
                background: #007aff;
                border: 1px solid #007aff;
                color: #ffffff;
            }
            QPushButton#FilterChipTX:checked:hover {
                background: #0a84ff;
                border-color: #0a84ff;
            }
            QPushButton#FilterChipTX:hover:!checked {
                background: #d1d1d6;
                color: #1d1d1f;
            }

            QPushButton#FilterChipRX {
                min-height: 30px;
                min-width: 52px;
                padding: 0 16px;
                background: #e5e5ea;
                border: 1px solid #d1d1d6;
                border-radius: 15px;
                font-size: 12px;
                font-weight: 700;
                color: #86868b;
            }
            QPushButton#FilterChipRX:checked {
                background: #34c759;
                border: 1px solid #34c759;
                color: #ffffff;
            }
            QPushButton#FilterChipRX:checked:hover {
                background: #30d158;
                border-color: #30d158;
            }
            QPushButton#FilterChipRX:hover:!checked {
                background: #d1d1d6;
                color: #1d1d1f;
            }

            /* ========== 日志控制台 ========== */
            QPlainTextEdit#LogConsole {
                background: #1d1d1f;
                color: #f5f5f7;
                border: 1px solid #e5e5ea;
                border-radius: 8px;
                padding: 12px;
                font-family: "SF Mono", Menlo, Consolas, monospace;
                font-size: 12px;
                selection-background-color: #007aff;
            }

            /* ========== 指令数值 ========== */
            QLabel#CommandValueLarge {
                color: #007aff;
                font-family: "SF Mono", "Menlo", "Consolas", monospace;
                font-size: 18px;
                font-weight: 700;
                letter-spacing: 0.3px;
                padding: 8px 0;
            }

            /* ========== 坐标数值 ========== */
            QLabel#CoordValue {
                color: #007aff;
                font-family: "SF Mono", "Menlo", "Consolas", monospace;
                font-size: 14px;
                font-weight: 600;
            }
            QLabel#OriginValue {
                color: #ff3b30;
                font-family: "SF Mono", "Menlo", "Consolas", monospace;
                font-size: 14px;
                font-weight: 600;
            }

            /* ========== 指标徽章 ========== */
            QLabel#MetricBadge {
                background: #f2f2f7;
                color: #86868b;
                border-radius: 6px;
                padding: 5px 10px;
                font-size: 12px;
                font-weight: 500;
            }

            /* ========== 开关风格 CheckBox ========== */
            QCheckBox#SwitchCheck {
                spacing: 10px;
                font-size: 12px;
                font-weight: 500;
                color: #86868b;
                padding: 4px 0;
            }
            QCheckBox#SwitchCheck::indicator {
                width: 38px;
                height: 22px;
                border-radius: 11px;
                background: #e5e5ea;
                border: none;
            }
            QCheckBox#SwitchCheck::indicator:checked {
                background: #34c759;
            }
            QCheckBox#SwitchCheck::indicator:unchecked:hover {
                background: #d1d1d6;
            }

            /* ========== 场地地图 ========== */
            QFrame#FieldMapView {
                background: #1d1d1f;
                border-radius: 8px;
            }

            /* ========== 分隔线 ========== */
            QFrame#Divider {
                background: #e5e5ea;
                max-height: 1px;
                border: none;
            }

            /* ========== 滚动条 ========== */
            QScrollBar:vertical {
                background: transparent;
                width: 8px;
                margin: 0;
            }
            QScrollBar::handle:vertical {
                background: #c7c7cc;
                border-radius: 4px;
                min-height: 30px;
            }
            QScrollBar::handle:vertical:hover {
                background: #aeaeb2;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0;
            }
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
                background: none;
            }
            """
        )

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        del watched

        if event.type() == QEvent.Type.ApplicationDeactivate:
            self._stop_motion()
            return False
        if event.type() not in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease):
            return False

        key = event.key()
        if key not in self.MOTION_KEYS:
            return False
        if event.isAutoRepeat():
            return True
        if not self.keyboard_check.isChecked():
            return False

        if event.type() == QEvent.Type.KeyPress:
            self._pressed_keys.add(key)
        else:
            self._pressed_keys.discard(key)

        self._update_command(send_now=True)
        return True

    def scan_ports(self) -> None:
        if self._connected:
            return

        previous = self.port_combo.currentData()
        if list_ports is None:
            ports = []
        else:
            ports = sorted(list_ports.comports(), key=lambda item: item.device)
        self.port_combo.clear()

        if not ports:
            self.port_combo.addItem("未发现串口", None)
            self.connect_button.setEnabled(False)
            self.debug_connect_btn.setEnabled(False)
            return

        for port in ports:
            description = port.description or "Serial Port"
            self.port_combo.addItem(
                f"{port.device}  ·  {description}",
                port.device,
            )

        if previous is not None:
            index = self.port_combo.findData(previous)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)
        self.connect_button.setEnabled(True)
        self.debug_connect_btn.setEnabled(True)

    def _on_baudrate_changed(self, index: int) -> None:
        baudrate = self.baud_combo.itemData(index)
        if baudrate is None:
            return
        self._current_baudrate = baudrate
        if not self._connected:
            self._append_log("SYS", f"波特率已设置为 {baudrate:,} bps", "#86868b")

    def _toggle_connection(self) -> None:
        if self._connected:
            self._clear_motion(send_now=False)
            self.disconnect_requested.emit(ZERO_FRAME)
            self._append_log("SYS", "串口已断开", "#ff9500")
            return

        port = self.port_combo.currentData()
        if port is None:
            self._set_status_message("没有可连接的串口", error=True)
            return

        self.connect_button.setEnabled(False)
        self.debug_connect_btn.setEnabled(False)
        self._set_status_message("正在连接…")
        self._append_log("SYS", f"正在连接 {port} @ {self._current_baudrate:,}…", "#ff9500")
        self.connect_requested.emit(port, self._current_baudrate)

    def _on_serial_status(self, connected: bool, detail: str) -> None:
        self._connected = connected
        self.action_control.set_connected(connected)
        self.status_label.setStyleSheet("")
        self.status_dot.setProperty("connected", connected)
        self.status_dot.style().unpolish(self.status_dot)
        self.status_dot.style().polish(self.status_dot)

        status_pill = self.status_dot.parent()
        if status_pill:
            status_pill.setProperty("connected", connected)
            status_pill.style().unpolish(status_pill)
            status_pill.style().polish(status_pill)

        self.status_label.setText(detail)
        self.port_combo.setEnabled(not connected)
        self.baud_combo.setEnabled(not connected)
        self.refresh_button.setEnabled(not connected)

        self.connect_button.setEnabled(connected or self.port_combo.currentData() is not None)
        self.connect_button.setText("断开" if connected else "连接")
        self.debug_connect_btn.setEnabled(connected or self.port_combo.currentData() is not None)
        self.debug_connect_btn.setText("断开" if connected else "连接")

        if connected:
            icon = QStyle.StandardPixmap.SP_DialogCloseButton
            self._append_log("SYS", f"串口已连接：{detail}", "#34c759")
            self.debug_status_text.setText("已连接")
            self.debug_status_text.setStyleSheet("color: #34c759;")
        else:
            icon = QStyle.StandardPixmap.SP_DialogApplyButton
            self.debug_status_text.setText("未连接")
            self.debug_status_text.setStyleSheet("color: #86868b;")
            self._yaw_parser.reset()
            self.yaw_label.setText("Yaw  --.-- deg")
            self._dt35_parser.reset()
            self.dt35_41_label.setText("DT35_L  -- cm")
            self.dt35_40_label.setText("DT35_F  -- cm")
            self._pnp_parser.reset()
            self.pnp_f_label.setText("PNP_F  0")
            self.pnp_b_label.setText("PNP_B  0")

        self.connect_button.setIcon(self.style().standardIcon(icon))
        self.debug_connect_btn.setIcon(self.style().standardIcon(icon))

        self.debug_status_dot.setProperty("connected", connected)
        self.debug_status_dot.style().unpolish(self.debug_status_dot)
        self.debug_status_dot.style().polish(self.debug_status_dot)

        self._clear_motion(send_now=connected)

    def _on_serial_error(self, message: str) -> None:
        self._set_status_message(message, error=True)
        self._append_log("ERR", message, "#ff3b30")
        if not self._connected:
            self.connect_button.setEnabled(self.port_combo.currentData() is not None)
            self.debug_connect_btn.setEnabled(self.port_combo.currentData() is not None)

    def _set_status_message(self, message: str, error: bool = False) -> None:
        self.status_label.setText(message)
        if error:
            self.status_label.setStyleSheet("color: #ff3b30;")
        else:
            self.status_label.setStyleSheet("")

    def _speed_changed(self) -> None:
        self._update_command(send_now=True)

    def _keyboard_toggled(self, enabled: bool) -> None:
        if not enabled:
            self._clear_motion(send_now=True)
        self.setFocus()

    def _update_command(self, send_now: bool) -> None:
        state = MotionState(
            forward=self.KEY_FORWARD in self._pressed_keys,
            backward=self.KEY_BACKWARD in self._pressed_keys,
            left=self.KEY_LEFT in self._pressed_keys,
            right=self.KEY_RIGHT in self._pressed_keys,
            counterclockwise=self.KEY_CCW in self._pressed_keys,
            clockwise=self.KEY_CW in self._pressed_keys,
        )
        self._command = calculate_velocity(
            state,
            self.linear_speed_spin.value(),
            self.angular_speed_spin.value(),
        )

        self.vx_label.setText(f"Vx  {self._command.vx_mm_s / 1000.0:+.3f} m/s")
        self.vy_label.setText(f"Vy  {self._command.vy_mm_s / 1000.0:+.3f} m/s")
        self.wz_label.setText(f"Wz  {self._command.wz_mrad_s / 1000.0:+.3f} rad/s")

        if send_now:
            self._send_current_command()

    def _send_current_command(self) -> None:
        if not self._connected:
            return
        self.send_requested.emit(build_velocity_frame(self._command))

    def _send_action(self, action: int, name: str) -> None:
        if not self._connected:
            return

        self.send_requested.emit(build_action_frame(action))
        self._append_log("SYS", f"已发送{name}指令", "#86868b")

    def _send_active_command(self) -> None:
        if self._command == VelocityCommand():
            return
        self._send_current_command()

    def _clear_motion(self, send_now: bool) -> None:
        self._pressed_keys.clear()
        self._update_command(send_now=send_now)

    def _stop_motion(self) -> None:
        self._clear_motion(send_now=True)
        self.setFocus()

    # ── 日志 ──

    def _on_format_toggled(self, checked: bool) -> None:
        self._show_hex = checked
        self.format_btn.setText("HEX" if checked else "ASCII")
        self._refresh_log_view()

    def _on_filter_tx_toggled(self, checked: bool) -> None:
        self._filter_tx = checked
        self._refresh_log_view()

    def _on_filter_rx_toggled(self, checked: bool) -> None:
        self._filter_rx = checked
        self._refresh_log_view()

    def _should_show_tag(self, tag: str) -> bool:
        if tag == "TX":
            return self._filter_tx
        if tag == "RX":
            return self._filter_rx
        return True

    def _refresh_log_view(self) -> None:
        self.log_text.clear()
        for timestamp, tag, data, color in self._log_buffer:
            if self._should_show_tag(tag):
                self._render_log_line(timestamp, tag, data, color)

        if self._auto_scroll:
            cursor = self.log_text.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.End)
            self.log_text.setTextCursor(cursor)

    def _format_data(self, data: bytes) -> str:
        if self._show_hex:
            return " ".join(f"{b:02X}" for b in data)
        else:
            result = []
            for b in data:
                if 32 <= b < 127:
                    result.append(chr(b))
                else:
                    result.append(".")
            return "".join(result)

    def _render_log_line(self, timestamp: str, tag: str, data: bytes, color: str) -> None:
        content = self._format_data(data)
        size = len(data)
        html = (
            f'<span style="color:#6b7280;">{timestamp}</span> '
            f'<span style="color:{color};font-weight:600;">[{tag}]</span> '
            f'<span style="color:#9aa0a6;">{size}B</span> '
            f'<span style="color:#f5f5f7;">{content}</span>'
        )
        self.log_text.appendHtml(html)

    def _append_log(self, tag: str, message: str, color: str = "#f5f5f7") -> None:
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        data = message.encode("utf-8", errors="replace")

        self._log_buffer.append((timestamp, tag, data, color))
        if len(self._log_buffer) > MAX_LOG_LINES:
            self._log_buffer = self._log_buffer[-MAX_LOG_LINES:]

        if self._should_show_tag(tag):
            html = (
                f'<span style="color:#6b7280;">{timestamp}</span> '
                f'<span style="color:{color};font-weight:600;">[{tag}]</span> '
                f'<span style="color:#f5f5f7;">{message}</span>'
            )
            self.log_text.appendHtml(html)
            if self._auto_scroll:
                cursor = self.log_text.textCursor()
                cursor.movePosition(QTextCursor.MoveOperation.End)
                self.log_text.setTextCursor(cursor)

    def _on_tx_bytes(self, payload: bytes) -> None:
        self._tx_count += 1
        self.tx_count_label.setText(f"TX {self._tx_count}")
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        color = "#007aff"

        self._log_buffer.append((timestamp, "TX", payload, color))
        if len(self._log_buffer) > MAX_LOG_LINES:
            self._log_buffer = self._log_buffer[-MAX_LOG_LINES:]

        if self._filter_tx:
            self._render_log_line(timestamp, "TX", payload, color)
            if self._auto_scroll:
                cursor = self.log_text.textCursor()
                cursor.movePosition(QTextCursor.MoveOperation.End)
                self.log_text.setTextCursor(cursor)

    def _on_rx_bytes(self, payload: bytes) -> None:
        yaw_values = self._yaw_parser.feed(payload)
        if yaw_values:
            self.yaw_label.setText(f"Yaw  {yaw_values[-1]:+.2f} deg")

        for address, distance_cm in self._dt35_parser.feed(payload):
            if address == DT35_ADDR_L:
                self.dt35_41_label.setText(f"DT35_L  {distance_cm} cm")
            elif address == DT35_ADDR_F:
                self.dt35_40_label.setText(f"DT35_F  {distance_cm} cm")

        for address, trigger in self._pnp_parser.feed(payload):
            if address == PNP_ADDR_F:
                self.pnp_f_label.setText(f"PNP_F  {trigger}")
            elif address == PNP_ADDR_B:
                self.pnp_b_label.setText(f"PNP_B  {trigger}")

        self._rx_count += len(payload)
        self.rx_count_label.setText(f"RX {self._rx_count}")
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        color = "#34c759"

        self._log_buffer.append((timestamp, "RX", payload, color))
        if len(self._log_buffer) > MAX_LOG_LINES:
            self._log_buffer = self._log_buffer[-MAX_LOG_LINES:]

        if self._filter_rx:
            self._render_log_line(timestamp, "RX", payload, color)
            if self._auto_scroll:
                cursor = self.log_text.textCursor()
                cursor.movePosition(QTextCursor.MoveOperation.End)
                self.log_text.setTextCursor(cursor)

    def _clear_log(self) -> None:
        self.log_text.clear()
        self._log_buffer.clear()
        self._tx_count = 0
        self._rx_count = 0
        self.tx_count_label.setText("TX 0")
        self.rx_count_label.setText("RX 0")

    def _on_auto_scroll_toggled(self, checked: bool) -> None:
        self._auto_scroll = checked
        if checked:
            cursor = self.log_text.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.End)
            self.log_text.setTextCursor(cursor)

    # ── 定位页 ──

    def _upload_field_image(self) -> None:
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "选择场地照片",
            "",
            "图片文件 (*.png *.jpg *.jpeg *.bmp *.gif *.webp)",
        )
        if not file_path:
            return

        if self.field_map.set_field_image(file_path):
            self._field_image_path = file_path
            self._save_config()
            self._append_log("SYS", f"场地照片已加载：{os.path.basename(file_path)}", "#86868b")

    def _on_field_size_changed(self) -> None:
        w = self.field_width_spin.value()
        h = self.field_height_spin.value()
        self._field_width_m = w
        self._field_height_m = h
        self.field_map.set_field_size(w, h)
        self._save_config()

    def _toggle_set_origin(self) -> None:
        if self.field_map.is_setting_origin():
            self.field_map.set_view_mode()
            self.origin_btn.setText("标记原点")
            self.origin_btn.setStyleSheet("")
        else:
            self.field_map.start_set_origin()
            self.origin_btn.setText("取消标记")
            self.origin_btn.setStyleSheet(
                "background: #ff9500; border-color: #ff9500; color: white;"
            )

    def _on_origin_set(self, x_m: float, y_m: float) -> None:
        self.origin_btn.setText("标记原点")
        self.origin_btn.setStyleSheet("")
        self.origin_x_label.setText(f"X: {x_m:+.3f} m")
        self.origin_y_label.setText(f"Y: {y_m:+.3f} m")

        origin_px = self.field_map._origin_px
        self._origin_px_x = origin_px.x()
        self._origin_px_y = origin_px.y()
        self._save_config()

    def _on_mouse_coordinate(self, x_m: float, y_m: float) -> None:
        self.mouse_x_label.setText(f"X: {x_m:+.3f} m")
        self.mouse_y_label.setText(f"Y: {y_m:+.3f} m")

    # ── B 样条轨迹 ──

    def _toggle_add_point(self) -> None:
        if self.field_map.is_adding_point():
            self.field_map.set_view_mode()
            self.add_point_btn.setText("添加轨迹点")
            self.add_point_btn.setStyleSheet("")
        else:
            self.field_map.start_add_point()
            self.add_point_btn.setText("完成添加")
            self.add_point_btn.setStyleSheet(
                "background: #ff9500; border-color: #ff9500; color: white;"
            )

    def _on_trajectory_changed(self, count: int) -> None:
        self.traj_count_label.setText(f"{count} 个点")
        # 自动保存控制点像素坐标
        self._trajectory_points = [
            {"x": p.x(), "y": p.y()}
            for p in self.field_map._control_points
        ]
        self._save_config()

    def _clear_trajectory(self) -> None:
        self.field_map.clear_trajectory()
        self._trajectory_points.clear()
        self._save_config()

    def _export_c_array(self) -> None:
        """导出 C 语言数组"""
        points_m = self.field_map.get_control_points_m()
        if not points_m:
            self._append_log("SYS", "没有轨迹点可导出", "#ff9500")
            return

        dialog = ExportCDialog(points_m, self)
        dialog.exec()

    # ── 配置持久化 ──

    def _load_config(self) -> None:
        try:
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                    config = json.load(f)
                self._field_image_path = config.get("field_image", "")
                self._field_width_m = config.get("field_width_m", 3.0)
                self._field_height_m = config.get("field_height_m", 2.0)
                self._origin_px_x = config.get("origin_px_x", 0.0)
                self._origin_px_y = config.get("origin_px_y", 0.0)
                self._trajectory_points = config.get("trajectory_points", [])
        except Exception:
            pass

    def _save_config(self) -> None:
        try:
            config = {
                "field_image": self._field_image_path,
                "field_width_m": self._field_width_m,
                "field_height_m": self._field_height_m,
                "origin_px_x": self._origin_px_x,
                "origin_px_y": self._origin_px_y,
                "trajectory_points": self._trajectory_points,
            }
            with open(CONFIG_FILE, "w", encoding="utf-8") as f:
                json.dump(config, f, indent=2, ensure_ascii=False)
        except Exception:
            pass

    def closeEvent(self, event: QCloseEvent) -> None:
        self._send_timer.stop()
        self._port_timer.stop()
        self._clear_motion(send_now=False)
        QApplication.instance().removeEventFilter(self)

        self._save_config()

        if self._serial_thread.isRunning():
            self.shutdown_requested.emit(ZERO_FRAME)
            self._serial_thread.quit()
            self._serial_thread.wait(1500)

        event.accept()
