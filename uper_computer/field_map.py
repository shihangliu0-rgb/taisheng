from __future__ import annotations

from PyQt6.QtCore import QEvent, QObject, QPointF, Qt, pyqtSignal
from PyQt6.QtGui import QColor, QPainter, QPen, QPixmap, QPolygonF
from PyQt6.QtWidgets import QFrame, QLabel, QSizePolicy, QVBoxLayout


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
        self._label.setScaledContents(False)
        self._label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(0)
        layout.addWidget(self._label)

        self._original_pixmap: QPixmap | None = None
        self._display_pixmap: QPixmap | None = None
        self._field_width_m = 3.0
        self._field_height_m = 2.0
        self._origin_px = QPointF(0, 0)
        self._mode = self.MODE_VIEW
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

        if len(self._curve_points) >= 2:
            pen = QPen(QColor("#007aff"))
            pen.setWidth(3)
            painter.setPen(pen)
            painter.drawPolyline(QPolygonF(self._curve_points))

        if len(self._control_points) >= 2:
            pen = QPen(QColor("#007aff"))
            pen.setWidth(1)
            pen.setStyle(Qt.PenStyle.DashLine)
            painter.setPen(pen)
            painter.drawPolyline(QPolygonF(self._control_points))

        for i, pt in enumerate(self._control_points):
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor("#ffffff"))
            painter.drawEllipse(pt, 8, 8)
            painter.setBrush(QColor("#007aff"))
            painter.drawEllipse(pt, 6, 6)
            painter.setPen(QColor("#ffffff"))
            font = painter.font()
            font.setBold(True)
            font.setPointSize(8)
            painter.setFont(font)
            painter.drawText(int(pt.x() - 4), int(pt.y() + 3), str(i + 1))

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
        if event.type() == QEvent.Type.MouseButtonPress:
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
        scale = min(label_w / img_w, label_h / img_h)
        actual_w = img_w * scale
        actual_h = img_h * scale
        offset_x = (label_w - actual_w) / 2
        offset_y = (label_h - actual_h) / 2
        return QPointF(
            (label_pos.x() - offset_x) / scale,
            (label_pos.y() - offset_y) / scale,
        )

    def _handle_mouse_move(self, pos) -> None:
        if self._original_pixmap is None:
            return
        x_m, y_m = self._pixel_to_meter(self._label_to_image_pos(pos))
        self.coordinate_changed.emit(x_m, y_m)

    def _handle_mouse_press(self, pos) -> None:
        if self._original_pixmap is None:
            return

        img_pos = self._label_to_image_pos(pos)
        if not self._original_pixmap.rect().toRectF().contains(img_pos):
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
