from __future__ import annotations

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

from PyQt6.QtCore import QObject, QTimer, pyqtSignal, pyqtSlot

from app_config import STOP_REPEAT_COUNT


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
