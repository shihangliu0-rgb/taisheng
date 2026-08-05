"""串口链路层：统一支持 USB 转 TTL 与无线透传模块（数传/蓝牙SPP/ESP-LINK 等）。

设计要点：
  * 收发在独立线程，GUI 不阻塞
  * 发送队列 + 可选发送节流，适配无线链路带宽有限的情况
  * 断线自动重连（无线链路常见掉线）
  * 支持 TCP 网络透传（部分无线 link 是 WiFi 转串口）
"""

from __future__ import annotations

import queue
import socket
import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Optional

import serial
import serial.tools.list_ports

from .protocol import FrameParser, Frame


@dataclass
class PortInfo:
    device: str
    description: str
    hwid: str

    @property
    def label(self) -> str:
        return f"{self.device}  —  {self.description}"


def list_ports() -> List[PortInfo]:
    out = []
    for p in serial.tools.list_ports.comports():
        out.append(PortInfo(p.device, p.description or "", p.hwid or ""))
    return out


def guess_link_kind(info: PortInfo) -> str:
    """粗略识别链路类型，用于给用户默认建议参数。"""
    s = (info.description + " " + info.hwid).lower()
    if any(k in s for k in ("bluetooth", "bt", "rfcomm", "spp")):
        return "wireless-bt"
    if any(k in s for k in ("cp210", "ch340", "ch910", "ft232", "pl2303", "usb-serial", "usb serial")):
        return "usb-ttl"
    return "unknown"


class BaseTransport:
    def open(self) -> None: ...
    def close(self) -> None: ...
    def read(self, n: int = 4096) -> bytes: ...
    def write(self, data: bytes) -> None: ...
    @property
    def is_open(self) -> bool: return False


class SerialTransport(BaseTransport):
    def __init__(self, port: str, baud: int = 115200, rtscts: bool = False):
        self.port, self.baud, self.rtscts = port, baud, rtscts
        self.ser: Optional[serial.Serial] = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.02,
                                 write_timeout=1.0, rtscts=self.rtscts)

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def read(self, n=4096) -> bytes:
        if not self.ser:
            return b""
        try:
            waiting = self.ser.in_waiting
            return self.ser.read(waiting or 1)
        except Exception:
            raise

    def write(self, data: bytes):
        if self.ser:
            self.ser.write(data)

    @property
    def is_open(self) -> bool:
        return bool(self.ser and self.ser.is_open)


class TcpTransport(BaseTransport):
    """WiFi 串口透传（如 ESP8266/ESP32 TCP Server、USR-WIFI232）。"""

    def __init__(self, host: str, port: int = 8888):
        self.host, self.port = host, port
        self.sock: Optional[socket.socket] = None

    def open(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=3)
        self.sock.settimeout(0.02)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None

    def read(self, n=4096) -> bytes:
        if not self.sock:
            return b""
        try:
            d = self.sock.recv(n)
            if d == b"":
                raise ConnectionError("peer closed")
            return d
        except socket.timeout:
            return b""

    def write(self, data: bytes):
        if self.sock:
            self.sock.sendall(data)

    @property
    def is_open(self) -> bool:
        return self.sock is not None


class Link:
    """带收发线程、自动重连与统计的链路。"""

    def __init__(self) -> None:
        self._tp: Optional[BaseTransport] = None
        self._factory: Optional[Callable[[], BaseTransport]] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_thread: Optional[threading.Thread] = None
        self._running = False
        self._txq: "queue.Queue[bytes]" = queue.Queue(maxsize=512)
        self.parser = FrameParser()
        self.auto_reconnect = True

        # 回调
        self.on_frame: Optional[Callable[[Frame], None]] = None
        self.on_raw_rx: Optional[Callable[[bytes], None]] = None
        self.on_raw_tx: Optional[Callable[[bytes], None]] = None
        self.on_status: Optional[Callable[[str, bool], None]] = None

        # 统计
        self.tx_bytes = 0
        self.rx_bytes = 0
        self.tx_frames = 0
        self.last_rx_time = 0.0
        self.connected = False

    # ---------------- 连接管理 ----------------
    def connect_serial(self, port: str, baud: int, rtscts: bool = False):
        self._start(lambda: SerialTransport(port, baud, rtscts), f"{port}@{baud}")

    def connect_tcp(self, host: str, port: int):
        self._start(lambda: TcpTransport(host, port), f"tcp://{host}:{port}")

    def _start(self, factory, label: str):
        self.disconnect()
        self._factory = factory
        self._label = label
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._rx_thread.start()
        self._tx_thread.start()

    def disconnect(self):
        self._running = False
        t = self._tp
        self._tp = None
        if t:
            t.close()
        self.connected = False

    # ---------------- 发送 ----------------
    def send(self, data: bytes, drop_if_full: bool = True):
        try:
            self._txq.put_nowait(data)
        except queue.Full:
            if not drop_if_full:
                self._txq.get_nowait()
                self._txq.put_nowait(data)

    # ---------------- 线程 ----------------
    def _notify(self, msg: str, ok: bool):
        if self.on_status:
            self.on_status(msg, ok)

    def _rx_loop(self):
        backoff = 0.5
        while self._running:
            if self._tp is None or not self._tp.is_open:
                try:
                    tp = self._factory()
                    tp.open()
                    self._tp = tp
                    self.connected = True
                    backoff = 0.5
                    self._notify(f"已连接 {self._label}", True)
                except Exception as e:
                    self.connected = False
                    self._notify(f"连接失败 {self._label}: {e}", False)
                    if not self.auto_reconnect:
                        self._running = False
                        return
                    time.sleep(backoff)
                    backoff = min(backoff * 1.5, 3.0)
                    continue
            try:
                data = self._tp.read()
                if data:
                    self.rx_bytes += len(data)
                    self.last_rx_time = time.time()
                    if self.on_raw_rx:
                        self.on_raw_rx(data)
                    for fr in self.parser.feed(data):
                        if self.on_frame:
                            self.on_frame(fr)
                else:
                    time.sleep(0.002)
            except Exception as e:
                self.connected = False
                self._notify(f"链路断开: {e}", False)
                if self._tp:
                    self._tp.close()
                self._tp = None
                if not self.auto_reconnect:
                    self._running = False
                    return
                time.sleep(0.5)

    def _tx_loop(self):
        while self._running:
            try:
                data = self._txq.get(timeout=0.1)
            except queue.Empty:
                continue
            tp = self._tp
            if tp is None or not tp.is_open:
                continue
            try:
                tp.write(data)
                self.tx_bytes += len(data)
                self.tx_frames += 1
                if self.on_raw_tx:
                    self.on_raw_tx(data)
            except Exception as e:
                self._notify(f"发送失败: {e}", False)
                self.connected = False
                if self._tp:
                    self._tp.close()
                self._tp = None
