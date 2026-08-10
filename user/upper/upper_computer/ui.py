"""上位机主界面。"""

from __future__ import annotations

import time
from typing import Dict, Optional

from PyQt5.QtCore import QEvent, Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QColor, QFont, QKeyEvent, QPainter, QPalette
from PyQt5.QtWidgets import (
    QAbstractItemView, QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
    QFileDialog, QFormLayout, QFrame, QGridLayout, QGroupBox, QHBoxLayout,
    QHeaderView, QLabel, QLineEdit, QMainWindow, QMessageBox, QPlainTextEdit,
    QPushButton, QSizePolicy, QSpinBox, QSplitter, QStatusBar, QTableWidget,
    QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget,
)

from . import protocol as P
from .channels import ChannelSelector
from .config import AppConfig, DEFAULT_KEYMAP
from .link import Link, list_ports, guess_link_kind
from .motion import MotionCommand, MotionEngine
from .plot import ScopeWidget

BAUDS = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 2000000]

KEY_TO_ACTION_DEFAULT = {
    Qt.Key_W: "forward", Qt.Key_S: "back", Qt.Key_A: "left", Qt.Key_D: "right",
    Qt.Key_Q: "ccw", Qt.Key_E: "cw", Qt.Key_Space: "brake",
    Qt.Key_Shift: "boost", Qt.Key_Control: "slow",
}

ACTION_LABEL = {
    "forward": "前进", "back": "后退", "left": "左平移", "right": "右平移",
    "ccw": "逆时针", "cw": "顺时针", "brake": "制动",
    "boost": "加速档", "slow": "慢速档",
}


class Bridge(QObject):
    """把链路线程的回调安全地转到 GUI 线程。"""
    frame = pyqtSignal(object)
    status = pyqtSignal(str, bool)
    raw_rx = pyqtSignal(bytes)
    raw_tx = pyqtSignal(bytes)


# ---------------------------------------------------------------------------
class KeyPad(QWidget):
    """键位状态可视化。"""

    def __init__(self, engine: MotionEngine):
        super().__init__()
        self.engine = engine
        self.setMinimumHeight(150)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#252526"))
        layout = [("Q", "ccw", 0, 0), ("W", "forward", 1, 0), ("E", "cw", 2, 0),
                  ("A", "left", 0, 1), ("S", "back", 1, 1), ("D", "right", 2, 1)]
        bw, bh, gap = 52, 42, 8
        x0, y0 = 14, 12
        p.setFont(QFont("Arial", 14, QFont.Bold))
        for label, act, cx, cy in layout:
            x = x0 + cx * (bw + gap)
            y = y0 + cy * (bh + gap)
            on = act in self.engine.actions
            p.setBrush(QColor("#0E7C4A") if on else QColor("#3A3A3C"))
            p.setPen(QColor("#111"))
            p.drawRoundedRect(x, y, bw, bh, 6, 6)
            p.setPen(QColor("#FFF") if on else QColor("#AAA"))
            p.drawText(x, y, bw, bh, Qt.AlignCenter, label)
        # 空格
        sx, sy, sw = x0, y0 + 2 * (bh + gap), bw * 3 + gap * 2
        on = "brake" in self.engine.actions
        p.setBrush(QColor("#B23A2E") if on else QColor("#3A3A3C"))
        p.setPen(QColor("#111"))
        p.drawRoundedRect(sx, sy, sw, bh - 8, 6, 6)
        p.setPen(QColor("#FFF") if on else QColor("#AAA"))
        p.setFont(QFont("Arial", 11, QFont.Bold))
        p.drawText(sx, sy, sw, bh - 8, Qt.AlignCenter, "SPACE  制动")
        # 档位
        p.setFont(QFont("Arial", 10))
        tx = x0 + 3 * (bw + gap) + 10
        for i, (name, act) in enumerate((("Shift 加速", "boost"), ("Ctrl 慢速", "slow"))):
            on = act in self.engine.actions
            p.setPen(QColor("#7CD992") if on else QColor("#777"))
            p.drawText(tx, y0 + 22 + i * 24, name)


# ---------------------------------------------------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.cfg = AppConfig.load()
        self.link = Link()
        self.engine = MotionEngine(self.cfg.motion)
        self.bridge = Bridge()
        self.seq = 0
        self.t0 = time.monotonic()
        self.keymap_enabled = False
        self.pid_cache: Dict[int, Dict[str, float]] = {}
        self.pid_ram: Dict[int, Dict[str, float]] = {}    # RAM 当前值
        self.pid_flash: Dict[int, Dict[str, float]] = {}  # Flash 已存值
        self.pending_flash: Optional[int] = None
        self.rx_frame_count = 0
        self._last_stat_t = time.time()
        self._last_tx_frames = 0
        self.sub_channels: list = []
        self.stream_t0: Optional[float] = None
        self.stream_mismatch = 0

        self.setWindowTitle("机器人调试上位机  ·  Upper Computer")
        self.resize(1280, 860)
        self._apply_dark()
        self._build_ui()
        self._wire_link()

        self.tx_timer = QTimer(self)
        self.tx_timer.timeout.connect(self._on_tx_tick)
        self.tx_timer.start(self.cfg.send.period_ms)

        self.hb_timer = QTimer(self)
        self.hb_timer.timeout.connect(self._send_heartbeat)
        self.hb_timer.start(self.cfg.send.heartbeat_ms)

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_status)
        self.ui_timer.start(200)

        self.setFocusPolicy(Qt.StrongFocus)

        # 全局键盘映射：装在 QApplication 上，任何页面/任何控件获得焦点时都能捕获
        QApplication.instance().installEventFilter(self)

    # ------------------------------------------------------------------
    def _apply_dark(self):
        pal = QPalette()
        pal.setColor(QPalette.Window, QColor("#2B2B2B"))
        pal.setColor(QPalette.WindowText, QColor("#E0E0E0"))
        pal.setColor(QPalette.Base, QColor("#1E1E1E"))
        pal.setColor(QPalette.AlternateBase, QColor("#2B2B2B"))
        pal.setColor(QPalette.Text, QColor("#E0E0E0"))
        pal.setColor(QPalette.Button, QColor("#3C3F41"))
        pal.setColor(QPalette.ButtonText, QColor("#E0E0E0"))
        pal.setColor(QPalette.Highlight, QColor("#0E639C"))
        pal.setColor(QPalette.HighlightedText, Qt.white)
        self.setPalette(pal)

    # ------------------------------------------------------------------
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)

        root.addWidget(self._build_connect_bar())

        tabs = QTabWidget()
        tabs.addTab(self._build_keyboard_tab(), "① 键盘映射 / 运动控制")
        tabs.addTab(self._build_pid_tab(), "② PID 在线调试")
        tabs.addTab(self._build_telemetry_tab(), "③ 遥测 / 曲线")
        tabs.addTab(self._build_console_tab(), "④ 原始数据 / 控制台")
        tabs.addTab(self._build_settings_tab(), "⑤ 设置")
        root.addWidget(tabs, 1)

        self.setStatusBar(QStatusBar())
        self.telem_bar = QLabel("")
        self.telem_bar.setFont(QFont("Consolas", 9))
        self.telem_bar.setStyleSheet("color:#9AA0A6")
        self.statusBar().addWidget(self.telem_bar, 1)
        self.lbl_keystate = QLabel("键盘映射: 关")
        self.statusBar().addPermanentWidget(self.lbl_keystate)
        self.lbl_conn = QLabel("未连接")
        self.lbl_rate = QLabel("TX 0 fps | RX 0 B/s")
        self.lbl_crc = QLabel("CRC err 0")
        for w in (self.lbl_conn, self.lbl_rate, self.lbl_crc):
            self.statusBar().addPermanentWidget(w)

    # ---------------- 连接栏 ----------------
    def _build_connect_bar(self) -> QWidget:
        box = QGroupBox("链路（USB 转 TTL / 无线数传 / 蓝牙 SPP / WiFi 透传 通用）")
        h = QHBoxLayout(box)

        self.cmb_kind = QComboBox()
        self.cmb_kind.addItems(["串口 Serial", "网络 TCP"])
        self.cmb_kind.currentIndexChanged.connect(self._on_kind_change)

        self.cmb_port = QComboBox()
        self.cmb_port.setMinimumWidth(300)
        self.btn_scan = QPushButton("刷新")
        self.btn_scan.clicked.connect(self.refresh_ports)

        self.cmb_baud = QComboBox()
        self.cmb_baud.setEditable(True)
        self.cmb_baud.addItems([str(b) for b in BAUDS])
        self.cmb_baud.setCurrentText(str(self.cfg.link.baud))

        self.ed_host = QLineEdit(self.cfg.link.host)
        self.ed_tcp_port = QSpinBox()
        self.ed_tcp_port.setRange(1, 65535)
        self.ed_tcp_port.setValue(self.cfg.link.tcp_port)
        self.ed_host.setVisible(False)
        self.ed_tcp_port.setVisible(False)

        self.chk_reconnect = QCheckBox("断线自动重连")
        self.chk_reconnect.setChecked(self.cfg.link.auto_reconnect)
        self.chk_rtscts = QCheckBox("硬件流控")
        self.chk_rtscts.setChecked(self.cfg.link.rtscts)

        self.cmb_fmt = QComboBox()
        self.cmb_fmt.addItems(["二进制帧 (推荐)", "ASCII 文本帧"])
        self.cmb_fmt.setCurrentIndex(0 if self.cfg.link.frame_format == "binary" else 1)

        self.btn_conn = QPushButton("连接")
        self.btn_conn.setCheckable(True)
        self.btn_conn.clicked.connect(self._toggle_connect)
        self.btn_conn.setMinimumWidth(90)

        for w in (QLabel("类型"), self.cmb_kind, QLabel("端口"), self.cmb_port,
                  self.btn_scan, self.ed_host, self.ed_tcp_port,
                  QLabel("波特率"), self.cmb_baud, self.chk_rtscts,
                  self.chk_reconnect, QLabel("帧格式"), self.cmb_fmt):
            h.addWidget(w)
        h.addStretch(1)
        h.addWidget(self.btn_conn)
        self.refresh_ports()
        return box

    def _on_kind_change(self, idx):
        serial_mode = idx == 0
        for w in (self.cmb_port, self.btn_scan, self.cmb_baud, self.chk_rtscts):
            w.setVisible(serial_mode)
        self.ed_host.setVisible(not serial_mode)
        self.ed_tcp_port.setVisible(not serial_mode)

    def refresh_ports(self):
        cur = self.cmb_port.currentData()
        self.cmb_port.clear()
        for p in list_ports():
            kind = guess_link_kind(p)
            tag = {"usb-ttl": "[USB-TTL]", "wireless-bt": "[无线/蓝牙]"}.get(kind, "")
            self.cmb_port.addItem(f"{tag} {p.label}", p.device)
        if self.cmb_port.count() == 0:
            self.cmb_port.addItem("（未检测到串口）", "")
        if cur:
            i = self.cmb_port.findData(cur)
            if i >= 0:
                self.cmb_port.setCurrentIndex(i)
        elif self.cfg.link.port:
            i = self.cmb_port.findData(self.cfg.link.port)
            if i >= 0:
                self.cmb_port.setCurrentIndex(i)

    # ---------------- ① 键盘 ----------------
    def _build_keyboard_tab(self) -> QWidget:
        w = QWidget()
        lay = QHBoxLayout(w)

        left = QVBoxLayout()

        g_en = QGroupBox("键盘映射")
        v = QVBoxLayout(g_en)
        self.btn_keymap = QPushButton("开启键盘映射  (F2)")
        self.btn_keymap.setCheckable(True)
        self.btn_keymap.setMinimumHeight(44)
        self.btn_keymap.clicked.connect(self._toggle_keymap)
        v.addWidget(self.btn_keymap)
        self.lbl_keyhint = QLabel(
            "W 前进 / S 后退 / A 左平移 / D 右平移 / Q 逆时针 / E 顺时针 / 空格 制动\n"
            "可组合：W+A 斜行，W+E 边走边转；Shift 加速档，Ctrl 慢速档。\n"
            "开启后请保持本窗口在前台；窗口失焦将自动急停（看门狗）。")
        self.lbl_keyhint.setStyleSheet("color:#9AA0A6")
        v.addWidget(self.lbl_keyhint)
        self.keypad = KeyPad(self.engine)
        v.addWidget(self.keypad)
        left.addWidget(g_en)

        g_sp = QGroupBox("最大速度 / 手感")
        f = QFormLayout(g_sp)
        m = self.cfg.motion
        self.sp_vx = self._spin(50, 20000, m.max_vx, " mm/s")
        self.sp_vy = self._spin(50, 20000, m.max_vy, " mm/s")
        self.sp_w = self._spin(50, 30000, m.max_omega, " mrad/s")
        self.sp_boost = self._dspin(1.0, 5.0, m.boost_scale, 0.1)
        self.sp_slow = self._dspin(0.05, 1.0, m.slow_scale, 0.05)
        self.sp_acc = self._spin(0, 3000, m.accel_ms, " ms")
        self.sp_dec = self._spin(0, 3000, m.decel_ms, " ms")
        self.chk_norm = QCheckBox("斜向归一化（W+A 不超速）")
        self.chk_norm.setChecked(m.normalize_diagonal)
        self.chk_field = QCheckBox("场地坐标系（flags bit3）")
        f.addRow("前后最大速度 vx", self.sp_vx)
        f.addRow("平移最大速度 vy", self.sp_vy)
        f.addRow("旋转最大角速度 ω", self.sp_w)
        f.addRow("Shift 加速倍率", self.sp_boost)
        f.addRow("Ctrl 慢速倍率", self.sp_slow)
        f.addRow("加速斜坡", self.sp_acc)
        f.addRow("减速斜坡", self.sp_dec)
        f.addRow(self.chk_norm)
        f.addRow(self.chk_field)
        for s in (self.sp_vx, self.sp_vy, self.sp_w, self.sp_acc, self.sp_dec):
            s.valueChanged.connect(self._apply_motion_cfg)
        for s in (self.sp_boost, self.sp_slow):
            s.valueChanged.connect(self._apply_motion_cfg)
        self.chk_norm.toggled.connect(self._apply_motion_cfg)
        self.chk_field.toggled.connect(self._apply_motion_cfg)
        left.addWidget(g_sp)
        left.addStretch(1)

        right = QVBoxLayout()
        g_mode = QGroupBox("发送模式")
        vm = QVBoxLayout(g_mode)
        self.cmb_sendmode = QComboBox()
        self.cmb_sendmode.addItems(["实时发送（周期连续下发）", "配置后点击发送（手动单次）"])
        self.cmb_sendmode.setCurrentIndex(0 if self.cfg.send.mode == "realtime" else 1)
        self.cmb_sendmode.currentIndexChanged.connect(self._apply_send_cfg)
        self.sp_period = self._spin(5, 1000, self.cfg.send.period_ms, " ms")
        self.sp_period.valueChanged.connect(self._apply_send_cfg)
        self.sp_hb = self._spin(0, 5000, self.cfg.send.heartbeat_ms, " ms")
        self.sp_hb.valueChanged.connect(self._apply_send_cfg)
        self.chk_zero = QCheckBox("空闲时仍周期发送零速（保活）")
        self.chk_zero.setChecked(self.cfg.send.send_zero_on_idle)
        self.chk_zero.toggled.connect(self._apply_send_cfg)
        fm = QFormLayout()
        fm.addRow("模式", self.cmb_sendmode)
        fm.addRow("实时周期", self.sp_period)
        fm.addRow("心跳周期(0关闭)", self.sp_hb)
        vm.addLayout(fm)
        vm.addWidget(self.chk_zero)
        self.btn_send_motion = QPushButton("发送当前运动指令 (Enter)")
        self.btn_send_motion.setMinimumHeight(38)
        self.btn_send_motion.clicked.connect(self._send_motion_once)
        vm.addWidget(self.btn_send_motion)
        right.addWidget(g_mode)

        g_man = QGroupBox("手动指令（不用键盘时直接填）")
        fman = QFormLayout(g_man)
        self.man_vx = self._spin(-32768, 32767, 0, " mm/s")
        self.man_vy = self._spin(-32768, 32767, 0, " mm/s")
        self.man_w = self._spin(-32768, 32767, 0, " mrad/s")
        self.chk_use_man = QCheckBox("使用手动值（覆盖键盘）")
        fman.addRow("vx", self.man_vx)
        fman.addRow("vy", self.man_vy)
        fman.addRow("ω", self.man_w)
        fman.addRow(self.chk_use_man)
        right.addWidget(g_man)

        g_out = QGroupBox("当前输出")
        vo = QVBoxLayout(g_out)
        self.lbl_out = QLabel("vx=0  vy=0  ω=0  flags=0x00")
        self.lbl_out.setFont(QFont("Consolas", 14, QFont.Bold))
        self.lbl_out.setStyleSheet("color:#7CD992")
        vo.addWidget(self.lbl_out)
        self.lbl_hex = QLabel("-")
        self.lbl_hex.setFont(QFont("Consolas", 9))
        self.lbl_hex.setStyleSheet("color:#888")
        self.lbl_hex.setWordWrap(True)
        vo.addWidget(self.lbl_hex)
        right.addWidget(g_out)

        g_es = QGroupBox("安全")
        ve = QVBoxLayout(g_es)
        btn_stop = QPushButton("急停 STOP (Esc)")
        btn_stop.setMinimumHeight(52)
        btn_stop.setStyleSheet("background:#B23A2E;color:white;font-size:16px;font-weight:bold")
        btn_stop.clicked.connect(self.emergency_stop)
        ve.addWidget(btn_stop)
        hmode = QHBoxLayout()
        for name, val in (("IDLE", 0), ("MANUAL", 1), ("AUTO", 2), ("PID_TUNE", 3)):
            b = QPushButton(name)
            b.clicked.connect(lambda _, v=val: self._send(P.Cmd.MODE, {"mode": v}))
            hmode.addWidget(b)
        ve.addLayout(hmode)
        right.addWidget(g_es)
        right.addStretch(1)

        lay.addLayout(left, 3)
        lay.addLayout(right, 2)
        return w

    # ---------------- ② PID ----------------
    def _build_pid_tab(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)

        top = QHBoxLayout()
        self.cmb_pid = QComboBox()
        for pid_id in self.cfg.pid_ids:
            label = P.PID_ID_ENUM.get(pid_id, f"USER 0x{pid_id:02X}")
            self.cmb_pid.addItem(f"0x{pid_id:02X}  {label}", pid_id)
        btn_read = QPushButton("读取 RAM 当前值")
        btn_read.setFocusPolicy(Qt.NoFocus)
        btn_read.clicked.connect(lambda: self._send(
            P.Cmd.PID_READ, {"pid_id": self.cmb_pid.currentData(), "source": 0}))
        btn_read_flash = QPushButton("读取 Flash 已存值")
        btn_read_flash.setFocusPolicy(Qt.NoFocus)
        btn_read_flash.clicked.connect(lambda: self._send(
            P.Cmd.PID_READ, {"pid_id": self.cmb_pid.currentData(), "source": 1}))
        btn_read_all = QPushButton("读取全部")
        btn_read_all.setFocusPolicy(Qt.NoFocus)
        btn_read_all.clicked.connect(lambda: self._send(
            P.Cmd.PID_READ, {"pid_id": 0xFF, "source": 0}))
        top.addWidget(QLabel("PID 通道"))
        top.addWidget(self.cmb_pid, 1)
        top.addWidget(btn_read)
        top.addWidget(btn_read_flash)
        top.addWidget(btn_read_all)
        self.cmb_pid.currentIndexChanged.connect(self._load_pid_to_form)
        lay.addLayout(top)

        body = QHBoxLayout()

        g_edit = QGroupBox("参数编辑（实时模式下改动即发送 / 手动模式下点击发送）")
        f = QFormLayout(g_edit)
        self.pid_fields: Dict[str, QDoubleSpinBox] = {}
        for key, label in (("kp", "Kp"), ("ki", "Ki"), ("kd", "Kd"),
                           ("i_limit", "积分限幅"), ("out_limit", "输出限幅")):
            sp = QDoubleSpinBox()
            sp.setRange(-1e6, 1e6)
            sp.setDecimals(4)
            sp.setSingleStep(0.01)
            sp.setKeyboardTracking(False)
            sp.valueChanged.connect(self._on_pid_changed)
            self.pid_fields[key] = sp
            f.addRow(label, sp)

        hstep = QHBoxLayout()
        self.sp_target = QDoubleSpinBox()
        self.sp_target.setRange(-1e6, 1e6)
        self.sp_target.setDecimals(3)
        btn_target = QPushButton("下发目标值")
        btn_target.clicked.connect(self._send_target)
        hstep.addWidget(self.sp_target, 1)
        hstep.addWidget(btn_target)
        f.addRow("阶跃目标值", hstep)

        hsq = QHBoxLayout()
        self.sp_sq_amp = QDoubleSpinBox()
        self.sp_sq_amp.setRange(0, 1e6)
        self.sp_sq_amp.setValue(100)
        self.sp_sq_period = QSpinBox()
        self.sp_sq_period.setRange(100, 20000)
        self.sp_sq_period.setValue(2000)
        self.sp_sq_period.setSuffix(" ms")
        self.btn_square = QPushButton("方波激励")
        self.btn_square.setCheckable(True)
        self.btn_square.toggled.connect(self._toggle_square)
        hsq.addWidget(self.sp_sq_amp)
        hsq.addWidget(self.sp_sq_period)
        hsq.addWidget(self.btn_square)
        f.addRow("方波幅值/周期", hsq)
        body.addWidget(g_edit, 1)

        # ---- 两级保存：先写 RAM 调，满意了再落 Flash ----
        g_store = QGroupBox("保存方案（先调 RAM，满意后再写 Flash）")
        vs = QVBoxLayout(g_store)

        self.btn_send_ram = QPushButton("① 写入 RAM（立即生效 · 掉电丢失）")
        self.btn_send_ram.setMinimumHeight(46)
        self.btn_send_ram.setFocusPolicy(Qt.NoFocus)
        self.btn_send_ram.setStyleSheet(
            "background:#0E639C;color:white;font-size:14px;font-weight:bold")
        self.btn_send_ram.setToolTip(
            "只改内存，马上就能看效果。反复调这个按钮，不会磨损 Flash。")
        self.btn_send_ram.clicked.connect(self._send_pid_ram)
        vs.addWidget(self.btn_send_ram)

        self.btn_save_flash = QPushButton("② 写入 Flash（固化 · 掉电不丢）")
        self.btn_save_flash.setMinimumHeight(46)
        self.btn_save_flash.setFocusPolicy(Qt.NoFocus)
        self.btn_save_flash.setStyleSheet(
            "background:#0E7C4A;color:white;font-size:14px;font-weight:bold")
        self.btn_save_flash.setToolTip(
            "把 RAM 里当前生效的参数固化到 Flash。调好了再点。")
        self.btn_save_flash.clicked.connect(self._save_pid_flash)
        vs.addWidget(self.btn_save_flash)

        hrow = QHBoxLayout()
        self.btn_revert = QPushButton("放弃改动，从 Flash 恢复")
        self.btn_revert.setFocusPolicy(Qt.NoFocus)
        self.btn_revert.setToolTip("调乱了想反悔时用，把 Flash 里的值重新load回 RAM。")
        self.btn_revert.clicked.connect(self._revert_pid)
        self.chk_save_all = QCheckBox("对全部通道操作")
        self.chk_save_all.setToolTip("勾上后 ②/恢复 作用于所有 PID 通道（pid_id=0xFF）")
        hrow.addWidget(self.btn_revert)
        hrow.addWidget(self.chk_save_all)
        vs.addLayout(hrow)

        self.lbl_dirty = QLabel("● 未读取参数")
        self.lbl_dirty.setFont(QFont("Arial", 11, QFont.Bold))
        self.lbl_dirty.setStyleSheet("color:#9AA0A6")
        vs.addWidget(self.lbl_dirty)

        self.lbl_pid_state = QLabel("尚未读取")
        self.lbl_pid_state.setStyleSheet("color:#9AA0A6")
        self.lbl_pid_state.setWordWrap(True)
        vs.addWidget(self.lbl_pid_state)

        hint = QLabel("实时模式下改动参数会自动写 RAM；Flash 永远只在点②时才写。")
        hint.setStyleSheet("color:#9AA0A6")
        hint.setWordWrap(True)
        vs.addWidget(hint)
        vs.addStretch(1)
        body.addWidget(g_store)

        g_tab = QGroupBox("已读取的 PID 表")
        vt = QVBoxLayout(g_tab)
        self.tbl_pid = QTableWidget(0, 7)
        self.tbl_pid.setHorizontalHeaderLabels(
            ["ID", "来源", "Kp", "Ki", "Kd", "积分限幅", "输出限幅"])
        self.tbl_pid.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tbl_pid.setEditTriggers(QAbstractItemView.NoEditTriggers)
        vt.addWidget(self.tbl_pid)
        body.addWidget(g_tab, 1)
        lay.addLayout(body)

        g_plot = QGroupBox("PID 响应曲线（目标 / 反馈 / 输出，来自 0x92 PID_CURVE）")
        vp = QVBoxLayout(g_plot)
        self.pid_scope = ScopeWidget()
        vp.addWidget(self.pid_scope)
        vp.addLayout(self._scope_toolbar(self.pid_scope, "pid"))
        lay.addWidget(g_plot, 1)
        return w

    # ---------------- ③ 遥测 ----------------
    def _build_telemetry_tab(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)

        g = QGroupBox("数据流控制（勾选哪些通道，下位机就只发哪些，省带宽）")
        h = QHBoxLayout(g)
        self.chk_stream = QCheckBox("开启数据流")
        self.chk_stream.toggled.connect(self._apply_stream)
        self.sp_stream_period = self._spin(5, 2000, 20, " ms")
        self.sp_stream_period.valueChanged.connect(self._apply_stream)
        btn_apply = QPushButton("重新下发订阅")
        btn_apply.setFocusPolicy(Qt.NoFocus)
        btn_apply.clicked.connect(self._apply_stream)
        btn_query = QPushButton("查询下位机通道")
        btn_query.setFocusPolicy(Qt.NoFocus)
        btn_query.clicked.connect(lambda: self._send(P.Cmd.STREAM_LIST, {}))
        self.chk_telem = QCheckBox("同时开启 TELEMETRY(0x91)")
        self.chk_telem.setChecked(True)
        self.chk_telem.toggled.connect(self._apply_telemetry)
        h.addWidget(self.chk_stream)
        h.addWidget(QLabel("上报周期"))
        h.addWidget(self.sp_stream_period)
        h.addWidget(btn_apply)
        h.addWidget(btn_query)
        h.addWidget(self.chk_telem)
        h.addStretch(1)
        self.lbl_stream = QLabel("未订阅")
        self.lbl_stream.setStyleSheet("color:#9AA0A6")
        h.addWidget(self.lbl_stream)
        lay.addWidget(g)

        split = QSplitter(Qt.Horizontal)

        left = QWidget()
        vl = QVBoxLayout(left)
        vl.setContentsMargins(0, 0, 0, 0)
        vl.addWidget(QLabel("选择要显示 / 订阅的数据通道："))
        self.chan_sel = ChannelSelector()
        self.chan_sel.selectionChanged.connect(self._on_channels_changed)
        vl.addWidget(self.chan_sel, 1)
        split.addWidget(left)

        right = QWidget()
        vr = QVBoxLayout(right)
        vr.setContentsMargins(0, 0, 0, 0)
        self.telem_scope = ScopeWidget()
        vr.addWidget(self.telem_scope, 1)
        vr.addLayout(self._scope_toolbar(self.telem_scope, "stream"))

        g2 = QGroupBox("当前值")
        self.telem_grid = QGridLayout(g2)
        self.telem_value_labels: Dict[int, QLabel] = {}
        g2.setMaximumHeight(190)
        vr.addWidget(g2)
        split.addWidget(right)
        split.setSizes([420, 900])
        lay.addWidget(split, 1)
        return w

    def _scope_toolbar(self, scope: ScopeWidget, tag: str) -> QHBoxLayout:
        """曲线通用工具条：暂停、清空、回到最新、时间窗、缓冲深度、导出。"""
        hb = QHBoxLayout()

        b_pause = QPushButton("暂停采集")
        b_pause.setCheckable(True)
        b_pause.setFocusPolicy(Qt.NoFocus)

        def _pause(on):
            scope.paused = on
            b_pause.setText("继续采集" if on else "暂停采集")
        b_pause.toggled.connect(_pause)

        b_live = QPushButton("回到最新")
        b_live.setFocusPolicy(Qt.NoFocus)
        b_live.clicked.connect(scope.go_live)

        b_clear = QPushButton("清空")
        b_clear.setFocusPolicy(Qt.NoFocus)
        b_clear.clicked.connect(scope.clear)

        sp_win = QDoubleSpinBox()
        sp_win.setRange(0.1, 3600.0)
        sp_win.setValue(10.0)
        sp_win.setSuffix(" s 窗口")
        sp_win.setFocusPolicy(Qt.ClickFocus)
        sp_win.valueChanged.connect(lambda v: (setattr(scope, "window_s", float(v)), scope.update()))

        cmb_cap = QComboBox()
        cmb_cap.setFocusPolicy(Qt.NoFocus)
        for label, n in (("5 万点", 50_000), ("20 万点", 200_000),
                         ("50 万点", 500_000), ("100 万点", 1_000_000)):
            cmb_cap.addItem(f"缓冲 {label}", n)
        cmb_cap.setCurrentIndex(1)
        cmb_cap.currentIndexChanged.connect(
            lambda: scope.set_capacity(cmb_cap.currentData()))

        b_csv = QPushButton("导出 CSV")
        b_csv.setFocusPolicy(Qt.NoFocus)
        b_csv.clicked.connect(lambda: self._export_csv(scope))

        lbl = QLabel("0 点 / 0.0 s")
        lbl.setStyleSheet("color:#9AA0A6")
        setattr(self, f"_lbl_buf_{tag}", lbl)

        for x in (b_pause, b_live, b_clear, sp_win, cmb_cap, b_csv):
            hb.addWidget(x)
        hb.addStretch(1)
        hb.addWidget(lbl)
        return hb

    def _export_csv(self, scope: ScopeWidget):
        path, _ = QFileDialog.getSaveFileName(
            self, "导出曲线数据", time.strftime("scope_%Y%m%d_%H%M%S.csv"), "CSV (*.csv)")
        if not path:
            return
        try:
            n = scope.export_csv(path)
            self.log(f"已导出 {n} 行到 {path}", "#7CD992")
        except Exception as e:
            self.log(f"导出失败: {e}", "#FF6E6E")

    # ---------------- ④ 控制台 ----------------
    def _build_console_tab(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        sp = QSplitter(Qt.Vertical)

        self.txt_log = QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumBlockCount(4000)
        self.txt_log.setFont(QFont("Consolas", 9))
        sp.addWidget(self.txt_log)

        bottom = QWidget()
        vb = QVBoxLayout(bottom)
        opts = QHBoxLayout()
        self.chk_show_rx = QCheckBox("显示 RX 帧")
        self.chk_show_rx.setChecked(True)
        self.chk_show_tx = QCheckBox("显示 TX 帧")
        self.chk_show_hex = QCheckBox("显示原始 HEX")
        self.chk_show_motion = QCheckBox("显示高频 MOTION 帧")
        btn_clear = QPushButton("清空")
        btn_clear.clicked.connect(self.txt_log.clear)
        for x in (self.chk_show_rx, self.chk_show_tx, self.chk_show_hex,
                  self.chk_show_motion, btn_clear):
            opts.addWidget(x)
        opts.addStretch(1)
        vb.addLayout(opts)

        send = QHBoxLayout()
        self.ed_raw = QLineEdit()
        self.ed_raw.setPlaceholderText("发送自定义文本（RAW_TEXT 0x7F），或以 hex: 开头发送裸字节，如 hex: AA 55 00 01")
        self.ed_raw.returnPressed.connect(self._send_raw)
        btn_raw = QPushButton("发送")
        btn_raw.clicked.connect(self._send_raw)
        send.addWidget(self.ed_raw, 1)
        send.addWidget(btn_raw)
        vb.addLayout(send)
        sp.addWidget(bottom)
        sp.setSizes([600, 120])
        lay.addWidget(sp)
        return w

    # ---------------- ⑤ 设置 ----------------
    def _build_settings_tab(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        g = QGroupBox("键位自定义（点击输入框后按下要绑定的键）")
        f = QFormLayout(g)
        self.key_edits: Dict[str, QLineEdit] = {}
        for act, label in ACTION_LABEL.items():
            ed = KeyCaptureEdit(self.cfg.keymap.get(act, DEFAULT_KEYMAP.get(act, "")))
            self.key_edits[act] = ed
            f.addRow(label, ed)
        lay.addWidget(g)

        hb = QHBoxLayout()
        btn_save = QPushButton("保存配置")
        btn_save.clicked.connect(self.save_config)
        btn_reset = QPushButton("恢复默认键位")
        btn_reset.clicked.connect(self._reset_keys)
        hb.addWidget(btn_save)
        hb.addWidget(btn_reset)
        hb.addStretch(1)
        lay.addLayout(hb)

        info = QLabel(
            f"协议版本 v{P.PROTOCOL_VERSION}\n"
            "数据格式文档：docs/PROTOCOL.md（运行 python tools/gen_protocol_doc.py 自动更新）\n"
            "扩展方式：在 upper_computer/protocol.py 中新增 Message 定义即可，"
            "文档、编解码同步生效。")
        info.setStyleSheet("color:#9AA0A6")
        lay.addWidget(info)
        lay.addStretch(1)
        return w

    def _reset_keys(self):
        for act, ed in self.key_edits.items():
            ed.setText(DEFAULT_KEYMAP.get(act, ""))

    # ------------------------------------------------------------------
    @staticmethod
    def _spin(lo, hi, val, suffix=""):
        s = QSpinBox()
        s.setRange(lo, hi)
        s.setValue(val)
        if suffix:
            s.setSuffix(suffix)
        s.setKeyboardTracking(False)
        return s

    @staticmethod
    def _dspin(lo, hi, val, step):
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setValue(val)
        s.setSingleStep(step)
        s.setKeyboardTracking(False)
        return s

    # ------------------------------------------------------------------
    def _wire_link(self):
        self.bridge.frame.connect(self._handle_frame)
        self.bridge.status.connect(self._handle_status)
        self.bridge.raw_rx.connect(self._handle_raw_rx)
        self.bridge.raw_tx.connect(self._handle_raw_tx)
        self.link.on_frame = self.bridge.frame.emit
        self.link.on_status = self.bridge.status.emit
        self.link.on_raw_rx = self.bridge.raw_rx.emit
        self.link.on_raw_tx = self.bridge.raw_tx.emit

    def _toggle_connect(self, checked):
        if checked:
            if self.cmb_kind.currentIndex() == 0:
                dev = self.cmb_port.currentData()
                if not dev:
                    QMessageBox.warning(self, "提示", "没有可用串口")
                    self.btn_conn.setChecked(False)
                    return
                try:
                    baud = int(self.cmb_baud.currentText())
                except ValueError:
                    baud = 115200
                self.link.auto_reconnect = self.chk_reconnect.isChecked()
                self.link.connect_serial(dev, baud, self.chk_rtscts.isChecked())
                self.cfg.link.port, self.cfg.link.baud = dev, baud
            else:
                self.link.auto_reconnect = self.chk_reconnect.isChecked()
                self.link.connect_tcp(self.ed_host.text(), self.ed_tcp_port.value())
                self.cfg.link.host = self.ed_host.text()
                self.cfg.link.tcp_port = self.ed_tcp_port.value()
            self.btn_conn.setText("断开")
        else:
            self.link.disconnect()
            self.btn_conn.setText("连接")
            self.log("链路已断开", "#FFB74D")

    # ------------------------------------------------------------------
    def _apply_motion_cfg(self):
        m = self.cfg.motion
        m.max_vx = self.sp_vx.value()
        m.max_vy = self.sp_vy.value()
        m.max_omega = self.sp_w.value()
        m.boost_scale = self.sp_boost.value()
        m.slow_scale = self.sp_slow.value()
        m.accel_ms = self.sp_acc.value()
        m.decel_ms = self.sp_dec.value()
        m.normalize_diagonal = self.chk_norm.isChecked()
        self.engine.field_frame = self.chk_field.isChecked()

    def _apply_send_cfg(self):
        s = self.cfg.send
        s.mode = "realtime" if self.cmb_sendmode.currentIndex() == 0 else "manual"
        s.period_ms = self.sp_period.value()
        s.heartbeat_ms = self.sp_hb.value()
        s.send_zero_on_idle = self.chk_zero.isChecked()
        self.tx_timer.setInterval(s.period_ms)
        if s.heartbeat_ms > 0:
            self.hb_timer.setInterval(s.heartbeat_ms)
            self.hb_timer.start()
        else:
            self.hb_timer.stop()
        self.btn_send_motion.setEnabled(True)
        if hasattr(self, "btn_send_ram"):
            self.btn_send_ram.setText(
                "① 写入 RAM（实时模式：改动即自动写）" if s.mode == "realtime"
                else "① 写入 RAM（立即生效 · 掉电丢失）")

    @property
    def realtime(self) -> bool:
        return self.cfg.send.mode == "realtime"

    # ------------------------------------------------------------------
    def _next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    def _send(self, cmd: P.Cmd, values: Optional[dict] = None, quiet: bool = False):
        if self.cmb_fmt.currentIndex() == 0:
            data = P.encode(cmd, values or {}, self._next_seq())
        else:
            data = P.encode_text(cmd, values or {})
        self.link.send(data)
        if not quiet and self.chk_show_tx.isChecked():
            self.log(f"TX  {P.MESSAGES[int(cmd)].name}  {values}", "#64B5F6")
        return data

    def _send_heartbeat(self):
        if self.link.connected:
            self._send(P.Cmd.HEARTBEAT,
                       {"timestamp_ms": int((time.monotonic() - self.t0) * 1000)}, quiet=True)

    def _current_command(self) -> MotionCommand:
        cmd = self.engine.update()
        if self.chk_use_man.isChecked():
            cmd.vx = self.man_vx.value()
            cmd.vy = self.man_vy.value()
            cmd.omega = self.man_w.value()
        return cmd

    def _on_tx_tick(self):
        cmd = self._current_command()
        self.keypad.update()
        self.lbl_out.setText(
            f"vx={cmd.vx:<7d} vy={cmd.vy:<7d} ω={cmd.omega:<7d} flags=0x{cmd.flags:02X}")
        if self.realtime and self.link.connected and self.keymap_enabled:
            if cmd.is_zero() and not self.cfg.send.send_zero_on_idle:
                return
            data = self._send(P.Cmd.MOTION, cmd.as_dict(),
                              quiet=not self.chk_show_motion.isChecked())
            self.lbl_hex.setText(data.hex(" ").upper())
        # 方波激励
        self._square_tick()

    def _send_motion_once(self):
        cmd = self._current_command()
        data = self._send(P.Cmd.MOTION, cmd.as_dict())
        self.lbl_hex.setText(data.hex(" ").upper())

    def emergency_stop(self):
        self.engine.clear()
        self._send(P.Cmd.BRAKE, {"level": 2})
        self.log("!! 急停已发送 (BRAKE level=2)", "#FF6E6E")

    # ------------------------------------------------------------------
    def _toggle_keymap(self, on):
        self.keymap_enabled = on
        if self.btn_keymap.isChecked() != on:
            self.btn_keymap.setChecked(on)
        self.btn_keymap.setText("关闭键盘映射  (F2)" if on else "开启键盘映射  (F2)")
        self.btn_keymap.setStyleSheet(
            "background:#0E7C4A;color:white;font-weight:bold" if on else "")
        self.lbl_keystate.setText("键盘映射: 全局开启" if on else "键盘映射: 关")
        self.lbl_keystate.setStyleSheet(
            "color:#7CD992;font-weight:bold" if on else "color:#9AA0A6")
        if on:
            self._send(P.Cmd.MODE, {"mode": 1})
            self.log("键盘映射已开启（全局生效，切到任何标签页都可用）", "#7CD992")
        else:
            self.engine.clear()
            self._send(P.Cmd.MOTION, MotionCommand().as_dict())
            self.log("键盘映射已关闭", "#FFB74D")

    def _action_of(self, e: QKeyEvent) -> Optional[str]:
        txt = e.text().upper().strip()
        key = e.key()
        for act, binding in self.cfg.keymap.items():
            b = binding.strip()
            if not b:
                continue
            if b.lower() == "space" and key == Qt.Key_Space:
                return act
            if b.lower() == "shift" and key == Qt.Key_Shift:
                return act
            if b.lower() == "ctrl" and key == Qt.Key_Control:
                return act
            if len(b) == 1 and txt == b.upper():
                return act
        return KEY_TO_ACTION_DEFAULT.get(key)

    # ------------------------------------------------------------------
    # 全局键盘映射
    #   事件过滤器装在 QApplication 上，因此无论当前在哪个标签页、
    #   焦点在哪个控件上，只要映射已开启就能捕获方向键。
    #   例外：焦点在文本框/数字框里时放行，否则没法打字。
    # ------------------------------------------------------------------
    @staticmethod
    def _is_text_input(w) -> bool:
        from PyQt5.QtWidgets import QAbstractSpinBox, QComboBox
        while w is not None:
            if isinstance(w, (QLineEdit, QPlainTextEdit, QAbstractSpinBox)):
                # 键位捕获框除外，它本来就要吃按键
                return not isinstance(w, KeyCaptureEdit)
            if isinstance(w, QComboBox) and w.isEditable():
                return True
            w = w.parentWidget()
        return False

    def eventFilter(self, obj, ev):
        t = ev.type()
        if t not in (QEvent.KeyPress, QEvent.KeyRelease):
            return super().eventFilter(obj, ev)
        # 只处理本窗口内的事件
        win = self.window()
        w = obj if isinstance(obj, QWidget) else None
        if w is not None and w.window() is not win:
            return super().eventFilter(obj, ev)

        key = ev.key()

        # 急停与开关：全局生效，文本框里也生效
        if t == QEvent.KeyPress:
            if key == Qt.Key_Escape:
                self.emergency_stop()
                return True
            if key == Qt.Key_F2:
                self.btn_keymap.setChecked(not self.btn_keymap.isChecked())
                self._toggle_keymap(self.btn_keymap.isChecked())
                return True

        if not self.keymap_enabled:
            return super().eventFilter(obj, ev)

        focus = QApplication.focusWidget()
        if self._is_text_input(focus):
            return super().eventFilter(obj, ev)   # 正在打字，放行

        if ev.isAutoRepeat():
            return True    # 吃掉系统自动重复，按住状态由我们自己维护

        act = self._action_of(ev)
        if act:
            if t == QEvent.KeyPress:
                self.engine.press(act)
            else:
                self.engine.release(act)
            self.keypad.update()
            return True    # 拦截，避免 WASD 触发按钮助记符/表格滚动

        if t == QEvent.KeyPress and key in (Qt.Key_Return, Qt.Key_Enter):
            self._send_motion_once()
            return True
        return super().eventFilter(obj, ev)

    def keyPressEvent(self, e: QKeyEvent):
        # 兜底：事件过滤器已处理绝大多数情况
        if e.key() in (Qt.Key_Return, Qt.Key_Enter):
            self._send_motion_once()
            return
        super().keyPressEvent(e)

    def changeEvent(self, e):
        # 失焦看门狗
        if self.cfg.motion.deadman and not self.isActiveWindow() and self.keymap_enabled:
            if self.engine.actions:
                self.engine.clear()
                self._send(P.Cmd.MOTION, MotionCommand().as_dict())
                self.log("窗口失焦，已自动归零", "#FFB74D")
        super().changeEvent(e)

    # ------------------------------------------------------------------
    # PID
    # ------------------------------------------------------------------
    def _pid_values(self, target: int = 0) -> dict:
        v = {k: sp.value() for k, sp in self.pid_fields.items()}
        v["pid_id"] = self.cmb_pid.currentData()
        v["target"] = target      # 0=只写 RAM，1=同时写 Flash
        return v

    def _save_scope_id(self) -> int:
        """②/恢复 的作用范围：当前通道 或 全部(0xFF)。"""
        return 0xFF if self.chk_save_all.isChecked() else self.cmb_pid.currentData()

    def _on_pid_changed(self):
        """界面数值变动：实时模式下自动写 RAM（绝不自动写 Flash）。"""
        pid_id = self.cmb_pid.currentData()
        self.pid_ram[pid_id] = {k: sp.value() for k, sp in self.pid_fields.items()}
        if self.realtime and self.link.connected:
            self._send(P.Cmd.PID_WRITE, self._pid_values(target=0))
        self._update_dirty()

    def _send_pid_ram(self):
        """① 写 RAM。"""
        self._send(P.Cmd.PID_WRITE, self._pid_values(target=0))
        pid_id = self.cmb_pid.currentData()
        self.pid_ram[pid_id] = {k: sp.value() for k, sp in self.pid_fields.items()}
        self.lbl_pid_state.setText("已写入 RAM，等待 ACK …")
        self._update_dirty()

    def _save_pid_flash(self):
        """② 固化到 Flash，先确认，避免误触磨损 Flash。"""
        pid_id = self._save_scope_id()
        scope = "全部通道" if pid_id == 0xFF else \
            f"通道 0x{pid_id:02X}（{P.PID_ID_ENUM.get(pid_id, '自定义')}）"
        r = QMessageBox.question(
            self, "写入 Flash",
            f"确认把 RAM 中当前参数固化到 Flash？\n\n作用范围：{scope}\n\n"
            "Flash 有擦写寿命，建议参数调好后再执行。",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        if r != QMessageBox.Yes:
            return
        # 先补一次 RAM 写，确保 Flash 存下来的就是界面上看到的
        self._send(P.Cmd.PID_WRITE, self._pid_values(target=0))
        self._send(P.Cmd.PID_SAVE, {"pid_id": pid_id})
        self.pending_flash = pid_id
        self.lbl_pid_state.setText("正在写入 Flash，等待 ACK …")
        self.log(f"PID_SAVE -> Flash（{scope}）", "#7CD992")

    def _revert_pid(self):
        pid_id = self._save_scope_id()
        r = QMessageBox.question(
            self, "放弃改动",
            "放弃 RAM 中的临时改动，从 Flash 重新加载参数？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if r != QMessageBox.Yes:
            return
        self._send(P.Cmd.PID_REVERT, {"pid_id": pid_id})
        self._send(P.Cmd.PID_READ, {"pid_id": self.cmb_pid.currentData(), "source": 0})
        self.lbl_pid_state.setText("已请求从 Flash 恢复 …")

    def _update_dirty(self):
        """比较 RAM 值与 Flash 值，提示是否有未保存改动。"""
        pid_id = self.cmb_pid.currentData()
        ram = self.pid_ram.get(pid_id)
        flash = self.pid_flash.get(pid_id)
        if ram is None:
            self.lbl_dirty.setText("● 未读取参数")
            self.lbl_dirty.setStyleSheet("color:#9AA0A6")
            return
        if flash is None:
            self.lbl_dirty.setText("● RAM 已改 · Flash 值未知（可点『读取 Flash 已存值』对比）")
            self.lbl_dirty.setStyleSheet("color:#FFB74D")
            return
        diff = [k for k in ("kp", "ki", "kd", "i_limit", "out_limit")
                if abs(float(ram.get(k, 0)) - float(flash.get(k, 0))) > 1e-6]
        if diff:
            self.lbl_dirty.setText(f"● 有未保存改动：{', '.join(diff)} —— 点②写入 Flash")
            self.lbl_dirty.setStyleSheet("color:#FFB74D;font-weight:bold")
        else:
            self.lbl_dirty.setText("● RAM 与 Flash 一致，已保存")
            self.lbl_dirty.setStyleSheet("color:#7CD992;font-weight:bold")

    def _send_target(self):
        self._send(P.Cmd.PID_TARGET, {"pid_id": self.cmb_pid.currentData(),
                                      "target": self.sp_target.value()})

    def _toggle_square(self, on):
        self._sq_last = 0.0
        self._sq_state = False
        if not on:
            self._send(P.Cmd.PID_TARGET, {"pid_id": self.cmb_pid.currentData(), "target": 0.0})

    def _square_tick(self):
        if not getattr(self, "btn_square", None) or not self.btn_square.isChecked():
            return
        now = time.monotonic()
        if now - getattr(self, "_sq_last", 0.0) >= self.sp_sq_period.value() / 2000.0:
            self._sq_last = now
            self._sq_state = not getattr(self, "_sq_state", False)
            amp = self.sp_sq_amp.value()
            self._send(P.Cmd.PID_TARGET,
                       {"pid_id": self.cmb_pid.currentData(),
                        "target": amp if self._sq_state else 0.0}, quiet=True)

    def _load_pid_to_form(self):
        pid_id = self.cmb_pid.currentData()
        d = self.pid_cache.get(pid_id)
        if not d:
            self.lbl_pid_state.setText("该通道尚未读取，点击『读取当前 PID』")
            return
        for k, sp in self.pid_fields.items():
            sp.blockSignals(True)
            sp.setValue(d.get(k, 0.0))
            sp.blockSignals(False)
        self.lbl_pid_state.setText("已载入下位机当前值")

    def _update_pid_table(self, pid_id: int, d: dict, source: int = 0):
        """RAM 与 Flash 各占一行，方便直接对比。"""
        tag = "Flash" if source == 1 else "RAM"
        key = f"0x{pid_id:02X}"
        row = None
        for r in range(self.tbl_pid.rowCount()):
            if (self.tbl_pid.item(r, 0).text() == key
                    and self.tbl_pid.item(r, 1).text() == tag):
                row = r
                break
        if row is None:
            row = self.tbl_pid.rowCount()
            self.tbl_pid.insertRow(row)
            self.tbl_pid.setItem(row, 0, QTableWidgetItem(key))
            it = QTableWidgetItem(tag)
            it.setForeground(QColor("#7CD992" if source == 1 else "#4FC3F7"))
            self.tbl_pid.setItem(row, 1, it)
        for c, k in enumerate(("kp", "ki", "kd", "i_limit", "out_limit"), start=2):
            self.tbl_pid.setItem(row, c, QTableWidgetItem(f"{d.get(k, 0.0):.4f}"))

    # ------------------------------------------------------------------
    # 数据流订阅
    # ------------------------------------------------------------------
    def _on_channels_changed(self, ids):
        self.sub_channels = list(ids)
        # 曲线里移除已取消勾选的通道
        names = {P.CHANNELS[c].name for c in ids if c in P.CHANNELS}
        for n in list(self.telem_scope.series.keys()):
            if n not in names:
                self.telem_scope.remove_series(n)
        self._rebuild_value_grid()
        if self.chk_stream.isChecked():
            self._apply_stream()
        else:
            self.lbl_stream.setText(f"已选 {len(ids)} 通道（未开启数据流）")

    def _rebuild_value_grid(self):
        while self.telem_grid.count():
            it = self.telem_grid.takeAt(0)
            if it.widget():
                it.widget().deleteLater()
        self.telem_value_labels.clear()
        for i, cid in enumerate(self.sub_channels):
            c = P.CHANNELS.get(cid)
            if not c:
                continue
            name = QLabel(f"{c.name} ({c.unit})")
            name.setStyleSheet("color:#9AA0A6")
            val = QLabel("-")
            val.setFont(QFont("Consolas", 12, QFont.Bold))
            col = i % 5
            row = i // 5
            self.telem_grid.addWidget(name, row * 2, col)
            self.telem_grid.addWidget(val, row * 2 + 1, col)
            self.telem_value_labels[cid] = val

    def _apply_stream(self):
        ids = getattr(self, "sub_channels", [])
        on = self.chk_stream.isChecked()
        if on and not ids:
            self.lbl_stream.setText("请先勾选通道")
            return
        self._send(P.Cmd.STREAM_CFG, {
            "enable": 1 if on else 0,
            "period_ms": self.sp_stream_period.value(),
            "count": len(ids),
            "channels": ids,
        })
        if on:
            names = ", ".join(P.CHANNELS[c].name for c in ids if c in P.CHANNELS)
            self.lbl_stream.setText(f"订阅 {len(ids)} 通道 @{self.sp_stream_period.value()}ms")
            self.log(f"数据流订阅: [{names}] @{self.sp_stream_period.value()}ms", "#7CD992")
        else:
            self.lbl_stream.setText("数据流已停止")

    def _apply_telemetry(self):
        self._send(P.Cmd.TELEM_CTRL, {"enable": 1 if self.chk_telem.isChecked() else 0,
                                      "period_ms": self.sp_stream_period.value(),
                                      "mask": 0x03})

    def _send_raw(self):
        s = self.ed_raw.text().strip()
        if not s:
            return
        if s.lower().startswith("hex:"):
            try:
                data = bytes.fromhex(s[4:].replace(",", " "))
            except ValueError:
                self.log("HEX 格式错误", "#FF6E6E")
                return
            self.link.send(data)
            self.log(f"TX RAW  {data.hex(' ').upper()}", "#64B5F6")
        else:
            self._send(P.Cmd.RAW_TEXT, {"text": s})
        self.ed_raw.clear()

    # ------------------------------------------------------------------
    # 接收处理
    # ------------------------------------------------------------------
    def _handle_frame(self, fr: P.Frame):
        self.rx_frame_count += 1
        d = fr.parse()
        t = time.monotonic() - self.t0

        if fr.cmd == int(P.Cmd.PID_VALUE):
            pid_id = d.get("pid_id", 0)
            src = d.get("source", 0)
            if src == 1:
                self.pid_flash[pid_id] = d      # Flash 里存的值
            else:
                self.pid_cache[pid_id] = d      # RAM 当前值
                self.pid_ram[pid_id] = d
                if d.get("dirty", 0) == 0 and pid_id not in self.pid_flash:
                    # 下位机说不脏，那 Flash 值就等于 RAM 值
                    self.pid_flash[pid_id] = dict(d)
            self._update_pid_table(pid_id, d, src)
            if pid_id == self.cmb_pid.currentData():
                if src == 0:
                    self._load_pid_to_form()
                else:
                    self.lbl_pid_state.setText(
                        "已读回 Flash 值：" + "  ".join(
                            f"{k}={d.get(k,0):.4g}"
                            for k in ("kp", "ki", "kd")))
                self._update_dirty()
        elif fr.cmd == int(P.Cmd.STREAM_DATA):
            self._on_stream_data(d)
            return
        elif fr.cmd == int(P.Cmd.STREAM_INFO):
            self.log(f"下位机通道 0x{d.get('channel_id',0):02X} = {d.get('name','')}", "#A5D6A7")
            return
        elif fr.cmd == int(P.Cmd.TELEMETRY):
            self._show_telemetry(d, t)
        elif fr.cmd == int(P.Cmd.PID_CURVE):
            self.pid_scope.add_point("target", t, d.get("target", 0.0))
            self.pid_scope.add_point("feedback", t, d.get("feedback", 0.0))
            self.pid_scope.add_point("output", t, d.get("output", 0.0))
            self.pid_scope.update()
            return
        elif fr.cmd == int(P.Cmd.ACK):
            ack_cmd = d.get("ack_cmd", 0)
            status = d.get("status", 0)
            st = P.ACK_STATUS.get(status, "?")
            self.lbl_pid_state.setText(f"ACK: 0x{ack_cmd:02X} -> {st}")
            if ack_cmd == int(P.Cmd.PID_SAVE):
                if status == 0:
                    pid_id = getattr(self, "pending_flash", None)
                    ids = list(self.pid_ram) if pid_id == 0xFF else [pid_id]
                    for i in ids:
                        if i in self.pid_ram:
                            self.pid_flash[i] = dict(self.pid_ram[i])
                    self.pending_flash = None
                    self.log("✓ Flash 写入成功，参数已固化", "#7CD992")
                else:
                    self.log(f"✗ Flash 写入失败: {st}", "#FF6E6E")
                self._update_dirty()
            elif ack_cmd == int(P.Cmd.PID_REVERT) and status == 0:
                self.log("已从 Flash 恢复参数", "#7CD992")
        elif fr.cmd == int(P.Cmd.LOG):
            lvl = d.get("level", 1)
            color = {0: "#888", 1: "#DDD", 2: "#FFB74D", 3: "#FF6E6E"}.get(lvl, "#DDD")
            self.log(f"MCU: {d.get('text','')}", color)
            return

        if self.chk_show_rx.isChecked():
            self.log(f"RX  {fr.name}  {d}", "#A5D6A7")

    def _on_stream_data(self, d: dict):
        vals = d.get("values", [])
        ids = getattr(self, "sub_channels", [])
        if len(vals) != len(ids):
            # 订阅刚变更，下位机还在发旧格式，跳过这帧
            self.stream_mismatch += 1
            return
        ts = d.get("timestamp_ms", 0) / 1000.0
        if self.stream_t0 is None:
            self.stream_t0 = ts
        t = ts - self.stream_t0
        for cid, v in zip(ids, vals):
            c = P.CHANNELS.get(cid)
            if not c:
                continue
            self.telem_scope.add_point(c.name, t, v)
            lbl = self.telem_value_labels.get(cid)
            if lbl:
                lbl.setText(f"{v:.4g}")
        self.telem_scope.update()

    def _show_telemetry(self, d: dict, t: float):
        self.telem_bar.setText(
            f"vx={d.get('vx',0)}  vy={d.get('vy',0)}  ω={d.get('omega',0)}  "
            f"yaw={d.get('yaw',0)/100.0:.2f}°  "
            f"电池={d.get('battery_mv',0)/1000.0:.2f}V  "
            f"状态={d.get('state',0)}  错误={d.get('err_code',0)}")

    def _handle_raw_rx(self, data: bytes):
        if self.chk_show_hex.isChecked():
            self.log(f"RX HEX  {data.hex(' ').upper()}", "#607D8B")

    def _handle_raw_tx(self, data: bytes):
        if self.chk_show_hex.isChecked() and self.chk_show_tx.isChecked():
            self.log(f"TX HEX  {data.hex(' ').upper()}", "#455A64")

    def _handle_status(self, msg: str, ok: bool):
        self.log(msg, "#7CD992" if ok else "#FF6E6E")
        self.lbl_conn.setText(msg)

    def _refresh_status(self):
        now = time.time()
        dt = now - self._last_stat_t
        if dt >= 0.5:
            fps = (self.link.tx_frames - self._last_tx_frames) / dt
            self._last_tx_frames = self.link.tx_frames
            self._last_stat_t = now
            self.lbl_rate.setText(
                f"TX {fps:.0f} fps / {self.link.tx_bytes} B | RX {self.link.rx_bytes} B | 帧 {self.link.parser.stat_ok}")
        self.lbl_crc.setText(
            f"CRC err {self.link.parser.stat_crc_err} | 丢弃 {self.link.parser.stat_dropped}")
        for tag, scope in (("stream", self.telem_scope), ("pid", self.pid_scope)):
            lbl = getattr(self, f"_lbl_buf_{tag}", None)
            if lbl:
                lbl.setText(f"{scope.point_count():,} 点 / {scope.time_span():.1f} s"
                            + ("" if scope.follow else "  · 回溯中"))
        color = "#7CD992" if self.link.connected else "#FF6E6E"
        self.lbl_conn.setStyleSheet(f"color:{color}")

    def log(self, text: str, color: str = "#DDD"):
        ts = time.strftime("%H:%M:%S")
        self.txt_log.appendHtml(
            f'<span style="color:#666">{ts}</span> <span style="color:{color}">{_esc(text)}</span>')

    # ------------------------------------------------------------------
    def save_config(self):
        self._apply_motion_cfg()
        self._apply_send_cfg()
        for act, ed in self.key_edits.items():
            self.cfg.keymap[act] = ed.text()
        self.cfg.link.frame_format = "binary" if self.cmb_fmt.currentIndex() == 0 else "text"
        self.cfg.link.auto_reconnect = self.chk_reconnect.isChecked()
        self.cfg.link.rtscts = self.chk_rtscts.isChecked()
        self.cfg.save()
        self.log("配置已保存 ~/.upper_computer/config.json", "#7CD992")

    def closeEvent(self, e):
        try:
            if self.link.connected:
                self._send(P.Cmd.BRAKE, {"level": 2})
                time.sleep(0.05)
            self.save_config()
        except Exception:
            pass
        self.link.disconnect()
        super().closeEvent(e)


def _esc(s: str) -> str:
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


class KeyCaptureEdit(QLineEdit):
    def __init__(self, text=""):
        super().__init__(text)
        self.setReadOnly(True)
        self.setPlaceholderText("点击后按键…")

    def keyPressEvent(self, e: QKeyEvent):
        k = e.key()
        if k == Qt.Key_Space:
            self.setText("Space")
        elif k == Qt.Key_Shift:
            self.setText("Shift")
        elif k == Qt.Key_Control:
            self.setText("Ctrl")
        elif e.text().strip():
            self.setText(e.text().upper())


def run():
    import sys
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
