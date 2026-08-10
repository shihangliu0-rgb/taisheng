"""
通信协议定义（唯一事实来源 / single source of truth）

本文件同时用于：
  1. 上位机收发编解码
  2. 自动生成 docs/PROTOCOL.md （tools/gen_protocol_doc.py）

修改本文件后运行：  python tools/gen_protocol_doc.py
即可自动更新数据格式文档，保证文档与代码永远一致。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Dict, List, Optional, Tuple

PROTOCOL_VERSION = "1.2.0"

# ---------------------------------------------------------------------------
# 帧结构常量
# ---------------------------------------------------------------------------
FRAME_HEAD_0 = 0xAA
FRAME_HEAD_1 = 0x55
MAX_PAYLOAD = 240
FRAME_OVERHEAD = 7  # HEAD(2) + LEN(1) + CMD(1) + SEQ(1) + CRC(2)


# ---------------------------------------------------------------------------
# 命令号
# ---------------------------------------------------------------------------
class Cmd(IntEnum):  # noqa: E301
    # ---- 上位机 -> 下位机 (0x00 - 0x7F) ----
    HEARTBEAT = 0x01      # 心跳 / 链路保活
    MOTION = 0x02         # 运动控制（键盘映射输出）
    BRAKE = 0x03          # 制动 / 急停
    MODE = 0x04           # 控制模式切换
    PID_READ = 0x10       # 请求读取 PID
    PID_WRITE = 0x11      # 写入 PID（可选只写 RAM 或同时写 Flash）
    PID_SAVE = 0x12       # 把 RAM 中当前 PID 固化到 Flash
    PID_TARGET = 0x13     # 设置某个环的目标值（阶跃调试）
    PID_REVERT = 0x14     # 放弃 RAM 改动，从 Flash 重新加载
    PARAM_READ = 0x20     # 通用参数读
    PARAM_WRITE = 0x21    # 通用参数写
    TELEM_CTRL = 0x30     # 遥测使能 / 周期设置
    STREAM_CFG = 0x31     # 订阅数据流：指定只上报哪些通道
    STREAM_LIST = 0x32    # 请求下位机上报它支持的通道清单
    RAW_TEXT = 0x7F       # 透传字符串（自定义调试命令）

    # ---- 下位机 -> 上位机 (0x80 - 0xFF) ----
    ACK = 0x80            # 通用应答
    PID_VALUE = 0x90      # PID 当前值回报
    TELEMETRY = 0x91      # 周期遥测（速度/里程计/电池等）
    PID_CURVE = 0x92      # PID 调试曲线（目标 vs 实际）
    STREAM_DATA = 0x93    # 按订阅上报的通道数据（IMU/里程计/自定义）
    STREAM_INFO = 0x94    # 通道清单应答（下位机自报家门）
    LOG = 0x9F            # 文本日志


# ---------------------------------------------------------------------------
# 字段声明（用于自动文档 + 编解码）
# ---------------------------------------------------------------------------
_FMT = {
    "u8": ("B", 1),
    "i8": ("b", 1),
    "u16": ("<H", 2),
    "i16": ("<h", 2),
    "u32": ("<I", 4),
    "i32": ("<i", 4),
    "f32": ("<f", 4),
}

# 变长类型：占满 payload 剩余部分
_ARRAY_TYPES = {"u8[]": "u8", "f32[]": "f32"}


@dataclass
class Field:
    name: str
    type: str            # u8/i8/u16/i16/u32/i32/f32/str
    unit: str = "-"
    desc: str = ""
    enum: Optional[Dict[int, str]] = None


@dataclass
class Message:
    cmd: Cmd
    name: str
    direction: str       # "PC->MCU" | "MCU->PC"
    desc: str
    fields: List[Field] = field(default_factory=list)
    notes: str = ""

    # ---------------- 编解码 ----------------
    def size(self) -> Optional[int]:
        n = 0
        for f in self.fields:
            if f.type == "str" or f.type in _ARRAY_TYPES:
                return None  # 变长
            n += _FMT[f.type][1]
        return n

    def pack(self, values: Dict[str, Any]) -> bytes:
        out = bytearray()
        for f in self.fields:
            v = values.get(f.name, 0)
            if f.type == "str":
                out += v.encode("utf-8") if isinstance(v, str) else bytes(v)
            elif f.type in _ARRAY_TYPES:
                elem = _ARRAY_TYPES[f.type]
                fmt = _FMT[elem][0]
                seq = v if isinstance(v, (list, tuple)) else []
                for item in seq:
                    out += struct.pack(fmt, float(item) if elem == "f32" else int(item))
            else:
                fmt = _FMT[f.type][0]
                if f.type == "f32":
                    out += struct.pack(fmt, float(v))
                else:
                    out += struct.pack(fmt, int(v))
        return bytes(out)

    def unpack(self, payload: bytes) -> Dict[str, Any]:
        res: Dict[str, Any] = {}
        off = 0
        for f in self.fields:
            if f.type == "str":
                res[f.name] = payload[off:].decode("utf-8", errors="replace")
                off = len(payload)
                continue
            if f.type in _ARRAY_TYPES:
                elem = _ARRAY_TYPES[f.type]
                fmt, n = _FMT[elem]
                items = []
                while off + n <= len(payload):
                    items.append(struct.unpack_from(fmt, payload, off)[0])
                    off += n
                res[f.name] = items
                continue
            fmt, n = _FMT[f.type]
            if off + n > len(payload):
                res[f.name] = 0
                continue
            res[f.name] = struct.unpack_from(fmt, payload, off)[0]
            off += n
        return res


# ---------------------------------------------------------------------------
# 枚举字典
# ---------------------------------------------------------------------------
MOTION_FLAGS = {
    0x01: "BRAKE  制动（忽略速度分量，抱死）",
    0x02: "BOOST  加速档（按住 Shift）",
    0x04: "SLOW   慢速档（按住 Ctrl）",
    0x08: "FIELD  场地坐标系（否则为车体坐标系）",
}

MODE_ENUM = {
    0: "IDLE      空闲/失能",
    1: "MANUAL    手动（键盘映射）",
    2: "AUTO      自动/自主",
    3: "PID_TUNE  PID 调试（允许阶跃输入）",
}

PID_ID_ENUM = {
    0: "CHASSIS_LF   左前轮速度环",
    1: "CHASSIS_RF   右前轮速度环",
    2: "CHASSIS_LB   左后轮速度环",
    3: "CHASSIS_RB   右后轮速度环",
    0x10: "YAW_ANGLE  云台/底盘 偏航角度环",
    0x11: "YAW_SPEED  偏航角速度环",
    0x20: "USER_0     用户自定义 0",
    0x21: "USER_1     用户自定义 1",
}

# ---------------------------------------------------------------------------
# 可订阅数据通道表
#   上位机勾选哪些通道 -> 发 STREAM_CFG(0x31) 告诉下位机 -> 下位机只发这几个
#   通道值统一按 f32 传输，顺序 = 订阅时给出的 ID 顺序
# ---------------------------------------------------------------------------
@dataclass
class Channel:
    id: int
    name: str
    unit: str
    desc: str
    group: str


CHANNELS: Dict[int, Channel] = {}


def _ch(cid: int, name: str, unit: str, desc: str, group: str) -> Channel:
    c = Channel(cid, name, unit, desc, group)
    CHANNELS[cid] = c
    return c


# --- IMU：姿态角 (0x01-0x0F) ---
_ch(0x01, "imu_roll", "deg", "横滚角", "IMU 姿态")
_ch(0x02, "imu_pitch", "deg", "俯仰角", "IMU 姿态")
_ch(0x03, "imu_yaw", "deg", "偏航角（航向）", "IMU 姿态")
_ch(0x04, "imu_quat_w", "-", "四元数 w", "IMU 姿态")
_ch(0x05, "imu_quat_x", "-", "四元数 x", "IMU 姿态")
_ch(0x06, "imu_quat_y", "-", "四元数 y", "IMU 姿态")
_ch(0x07, "imu_quat_z", "-", "四元数 z", "IMU 姿态")

# --- IMU：陀螺仪 (0x10-0x1F) ---
_ch(0x10, "gyro_x", "deg/s", "陀螺仪 X 轴角速度", "IMU 陀螺仪")
_ch(0x11, "gyro_y", "deg/s", "陀螺仪 Y 轴角速度", "IMU 陀螺仪")
_ch(0x12, "gyro_z", "deg/s", "陀螺仪 Z 轴角速度", "IMU 陀螺仪")

# --- IMU：加速度计 (0x20-0x2F) ---
_ch(0x20, "accel_x", "m/s^2", "加速度 X", "IMU 加速度计")
_ch(0x21, "accel_y", "m/s^2", "加速度 Y", "IMU 加速度计")
_ch(0x22, "accel_z", "m/s^2", "加速度 Z", "IMU 加速度计")

# --- IMU：磁力计 + 温度 (0x30-0x3F) ---
_ch(0x30, "mag_x", "uT", "磁力计 X", "IMU 磁力计")
_ch(0x31, "mag_y", "uT", "磁力计 Y", "IMU 磁力计")
_ch(0x32, "mag_z", "uT", "磁力计 Z", "IMU 磁力计")
_ch(0x33, "imu_temp", "degC", "IMU 温度", "IMU 磁力计")

# --- 里程计 / 位移 (0x40-0x4F) ---
_ch(0x40, "odom_x", "m", "位移 X（累计）", "里程计")
_ch(0x41, "odom_y", "m", "位移 Y（累计）", "里程计")
_ch(0x42, "odom_theta", "deg", "航向（里程计推算）", "里程计")
_ch(0x43, "odom_dist", "m", "累计行驶距离", "里程计")
_ch(0x44, "vel_x", "mm/s", "实际速度 vx", "里程计")
_ch(0x45, "vel_y", "mm/s", "实际速度 vy", "里程计")
_ch(0x46, "vel_omega", "mrad/s", "实际角速度", "里程计")

# --- 轮子 (0x50-0x5F) ---
_ch(0x50, "wheel_lf_spd", "rpm", "左前轮实际转速", "轮子")
_ch(0x51, "wheel_rf_spd", "rpm", "右前轮实际转速", "轮子")
_ch(0x52, "wheel_lb_spd", "rpm", "左后轮实际转速", "轮子")
_ch(0x53, "wheel_rb_spd", "rpm", "右后轮实际转速", "轮子")
_ch(0x54, "wheel_lf_set", "rpm", "左前轮目标转速", "轮子")
_ch(0x55, "wheel_rf_set", "rpm", "右前轮目标转速", "轮子")
_ch(0x56, "wheel_lb_set", "rpm", "左后轮目标转速", "轮子")
_ch(0x57, "wheel_rb_set", "rpm", "右后轮目标转速", "轮子")

# --- DT35 激光测距 (0x80-0x8F) ---
# 常见用法：车身四周各装一个，用于贴边/找墙/定位校正
_ch(0x80, "dt35_front", "mm", "DT35 前向距离", "DT35 激光测距")
_ch(0x81, "dt35_back", "mm", "DT35 后向距离", "DT35 激光测距")
_ch(0x82, "dt35_left", "mm", "DT35 左侧距离", "DT35 激光测距")
_ch(0x83, "dt35_right", "mm", "DT35 右侧距离", "DT35 激光测距")
_ch(0x84, "dt35_lf", "mm", "DT35 左前距离", "DT35 激光测距")
_ch(0x85, "dt35_rf", "mm", "DT35 右前距离", "DT35 激光测距")
_ch(0x86, "dt35_lb", "mm", "DT35 左后距离", "DT35 激光测距")
_ch(0x87, "dt35_rb", "mm", "DT35 右后距离", "DT35 激光测距")
_ch(0x88, "dt35_raw_mv", "mV", "DT35 原始模拟量（模拟输出型接 ADC 时用）", "DT35 激光测距")
_ch(0x89, "dt35_yaw_calc", "deg", "由左右两个 DT35 差值算出的贴边角度", "DT35 激光测距")
_ch(0x8A, "dt35_valid", "-", "有效性位图 bit0..bit7 对应 0x80..0x87，1=数据有效",
    "DT35 激光测距")

# --- 电源 / 系统 (0x60-0x6F) ---
_ch(0x60, "battery_v", "V", "电池电压", "电源/系统")
_ch(0x61, "battery_a", "A", "总电流", "电源/系统")
_ch(0x62, "cpu_load", "%", "CPU 占用", "电源/系统")
_ch(0x63, "loop_dt", "ms", "主控制周期耗时", "电源/系统")

# --- 用户自定义 (0x70-0x7F) ---
for _i in range(8):
    _ch(0x70 + _i, f"user_{_i}", "-", f"用户自定义通道 {_i}", "用户自定义")

CHANNEL_BY_NAME = {c.name: c for c in CHANNELS.values()}


def channel_groups() -> Dict[str, List[Channel]]:
    """按分组返回通道，供 UI 构建勾选树。"""
    out: Dict[str, List[Channel]] = {}
    for c in CHANNELS.values():
        out.setdefault(c.group, []).append(c)
    return out


ACK_STATUS = {
    0: "OK        执行成功",
    1: "ERR_CRC   校验错误",
    2: "ERR_CMD   未知命令",
    3: "ERR_ARG   参数非法",
    4: "ERR_BUSY  设备忙",
    5: "ERR_FLASH Flash 擦写失败",
    6: "ERR_LOCK  参数被锁定，不允许修改",
}

# 参数存储目标：调试时只写 RAM，满意了再落 Flash
STORE_TARGET = {
    0: "RAM    只写内存，立即生效、掉电丢失（调试用，不磨损 Flash）",
    1: "FLASH  写内存的同时固化到 Flash，掉电不丢",
}

# PID_VALUE 回报里的来源标记
PID_SRC = {
    0: "RAM    当前生效值（可能含未保存的临时改动）",
    1: "FLASH  Flash 中已保存的值",
}


# ---------------------------------------------------------------------------
# 报文表
# ---------------------------------------------------------------------------
MESSAGES: Dict[int, Message] = {}


def _reg(m: Message) -> Message:
    MESSAGES[int(m.cmd)] = m
    return m


MSG_HEARTBEAT = _reg(Message(
    Cmd.HEARTBEAT, "HEARTBEAT", "PC->MCU",
    "链路保活。上位机按固定周期（默认 500ms）发送；下位机若超时（建议 1s）未收到，"
    "应自动进入制动/失能状态，防止无线链路断开后失控。",
    [
        Field("timestamp_ms", "u32", "ms", "上位机单调时间戳"),
    ],
))

MSG_MOTION = _reg(Message(
    Cmd.MOTION, "MOTION", "PC->MCU",
    "运动控制指令。键盘映射与摇杆均输出此帧。实时模式下按发送周期（默认 20ms/50Hz）"
    "连续发送；配置模式下点击『发送』时发一帧。",
    [
        Field("vx", "i16", "mm/s", "前后速度，前为正（W 前进 / S 后退）"),
        Field("vy", "i16", "mm/s", "左右平移速度，左为正（A 左移 / D 右移）"),
        Field("omega", "i16", "mrad/s", "旋转角速度，逆时针为正（Q 逆时针 / E 顺时针）"),
        Field("flags", "u8", "-", "标志位，见下表", enum=MOTION_FLAGS),
        Field("seq_echo", "u8", "-", "预留，回显用，当前固定 0"),
    ],
    notes="三个分量可同时非零（如 W+A 斜走、W+E 边走边转）。"
          "下位机应对合速度做限幅，不要相信上位机不越界。",
))

MSG_BRAKE = _reg(Message(
    Cmd.BRAKE, "BRAKE", "PC->MCU",
    "制动 / 急停。空格键触发。level=0 缓停(速度归零)，1 抱死，2 急停（切断输出，需要重新使能）。",
    [
        Field("level", "u8", "-", "0=缓停 1=抱死 2=急停"),
    ],
))

MSG_MODE = _reg(Message(
    Cmd.MODE, "MODE", "PC->MCU",
    "控制模式切换。",
    [
        Field("mode", "u8", "-", "模式枚举", enum=MODE_ENUM),
    ],
))

MSG_PID_READ = _reg(Message(
    Cmd.PID_READ, "PID_READ", "PC->MCU",
    "请求读取指定 PID 参数。下位机收到后应回 PID_VALUE(0x90)。pid_id=0xFF 表示读取全部（逐条回）。",
    [
        Field("pid_id", "u8", "-", "PID 通道号", enum=PID_ID_ENUM),
        Field("source", "u8", "-", "读哪一份：0=RAM当前值(默认) 1=Flash已存值",
              enum=PID_SRC),
    ],
    notes="读 Flash 值可用来和当前 RAM 值做对比，确认改动是否已保存。"
          "老固件按 1 字节解析也不受影响（source 在末尾，缺省按 0）。",
))

MSG_PID_WRITE = _reg(Message(
    Cmd.PID_WRITE, "PID_WRITE", "PC->MCU",
    "写入 PID 参数。**默认只写 RAM（掉电丢失），适合反复试参数**；"
    "调好之后再用 PID_SAVE(0x12) 固化到 Flash。也可以把 target 直接设成 1 一步到位写 Flash，"
    "但调试阶段不建议，Flash 有擦写寿命。",
    [
        Field("pid_id", "u8", "-", "PID 通道号", enum=PID_ID_ENUM),
        Field("kp", "f32", "-", "比例系数"),
        Field("ki", "f32", "-", "积分系数"),
        Field("kd", "f32", "-", "微分系数"),
        Field("i_limit", "f32", "-", "积分限幅（0 表示不限）"),
        Field("out_limit", "f32", "-", "输出限幅（0 表示不限）"),
        Field("target", "u8", "-", "存储目标：0=只写RAM(默认) 1=同时写Flash",
              enum=STORE_TARGET),
    ],
    notes="target 放在末尾，向后兼容：老固件按 21 字节解析前面字段不受影响；"
          "新固件收到 22 字节时取最后一字节，收到 21 字节按 0（只写 RAM）处理。",
))

MSG_PID_SAVE = _reg(Message(
    Cmd.PID_SAVE, "PID_SAVE", "PC->MCU",
    "把 RAM 里当前生效的 PID 固化到 Flash/EEPROM。**这是『调好了，存下来』的动作**，"
    "上位机点『写入 Flash』时发送。",
    [
        Field("pid_id", "u8", "-", "通道号，0xFF = 全部", enum=PID_ID_ENUM),
    ],
    notes="建议下位机写完回 ACK(0x80)，status=0 表示烧写成功。"
          "Flash 擦写期间若会关中断导致通信卡顿，请在回 ACK 后再执行。",
))

MSG_PID_REVERT = _reg(Message(
    Cmd.PID_REVERT, "PID_REVERT", "PC->MCU",
    "放弃 RAM 中的临时改动，从 Flash 重新加载参数到 RAM。"
    "调参调乱了想一键还原到上次保存的状态时用。",
    [
        Field("pid_id", "u8", "-", "通道号，0xFF = 全部", enum=PID_ID_ENUM),
    ],
    notes="下位机重新加载后，建议主动回一条 PID_VALUE(0x90) 让上位机界面同步刷新。",
))

MSG_PID_TARGET = _reg(Message(
    Cmd.PID_TARGET, "PID_TARGET", "PC->MCU",
    "给指定环下发目标值（阶跃/方波调试用）。",
    [
        Field("pid_id", "u8", "-", "通道号", enum=PID_ID_ENUM),
        Field("target", "f32", "-", "目标值，单位由该环自身定义"),
    ],
))

MSG_PARAM_READ = _reg(Message(
    Cmd.PARAM_READ, "PARAM_READ", "PC->MCU",
    "通用参数读取（预留扩展：轮距、轮径、限速、遥控死区等）。",
    [
        Field("param_id", "u16", "-", "参数 ID，由用户自行约定"),
    ],
))

MSG_PARAM_WRITE = _reg(Message(
    Cmd.PARAM_WRITE, "PARAM_WRITE", "PC->MCU",
    "通用参数写入。value 统一按 f32 传输，下位机自行转换类型。",
    [
        Field("param_id", "u16", "-", "参数 ID"),
        Field("value", "f32", "-", "参数值"),
    ],
))

MSG_TELEM_CTRL = _reg(Message(
    Cmd.TELEM_CTRL, "TELEM_CTRL", "PC->MCU",
    "遥测上报控制。",
    [
        Field("enable", "u8", "-", "0=停止上报 1=开启上报"),
        Field("period_ms", "u16", "ms", "上报周期，建议 >= 10ms"),
        Field("mask", "u8", "-", "bit0=TELEMETRY bit1=PID_CURVE"),
    ],
))

MSG_STREAM_CFG = _reg(Message(
    Cmd.STREAM_CFG, "STREAM_CFG", "PC->MCU",
    "订阅数据流：告诉下位机『我只要这几个通道』。上位机在曲线界面勾选后自动下发。"
    "下位机之后按 period_ms 周期，用 STREAM_DATA(0x93) 上报，"
    "**且只发订阅列表里的通道，顺序与本帧 channels 完全一致**，不用全发。",
    [
        Field("enable", "u8", "-", "0=停止数据流 1=开始"),
        Field("period_ms", "u16", "ms", "上报周期，建议 10~100ms；无线链路别低于 20ms"),
        Field("count", "u8", "-", "订阅通道数量，0~60"),
        Field("channels", "u8[]", "-", "通道 ID 列表，长度 = count，见『可订阅通道表』"),
    ],
    notes="举例：只想看 yaw 和位移，就订阅 [0x03, 0x40, 0x41]，"
          "之后每帧 STREAM_DATA 只有 3 个 f32，共 12 字节 payload，非常省带宽。"
          "重新勾选会重发本帧覆盖旧订阅。",
))

MSG_STREAM_LIST = _reg(Message(
    Cmd.STREAM_LIST, "STREAM_LIST", "PC->MCU",
    "询问下位机支持哪些通道（可选实现）。下位机回一条或多条 STREAM_INFO(0x94)。"
    "不实现也没关系，上位机会用内置的标准通道表。",
    [],
))

MSG_RAW_TEXT = _reg(Message(
    Cmd.RAW_TEXT, "RAW_TEXT", "PC->MCU",
    "透传自定义字符串命令，方便临时加调试指令而不改协议。",
    [
        Field("text", "str", "-", "UTF-8 字符串，不含结束符"),
    ],
))

MSG_ACK = _reg(Message(
    Cmd.ACK, "ACK", "MCU->PC",
    "通用应答。建议对所有写类命令（PID_WRITE/PARAM_WRITE/MODE 等）回应答。",
    [
        Field("ack_cmd", "u8", "-", "被应答的命令号"),
        Field("ack_seq", "u8", "-", "被应答帧的 SEQ"),
        Field("status", "u8", "-", "状态码", enum=ACK_STATUS),
    ],
))

MSG_PID_VALUE = _reg(Message(
    Cmd.PID_VALUE, "PID_VALUE", "MCU->PC",
    "PID 当前值回报。上位机据此刷新界面上『当前读取到的 PID』。",
    [
        Field("pid_id", "u8", "-", "通道号", enum=PID_ID_ENUM),
        Field("kp", "f32", "-", "比例"),
        Field("ki", "f32", "-", "积分"),
        Field("kd", "f32", "-", "微分"),
        Field("i_limit", "f32", "-", "积分限幅"),
        Field("out_limit", "f32", "-", "输出限幅"),
        Field("source", "u8", "-", "本条数据来自哪：0=RAM 1=Flash", enum=PID_SRC),
        Field("dirty", "u8", "-", "1 = RAM 值与 Flash 不一致（有未保存改动）"),
    ],
    notes="`dirty` 让上位机能提示『当前参数尚未写入 Flash』。"
          "下位机若不想实现，固定填 0 即可。",
))

MSG_TELEMETRY = _reg(Message(
    Cmd.TELEMETRY, "TELEMETRY", "MCU->PC",
    "周期遥测，用于状态栏与曲线显示。",
    [
        Field("timestamp_ms", "u32", "ms", "下位机时间戳"),
        Field("vx", "i16", "mm/s", "实际前后速度"),
        Field("vy", "i16", "mm/s", "实际平移速度"),
        Field("omega", "i16", "mrad/s", "实际角速度"),
        Field("yaw", "i16", "0.01deg", "航向角"),
        Field("battery_mv", "u16", "mV", "电池电压"),
        Field("state", "u8", "-", "0=IDLE 1=RUN 2=ERROR"),
        Field("err_code", "u8", "-", "错误码，用户自定义"),
    ],
))

MSG_PID_CURVE = _reg(Message(
    Cmd.PID_CURVE, "PID_CURVE", "MCU->PC",
    "PID 调试曲线数据，上位机实时绘图（目标 vs 实际）。",
    [
        Field("pid_id", "u8", "-", "通道号", enum=PID_ID_ENUM),
        Field("timestamp_ms", "u32", "ms", "下位机时间戳"),
        Field("target", "f32", "-", "目标值"),
        Field("feedback", "f32", "-", "反馈值"),
        Field("output", "f32", "-", "PID 输出"),
    ],
))

MSG_STREAM_DATA = _reg(Message(
    Cmd.STREAM_DATA, "STREAM_DATA", "MCU->PC",
    "按订阅上报的通道数据。**只包含 STREAM_CFG 里订阅的通道，顺序完全一致**，"
    "不带通道 ID（省带宽），上位机按订阅顺序对号入座。",
    [
        Field("timestamp_ms", "u32", "ms", "下位机时间戳，用于曲线横轴"),
        Field("values", "f32[]", "-", "各通道值，数量 = 订阅的 count，顺序同订阅列表"),
    ],
    notes="payload 长度 = 4 + 4*count。若上位机收到的数量与当前订阅不符，会忽略该帧"
          "（说明订阅刚变更，下一帧就对上了）。",
))

MSG_STREAM_INFO = _reg(Message(
    Cmd.STREAM_INFO, "STREAM_INFO", "MCU->PC",
    "通道清单应答：下位机自报支持的通道，让上位机动态显示。可选实现。",
    [
        Field("channel_id", "u8", "-", "通道 ID"),
        Field("unit", "u8", "-", "单位枚举（0=无 1=deg 2=deg/s 3=m/s^2 4=m 5=mm/s 6=V 7=A 8=rpm 9=自定义）"),
        Field("name", "str", "-", "通道名称，UTF-8"),
    ],
))

MSG_LOG = _reg(Message(
    Cmd.LOG, "LOG", "MCU->PC",
    "下位机文本日志，直接打印到上位机控制台。",
    [
        Field("level", "u8", "-", "0=DEBUG 1=INFO 2=WARN 3=ERROR"),
        Field("text", "str", "-", "UTF-8 日志正文"),
    ],
))


# ---------------------------------------------------------------------------
# CRC16 / MODBUS  (poly 0xA001, init 0xFFFF)
# ---------------------------------------------------------------------------
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# ---------------------------------------------------------------------------
# 二进制帧编解码
# ---------------------------------------------------------------------------
def build_frame(cmd: int, payload: bytes = b"", seq: int = 0) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes([len(payload), int(cmd) & 0xFF, seq & 0xFF]) + payload
    c = crc16(body)
    return bytes([FRAME_HEAD_0, FRAME_HEAD_1]) + body + struct.pack("<H", c)


def encode(cmd: Cmd, values: Optional[Dict[str, Any]] = None, seq: int = 0) -> bytes:
    msg = MESSAGES[int(cmd)]
    return build_frame(int(cmd), msg.pack(values or {}), seq)


@dataclass
class Frame:
    cmd: int
    seq: int
    payload: bytes

    @property
    def name(self) -> str:
        m = MESSAGES.get(self.cmd)
        return m.name if m else f"UNKNOWN_0x{self.cmd:02X}"

    def parse(self) -> Dict[str, Any]:
        m = MESSAGES.get(self.cmd)
        return m.unpack(self.payload) if m else {"raw": self.payload.hex(" ")}


class FrameParser:
    """流式帧解析器，容忍无线链路上的丢包/串扰。"""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.stat_ok = 0
        self.stat_crc_err = 0
        self.stat_dropped = 0

    def feed(self, data: bytes) -> List[Frame]:
        self.buf += data
        out: List[Frame] = []
        while True:
            i = self.buf.find(bytes([FRAME_HEAD_0, FRAME_HEAD_1]))
            if i < 0:
                if len(self.buf) > 1:
                    self.stat_dropped += len(self.buf) - 1
                    del self.buf[:-1]
                break
            if i > 0:
                self.stat_dropped += i
                del self.buf[:i]
            if len(self.buf) < FRAME_OVERHEAD:
                break
            ln = self.buf[2]
            total = FRAME_OVERHEAD + ln
            if ln > MAX_PAYLOAD:
                self.stat_dropped += 2
                del self.buf[:2]
                continue
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:3 + 2 + ln])  # LEN CMD SEQ payload
            rx_crc = struct.unpack_from("<H", self.buf, 5 + ln)[0]
            if crc16(body) == rx_crc:
                out.append(Frame(cmd=self.buf[3], seq=self.buf[4],
                                 payload=bytes(self.buf[5:5 + ln])))
                self.stat_ok += 1
                del self.buf[:total]
            else:
                self.stat_crc_err += 1
                del self.buf[:2]
        return out


# ---------------------------------------------------------------------------
# ASCII 文本帧（可选模式，便于串口助手/低端 MCU 直接解析）
#   格式: $NAME,arg1,arg2,...*XX\r\n     XX = 逗号内容按字节异或的 2 位大写 HEX
# ---------------------------------------------------------------------------
def build_text_frame(name: str, args: List[Any]) -> bytes:
    body = name + ("," + ",".join(_fmt_arg(a) for a in args) if args else "")
    cs = 0
    for ch in body.encode():
        cs ^= ch
    return f"${body}*{cs:02X}\r\n".encode()


def _fmt_arg(a: Any) -> str:
    if isinstance(a, float):
        return f"{a:.6g}"
    return str(a)


def encode_text(cmd: Cmd, values: Optional[Dict[str, Any]] = None) -> bytes:
    msg = MESSAGES[int(cmd)]
    values = values or {}
    return build_text_frame(msg.name, [values.get(f.name, 0) for f in msg.fields])


def parse_text_line(line: str) -> Optional[Tuple[str, List[str]]]:
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None
    body, _, cs = line[1:].partition("*")
    calc = 0
    for ch in body.encode():
        calc ^= ch
    try:
        if calc != int(cs[:2], 16):
            return None
    except ValueError:
        return None
    parts = body.split(",")
    return parts[0], parts[1:]
