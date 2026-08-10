#!/usr/bin/env python3
"""由 upper_computer/protocol.py 自动生成 docs/PROTOCOL.md。

用法:  python tools/gen_protocol_doc.py
以后每次改协议，跑一次这个脚本，文档自动同步（也可由 upper_computer/__main__.py 启动时自动执行）。
"""

from __future__ import annotations

import os
import sys
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from upper_computer import protocol as P  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "docs", "PROTOCOL.md")

TYPE_DESC = {
    "u8": "uint8", "i8": "int8", "u16": "uint16 (小端)", "i16": "int16 (小端)",
    "u32": "uint32 (小端)", "i32": "int32 (小端)", "f32": "float32 IEEE754 (小端)",
    "str": "UTF-8 字符串（变长，占满剩余 payload）",
    "u8[]": "uint8 数组（变长，占满剩余 payload）",
    "f32[]": "float32 数组（变长，小端，占满剩余 payload）",
}

_VARLEN = ("str",) + tuple(P._ARRAY_TYPES)


def field_table(msg: P.Message) -> str:
    if not msg.fields:
        return "_无 payload（LEN = 0）_\n"
    lines = ["| 偏移 | 字段 | 类型 | 单位 | 说明 |", "|---:|---|---|---|---|"]
    off = 0
    for f in msg.fields:
        varlen = f.type in _VARLEN
        n = 0 if varlen else P._FMT[f.type][1]
        off_s = f"{off}.." if varlen else str(off)
        lines.append(f"| {off_s} | `{f.name}` | {TYPE_DESC[f.type]} | {f.unit} | {f.desc} |")
        off += n
    size = msg.size()
    lines.append("")
    lines.append(f"payload 长度 = **{size if size is not None else '变长'}** 字节")
    return "\n".join(lines) + "\n"


def enum_tables(msg: P.Message) -> str:
    out = []
    for f in msg.fields:
        if not f.enum:
            continue
        out.append(f"\n`{f.name}` 取值：\n")
        out.append("| 值 | 含义 |")
        out.append("|---:|---|")
        for k, v in f.enum.items():
            key = f"0x{k:02X}" if k > 9 else str(k)
            out.append(f"| {key} | {v} |")
        out.append("")
    return "\n".join(out)


def example(msg: P.Message) -> str:
    """给出一个可复制的示例帧。"""
    vals = {}
    for f in msg.fields:
        if f.type == "str":
            vals[f.name] = "hello"
        elif f.type == "u8[]":
            vals[f.name] = [1, 2, 3]
        elif f.type == "f32[]":
            vals[f.name] = [1.5, 2.5, 3.5]
        elif f.type == "f32":
            vals[f.name] = 1.5
        else:
            vals[f.name] = 1
    try:
        b = P.encode(msg.cmd, vals, seq=1)
        t = P.encode_text(msg.cmd, vals).decode().strip()
        return (f"示例（字段全部填 1 / 1.5 / \"hello\"，SEQ=1）：\n\n"
                f"```\n二进制: {b.hex(' ').upper()}\nASCII : {t}\n```\n")
    except Exception:
        return ""


def gen() -> str:
    L = []
    a = L.append
    a(f"# 通信数据格式说明  ·  Protocol v{P.PROTOCOL_VERSION}\n")
    a("> **本文件由 `tools/gen_protocol_doc.py` 从 `upper_computer/protocol.py` 自动生成，请勿手工编辑。**  ")
    a(f"> 生成时间：{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    a("上位机每次更新协议后重新生成本文档，因此这里永远是最新的接入依据。\n")

    a("---\n")
    a("## 0. 物理链路\n")
    a("协议与物理层无关，以下链路均已在上位机支持并共用同一套帧格式：\n")
    a("| 链路 | 说明 | 建议参数 |")
    a("|---|---|---|")
    a("| USB 转 TTL | CH340 / CP2102 / FT232 等 | 115200 ~ 921600 8N1 |")
    a("| 无线数传 | NRF24 / LoRa / 2.4G 透传模块 | 建议 ≤115200，降低发送频率至 20~50Hz |")
    a("| 蓝牙 SPP | HC-05/06、蓝牙串口 | 115200 8N1 |")
    a("| WiFi 透传 | ESP8266/ESP32、USR-WIFI232（TCP Client 模式） | TCP，默认端口 8888 |\n")
    a("> 无线链路会丢包、粘包、乱序拼接，因此帧格式带帧头 + 长度 + CRC16，")
    a("> 下位机务必使用**状态机流式解析**，不要假设一次 DMA/中断刚好收到一整帧。\n")

    a("---\n")
    a("## 1. 帧格式（二进制，默认）\n")
    a("```")
    a("+--------+--------+-------+-------+-------+------------------+-----------+")
    a("| HEAD0  | HEAD1  |  LEN  |  CMD  |  SEQ  |     PAYLOAD      |   CRC16   |")
    a("| 0xAA   | 0x55   | 1 B   | 1 B   | 1 B   |  LEN 字节 (0~240)|   2 B LE  |")
    a("+--------+--------+-------+-------+-------+------------------+-----------+")
    a("```\n")
    a("| 字段 | 长度 | 说明 |")
    a("|---|---|---|")
    a("| HEAD0/HEAD1 | 2 | 固定 `0xAA 0x55` |")
    a("| LEN | 1 | PAYLOAD 字节数，0~240 |")
    a("| CMD | 1 | 命令号，见第 2 节。**bit7=0 上位机→下位机，bit7=1 下位机→上位机** |")
    a("| SEQ | 1 | 帧序号，0~255 循环。用于 ACK 匹配与丢包统计；不关心可忽略 |")
    a("| PAYLOAD | LEN | 命令参数，**全部小端 (little-endian)**，无对齐填充（紧凑排列） |")
    a("| CRC16 | 2 | CRC16/MODBUS，小端。**计算范围 = LEN + CMD + SEQ + PAYLOAD**（不含帧头） |\n")
    a(f"最小帧长 {P.FRAME_OVERHEAD} 字节，最大 {P.FRAME_OVERHEAD + P.MAX_PAYLOAD} 字节。\n")

    a("### CRC16/MODBUS 参考实现（C）\n")
    a("```c")
    a("uint16_t crc16(const uint8_t *d, uint16_t len) {")
    a("    uint16_t crc = 0xFFFF;")
    a("    for (uint16_t i = 0; i < len; i++) {")
    a("        crc ^= d[i];")
    a("        for (uint8_t j = 0; j < 8; j++)")
    a("            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);")
    a("    }")
    a("    return crc;   /* 低字节先发 */")
    a("}")
    a("```\n")

    a("### 接收状态机（C 伪码）\n")
    a("```c")
    a("// 逐字节喂入，收齐一帧回调 on_frame(cmd, seq, payload, len)")
    a("static uint8_t buf[247]; static uint16_t idx = 0;")
    a("void proto_feed(uint8_t b) {")
    a("    if (idx == 0) { if (b == 0xAA) buf[idx++] = b; return; }")
    a("    if (idx == 1) { if (b == 0x55) buf[idx++] = b; else idx = 0; return; }")
    a("    buf[idx++] = b;")
    a("    if (idx == 3 && buf[2] > 240) { idx = 0; return; }      // 长度非法")
    a("    if (idx >= 7 && idx == (uint16_t)buf[2] + 7) {")
    a("        uint16_t rx = buf[idx-2] | (buf[idx-1] << 8);")
    a("        if (crc16(&buf[2], buf[2] + 3) == rx)")
    a("            on_frame(buf[3], buf[4], &buf[5], buf[2]);")
    a("        idx = 0;")
    a("    }")
    a("    if (idx >= sizeof(buf)) idx = 0;")
    a("}")
    a("```\n")

    a("---\n")
    a("## 2. 命令号总表\n")
    a("| CMD | 名称 | 方向 | payload 长度 | 说明 |")
    a("|---|---|---|---|---|")
    for cmd in sorted(P.MESSAGES):
        m = P.MESSAGES[cmd]
        s = m.size()
        a(f"| `0x{cmd:02X}` | **{m.name}** | {m.direction} | "
          f"{s if s is not None else '变长'} | {m.desc.splitlines()[0]} |")
    a("")
    a("命令号分配约定（方便你自行扩展）：\n")
    a("| 区间 | 用途 |")
    a("|---|---|")
    a("| 0x01–0x0F | 系统/运动控制 |")
    a("| 0x10–0x1F | PID 相关 |")
    a("| 0x20–0x2F | 通用参数 |")
    a("| 0x30–0x3F | 遥测配置 |")
    a("| 0x40–0x7E | **预留给你自定义** |")
    a("| 0x7F | 文本透传 |")
    a("| 0x80–0x8F | 应答类 |")
    a("| 0x90–0x9F | 上报类 |")
    a("| 0xA0–0xFE | **预留给你自定义上报** |\n")

    a("---\n")
    a("## 3. 报文详解\n")
    for cmd in sorted(P.MESSAGES):
        m = P.MESSAGES[cmd]
        a(f"### 0x{cmd:02X}  {m.name}   `{m.direction}`\n")
        a(m.desc + "\n")
        a(field_table(m))
        et = enum_tables(m)
        if et:
            a(et)
        if m.notes:
            a(f"> {m.notes}\n")
        a(example(m))
        a("")

    a("---\n")
    a("## 3.5 可订阅通道表（数据流 / IMU）\n")
    a("上位机在「遥测 / 曲线」页勾选通道后，会用 **STREAM_CFG(0x31)** 把通道 ID 列表发给你，")
    a("你之后只需按 **STREAM_DATA(0x93)** 上报这几个通道的值即可，**不用全发**。\n")
    a("值一律用 **float32 小端**，顺序与订阅列表一致。\n")
    for group, chans in P.channel_groups().items():
        a(f"\n**{group}**\n")
        a("| ID | 名称 | 单位 | 说明 |")
        a("|---|---|---|---|")
        for c in chans:
            a(f"| `0x{c.id:02X}` | `{c.name}` | {c.unit} | {c.desc} |")
    a("")
    a("> `0x70–0x7F` 是留给你的自定义通道；要加更多通道，在 `protocol.py` 里")
    a("> 调 `_ch(id, name, unit, desc, group)` 注册一条即可，界面和本文档都会自动出现。\n")
    a("### 下位机实现建议\n")
    a("```c")
    a("// 保存订阅列表")
    a("static uint8_t g_sub[64]; static uint8_t g_sub_n = 0;")
    a("static uint16_t g_period = 20; static uint8_t g_stream_on = 0;")
    a("")
    a("void on_stream_cfg(const uint8_t *p, uint8_t len) {")
    a("    g_stream_on = p[0];")
    a("    g_period    = p[1] | (p[2] << 8);")
    a("    g_sub_n     = p[3] > 60 ? 60 : p[3];")
    a("    for (uint8_t i = 0; i < g_sub_n; i++) g_sub[i] = p[4 + i];")
    a("}")
    a("")
    a("float read_channel(uint8_t id) {   // 按 ID 返回对应的值")
    a("    switch (id) {")
    a("        case 0x03: return imu.yaw;")
    a("        case 0x40: return odom.x;")
    a("        case 0x12: return imu.gyro_z;")
    a("        /* ... 只需实现你关心的通道，其余返回 0 ... */")
    a("        default:   return 0.0f;")
    a("    }")
    a("}")
    a("")
    a("void stream_tick(void) {           // 每 g_period 毫秒调用一次")
    a("    if (!g_stream_on || !g_sub_n) return;")
    a("    uint8_t buf[4 + 60 * 4]; uint16_t n = 0;")
    a("    uint32_t ts = HAL_GetTick();")
    a("    memcpy(buf, &ts, 4); n = 4;")
    a("    for (uint8_t i = 0; i < g_sub_n; i++) {")
    a("        float v = read_channel(g_sub[i]);")
    a("        memcpy(buf + n, &v, 4); n += 4;")
    a("    }")
    a("    send_frame(0x93, buf, n);      // STREAM_DATA")
    a("}")
    a("```\n")

    a("---\n")
    a("## 3.6 PID 参数的两级保存：RAM 与 Flash\n")
    a("调参时反复改参数，如果每次都写 Flash，几千次就把 Flash 擦写寿命耗掉了。")
    a("所以上位机把「改参数」和「保存参数」拆成两个独立动作：\n")
    a("| 动作 | 界面按钮 | 报文 | 效果 |")
    a("|---|---|---|---|")
    a("| 改参数 | ① 写入 RAM | `PID_WRITE(0x11)` target=0 | 立即生效，掉电丢失，**不碰 Flash** |")
    a("| 存参数 | ② 写入 Flash | `PID_SAVE(0x12)` | 把 RAM 当前值固化，掉电不丢 |")
    a("| 反悔 | 放弃改动 | `PID_REVERT(0x14)` | 从 Flash 重新加载回 RAM |")
    a("| 一步到位 | —— | `PID_WRITE(0x11)` target=1 | 写 RAM 同时落 Flash（调试期不建议） |\n")
    a("典型流程：\n")
    a("```")
    a("反复调试（只动 RAM，Flash 不受影响）")
    a(" PC --0x11 PID_WRITE(kp=1.2, target=0)--> MCU   改一下看效果")
    a(" PC --0x11 PID_WRITE(kp=1.5, target=0)--> MCU   再改")
    a(" PC --0x11 PID_WRITE(kp=1.4, target=0)--> MCU   还是 1.4 好")
    a("                                                 ↓ 满意了")
    a(" PC --0x12 PID_SAVE(pid_id)-------------> MCU   固化到 Flash")
    a(" PC <-0x80 ACK(0x12, OK)----------------- MCU")
    a("")
    a("调乱了想反悔：")
    a(" PC --0x14 PID_REVERT(pid_id)-----------> MCU   从 Flash 重新加载")
    a(" PC <-0x90 PID_VALUE(source=0)----------- MCU   回报恢复后的值")
    a("```\n")
    a("上位机还能读回 Flash 里的值和 RAM 做对比（`PID_READ` 的 `source=1`），")
    a("界面上会提示「有未保存改动」还是「RAM 与 Flash 一致」。\n")
    a("### 下位机参考实现\n")
    a("```c")
    a("typedef struct { float kp, ki, kd, i_limit, out_limit; } pid_param_t;")
    a("pid_param_t g_pid[PID_NUM];        // RAM：当前生效")
    a("")
    a("void on_pid_write(const uint8_t *p, uint8_t len) {")
    a("    uint8_t id = p[0];")
    a("    memcpy(&g_pid[id], p + 1, 20);            // kp..out_limit")
    a("    uint8_t target = (len >= 22) ? p[21] : 0; // 老格式无此字段 -> 只写 RAM")
    a("    if (target == 1) flash_save_pid(id);      // 少用，Flash 有寿命")
    a("    send_ack(0x11, seq, 0);")
    a("}")
    a("")
    a("void on_pid_save(uint8_t id) {                // 0x12：调好了才调这里")
    a("    int ok = (id == 0xFF) ? flash_save_all() : flash_save_pid(id);")
    a("    send_ack(0x12, seq, ok ? 0 : 5);          // 5 = ERR_FLASH")
    a("}")
    a("")
    a("void on_pid_revert(uint8_t id) {              // 0x14：一键还原")
    a("    flash_load_pid(id, &g_pid[id]);")
    a("    send_ack(0x14, seq, 0);")
    a("    report_pid_value(id, 0);                  // 主动回报，刷新上位机")
    a("}")
    a("```")
    a("> 开机时记得 `flash_load_all()` 把 Flash 参数载入 RAM，否则重启后是默认值。\n")

    a("---\n")
    a("## 4. ASCII 文本帧（可选模式）\n")
    a("如果你的下位机不方便处理二进制（比如用串口助手手调、或 Arduino 初学阶段），")
    a("可在上位机『帧格式』里切到 **ASCII 文本帧**：\n")
    a("```")
    a("$NAME,arg1,arg2,...*XX\\r\\n")
    a("```")
    a("* `NAME`：第 2 节表中的名称，如 `MOTION`")
    a("* 参数顺序与该报文字段顺序完全一致，整数按十进制，浮点按最短精确表示")
    a("* `XX`：`$` 与 `*` 之间所有字符的**逐字节异或**，2 位大写 HEX\n")
    a("示例：\n")
    a("```")
    a(P.encode_text(P.Cmd.MOTION, {"vx": 500, "vy": 0, "omega": -1200,
                                   "flags": 0, "seq_echo": 0}).decode().strip())
    a(P.encode_text(P.Cmd.PID_WRITE, {"pid_id": 0, "kp": 1.2, "ki": 0.05,
                                      "kd": 0.0, "i_limit": 1000,
                                      "out_limit": 8000}).decode().strip())
    a("```\n")
    a("> 注意：文本帧无 SEQ 字段，长度是二进制帧的 2~3 倍，无线链路下不推荐。\n")

    a("---\n")
    a("## 5. 典型交互时序\n")
    a("### 5.1 键盘遥控（实时模式）\n")
    a("```")
    a("PC                                   MCU")
    a(" |-- 0x04 MODE(mode=1 MANUAL) ------->|")
    a(" |-- 0x02 MOTION(vx,vy,w,flags) ----->|   每 20ms 一帧（50Hz）")
    a(" |-- 0x02 MOTION ...  ---------------->|")
    a(" |-- 0x01 HEARTBEAT ----------------->|   每 500ms")
    a(" |                                    |")
    a(" |   (松开所有键，仍持续发 0 速)        |")
    a(" |-- 0x03 BRAKE(level=2)  ----------->|   按 Esc 急停")
    a("```")
    a("**下位机必须实现通信看门狗**：超过 ~500ms 未收到 MOTION 或 HEARTBEAT，")
    a("立即将输出归零并进入制动，防止无线链路断开后车子飞出去。\n")
    a("### 5.2 PID 在线整定\n")
    a("```")
    a("PC                                   MCU")
    a(" |-- 0x10 PID_READ(id) -------------->|")
    a(" |<-- 0x90 PID_VALUE(id,kp,ki,kd,..) -|   上位机据此填充界面")
    a(" |   (用户在界面上改 Kp)               |")
    a(" |-- 0x11 PID_WRITE(id,kp,ki,kd,..) ->|   实时模式：改一下发一次")
    a(" |<-- 0x80 ACK(0x11,seq,OK) ----------|")
    a(" |-- 0x30 TELEM_CTRL(1,10ms,0x02) --->|   打开曲线上报")
    a(" |-- 0x13 PID_TARGET(id,target) ----->|   阶跃 / 方波激励")
    a(" |<-- 0x92 PID_CURVE(target,fb,out) --|   100Hz 连续上报，上位机绘图")
    a(" |-- 0x12 PID_SAVE(id) -------------->|   调好后固化")
    a(" |<-- 0x80 ACK(0x12,seq,OK) ----------|")
    a("```\n")

    a("---\n")
    a("## 6. 运动学参考（下位机侧）\n")
    a("上位机只发车体系速度 `vx / vy / omega`，具体轮速由下位机解算。麦克纳姆轮示例：\n")
    a("```c")
    a("// vx: mm/s 前正   vy: mm/s 左正   omega: mrad/s 逆时针正")
    a("// L: 半轮距(mm)  W: 半轮距(mm)   k = (L + W)")
    a("float w = omega / 1000.0f;                 // rad/s")
    a("v_lf = vx - vy - k * w;")
    a("v_rf = vx + vy + k * w;")
    a("v_lb = vx + vy - k * w;")
    a("v_rb = vx - vy + k * w;")
    a("```")
    a("差速底盘忽略 `vy` 即可：`v_l = vx - k*w; v_r = vx + k*w;`\n")

    a("---\n")
    a("## 7. 扩展方式\n")
    a("在 `upper_computer/protocol.py` 里加一条：\n")
    a("```python")
    a("MSG_MY_CMD = _reg(Message(")
    a("    Cmd.MY_CMD, \"MY_CMD\", \"PC->MCU\", \"我的自定义命令\",")
    a("    [Field(\"foo\", \"u16\", \"-\", \"说明\"),")
    a("     Field(\"bar\", \"f32\", \"A\", \"电流\")],")
    a("))")
    a("```")
    a("然后 `python tools/gen_protocol_doc.py`，本文档会自动出现新命令，")
    a("上位机的编解码也同时生效，无需改其它代码。\n")
    return "\n".join(L) + "\n"


if __name__ == "__main__":
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(gen())
    print(f"generated: {OUT}")
