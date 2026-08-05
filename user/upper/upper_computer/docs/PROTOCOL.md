# 通信数据格式说明  ·  Protocol v1.1.0

> **本文件由 `tools/gen_protocol_doc.py` 从 `upper_computer/protocol.py` 自动生成，请勿手工编辑。**  
> 生成时间：2026-08-04 13:07:56

上位机每次更新协议后重新生成本文档，因此这里永远是最新的接入依据。

---

## 0. 物理链路

协议与物理层无关，以下链路均已在上位机支持并共用同一套帧格式：

| 链路 | 说明 | 建议参数 |
|---|---|---|
| USB 转 TTL | CH340 / CP2102 / FT232 等 | 115200 ~ 921600 8N1 |
| 无线数传 | NRF24 / LoRa / 2.4G 透传模块 | 建议 ≤115200，降低发送频率至 20~50Hz |
| 蓝牙 SPP | HC-05/06、蓝牙串口 | 115200 8N1 |
| WiFi 透传 | ESP8266/ESP32、USR-WIFI232（TCP Client 模式） | TCP，默认端口 8888 |

> 无线链路会丢包、粘包、乱序拼接，因此帧格式带帧头 + 长度 + CRC16，
> 下位机务必使用**状态机流式解析**，不要假设一次 DMA/中断刚好收到一整帧。

---

## 1. 帧格式（二进制，默认）

```
+--------+--------+-------+-------+-------+------------------+-----------+
| HEAD0  | HEAD1  |  LEN  |  CMD  |  SEQ  |     PAYLOAD      |   CRC16   |
| 0xAA   | 0x55   | 1 B   | 1 B   | 1 B   |  LEN 字节 (0~240)|   2 B LE  |
+--------+--------+-------+-------+-------+------------------+-----------+
```

| 字段 | 长度 | 说明 |
|---|---|---|
| HEAD0/HEAD1 | 2 | 固定 `0xAA 0x55` |
| LEN | 1 | PAYLOAD 字节数，0~240 |
| CMD | 1 | 命令号，见第 2 节。**bit7=0 上位机→下位机，bit7=1 下位机→上位机** |
| SEQ | 1 | 帧序号，0~255 循环。用于 ACK 匹配与丢包统计；不关心可忽略 |
| PAYLOAD | LEN | 命令参数，**全部小端 (little-endian)**，无对齐填充（紧凑排列） |
| CRC16 | 2 | CRC16/MODBUS，小端。**计算范围 = LEN + CMD + SEQ + PAYLOAD**（不含帧头） |

最小帧长 7 字节，最大 247 字节。

### CRC16/MODBUS 参考实现（C）

```c
uint16_t crc16(const uint8_t *d, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= d[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
    return crc;   /* 低字节先发 */
}
```

### 接收状态机（C 伪码）

```c
// 逐字节喂入，收齐一帧回调 on_frame(cmd, seq, payload, len)
static uint8_t buf[247]; static uint16_t idx = 0;
void proto_feed(uint8_t b) {
    if (idx == 0) { if (b == 0xAA) buf[idx++] = b; return; }
    if (idx == 1) { if (b == 0x55) buf[idx++] = b; else idx = 0; return; }
    buf[idx++] = b;
    if (idx == 3 && buf[2] > 240) { idx = 0; return; }      // 长度非法
    if (idx >= 7 && idx == (uint16_t)buf[2] + 7) {
        uint16_t rx = buf[idx-2] | (buf[idx-1] << 8);
        if (crc16(&buf[2], buf[2] + 3) == rx)
            on_frame(buf[3], buf[4], &buf[5], buf[2]);
        idx = 0;
    }
    if (idx >= sizeof(buf)) idx = 0;
}
```

---

## 2. 命令号总表

| CMD | 名称 | 方向 | payload 长度 | 说明 |
|---|---|---|---|---|
| `0x01` | **HEARTBEAT** | PC->MCU | 4 | 链路保活。上位机按固定周期（默认 500ms）发送；下位机若超时（建议 1s）未收到，应自动进入制动/失能状态，防止无线链路断开后失控。 |
| `0x02` | **MOTION** | PC->MCU | 8 | 运动控制指令。键盘映射与摇杆均输出此帧。实时模式下按发送周期（默认 20ms/50Hz）连续发送；配置模式下点击『发送』时发一帧。 |
| `0x03` | **BRAKE** | PC->MCU | 1 | 制动 / 急停。空格键触发。level=0 缓停(速度归零)，1 抱死，2 急停（切断输出，需要重新使能）。 |
| `0x04` | **MODE** | PC->MCU | 1 | 控制模式切换。 |
| `0x10` | **PID_READ** | PC->MCU | 1 | 请求读取指定 PID 参数。下位机收到后应回 PID_VALUE(0x90)。pid_id=0xFF 表示读取全部（逐条回）。 |
| `0x11` | **PID_WRITE** | PC->MCU | 21 | 写入 PID 参数（仅内存生效，掉电丢失；需要固化调用 PID_SAVE）。 |
| `0x12` | **PID_SAVE** | PC->MCU | 1 | 将当前 PID 参数写入 Flash/EEPROM。 |
| `0x13` | **PID_TARGET** | PC->MCU | 5 | 给指定环下发目标值（阶跃/方波调试用）。 |
| `0x20` | **PARAM_READ** | PC->MCU | 2 | 通用参数读取（预留扩展：轮距、轮径、限速、遥控死区等）。 |
| `0x21` | **PARAM_WRITE** | PC->MCU | 6 | 通用参数写入。value 统一按 f32 传输，下位机自行转换类型。 |
| `0x30` | **TELEM_CTRL** | PC->MCU | 4 | 遥测上报控制。 |
| `0x31` | **STREAM_CFG** | PC->MCU | 变长 | 订阅数据流：告诉下位机『我只要这几个通道』。上位机在曲线界面勾选后自动下发。下位机之后按 period_ms 周期，用 STREAM_DATA(0x93) 上报，**且只发订阅列表里的通道，顺序与本帧 channels 完全一致**，不用全发。 |
| `0x32` | **STREAM_LIST** | PC->MCU | 0 | 询问下位机支持哪些通道（可选实现）。下位机回一条或多条 STREAM_INFO(0x94)。不实现也没关系，上位机会用内置的标准通道表。 |
| `0x7F` | **RAW_TEXT** | PC->MCU | 变长 | 透传自定义字符串命令，方便临时加调试指令而不改协议。 |
| `0x80` | **ACK** | MCU->PC | 3 | 通用应答。建议对所有写类命令（PID_WRITE/PARAM_WRITE/MODE 等）回应答。 |
| `0x90` | **PID_VALUE** | MCU->PC | 21 | PID 当前值回报。上位机据此刷新界面上『当前读取到的 PID』。 |
| `0x91` | **TELEMETRY** | MCU->PC | 16 | 周期遥测，用于状态栏与曲线显示。 |
| `0x92` | **PID_CURVE** | MCU->PC | 17 | PID 调试曲线数据，上位机实时绘图（目标 vs 实际）。 |
| `0x93` | **STREAM_DATA** | MCU->PC | 变长 | 按订阅上报的通道数据。**只包含 STREAM_CFG 里订阅的通道，顺序完全一致**，不带通道 ID（省带宽），上位机按订阅顺序对号入座。 |
| `0x94` | **STREAM_INFO** | MCU->PC | 变长 | 通道清单应答：下位机自报支持的通道，让上位机动态显示。可选实现。 |
| `0x9F` | **LOG** | MCU->PC | 变长 | 下位机文本日志，直接打印到上位机控制台。 |

命令号分配约定（方便你自行扩展）：

| 区间 | 用途 |
|---|---|
| 0x01–0x0F | 系统/运动控制 |
| 0x10–0x1F | PID 相关 |
| 0x20–0x2F | 通用参数 |
| 0x30–0x3F | 遥测配置 |
| 0x40–0x7E | **预留给你自定义** |
| 0x7F | 文本透传 |
| 0x80–0x8F | 应答类 |
| 0x90–0x9F | 上报类 |
| 0xA0–0xFE | **预留给你自定义上报** |

---

## 3. 报文详解

### 0x01  HEARTBEAT   `PC->MCU`

链路保活。上位机按固定周期（默认 500ms）发送；下位机若超时（建议 1s）未收到，应自动进入制动/失能状态，防止无线链路断开后失控。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `timestamp_ms` | uint32 (小端) | ms | 上位机单调时间戳 |

payload 长度 = **4** 字节

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 04 01 01 01 00 00 00 63 2D
ASCII : $HEARTBEAT,1*45
```


### 0x02  MOTION   `PC->MCU`

运动控制指令。键盘映射与摇杆均输出此帧。实时模式下按发送周期（默认 20ms/50Hz）连续发送；配置模式下点击『发送』时发一帧。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `vx` | int16 (小端) | mm/s | 前后速度，前为正（W 前进 / S 后退） |
| 2 | `vy` | int16 (小端) | mm/s | 左右平移速度，左为正（A 左移 / D 右移） |
| 4 | `omega` | int16 (小端) | mrad/s | 旋转角速度，逆时针为正（Q 逆时针 / E 顺时针） |
| 6 | `flags` | uint8 | - | 标志位，见下表 |
| 7 | `seq_echo` | uint8 | - | 预留，回显用，当前固定 0 |

payload 长度 = **8** 字节


`flags` 取值：

| 值 | 含义 |
|---:|---|
| 1 | BRAKE  制动（忽略速度分量，抱死） |
| 2 | BOOST  加速档（按住 Shift） |
| 4 | SLOW   慢速档（按住 Ctrl） |
| 8 | FIELD  场地坐标系（否则为车体坐标系） |

> 三个分量可同时非零（如 W+A 斜走、W+E 边走边转）。下位机应对合速度做限幅，不要相信上位机不越界。

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 08 02 01 01 00 01 00 01 00 01 01 81 6F
ASCII : $MOTION,1,1,1,1,1*03
```


### 0x03  BRAKE   `PC->MCU`

制动 / 急停。空格键触发。level=0 缓停(速度归零)，1 抱死，2 急停（切断输出，需要重新使能）。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `level` | uint8 | - | 0=缓停 1=抱死 2=急停 |

payload 长度 = **1** 字节

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 01 03 01 01 31 88
ASCII : $BRAKE,1*42
```


### 0x04  MODE   `PC->MCU`

控制模式切换。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `mode` | uint8 | - | 模式枚举 |

payload 长度 = **1** 字节


`mode` 取值：

| 值 | 含义 |
|---:|---|
| 0 | IDLE      空闲/失能 |
| 1 | MANUAL    手动（键盘映射） |
| 2 | AUTO      自动/自主 |
| 3 | PID_TUNE  PID 调试（允许阶跃输入） |

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 01 04 01 01 80 49
ASCII : $MODE,1*1E
```


### 0x10  PID_READ   `PC->MCU`

请求读取指定 PID 参数。下位机收到后应回 PID_VALUE(0x90)。pid_id=0xFF 表示读取全部（逐条回）。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `pid_id` | uint8 | - | PID 通道号 |

payload 长度 = **1** 字节


`pid_id` 取值：

| 值 | 含义 |
|---:|---|
| 0 | CHASSIS_LF   左前轮速度环 |
| 1 | CHASSIS_RF   右前轮速度环 |
| 2 | CHASSIS_LB   左后轮速度环 |
| 3 | CHASSIS_RB   右后轮速度环 |
| 0x10 | YAW_ANGLE  云台/底盘 偏航角度环 |
| 0x11 | YAW_SPEED  偏航角速度环 |
| 0x20 | USER_0     用户自定义 0 |
| 0x21 | USER_1     用户自定义 1 |

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 01 10 01 01 C0 4D
ASCII : $PID_READ,1*0D
```


### 0x11  PID_WRITE   `PC->MCU`

写入 PID 参数（仅内存生效，掉电丢失；需要固化调用 PID_SAVE）。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `pid_id` | uint8 | - | PID 通道号 |
| 1 | `kp` | float32 IEEE754 (小端) | - | 比例系数 |
| 5 | `ki` | float32 IEEE754 (小端) | - | 积分系数 |
| 9 | `kd` | float32 IEEE754 (小端) | - | 微分系数 |
| 13 | `i_limit` | float32 IEEE754 (小端) | - | 积分限幅（0 表示不限） |
| 17 | `out_limit` | float32 IEEE754 (小端) | - | 输出限幅（0 表示不限） |

payload 长度 = **21** 字节


`pid_id` 取值：

| 值 | 含义 |
|---:|---|
| 0 | CHASSIS_LF   左前轮速度环 |
| 1 | CHASSIS_RF   右前轮速度环 |
| 2 | CHASSIS_LB   左后轮速度环 |
| 3 | CHASSIS_RB   右后轮速度环 |
| 0x10 | YAW_ANGLE  云台/底盘 偏航角度环 |
| 0x11 | YAW_SPEED  偏航角速度环 |
| 0x20 | USER_0     用户自定义 0 |
| 0x21 | USER_1     用户自定义 1 |

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 15 11 01 01 00 00 C0 3F 00 00 C0 3F 00 00 C0 3F 00 00 C0 3F 00 00 C0 3F D5 88
ASCII : $PID_WRITE,1,1.5,1.5,1.5,1.5,1.5*44
```


### 0x12  PID_SAVE   `PC->MCU`

将当前 PID 参数写入 Flash/EEPROM。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `pid_id` | uint8 | - | 通道号，0xFF = 全部 |

payload 长度 = **1** 字节


`pid_id` 取值：

| 值 | 含义 |
|---:|---|
| 0 | CHASSIS_LF   左前轮速度环 |
| 1 | CHASSIS_RF   右前轮速度环 |
| 2 | CHASSIS_LB   左后轮速度环 |
| 3 | CHASSIS_RB   右后轮速度环 |
| 0x10 | YAW_ANGLE  云台/底盘 偏航角度环 |
| 0x11 | YAW_SPEED  偏航角速度环 |
| 0x20 | USER_0     用户自定义 0 |
| 0x21 | USER_1     用户自定义 1 |

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 01 12 01 01 61 8D
ASCII : $PID_SAVE,1*1E
```


### 0x13  PID_TARGET   `PC->MCU`

给指定环下发目标值（阶跃/方波调试用）。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `pid_id` | uint8 | - | 通道号 |
| 1 | `target` | float32 IEEE754 (小端) | - | 目标值，单位由该环自身定义 |

payload 长度 = **5** 字节


`pid_id` 取值：

| 值 | 含义 |
|---:|---|
| 0 | CHASSIS_LF   左前轮速度环 |
| 1 | CHASSIS_RF   右前轮速度环 |
| 2 | CHASSIS_LB   左后轮速度环 |
| 3 | CHASSIS_RB   右后轮速度环 |
| 0x10 | YAW_ANGLE  云台/底盘 偏航角度环 |
| 0x11 | YAW_SPEED  偏航角速度环 |
| 0x20 | USER_0     用户自定义 0 |
| 0x21 | USER_1     用户自定义 1 |

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 05 13 01 01 00 00 C0 3F 8E F4
ASCII : $PID_TARGET,1,1.5*08
```


### 0x20  PARAM_READ   `PC->MCU`

通用参数读取（预留扩展：轮距、轮径、限速、遥控死区等）。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `param_id` | uint16 (小端) | - | 参数 ID，由用户自行约定 |

payload 长度 = **2** 字节

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 02 20 01 01 00 06 50
ASCII : $PARAM_READ,1*1F
```


### 0x21  PARAM_WRITE   `PC->MCU`

通用参数写入。value 统一按 f32 传输，下位机自行转换类型。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `param_id` | uint16 (小端) | - | 参数 ID |
| 2 | `value` | float32 IEEE754 (小端) | - | 参数值 |

payload 长度 = **6** 字节

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 06 21 01 01 00 00 00 C0 3F E3 85
ASCII : $PARAM_WRITE,1,1.5*56
```


### 0x30  TELEM_CTRL   `PC->MCU`

遥测上报控制。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `enable` | uint8 | - | 0=停止上报 1=开启上报 |
| 1 | `period_ms` | uint16 (小端) | ms | 上报周期，建议 >= 10ms |
| 3 | `mask` | uint8 | - | bit0=TELEMETRY bit1=PID_CURVE |

payload 长度 = **4** 字节

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 04 30 01 01 01 00 01 F7 0C
ASCII : $TELEM_CTRL,1,1,1*1E
```


### 0x31  STREAM_CFG   `PC->MCU`

订阅数据流：告诉下位机『我只要这几个通道』。上位机在曲线界面勾选后自动下发。下位机之后按 period_ms 周期，用 STREAM_DATA(0x93) 上报，**且只发订阅列表里的通道，顺序与本帧 channels 完全一致**，不用全发。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `enable` | uint8 | - | 0=停止数据流 1=开始 |
| 1 | `period_ms` | uint16 (小端) | ms | 上报周期，建议 10~100ms；无线链路别低于 20ms |
| 3 | `count` | uint8 | - | 订阅通道数量，0~60 |
| 4.. | `channels` | uint8 数组（变长，占满剩余 payload） | - | 通道 ID 列表，长度 = count，见『可订阅通道表』 |

payload 长度 = **变长** 字节

> 举例：只想看 yaw 和位移，就订阅 [0x03, 0x40, 0x41]，之后每帧 STREAM_DATA 只有 3 个 f32，共 12 字节 payload，非常省带宽。重新勾选会重发本帧覆盖旧订阅。

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 07 31 01 01 01 00 01 01 02 03 59 CC
ASCII : $STREAM_CFG,1,1,1,[1, 2, 3]*06
```


### 0x32  STREAM_LIST   `PC->MCU`

询问下位机支持哪些通道（可选实现）。下位机回一条或多条 STREAM_INFO(0x94)。不实现也没关系，上位机会用内置的标准通道表。

_无 payload（LEN = 0）_

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 00 32 01 A5 60
ASCII : $STREAM_LIST*41
```


### 0x7F  RAW_TEXT   `PC->MCU`

透传自定义字符串命令，方便临时加调试指令而不改协议。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0.. | `text` | UTF-8 字符串（变长，占满剩余 payload） | - | UTF-8 字符串，不含结束符 |

payload 长度 = **变长** 字节

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 05 7F 01 68 65 6C 6C 6F DD 16
ASCII : $RAW_TEXT,hello*48
```


### 0x80  ACK   `MCU->PC`

通用应答。建议对所有写类命令（PID_WRITE/PARAM_WRITE/MODE 等）回应答。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `ack_cmd` | uint8 | - | 被应答的命令号 |
| 1 | `ack_seq` | uint8 | - | 被应答帧的 SEQ |
| 2 | `status` | uint8 | - | 状态码 |

payload 长度 = **3** 字节


`status` 取值：

| 值 | 含义 |
|---:|---|
| 0 | OK        执行成功 |
| 1 | ERR_CRC   校验错误 |
| 2 | ERR_CMD   未知命令 |
| 3 | ERR_ARG   参数非法 |
| 4 | ERR_BUSY  设备忙 |

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 03 80 01 01 01 01 91 9A
ASCII : $ACK,1,1,1*54
```


### 0x90  PID_VALUE   `MCU->PC`

PID 当前值回报。上位机据此刷新界面上『当前读取到的 PID』。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `pid_id` | uint8 | - | 通道号 |
| 1 | `kp` | float32 IEEE754 (小端) | - | 比例 |
| 5 | `ki` | float32 IEEE754 (小端) | - | 积分 |
| 9 | `kd` | float32 IEEE754 (小端) | - | 微分 |
| 13 | `i_limit` | float32 IEEE754 (小端) | - | 积分限幅 |
| 17 | `out_limit` | float32 IEEE754 (小端) | - | 输出限幅 |

payload 长度 = **21** 字节


`pid_id` 取值：

| 值 | 含义 |
|---:|---|
| 0 | CHASSIS_LF   左前轮速度环 |
| 1 | CHASSIS_RF   右前轮速度环 |
| 2 | CHASSIS_LB   左后轮速度环 |
| 3 | CHASSIS_RB   右后轮速度环 |
| 0x10 | YAW_ANGLE  云台/底盘 偏航角度环 |
| 0x11 | YAW_SPEED  偏航角速度环 |
| 0x20 | USER_0     用户自定义 0 |
| 0x21 | USER_1     用户自定义 1 |

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 15 90 01 01 00 00 C0 3F 00 00 C0 3F 00 00 C0 3F 00 00 C0 3F 00 00 C0 3F 70 72
ASCII : $PID_VALUE,1,1.5,1.5,1.5,1.5,1.5*52
```


### 0x91  TELEMETRY   `MCU->PC`

周期遥测，用于状态栏与曲线显示。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `timestamp_ms` | uint32 (小端) | ms | 下位机时间戳 |
| 4 | `vx` | int16 (小端) | mm/s | 实际前后速度 |
| 6 | `vy` | int16 (小端) | mm/s | 实际平移速度 |
| 8 | `omega` | int16 (小端) | mrad/s | 实际角速度 |
| 10 | `yaw` | int16 (小端) | 0.01deg | 航向角 |
| 12 | `battery_mv` | uint16 (小端) | mV | 电池电压 |
| 14 | `state` | uint8 | - | 0=IDLE 1=RUN 2=ERROR |
| 15 | `err_code` | uint8 | - | 错误码，用户自定义 |

payload 长度 = **16** 字节

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 10 91 01 01 00 00 00 01 00 01 00 01 00 01 00 01 00 01 01 A1 81
ASCII : $TELEMETRY,1,1,1,1,1,1,1,1*4F
```


### 0x92  PID_CURVE   `MCU->PC`

PID 调试曲线数据，上位机实时绘图（目标 vs 实际）。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `pid_id` | uint8 | - | 通道号 |
| 1 | `timestamp_ms` | uint32 (小端) | ms | 下位机时间戳 |
| 5 | `target` | float32 IEEE754 (小端) | - | 目标值 |
| 9 | `feedback` | float32 IEEE754 (小端) | - | 反馈值 |
| 13 | `output` | float32 IEEE754 (小端) | - | PID 输出 |

payload 长度 = **17** 字节


`pid_id` 取值：

| 值 | 含义 |
|---:|---|
| 0 | CHASSIS_LF   左前轮速度环 |
| 1 | CHASSIS_RF   右前轮速度环 |
| 2 | CHASSIS_LB   左后轮速度环 |
| 3 | CHASSIS_RB   右后轮速度环 |
| 0x10 | YAW_ANGLE  云台/底盘 偏航角度环 |
| 0x11 | YAW_SPEED  偏航角速度环 |
| 0x20 | USER_0     用户自定义 0 |
| 0x21 | USER_1     用户自定义 1 |

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 11 92 01 01 01 00 00 00 00 00 C0 3F 00 00 C0 3F 00 00 C0 3F 8B D8
ASCII : $PID_CURVE,1,1,1.5,1.5,1.5*53
```


### 0x93  STREAM_DATA   `MCU->PC`

按订阅上报的通道数据。**只包含 STREAM_CFG 里订阅的通道，顺序完全一致**，不带通道 ID（省带宽），上位机按订阅顺序对号入座。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `timestamp_ms` | uint32 (小端) | ms | 下位机时间戳，用于曲线横轴 |
| 4.. | `values` | float32 数组（变长，小端，占满剩余 payload） | - | 各通道值，数量 = 订阅的 count，顺序同订阅列表 |

payload 长度 = **变长** 字节

> payload 长度 = 4 + 4*count。若上位机收到的数量与当前订阅不符，会忽略该帧（说明订阅刚变更，下一帧就对上了）。

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 10 93 01 01 00 00 00 00 00 C0 3F 00 00 20 40 00 00 60 40 92 1C
ASCII : $STREAM_DATA,1,[1.5, 2.5, 3.5]*4F
```


### 0x94  STREAM_INFO   `MCU->PC`

通道清单应答：下位机自报支持的通道，让上位机动态显示。可选实现。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `channel_id` | uint8 | - | 通道 ID |
| 1 | `unit` | uint8 | - | 单位枚举（0=无 1=deg 2=deg/s 3=m/s^2 4=m 5=mm/s 6=V 7=A 8=rpm 9=自定义） |
| 2.. | `name` | UTF-8 字符串（变长，占满剩余 payload） | - | 通道名称，UTF-8 |

payload 长度 = **变长** 字节

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 07 94 01 01 01 68 65 6C 6C 6F 5D F2
ASCII : $STREAM_INFO,1,1,hello*03
```


### 0x9F  LOG   `MCU->PC`

下位机文本日志，直接打印到上位机控制台。

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | `level` | uint8 | - | 0=DEBUG 1=INFO 2=WARN 3=ERROR |
| 1.. | `text` | UTF-8 字符串（变长，占满剩余 payload） | - | UTF-8 日志正文 |

payload 长度 = **变长** 字节

示例（字段全部填 1 / 1.5 / "hello"，SEQ=1）：

```
二进制: AA 55 06 9F 01 01 68 65 6C 6C 6F AB 59
ASCII : $LOG,1,hello*17
```


---

## 3.5 可订阅通道表（数据流 / IMU）

上位机在「遥测 / 曲线」页勾选通道后，会用 **STREAM_CFG(0x31)** 把通道 ID 列表发给你，
你之后只需按 **STREAM_DATA(0x93)** 上报这几个通道的值即可，**不用全发**。

值一律用 **float32 小端**，顺序与订阅列表一致。


**IMU 姿态**

| ID | 名称 | 单位 | 说明 |
|---|---|---|---|
| `0x01` | `imu_roll` | deg | 横滚角 |
| `0x02` | `imu_pitch` | deg | 俯仰角 |
| `0x03` | `imu_yaw` | deg | 偏航角（航向） |
| `0x04` | `imu_quat_w` | - | 四元数 w |
| `0x05` | `imu_quat_x` | - | 四元数 x |
| `0x06` | `imu_quat_y` | - | 四元数 y |
| `0x07` | `imu_quat_z` | - | 四元数 z |

**IMU 陀螺仪**

| ID | 名称 | 单位 | 说明 |
|---|---|---|---|
| `0x10` | `gyro_x` | deg/s | 陀螺仪 X 轴角速度 |
| `0x11` | `gyro_y` | deg/s | 陀螺仪 Y 轴角速度 |
| `0x12` | `gyro_z` | deg/s | 陀螺仪 Z 轴角速度 |

**IMU 加速度计**

| ID | 名称 | 单位 | 说明 |
|---|---|---|---|
| `0x20` | `accel_x` | m/s^2 | 加速度 X |
| `0x21` | `accel_y` | m/s^2 | 加速度 Y |
| `0x22` | `accel_z` | m/s^2 | 加速度 Z |

**IMU 磁力计**

| ID | 名称 | 单位 | 说明 |
|---|---|---|---|
| `0x30` | `mag_x` | uT | 磁力计 X |
| `0x31` | `mag_y` | uT | 磁力计 Y |
| `0x32` | `mag_z` | uT | 磁力计 Z |
| `0x33` | `imu_temp` | degC | IMU 温度 |

**里程计**

| ID | 名称 | 单位 | 说明 |
|---|---|---|---|
| `0x40` | `odom_x` | m | 位移 X（累计） |
| `0x41` | `odom_y` | m | 位移 Y（累计） |
| `0x42` | `odom_theta` | deg | 航向（里程计推算） |
| `0x43` | `odom_dist` | m | 累计行驶距离 |
| `0x44` | `vel_x` | mm/s | 实际速度 vx |
| `0x45` | `vel_y` | mm/s | 实际速度 vy |
| `0x46` | `vel_omega` | mrad/s | 实际角速度 |

**轮子**

| ID | 名称 | 单位 | 说明 |
|---|---|---|---|
| `0x50` | `wheel_lf_spd` | rpm | 左前轮实际转速 |
| `0x51` | `wheel_rf_spd` | rpm | 右前轮实际转速 |
| `0x52` | `wheel_lb_spd` | rpm | 左后轮实际转速 |
| `0x53` | `wheel_rb_spd` | rpm | 右后轮实际转速 |
| `0x54` | `wheel_lf_set` | rpm | 左前轮目标转速 |
| `0x55` | `wheel_rf_set` | rpm | 右前轮目标转速 |
| `0x56` | `wheel_lb_set` | rpm | 左后轮目标转速 |
| `0x57` | `wheel_rb_set` | rpm | 右后轮目标转速 |

**电源/系统**

| ID | 名称 | 单位 | 说明 |
|---|---|---|---|
| `0x60` | `battery_v` | V | 电池电压 |
| `0x61` | `battery_a` | A | 总电流 |
| `0x62` | `cpu_load` | % | CPU 占用 |
| `0x63` | `loop_dt` | ms | 主控制周期耗时 |

**用户自定义**

| ID | 名称 | 单位 | 说明 |
|---|---|---|---|
| `0x70` | `user_0` | - | 用户自定义通道 0 |
| `0x71` | `user_1` | - | 用户自定义通道 1 |
| `0x72` | `user_2` | - | 用户自定义通道 2 |
| `0x73` | `user_3` | - | 用户自定义通道 3 |
| `0x74` | `user_4` | - | 用户自定义通道 4 |
| `0x75` | `user_5` | - | 用户自定义通道 5 |
| `0x76` | `user_6` | - | 用户自定义通道 6 |
| `0x77` | `user_7` | - | 用户自定义通道 7 |

> `0x70–0x7F` 是留给你的自定义通道；要加更多通道，在 `protocol.py` 里
> 调 `_ch(id, name, unit, desc, group)` 注册一条即可，界面和本文档都会自动出现。

### 下位机实现建议

```c
// 保存订阅列表
static uint8_t g_sub[64]; static uint8_t g_sub_n = 0;
static uint16_t g_period = 20; static uint8_t g_stream_on = 0;

void on_stream_cfg(const uint8_t *p, uint8_t len) {
    g_stream_on = p[0];
    g_period    = p[1] | (p[2] << 8);
    g_sub_n     = p[3] > 60 ? 60 : p[3];
    for (uint8_t i = 0; i < g_sub_n; i++) g_sub[i] = p[4 + i];
}

float read_channel(uint8_t id) {   // 按 ID 返回对应的值
    switch (id) {
        case 0x03: return imu.yaw;
        case 0x40: return odom.x;
        case 0x12: return imu.gyro_z;
        /* ... 只需实现你关心的通道，其余返回 0 ... */
        default:   return 0.0f;
    }
}

void stream_tick(void) {           // 每 g_period 毫秒调用一次
    if (!g_stream_on || !g_sub_n) return;
    uint8_t buf[4 + 60 * 4]; uint16_t n = 0;
    uint32_t ts = HAL_GetTick();
    memcpy(buf, &ts, 4); n = 4;
    for (uint8_t i = 0; i < g_sub_n; i++) {
        float v = read_channel(g_sub[i]);
        memcpy(buf + n, &v, 4); n += 4;
    }
    send_frame(0x93, buf, n);      // STREAM_DATA
}
```

---

## 4. ASCII 文本帧（可选模式）

如果你的下位机不方便处理二进制（比如用串口助手手调、或 Arduino 初学阶段），
可在上位机『帧格式』里切到 **ASCII 文本帧**：

```
$NAME,arg1,arg2,...*XX\r\n
```
* `NAME`：第 2 节表中的名称，如 `MOTION`
* 参数顺序与该报文字段顺序完全一致，整数按十进制，浮点按最短精确表示
* `XX`：`$` 与 `*` 之间所有字符的**逐字节异或**，2 位大写 HEX

示例：

```
$MOTION,500,0,-1200,0,0*19
$PID_WRITE,0,1.2,0.05,0,1000,8000*60
```

> 注意：文本帧无 SEQ 字段，长度是二进制帧的 2~3 倍，无线链路下不推荐。

---

## 5. 典型交互时序

### 5.1 键盘遥控（实时模式）

```
PC                                   MCU
 |-- 0x04 MODE(mode=1 MANUAL) ------->|
 |-- 0x02 MOTION(vx,vy,w,flags) ----->|   每 20ms 一帧（50Hz）
 |-- 0x02 MOTION ...  ---------------->|
 |-- 0x01 HEARTBEAT ----------------->|   每 500ms
 |                                    |
 |   (松开所有键，仍持续发 0 速)        |
 |-- 0x03 BRAKE(level=2)  ----------->|   按 Esc 急停
```
**下位机必须实现通信看门狗**：超过 ~500ms 未收到 MOTION 或 HEARTBEAT，
立即将输出归零并进入制动，防止无线链路断开后车子飞出去。

### 5.2 PID 在线整定

```
PC                                   MCU
 |-- 0x10 PID_READ(id) -------------->|
 |<-- 0x90 PID_VALUE(id,kp,ki,kd,..) -|   上位机据此填充界面
 |   (用户在界面上改 Kp)               |
 |-- 0x11 PID_WRITE(id,kp,ki,kd,..) ->|   实时模式：改一下发一次
 |<-- 0x80 ACK(0x11,seq,OK) ----------|
 |-- 0x30 TELEM_CTRL(1,10ms,0x02) --->|   打开曲线上报
 |-- 0x13 PID_TARGET(id,target) ----->|   阶跃 / 方波激励
 |<-- 0x92 PID_CURVE(target,fb,out) --|   100Hz 连续上报，上位机绘图
 |-- 0x12 PID_SAVE(id) -------------->|   调好后固化
 |<-- 0x80 ACK(0x12,seq,OK) ----------|
```

---

## 6. 运动学参考（下位机侧）

上位机只发车体系速度 `vx / vy / omega`，具体轮速由下位机解算。麦克纳姆轮示例：

```c
// vx: mm/s 前正   vy: mm/s 左正   omega: mrad/s 逆时针正
// L: 半轮距(mm)  W: 半轮距(mm)   k = (L + W)
float w = omega / 1000.0f;                 // rad/s
v_lf = vx - vy - k * w;
v_rf = vx + vy + k * w;
v_lb = vx + vy - k * w;
v_rb = vx - vy + k * w;
```
差速底盘忽略 `vy` 即可：`v_l = vx - k*w; v_r = vx + k*w;`

---

## 7. 扩展方式

在 `upper_computer/protocol.py` 里加一条：

```python
MSG_MY_CMD = _reg(Message(
    Cmd.MY_CMD, "MY_CMD", "PC->MCU", "我的自定义命令",
    [Field("foo", "u16", "-", "说明"),
     Field("bar", "f32", "A", "电流")],
))
```
然后 `python tools/gen_protocol_doc.py`，本文档会自动出现新命令，
上位机的编解码也同时生效，无需改其它代码。

