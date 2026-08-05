# user/upper —— 上位机调试协议（对接 upper_computer）

本目录对接 [upper_computer](https://github.com/shihangliu0-rgb/upper-computer)
上位机仓库（`arena/019fc198-upper-computer` 分支），在 H723 下位机侧实现其
`docs/PROTOCOL.md`（v1.1.0）定义的二进制通信协议，替代原来的 `user/com_link`。

> **协议一句话**：帧 `AA 55 | LEN | CMD | SEQ | PAYLOAD | CRC16(LE)`，
> CRC16/MODBUS 校验范围 = `LEN+CMD+SEQ+PAYLOAD`，CMD bit7 区分方向。
> 上位机发 MOTION/HEARTBEAT/PID/STREAM 等命令，下位机回 ACK/TELEMETRY/STREAM_DATA 等。

## 目录结构

```
user/upper/
├── README.md              本文件
├── upper_config.h         ★ 总开关 UPPER_DEBUG + 可调参数（改这一个文件即可开关）
├── upper_protocol.h       命令号/通道号/对外 API（关闭态自动退化为空实现）
├── upper_protocol.c       协议实现（整文件被 UPPER_DEBUG 包裹）
└── upper_computer/        上位机 PC 程序（PyQt5）+ docs/PROTOCOL.md（协议规范）
    ├── docs/PROTOCOL.md     ← 协议唯一事实来源，接入必看
    ├── upper_computer/      Python 包
    ├── tools/simulator.py   无硬件模拟器
    └── ...
```

## ★ 一键开关：UPPER_DEBUG

所有功能由 `upper_config.h` 里的一个宏控制：

```c
/* upper_config.h */
/* #define UPPER_DEBUG */      ← 默认注释掉 = 关闭
```

- **未定义（默认）**：`upper_protocol.c` 编译为空，`Upper_*` 全部退化为空函数，
  对现有 H723 固件 **零影响**——不占用串口、不改变任何行为。
- **开启**：取消注释（或在 Keil `Options → C/C++ → Define` 里加 `UPPER_DEBUG`），
  即启用完整上位机调试协议。

也可以不改文件，直接在 Keil 工程的宏定义里加 `UPPER_DEBUG`，避免改源码。

## 接线（需要你确认的 3 处改动，均在现有代码里）

模块本身自包含、不修改 `usart.c`。串口沿用 H7 既有配置，默认用 `huart4`（UART4 @115200，
与原 `computer_link` 一致）。

> 形态与原 `computer_link` 完全一致，**把 `ComputerLink_*` 改名为 `Upper_*` 即可**。

1. **`Core/Src/freertos.c`** —— `StartCommTask`（原 `ComputerLink_*`）：
   ```c
   #include "upper_protocol.h"          // 替换 #include "computer_link.h"
   ...
   (void)Upper_Init(&huart4);           // 替换 ComputerLink_Init
   Upper_Run();                         // 替换 ComputerLink_Run
   ```
2. **`Core/Src/main.c`** —— HAL 回调（原 `ComputerLink_RxCplt/Error`）：
   ```c
   #include "upper_protocol.h"
   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  { Upper_RxCplt(huart); }
   void HAL_UART_ErrorCallback (UART_HandleTypeDef *huart)  { Upper_Error(huart); }
   ```
3. **`MDK-ARM/b-up.uvprojx`** —— Keil 工程树里：
   - 移除已删除的 `user/com_link/computer_link.c(.h)`；
   - 加入 `user/upper/upper_protocol.c(.h)`；
   - Include 路径加 `..\user\upper`。

> 这些属于基础文件改动，**按你要求留给你决定**，我没有擅自修改 `freertos.c / main.c / usart.c`。

## 默认行为（开启 UPPER_DEBUG 后）

- **MOTION(0x02)** → 直接 `Chassis_SetVelocity(vx, vy, omega)`；带 BRAKE 标志或收
  **BRAKE(0x03)** → `Chassis_StopAll()`（与原 computer_link 行为一致，开箱可用）。
- **HEARTBEAT(0x01)** 刷新链路在线时间戳。
- **看门狗**：超过 `UPPER_WATCHDOG_MS`（默认 1000ms）未收到任何帧 → `Chassis_StopAll()`。
- **TELEM_CTRL(0x30) / STREAM_CFG(0x31)**：在线开/关周期遥测与按需数据流。
- **PID_READ/WRITE/SAVE/TARGET、PARAM、RAW_TEXT**：走弱钩子（见下）。

## 业务数据接入（弱钩子，按需重写）

PID / IMU / 里程计 / 电池等真实数据源在不同板子上不一样，因此用 `__weak` 钩子默认返回 0。
在你的业务代码里写一个**同名非 weak**函数即可覆盖，例如：

```c
/* 例：把 IMU 偏航角、四轮转速接进数据流 */
float upper_read_channel(uint8_t id) {
    switch (id) {
        case UPPER_CH_IMU_YAW:        return imu_yaw_deg;
        case UPPER_CH_WHEEL_LF_SPD:   return wheel_lf_rpm;
        /* ... */
        default: return 0.0f;
    }
}
void upper_on_pid_write(uint8_t id, const float *p) { /* 写到你的 PID 结构体 */ }
void upper_get_pid(uint8_t id, float *out)          { /* 回填 5 个参数 */ }
```

可重写的钩子（都在 `upper_protocol.c` 末尾，均为 `__weak`）：
`upper_read_channel / upper_on_mode / upper_on_pid_write / upper_get_pid /
upper_on_pid_target / upper_on_pid_save / upper_on_raw_text /
upper_channel_name / upper_on_param`。

> 说明：周期 `TELEMETRY(0x91)` 默认用底盘**目标**速度占位；真实反馈请覆盖
> `upper_read_channel` 或在业务里直接调 `Upper_SendTelemetry(...)`。

## 备注

- 本模块只用了 `chassis_main.h`（仅依赖 `vesc_motor.h`，**不**会拉入已删除的 `imu/up`）。
- 协议规范以 `upper_computer/docs/PROTOCOL.md` 为准；改协议在那边改 `protocol.py`
  并 `python tools/gen_protocol_doc.py` 重新生成文档。
