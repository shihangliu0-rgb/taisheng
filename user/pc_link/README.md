# pc_link —— 小电脑(ROS2 competition_gateway)串口对接模块

本模块实现下位机与**小电脑**(车上 x86 板卡,运行 `b-team` 仓库小电脑分支的
`competition_gateway` ROS2 节点)之间的串口协议。协议文档见小电脑分支
`docs/串口通信协议.md`,上位机实现见 `src/competition_gateway/`。

## 1. 串口选择结论

本地(抬升分支)工程**没有**专门给小电脑通信的串口,经排查后选用空闲串口:

| 串口 | 引脚 | 状态 | 说明 |
| --- | --- | --- | --- |
| **UART7** | PE7=RX / PE8=TX | **本模块默认使用** | 空闲,MX 已配 115200 + 中断 + DMA |
| UART8 | PE0=RX / PE1=TX | 备用 | 空闲,可通过宏一键切换 |
| USART2 | PA2/PA3 | 预留 | DMA 环形接收、极高优先级,疑似遥控器,勿动 |
| UART4 | PA0/PA1 | 已占用 | 操作手端上位机无线串口(`com_link/computer_link`) |
| USART1 | PB14/PB15 | 已占用 | IMU,921600 |
| UART9 | PD14/PD15 | 已占用 | DT35 + PNP 测距 |

**快捷切换宏** 在 `pc_link_config.h` 中,换串口只改一行:

```c
#define PC_LINK_UART_HANDLE   huart7     /* 默认 */
/* #define PC_LINK_UART_HANDLE  huart8   /* 备用 */
```

`PcLink_Init()` 会把宏选中的串口自动重配为 `PC_LINK_BAUD_RATE`(默认 115200,
与小电脑 `serial_gateway.yaml` 一致),所以切换到 huart8(原 2000000)也无需改
CubeMX 配置。

## 2. 硬件接线

USB-TTL(小电脑 `/dev/ttyUSB0` 侧)→ 板端:

```
USB-TTL TXD  ->  PE7 (UART7_RX)
USB-TTL RXD  <-  PE8 (UART7_TX)
USB-TTL GND  --  GND
```

若切换为 huart8: TXD -> PE0, RXD <- PE1。

## 3. 协议速览

- **上位机下发** 感知帧 `AA 55 10`(44B,红蓝块+金球 xyz,float 小端)
  与位置帧 `AA 55 11`(24B,赛场 xyz + 四元数 w),同一串口 50Hz 交替;
- **下位机回传** 状态帧 `55 AA 20`(8B,state + error),要求 ≥10Hz
  (本模块默认 20Hz,`PC_LINK_STATUS_PERIOD_MS` 可调);
- 校验和 = 帧类型字节起 8 位累加和,帧尾 `0D 0A`;
- 有效位 `flags` 无效时对应数据清零,超时(`PC_LINK_DATA_TIMEOUT_MS`)
  自动清除有效位,不沿用旧数据。

## 4. 编译接入(需要手动做的 3 处,本模块未改动任何现有文件)

### (1) Keil 工程添加源文件

把 `user/pc_link/pc_link.c` 加入 MDK-ARM 工程(建议放在与
`user/com_link/computer_link.c` 相同的分组,头文件可选)。

### (2) 串口回调分发 —— `Core/Src/main.c` 的 `USER CODE 4` 区

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  ComputerLink_RxCplt(huart);
  DT35PnpLink_RxCplt(huart);
  PcLink_RxCplt(huart);            /* ← 新增这一行 */
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  ComputerLink_Error(huart);
  DT35PnpLink_Error(huart);
  ImuMain_HandleUartError(huart);
  PcLink_Error(huart);             /* ← 新增这一行 */
}
```

并在 `main.c` 头文件区加入 `#include "pc_link.h"`。

### (3) 初始化与周期调用 —— `Core/Src/freertos.c` 的 `StartCommTask`

```c
(void)ComputerLink_Init(&huart4);
(void)DT35PnpLink_Init(&huart9);
(void)PcLink_Init();               /* ← 新增:使用 pc_link_config.h 宏选定的串口 */

for(;;)
{
  DT35PnpLink_Run();
  Action_UpdatePnp(...);
  ComputerLink_Run();
  PcLink_Run();                    /* ← 新增:1ms 周期调用 */
  osDelay(1);
}
```

## 5. 使用示例(其他模块读取数据 / 上报状态)

```c
#include "pc_link.h"

void Example_UsePerception(void)
{
    pc_perception_t p;

    if (PcLink_GetPerception(&p))
    {
        if (p.flags & PC_LINK_FLAG_BALL_VALID)
        {
            /* 使用 p.ball_x_m / p.ball_y_m / p.ball_z_m 对接球目标 */
        }
        if (p.flags & PC_LINK_FLAG_RED_VALID)
        {
            /* 使用 p.red_x_m ... */
        }
    }

    if (PcLink_IsOnline() == false)
    {
        /* 链路超时,自行决定进入降级策略 */
    }

    /* 状态机运行时把板端状态回传给小电脑 */
    PcLink_SetStatus(/*state*/ 1U, /*error*/ 0U);
}
```

> 提示:感知数据可进一步接入现有底盘接口(如 `Chassis_SetVelocity`)或
> 抬升动作接口(`Action_Request`),建议在 `chassis_main` / `up_main` 中读取
> `PcLink_GetPerception()` 的返回值后再下发目标,不要在本模块内做控制闭环。

## 6. 与上位机参数的对应关系

| 本模块宏 | 小电脑 yaml / 协议要求 |
| --- | --- |
| `PC_LINK_BAUD_RATE` = 115200 | `serial_gateway.yaml: baudrate: 115200` |
| `PC_LINK_STATUS_PERIOD_MS` = 50 (20Hz) | 状态帧 ≥10Hz;上位机 500ms 判定离线 |
| `PC_LINK_DATA_TIMEOUT_MS` = 200 | `data_timeout_ms: 200` |
| 帧类型 0x10 / 0x11 / 0x20 | `serial_protocol.hpp` 中 `k_tx_perception_type` 等 |
