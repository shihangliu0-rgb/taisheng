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
  与位置帧 `AA 55 11`(24B,赛场 xyz + yaw_rad),同一串口 50Hz 交替
  (小电脑侧已修复 field_w 语义,现在按 yaw_rad 解析);
- **下位机回传** 状态帧 `55 AA 20`(8B,state + error),要求 ≥10Hz
  (本模块默认 20Hz,`PC_LINK_STATUS_PERIOD_MS` 可调);
- 校验和 = 帧类型字节起 8 位累加和,帧尾 `0D 0A`;
- 有效位 `flags` 无效时对应数据清零,超时(`PC_LINK_DATA_TIMEOUT_MS`)
  自动清除有效位,不沿用旧数据。

## 4. 编译接入(已全部落地,无需手工操作)

以下三处已直接写入工程:

1. `MDK-ARM/b-up.uvprojx`:pc_link.c 已加入 `Application/User/com_link`
   分组,IncludePath 已含 `../user/pc_link`;
2. `Core/Src/main.c` USER CODE 4 区:已挂 `PcLink_RxCplt(huart)` 与
   `PcLink_Error(huart)`(UART7 收到的每个字节都进 pc_link 解析);
3. `Core/Src/freertos.c` StartCommTask:已调用 `PcLink_Init()`(自动把
   huart7 重配为 115200 并启动接收)与 `PcLink_Run()`(1ms 周期解析
   环形缓冲、超时管理、50ms 周期回传状态帧)。

上电后小电脑即可在下位机侧拿到:感知帧/位置帧数据(下方 API),下位机
自动回传 0x20 状态帧(规划器状态 + 停止原因),上位机网关据此判断控制器
在线。

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
