#ifndef XIAODIANJI_LINK_H
#define XIAODIANJI_LINK_H

/**
 * @file xiaodianji_link.h
 * @brief 小电脑(ROS2 感知上位机) ↔ STM32 串口通信驱动
 *
 * 协议(来自 b-team 小电脑分支 docs/串口通信协议.md):
 *
 * 上位机→STM32:
 *   位置帧(0x11, 24B): AA 55 11 seq flags field_x field_y field_z field_w checksum 0D 0A
 *   感知帧(0x10, 44B): AA 55 10 seq flags red_xyz blue_xyz ball_xyz checksum 0D 0A
 *
 * STM32→上位机:
 *   状态帧(0x20, 8B):  55 AA 20 state error checksum 0D 0A
 *
 * 波特率 115200, 小端 IEEE754 float, checksum=偏移2到末尾数据的8位累加和。
 */

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ====== 小电脑串口选择(快捷修改) ======
 * 默认 huart2(PA2/PA3)。注意: USART2 当前波特率可能需要改为 115200(在 CubeMX/usart.c)。
 * 如换口改这里即可。 */
#ifndef XIAODIANJI_UART_HANDLE
#define XIAODIANJI_UART_HANDLE   huart2
#endif

#define XIAODIANJI_TIMEOUT_MS   1000U   /* 超过此时间未收到帧 → 判离线 */

/* ====== 小电脑下发的数据(全局, 其他模块只读) ====== */
typedef struct
{
    /* 位置帧(0x11): 机器人赛场坐标 */
    float field_x_m;
    float field_y_m;
    float field_z_m;
    float field_w;          /* 四元数 w 分量(旋转) */
    uint8_t pose_valid;     /* 位 0: 定位有效 */

    /* 感知帧(0x10): 目标物位置 */
    float red_x_m, red_y_m, red_z_m;
    float blue_x_m, blue_y_m, blue_z_m;
    float ball_x_m, ball_y_m, ball_z_m;
    uint8_t red_valid;      /* 红块有效 */
    uint8_t blue_valid;     /* 蓝块有效 */
    uint8_t ball_valid;     /* 球有效 */

    uint8_t online;         /* 小电脑在线 */
} Xiaodianji_Data_t;

extern volatile Xiaodianji_Data_t xiaodianji_data;

/* ====== API ====== */
void Xiaodianji_Init(void);
void Xiaodianji_Run(void);                                /* 周期调用: 超时检查 */
void Xiaodianji_RxCplt(UART_HandleTypeDef *huart);        /* HAL_UART_RxCpltCallback 中调 */
void Xiaodianji_Error(UART_HandleTypeDef *huart);         /* HAL_UART_ErrorCallback 中调 */
void Xiaodianji_SendStatus(uint8_t state, uint8_t error); /* 发状态帧(0x20)给小电脑 */

#endif /* XIAODIANJI_LINK_H */
