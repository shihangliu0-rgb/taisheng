/**
 * @file    imu_main.h
 * @brief   IMU 在 723(H723) 主板上的任务接入层（移植自 F405 imu 工程）
 *
 * 说明：
 *  - F405 板级代码(my_main.c/h：main 循环 / USART1 日志 / USART6 回调)已丢弃，
 *    本文件按 723 工程的任务风格(Init + Run1ms + 回调)重新组织，
 *    内部调用 user/imu 驱动(Imu_*)与算法(imu_algo)。
 *  - 串口不写死、不修改 usart.c：用 IMU_UART_HANDLE 宏选择，由你决定开哪个口。
 */

#ifndef IMU_MAIN_H
#define IMU_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "imu.h"            /* 顺带带入驱动 API(Imu_GetYaw 等)与算法调参宏 */

/* ==========================================================================
 *  ★ IMU 传感器串口选择（由你决定）
 * --------------------------------------------------------------------------
 *  F405 上 IMU 接在 USART6；723 没有 USART6，请按你的硬件接线选一个已开启的串口，
 *  并在 CubeMX / usart.c 里确认【该串口已使能 + 配了 DMA RX】(用于空闲中断接收)。
 *  现有候选(723)：huart1 / huart2(均已配 DMA RX)；huart4 已被上位机占用。
 *  —— 禁止为执行而擅自把 F405 的 usart 搬进 usart.c；开哪个口由你拍板。
 * ========================================================================== */
#ifndef IMU_UART_HANDLE
#warning "IMU_UART_HANDLE 未指定，默认使用 huart2；请按硬件接线在 imu_main.h 修改，并确认已在 CubeMX 开启该串口及 DMA RX"
#define IMU_UART_HANDLE   huart2
#endif

/**
 * @brief  IMU 任务初始化：绑定串口 -> 初始化算法 -> 启动 DMA 接收 -> 启动零漂校准
 * @retval HAL_OK
 */
HAL_StatusTypeDef ImuMain_Init(void);

/**
 * @brief  IMU 任务 1ms 周期调用：驱动指令序列 / 延时回调调度
 */
void ImuMain_Run1ms(void);

/**
 * @brief  串口 DMA 空闲接收完成回调（放到 HAL_UARTEx_RxEventCallback）
 * @param  huart 触发回调的串口句柄
 * @param  size  本次接收字节数
 */
void ImuMain_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t size);

/**
 * @brief  串口错误回调（放到 HAL_UART_ErrorCallback）
 */
void ImuMain_HandleUartError(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* IMU_MAIN_H */
