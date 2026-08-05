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
 *  ★ IMU 传感器串口 —— 485 口，723 上为 USART3 (PD8=TX, PD9=RX)
 * --------------------------------------------------------------------------
 *  F405 工程用 USART6；723 没有，按你的硬件接在 PD8/PD9 = USART3。
 *  ★ 需你在 CubeMX 里【把 USART3 开起来 + PD8/PD9 复用 + 配 DMA RX(空闲中断)】，
 *    并把 PD10/PD11 配为推挽输出(RS485 方向控制)。这些属于基础配置，由你决定、
 *    在 CubeMX 里改 usart.c/gpio.c，本工程不擅自修改基础文件、也不搬 F4 的配置过来。
 *  huart3 由 CubeMX 生成的 usart.c 提供；未生成前 imu_main.c 里有 extern 前向声明可先编译。
 * ========================================================================== */
#ifndef IMU_UART_HANDLE
#define IMU_UART_HANDLE   huart3
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
