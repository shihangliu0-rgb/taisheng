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
 *  ★ IMU 传感器串口 —— 485 口，723 上用 USART1 (PB14=TX, PB15=RX，DMA RX 已配)
 * --------------------------------------------------------------------------
 *  USART1 已在 usart.c/HAL_UART_MspInit 中配好(PB14/PB15, AF4, DMA1_Stream2 RX
 *  circular)，故改这里的宏即可切换 IMU 串口，不必动 usart.c。
 *  RS485 方向由硬件自动控制，无方向引脚。
 *  如换其它串口：把下面宏改成对应 huartX，并确保该串口在 usart.c 里已初始化。
 * ========================================================================== */
#ifndef IMU_UART_HANDLE
#define IMU_UART_HANDLE   huart1
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
 * @brief  航向保持(Yaw Hold)：根据当前偏航角计算底盘旋转输出
 * @param  vx    X 方向速度指令(用于判静止)
 * @param  vy    Y 方向速度指令(用于判静止)
 * @param  omega 手动旋转指令
 * @retval 修正后的底盘旋转指令；IMU 未就绪时直通 omega
 */
int16_t ImuMain_CalcOmega(int16_t vx, int16_t vy, int16_t omega);

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
