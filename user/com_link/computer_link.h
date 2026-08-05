#ifndef COMPUTER_LINK_H
#define COMPUTER_LINK_H

#include "stm32h7xx_hal.h"

/**
 * @brief 初始化电脑端 UART 接收
 * @param uart 电脑无线串口句柄
 * @retval HAL 状态
 */
HAL_StatusTypeDef ComputerLink_Init(UART_HandleTypeDef *uart);

/**
 * @brief 处理电脑速度指令和通信超时
 * @retval None
 */
void ComputerLink_Run(void);

/**
 * @brief 处理 UART 单字节接收完成事件
 * @param uart 触发回调的串口句柄
 * @retval None
 */
void ComputerLink_RxCplt(UART_HandleTypeDef *uart);

/**
 * @brief 处理 UART 错误并请求恢复接收
 * @param uart 触发错误的串口句柄
 * @retval None
 */
void ComputerLink_Error(UART_HandleTypeDef *uart);

#endif
