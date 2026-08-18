#ifndef MCU_LINK_H
#define MCU_LINK_H

#include "stm32h7xx_hal.h"

#include <stdint.h>

#define MCU_LINK_TX_BUFFER_SIZE 512U

extern volatile uint32_t mcu_link_rx_bytes;
extern volatile uint32_t mcu_link_uart_error_count;
extern volatile uint32_t mcu_link_tx_error_count;

/**
 * @brief 初始化 USART6 主控间 DMA 链路
 * @param uart USART6 句柄
 * @retval HAL 状态
 */
HAL_StatusTypeDef McuLink_Init(UART_HandleTypeDef *uart);

/**
 * @brief 处理 DMA 接收数据和串口错误恢复
 * @retval None
 */
void McuLink_Run(void);

/**
 * @brief 从主控间接收缓冲区读取数据
 * @param data 输出缓冲区
 * @param max_length 最多读取的字节数
 * @retval 实际读取的字节数
 */
uint16_t McuLink_Read(uint8_t *data, uint16_t max_length);

/**
 * @brief 通过 USART6 DMA 发送数据
 * @param data 待发送数据
 * @param length 数据长度，不得超过 MCU_LINK_TX_BUFFER_SIZE
 * @retval HAL 状态
 */
HAL_StatusTypeDef McuLink_Send(const uint8_t *data, uint16_t length);

void McuLink_HandleTxCplt(UART_HandleTypeDef *uart);
void McuLink_HandleUartError(UART_HandleTypeDef *uart);

#endif
