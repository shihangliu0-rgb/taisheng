#ifndef DT35_LINK_H
#define DT35_LINK_H

#include "stm32h7xx_hal.h"

/* ====== DT35 激光总线串口选择 ======
 * b-team 抬升分支用 USART9(huart9)；723 工程未开启 USART9，
 * 这里默认用空闲的 huart2(USART2 PA2/PA3，已在 usart.c 初始化)。
 * 换口只改这里，不修改 usart.c；需保证所填 huartX 已在 usart.c 里初始化。 */
#ifndef DT35_UART_HANDLE
#define DT35_UART_HANDLE   huart9
#endif

/* UART 总线上两个 DT35 传感器的固定地址。 */
#define DT35_LINK_ADDR_41 0x41U
#define DT35_LINK_ADDR_40 0x40U
/* Frame: AA, address, distance_cm low byte, high byte, XOR checksum. */

extern volatile uint16_t dt35_distance_40_cm;
extern volatile uint16_t dt35_distance_41_cm;
extern volatile uint8_t dt35_online_40;
extern volatile uint8_t dt35_online_41;

HAL_StatusTypeDef DT35Link_Init(UART_HandleTypeDef *uart);
void DT35Link_Run(void);
void DT35Link_Send(UART_HandleTypeDef *uart);
void DT35Link_RxCplt(UART_HandleTypeDef *uart);
void DT35Link_Error(UART_HandleTypeDef *uart);

#endif
