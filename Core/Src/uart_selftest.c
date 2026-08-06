/* =====================================================================
 *  串口自测实现 —— 见 uart_selftest.h 说明
 *  整文件被 UART_SELFTEST 包裹；未定义时编译为空。
 * ===================================================================== */

#include "uart_selftest.h"

#ifdef UART_SELFTEST

#include "stm32h7xx_hal.h"
#include "usart.h"          /* huart1 / huart2 / huart4(均已由 main 里 MX_*_Init 初始化) */

#include <string.h>

#define UART_TEST_TX_TIMEOUT_MS  100U
#define UART_TEST_GAP_MS         500U

static void send_line(UART_HandleTypeDef *huart, const char *s)
{
    (void)HAL_UART_Transmit(huart, (const uint8_t *)s, (uint16_t)strlen(s),
                            UART_TEST_TX_TIMEOUT_MS);
}

void UartSelftest_Run(void)
{
    /* 注：各串口已在 main() 里被 MX_USART1/USART2/UART4_Init 初始化，
     * 此处只做阻塞发送，不碰中断/DMA，不启动 FreeRTOS。 */
    for (;;)
    {
        send_line(&huart1, "\r\n[USART1 PB14/PB15] usart1 is ok\r\n");
        HAL_Delay(UART_TEST_GAP_MS);

        send_line(&huart2, "\r\n[USART2 PA2/PA3]   usart2 is ok\r\n");
        HAL_Delay(UART_TEST_GAP_MS);

        send_line(&huart4, "\r\n[UART4  PA0/PA1]   uart4 is ok\r\n");
        HAL_Delay(UART_TEST_GAP_MS);

        send_line(&huart1, "\r\n---- next round ----\r\n");
        HAL_Delay(UART_TEST_GAP_MS);
    }
}

#endif /* UART_SELFTEST */
