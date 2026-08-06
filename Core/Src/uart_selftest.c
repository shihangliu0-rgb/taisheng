/* =====================================================================
 *  串口自测实现 —— 见 uart_selftest.h 说明
 *  整文件被 UART_SELFTEST 包裹；未定义时编译为空。
 *
 *  完全自包含：不依赖 main 的 MX_*_Init，自己使能时钟、配 GPIO 复用、初始化
 *  各 UART，然后用阻塞 HAL_UART_Transmit(无中断/无 DMA) 逐个发送。
 * ===================================================================== */

#include "uart_selftest.h"

#ifdef UART_SELFTEST

#include "stm32h7xx_hal.h"

#include <stdio.h>
#include <string.h>

#define UART_TEST_BAUD          115200U
#define UART_TEST_TX_TIMEOUT_MS 100U
#define UART_TEST_ROUND_MS      1000U

typedef struct
{
    const char   *name;        /* 发送时显示的名字(含引脚) */
    USART_TypeDef *instance;   /* USART1/USART2/.../UART8 */
    GPIO_TypeDef *port;        /* TX/RX 同一组 GPIO(本表均如此) */
    uint16_t      tx_pin;
    uint16_t      rx_pin;
    uint32_t      af;          /* GPIO 复用号(用 HAL 的 GPIO_AFx 宏，保证正确) */
} uart_entry_t;

/* 被测串口表。引脚均避开 .ioc 已占用脚(PA11/12 FDCAN1、PA13/14 SWD、
 * PB12/13 FDCAN2、PD12/13 FDCAN3)。不用的条目直接注释掉即可。 */
static const uart_entry_t uart_list[] =
{
    { "USART1 PB14/PB15", USART1, GPIOB, GPIO_PIN_14, GPIO_PIN_15, GPIO_AF4_USART1 },
    { "USART2 PA2/PA3",   USART2, GPIOA, GPIO_PIN_2,  GPIO_PIN_3,  GPIO_AF7_USART2 },
    { "USART3 PB10/PB11", USART3, GPIOB, GPIO_PIN_10, GPIO_PIN_11, GPIO_AF7_USART3 },
    { "UART4  PA0/PA1",   UART4,  GPIOA, GPIO_PIN_0,  GPIO_PIN_1,  GPIO_AF8_UART4  },
    { "USART6 PG14/PG9",  USART6, GPIOG, GPIO_PIN_14, GPIO_PIN_9,  GPIO_AF7_USART6 },
    { "UART7  PE8/PE7",   UART7,  GPIOE, GPIO_PIN_8,  GPIO_PIN_7,  GPIO_AF7_UART7  },
    { "UART8  PE1/PE0",   UART8,  GPIOE, GPIO_PIN_1,  GPIO_PIN_0,  GPIO_AF8_UART8  },
};
#define UART_LIST_LEN  (sizeof(uart_list) / sizeof(uart_list[0]))

static UART_HandleTypeDef htest[UART_LIST_LEN];

static void enable_uart_clk(USART_TypeDef *i)
{
    if      (i == USART1) { __HAL_RCC_USART1_CLK_ENABLE(); }
    else if (i == USART2) { __HAL_RCC_USART2_CLK_ENABLE(); }
    else if (i == USART3) { __HAL_RCC_USART3_CLK_ENABLE(); }
    else if (i == UART4)  { __HAL_RCC_UART4_CLK_ENABLE();  }
    else if (i == USART6) { __HAL_RCC_USART6_CLK_ENABLE(); }
    else if (i == UART7)  { __HAL_RCC_UART7_CLK_ENABLE();  }
    else if (i == UART8)  { __HAL_RCC_UART8_CLK_ENABLE();  }
}

static void enable_gpio_clk(GPIO_TypeDef *p)
{
    if      (p == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
    else if (p == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
    else if (p == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
    else if (p == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
    else if (p == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
    else if (p == GPIOG) { __HAL_RCC_GPIOG_CLK_ENABLE(); }
}

static void init_one(uint8_t idx)
{
    const uart_entry_t *e = &uart_list[idx];
    UART_HandleTypeDef *h = &htest[idx];
    GPIO_InitTypeDef g = {0};

    enable_uart_clk(e->instance);
    enable_gpio_clk(e->port);

    g.Pin       = e->tx_pin | e->rx_pin;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = e->af;
    HAL_GPIO_Init(e->port, &g);

    h->Instance        = e->instance;
    h->Init.BaudRate   = UART_TEST_BAUD;
    h->Init.WordLength = UART_WORDLENGTH_8B;
    h->Init.StopBits   = UART_STOPBITS_1;
    h->Init.Parity     = UART_PARITY_NONE;
    h->Init.Mode       = UART_MODE_TX_RX;
    h->Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    h->Init.OverSampling = UART_OVERSAMPLING_16;
    (void)HAL_UART_Init(h);
}

void UartSelftest_Run(void)
{
    char buf[64];
    int32_t n;
    uint8_t i;

    for (i = 0U; i < UART_LIST_LEN; i++)
    {
        init_one(i);
    }

    for (;;)
    {
        for (i = 0U; i < UART_LIST_LEN; i++)
        {
            n = snprintf(buf, sizeof(buf), "\r\n[%s] is ok\r\n", uart_list[i].name);
            if (n > 0)
            {
                (void)HAL_UART_Transmit(&htest[i], (uint8_t *)buf,
                                        (uint16_t)n, UART_TEST_TX_TIMEOUT_MS);
            }
        }
        HAL_Delay(UART_TEST_ROUND_MS);
    }
}

#endif /* UART_SELFTEST */
