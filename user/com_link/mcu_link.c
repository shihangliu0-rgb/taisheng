#include "mcu_link.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define MCU_LINK_DMA_BUFFER_SIZE 1024U

static UART_HandleTypeDef *mcu_uart;
static uint8_t mcu_rx_buffer[MCU_LINK_DMA_BUFFER_SIZE];
static uint8_t mcu_tx_buffer[MCU_LINK_TX_BUFFER_SIZE];
static uint16_t mcu_rx_read_pos;
static volatile bool mcu_restart_requested;
static volatile bool mcu_tx_busy;
static bool mcu_initialized;

volatile uint32_t mcu_link_rx_bytes;
volatile uint32_t mcu_link_uart_error_count;
volatile uint32_t mcu_link_tx_error_count;

static HAL_StatusTypeDef McuLink_StartReceive(void)
{
    HAL_StatusTypeDef status;

    mcu_rx_read_pos = 0U;
    __HAL_UART_CLEAR_FLAG(mcu_uart,
                          UART_CLEAR_OREF | UART_CLEAR_NEF |
                          UART_CLEAR_PEF | UART_CLEAR_FEF);
    __HAL_UART_SEND_REQ(mcu_uart, UART_RXDATA_FLUSH_REQUEST);
    status = HAL_UART_Receive_DMA(mcu_uart, mcu_rx_buffer,
                                  MCU_LINK_DMA_BUFFER_SIZE);
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(mcu_uart->hdmarx, DMA_IT_HT | DMA_IT_TC);
    }
    return status;
}

HAL_StatusTypeDef McuLink_Init(UART_HandleTypeDef *uart)
{
    if ((uart == NULL) || (uart->Instance != USART6) ||
        (uart->hdmarx == NULL) || (uart->hdmatx == NULL) ||
        (uart->hdmarx->Init.Mode != DMA_CIRCULAR))
    {
        return HAL_ERROR;
    }
    if (mcu_initialized)
    {
        return (mcu_uart == uart) ? HAL_OK : HAL_ERROR;
    }

    mcu_uart = uart;
    (void)memset(mcu_rx_buffer, 0, sizeof(mcu_rx_buffer));
    (void)memset(mcu_tx_buffer, 0, sizeof(mcu_tx_buffer));
    mcu_restart_requested = false;
    mcu_tx_busy = false;
    mcu_link_rx_bytes = 0U;
    mcu_link_uart_error_count = 0U;
    mcu_link_tx_error_count = 0U;

    if (McuLink_StartReceive() != HAL_OK)
    {
        mcu_uart = NULL;
        return HAL_ERROR;
    }

    mcu_initialized = true;
    return HAL_OK;
}

void McuLink_Run(void)
{
    if (!mcu_initialized || !mcu_restart_requested)
    {
        return;
    }

    mcu_restart_requested = false;
    (void)HAL_UART_AbortReceive(mcu_uart);
    (void)HAL_UART_AbortTransmit(mcu_uart);
    mcu_tx_busy = false;
    if (McuLink_StartReceive() != HAL_OK)
    {
        mcu_restart_requested = true;
    }
}

uint16_t McuLink_Read(uint8_t *data, uint16_t max_length)
{
    uint16_t count = 0U;
    uint16_t write_pos;

    if (!mcu_initialized || (data == NULL) || (max_length == 0U))
    {
        return 0U;
    }

    write_pos = (uint16_t)(MCU_LINK_DMA_BUFFER_SIZE -
                           __HAL_DMA_GET_COUNTER(mcu_uart->hdmarx));
    if (write_pos >= MCU_LINK_DMA_BUFFER_SIZE)
    {
        write_pos = 0U;
    }
    __DMB();
    while ((count < max_length) && (mcu_rx_read_pos != write_pos))
    {
        data[count] = mcu_rx_buffer[mcu_rx_read_pos];
        count++;
        mcu_rx_read_pos++;
        if (mcu_rx_read_pos >= MCU_LINK_DMA_BUFFER_SIZE)
        {
            mcu_rx_read_pos = 0U;
        }
    }

    mcu_link_rx_bytes += count;
    return count;
}

HAL_StatusTypeDef McuLink_Send(const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;

    if (!mcu_initialized || (data == NULL) || (length == 0U) ||
        (length > MCU_LINK_TX_BUFFER_SIZE))
    {
        return HAL_ERROR;
    }
    if (mcu_tx_busy)
    {
        return HAL_BUSY;
    }

    (void)memcpy(mcu_tx_buffer, data, length);
    mcu_tx_busy = true;
    status = HAL_UART_Transmit_DMA(mcu_uart, mcu_tx_buffer, length);
    if (status != HAL_OK)
    {
        mcu_tx_busy = false;
        mcu_link_tx_error_count++;
    }
    return status;
}

void McuLink_HandleTxCplt(UART_HandleTypeDef *uart)
{
    if (mcu_initialized && (uart == mcu_uart))
    {
        mcu_tx_busy = false;
    }
}

void McuLink_HandleUartError(UART_HandleTypeDef *uart)
{
    if (!mcu_initialized || (uart != mcu_uart))
    {
        return;
    }

    mcu_link_uart_error_count++;
    mcu_restart_requested = true;
}
