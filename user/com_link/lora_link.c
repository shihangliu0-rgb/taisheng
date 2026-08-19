#include "lora_link.h"
#include "chassis_main.h"
#include "mcu_link.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define LORA_LINK_DMA_BUFFER_SIZE 256U
#define LORA_READ_BUFFER_SIZE      32U

#define REMOTE_FRAME_HEADER_0      0xA5U
#define REMOTE_FRAME_HEADER_1      0x5AU
#define REMOTE_FRAME_HEADER_SIZE   2U
#define REMOTE_LOCAL_PAYLOAD_SIZE  6U
#define REMOTE_SECOND_PAYLOAD_SIZE 2U
#define REMOTE_FRAME_PAYLOAD_SIZE  (REMOTE_LOCAL_PAYLOAD_SIZE + \
                                    REMOTE_SECOND_PAYLOAD_SIZE)
#define REMOTE_FRAME_SIZE          (REMOTE_FRAME_HEADER_SIZE + \
                                    REMOTE_FRAME_PAYLOAD_SIZE)
#define REMOTE_FRAME_GAP_MS        5U
#define MCU_RETURN_BUFFER_SIZE     32U

#define REMOTE_AXIS_CENTER         128U
#define REMOTE_FAST_SPEED_MM_S     150
#define REMOTE_FINE_SPEED_MM_S     75
#define REMOTE_ROTATION_MRAD_S     10
#define REMOTE_TIMEOUT_MS          200U
#define REMOTE_LEFT_SHOULDER_BIT   (1U << 6U)
#define REMOTE_RIGHT_SHOULDER_BIT  (1U << 7U)
#define REMOTE_LOCAL_BUTTON_MASK    0x3FU
#define REMOTE_PE0_SWITCH_BIT      (1U << 0U)
#define REMOTE_PD6_SWITCH_BIT      (1U << 1U)

static UART_HandleTypeDef *lora_uart;
static uint8_t lora_rx_buffer[LORA_LINK_DMA_BUFFER_SIZE];
static uint8_t lora_tx_buffer[LORA_LINK_TX_BUFFER_SIZE];
static uint16_t lora_rx_read_pos;
static volatile bool lora_restart_requested;
static volatile bool lora_tx_busy;
static bool lora_initialized;
static uint8_t remote_frame[REMOTE_FRAME_SIZE];
static uint8_t remote_frame_index;
static uint32_t remote_frame_last_rx_ms;
static uint32_t remote_last_rx_ms;
static uint8_t mcu_return_buffer[MCU_RETURN_BUFFER_SIZE];
static uint16_t mcu_return_length;
static bool remote_pe0_initialized;
static uint8_t remote_pe0_last;

volatile uint32_t lora_link_rx_bytes;
volatile uint32_t lora_link_uart_error_count;
volatile uint32_t lora_link_tx_error_count;
volatile uint32_t lora_link_valid_frame_count;
volatile uint32_t lora_link_forward_error_count;
volatile uint32_t lora_link_return_error_count;
volatile uint32_t lora_link_returned_bytes;
volatile uint8_t lora_remote_buttons;
volatile uint8_t lora_remote_pe0_switch;
volatile uint8_t lora_remote_pd6_switch;
volatile uint8_t lora_remote_online;

static int16_t LoraLink_MapAxis(uint8_t value, int16_t max_speed)
{
    int16_t offset = (int16_t)value - (int16_t)REMOTE_AXIS_CENTER;

    if (offset < 0)
    {
        return (int16_t)(((int32_t)offset * max_speed) / 128);
    }
    return (int16_t)(((int32_t)offset * max_speed) / 127);
}

static void LoraLink_MapStick(uint8_t x, uint8_t y, int16_t max_speed,
                              int16_t *vx, int16_t *vy)
{
    float magnitude;

    *vx = LoraLink_MapAxis(x, max_speed);
    *vy = (int16_t)-LoraLink_MapAxis(y, max_speed);
    magnitude = sqrtf((float)((int32_t)*vx * *vx + (int32_t)*vy * *vy));
    if (magnitude > (float)max_speed)
    {
        *vx = (int16_t)((float)*vx * (float)max_speed / magnitude);
        *vy = (int16_t)((float)*vy * (float)max_speed / magnitude);
    }
}

static void LoraLink_HandleLocalPayload(const uint8_t *payload)
{
    /* payload: axes[0..3], local keys[4], PE0[5] bit0, PD6[5] bit1. */
    uint8_t buttons = payload[4];
    bool left_active;
    bool left_shoulder;
    bool right_shoulder;
    int16_t vx;
    int16_t vy;
    int16_t z = 0;

    left_active = (payload[0] != REMOTE_AXIS_CENTER) ||
                  (payload[1] != REMOTE_AXIS_CENTER);
    if (left_active)
    {
        LoraLink_MapStick(payload[0], payload[1], REMOTE_FAST_SPEED_MM_S,
                          &vx, &vy);
    }
    else
    {
        LoraLink_MapStick(payload[2], payload[3], REMOTE_FINE_SPEED_MM_S,
                          &vx, &vy);
    }

    left_shoulder = (buttons & REMOTE_LEFT_SHOULDER_BIT) != 0U;
    right_shoulder = (buttons & REMOTE_RIGHT_SHOULDER_BIT) != 0U;
    if (left_shoulder != right_shoulder)
    {
        z = left_shoulder ? REMOTE_ROTATION_MRAD_S : -REMOTE_ROTATION_MRAD_S;
    }

    lora_remote_buttons = (uint8_t)(buttons & REMOTE_LOCAL_BUTTON_MASK);
    lora_remote_pe0_switch =
        ((payload[5] & REMOTE_PE0_SWITCH_BIT) != 0U) ? 1U : 0U;
    lora_remote_pd6_switch =
        ((payload[5] & REMOTE_PD6_SWITCH_BIT) != 0U) ? 1U : 0U;
    lora_remote_online = 1U;
    remote_last_rx_ms = HAL_GetTick();
    if (!remote_pe0_initialized)
    {
        remote_pe0_last = lora_remote_pe0_switch;
        remote_pe0_initialized = true;
    }
    else if (remote_pe0_last != lora_remote_pe0_switch)
    {
        remote_pe0_last = lora_remote_pe0_switch;
        Chassis_SetControlMode(CHASSIS_CONTROL_MANUAL);
    }

    if (Chassis_GetControlMode() == CHASSIS_CONTROL_MANUAL)
    {
        (void)Chassis_RequestVelocity(CHASSIS_CMD_SOURCE_LORA,
                                      vx, vy, z, REMOTE_TIMEOUT_MS);
    }
    else
    {
        Chassis_ReleaseVelocity(CHASSIS_CMD_SOURCE_LORA);
    }
}

static void LoraLink_ResetRemoteFrame(void)
{
    remote_frame_index = 0U;
    remote_frame_last_rx_ms = 0U;
}

static void LoraLink_HandleRemoteFrame(void)
{
    const uint8_t *local_payload =
        &remote_frame[REMOTE_FRAME_HEADER_SIZE];

    lora_link_valid_frame_count++;
    LoraLink_HandleLocalPayload(local_payload);
    if (McuLink_Send(remote_frame, REMOTE_FRAME_SIZE) != HAL_OK)
    {
        lora_link_forward_error_count++;
    }
}

static void LoraLink_ParseByte(uint8_t data)
{
    uint32_t now_ms = HAL_GetTick();

    /* A complete frame is sent in one UART transaction. Use the idle gap to
     * discard a truncated frame before looking for the next header. */
    if ((remote_frame_index != 0U) &&
        ((now_ms - remote_frame_last_rx_ms) >= REMOTE_FRAME_GAP_MS))
    {
        LoraLink_ResetRemoteFrame();
    }

    if (remote_frame_index == 0U)
    {
        if (data == REMOTE_FRAME_HEADER_0)
        {
            remote_frame[0] = data;
            remote_frame_index = 1U;
            remote_frame_last_rx_ms = now_ms;
        }
        return;
    }

    if (remote_frame_index == 1U)
    {
        if (data == REMOTE_FRAME_HEADER_1)
        {
            remote_frame[1] = data;
            remote_frame_index = REMOTE_FRAME_HEADER_SIZE;
            remote_frame_last_rx_ms = now_ms;
        }
        else if (data == REMOTE_FRAME_HEADER_0)
        {
            /* Treat a repeated header byte as the start of a new frame. */
            remote_frame[0] = data;
            remote_frame_last_rx_ms = now_ms;
        }
        else
        {
            LoraLink_ResetRemoteFrame();
        }
        return;
    }

    remote_frame[remote_frame_index++] = data;
    remote_frame_last_rx_ms = now_ms;
    if (remote_frame_index == REMOTE_FRAME_SIZE)
    {
        LoraLink_HandleRemoteFrame();
        LoraLink_ResetRemoteFrame();
    }
}

static void LoraLink_ForwardMcuData(void)
{
    HAL_StatusTypeDef status;

    if (mcu_return_length == 0U)
    {
        mcu_return_length = McuLink_Read(mcu_return_buffer,
                                         sizeof(mcu_return_buffer));
    }
    if (mcu_return_length == 0U)
    {
        return;
    }

    status = LoraLink_Send(mcu_return_buffer, mcu_return_length);
    if (status == HAL_OK)
    {
        lora_link_returned_bytes += mcu_return_length;
        mcu_return_length = 0U;
    }
    else if (status != HAL_BUSY)
    {
        lora_link_return_error_count++;
    }
}

static HAL_StatusTypeDef LoraLink_StartReceive(void)
{
    HAL_StatusTypeDef status;

    lora_rx_read_pos = 0U;
    __HAL_UART_CLEAR_FLAG(lora_uart,
                          UART_CLEAR_OREF | UART_CLEAR_NEF |
                          UART_CLEAR_PEF | UART_CLEAR_FEF);
    __HAL_UART_SEND_REQ(lora_uart, UART_RXDATA_FLUSH_REQUEST);
    status = HAL_UART_Receive_DMA(lora_uart, lora_rx_buffer,
                                  LORA_LINK_DMA_BUFFER_SIZE);
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(lora_uart->hdmarx, DMA_IT_HT | DMA_IT_TC);
    }
    return status;
}

HAL_StatusTypeDef LoraLink_Init(UART_HandleTypeDef *uart)
{
    if ((uart == NULL) || (uart->Instance != UART7) ||
        (uart->hdmarx == NULL) || (uart->hdmatx == NULL) ||
        (uart->hdmarx->Init.Mode != DMA_CIRCULAR))
    {
        return HAL_ERROR;
    }
    if (lora_initialized)
    {
        return (lora_uart == uart) ? HAL_OK : HAL_ERROR;
    }

    lora_uart = uart;
    (void)memset(lora_rx_buffer, 0, sizeof(lora_rx_buffer));
    (void)memset(lora_tx_buffer, 0, sizeof(lora_tx_buffer));
    lora_restart_requested = false;
    lora_tx_busy = false;
    lora_link_rx_bytes = 0U;
    lora_link_uart_error_count = 0U;
    lora_link_tx_error_count = 0U;
    lora_link_valid_frame_count = 0U;
    lora_link_forward_error_count = 0U;
    lora_link_return_error_count = 0U;
    lora_link_returned_bytes = 0U;
    lora_remote_buttons = 0U;
    lora_remote_pe0_switch = 0U;
    lora_remote_pd6_switch = 0U;
    lora_remote_online = 0U;
    remote_pe0_initialized = false;
    remote_pe0_last = 0U;
    LoraLink_ResetRemoteFrame();
    remote_last_rx_ms = 0U;
    mcu_return_length = 0U;

    if (LoraLink_StartReceive() != HAL_OK)
    {
        lora_uart = NULL;
        return HAL_ERROR;
    }

    lora_initialized = true;
    return HAL_OK;
}

void LoraLink_Run(void)
{
    uint8_t data[LORA_READ_BUFFER_SIZE];
    uint16_t count;
    uint16_t i;

    if (!lora_initialized)
    {
        return;
    }
    if (lora_restart_requested)
    {
        lora_restart_requested = false;
        (void)HAL_UART_AbortReceive(lora_uart);
        (void)HAL_UART_AbortTransmit(lora_uart);
        lora_tx_busy = false;
        LoraLink_ResetRemoteFrame();
        if (LoraLink_StartReceive() != HAL_OK)
        {
            lora_restart_requested = true;
            return;
        }
    }

    do
    {
        count = LoraLink_Read(data, sizeof(data));
        for (i = 0U; i < count; i++)
        {
            LoraLink_ParseByte(data[i]);
        }
    } while (count == sizeof(data));

    LoraLink_ForwardMcuData();

    if ((lora_remote_online != 0U) &&
        ((HAL_GetTick() - remote_last_rx_ms) > REMOTE_TIMEOUT_MS))
    {
        lora_remote_online = 0U;
        lora_remote_buttons = 0U;
        lora_remote_pe0_switch = 0U;
        lora_remote_pd6_switch = 0U;
        remote_pe0_initialized = false;
        Chassis_ReleaseVelocity(CHASSIS_CMD_SOURCE_LORA);
    }
}

uint16_t LoraLink_Read(uint8_t *data, uint16_t max_length)
{
    uint16_t count = 0U;
    uint16_t write_pos;

    if (!lora_initialized || (data == NULL) || (max_length == 0U))
    {
        return 0U;
    }

    write_pos = (uint16_t)(LORA_LINK_DMA_BUFFER_SIZE -
                           __HAL_DMA_GET_COUNTER(lora_uart->hdmarx));
    if (write_pos >= LORA_LINK_DMA_BUFFER_SIZE)
    {
        write_pos = 0U;
    }
    __DMB();
    while ((count < max_length) && (lora_rx_read_pos != write_pos))
    {
        data[count] = lora_rx_buffer[lora_rx_read_pos];
        count++;
        lora_rx_read_pos++;
        if (lora_rx_read_pos >= LORA_LINK_DMA_BUFFER_SIZE)
        {
            lora_rx_read_pos = 0U;
        }
    }

    lora_link_rx_bytes += count;
    return count;
}

HAL_StatusTypeDef LoraLink_Send(const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;

    if (!lora_initialized || (data == NULL) || (length == 0U) ||
        (length > LORA_LINK_TX_BUFFER_SIZE))
    {
        return HAL_ERROR;
    }
    if (lora_tx_busy)
    {
        return HAL_BUSY;
    }

    (void)memcpy(lora_tx_buffer, data, length);
    lora_tx_busy = true;
    status = HAL_UART_Transmit_DMA(lora_uart, lora_tx_buffer, length);
    if (status != HAL_OK)
    {
        lora_tx_busy = false;
        lora_link_tx_error_count++;
    }
    return status;
}

void LoraLink_HandleTxCplt(UART_HandleTypeDef *uart)
{
    if (lora_initialized && (uart == lora_uart))
    {
        lora_tx_busy = false;
    }
}

void LoraLink_HandleUartError(UART_HandleTypeDef *uart)
{
    if (!lora_initialized || (uart != lora_uart))
    {
        return;
    }

    lora_link_uart_error_count++;
    lora_restart_requested = true;
}
