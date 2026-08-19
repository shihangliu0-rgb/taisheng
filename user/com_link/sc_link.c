#include "sc_link.h"

#include "up_main.h"

#include <stddef.h>
#include <string.h>

#define SC_LINK_HEADER_0             0xAAU
#define SC_LINK_HEADER_1             0x55U
#define SC_LINK_TAIL_0               0x0DU
#define SC_LINK_TAIL_1               0x0AU
#define SC_LINK_PERCEPTION_TYPE      0x10U
#define SC_LINK_POSE_TYPE             0x11U
#define SC_LINK_STATUS_TYPE           0x20U
#define SC_LINK_PERCEPTION_LENGTH     37U
#define SC_LINK_POSE_LENGTH            29U
#define SC_LINK_STATUS_LENGTH           9U
#define SC_LINK_MAX_FRAME_LENGTH      SC_LINK_PERCEPTION_LENGTH
#define SC_LINK_READ_CHUNK_SIZE       64U

typedef enum
{
    SC_RX_HEADER_0,
    SC_RX_HEADER_1,
    SC_RX_FRAME
} sc_rx_state_t;

static UART_HandleTypeDef *sc_uart;
static uint8_t sc_rx_dma[SC_LINK_RX_BUFFER_SIZE];
static uint8_t sc_tx_buffer[SC_LINK_TX_BUFFER_SIZE];
static uint16_t sc_rx_read_pos;
static uint8_t sc_frame[SC_LINK_MAX_FRAME_LENGTH];
static uint16_t sc_frame_index;
static uint16_t sc_frame_length;
static sc_rx_state_t sc_rx_state;
static volatile bool sc_restart_requested;
static volatile bool sc_tx_busy;
static bool sc_initialized;
static uint32_t sc_last_status_ms;
static uint8_t sc_status_state;
static uint8_t sc_status_error;
static bool sc_status_override;
static sc_link_perception_t sc_perception;
static sc_link_pose_t sc_pose;
static uint32_t sc_block_last_update_ms;
static uint32_t sc_ball_last_update_ms;
static uint32_t sc_pose_last_update_ms;

volatile uint32_t sc_link_rx_bytes;
volatile uint32_t sc_link_uart_error_count;
volatile uint32_t sc_link_tx_error_count;
volatile uint32_t sc_link_valid_frame_count;
volatile uint32_t sc_link_invalid_frame_count;

static uint16_t ScLink_Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t i;
    uint8_t bit;

    for (i = 0U; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8U;
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }
    return crc;
}

static uint16_t ScLink_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t ScLink_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static float ScLink_ReadFloat(const uint8_t *data)
{
    float value;

    (void)memcpy(&value, data, sizeof(value));
    return value;
}

static void ScLink_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void ScLink_ResetParser(void)
{
    sc_frame_index = 0U;
    sc_frame_length = 0U;
    sc_rx_state = SC_RX_HEADER_0;
}

static void ScLink_ClearBlock(void)
{
    sc_perception.flags &= (uint8_t)~SC_LINK_FLAG_BLOCK_VALID;
    sc_perception.block_valid = false;
    sc_perception.block_x_m = 0.0f;
    sc_perception.block_y_m = 0.0f;
    sc_perception.block_z_m = 0.0f;
}

static void ScLink_ClearBall(void)
{
    sc_perception.flags &= (uint8_t)~SC_LINK_FLAG_BALL_VALID;
    sc_perception.ball_valid = false;
    sc_perception.ball_x_m = 0.0f;
    sc_perception.ball_y_m = 0.0f;
    sc_perception.ball_z_m = 0.0f;
}

static void ScLink_ClearPose(void)
{
    sc_pose.flags &= (uint8_t)~SC_LINK_FLAG_POSE_VALID;
    sc_pose.valid = false;
    sc_pose.field_x_m = 0.0f;
    sc_pose.field_y_m = 0.0f;
    sc_pose.field_z_m = 0.0f;
    sc_pose.field_yaw = 0.0f;
}

static bool ScLink_ValidateFrame(void)
{
    uint16_t crc_received;
    uint16_t crc_expected;

    if ((sc_frame_length < SC_LINK_STATUS_LENGTH) ||
        (sc_frame[sc_frame_length - 2U] != SC_LINK_TAIL_0) ||
        (sc_frame[sc_frame_length - 1U] != SC_LINK_TAIL_1))
    {
        return false;
    }

    crc_received = ScLink_ReadU16(&sc_frame[sc_frame_length - 4U]);
    crc_expected = ScLink_Crc16(&sc_frame[2], sc_frame_length - 6U);
    return crc_received == crc_expected;
}

static void ScLink_HandlePerception(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint8_t flags = sc_frame[4];

    sc_perception.sequence = sc_frame[3];
    sc_perception.flags = flags;
    sc_perception.timestamp_ms = ScLink_ReadU32(&sc_frame[5]);
    sc_perception.received_ms = now_ms;

    if ((flags & SC_LINK_FLAG_BLOCK_VALID) != 0U)
    {
        sc_perception.block_x_m = ScLink_ReadFloat(&sc_frame[9]);
        sc_perception.block_y_m = ScLink_ReadFloat(&sc_frame[13]);
        sc_perception.block_z_m = ScLink_ReadFloat(&sc_frame[17]);
        sc_perception.block_valid = true;
        sc_block_last_update_ms = now_ms;
    }
    else
    {
        ScLink_ClearBlock();
    }

    if ((flags & SC_LINK_FLAG_BALL_VALID) != 0U)
    {
        sc_perception.ball_x_m = ScLink_ReadFloat(&sc_frame[21]);
        sc_perception.ball_y_m = ScLink_ReadFloat(&sc_frame[25]);
        sc_perception.ball_z_m = ScLink_ReadFloat(&sc_frame[29]);
        sc_perception.ball_valid = true;
        sc_ball_last_update_ms = now_ms;
    }
    else
    {
        ScLink_ClearBall();
    }
}

static void ScLink_HandlePose(void)
{
    uint32_t now_ms = HAL_GetTick();

    sc_pose.sequence = sc_frame[3];
    sc_pose.flags = sc_frame[4];
    sc_pose.timestamp_ms = ScLink_ReadU32(&sc_frame[5]);
    sc_pose.received_ms = now_ms;
    if ((sc_pose.flags & SC_LINK_FLAG_POSE_VALID) != 0U)
    {
        sc_pose.field_x_m = ScLink_ReadFloat(&sc_frame[9]);
        sc_pose.field_y_m = ScLink_ReadFloat(&sc_frame[13]);
        sc_pose.field_z_m = ScLink_ReadFloat(&sc_frame[17]);
        sc_pose.field_yaw = ScLink_ReadFloat(&sc_frame[21]);
        sc_pose.valid = true;
        sc_pose_last_update_ms = now_ms;
    }
    else
    {
        ScLink_ClearPose();
    }
}

static void ScLink_HandleFrame(void)
{
    if (!ScLink_ValidateFrame())
    {
        sc_link_invalid_frame_count++;
        return;
    }

    if ((sc_frame[2] == SC_LINK_PERCEPTION_TYPE) &&
        (sc_frame_length == SC_LINK_PERCEPTION_LENGTH))
    {
        ScLink_HandlePerception();
    }
    else if ((sc_frame[2] == SC_LINK_POSE_TYPE) &&
             (sc_frame_length == SC_LINK_POSE_LENGTH))
    {
        ScLink_HandlePose();
    }
    else
    {
        sc_link_invalid_frame_count++;
        return;
    }
    sc_link_valid_frame_count++;
}

static void ScLink_ParseByte(uint8_t data)
{
    switch (sc_rx_state)
    {
    case SC_RX_HEADER_0:
        if (data == SC_LINK_HEADER_0)
        {
            sc_frame[0] = data;
            sc_rx_state = SC_RX_HEADER_1;
        }
        break;

    case SC_RX_HEADER_1:
        if (data == SC_LINK_HEADER_1)
        {
            sc_frame[1] = data;
            sc_frame_index = 2U;
            sc_frame_length = 0U;
            sc_rx_state = SC_RX_FRAME;
        }
        else if (data != SC_LINK_HEADER_0)
        {
            ScLink_ResetParser();
        }
        break;

    case SC_RX_FRAME:
        if (sc_frame_index >= SC_LINK_MAX_FRAME_LENGTH)
        {
            ScLink_ResetParser();
            break;
        }
        sc_frame[sc_frame_index++] = data;
        if (sc_frame_index == 3U)
        {
            if (data == SC_LINK_PERCEPTION_TYPE)
            {
                sc_frame_length = SC_LINK_PERCEPTION_LENGTH;
            }
            else if (data == SC_LINK_POSE_TYPE)
            {
                sc_frame_length = SC_LINK_POSE_LENGTH;
            }
            else
            {
                ScLink_ResetParser();
            }
        }
        if ((sc_frame_length != 0U) &&
            (sc_frame_index >= sc_frame_length))
        {
            ScLink_HandleFrame();
            ScLink_ResetParser();
        }
        break;

    default:
        ScLink_ResetParser();
        break;
    }
}

static HAL_StatusTypeDef ScLink_StartReceive(void)
{
    HAL_StatusTypeDef status;

    if ((sc_uart == NULL) || (sc_uart->hdmarx == NULL))
    {
        return HAL_ERROR;
    }
    sc_rx_read_pos = 0U;
    __HAL_UART_CLEAR_FLAG(sc_uart,
                          UART_CLEAR_OREF | UART_CLEAR_NEF |
                          UART_CLEAR_PEF | UART_CLEAR_FEF);
    __HAL_UART_SEND_REQ(sc_uart, UART_RXDATA_FLUSH_REQUEST);
    status = HAL_UART_Receive_DMA(sc_uart, sc_rx_dma, SC_LINK_RX_BUFFER_SIZE);
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(sc_uart->hdmarx, DMA_IT_HT | DMA_IT_TC);
    }
    return status;
}

static uint16_t ScLink_Read(uint8_t *data, uint16_t max_length)
{
    uint16_t count = 0U;
    uint16_t write_pos;

    if (!sc_initialized || (data == NULL) || (max_length == 0U))
    {
        return 0U;
    }

    write_pos = (uint16_t)(SC_LINK_RX_BUFFER_SIZE -
                           __HAL_DMA_GET_COUNTER(sc_uart->hdmarx));
    if (write_pos >= SC_LINK_RX_BUFFER_SIZE)
    {
        write_pos = 0U;
    }
    __DMB();
    while ((count < max_length) && (sc_rx_read_pos != write_pos))
    {
        data[count++] = sc_rx_dma[sc_rx_read_pos++];
        if (sc_rx_read_pos >= SC_LINK_RX_BUFFER_SIZE)
        {
            sc_rx_read_pos = 0U;
        }
    }
    sc_link_rx_bytes += count;
    return count;
}

static HAL_StatusTypeDef ScLink_SendStatus(void)
{
    uint16_t crc;

    if (sc_tx_busy)
    {
        return HAL_BUSY;
    }

    sc_tx_buffer[0] = SC_LINK_HEADER_1;
    sc_tx_buffer[1] = SC_LINK_HEADER_0;
    sc_tx_buffer[2] = SC_LINK_STATUS_TYPE;
    sc_tx_buffer[3] = sc_status_state;
    sc_tx_buffer[4] = sc_status_error;
    crc = ScLink_Crc16(&sc_tx_buffer[2], 3U);
    ScLink_WriteU16(&sc_tx_buffer[5], crc);
    sc_tx_buffer[7] = SC_LINK_TAIL_0;
    sc_tx_buffer[8] = SC_LINK_TAIL_1;
    sc_tx_busy = true;
    if (HAL_UART_Transmit_DMA(sc_uart, sc_tx_buffer, SC_LINK_STATUS_LENGTH) != HAL_OK)
    {
        sc_tx_busy = false;
        sc_link_tx_error_count++;
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef ScLink_Init(UART_HandleTypeDef *uart)
{
    if ((uart == NULL) || (uart->Instance != UART8) ||
        (uart->Init.BaudRate != SC_LINK_BAUDRATE) ||
        (uart->hdmarx == NULL) || (uart->hdmatx == NULL) ||
        (uart->hdmarx->Init.Mode != DMA_CIRCULAR))
    {
        return HAL_ERROR;
    }
    if (sc_initialized)
    {
        return (sc_uart == uart) ? HAL_OK : HAL_ERROR;
    }

    sc_uart = uart;
    (void)memset(sc_rx_dma, 0, sizeof(sc_rx_dma));
    (void)memset(sc_tx_buffer, 0, sizeof(sc_tx_buffer));
    (void)memset(&sc_perception, 0, sizeof(sc_perception));
    (void)memset(&sc_pose, 0, sizeof(sc_pose));
    sc_link_rx_bytes = 0U;
    sc_link_uart_error_count = 0U;
    sc_link_tx_error_count = 0U;
    sc_link_valid_frame_count = 0U;
    sc_link_invalid_frame_count = 0U;
    sc_status_state = 0U;
    sc_status_error = 0U;
    sc_status_override = false;
    sc_block_last_update_ms = 0U;
    sc_ball_last_update_ms = 0U;
    sc_pose_last_update_ms = 0U;
    sc_last_status_ms = HAL_GetTick();
    sc_restart_requested = false;
    sc_tx_busy = false;
    ScLink_ResetParser();

    if (ScLink_StartReceive() != HAL_OK)
    {
        sc_uart = NULL;
        return HAL_ERROR;
    }
    sc_initialized = true;
    return HAL_OK;
}

void ScLink_Run(void)
{
    uint8_t data[SC_LINK_READ_CHUNK_SIZE];
    uint16_t count;
    uint16_t i;
    uint32_t now_ms;

    if (!sc_initialized)
    {
        return;
    }
    if (sc_restart_requested)
    {
        sc_restart_requested = false;
        (void)HAL_UART_AbortReceive(sc_uart);
        (void)HAL_UART_AbortTransmit(sc_uart);
        sc_tx_busy = false;
        ScLink_ResetParser();
        if (ScLink_StartReceive() != HAL_OK)
        {
            sc_restart_requested = true;
            return;
        }
    }

    do
    {
        count = ScLink_Read(data, sizeof(data));
        for (i = 0U; i < count; i++)
        {
            ScLink_ParseByte(data[i]);
        }
    } while (count == sizeof(data));

    now_ms = HAL_GetTick();
    if (sc_perception.block_valid &&
        ((now_ms - sc_block_last_update_ms) > SC_LINK_DATA_TIMEOUT_MS))
    {
        ScLink_ClearBlock();
    }
    if (sc_perception.ball_valid &&
        ((now_ms - sc_ball_last_update_ms) > SC_LINK_DATA_TIMEOUT_MS))
    {
        ScLink_ClearBall();
    }
    if (sc_pose.valid &&
        ((now_ms - sc_pose_last_update_ms) > SC_LINK_DATA_TIMEOUT_MS))
    {
        ScLink_ClearPose();
    }

    if ((now_ms - sc_last_status_ms) >= SC_LINK_STATUS_PERIOD_MS)
    {
        sc_last_status_ms = now_ms;
        if (!sc_status_override)
        {
            sc_status_state = (uint8_t)up_state;
            if (up_state == UP_STATE_ERROR)
            {
                sc_status_error = (up_last_result == HAL_OK)
                                      ? 1U
                                      : (uint8_t)up_last_result;
            }
            else
            {
                sc_status_error = 0U;
            }
        }
        (void)ScLink_SendStatus();
    }
}

bool ScLink_GetPerception(sc_link_perception_t *perception)
{
    uint32_t primask;

    if (!sc_initialized || (perception == NULL))
    {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    (void)memcpy(perception, &sc_perception, sizeof(*perception));
    if (primask == 0U)
    {
        __enable_irq();
    }
    return true;
}

bool ScLink_GetPose(sc_link_pose_t *pose)
{
    uint32_t primask;

    if (!sc_initialized || (pose == NULL))
    {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    (void)memcpy(pose, &sc_pose, sizeof(*pose));
    if (primask == 0U)
    {
        __enable_irq();
    }
    return true;
}

void ScLink_SetStatus(uint8_t state, uint8_t error)
{
    sc_status_state = state;
    sc_status_error = error;
    sc_status_override = true;
}

void ScLink_HandleTxCplt(UART_HandleTypeDef *uart)
{
    if (sc_initialized && (uart == sc_uart))
    {
        sc_tx_busy = false;
    }
}

void ScLink_HandleUartError(UART_HandleTypeDef *uart)
{
    if (!sc_initialized || (uart != sc_uart))
    {
        return;
    }
    sc_link_uart_error_count++;
    sc_restart_requested = true;
}
