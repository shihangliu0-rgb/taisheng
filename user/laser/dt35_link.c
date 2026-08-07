#include "dt35_link.h"

/* DT35 串口帧格式和通信超时。 */
#define DT35_LINK_FRAME_HEADER   0xAAU
#define DT35_LINK_FRAME_LENGTH   5U
#define DT35_LINK_TIMEOUT_MS     500U
#define DT35_LINK_TX_TIMEOUT_MS  20U

volatile uint16_t dt35_distance_40_cm;
volatile uint16_t dt35_distance_41_cm;
volatile uint8_t dt35_online_40;
volatile uint8_t dt35_online_41;

static UART_HandleTypeDef *dt35_uart;
static uint8_t rx_byte;
static uint8_t rx_frame[DT35_LINK_FRAME_LENGTH];
static uint8_t rx_index;
static volatile uint8_t frame_pending_40;
static volatile uint8_t frame_pending_41;
static volatile uint32_t last_rx_40_ms;
static volatile uint32_t last_rx_41_ms;
static volatile uint8_t restart_requested;

static uint8_t calculate_checksum(const uint8_t *data, uint8_t length)
{
    uint8_t checksum = 0U;
    uint8_t i;

    for (i = 0U; i < length; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

static void reset_parser(void)
{
    rx_index = 0U;
}

static void store_frame(void)
{
    uint16_t distance_cm;
    uint8_t address;

    address = rx_frame[1];
    distance_cm = (uint16_t)rx_frame[2] |
                  ((uint16_t)rx_frame[3] << 8U);

    if (address == DT35_LINK_ADDR_40)
    {
        dt35_distance_40_cm = distance_cm;
        dt35_online_40 = 1U;
        last_rx_40_ms = HAL_GetTick();
        frame_pending_40 = 1U;
    }
    else if (address == DT35_LINK_ADDR_41)
    {
        dt35_distance_41_cm = distance_cm;
        dt35_online_41 = 1U;
        last_rx_41_ms = HAL_GetTick();
        frame_pending_41 = 1U;
    }
}

static void parse_byte(uint8_t data)
{
    if (rx_index == 0U)
    {
        if (data == DT35_LINK_FRAME_HEADER)
        {
            rx_frame[0] = data;
            rx_index = 1U;
        }
        return;
    }

    rx_frame[rx_index] = data;
    rx_index++;
    if (rx_index < DT35_LINK_FRAME_LENGTH)
    {
        return;
    }

    if ((rx_frame[1] == DT35_LINK_ADDR_40 ||
         rx_frame[1] == DT35_LINK_ADDR_41) &&
        (rx_frame[4] == calculate_checksum(rx_frame, 4U)))
    {
        store_frame();
    }
    reset_parser();
}

static HAL_StatusTypeDef start_receive(void)
{
    if (dt35_uart == NULL)
    {
        return HAL_ERROR;
    }
    return HAL_UART_Receive_IT(dt35_uart, &rx_byte, 1U);
}

HAL_StatusTypeDef DT35Link_Init(UART_HandleTypeDef *uart)
{
    if (uart == NULL)
    {
        return HAL_ERROR;
    }

    dt35_uart = uart;
    dt35_distance_40_cm = 0U;
    dt35_distance_41_cm = 0U;
    dt35_online_40 = 0U;
    dt35_online_41 = 0U;
    frame_pending_40 = 0U;
    frame_pending_41 = 0U;
    last_rx_40_ms = 0U;
    last_rx_41_ms = 0U;
    restart_requested = 0U;
    reset_parser();

    if (start_receive() != HAL_OK)
    {
        restart_requested = 1U;
        return HAL_ERROR;
    }
    return HAL_OK;
}

void DT35Link_Run(void)
{
    uint32_t now_ms;

    if (dt35_uart == NULL)
    {
        return;
    }

    if (restart_requested != 0U)
    {
        restart_requested = 0U;
        reset_parser();
        if (start_receive() != HAL_OK)
        {
            restart_requested = 1U;
        }
    }

    now_ms = HAL_GetTick();
    if ((dt35_online_40 != 0U) &&
        ((uint32_t)(now_ms - last_rx_40_ms) > DT35_LINK_TIMEOUT_MS))
    {
        dt35_online_40 = 0U;
    }
    if ((dt35_online_41 != 0U) &&
        ((uint32_t)(now_ms - last_rx_41_ms) > DT35_LINK_TIMEOUT_MS))
    {
        dt35_online_41 = 0U;
    }
}

void DT35Link_Send(UART_HandleTypeDef *uart)
{
    uint8_t send_40;
    uint8_t send_41;
    uint16_t distance_40;
    uint16_t distance_41;
    uint8_t frame[DT35_LINK_FRAME_LENGTH];
    uint32_t primask;

    if (uart == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    send_40 = frame_pending_40;
    send_41 = frame_pending_41;
    distance_40 = dt35_distance_40_cm;
    distance_41 = dt35_distance_41_cm;
    frame_pending_40 = 0U;
    frame_pending_41 = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    if (send_41 != 0U)
    {
        frame[0] = DT35_LINK_FRAME_HEADER;
        frame[1] = DT35_LINK_ADDR_41;
        frame[2] = (uint8_t)distance_41;
        frame[3] = (uint8_t)(distance_41 >> 8U);
        frame[4] = calculate_checksum(frame, 4U);
        (void)HAL_UART_Transmit(uart, frame, sizeof(frame),
                                DT35_LINK_TX_TIMEOUT_MS);
    }

    if (send_40 != 0U)
    {
        frame[0] = DT35_LINK_FRAME_HEADER;
        frame[1] = DT35_LINK_ADDR_40;
        frame[2] = (uint8_t)distance_40;
        frame[3] = (uint8_t)(distance_40 >> 8U);
        frame[4] = calculate_checksum(frame, 4U);
        (void)HAL_UART_Transmit(uart, frame, sizeof(frame),
                                DT35_LINK_TX_TIMEOUT_MS);
    }
}

void DT35Link_RxCplt(UART_HandleTypeDef *uart)
{
    if ((dt35_uart == NULL) || (uart != dt35_uart))
    {
        return;
    }

    parse_byte(rx_byte);
    if (start_receive() != HAL_OK)
    {
        restart_requested = 1U;
    }
}

void DT35Link_Error(UART_HandleTypeDef *uart)
{
    if ((dt35_uart != NULL) && (uart == dt35_uart))
    {
        restart_requested = 1U;
    }
}
