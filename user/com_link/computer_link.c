#include "computer_link.h"

#include "action_api.h"
#include "chassis_main.h"
#include "dt35_pnp_link.h"
#include "imu_main.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* 上位机到控制器的数据帧格式。 */
#define COMPUTER_FRAME_HEADER_0   0xA5U
#define COMPUTER_VELOCITY_HEADER  0x5AU
#define COMPUTER_ACTION_HEADER    0x5BU
#define COMPUTER_VELOCITY_LENGTH  9U
#define COMPUTER_ACTION_LENGTH    3U
#define COMPUTER_MAX_FRAME_LENGTH 9U
#define COMPUTER_LINK_TIMEOUT_MS  500U

typedef enum
{
    COMPUTER_RX_HEADER_0,
    COMPUTER_RX_HEADER_1,
    COMPUTER_RX_FRAME
} computer_rx_state_t;

typedef struct
{
    int16_t vx;
    int16_t vy;
    int16_t z;
} computer_cmd_t;

static UART_HandleTypeDef *computer_uart;
static uint8_t rx_byte;
static uint8_t rx_frame[COMPUTER_MAX_FRAME_LENGTH];
static uint8_t rx_index;
static uint8_t rx_length;
static computer_rx_state_t rx_state;
static volatile computer_cmd_t pending_cmd;
static volatile uint8_t pending_action;
static volatile uint32_t last_rx_ms;
static volatile bool cmd_pending;
static volatile bool action_frame_pending;
static volatile bool link_online;
static volatile bool restart_requested;
static bool computer_link_initialized;

static void reset_parser(void)
{
    rx_index = 0U;
    rx_length = 0U;
    rx_state = COMPUTER_RX_HEADER_0;
}

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

static int16_t read_le_i16(const uint8_t *data)
{
    uint16_t value;

    value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return (int16_t)value;
}

static void store_command(void)
{
    pending_cmd.vx = read_le_i16(&rx_frame[2]);
    pending_cmd.vy = read_le_i16(&rx_frame[4]);
    pending_cmd.z = read_le_i16(&rx_frame[6]);
    last_rx_ms = HAL_GetTick();
    cmd_pending = true;
    link_online = true;
}

static void store_action(void)
{
    if (rx_frame[2] > ACTION_CMD_MAX)
    {
        return;
    }

    pending_action = rx_frame[2];
    action_frame_pending = true;
}

static void parse_byte(uint8_t data)
{
    switch (rx_state)
    {
    case COMPUTER_RX_HEADER_0:
        if (data == COMPUTER_FRAME_HEADER_0)
        {
            rx_frame[0] = data;
            rx_state = COMPUTER_RX_HEADER_1;
        }
        break;

    case COMPUTER_RX_HEADER_1:
        if ((data == COMPUTER_VELOCITY_HEADER) ||
            (data == COMPUTER_ACTION_HEADER))
        {
            rx_frame[1] = data;
            rx_index = 2U;
            rx_length = (data == COMPUTER_VELOCITY_HEADER)
                            ? COMPUTER_VELOCITY_LENGTH
                            : COMPUTER_ACTION_LENGTH;    //速度帧9字节，动作帧3字节
            rx_state = COMPUTER_RX_FRAME;
        }
        else if (data != COMPUTER_FRAME_HEADER_0)   // 不是 0xA5 也不是有效帧头，重置
        {
            reset_parser();
        }
        break;

    case COMPUTER_RX_FRAME:
        rx_frame[rx_index] = data;
        rx_index++;
        if (rx_index >= rx_length)
        {
            if ((rx_frame[1] == COMPUTER_VELOCITY_HEADER) &&
                (rx_frame[8] == calculate_checksum(&rx_frame[2], 6U)))
            {
                store_command();
            }
            else if (rx_frame[1] == COMPUTER_ACTION_HEADER)
            {
                store_action();
            }
            reset_parser();
        }
        break;

    default:
        reset_parser();
        break;
    }
}

static HAL_StatusTypeDef start_receive(void)
{
    if (computer_uart == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Receive_IT(computer_uart, &rx_byte, 1U);
}

static void restart_receive(void)
{
    (void)HAL_UART_AbortReceive(computer_uart);
    reset_parser();
    if (start_receive() != HAL_OK)
    {
        restart_requested = true;
    }
}

HAL_StatusTypeDef ComputerLink_Init(UART_HandleTypeDef *uart)
{
    HAL_StatusTypeDef status;

    if (uart == NULL)
    {
        return HAL_ERROR;
    }
    if (computer_link_initialized)
    {
        return (computer_uart == uart) ? HAL_OK : HAL_ERROR;
    }

    computer_uart = uart;
    rx_byte = 0U;
    (void)memset(rx_frame, 0, sizeof(rx_frame));
    pending_cmd.vx = 0;
    pending_cmd.vy = 0;
    pending_cmd.z = 0;
    pending_action = ACTION_CMD_NONE;
    last_rx_ms = 0U;
    cmd_pending = false;
    action_frame_pending = false;
    link_online = false;
    restart_requested = false;
    reset_parser();
    computer_link_initialized = true;

    status = start_receive();
    if (status != HAL_OK)
    {
        restart_requested = true;
    }

    return status;
}

void ComputerLink_Run(void)
{
    computer_cmd_t cmd;
    uint8_t action = ACTION_CMD_NONE;
    uint32_t now_ms;
    uint32_t primask;
    bool has_command = false;
    bool has_action = false;

    if (computer_uart == NULL)
    {
        return;
    }

    if (restart_requested)
    {
        restart_requested = false;
        restart_receive();
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (cmd_pending)
    {
        cmd.vx = pending_cmd.vx;
        cmd.vy = pending_cmd.vy;
        cmd.z = pending_cmd.z;
        cmd_pending = false;
        has_command = true;
    }
    if (action_frame_pending)
    {
        action = pending_action;
        action_frame_pending = false;
        has_action = true;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    if (has_command)
    {
        (void)Chassis_SetVelocity(cmd.vx, cmd.vy, cmd.z);
    }
    if (has_action)
    {
        (void)Action_Request((action_cmd_t)action);
    }

    (void)ImuMain_SendYaw(computer_uart);
    DT35PnpLink_Send(computer_uart);

    now_ms = HAL_GetTick();
    if (link_online && ((now_ms - last_rx_ms) > COMPUTER_LINK_TIMEOUT_MS))
    {
        link_online = false;
        Chassis_StopAll();
    }
}

void ComputerLink_RxCplt(UART_HandleTypeDef *uart)
{
    if ((computer_uart == NULL) || (uart != computer_uart))
    {
        return;
    }

    parse_byte(rx_byte);
    if (start_receive() != HAL_OK)
    {
        restart_requested = true;
    }
}

void ComputerLink_Error(UART_HandleTypeDef *uart)
{
    if ((computer_uart == NULL) || (uart != computer_uart))
    {
        return;
    }

    restart_requested = true;
}
