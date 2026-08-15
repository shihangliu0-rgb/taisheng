#include "computer_link.h"

#include "action_api.h"
#include "dt35_pnp_link.h"
#include "imu_main.h"

#include <stddef.h>
#include <string.h>

/*
 * 上位机到控制器的数据帧格式:
 *   速度帧  A5 5A | vx(le16) | vy(le16) | z(le16) | checksum    (9B)
 *   动作帧  A5 5B | action                                       (3B)
 *   急停帧  A5 5D | sub(0x01 锁存 / 0x00 清除复位)                (3B)
 *   模式帧  A5 5E | mode(0x01 手动 / 0x00 恢复自主)               (3B)
 * 本模块只负责解析与提交,不直接写底盘;底盘指令由 PathRunner_Arbitrate
 * 按"急停 > 人工 > 自主"优先级统一仲裁。
 */
#define COMPUTER_FRAME_HEADER_0   0xA5U
#define COMPUTER_VELOCITY_HEADER  0x5AU
#define COMPUTER_ACTION_HEADER    0x5BU
#define COMPUTER_ESTOP_HEADER     0x5DU
#define COMPUTER_MODE_HEADER      0x5EU
#define COMPUTER_VELOCITY_LENGTH  9U
#define COMPUTER_ACTION_LENGTH    3U
#define COMPUTER_ESTOP_LENGTH     3U
#define COMPUTER_MAX_FRAME_LENGTH 9U
#define COMPUTER_LINK_TIMEOUT_MS  500U

#define COMPUTER_ESTOP_ENGAGE     0x01U
#define COMPUTER_ESTOP_CLEAR      0x00U
#define COMPUTER_MODE_MANUAL      0x01U
#define COMPUTER_MODE_AUTO        0x00U

typedef enum
{
    COMPUTER_RX_HEADER_0,
    COMPUTER_RX_HEADER_1,
    COMPUTER_RX_FRAME
} computer_rx_state_t;

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

/* 安全/模式信号:ISR 置位,任务侧消费 */
static volatile bool estop_latched;
static volatile bool estop_clear_edge;
static volatile bool mode_manual_edge;
static volatile bool mode_auto_edge;

/* 每周期提交给仲裁器的手动指令(commTask 单消费者) */
static computer_cmd_t manual_cmd;
static uint8_t manual_action;
static bool manual_cmd_valid;
static bool manual_action_valid;

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

static void touch_link(void)
{
    last_rx_ms = HAL_GetTick();
    link_online = true;
}

static void store_command(void)
{
    pending_cmd.vx = read_le_i16(&rx_frame[2]);
    pending_cmd.vy = read_le_i16(&rx_frame[4]);
    pending_cmd.z = read_le_i16(&rx_frame[6]);
    touch_link();
    cmd_pending = true;
}

static void store_action(void)
{
    if (rx_frame[2] > ACTION_CMD_REAR_DOWN)
    {
        return;
    }

    pending_action = rx_frame[2];
    touch_link();
    action_frame_pending = true;
}

static void store_estop(uint8_t sub)
{
    touch_link();
    if (sub == COMPUTER_ESTOP_ENGAGE)
    {
        estop_latched = true;
    }
    else if (sub == COMPUTER_ESTOP_CLEAR)
    {
        estop_latched = false;
        estop_clear_edge = true;
    }
}

static void store_mode(uint8_t mode)
{
    touch_link();
    if (mode == COMPUTER_MODE_MANUAL)
    {
        mode_manual_edge = true;
    }
    else if (mode == COMPUTER_MODE_AUTO)
    {
        mode_auto_edge = true;
    }
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
            (data == COMPUTER_ACTION_HEADER) ||
            (data == COMPUTER_ESTOP_HEADER) ||
            (data == COMPUTER_MODE_HEADER))
        {
            rx_frame[1] = data;
            rx_index = 2U;
            rx_length = (data == COMPUTER_VELOCITY_HEADER)
                            ? COMPUTER_VELOCITY_LENGTH
                            : COMPUTER_ACTION_LENGTH;   /* 0x5B/0x5D/0x5E 均为 3B */
            rx_state = COMPUTER_RX_FRAME;
        }
        else if (data != COMPUTER_FRAME_HEADER_0)
        {
            /* 不是 0xA5 也不是有效帧头,重置 */
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
            else if (rx_frame[1] == COMPUTER_ESTOP_HEADER)
            {
                store_estop(rx_frame[2]);
            }
            else if (rx_frame[1] == COMPUTER_MODE_HEADER)
            {
                store_mode(rx_frame[2]);
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
    estop_latched = false;
    estop_clear_edge = false;
    mode_manual_edge = false;
    mode_auto_edge = false;
    manual_cmd.vx = 0;
    manual_cmd.vy = 0;
    manual_cmd.z = 0;
    manual_action = ACTION_CMD_NONE;
    manual_cmd_valid = false;
    manual_action_valid = false;
    reset_parser();

    status = start_receive();
    if (status != HAL_OK)
    {
        restart_requested = true;
    }

    return status;
}

void ComputerLink_Run(void)
{
    uint32_t now_ms;
    uint32_t primask;

    if (computer_uart == NULL)
    {
        return;
    }

    if (restart_requested)
    {
        restart_requested = false;
        restart_receive();
    }

    /* 从 ISR 缓存取出本周期手动速度/动作(不写底盘) */
    primask = __get_PRIMASK();
    __disable_irq();
    if (cmd_pending)
    {
        manual_cmd.vx = pending_cmd.vx;
        manual_cmd.vy = pending_cmd.vy;
        manual_cmd.z = pending_cmd.z;
        cmd_pending = false;
        manual_cmd_valid = true;
    }
    if (action_frame_pending)
    {
        manual_action = pending_action;
        action_frame_pending = false;
        manual_action_valid = true;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    /* 遥测回传 */
    (void)ImuMain_SendYaw(computer_uart);
    DT35PnpLink_Send(computer_uart);

    /* 链路存活更新:停机决策由仲裁器负责 */
    now_ms = HAL_GetTick();
    if (link_online && ((now_ms - last_rx_ms) > COMPUTER_LINK_TIMEOUT_MS))
    {
        link_online = false;
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

/* ---- 仲裁器查询接口 ---- */

bool ComputerLink_EstopLatched(void)
{
    return estop_latched;
}

bool ComputerLink_LinkOnline(void)
{
    return link_online;
}

bool ComputerLink_ResetRequested(void)
{
    bool r = estop_clear_edge;
    estop_clear_edge = false;
    return r;
}

bool ComputerLink_AutoResumeRequested(void)
{
    bool r = mode_auto_edge;
    mode_auto_edge = false;
    return r;
}

bool ComputerLink_ManualRequested(void)
{
    /* 仅显式模式命令(A5 5E 01)触发自动->手动切换;
     * 普通速度/动作帧只在已处于手动模式时被执行,不触发模式切换 */
    bool r = mode_manual_edge;
    mode_manual_edge = false;
    return r;
}

bool ComputerLink_GetCommand(computer_cmd_t *cmd)
{
    if ((cmd != NULL) && manual_cmd_valid)
    {
        *cmd = manual_cmd;
        manual_cmd_valid = false;
        return true;
    }
    return false;
}

bool ComputerLink_GetAction(uint8_t *action)
{
    if ((action != NULL) && manual_action_valid)
    {
        *action = manual_action;
        manual_action_valid = false;
        return true;
    }
    return false;
}
