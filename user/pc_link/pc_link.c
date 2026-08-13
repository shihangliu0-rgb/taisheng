/**
 ******************************************************************************
 * @file    pc_link.c
 * @brief   小电脑(ROS2 competition_gateway)串口对接 —— 协议实现
 *
 * 实现要点(遵循上位机 docs/串口通信协议.md 中"STM32 实现要求"):
 *   1. 使用环形缓冲区接收字节流,逐字节查找帧头,不假设一次 UART 中断
 *      恰好收到一帧;
 *   2. 先校验帧头、帧类型、帧尾与校验和,全部通过后才使用有效载荷;
 *   3. 无效标志位对应的数据一律清零,不沿用上一次的有效数据;
 *   4. 数据超时只清除本模块的有效位,不干预底盘 / 抬升等核心闭环。
 *
 * 本文件只依赖配置宏与 HAL,不修改任何其他模块。
 ******************************************************************************
 */
#include "pc_link.h"

#if PC_LINK_ENABLE

#include "usart.h"   /* huart7 / huart8 等句柄的外部声明 */

#include <stddef.h>
#include <string.h>

/* 接收状态机 */
typedef enum
{
    PC_RX_WAIT_HEADER_0,   /* 等待 AA */
    PC_RX_WAIT_HEADER_1,   /* 已收 AA,等待 55 */
    PC_RX_WAIT_TYPE,       /* 已收 AA 55,等待帧类型 */
    PC_RX_COLLECT          /* 收集剩余字节直至整帧 */
} pc_rx_state_t;

static UART_HandleTypeDef *pc_uart;

static uint8_t rx_byte;                                     /* 单字节中断接收缓冲 */
static uint8_t rx_ring[PC_LINK_RX_BUFFER_SIZE];             /* 环形缓冲区 */
static volatile uint16_t rx_ring_head;                      /* ISR 写入 */
static uint16_t rx_ring_tail;                               /* 任务读取 */

static volatile uint32_t rx_frame_count;                    /* 通过校验的好帧数 */
static volatile uint32_t rx_position_frame_count;           /* 通过校验的 0x11 位置帧数 */
static volatile uint32_t crc_error_count;                   /* 帧尾/校验和错误帧数 */

static pc_rx_state_t rx_state;
static uint8_t rx_frame[PC_LINK_PERCEPTION_FRAME_SIZE];     /* 最长帧 44B,兼容 24B */
static uint8_t rx_index;
static uint8_t rx_expect;                                   /* 当前帧期望总长 */

static pc_perception_t perception;
static pc_position_t position;

static volatile uint8_t status_state;                       /* 回传状态机的状态 */
static volatile uint8_t status_error;                       /* 回传板端错误码 */
static volatile uint32_t last_perception_ms;
static volatile uint32_t last_position_ms;
static uint32_t last_status_ms;

static volatile bool restart_requested;

/* ----------------------------------------------------------------------------
 * 工具函数
 * --------------------------------------------------------------------------*/

/**
 * @brief 从字节流读取 IEEE754 单精度浮点数(小端,与上位机 memcpy 编码一致)
 */
static float read_le_float(const uint8_t *data)
{
    float value;

    (void)memcpy(&value, data, sizeof(value));
    return value;
}

/**
 * @brief 8 位累加校验和(与上位机 SerialProtocol_Checksum 一致)
 */
static uint8_t checksum8(const uint8_t *data, uint8_t length)
{
    uint8_t checksum = 0U;
    uint8_t i;

    for (i = 0U; i < length; i++)
    {
        checksum = (uint8_t)(checksum + data[i]);
    }

    return checksum;
}

/**
 * @brief 环形缓冲区压入一个字节(ISR 上下文)
 * @note  缓冲区满时丢弃该字节,靠帧校验兜底恢复同步
 */
static void ring_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((rx_ring_head + 1U) &
                               (PC_LINK_RX_BUFFER_SIZE - 1U));

    if (next == rx_ring_tail)
    {
        return;
    }

    rx_ring[rx_ring_head] = byte;
    rx_ring_head = next;
}

static void reset_parser(void)
{
    rx_state = PC_RX_WAIT_HEADER_0;
    rx_index = 0U;
    rx_expect = 0U;
}

/**
 * @brief 校验通过后解帧:帧尾 + 校验和均已确认,这里只管取数据
 */
static void decode_frame(void)
{
    const uint8_t type = rx_frame[2];
    const uint8_t length = rx_expect;
    const uint8_t checksum_index = (uint8_t)(length - 3U);
    const uint8_t flags = rx_frame[4];
    const uint32_t now = HAL_GetTick();

    /* 帧尾校验 */
    if ((rx_frame[length - 2U] != PC_LINK_TAIL_0) ||
        (rx_frame[length - 1U] != PC_LINK_TAIL_1))
    {
        crc_error_count++;
        return;
    }

    /* 校验和:帧类型字节(偏移 2)起,至校验字节前一字节 */
    if (rx_frame[checksum_index] !=
        checksum8(&rx_frame[2], (uint8_t)(checksum_index - 2U)))
    {
        crc_error_count++;
        return;
    }

    rx_frame_count++;
    if (type == PC_LINK_TYPE_POSITION)
    {
        rx_position_frame_count++;
    }

    if (type == PC_LINK_TYPE_PERCEPTION)
    {
        pc_perception_t fresh;

        fresh.red_x_m  = read_le_float(&rx_frame[5]);
        fresh.red_y_m  = read_le_float(&rx_frame[9]);
        fresh.red_z_m  = read_le_float(&rx_frame[13]);
        fresh.blue_x_m = read_le_float(&rx_frame[17]);
        fresh.blue_y_m = read_le_float(&rx_frame[21]);
        fresh.blue_z_m = read_le_float(&rx_frame[25]);
        fresh.ball_x_m = read_le_float(&rx_frame[29]);
        fresh.ball_y_m = read_le_float(&rx_frame[33]);
        fresh.ball_z_m = read_le_float(&rx_frame[37]);

        /* 无效位对应的数据清零,防止下位机沿用旧值 */
        if ((flags & PC_LINK_FLAG_RED_VALID) == 0U)
        {
            fresh.red_x_m = 0.0F;
            fresh.red_y_m = 0.0F;
            fresh.red_z_m = 0.0F;
        }
        if ((flags & PC_LINK_FLAG_BLUE_VALID) == 0U)
        {
            fresh.blue_x_m = 0.0F;
            fresh.blue_y_m = 0.0F;
            fresh.blue_z_m = 0.0F;
        }
        if ((flags & PC_LINK_FLAG_BALL_VALID) == 0U)
        {
            fresh.ball_x_m = 0.0F;
            fresh.ball_y_m = 0.0F;
            fresh.ball_z_m = 0.0F;
        }

        fresh.flags = flags;
        perception = fresh;
        last_perception_ms = now;
    }
    else if (type == PC_LINK_TYPE_POSITION)
    {
        pc_position_t fresh;

        fresh.field_x_m = read_le_float(&rx_frame[5]);
        fresh.field_y_m = read_le_float(&rx_frame[9]);
        fresh.field_z_m = read_le_float(&rx_frame[13]);
        fresh.field_w   = read_le_float(&rx_frame[17]);

        if ((flags & PC_LINK_FLAG_FIELD_VALID) == 0U)
        {
            fresh.field_x_m = 0.0F;
            fresh.field_y_m = 0.0F;
            fresh.field_z_m = 0.0F;
            fresh.field_w   = 0.0F;
        }

        fresh.flags = flags;
        position = fresh;
        last_position_ms = now;
    }
    else
    {
        /* 未知帧类型,丢弃 */
    }
}

/**
 * @brief 逐字节解析(任务上下文)
 */
static void parse_byte(uint8_t data)
{
    switch (rx_state)
    {
    case PC_RX_WAIT_HEADER_0:
        if (data == PC_LINK_HEADER_0)
        {
            rx_state = PC_RX_WAIT_HEADER_1;
        }
        break;

    case PC_RX_WAIT_HEADER_1:
        if (data == PC_LINK_HEADER_1)
        {
            rx_state = PC_RX_WAIT_TYPE;
        }
        else if (data != PC_LINK_HEADER_0)
        {
            rx_state = PC_RX_WAIT_HEADER_0;
        }
        /* data == 0xAA:可能是一个新帧头的开始,保持本状态继续等 0x55 */
        break;

    case PC_RX_WAIT_TYPE:
        if (data == PC_LINK_TYPE_PERCEPTION)
        {
            rx_expect = PC_LINK_PERCEPTION_FRAME_SIZE;
            rx_frame[0] = PC_LINK_HEADER_0;
            rx_frame[1] = PC_LINK_HEADER_1;
            rx_frame[2] = data;
            rx_index = 3U;
            rx_state = PC_RX_COLLECT;
        }
        else if (data == PC_LINK_TYPE_POSITION)
        {
            rx_expect = PC_LINK_POSITION_FRAME_SIZE;
            rx_frame[0] = PC_LINK_HEADER_0;
            rx_frame[1] = PC_LINK_HEADER_1;
            rx_frame[2] = data;
            rx_index = 3U;
            rx_state = PC_RX_COLLECT;
        }
        else if (data == PC_LINK_HEADER_0)
        {
            /* 可能是新帧头 AA,回退到等 0x55 */
            rx_state = PC_RX_WAIT_HEADER_1;
        }
        else
        {
            rx_state = PC_RX_WAIT_HEADER_0;
        }
        break;

    case PC_RX_COLLECT:
        rx_frame[rx_index] = data;
        rx_index++;
        if (rx_index >= rx_expect)
        {
            decode_frame();
            reset_parser();
        }
        break;

    default:
        reset_parser();
        break;
    }
}

/**
 * @brief 发送 8 字节状态帧:55 AA | 20 | state | error | checksum | 0D 0A
 */
static void send_status_frame(void)
{
    uint8_t frame[PC_LINK_STATUS_FRAME_SIZE];

    frame[0] = PC_LINK_HEADER_1;   /* 状态帧头为 55 AA(与下发帧相反) */
    frame[1] = PC_LINK_HEADER_0;
    frame[2] = PC_LINK_TYPE_STATUS;
    frame[3] = status_state;
    frame[4] = status_error;
    frame[5] = checksum8(&frame[2], 3U);
    frame[6] = PC_LINK_TAIL_0;
    frame[7] = PC_LINK_TAIL_1;

    /* 8 字节 @115200 约 0.7ms,在通信任务中阻塞发送足够安全 */
    (void)HAL_UART_Transmit(pc_uart, frame, sizeof(frame), 2U);
}

/* ----------------------------------------------------------------------------
 * 公共接口
 * --------------------------------------------------------------------------*/

HAL_StatusTypeDef PcLink_Init(void)
{
    HAL_StatusTypeDef status;

    pc_uart = &PC_LINK_UART_HANDLE;

    /* 按配置宏重配波特率,确保与上位机 yaml 一致。
     * DeInit 后 State 复位,HAL_UART_Init 会重新执行 MspInit,
     * 引脚 / NVIC / DMA 配置不受影响。 */
    (void)HAL_UART_DeInit(pc_uart);
    pc_uart->Init.BaudRate = PC_LINK_BAUD_RATE;
    status = HAL_UART_Init(pc_uart);
    if (status != HAL_OK)
    {
        return status;
    }

    rx_ring_head = 0U;
    rx_ring_tail = 0U;
    rx_frame_count = 0U;
    rx_position_frame_count = 0U;
    crc_error_count = 0U;
    (void)memset(rx_frame, 0, sizeof(rx_frame));
    (void)memset(&perception, 0, sizeof(perception));
    (void)memset(&position, 0, sizeof(position));
    status_state = 0U;
    status_error = 0U;
    last_perception_ms = 0U;
    last_position_ms = 0U;
    last_status_ms = HAL_GetTick();
    restart_requested = false;
    reset_parser();

    status = HAL_UART_Receive_IT(pc_uart, &rx_byte, 1U);
    if (status != HAL_OK)
    {
        restart_requested = true;
    }

    return status;
}

void PcLink_Run(void)
{
    uint8_t byte;
    uint32_t now;

    if (pc_uart == NULL)
    {
        return;
    }

    /* 接收中断异常后的恢复 */
    if (restart_requested)
    {
        restart_requested = false;
        (void)HAL_UART_AbortReceive(pc_uart);
        reset_parser();
        rx_ring_head = 0U;
        rx_ring_tail = 0U;
        if (HAL_UART_Receive_IT(pc_uart, &rx_byte, 1U) != HAL_OK)
        {
            restart_requested = true;
        }
    }

    /* 解析环形缓冲区中积压的字节 */
    while (rx_ring_tail != rx_ring_head)
    {
        byte = rx_ring[rx_ring_tail];
        rx_ring_tail = (uint16_t)((rx_ring_tail + 1U) &
                                  (PC_LINK_RX_BUFFER_SIZE - 1U));
        parse_byte(byte);
    }

    now = HAL_GetTick();

    /* 数据超时 -> 清除有效位,不沿用旧数据 */
    if ((uint32_t)(now - last_perception_ms) > PC_LINK_DATA_TIMEOUT_MS)
    {
        perception.flags = 0U;
    }
    if ((uint32_t)(now - last_position_ms) > PC_LINK_DATA_TIMEOUT_MS)
    {
        position.flags = 0U;
    }

    /* 状态帧周期回传(>=10Hz) */
    if ((uint32_t)(now - last_status_ms) >= PC_LINK_STATUS_PERIOD_MS)
    {
        last_status_ms = now;
        send_status_frame();
    }
}

void PcLink_RxCplt(UART_HandleTypeDef *uart)
{
    if ((pc_uart == NULL) || (uart != pc_uart))
    {
        return;
    }

    ring_push(rx_byte);
    if (HAL_UART_Receive_IT(pc_uart, &rx_byte, 1U) != HAL_OK)
    {
        restart_requested = true;
    }
}

void PcLink_Error(UART_HandleTypeDef *uart)
{
    if ((pc_uart == NULL) || (uart != pc_uart))
    {
        return;
    }

    restart_requested = true;
}

bool PcLink_GetPerception(pc_perception_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out = perception;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return (out->flags & (PC_LINK_FLAG_RED_VALID |
                          PC_LINK_FLAG_BLUE_VALID |
                          PC_LINK_FLAG_BALL_VALID)) != 0U;
}

bool PcLink_GetPosition(pc_position_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out = position;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return (out->flags & PC_LINK_FLAG_FIELD_VALID) != 0U;
}

void PcLink_SetStatus(uint8_t state, uint8_t error)
{
    /* 单字节写对 Cortex-M7 原子,可直接赋值 */
    status_state = state;
    status_error = error;
}

bool PcLink_IsOnline(void)
{
    uint32_t now = HAL_GetTick();

    return (((uint32_t)(now - last_perception_ms) <= PC_LINK_DATA_TIMEOUT_MS) ||
            ((uint32_t)(now - last_position_ms) <= PC_LINK_DATA_TIMEOUT_MS));
}

void PcLink_GetStats(uint32_t *frames, uint32_t *crc_errors)
{
    if (frames != NULL)
    {
        *frames = rx_frame_count;
    }
    if (crc_errors != NULL)
    {
        *crc_errors = crc_error_count;
    }
}

uint32_t PcLink_GetPositionSeq(void)
{
    return rx_position_frame_count;
}

#endif /* PC_LINK_ENABLE */
