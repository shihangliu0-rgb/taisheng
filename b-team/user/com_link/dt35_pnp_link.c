#include "dt35_pnp_link.h"

#define DT35_FRAME_HEADER       0xAAU  /* DT35 数据帧头 */
#define PNP_FRAME_HEADER        0xABU  /* PNP 数据帧头 */
#define SENSOR_FRAME_LENGTH     5U     /* 串口帧长度 */
#define SENSOR_LINK_TIMEOUT_MS  500U   /* 传感器离线时间(ms) */
#define SENSOR_TX_TIMEOUT_MS    20U    /* 串口发送超时(ms) */

typedef struct
{
    UART_HandleTypeDef *uart;
    uint8_t rx_byte;
    uint8_t frame[SENSOR_FRAME_LENGTH];
    uint8_t index;
    volatile uint8_t restart_requested;
} sensor_parser_t;

static sensor_parser_t sensor_parser;
static uint8_t sensor_link_initialized;

volatile dt35_link_t dt35_link[SENSOR_LINK_COUNT];
volatile pnp_link_t pnp_link[SENSOR_LINK_COUNT];

/**
 * @brief 计算传感器帧异或校验值
 * @param data 待校验数据
 * @param length 数据长度
 * @retval 异或校验值
 */
static uint8_t Link_Checksum(const uint8_t *data, uint8_t length)
{
    uint8_t checksum = 0U;
    uint8_t i;

    for (i = 0U; i < length; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * @brief 复位串口帧解析位置
 * @param None
 * @retval None
 */
static void Link_ResetParser(void)
{
    sensor_parser.index = 0U;
}

/**
 * @brief 将传感器地址转换为数组下标
 * @param address 传感器地址
 * @retval 传感器数组下标
 */
static uint8_t Link_GetIndex(uint8_t address)
{
    return (address == SENSOR_LINK_ADDR_F) ?
           SENSOR_LINK_F_INDEX : SENSOR_LINK_L_B_INDEX;
}

/**
 * @brief 检查当前接收帧是否合法
 * @param None
 * @retval 1 表示合法，0 表示无效
 */
static uint8_t Link_FrameIsValid(void)
{
    uint16_t value;

    if ((sensor_parser.frame[1] != SENSOR_LINK_ADDR_F) &&
        (sensor_parser.frame[1] != SENSOR_LINK_ADDR_L))
    {
        return 0U;
    }
    if (sensor_parser.frame[4] !=
        Link_Checksum(sensor_parser.frame, SENSOR_FRAME_LENGTH - 1U))
    {
        return 0U;
    }
    if (sensor_parser.frame[0] == DT35_FRAME_HEADER)
    {
        return 1U;
    }
    if (sensor_parser.frame[0] != PNP_FRAME_HEADER)
    {
        return 0U;
    }

    value = (uint16_t)sensor_parser.frame[2] |
            ((uint16_t)sensor_parser.frame[3] << 8U);
    return (value <= 1U) ? 1U : 0U;
}

/**
 * @brief 保存当前接收帧
 * @param None
 * @retval None
 */
static void Link_StoreFrame(void)
{
    uint8_t index;
    uint16_t value;
    uint32_t now_ms;

    index = Link_GetIndex(sensor_parser.frame[1]);
    value = (uint16_t)sensor_parser.frame[2] |
            ((uint16_t)sensor_parser.frame[3] << 8U);
    now_ms = HAL_GetTick();

    if (sensor_parser.frame[0] == DT35_FRAME_HEADER)
    {
        /*
         * 主控串口：0x40/0x41 与车上前/左对调后再入库。
         * 不改传感器子板。PNP 地址不动。
         */
        index = (index == SENSOR_LINK_F_INDEX) ?
                SENSOR_LINK_L_B_INDEX : SENSOR_LINK_F_INDEX;
        dt35_link[index].distance_cm = value;
        dt35_link[index].last_rx_ms = now_ms;
        dt35_link[index].online = 1U;
        dt35_link[index].frame_pending = 1U;
    }
    else
    {
        pnp_link[index].trigger = (uint8_t)value;
        pnp_link[index].last_rx_ms = now_ms;
        pnp_link[index].online = 1U;
        pnp_link[index].frame_pending = 1U;
    }
}

/**
 * @brief 将一个串口字节送入帧解析器
 * @param data 接收字节
 * @retval None
 */
static void Link_ParseByte(uint8_t data)
{
    if (sensor_parser.index == 0U)
    {
        if ((data == DT35_FRAME_HEADER) || (data == PNP_FRAME_HEADER))
        {
            sensor_parser.frame[0] = data;
            sensor_parser.index = 1U;
        }
        return;
    }

    sensor_parser.frame[sensor_parser.index] = data;
    sensor_parser.index++;
    if (sensor_parser.index < SENSOR_FRAME_LENGTH)
    {
        return;
    }

    if (Link_FrameIsValid() != 0U)
    {
        Link_StoreFrame();
    }
    Link_ResetParser();
}

/**
 * @brief 启动单字节串口中断接收
 * @param None
 * @retval HAL 状态
 */
static HAL_StatusTypeDef Link_StartReceive(void)
{
    if (sensor_parser.uart == NULL)
    {
        return HAL_ERROR;
    }
    return HAL_UART_Receive_IT(sensor_parser.uart,
                               &sensor_parser.rx_byte, 1U);
}

/**
 * @brief 发送一帧传感器数据
 * @param uart 目标串口
 * @param header 帧头
 * @param address 传感器地址
 * @param value 传感器数值
 * @retval None
 */
static void Link_SendFrame(UART_HandleTypeDef *uart, uint8_t header,
                           uint8_t address, uint16_t value)
{
    uint8_t frame[SENSOR_FRAME_LENGTH];

    frame[0] = header;
    frame[1] = address;
    frame[2] = (uint8_t)value;
    frame[3] = (uint8_t)(value >> 8U);
    frame[4] = Link_Checksum(frame, SENSOR_FRAME_LENGTH - 1U);
    (void)HAL_UART_Transmit(uart, frame, sizeof(frame),
                            SENSOR_TX_TIMEOUT_MS);
}

/**
 * @brief 初始化 DT35 和 PNP 串口链路
 * @param uart 传感器串口
 * @retval HAL 状态
 */
HAL_StatusTypeDef DT35PnpLink_Init(UART_HandleTypeDef *uart)
{
    uint8_t i;

    if (uart == NULL)
    {
        return HAL_ERROR;
    }
    if (sensor_link_initialized != 0U)
    {
        return (sensor_parser.uart == uart) ? HAL_OK : HAL_ERROR;
    }

    sensor_parser.uart = uart;
    sensor_parser.restart_requested = 0U;
    Link_ResetParser();

    for (i = 0U; i < SENSOR_LINK_COUNT; i++)
    {
        dt35_link[i].distance_cm = 0U;
        dt35_link[i].last_rx_ms = 0U;
        dt35_link[i].online = 0U;
        dt35_link[i].frame_pending = 0U;

        pnp_link[i].trigger = 0U;
        pnp_link[i].last_rx_ms = 0U;
        pnp_link[i].online = 0U;
        pnp_link[i].frame_pending = 0U;
    }
    sensor_link_initialized = 1U;

    if (Link_StartReceive() != HAL_OK)
    {
        sensor_parser.restart_requested = 1U;
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
 * @brief 处理串口重启和传感器离线状态
 * @param None
 * @retval None
 */
void DT35PnpLink_Run(void)
{
    uint8_t i;
    uint32_t now_ms;

    if (sensor_parser.uart == NULL)
    {
        return;
    }

    if (sensor_parser.restart_requested != 0U)
    {
        sensor_parser.restart_requested = 0U;
        Link_ResetParser();
        if (Link_StartReceive() != HAL_OK)
        {
            sensor_parser.restart_requested = 1U;
        }
    }

    now_ms = HAL_GetTick();
    for (i = 0U; i < SENSOR_LINK_COUNT; i++)
    {
        if ((dt35_link[i].online != 0U) &&
            ((uint32_t)(now_ms - dt35_link[i].last_rx_ms) >
             SENSOR_LINK_TIMEOUT_MS))
        {
            dt35_link[i].online = 0U;
        }

        if ((pnp_link[i].online != 0U) &&
            ((uint32_t)(now_ms - pnp_link[i].last_rx_ms) >
             SENSOR_LINK_TIMEOUT_MS))
        {
            pnp_link[i].online = 0U;
            pnp_link[i].trigger = 0U;
            pnp_link[i].frame_pending = 1U;
        }
    }
}

/**
 * @brief 将有变化的 DT35 和 PNP 数据发送到上位机
 * @param uart 上位机串口
 * @retval None
 */
void DT35PnpLink_Send(UART_HandleTypeDef *uart)
{
    dt35_link_t dt35_tx[SENSOR_LINK_COUNT];
    pnp_link_t pnp_tx[SENSOR_LINK_COUNT];
    uint8_t i;
    uint32_t primask;

    if (uart == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0U; i < SENSOR_LINK_COUNT; i++)
    {
        dt35_tx[i] = dt35_link[i];
        pnp_tx[i] = pnp_link[i];
        dt35_link[i].frame_pending = 0U;
        pnp_link[i].frame_pending = 0U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    if (dt35_tx[SENSOR_LINK_L_B_INDEX].frame_pending != 0U)
    {
        Link_SendFrame(uart, DT35_FRAME_HEADER, SENSOR_LINK_ADDR_L,
                       dt35_tx[SENSOR_LINK_L_B_INDEX].distance_cm);
    }
    if (dt35_tx[SENSOR_LINK_F_INDEX].frame_pending != 0U)
    {
        Link_SendFrame(uart, DT35_FRAME_HEADER, SENSOR_LINK_ADDR_F,
                       dt35_tx[SENSOR_LINK_F_INDEX].distance_cm);
    }
    if (pnp_tx[SENSOR_LINK_L_B_INDEX].frame_pending != 0U)
    {
        Link_SendFrame(uart, PNP_FRAME_HEADER, PNP_LINK_ADDR_B,
                       pnp_tx[SENSOR_LINK_L_B_INDEX].trigger);
    }
    if (pnp_tx[SENSOR_LINK_F_INDEX].frame_pending != 0U)
    {
        Link_SendFrame(uart, PNP_FRAME_HEADER, PNP_LINK_ADDR_F,
                       pnp_tx[SENSOR_LINK_F_INDEX].trigger);
    }
}

/**
 * @brief 处理传感器串口单字节接收完成
 * @param uart 触发回调的串口
 * @retval None
 */
void DT35PnpLink_RxCplt(UART_HandleTypeDef *uart)
{
    if ((sensor_parser.uart == NULL) || (uart != sensor_parser.uart))
    {
        return;
    }

    Link_ParseByte(sensor_parser.rx_byte);
    if (Link_StartReceive() != HAL_OK)
    {
        sensor_parser.restart_requested = 1U;
    }
}

/**
 * @brief 记录传感器串口错误并请求重启接收
 * @param uart 触发回调的串口
 * @retval None
 */
void DT35PnpLink_Error(UART_HandleTypeDef *uart)
{
    if ((sensor_parser.uart != NULL) &&
        (uart == sensor_parser.uart))
    {
        sensor_parser.restart_requested = 1U;
    }
}
