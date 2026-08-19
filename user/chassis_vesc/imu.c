#include "imu.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* DM-IMU 接收缓冲区和数据帧字段。 */
#define IMU_DMA_BUFFER_SIZE       256U
#define IMU_RX_FIFO_SIZE          512U
#define IMU_FRAME_BUFFER_SIZE     24U
#define IMU_FRAME_HEADER_1        0x55U
#define IMU_FRAME_HEADER_2        0xAAU
#define IMU_FRAME_END             0x0AU
#define IMU_FRAME_TYPE_GYRO       0x02U
#define IMU_FRAME_TYPE_EULER      0x03U
#define IMU_FRAME_LENGTH_STANDARD 19U
#define IMU_FRAME_LENGTH_QUAT     23U
#define IMU_VALUE_OFFSET          12U

typedef struct
{
    UART_HandleTypeDef *uart;
    uint8_t dma_buffer[IMU_DMA_BUFFER_SIZE];
    uint8_t rx_fifo[IMU_RX_FIFO_SIZE];           //软件接收队列
    uint8_t frame_buffer[IMU_FRAME_BUFFER_SIZE];
    volatile uint16_t fifo_write;                //fifo写指针
    volatile uint16_t fifo_read;                 //fifo读指针
    uint16_t dma_read_pos;
    uint8_t frame_index;
    uint8_t expected_length;
    imu_raw_data_t raw_data;
    imu_stats_t stats;
    volatile bool restart_requested;
    bool initialized;                            //是否已初始化
} imu_driver_t;

static imu_driver_t imu_driver;

static HAL_StatusTypeDef start_receive(void)
{
    HAL_StatusTypeDef status;

    if ((imu_driver.uart == NULL) || (imu_driver.uart->hdmarx == NULL))
    {
        return HAL_ERROR;
    }

    imu_driver.dma_read_pos = 0U;
    /* 清除启动延时期间累计的错误和残留数据，避免 DMA 启动后立即中止。 */
    __HAL_UART_CLEAR_FLAG(imu_driver.uart,
                          UART_CLEAR_OREF | UART_CLEAR_NEF |
                          UART_CLEAR_PEF | UART_CLEAR_FEF);
    __HAL_UART_SEND_REQ(imu_driver.uart, UART_RXDATA_FLUSH_REQUEST);
    status = HAL_UARTEx_ReceiveToIdle_DMA(imu_driver.uart,
                                         imu_driver.dma_buffer,
                                         IMU_DMA_BUFFER_SIZE);
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(imu_driver.uart->hdmarx, DMA_IT_HT);
    }

    return status;
}

static void fifo_push(uint8_t byte)  //环形缓存区入队
{
    uint16_t next_write = (uint16_t)(imu_driver.fifo_write + 1U);

    if (next_write >= IMU_RX_FIFO_SIZE)
    {
        next_write = 0U;
    }
    if (next_write == imu_driver.fifo_read)  //防止写的数据追上读的造成覆盖
    {
        imu_driver.stats.rx_overflow_count++;
        return;
    }

    imu_driver.rx_fifo[imu_driver.fifo_write] = byte;
    imu_driver.fifo_write = next_write;
}

static bool fifo_pop(uint8_t *byte) //环形缓存区出队
{
    uint16_t next_read;

    if ((byte == NULL) || (imu_driver.fifo_read == imu_driver.fifo_write))
    {
        return false;
    }

    *byte = imu_driver.rx_fifo[imu_driver.fifo_read];
    next_read = (uint16_t)(imu_driver.fifo_read + 1U);
    if (next_read >= IMU_RX_FIFO_SIZE)
    {
        next_read = 0U;
    }
    imu_driver.fifo_read = next_read;
    return true;
}

static void copy_dma_range(uint16_t begin, uint16_t end)
{
    uint16_t index;

    for (index = begin; index < end; index++)
    {
        fifo_push(imu_driver.dma_buffer[index]);
    }
}

static void copy_dma_data(uint16_t size)
{
    // 循环 DMA 回调中的 size 是写入位置，不是本次收到的字节数
    if (size > IMU_DMA_BUFFER_SIZE)
    {
        imu_driver.stats.invalid_frame_count++;
        return;
    }

    if (size == IMU_DMA_BUFFER_SIZE)
    {
        copy_dma_range(imu_driver.dma_read_pos, IMU_DMA_BUFFER_SIZE);
        imu_driver.dma_read_pos = 0U;
    }
    else if (size > imu_driver.dma_read_pos)
    {
        copy_dma_range(imu_driver.dma_read_pos, size);
        imu_driver.dma_read_pos = size;
    }
    else if (size < imu_driver.dma_read_pos)
    {
        copy_dma_range(imu_driver.dma_read_pos, IMU_DMA_BUFFER_SIZE);
        copy_dma_range(0U, size);
        imu_driver.dma_read_pos = size;
    }
    else
    {
        // DMA 写入位置没有变化，无新数据需要处理
    }
}

static bool frame_type_valid(uint8_t type, uint8_t *length)
{
    if (length == NULL)
    {
        return false;
    }

    if ((type >= 0x01U) && (type <= IMU_FRAME_TYPE_EULER))
    {
        *length = IMU_FRAME_LENGTH_STANDARD;
        return true;
    }
    if (type == 0x04U)
    {
        *length = IMU_FRAME_LENGTH_QUAT;
        return true;
    }

    return false;
}

static bool parse_frame(void)
{
    float value;
    uint32_t now_ms;
    uint8_t frame_type = imu_driver.frame_buffer[3];

    if (imu_driver.frame_buffer[imu_driver.expected_length - 1U] !=
        IMU_FRAME_END)
    {
        return false;
    }

    if ((frame_type != IMU_FRAME_TYPE_GYRO) &&
        (frame_type != IMU_FRAME_TYPE_EULER))
    {
        return true;
    }

    // 协议中 X、Y、Z 为连续小端 float，偏移 12 对应 Z 轴数据
    memcpy(&value, &imu_driver.frame_buffer[IMU_VALUE_OFFSET],
           sizeof(value));
    if (isnan(value) || isinf(value))
    {
        return false;
    }
    now_ms = HAL_GetTick();

    if (frame_type == IMU_FRAME_TYPE_GYRO)
    {
        if (fabsf(value) > 2000.0f)
        {
            return false;
        }
        imu_driver.raw_data.gyro_z_deg_s = value;
        imu_driver.raw_data.gyro_sequence++;
        imu_driver.raw_data.gyro_valid = true;
        imu_driver.stats.last_gyro_ms = now_ms;
    }
    else
    {
        if (fabsf(value) > 360.0f)
        {
            return false;
        }
        imu_driver.raw_data.yaw_deg = value;
        imu_driver.raw_data.yaw_sequence++;
        imu_driver.raw_data.yaw_valid = true;
        imu_driver.stats.last_yaw_ms = now_ms;
    }

    imu_driver.stats.last_valid_ms = now_ms;
    return true;
}

static void reset_parser(uint8_t last_byte)
{
    imu_driver.expected_length = 0U;
    if (last_byte == IMU_FRAME_HEADER_1)
    {
        imu_driver.frame_buffer[0] = last_byte;
        imu_driver.frame_index = 1U;
    }
    else
    {
        imu_driver.frame_index = 0U;
    }
}

static void discard_receive_pipeline(void)
{
    imu_driver.fifo_read = imu_driver.fifo_write;
    imu_driver.frame_index = 0U;
    imu_driver.expected_length = 0U;
}

static void parse_byte(uint8_t byte)
{
    if (imu_driver.frame_index == 0U)
    {
        if (byte == IMU_FRAME_HEADER_1)
        {
            imu_driver.frame_buffer[0] = byte;
            imu_driver.frame_index = 1U;
        }
        return;
    }

    if (imu_driver.frame_index == 1U)
    {
        if (byte == IMU_FRAME_HEADER_2)
        {
            imu_driver.frame_buffer[1] = byte;
            imu_driver.frame_index = 2U;
        }
        else
        {
            reset_parser(byte);
        }
        return;
    }

    if (imu_driver.frame_index >= IMU_FRAME_BUFFER_SIZE)
    {
        imu_driver.stats.invalid_frame_count++;
        reset_parser(byte);
        return;
    }

    imu_driver.frame_buffer[imu_driver.frame_index] = byte;
    if (imu_driver.frame_index == 2U)
    {
        imu_driver.frame_index = 3U;
        return;
    }

    if (imu_driver.frame_index == 3U)
    {
        if (!frame_type_valid(byte, &imu_driver.expected_length))
        {
            imu_driver.stats.invalid_frame_count++;
            reset_parser(byte);
            return;
        }
        imu_driver.frame_index = 4U;
        return;
    }

    imu_driver.frame_index++;
    if ((imu_driver.expected_length > 0U) &&
        (imu_driver.frame_index == imu_driver.expected_length))
    {
        if (parse_frame())
        {
            imu_driver.stats.valid_frame_count++;
        }
        else
        {
            imu_driver.stats.invalid_frame_count++;
        }
        reset_parser(byte);
    }
}

/**
 * @brief 初始化 DM-IMU L1 串口驱动
 * @param uart 已配置为 921600 波特率和循环接收 DMA 的串口
 * @retval HAL 状态
 */
HAL_StatusTypeDef Imu_Init(UART_HandleTypeDef *uart)
{
    if (uart == NULL)
    {
        return HAL_ERROR;
    }
    if (imu_driver.initialized)
    {
        return (imu_driver.uart == uart) ? HAL_OK : HAL_ERROR;
    }

    memset(&imu_driver, 0, sizeof(imu_driver));
    imu_driver.uart = uart;
    if (start_receive() != HAL_OK)
    {
        imu_driver.uart = NULL;
        return HAL_ERROR;
    }

    imu_driver.initialized = true;
    return HAL_OK;
}

/**
 * @brief 处理接收队列中的数据，并在串口错误后恢复 DMA
 * @retval None
 */
void Imu_Process(void)
{
    uint8_t byte;

    if (!imu_driver.initialized)
    {
        return;
    }

    if (imu_driver.restart_requested)
    {
        imu_driver.restart_requested = false;
        (void)HAL_UART_AbortReceive(imu_driver.uart);
        discard_receive_pipeline();
        if (start_receive() != HAL_OK)
        {
            imu_driver.restart_requested = true;
        }
    }

    while (fifo_pop(&byte))
    {
        parse_byte(byte);
    }
}

void Imu_RequestRestart(void)
{
    if (imu_driver.initialized)
    {
        imu_driver.restart_requested = true;
    }
}

/**
 * @brief 向 DM-IMU L1 发送一条配置命令
 * @param data 命令数据
 * @param length 命令长度
 * @retval HAL 状态
 */
HAL_StatusTypeDef Imu_Send(const uint8_t *data, uint16_t length)
{
    if (!imu_driver.initialized || (data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(imu_driver.uart, (uint8_t *)data, length,
                             100U);
}

bool Imu_GetRawData(imu_raw_data_t *data)
{
    if (!imu_driver.initialized || (data == NULL))
    {
        return false;
    }

    *data = imu_driver.raw_data;
    return true;
}

bool Imu_GetStats(imu_stats_t *stats)
{
    if (!imu_driver.initialized || (stats == NULL))
    {
        return false;
    }

    *stats = imu_driver.stats;
    return true;
}

/**
 * @brief 将循环 DMA 中的新数据搬入软件接收队列
 * @param uart 触发回调的串口
 * @param size HAL 返回的当前 DMA 写入位置
 * @retval None
 */
void Imu_HandleRxEvent(UART_HandleTypeDef *uart, uint16_t size)
{
    if (!imu_driver.initialized || (uart != imu_driver.uart))
    {
        return;
    }

    copy_dma_data(size);
}

/**
 * @brief 记录串口错误，并请求任务上下文恢复 DMA
 * @param uart 触发错误的串口
 * @retval None
 */
void Imu_HandleUartError(UART_HandleTypeDef *uart)
{
    if (!imu_driver.initialized || (uart != imu_driver.uart))
    {
        return;
    }

    imu_driver.stats.uart_error_count++;
    imu_driver.restart_requested = true;
}
