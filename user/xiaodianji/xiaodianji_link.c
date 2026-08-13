/**
 * @file xiaodianji_link.c
 * @brief 小电脑串口通信实现: 逐字节 IT 接收 + 状态机解析 + 状态帧发送
 */

#include "xiaodianji_link.h"
#include "usart.h"          /* huart2 等 UART 句柄声明，仅引用不修改 usart.c */
#include <string.h>

/* ============================ 协议常量 ============================ */
#define XDJ_HEAD0          0xAAU
#define XDJ_HEAD1          0x55U
#define XDJ_TYPE_PERCEPTION 0x10U   /* 感知帧 44B */
#define XDJ_TYPE_POSITION   0x11U   /* 位置帧 24B */
#define XDJ_TYPE_STATUS_TX  0x20U   /* 状态帧(STM32→上位机) 8B */
#define XDJ_TAIL0           0x0DU
#define XDJ_TAIL1           0x0AU

#define XDJ_PERCEPTION_LEN  44U
#define XDJ_POSITION_LEN    24U
#define XDJ_RX_BUF_SIZE     48U   /* 取两者最大 + 余量 */

/* ============================ 全局数据 ============================ */
volatile Xiaodianji_Data_t xiaodianji_data;

/* ============================ 内部状态 ============================ */
static UART_HandleTypeDef *s_uart;
static uint8_t s_rx_byte;
static uint8_t s_rx_buf[XDJ_RX_BUF_SIZE];
static uint16_t s_rx_idx;
static uint8_t s_rx_state;     /* 0=等HEAD0, 1=等HEAD1, 2=收body */
static uint8_t s_frame_type;
static uint8_t s_frame_len;
static volatile uint32_t s_last_rx_ms;
static volatile uint8_t s_restart_requested;

/* ============================ 工具 ============================ */
static uint8_t checksum_calc(const uint8_t *data, uint16_t from, uint16_t to)
{
    uint16_t i;
    uint8_t sum = 0U;
    for (i = from; i <= to; i++) { sum += data[i]; }
    return sum;
}

static float read_f32(const uint8_t *p)
{
    float v;
    memcpy(&v, p, 4);
    return v;   /* Cortex-M 小端 */
}

/* ============================ 帧处理 ============================ */
static void handle_frame(void)
{
    uint16_t checksum_end;

    /* 校验 checksum(偏移 2 到 倒数第 3 字节) */
    checksum_end = s_frame_len - 3U;   /* 去掉 checksum + 0D + 0A */
    if (s_rx_buf[s_frame_len - 3U] != checksum_calc(s_rx_buf, 2U, checksum_end))
    {
        return;   /* 校验失败 */
    }
    if (s_rx_buf[s_frame_len - 2U] != XDJ_TAIL0 || s_rx_buf[s_frame_len - 1U] != XDJ_TAIL1)
    {
        return;   /* 帧尾错误 */
    }

    s_last_rx_ms = HAL_GetTick();
    xiaodianji_data.online = 1U;

    if (s_frame_type == XDJ_TYPE_POSITION && s_frame_len == XDJ_POSITION_LEN)
    {
        /* 位置帧 */
        uint8_t flags = s_rx_buf[4];
        xiaodianji_data.pose_valid = (flags & 0x01U) ? 1U : 0U;
        xiaodianji_data.field_x_m = read_f32(&s_rx_buf[5]);
        xiaodianji_data.field_y_m = read_f32(&s_rx_buf[9]);
        xiaodianji_data.field_z_m = read_f32(&s_rx_buf[13]);
        xiaodianji_data.field_w   = read_f32(&s_rx_buf[17]);
    }
    else if (s_frame_type == XDJ_TYPE_PERCEPTION && s_frame_len == XDJ_PERCEPTION_LEN)
    {
        /* 感知帧 */
        uint8_t flags = s_rx_buf[4];
        xiaodianji_data.red_valid  = (flags & 0x01U) ? 1U : 0U;
        xiaodianji_data.blue_valid = (flags & 0x02U) ? 1U : 0U;
        xiaodianji_data.ball_valid = (flags & 0x04U) ? 1U : 0U;
        xiaodianji_data.red_x_m  = read_f32(&s_rx_buf[5]);   /* 偏移 5 */
        xiaodianji_data.red_y_m  = read_f32(&s_rx_buf[9]);
        xiaodianji_data.red_z_m  = read_f32(&s_rx_buf[13]);
        xiaodianji_data.blue_x_m = read_f32(&s_rx_buf[17]);
        xiaodianji_data.blue_y_m = read_f32(&s_rx_buf[21]);
        xiaodianji_data.blue_z_m = read_f32(&s_rx_buf[25]);
        xiaodianji_data.ball_x_m = read_f32(&s_rx_buf[29]);
        xiaodianji_data.ball_y_m = read_f32(&s_rx_buf[33]);
        xiaodianji_data.ball_z_m = read_f32(&s_rx_buf[37]);
    }
}

/* ============================ 逐字节状态机 ============================ */
static void feed_byte(uint8_t b)
{
    switch (s_rx_state)
    {
    case 0:   /* 等帧头 0xAA */
        if (b == XDJ_HEAD0) { s_rx_buf[0] = b; s_rx_idx = 1; s_rx_state = 1; }
        break;

    case 1:   /* 等帧头 0x55 */
        if (b == XDJ_HEAD1)
        {
            s_rx_buf[1] = b; s_rx_idx = 2; s_rx_state = 2;
        }
        else if (b == XDJ_HEAD0) { s_rx_buf[0] = b; s_rx_idx = 1; }
        else { s_rx_state = 0; }
        break;

    case 2:   /* 收 type */
        s_rx_buf[2] = b;
        s_rx_idx = 3;
        if (b == XDJ_TYPE_PERCEPTION)      { s_frame_type = b; s_frame_len = XDJ_PERCEPTION_LEN; }
        else if (b == XDJ_TYPE_POSITION)   { s_frame_type = b; s_frame_len = XDJ_POSITION_LEN; }
        else                               { s_rx_state = 0; break; }
        s_rx_state = 3;
        break;

    case 3:   /* 收剩余 body */
        s_rx_buf[s_rx_idx++] = b;
        if (s_rx_idx >= s_frame_len)
        {
            handle_frame();
            s_rx_state = 0;
        }
        if (s_rx_idx >= XDJ_RX_BUF_SIZE) { s_rx_state = 0; }   /* 溢出保护 */
        break;

    default:
        s_rx_state = 0;
        break;
    }
}

/* ============================ UART 收发 ============================ */
static HAL_StatusTypeDef start_rx(void)
{
    if (s_uart == NULL) { return HAL_ERROR; }
    return HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
}

static void restart_rx(void)
{
    (void)HAL_UART_AbortReceive(s_uart);
    s_rx_state = 0;
    if (start_rx() != HAL_OK) { s_restart_requested = 1U; }
}

/* ============================ 公开 API ============================ */
void Xiaodianji_Init(void)
{
    memset((void *)&xiaodianji_data, 0, sizeof(xiaodianji_data));
    s_uart = &XIAODIANJI_UART_HANDLE;
    s_rx_idx = 0;
    s_rx_state = 0;
    s_restart_requested = 0;
    s_last_rx_ms = 0;

    if (start_rx() != HAL_OK) { s_restart_requested = 1U; }
}

void Xiaodianji_Run(void)
{
    if (s_uart == NULL) { return; }

    if (s_restart_requested)
    {
        s_restart_requested = 0;
        restart_rx();
    }

    /* 超时检查 */
    uint32_t now = HAL_GetTick();
    if (xiaodianji_data.online && ((now - s_last_rx_ms) > XIAODIANJI_TIMEOUT_MS))
    {
        xiaodianji_data.online = 0;
        xiaodianji_data.pose_valid = 0;
        xiaodianji_data.red_valid = 0;
        xiaodianji_data.blue_valid = 0;
        xiaodianji_data.ball_valid = 0;
    }
}

void Xiaodianji_RxCplt(UART_HandleTypeDef *huart)
{
    if (s_uart == NULL || huart != s_uart) { return; }
    feed_byte(s_rx_byte);
    if (start_rx() != HAL_OK) { s_restart_requested = 1U; }
}

void Xiaodianji_Error(UART_HandleTypeDef *huart)
{
    if (s_uart != NULL && huart == s_uart) { s_restart_requested = 1U; }
}

void Xiaodianji_SendStatus(uint8_t state, uint8_t error)
{
    uint8_t frame[8];
    if (s_uart == NULL) { return; }
    frame[0] = 0x55U;   /* 注意: STM32→上位机帧头是 55 AA(反过来) */
    frame[1] = 0xAAU;
    frame[2] = XDJ_TYPE_STATUS_TX;
    frame[3] = state;
    frame[4] = error;
    frame[5] = (uint8_t)(frame[2] + frame[3] + frame[4]);   /* checksum = 偏移2-4的累加和 */
    frame[6] = XDJ_TAIL0;
    frame[7] = XDJ_TAIL1;
    (void)HAL_UART_Transmit(s_uart, frame, 8U, 20U);
}
