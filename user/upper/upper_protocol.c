/* =====================================================================
 *  user/upper/upper_protocol.c —— 上位机调试协议 MCU 侧实现
 *  对接 upper_computer/docs/PROTOCOL.md v1.1.0
 *
 *  设计要点：
 *   - 整个文件被 UPPER_DEBUG 包裹；未定义时编译为空，零影响。
 *   - 不修改 usart.c：串口句柄由 Upper_Init(&huart4) 传入，遵循 H7 既有配置。
 *   - RX：HAL_UART_Receive_IT 单字节 -> Upper_RxCplt(ISR) 喂状态机，
 *         完整帧入队；命令执行统一放到 Upper_Run(任务上下文)，不在中断里调业务。
 *   - TX：帧编码后压入环形缓冲，Upper_Run 里用 HAL_UART_Transmit_IT 发出。
 *   - MOTION/BRAKE/看门狗 默认直连 chassis_main(Chassis_SetVelocity/StopAll)，
 *     与原 computer_link 行为一致、开箱可用；其余数据源(PID/IMU/里程计)用弱钩子，
 *     默认返回 0，业务侧可重写。
 * ===================================================================== */

#include "upper_protocol.h"

#ifdef UPPER_DEBUG

#include "chassis_main.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* ============================ 模块状态(前置定义，供下方所有函数使用) ============================ */
static UART_HandleTypeDef *upper_uart;
static uint8_t  rx_byte;
static volatile uint8_t  restart_requested;
static volatile uint32_t last_rx_ms;
static volatile bool     link_online;

/* 周期遥测 / 数据流状态 */
static volatile uint8_t  telem_on;
static volatile uint16_t telem_period;
static uint32_t          last_telem_ms;

static volatile uint8_t  stream_on;
static volatile uint16_t stream_period;
static uint8_t           sub_channels[UPPER_MAX_SUB_CH];
static volatile uint8_t  sub_count;
static uint32_t          last_stream_ms;

static volatile uint8_t  cur_mode;

/* ============================ CRC16/MODBUS ============================ */
static uint16_t upper_crc16(const uint8_t *d, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t  j;

    for (i = 0U; i < len; i++)
    {
        crc ^= (uint16_t)d[i];
        for (j = 0U; j < 8U; j++)
        {
            crc = (crc & 1U) ? ((crc >> 1) ^ 0xA001U) : (crc >> 1);
        }
    }
    return crc;   /* 低字节先发 */
}

/* ============================ 小端读写 ============================ */
static int16_t  rd_i16(const uint8_t *p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static uint16_t rd_u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void wr_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFFU); p[1] = (uint8_t)((v >> 8) & 0xFFU); }
static void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU); p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU); p[3] = (uint8_t)((v >> 24) & 0xFFU);
}
static void wr_f32(uint8_t *p, float v) { memcpy(p, &v, sizeof(float)); } /* Cortex-M 小端 */

/* ============================ 弱钩子（业务侧可重写） ============================ */
/* 读某通道当前值，供 STREAM_DATA 上报。默认 0。 */
__weak float upper_read_channel(uint8_t channel_id) { (void)channel_id; return 0.0f; }
/* 模式切换通知 */
__weak void  upper_on_mode(uint8_t mode) { (void)mode; }
/* PID 写入(仅内存)。param: [kp,ki,kd,i_limit,out_limit] */
__weak void  upper_on_pid_write(uint8_t pid_id, const float *param) { (void)pid_id; (void)param; }
/* PID 读取回填。out: [kp,ki,kd,i_limit,out_limit] */
__weak void  upper_get_pid(uint8_t pid_id, float *out_param)
{
    uint8_t i; for (i = 0U; i < 5U; i++) { out_param[i] = 0.0f; } (void)pid_id;
}
/* PID 下发目标(阶跃/方波) */
__weak void  upper_on_pid_target(uint8_t pid_id, float target) { (void)pid_id; (void)target; }
/* PID 固化到 Flash */
__weak void  upper_on_pid_save(uint8_t pid_id) { (void)pid_id; }
/* 透传文本命令 */
__weak void  upper_on_raw_text(const char *text, uint8_t len) { (void)text; (void)len; }
/* 通道清单(可选)：返回通道名，unit_out 回填单位枚举；返回 NULL 表示不支持该通道 */
__weak const char *upper_channel_name(uint8_t channel_id, uint8_t *unit_out)
{
    (void)channel_id; if (unit_out != NULL) { *unit_out = 0U; } return NULL;
}
/* 通用参数读/写通知 */
__weak void  upper_on_param(uint16_t param_id, float value, uint8_t is_write)
{
    (void)param_id; (void)value; (void)is_write;
}

/* ============================ TX 环形缓冲 ============================ */
static volatile uint16_t tx_head, tx_tail;
static uint8_t  tx_ring[UPPER_TX_RING_SIZE];
static uint8_t  tx_line[UPPER_TX_BUF_SIZE];

static uint16_t tx_count(void)
{
    return (uint16_t)((tx_head + UPPER_TX_RING_SIZE - tx_tail) % UPPER_TX_RING_SIZE);
}
static bool tx_push_byte(uint8_t b)
{
    uint16_t next = (uint16_t)((tx_head + 1U) % UPPER_TX_RING_SIZE);
    if (next == tx_tail) { return false; }   /* 满 */
    tx_ring[tx_head] = b;
    tx_head = next;
    return true;
}
static uint8_t tx_pop_byte(void)
{
    uint8_t b = tx_ring[tx_tail];
    tx_tail = (uint16_t)((tx_tail + 1U) % UPPER_TX_RING_SIZE);
    return b;
}
/* 把一整帧压入发送队列；空间不足则整帧丢弃(避免半帧) */
static void upper_enqueue_frame(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint8_t len)
{
    uint8_t  hdr[5];
    uint16_t crc;

    if (len > UPPER_MAX_PAYLOAD) { return; }
    if ((uint16_t)tx_count() > (uint16_t)(UPPER_TX_RING_SIZE - 7U - len)) { return; }

    hdr[0] = UPPER_HEAD0; hdr[1] = UPPER_HEAD1; hdr[2] = len; hdr[3] = cmd; hdr[4] = seq;
    crc = upper_crc16(&hdr[2], (uint16_t)len + 3U);

    (void)tx_push_byte(hdr[0]); (void)tx_push_byte(hdr[1]);
    (void)tx_push_byte(hdr[2]); (void)tx_push_byte(hdr[3]); (void)tx_push_byte(hdr[4]);
    for (uint8_t i = 0U; i < len; i++) { (void)tx_push_byte(payload[i]); }
    (void)tx_push_byte((uint8_t)(crc & 0xFFU));
    (void)tx_push_byte((uint8_t)((crc >> 8) & 0xFFU));
}
/* 任务上下文：把队列里的字节用 IT 发出去 */
static void upper_tx_flush(void)
{
    uint16_t n;

    if (upper_uart == NULL) { return; }
    if (upper_uart->gState != HAL_UART_STATE_READY) { return; }
    if (tx_count() == 0U) { return; }

    n = 0U;
    while ((n < UPPER_TX_BUF_SIZE) && (tx_count() > 0U))
    {
        tx_line[n++] = tx_pop_byte();
    }
    (void)HAL_UART_Transmit_IT(upper_uart, tx_line, n);
}

/* 发送 ACK */
static void upper_send_ack(uint8_t ack_cmd, uint8_t ack_seq, uint8_t status)
{
    uint8_t p[3];
    p[0] = ack_cmd; p[1] = ack_seq; p[2] = status;
    upper_enqueue_frame(UPPER_CMD_ACK, ack_seq, p, 3U);
}

/* ============================ RX 状态机 ============================ */
typedef enum { RX_HEAD0, RX_HEAD1, RX_BODY } rx_state_t;
static uint8_t     rx_buf[7U + UPPER_MAX_PAYLOAD];
static uint16_t    rx_idx;
static uint8_t     rx_len;
static rx_state_t  rx_state;

typedef struct
{
    uint8_t cmd;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[UPPER_MAX_PAYLOAD];
} upper_evt_t;
static upper_evt_t       evt_ring[UPPER_RX_RING_SIZE];
static volatile uint16_t evt_head, evt_tail;

static bool evt_push(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint8_t len)
{
    uint16_t next = (uint16_t)((evt_head + 1U) % UPPER_RX_RING_SIZE);
    if (next == evt_tail) { return false; }
    evt_ring[evt_head].cmd = cmd;
    evt_ring[evt_head].seq = seq;
    evt_ring[evt_head].len = len;
    if (len > 0U) { memcpy(evt_ring[evt_head].payload, payload, len); }
    evt_head = next;
    return true;
}
static bool evt_pop(upper_evt_t *out)
{
    if (evt_tail == evt_head) { return false; }
    *out = evt_ring[evt_tail];
    evt_tail = (uint16_t)((evt_tail + 1U) % UPPER_RX_RING_SIZE);
    return true;
}

static void rx_reset(void)
{
    rx_idx = 0U;
    rx_state = RX_HEAD0;
}

/* ISR 安全：只碰本文件静态变量 */
static void rx_feed(uint8_t b)
{
    switch (rx_state)
    {
    case RX_HEAD0:
        if (b == UPPER_HEAD0) { rx_buf[0] = b; rx_idx = 1U; rx_state = RX_HEAD1; }
        break;

    case RX_HEAD1:
        if (b == UPPER_HEAD1) { rx_buf[1] = b; rx_idx = 2U; rx_state = RX_BODY; }
        else if (b == UPPER_HEAD0) { rx_buf[0] = b; rx_idx = 1U; /* 仍 RX_HEAD1 */ }
        else { rx_reset(); }
        break;

    case RX_BODY:
        rx_buf[rx_idx++] = b;
        if (rx_idx == 3U)
        {
            rx_len = rx_buf[2];
            if (rx_len > UPPER_MAX_PAYLOAD) { rx_reset(); break; }
        }
        if (rx_idx >= (uint16_t)rx_len + 7U)
        {
            uint16_t crc_calc = upper_crc16(&rx_buf[2], (uint16_t)rx_len + 3U);
            uint16_t crc_rx   = (uint16_t)rx_buf[rx_len + 5U] | ((uint16_t)rx_buf[rx_len + 6U] << 8);
            if (crc_calc == crc_rx)
            {
                (void)evt_push(rx_buf[3], rx_buf[4], &rx_buf[5], rx_len);
            }
            rx_reset();
        }
        else if (rx_idx >= sizeof(rx_buf))
        {
            rx_reset();
        }
        break;

    default:
        rx_reset();
        break;
    }
}

static HAL_StatusTypeDef upper_start_rx(void)
{
    if (upper_uart == NULL) { return HAL_ERROR; }
    return HAL_UART_Receive_IT(upper_uart, &rx_byte, 1U);
}
static void upper_restart_rx(void)
{
    (void)HAL_UART_AbortReceive(upper_uart);
    rx_reset();
    if (upper_start_rx() != HAL_OK) { restart_requested = 1U; }
}

/* ============================ 命令处理（任务上下文） ============================ */
static void handle_motion(const uint8_t *p, uint8_t len, uint8_t seq)
{
    int16_t vx, vy, omega;
    uint8_t flags;

    if (len < 8U) { upper_send_ack(UPPER_CMD_MOTION, seq, UPPER_ACK_ERR_ARG); return; }
    vx    = rd_i16(&p[0]);
    vy    = rd_i16(&p[2]);
    omega = rd_i16(&p[4]);
    flags = p[6];

    if (flags & UPPER_MOTION_F_BRAKE)
    {
        Chassis_StopAll();
    }
    else
    {
        (void)Chassis_SetVelocity(vx, vy, omega);
    }
    upper_send_ack(UPPER_CMD_MOTION, seq, UPPER_ACK_OK);
}

static void handle_pid_read(uint8_t pid_id, uint8_t seq)
{
    uint8_t p[21];
    float   param[5];
    upper_get_pid(pid_id, param);
    p[0] = pid_id;
    wr_f32(&p[1], param[0]); wr_f32(&p[5], param[1]); wr_f32(&p[9], param[2]);
    wr_f32(&p[13], param[3]); wr_f32(&p[17], param[4]);
    upper_enqueue_frame(UPPER_CMD_PID_VALUE, seq, p, 21U);
}

static void handle_event(const upper_evt_t *e)
{
    const uint8_t *p = e->payload;
    uint8_t len = e->len;

    switch (e->cmd)
    {
    case UPPER_CMD_HEARTBEAT:
        last_rx_ms = HAL_GetTick();
        link_online = true;
        break;

    case UPPER_CMD_MOTION:
        last_rx_ms = HAL_GetTick();
        link_online = true;
        handle_motion(p, len, e->seq);
        break;

    case UPPER_CMD_BRAKE:
        Chassis_StopAll();
        last_rx_ms = HAL_GetTick();
        link_online = true;
        upper_send_ack(UPPER_CMD_BRAKE, e->seq, UPPER_ACK_OK);
        break;

    case UPPER_CMD_MODE:
        cur_mode = (len >= 1U) ? p[0] : UPPER_MODE_IDLE;
        upper_on_mode(cur_mode);
        upper_send_ack(UPPER_CMD_MODE, e->seq, UPPER_ACK_OK);
        break;

    case UPPER_CMD_PID_READ:
        handle_pid_read((len >= 1U) ? p[0] : 0U, e->seq);
        break;

    case UPPER_CMD_PID_WRITE:
        if (len >= 21U)
        {
            float param[5];
            memcpy(&param[0], &p[1], 4); memcpy(&param[1], &p[5], 4);
            memcpy(&param[2], &p[9], 4); memcpy(&param[3], &p[13], 4);
            memcpy(&param[4], &p[17], 4);
            upper_on_pid_write(p[0], param);
            upper_send_ack(UPPER_CMD_PID_WRITE, e->seq, UPPER_ACK_OK);
        }
        else { upper_send_ack(UPPER_CMD_PID_WRITE, e->seq, UPPER_ACK_ERR_ARG); }
        break;

    case UPPER_CMD_PID_SAVE:
        upper_on_pid_save((len >= 1U) ? p[0] : 0U);
        upper_send_ack(UPPER_CMD_PID_SAVE, e->seq, UPPER_ACK_OK);
        break;

    case UPPER_CMD_PID_TARGET:
        if (len >= 5U)
        {
            float t; memcpy(&t, &p[1], 4); upper_on_pid_target(p[0], t);
        }
        upper_send_ack(UPPER_CMD_PID_TARGET, e->seq, UPPER_ACK_OK);
        break;

    case UPPER_CMD_PARAM_READ:
        upper_on_param((len >= 2U) ? rd_u16(&p[0]) : 0U, 0.0f, 0U);
        upper_send_ack(UPPER_CMD_PARAM_READ, e->seq, UPPER_ACK_OK);
        break;

    case UPPER_CMD_PARAM_WRITE:
        if (len >= 6U)
        {
            float v; memcpy(&v, &p[2], 4);
            upper_on_param(rd_u16(&p[0]), v, 1U);
        }
        upper_send_ack(UPPER_CMD_PARAM_WRITE, e->seq, UPPER_ACK_OK);
        break;

    case UPPER_CMD_TELEM_CTRL:
        if (len >= 4U)
        {
            telem_on    = p[0] ? 1U : 0U;
            telem_period = rd_u16(&p[1]);
            if (telem_period == 0U) { telem_period = UPPER_TELEM_PERIOD_MS; }
        }
        upper_send_ack(UPPER_CMD_TELEM_CTRL, e->seq, UPPER_ACK_OK);
        break;

    case UPPER_CMD_STREAM_CFG:
    {
        uint8_t  on   = (len >= 1U) ? p[0] : 0U;
        uint16_t per  = (len >= 3U) ? rd_u16(&p[1]) : 20U;
        uint8_t  cnt  = (len >= 4U) ? p[3] : 0U;
        uint8_t  i;
        if (per == 0U) { per = 20U; }
        if (cnt > UPPER_MAX_SUB_CH) { cnt = UPPER_MAX_SUB_CH; }
        for (i = 0U; i < cnt; i++) { sub_channels[i] = (len >= (uint8_t)(4U + i)) ? p[4U + i] : 0U; }
        stream_on = on ? 1U : 0U;
        stream_period = per;
        sub_count = cnt;
        upper_send_ack(UPPER_CMD_STREAM_CFG, e->seq, UPPER_ACK_OK);
        break;
    }

    case UPPER_CMD_STREAM_LIST:
    {
        /* 回报内置标准通道(节选)，让上位机动态显示 */
        static const uint8_t std_ids[] = {
            UPPER_CH_VEL_X, UPPER_CH_VEL_Y, UPPER_CH_VEL_OMEGA,
            UPPER_CH_WHEEL_LF_SPD, UPPER_CH_WHEEL_RF_SPD, UPPER_CH_WHEEL_LR_SPD, UPPER_CH_WHEEL_RR_SPD,
        };
        uint8_t i;
        for (i = 0U; i < (uint8_t)sizeof(std_ids); i++)
        {
            uint8_t     unit = 0U;
            const char *name = upper_channel_name(std_ids[i], &unit);
            uint8_t     info[UPPER_MAX_PAYLOAD];
            uint8_t     nl = 0U;
            if (name != NULL)
            {
                while (name[nl] != '\0' && nl < (UPPER_MAX_PAYLOAD - 3U)) { info[2U + nl] = (uint8_t)name[nl]; nl++; }
            }
            info[0] = std_ids[i];
            info[1] = unit;
            upper_enqueue_frame(UPPER_CMD_STREAM_INFO, e->seq, info, (uint8_t)(2U + nl));
        }
        upper_send_ack(UPPER_CMD_STREAM_LIST, e->seq, UPPER_ACK_OK);
        break;
    }

    case UPPER_CMD_RAW_TEXT:
        upper_on_raw_text((const char *)p, len);
        upper_send_ack(UPPER_CMD_RAW_TEXT, e->seq, UPPER_ACK_OK);
        break;

    default:
        upper_send_ack(e->cmd, e->seq, UPPER_ACK_ERR_CMD);
        break;
    }
    (void)len;
}

/* ============================ 周期上报 ============================ */
static void send_periodic_telemetry(uint32_t now)
{
    uint8_t  p[16];
    int16_t  vx    = (int16_t)chassis_target_vx;
    int16_t  vy    = (int16_t)chassis_target_vy;
    int16_t  omega = (int16_t)chassis_target_z;

    if (!telem_on) { return; }
    if ((now - last_telem_ms) < telem_period) { return; }
    last_telem_ms = now;

    /* 注意：这里 vx/vy/omega 用底盘【目标】速度作占位；真实反馈请改写
     * upper_read_channel 或直接调用 Upper_SendTelemetry() 上报实测值。
     * Cortex-M 为小端，直接 memcpy 即可。 */
    wr_u32(&p[0], now);
    memcpy(&p[4], &vx, 2);
    memcpy(&p[6], &vy, 2);
    memcpy(&p[8], &omega, 2);
    wr_u16(&p[10], 0U);            /* yaw 0.01deg，占位 */
    wr_u16(&p[12], 0U);            /* battery mV，占位 */
    p[14] = cur_mode;              /* state */
    p[15] = 0U;                    /* err_code */
    upper_enqueue_frame(UPPER_CMD_TELEMETRY, 0U, p, 16U);
}

static void send_periodic_stream(uint32_t now)
{
    uint8_t  buf[4U + UPPER_MAX_SUB_CH * 4U];
    uint16_t n;
    uint8_t  i;

    if (!stream_on || sub_count == 0U) { return; }
    if ((now - last_stream_ms) < stream_period) { return; }
    last_stream_ms = now;

    wr_u32(&buf[0], now);
    n = 4U;
    for (i = 0U; i < sub_count; i++)
    {
        float v = upper_read_channel(sub_channels[i]);
        wr_f32(&buf[n], v);
        n = (uint16_t)(n + 4U);
    }
    upper_enqueue_frame(UPPER_CMD_STREAM_DATA, 0U, buf, (uint8_t)n);
}

/* ============================ 对外 API ============================ */
HAL_StatusTypeDef Upper_Init(UART_HandleTypeDef *uart)
{
    if (uart == NULL) { return HAL_ERROR; }

    upper_uart = uart;
    rx_byte = 0U;
    tx_head = 0U; tx_tail = 0U;
    evt_head = 0U; evt_tail = 0U;
    restart_requested = 0U;
    last_rx_ms = 0U;
    link_online = false;
    rx_reset();

    telem_on = 0U; telem_period = UPPER_TELEM_PERIOD_MS; last_telem_ms = 0U;
    stream_on = 0U; stream_period = 20U; sub_count = 0U; last_stream_ms = 0U;
    cur_mode = UPPER_MODE_IDLE;

    if (upper_start_rx() != HAL_OK) { restart_requested = 1U; }
    return HAL_OK;
}

void Upper_Run(void)
{
    upper_evt_t e;
    uint32_t    now;

    if (upper_uart == NULL) { return; }

    if (restart_requested)
    {
        restart_requested = 0U;
        upper_restart_rx();
    }

    /* 出队执行命令（任务上下文，安全调用业务） */
    while (evt_pop(&e)) { handle_event(&e); }

    now = HAL_GetTick();

    /* 通信看门狗：超时未收帧 -> 制动/失能 */
    if (link_online && ((now - last_rx_ms) > UPPER_WATCHDOG_MS))
    {
        link_online = false;
        Chassis_StopAll();
    }

    send_periodic_telemetry(now);
    send_periodic_stream(now);

    upper_tx_flush();
}

void Upper_RxCplt(UART_HandleTypeDef *uart)
{
    if ((upper_uart == NULL) || (uart != upper_uart)) { return; }
    rx_feed(rx_byte);
    if (upper_start_rx() != HAL_OK) { restart_requested = 1U; }
}

void Upper_Error(UART_HandleTypeDef *uart)
{
    if ((upper_uart == NULL) || (uart != upper_uart)) { return; }
    restart_requested = 1U;
}

/* ---------- 业务侧主动上报 ---------- */
void Upper_SendLog(uint8_t level, const char *text)
{
    uint8_t buf[UPPER_MAX_PAYLOAD];
    uint8_t n = 0U;
    if (text == NULL) { return; }
    buf[0] = level;
    while (text[n] != '\0' && n < (UPPER_MAX_PAYLOAD - 2U)) { buf[1U + n] = (uint8_t)text[n]; n++; }
    upper_enqueue_frame(UPPER_CMD_LOG, 0U, buf, (uint8_t)(1U + n));
}

void Upper_SendTelemetry(uint32_t ts_ms, int16_t vx, int16_t vy, int16_t omega,
                         int16_t yaw_cdeg, uint16_t battery_mv,
                         uint8_t state, uint8_t err_code)
{
    uint8_t p[16];
    wr_u32(&p[0], ts_ms);
    memcpy(&p[4], &vx, 2); memcpy(&p[6], &vy, 2); memcpy(&p[8], &omega, 2);
    memcpy(&p[10], &yaw_cdeg, 2); wr_u16(&p[12], battery_mv);
    p[14] = state; p[15] = err_code;
    upper_enqueue_frame(UPPER_CMD_TELEMETRY, 0U, p, 16U);
}

void Upper_SendPidCurve(uint8_t pid_id, uint32_t ts_ms,
                        float target, float feedback, float output)
{
    uint8_t p[17];
    p[0] = pid_id;
    wr_u32(&p[1], ts_ms);
    wr_f32(&p[5], target); wr_f32(&p[9], feedback); wr_f32(&p[13], output);
    upper_enqueue_frame(UPPER_CMD_PID_CURVE, 0U, p, 17U);
}

#endif /* UPPER_DEBUG */
