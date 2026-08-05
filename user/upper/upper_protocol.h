#ifndef UPPER_PROTOCOL_H
#define UPPER_PROTOCOL_H

/* =====================================================================
 *  user/upper —— 上位机调试协议 (MCU 侧)
 *  对接 upper_computer 仓库 docs/PROTOCOL.md v1.1.0
 *  帧格式:  AA 55 | LEN | CMD | SEQ | PAYLOAD[LEN] | CRC16(LE)
 *           CRC16/MODBUS 计算范围 = LEN + CMD + SEQ + PAYLOAD
 *           CMD bit7: 0 = 上位机->下位机, 1 = 下位机->上位机
 * ===================================================================== */

#include "stm32h7xx_hal.h"
#include "upper_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 帧头 ---------------- */
#define UPPER_HEAD0              0xAAU
#define UPPER_HEAD1              0x55U

/* ---------------- 命令号：上位机 -> 下位机 ---------------- */
#define UPPER_CMD_HEARTBEAT      0x01U
#define UPPER_CMD_MOTION         0x02U
#define UPPER_CMD_BRAKE          0x03U
#define UPPER_CMD_MODE           0x04U
#define UPPER_CMD_PID_READ       0x10U
#define UPPER_CMD_PID_WRITE      0x11U
#define UPPER_CMD_PID_SAVE       0x12U
#define UPPER_CMD_PID_TARGET     0x13U
#define UPPER_CMD_PARAM_READ     0x20U
#define UPPER_CMD_PARAM_WRITE    0x21U
#define UPPER_CMD_TELEM_CTRL     0x30U
#define UPPER_CMD_STREAM_CFG     0x31U
#define UPPER_CMD_STREAM_LIST    0x32U
#define UPPER_CMD_RAW_TEXT       0x7FU

/* ---------------- 命令号：下位机 -> 上位机 ---------------- */
#define UPPER_CMD_ACK            0x80U
#define UPPER_CMD_PID_VALUE      0x90U
#define UPPER_CMD_TELEMETRY      0x91U
#define UPPER_CMD_PID_CURVE      0x92U
#define UPPER_CMD_STREAM_DATA    0x93U
#define UPPER_CMD_STREAM_INFO    0x94U
#define UPPER_CMD_LOG            0x9FU

/* ---------------- ACK status ---------------- */
#define UPPER_ACK_OK             0x00U
#define UPPER_ACK_ERR_CRC        0x01U
#define UPPER_ACK_ERR_CMD        0x02U
#define UPPER_ACK_ERR_ARG        0x03U
#define UPPER_ACK_ERR_BUSY       0x04U

/* ---------------- MODE 枚举 ---------------- */
#define UPPER_MODE_IDLE          0x00U
#define UPPER_MODE_MANUAL        0x01U
#define UPPER_MODE_AUTO          0x02U
#define UPPER_MODE_PID_TUNE      0x03U

/* ---------------- MOTION flags ---------------- */
#define UPPER_MOTION_F_BRAKE     0x01U
#define UPPER_MOTION_F_BOOST     0x02U
#define UPPER_MOTION_F_SLOW      0x04U
#define UPPER_MOTION_F_FIELD     0x08U

/* ---------------- 可订阅通道 ID（节选，详见 PROTOCOL.md 第 3.5 节） ---------------- */
#define UPPER_CH_IMU_ROLL        0x01U
#define UPPER_CH_IMU_PITCH       0x02U
#define UPPER_CH_IMU_YAW         0x03U
#define UPPER_CH_GYRO_X          0x10U
#define UPPER_CH_GYRO_Y          0x11U
#define UPPER_CH_GYRO_Z          0x12U
#define UPPER_CH_ACCEL_X         0x20U
#define UPPER_CH_ACCEL_Y         0x21U
#define UPPER_CH_ACCEL_Z         0x22U
#define UPPER_CH_ODOM_X          0x40U
#define UPPER_CH_ODOM_Y          0x41U
#define UPPER_CH_ODOM_THETA      0x42U
#define UPPER_CH_VEL_X           0x44U
#define UPPER_CH_VEL_Y           0x45U
#define UPPER_CH_VEL_OMEGA       0x46U
#define UPPER_CH_WHEEL_LF_SPD    0x50U
#define UPPER_CH_WHEEL_RF_SPD    0x51U
#define UPPER_CH_WHEEL_LR_SPD    0x52U
#define UPPER_CH_WHEEL_RR_SPD    0x53U
#define UPPER_CH_BATTERY_V       0x60U
#define UPPER_CH_BATTERY_A       0x61U
#define UPPER_CH_CPU_LOAD        0x62U


/* =====================================================================
 *  对外 API
 *  形态与原 user/com_link(computer_link) 完全一致，便于直接替换：
 *    ComputerLink_Init/Run/RxCplt/Error  ->  Upper_Init/Run/RxCplt/Error
 * ===================================================================== */
#ifdef UPPER_DEBUG

/* 初始化并启动串口 IT 接收。uart 默认传 &huart4（遵循 H7 既有配置） */
HAL_StatusTypeDef Upper_Init(UART_HandleTypeDef *uart);

/* 在通信任务里周期调用：处理 watchdog、出队执行命令、发送队列、
 * 周期遥测/数据流上报。建议放在 StartCommTask 的循环里(每 1ms)。 */
void Upper_Run(void);

/* 放到 HAL_UART_RxCpltCallback() 中 */
void Upper_RxCplt(UART_HandleTypeDef *uart);

/* 放到 HAL_UART_ErrorCallback() 中 */
void Upper_Error(UART_HandleTypeDef *uart);

/* ---------- 业务侧主动上报接口 ---------- */
/* 文本日志到上位机控制台。level: 0=DEBUG 1=INFO 2=WARN 3=ERROR */
void Upper_SendLog(uint8_t level, const char *text);

/* 周期遥测(0x91)：状态栏/曲线用 */
void Upper_SendTelemetry(uint32_t ts_ms, int16_t vx, int16_t vy, int16_t omega,
                         int16_t yaw_cdeg, uint16_t battery_mv,
                         uint8_t state, uint8_t err_code);

/* PID 调试曲线(0x92) */
void Upper_SendPidCurve(uint8_t pid_id, uint32_t ts_ms,
                        float target, float feedback, float output);

#else /* !UPPER_DEBUG —— 关闭态：全部退化为空实现，零影响 */

static inline HAL_StatusTypeDef Upper_Init(UART_HandleTypeDef *uart) { (void)uart; return HAL_OK; }
static inline void Upper_Run(void) {}
static inline void Upper_RxCplt(UART_HandleTypeDef *uart) { (void)uart; }
static inline void Upper_Error(UART_HandleTypeDef *uart) { (void)uart; }
static inline void Upper_SendLog(uint8_t level, const char *text) { (void)level; (void)text; }
static inline void Upper_SendTelemetry(uint32_t ts_ms, int16_t vx, int16_t vy, int16_t omega,
                                       int16_t yaw_cdeg, uint16_t battery_mv,
                                       uint8_t state, uint8_t err_code)
{
    (void)ts_ms; (void)vx; (void)vy; (void)omega;
    (void)yaw_cdeg; (void)battery_mv; (void)state; (void)err_code;
}
static inline void Upper_SendPidCurve(uint8_t pid_id, uint32_t ts_ms,
                                      float target, float feedback, float output)
{
    (void)pid_id; (void)ts_ms; (void)target; (void)feedback; (void)output;
}

#endif /* UPPER_DEBUG */

#ifdef __cplusplus
}
#endif
#endif /* UPPER_PROTOCOL_H */
