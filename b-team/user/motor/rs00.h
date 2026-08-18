#ifndef RS00_H
#define RS00_H

#include "rs_bus.h"

#include <stdbool.h>

#define RS_MOTOR_ID_MAX 0x7FU

typedef enum
{
    RS_MOTION = 0,
    RS_PP = 1,
    RS_SPD = 2,
    RS_IQ = 3,
    RS_CSP = 5
} rs_mode_t;

typedef enum
{
    RS_RESET = 0,
    RS_CALI = 1,
    RS_RUN = 2
} rs_state_t;

typedef enum
{
    RS_START_IDLE = 0U,
    RS_START_MODE = 1U,
    RS_START_ENABLE = 2U,
    RS_START_WAIT = 3U
} rs_start_step_t;

typedef enum
{
    RS_FDB_POSITION = 1U << 0,
    RS_FDB_SPEED = 1U << 1,
    RS_FDB_TORQUE = 1U << 2,
    RS_FDB_TEMP = 1U << 3,
    RS_FDB_STATE = 1U << 4,
    RS_FDB_FAULT = 1U << 5
} rs_feedback_valid_t;

typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;
    uint32_t fault;
    uint32_t warning;
    uint32_t valid;
    uint32_t sequence;
    uint8_t state;
} rs_feedback_t;

typedef struct
{
    uint32_t sent_ms;
    uint32_t feedback_sequence;
    HAL_StatusTypeDef result;
    bool pending;
    bool completed;
} rs_zero_state_t;

typedef struct
{
    rs_bus_t *bus;
    rs_feedback_t feedback;
    rs_zero_state_t zero;
    uint32_t last_rx_ms;
    float pp_speed_rad_s;
    float pp_acceleration_rad_s2;
    float limit_speed_rad_s;
    float limit_iq;
    uint32_t transition_due_ms; /* 非阻塞模式切换的下一步时间。 */
    uint8_t id;
    uint8_t mode;
    bool active;
    bool pp_configured;
    uint8_t limit_valid;        /* 已写入电机的限幅参数位图。 */
    rs_start_step_t start_step; /* 模式切换状态机步骤。 */
    uint8_t requested_mode;     /* 当前状态机的目标模式。 */
} rs_motor_t;

/** @brief 初始化一台 RS00 电机。 */
HAL_StatusTypeDef RsMotor_Init(rs_motor_t *motor, rs_bus_t *bus, uint8_t id);

/** @brief 写入当前位置为机械零点，不用于普通位置回零 */
HAL_StatusTypeDef RsMotor_SetZero(rs_motor_t *motor);
/** @brief 查询机械标零结果，等待新反馈时返回 HAL_BUSY。 */
HAL_StatusTypeDef RsMotor_GetZeroStatus(rs_motor_t *motor);

/** @brief 非阻塞地停止、配置并使能指定控制模式。 */
HAL_StatusTypeDef RsMotor_Start(rs_motor_t *motor, rs_mode_t mode,
                                uint32_t now_ms);
/** @brief 失能 RS00 电机。 */
HAL_StatusTypeDef RsMotor_Stop(rs_motor_t *motor);
/** @brief 发送清除故障命令并失能电机。 */
HAL_StatusTypeDef RsMotor_ClearFault(rs_motor_t *motor);

/** @brief 发送 MIT 运动控制帧。 */
HAL_StatusTypeDef RsMotor_SetMotion(rs_motor_t *motor, float position_rad,
                                    float velocity_rad_s, float torque_nm,
                                    float kp, float kd);
/** @brief 设置电流控制目标。 */
HAL_StatusTypeDef RsMotor_SetIq(rs_motor_t *motor, float iq);
/** @brief 设置速度控制目标和最大电流。 */
HAL_StatusTypeDef RsMotor_SetSpeed(rs_motor_t *motor, float velocity_rad_s,
                                   float max_iq);
/** @brief 设置循环同步位置控制目标。 */
HAL_StatusTypeDef RsMotor_SetCsp(rs_motor_t *motor, float position_rad,
                                 float max_velocity_rad_s);
/** @brief 设置点到点位置控制目标。 */
HAL_StatusTypeDef RsMotor_SetPp(rs_motor_t *motor, float position_rad,
                                float max_velocity_rad_s,
                                float acceleration_rad_s2);
/** @brief 暂停点到点运动。 */
HAL_StatusTypeDef RsMotor_HaltPp(rs_motor_t *motor);

/** @brief 复制最近一次 RS00 反馈。 */
HAL_StatusTypeDef RsMotor_GetFeedback(const rs_motor_t *motor,
                                      rs_feedback_t *feedback);
/** @brief 解析一帧 RS00 扩展帧反馈。 */
void RsMotor_Parse(rs_motor_t *motor, uint32_t id, const uint8_t data[8],
                   uint32_t tick_ms);
#endif
