#ifndef RS00_H
#define RS00_H

#include "rs_bus.h"

#include <stdbool.h>

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
    rs_bus_t *bus;
    rs_feedback_t feedback;
    uint32_t last_rx_ms;
    float pp_speed_rad_s;
    float pp_acceleration_rad_s2;
    float limit_speed_rad_s;
    float limit_iq;
    uint32_t transition_due_ms; // 非阻塞模式切换的下一步时间
    uint8_t id;
    uint8_t mode;
    bool active;
    bool pp_configured;
    uint8_t limit_valid;        // 已写入电机的限幅参数位图
    uint8_t start_step;         // 模式切换状态机步骤
    uint8_t requested_mode;     // 当前状态机的目标模式
} rs_motor_t;

HAL_StatusTypeDef RsMotor_Init(rs_motor_t *motor, rs_bus_t *bus, uint8_t id);

/** @brief 写入当前位置为机械零点，不用于普通位置回零 */
HAL_StatusTypeDef RsMotor_SetZero(rs_motor_t *motor);

HAL_StatusTypeDef RsMotor_Start(rs_motor_t *motor, rs_mode_t mode,
                                uint32_t now_ms);
HAL_StatusTypeDef RsMotor_Stop(rs_motor_t *motor);
HAL_StatusTypeDef RsMotor_ClearFault(rs_motor_t *motor);

HAL_StatusTypeDef RsMotor_SetMotion(rs_motor_t *motor, float position_rad,
                                    float velocity_rad_s, float torque_nm,
                                    float kp, float kd);
HAL_StatusTypeDef RsMotor_SetIq(rs_motor_t *motor, float iq);
HAL_StatusTypeDef RsMotor_SetSpeed(rs_motor_t *motor, float velocity_rad_s,
                                   float max_iq);
HAL_StatusTypeDef RsMotor_SetCsp(rs_motor_t *motor, float position_rad,
                                 float max_velocity_rad_s);
HAL_StatusTypeDef RsMotor_SetPp(rs_motor_t *motor, float position_rad,
                                float max_velocity_rad_s,
                                float acceleration_rad_s2);
HAL_StatusTypeDef RsMotor_HaltPp(rs_motor_t *motor);

HAL_StatusTypeDef RsMotor_GetFeedback(const rs_motor_t *motor,
                                      rs_feedback_t *feedback);
void RsMotor_Parse(rs_motor_t *motor, uint32_t id, const uint8_t data[8],
                   uint32_t tick_ms);
#endif
