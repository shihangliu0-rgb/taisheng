#ifndef C610_2006_H
#define C610_2006_H

#include "dm_2006_bus.h"

#include <stdbool.h>
#include <stdint.h>

#define C610_MAX_MOTORS         8U
#define C610_FEEDBACK_ID_BASE   0x201U
#define C610_CONTROL_PERIOD_MS  2U
#define C610_FEEDBACK_ID(id) \
    ((uint16_t)(C610_FEEDBACK_ID_BASE + (uint16_t)(id) - 1U))

typedef struct
{
    float kp;
    float ki;
    float kd;
} m2006_pid_gains_t;

typedef struct
{
    uint8_t id;
    float target_position_deg;
    m2006_pid_gains_t pid;
} m2006_config_t;

typedef struct
{
    m2006_pid_gains_t gains;
    float integral;
    float last_error;
    float integral_limit;
    float output_limit;
    bool started;
} m2006_pid_t;

typedef struct
{
    m2006_pid_t pid;
    uint8_t id;
    uint16_t rotor_angle;
    int32_t rotor_total; // 跨编码器零点累计的转子计数
    int16_t rotor_rpm;
    int16_t current_feedback;
    int16_t current_command;
    float target_position_deg;
    float output_speed_rpm;
    float output_angle_deg;
    uint32_t last_feedback_ms;
    bool feedback_seen;
    bool online;
    bool enabled;
} m2006_motor_t;

typedef struct
{
    std_can_t *can;
    m2006_motor_t *motors[C610_MAX_MOTORS];
    m2006_motor_t *motor_list;
    uint32_t last_control_ms; // M2006 组控制的 2 ms 调度基准
    uint8_t motor_count;
    bool group_used[2]; // ID 1-4 与 5-8 两个组控制帧
    bool ready;
} c610_bus_t;

typedef struct
{
    float output_angle_deg;
    float output_speed_rpm;
    int16_t current_feedback;
    int16_t current_command;
    uint32_t last_feedback_ms;
    bool feedback_seen;
    bool online;
    bool enabled;
} m2006_status_t;

HAL_StatusTypeDef C610_Init(c610_bus_t *bus, std_can_t *can,
                            m2006_motor_t *motors,
                            const m2006_config_t *configs,
                            uint8_t motor_count);
void C610_Run(c610_bus_t *bus, uint32_t now_ms);
/**
 * @brief 设置目标位置并使能对应 M2006
 * @param id 电机 ID
 * @param target_position_deg 输出轴目标角度(deg)
 * @retval HAL 状态
 */
HAL_StatusTypeDef C610_SetPos(c610_bus_t *bus, uint8_t id,
                              float target_position_deg);
HAL_StatusTypeDef C610_SetPid(c610_bus_t *bus, uint8_t id,
                              m2006_pid_gains_t gains);
void C610_Enable(c610_bus_t *bus, uint8_t id, bool enabled);
void C610_StopAll(c610_bus_t *bus);
bool C610_GetStatus(const c610_bus_t *bus, uint8_t id,
                    m2006_status_t *status);

#endif
