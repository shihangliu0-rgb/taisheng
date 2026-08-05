#ifndef DM_APP_H
#define DM_APP_H

#include "dm_2006_bus.h"
#include "dm_motor.h"

#include <stdbool.h>
#include <stdint.h>

#define DM_APP_MAX_MOTORS          8U
#define DM_APP_CONTROL_PERIOD_MS   1U
#define DM_APP_FEEDBACK_TIMEOUT_MS 50U

typedef struct
{
    float angle_deg;
    float speed_rad_s;
    float torque_nm;
    float kp;
    float kd;
} dm_app_mit_command_t;

typedef struct
{
    uint16_t tx_id;
    uint16_t master_id;
    uint8_t feedback_id;
    dm_app_mit_command_t command;
} dm_app_motor_config_t;

typedef struct
{
    float angle_deg;
    float speed_rad_s;
    float torque_nm;
    uint8_t mos_temp_c;
    uint8_t rotor_temp_c;
    dm_fault_t fault;
    uint32_t rx_tick_ms;
    uint32_t rx_count;
} dm_app_feedback_t;

typedef struct
{
    dm_motor_t motor;
    dm_app_mit_command_t command;
    dm_result_t last_result;
    uint32_t next_control_ms; // 此电机下一次允许发送控制帧的时间
    bool enable_requested;    // 上层期望的使能状态
    bool active;              // 已成功发送的实际使能状态
} dm_app_motor_t;

typedef struct
{
    std_can_t *bus;
    dm_app_motor_t motors[DM_APP_MAX_MOTORS];
    uint8_t motor_count;
    bool ready;
} dm_app_t;

typedef struct
{
    dm_app_feedback_t feedback;
    dm_result_t last_result;
    bool has_feedback;
    bool online;
    bool enable_requested;
    bool active;
} dm_app_status_t;

dm_result_t DmApp_Init(dm_app_t *app, std_can_t *bus,
                       const dm_app_motor_config_t *config, uint8_t count);
void DmApp_Run(dm_app_t *app, uint32_t now_ms);
dm_result_t DmApp_SetMitCmd(dm_app_t *app, uint16_t id,
                            const dm_app_mit_command_t *command);
dm_result_t DmApp_Enable(dm_app_t *app, uint16_t id, bool enabled);
dm_result_t DmApp_Restart(dm_app_t *app, uint16_t id);
dm_result_t DmApp_SetZero(dm_app_t *app, uint16_t id);
void DmApp_StopAll(dm_app_t *app);
bool DmApp_GetStatus(const dm_app_t *app, uint16_t id, uint32_t now_ms,
                     dm_app_status_t *status);

#endif
