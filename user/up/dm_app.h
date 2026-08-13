#ifndef DM_APP_H
#define DM_APP_H

#include "dm_2006_bus.h"
#include "dm_motor.h"

#include <stdbool.h>
#include <stdint.h>

/* 应用层调度周期和电机容量。 */
#define DM_APP_MAX_MOTORS          8U
#define DM_APP_CONTROL_PERIOD_MS   1U
#define DM_APP_FEEDBACK_TIMEOUT_MS 50U

/* 应用层命令和反馈使用输出轴单位，发送或读取时由 dm_app 换算。 */
#define DM_APP_OUTPUT_TO_MOTOR_RATIO (9.0f / 5.0f)
#define DM_APP_MOTOR_TO_OUTPUT_RATIO (5.0f / 9.0f)

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
    uint32_t next_control_ms; /* 此电机下一次允许发送控制帧的时间。 */
    bool enable_requested;    /* 上层期望的使能状态。 */
    bool active;              /* 已成功发送的实际使能状态。 */
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

/** @brief 初始化 DM 应用层电机列表并注册反馈处理。 */
dm_result_t DmApp_Init(dm_app_t *app, std_can_t *bus,
                       const dm_app_motor_config_t *config, uint8_t count);
/** @brief 按固定周期发送 DM 控制帧。 */
void DmApp_Run(dm_app_t *app, uint32_t now_ms);
/** @brief 更新一台 DM 电机的 MIT 命令。 */
dm_result_t DmApp_SetMitCmd(dm_app_t *app, uint16_t id,
                            const dm_app_mit_command_t *command);
/** @brief 请求一台 DM 电机使能或失能。 */
dm_result_t DmApp_Enable(dm_app_t *app, uint16_t id, bool enabled);
/** @brief 请求一台 DM 电机重新执行使能流程。 */
dm_result_t DmApp_Restart(dm_app_t *app, uint16_t id);
/** @brief 发送一台 DM 电机的机械零点命令。 */
dm_result_t DmApp_SetZero(dm_app_t *app, uint16_t id);
/** @brief 请求全部 DM 电机失能。 */
void DmApp_StopAll(dm_app_t *app);
/** @brief 读取一台 DM 电机的在线、故障和反馈状态。 */
bool DmApp_GetStatus(const dm_app_t *app, uint16_t id, uint32_t now_ms,
                     dm_app_status_t *status);

#endif
