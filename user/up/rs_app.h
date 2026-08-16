#ifndef RS_APP_H
#define RS_APP_H

#include "rs00.h"

#include <stdbool.h>

/* 应用层调度周期和电机容量。 */
#define RS_APP_MAX_MOTORS          8U
#define RS_APP_CONTROL_PERIOD_MS   10U
#define RS_APP_FEEDBACK_TIMEOUT_MS 50U

/* 应用层命令和反馈使用输出轴单位，发送或读取时由 rs_app 换算。 */
typedef struct
{
    float angle_deg;
    float speed_rad_s;
    float torque_nm;
    float kp;
    float kd;
} rs_motion_command_t;

typedef struct
{
    float iq;
} rs_iq_command_t;

typedef struct
{
    float speed_rad_s;
    float max_iq;
} rs_speed_command_t;

typedef struct
{
    float angle_deg;
    float max_speed_rad_s;
} rs_csp_command_t;

typedef struct
{
    float angle_deg;
    float max_speed_rad_s;
    float acceleration_rad_s2;
} rs_pp_command_t;

typedef union
{
    rs_motion_command_t motion;
    rs_iq_command_t iq;
    rs_speed_command_t speed;
    rs_csp_command_t csp;
    rs_pp_command_t pp;
} rs_command_data_t;

typedef struct
{
    rs_mode_t mode;
    rs_command_data_t data;
} rs_command_t;

typedef struct
{
    uint8_t id;
    int8_t direction;
    uint32_t period_ms;
    rs_command_t command;
} rs_app_motor_config_t;

typedef struct
{
    rs_motor_t motor;
    rs_command_t command;
    int8_t direction;
    HAL_StatusTypeDef last_result;
    uint32_t period_ms;
    uint32_t next_control_ms; /* 此电机下一次允许发送控制帧的时间。 */
    bool enable_requested;    /* 上层请求，实际 active 由驱动反馈决定。 */
} rs_app_motor_t;

typedef struct
{
    float angle_deg;
    float speed_rad_s;
    float torque_nm;
    float temperature_c;
    uint32_t fault;
    uint32_t warning;
    uint32_t valid;
    uint32_t sequence;
    uint8_t state;
} rs_app_feedback_t;

typedef struct
{
    rs_bus_t *bus;
    rs_app_motor_t motors[RS_APP_MAX_MOTORS];
    uint8_t motor_count;
    bool ready;
} rs_app_t;

typedef struct
{
    rs_app_feedback_t feedback;
    HAL_StatusTypeDef last_result;
    bool has_feedback;
    bool online;
    bool enable_requested;
    bool active;
} rs_app_status_t;

/** @brief 初始化 RS00 应用层电机列表并注册反馈处理。 */
HAL_StatusTypeDef RsApp_Init(rs_app_t *app, rs_bus_t *bus,
                             const rs_app_motor_config_t *config,
                             uint8_t count);
/** @brief 按各电机周期执行使能、模式切换和控制发送。 */
void RsApp_Run(rs_app_t *app, uint32_t now_ms);
/** @brief 更新一台 RS00 的控制命令。 */
HAL_StatusTypeDef RsApp_SetCmd(rs_app_t *app, uint8_t id,
                               const rs_command_t *command);
/** @brief 请求一台 RS00 使能或失能。 */
HAL_StatusTypeDef RsApp_Enable(rs_app_t *app, uint8_t id, bool enabled);
/** @brief 请求一台 RS00 重新执行启动流程。 */
HAL_StatusTypeDef RsApp_Restart(rs_app_t *app, uint8_t id);
/** @brief 写入一台 RS00 的机械零点。 */
HAL_StatusTypeDef RsApp_SetZero(rs_app_t *app, uint8_t id);
/** @brief 查询一台 RS00 的机械标零结果。 */
HAL_StatusTypeDef RsApp_GetZeroStatus(rs_app_t *app, uint8_t id);
/** @brief 清除一台 RS00 的故障。 */
HAL_StatusTypeDef RsApp_ClearFault(rs_app_t *app, uint8_t id);
/** @brief 请求全部 RS00 失能。 */
void RsApp_StopAll(rs_app_t *app);
/** @brief 读取一台 RS00 的在线、故障和反馈状态。 */
bool RsApp_GetStatus(const rs_app_t *app, uint8_t id, uint32_t now_ms,
                     rs_app_status_t *status);
#endif
