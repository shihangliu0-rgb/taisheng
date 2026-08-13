#include "up_main.h"

#include "fdcan_task.h"

#include <float.h>
#include <stddef.h>

/* 仅用于本文件的静态数组遍历。 */
#define ARRAY_SIZE(array) ((uint8_t)(sizeof(array) / sizeof((array)[0])))

/* 电机总线 ID 由当前接线固定。 */
#define RS_HOST_ID       0xFDU
#define RS_MOTOR_L_ID    39U
#define RS_MOTOR_R_ID    40U
#define DM_MOTOR_L_ID    0x07U
#define DM_MOTOR_R_ID    0x05U
#define DM_MOTOR_L_RX_ID 0x17U
#define DM_MOTOR_R_RX_ID 0x15U
#define M2006_MOTOR_L_ID 1U
#define M2006_MOTOR_R_ID 2U

#define RS_HOME_ANGLE_DEG    0.0f
#define RS_HOME_SPEED_RAD_S  9.5f
#define DM_HOME_ANGLE_DEG    0.0f
#define DM_HOME_SPEED_RAD_S  0.2f
#define DM_HOME_TORQUE_NM    0.0f
#define DM_HOME_KP           70.0f
#define DM_HOME_KD           1.25f
#define M2006_HOME_ANGLE_DEG 0.0f
#define M2006_PID_KP         45.0f
#define M2006_PID_KI         0.0f
#define M2006_PID_KD         5.0f
#define MOTOR_MOVE_TIME_MS   2000U
#define MOTOR_RETRY_MS       500U
#define MOTOR_DONE_ERROR_DEG 3.0f
#define RAD_TO_DEG           57.2957795f
/* DM 协议位置范围换算到输出轴后的角度上限。 */
#define DM_MAX_OUTPUT_ANGLE_DEG (DM_J4310_P_MAX * RAD_TO_DEG * \
                                 DM_APP_MOTOR_TO_OUTPUT_RATIO)

/* RS、DM 和 M2006 的接线 ID 与默认控制参数集中在本文件。 */
static const rs_app_motor_config_t rs_motor_config[] = {
    {
        .id = RS_MOTOR_L_ID,
        .period_ms = RS_APP_CONTROL_PERIOD_MS,
        .command = {
            .mode = RS_CSP,
            .data.csp = {
                .angle_deg = RS_HOME_ANGLE_DEG,
                .max_speed_rad_s = RS_HOME_SPEED_RAD_S
            }
        }
    },
    {
        .id = RS_MOTOR_R_ID,
        .period_ms = RS_APP_CONTROL_PERIOD_MS,
        .command = {
            .mode = RS_CSP,
            .data.csp = {
                .angle_deg = RS_HOME_ANGLE_DEG,
                .max_speed_rad_s = RS_HOME_SPEED_RAD_S
            }
        }
    }
};

static const dm_app_motor_config_t dm_motor_config[] = {
    {
        .tx_id = DM_MOTOR_L_ID,
        .master_id = DM_MOTOR_L_RX_ID,
        .feedback_id = DM_MOTOR_L_ID,
        .command = {
            .angle_deg = DM_HOME_ANGLE_DEG,
            .speed_rad_s = DM_HOME_SPEED_RAD_S,
            .torque_nm = DM_HOME_TORQUE_NM,
            .kp = DM_HOME_KP,
            .kd = DM_HOME_KD
        }
    },
    {
        .tx_id = DM_MOTOR_R_ID,
        .master_id = DM_MOTOR_R_RX_ID,
        .feedback_id = DM_MOTOR_R_ID,
        .command = {
            .angle_deg = DM_HOME_ANGLE_DEG,
            .speed_rad_s = DM_HOME_SPEED_RAD_S,
            .torque_nm = DM_HOME_TORQUE_NM,
            .kp = DM_HOME_KP,
            .kd = DM_HOME_KD
        }
    }
};

static const m2006_config_t m2006_motor_config[] = {
    {
        .id = M2006_MOTOR_L_ID,
        .target_position_deg = M2006_HOME_ANGLE_DEG,
        .pid = {
            .kp = M2006_PID_KP,
            .ki = M2006_PID_KI,
            .kd = M2006_PID_KD
        }
    },
    {
        .id = M2006_MOTOR_R_ID,
        .target_position_deg = M2006_HOME_ANGLE_DEG,
        .pid = {
            .kp = M2006_PID_KP,
            .ki = M2006_PID_KI,
            .kd = M2006_PID_KD
        }
    }
};

/* FDCAN3 同时接收 DM 和 M2006 的标准帧反馈。 */
static const uint16_t std_rx_ids[] = {
    DM_MOTOR_L_RX_ID,
    DM_MOTOR_R_RX_ID,
    C610_FEEDBACK_ID(M2006_MOTOR_L_ID),
    C610_FEEDBACK_ID(M2006_MOTOR_R_ID)
};

static const fdcan_config_t fdcan_config = {
    .rs_host_id = RS_HOST_ID,
    .std_rx_ids = std_rx_ids,
    .std_rx_id_count = ARRAY_SIZE(std_rx_ids)
};

static rs_app_t rs_app;
static dm_app_t dm_app;
static c610_bus_t c610_bus;
static m2006_motor_t m2006_motors[ARRAY_SIZE(m2006_motor_config)];
static up_motor_angles_t curve_start_angles;
static uint32_t curve_start_ms;
static uint32_t rs_retry_ms[ARRAY_SIZE(rs_motor_config)];
static uint32_t dm_retry_ms[ARRAY_SIZE(dm_motor_config)];
static bool app_ready;
static bool auto_initialize_enabled;
static bool motors_initialized;
static bool bus_off_handled;

up_motor_angles_t up_target_angles;
up_motor_angles_t up_command_angles;
HAL_StatusTypeDef up_last_result = HAL_OK;
bool up_curve_running;

static bool finite_float(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static bool time_reached(uint32_t now_ms, uint32_t due_ms)
{
    return (int32_t)(now_ms - due_ms) >= 0;
}

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool motor_angles_valid(const up_motor_angles_t *angles)
{
    if ((angles == NULL) || !finite_float(angles->rs_l_deg) ||
        !finite_float(angles->rs_r_deg) ||
        !finite_float(angles->dm_l_deg) ||
        !finite_float(angles->dm_r_deg))
    {
        return false;
    }

    return (angles->dm_l_deg <= DM_MAX_OUTPUT_ANGLE_DEG) &&
           (angles->dm_l_deg >= -DM_MAX_OUTPUT_ANGLE_DEG) &&
           (angles->dm_r_deg <= DM_MAX_OUTPUT_ANGLE_DEG) &&
           (angles->dm_r_deg >= -DM_MAX_OUTPUT_ANGLE_DEG);
}

static HAL_StatusTypeDef start_motor_curve(const up_motor_angles_t *target)
{
    if (!app_ready || Fdcan_BusOff() || !motor_angles_valid(target))
    {
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }

    curve_start_angles = up_command_angles;
    up_target_angles = *target;
    curve_start_ms = HAL_GetTick();
    up_curve_running = true;
    up_last_result = HAL_OK;
    return HAL_OK;
}

static float curve_value(float start, float target, float progress)
{
    float progress_2 = progress * progress;
    float progress_3 = progress_2 * progress;
    float blend = progress_3 *
                  (10.0f + progress * (-15.0f + 6.0f * progress));

    return start + (target - start) * blend;
}

static HAL_StatusTypeDef set_motor_angles(const up_motor_angles_t *angles)
{
    rs_command_t rs_command = rs_motor_config[0].command;
    dm_app_mit_command_t dm_command = dm_motor_config[0].command;

    rs_command.data.csp.angle_deg = angles->rs_l_deg;
    if (RsApp_SetCmd(&rs_app, RS_MOTOR_L_ID, &rs_command) != HAL_OK)
    {
        return HAL_ERROR;
    }
    rs_command = rs_motor_config[1].command;
    rs_command.data.csp.angle_deg = angles->rs_r_deg;
    if (RsApp_SetCmd(&rs_app, RS_MOTOR_R_ID, &rs_command) != HAL_OK)
    {
        return HAL_ERROR;
    }

    dm_command.angle_deg = angles->dm_l_deg;
    if (DmApp_SetMitCmd(&dm_app, DM_MOTOR_L_ID, &dm_command) != DM_OK)
    {
        return HAL_ERROR;
    }
    dm_command = dm_motor_config[1].command;
    dm_command.angle_deg = angles->dm_r_deg;
    if (DmApp_SetMitCmd(&dm_app, DM_MOTOR_R_ID, &dm_command) != DM_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static void stop_outputs(void)
{
    uint32_t now_ms;

    if (!app_ready)
    {
        return;
    }

    auto_initialize_enabled = false;
    up_curve_running = false;
    RsApp_StopAll(&rs_app);
    DmApp_StopAll(&dm_app);
    C610_StopAll(&c610_bus);

    /* 正常总线下立即补发一次失能和零电流，减少停机延迟。 */
    now_ms = HAL_GetTick();
    RsApp_Run(&rs_app, now_ms);
    DmApp_Run(&dm_app, now_ms);
    C610_Run(&c610_bus, now_ms);
}

static void update_motor_curve(uint32_t now_ms)
{
    uint32_t elapsed_ms;
    float progress;

    if (!up_curve_running)
    {
        return;
    }

    elapsed_ms = now_ms - curve_start_ms;
    if (elapsed_ms >= MOTOR_MOVE_TIME_MS)
    {
        progress = 1.0f;
        up_curve_running = false;
    }
    else
    {
        progress = (float)elapsed_ms / (float)MOTOR_MOVE_TIME_MS;
    }

    up_command_angles.rs_l_deg = curve_value(
        curve_start_angles.rs_l_deg, up_target_angles.rs_l_deg, progress);
    up_command_angles.rs_r_deg = curve_value(
        curve_start_angles.rs_r_deg, up_target_angles.rs_r_deg, progress);
    up_command_angles.dm_l_deg = curve_value(
        curve_start_angles.dm_l_deg, up_target_angles.dm_l_deg, progress);
    up_command_angles.dm_r_deg = curve_value(
        curve_start_angles.dm_r_deg, up_target_angles.dm_r_deg, progress);

    up_last_result = set_motor_angles(&up_command_angles);
    if (up_last_result != HAL_OK)
    {
        up_curve_running = false;
        stop_outputs();
    }
}

static HAL_StatusTypeDef initialize_rs_motor(uint8_t index, bool restart)
{
    const rs_app_motor_config_t *config = &rs_motor_config[index];
    rs_command_t command = config->command;
    HAL_StatusTypeDef status;

    command.data.csp.angle_deg = (index == 0U) ? up_command_angles.rs_l_deg
                                               : up_command_angles.rs_r_deg;
    if (RsApp_SetCmd(&rs_app, config->id, &command) != HAL_OK)
    {
        return HAL_ERROR;
    }
    status = restart ? RsApp_Restart(&rs_app, config->id)
                     : RsApp_Enable(&rs_app, config->id, true);
    if (status != HAL_OK)
    {
        return HAL_ERROR;
    }

    rs_retry_ms[index] = HAL_GetTick() + MOTOR_RETRY_MS;
    return HAL_OK;
}

static HAL_StatusTypeDef initialize_dm_motor(uint8_t index, bool restart)
{
    const dm_app_motor_config_t *config = &dm_motor_config[index];
    dm_app_mit_command_t command = config->command;
    dm_result_t result;

    command.angle_deg = (index == 0U) ? up_command_angles.dm_l_deg
                                      : up_command_angles.dm_r_deg;
    if (DmApp_SetMitCmd(&dm_app, config->tx_id, &command) != DM_OK)
    {
        return HAL_ERROR;
    }
    result = restart ? DmApp_Restart(&dm_app, config->tx_id)
                     : DmApp_Enable(&dm_app, config->tx_id, true);
    if (result != DM_OK)
    {
        return HAL_ERROR;
    }

    dm_retry_ms[index] = HAL_GetTick() + MOTOR_RETRY_MS;
    return HAL_OK;
}

static void initialize_offline_motors(uint32_t now_ms)
{
    uint8_t i;

    if (!auto_initialize_enabled)
    {
        return;
    }

    for (i = 0U; i < ARRAY_SIZE(rs_motor_config); i++)
    {
        rs_app_status_t status;

        if (!RsApp_GetStatus(&rs_app, rs_motor_config[i].id, now_ms, &status))
        {
            continue;
        }
        if (status.online)
        {
            rs_retry_ms[i] = now_ms + MOTOR_RETRY_MS;
        }
        else if (time_reached(now_ms, rs_retry_ms[i]) &&
                 (!status.has_feedback || (status.feedback.fault == 0U)))
        {
            if (initialize_rs_motor(i, true) != HAL_OK)
            {
                up_last_result = HAL_ERROR;
                rs_retry_ms[i] = now_ms + MOTOR_RETRY_MS;
            }
        }
    }

    for (i = 0U; i < ARRAY_SIZE(dm_motor_config); i++)
    {
        dm_app_status_t status;

        if (!DmApp_GetStatus(&dm_app, dm_motor_config[i].tx_id, now_ms,
                             &status))
        {
            continue;
        }
        if (status.online)
        {
            dm_retry_ms[i] = now_ms + MOTOR_RETRY_MS;
        }
        else if (time_reached(now_ms, dm_retry_ms[i]) &&
                 (!status.has_feedback ||
                  (status.feedback.fault == DM_FAULT_NONE)))
        {
            if (initialize_dm_motor(i, true) != HAL_OK)
            {
                up_last_result = HAL_ERROR;
                dm_retry_ms[i] = now_ms + MOTOR_RETRY_MS;
            }
        }
    }
}

static bool motors_at_target(uint32_t now_ms)
{
    rs_app_status_t rs_l_status;
    rs_app_status_t rs_r_status;
    dm_app_status_t dm_l_status;
    dm_app_status_t dm_r_status;

    if (!RsApp_GetStatus(&rs_app, RS_MOTOR_L_ID, now_ms, &rs_l_status) ||
        !RsApp_GetStatus(&rs_app, RS_MOTOR_R_ID, now_ms, &rs_r_status) ||
        !DmApp_GetStatus(&dm_app, DM_MOTOR_L_ID, now_ms, &dm_l_status) ||
        !DmApp_GetStatus(&dm_app, DM_MOTOR_R_ID, now_ms, &dm_r_status))
    {
        return false;
    }

    if (!rs_l_status.online || !rs_r_status.online ||
        !dm_l_status.online || !dm_r_status.online ||
        (rs_l_status.feedback.fault != 0U) ||
        (rs_r_status.feedback.fault != 0U) ||
        (dm_l_status.feedback.fault != DM_FAULT_NONE) ||
        (dm_r_status.feedback.fault != DM_FAULT_NONE) ||
        ((rs_l_status.feedback.valid & RS_FDB_POSITION) == 0U) ||
        ((rs_r_status.feedback.valid & RS_FDB_POSITION) == 0U))
    {
        return false;
    }

    return (abs_float(rs_l_status.feedback.angle_deg -
                      up_target_angles.rs_l_deg) <= MOTOR_DONE_ERROR_DEG) &&
           (abs_float(rs_r_status.feedback.angle_deg -
                      up_target_angles.rs_r_deg) <= MOTOR_DONE_ERROR_DEG) &&
           (abs_float(dm_l_status.feedback.angle_deg -
                      up_target_angles.dm_l_deg) <= MOTOR_DONE_ERROR_DEG) &&
           (abs_float(dm_r_status.feedback.angle_deg -
                      up_target_angles.dm_r_deg) <= MOTOR_DONE_ERROR_DEG);
}

HAL_StatusTypeDef Up_Init(void)
{
    if (app_ready)
    {
        return HAL_OK;
    }

    bus_off_handled = false;

    if (Fdcan_Init(&fdcan_config) != HAL_OK)
    {
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }
    if (RsApp_Init(&rs_app, Fdcan_GetRsBus(), rs_motor_config,
                   ARRAY_SIZE(rs_motor_config)) != HAL_OK)
    {
        up_last_result = HAL_ERROR;
        Fdcan_Stop();
        return HAL_ERROR;
    }
    if (DmApp_Init(&dm_app, Fdcan_GetStdBus(), dm_motor_config,
                   ARRAY_SIZE(dm_motor_config)) != DM_OK)
    {
        up_last_result = HAL_ERROR;
        Fdcan_Stop();
        return HAL_ERROR;
    }
    if (C610_Init(&c610_bus, Fdcan_GetStdBus(), m2006_motors,
                  m2006_motor_config,
                  ARRAY_SIZE(m2006_motor_config)) != HAL_OK)
    {
        up_last_result = HAL_ERROR;
        Fdcan_Stop();
        return HAL_ERROR;
    }
    app_ready = true;
    if (Up_HomeMotors() != HAL_OK)
    {
        stop_outputs();
        app_ready = false;
        Fdcan_Stop();
        return HAL_ERROR;
    }

    up_last_result = HAL_OK;
    return up_last_result;
}

HAL_StatusTypeDef Up_HomeMotors(void)
{
    bool restart = motors_initialized;
    uint8_t i;

    if (!app_ready || Fdcan_BusOff())
    {
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }

    auto_initialize_enabled = true;
    for (i = 0U; i < ARRAY_SIZE(rs_motor_config); i++)
    {
        if (initialize_rs_motor(i, restart) != HAL_OK)
        {
            up_last_result = HAL_ERROR;
            stop_outputs();
            return HAL_ERROR;
        }
    }
    for (i = 0U; i < ARRAY_SIZE(dm_motor_config); i++)
    {
        if (initialize_dm_motor(i, restart) != HAL_OK)
        {
            up_last_result = HAL_ERROR;
            stop_outputs();
            return HAL_ERROR;
        }
    }
    motors_initialized = true;

    return Up_SetMotorPos(RS_HOME_ANGLE_DEG, RS_HOME_ANGLE_DEG,
                          DM_HOME_ANGLE_DEG, DM_HOME_ANGLE_DEG);
}

HAL_StatusTypeDef Up_SetMotorPos(float rs_l_deg, float rs_r_deg,
                                 float dm_l_deg, float dm_r_deg)
{
    up_motor_angles_t target = {
        .rs_l_deg = rs_l_deg,
        .rs_r_deg = rs_r_deg,
        .dm_l_deg = dm_l_deg,
        .dm_r_deg = dm_r_deg
    };

    return start_motor_curve(&target);
}

bool Up_MotorMoveDone(void)
{
    return app_ready && !Fdcan_BusOff() && !up_curve_running &&
           motors_at_target(HAL_GetTick());
}

/**
 * @brief 设置两台 RS00 的输出轴目标角度
 * @param angle_deg 两台 RS00 的输出轴目标角度(deg)
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_SetRsPos(float angle_deg)
{
    up_motor_angles_t target = up_target_angles;

    target.rs_l_deg = angle_deg;
    target.rs_r_deg = angle_deg;
    return start_motor_curve(&target);
}

/**
 * @brief 设置两台达妙电机的输出轴目标角度
 * @param angle_deg 两台达妙电机的输出轴目标角度(deg)
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_SetDmPos(float angle_deg)
{
    up_motor_angles_t target = up_target_angles;

    target.dm_l_deg = angle_deg;
    target.dm_r_deg = angle_deg;
    return start_motor_curve(&target);
}

HAL_StatusTypeDef Up_SetM2006Pos(uint8_t id, float position_deg)
{
    HAL_StatusTypeDef status;

    if (!app_ready || Fdcan_BusOff())
    {
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }

    status = C610_SetPos(&c610_bus, id, position_deg);
    up_last_result = status;
    return status;
}

/**
 * @brief 两台 M2006 输出轴分别相对当前位置转动指定角度
 * @param offset_deg 输出轴相对转动角度(deg)
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_MoveM2006(float offset_deg)
{
    m2006_status_t left_status;
    m2006_status_t right_status;

    if (!app_ready || Fdcan_BusOff() || (offset_deg != offset_deg) ||
        (offset_deg > FLT_MAX) || (offset_deg < -FLT_MAX))
    {
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }
    if (!C610_GetStatus(&c610_bus, M2006_MOTOR_L_ID, &left_status) ||
        !C610_GetStatus(&c610_bus, M2006_MOTOR_R_ID, &right_status) ||
        !left_status.online || !right_status.online)
    {
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }

    if (C610_SetPos(&c610_bus, M2006_MOTOR_L_ID,
                    left_status.output_angle_deg + offset_deg) != HAL_OK)
    {
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }
    up_last_result = C610_SetPos(&c610_bus, M2006_MOTOR_R_ID,
                                 right_status.output_angle_deg + offset_deg);
    return up_last_result;
}

void Up_Run1ms(void)
{
    uint32_t now_ms;

    if (!app_ready)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if (Fdcan_BusOff())
    {
        if (!bus_off_handled)
        {
            up_last_result = HAL_ERROR;
            bus_off_handled = true;
        }
        stop_outputs();
        return;
    }

    /* 固定顺序：处理反馈，再恢复状态，最后按各自周期发送控制帧。 */
    Fdcan_Run1ms();
    initialize_offline_motors(now_ms);
    update_motor_curve(now_ms);
    RsApp_Run(&rs_app, now_ms);
    DmApp_Run(&dm_app, now_ms);
    C610_Run(&c610_bus, now_ms);
}

void Up_StopAll(void)
{
    stop_outputs();
}

bool Up_GetRsStatus(uint8_t id, rs_app_status_t *status)
{
    return app_ready && RsApp_GetStatus(&rs_app, id, HAL_GetTick(), status);
}

bool Up_GetDmStatus(uint16_t id, dm_app_status_t *status)
{
    return app_ready && DmApp_GetStatus(&dm_app, id, HAL_GetTick(), status);
}

bool Up_GetM2006Status(uint8_t id, m2006_status_t *status)
{
    return app_ready && C610_GetStatus(&c610_bus, id, status);
}
