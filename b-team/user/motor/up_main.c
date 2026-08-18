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
#define DM_MOTOR_L_ID    0x05U
#define DM_MOTOR_R_ID    0x07U
#define DM_MOTOR_L_RX_ID 0x15U
#define DM_MOTOR_R_RX_ID 0x17U
#define M2006_MOTOR_L_ID 1U
#define M2006_MOTOR_R_ID 2U

#define RS_HOME_ANGLE_DEG    0.0f
#define RS_MIT_SPEED_RAD_S   0.0f
#define RS_MIT_TORQUE_NM     0.0f
#define RS_MIT_KP            80.0f
#define RS_MIT_KD            2.0f
#define DM_HOME_ANGLE_DEG    0.0f
#define DM_HOME_SPEED_RAD_S  0.2f
#define DM_HOME_TORQUE_NM    0.0f
#define DM_HOME_KP           70.0f
#define DM_HOME_KD           1.25f
#define M2006_HOME_ANGLE_DEG 0.0f
#define M2006_PID_KP         30.0f
#define M2006_PID_KI         0.0f
#define M2006_PID_KD         4.0f
#define MOTOR_WORK_ZERO_DEG (-UP_SECOND_ZERO_OFFSET_DEG)
#define MOTOR_MOVE_TIME_MS   2000U
#define MOTOR_DONE_ERROR_DEG 3.0f
#define MOTOR_START_DELAY_MS 20U
#define MOTOR_START_RETRY_MS 100U
#define MOTOR_START_TIMEOUT_MS 15000U
#define MOTOR_REBASE_DELAY_MS   5U
#define MOTOR_REBASE_TIMEOUT_MS 5000U
#define RAD_TO_DEG           57.2957795f
/* DM 协议位置范围换算到输出轴后的角度上限。 */
#define DM_MAX_OUTPUT_ANGLE_DEG (DM_J4310_P_MAX * RAD_TO_DEG * \
                                 DM_APP_MOTOR_TO_OUTPUT_RATIO)

/* RS、DM 和 M2006 的接线 ID 与默认控制参数集中 */
static const rs_app_motor_config_t rs_motor_config[] = {
    {
        .id = RS_MOTOR_L_ID,
        .direction = 1,
        .period_ms = RS_APP_CONTROL_PERIOD_MS,
        .command = {
            .mode = RS_MOTION,
            .data.motion = {
                .angle_deg = RS_HOME_ANGLE_DEG,
                .speed_rad_s = RS_MIT_SPEED_RAD_S,
                .torque_nm = RS_MIT_TORQUE_NM,
                .kp = RS_MIT_KP,
                .kd = RS_MIT_KD
            }
        }
    },
    {
        .id = RS_MOTOR_R_ID,
        .direction = -1,
        .period_ms = RS_APP_CONTROL_PERIOD_MS,
        .command = {
            .mode = RS_MOTION,
            .data.motion = {
                .angle_deg = RS_HOME_ANGLE_DEG,
                .speed_rad_s = RS_MIT_SPEED_RAD_S,
                .torque_nm = RS_MIT_TORQUE_NM,
                .kp = RS_MIT_KP,
                .kd = RS_MIT_KD
            }
        }
    }
};

static const dm_app_motor_config_t dm_motor_config[] = {
    {
        .tx_id = DM_MOTOR_L_ID,
        .master_id = DM_MOTOR_L_RX_ID,
        .feedback_id = DM_MOTOR_L_ID,
        .direction = 1,
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
        .direction = -1,
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
        .direction = -1,
        .target_position_deg = M2006_HOME_ANGLE_DEG,
        .pid = {
            .kp = M2006_PID_KP,
            .ki = M2006_PID_KI,
            .kd = M2006_PID_KD
        }
    },
    {
        .id = M2006_MOTOR_R_ID,
        .direction = 1,
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
static bool app_ready;
static bool bus_off_handled;
static uint32_t start_due_ms;
static uint32_t state_start_ms;
static uint8_t start_motor_index;
static bool start_cmd_sent;

typedef enum
{
    UP_REBASE_IDLE,
    UP_REBASE_RS_WAIT_DISABLED,
    UP_REBASE_RS_WAIT_ZERO,
    UP_REBASE_RS_WAIT_READY,
    UP_REBASE_DM_WAIT_DISABLED,
    UP_REBASE_DM_WAIT_ZERO,
    UP_REBASE_DM_WAIT_READY
} up_rebase_state_t;

static up_rebase_state_t rebase_state;
static uint32_t rebase_start_ms;
static uint32_t rebase_due_ms;

up_motor_angles_t up_target_angles;
up_motor_angles_t up_command_angles;
HAL_StatusTypeDef up_last_result = HAL_OK;
bool up_curve_running;
up_state_t up_state = UP_STATE_INIT;
up_zero_step_t up_zero_step = UP_ZERO_DISABLE_RS;
float up_rs_feedforward_nm;
float up_dm_feedforward_nm;

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
    if (!Up_IsReady() ||
        (rebase_state != UP_REBASE_IDLE) || !motor_angles_valid(target))
    {
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }

    curve_start_angles = up_command_angles;
    up_target_angles = *target;
    curve_start_ms = HAL_GetTick();
    up_rs_feedforward_nm = 0.0f;
    up_dm_feedforward_nm = 0.0f;
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

typedef enum
{
    UP_MOTOR_GROUP_RS = 1U << 0,
    UP_MOTOR_GROUP_DM = 1U << 1,
    UP_MOTOR_GROUP_ALL = UP_MOTOR_GROUP_RS | UP_MOTOR_GROUP_DM
} up_motor_group_t;

static HAL_StatusTypeDef set_motor_angles(const up_motor_angles_t *angles,
                                          uint8_t groups)
{
    rs_command_t rs_command = rs_motor_config[0].command;
    dm_app_mit_command_t dm_command = dm_motor_config[0].command;

    if ((groups & UP_MOTOR_GROUP_RS) != 0U)
    {
        rs_command.data.motion.angle_deg = angles->rs_l_deg;
        rs_command.data.motion.torque_nm = up_rs_feedforward_nm;
        if (RsApp_SetCmd(&rs_app, RS_MOTOR_L_ID, &rs_command) != HAL_OK)
        {
            return HAL_ERROR;
        }
        rs_command = rs_motor_config[1].command;
        rs_command.data.motion.angle_deg = angles->rs_r_deg;
        rs_command.data.motion.torque_nm = up_rs_feedforward_nm;
        if (RsApp_SetCmd(&rs_app, RS_MOTOR_R_ID, &rs_command) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    if ((groups & UP_MOTOR_GROUP_DM) != 0U)
    {
        dm_command.angle_deg = angles->dm_l_deg;
        dm_command.torque_nm = up_dm_feedforward_nm;
        if (DmApp_SetMitCmd(&dm_app, DM_MOTOR_L_ID, &dm_command) != DM_OK)
        {
            return HAL_ERROR;
        }
        dm_command = dm_motor_config[1].command;
        dm_command.angle_deg = angles->dm_r_deg;
        dm_command.torque_nm = up_dm_feedforward_nm;
        if (DmApp_SetMitCmd(&dm_app, DM_MOTOR_R_ID, &dm_command) != DM_OK)
        {
            return HAL_ERROR;
        }
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

    up_curve_running = false;
    up_rs_feedforward_nm = 0.0f;
    up_dm_feedforward_nm = 0.0f;
    rebase_state = UP_REBASE_IDLE;
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

    up_last_result = set_motor_angles(&up_command_angles, UP_MOTOR_GROUP_ALL);
    if (up_last_result != HAL_OK)
    {
        up_curve_running = false;
        stop_outputs();
        up_state = UP_STATE_ERROR;
    }
}

static HAL_StatusTypeDef start_rs_motor(uint8_t index)
{
    const rs_app_motor_config_t *config = &rs_motor_config[index];
    rs_command_t command = config->command;
    HAL_StatusTypeDef status;

    command.data.motion.angle_deg = (index == 0U)
                                        ? up_command_angles.rs_l_deg
                                        : up_command_angles.rs_r_deg;
    if (RsApp_SetCmd(&rs_app, config->id, &command) != HAL_OK)
    {
        return HAL_ERROR;
    }
    status = RsApp_Enable(&rs_app, config->id, true);
    if (status != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef start_dm_motor(uint8_t index)
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
    result = DmApp_Enable(&dm_app, config->tx_id, true);
    if (result != DM_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static bool rs_pair_ready(uint32_t now_ms)
{
    rs_app_status_t left;
    rs_app_status_t right;

    if (!RsApp_GetStatus(&rs_app, RS_MOTOR_L_ID, now_ms, &left) ||
        !RsApp_GetStatus(&rs_app, RS_MOTOR_R_ID, now_ms, &right))
    {
        return false;
    }

    return left.online && right.online && left.active && right.active &&
           ((left.feedback.valid & RS_FDB_POSITION) != 0U) &&
           ((right.feedback.valid & RS_FDB_POSITION) != 0U) &&
           (left.feedback.fault == 0U) && (right.feedback.fault == 0U) &&
           (abs_float(left.feedback.angle_deg) <= MOTOR_DONE_ERROR_DEG) &&
           (abs_float(right.feedback.angle_deg) <= MOTOR_DONE_ERROR_DEG);
}

static bool rs_pair_disabled(uint32_t now_ms)
{
    rs_app_status_t left;
    rs_app_status_t right;

    if (!RsApp_GetStatus(&rs_app, RS_MOTOR_L_ID, now_ms, &left) ||
        !RsApp_GetStatus(&rs_app, RS_MOTOR_R_ID, now_ms, &right))
    {
        return false;
    }

    return !left.active && !right.active;
}

static bool dm_pair_active(uint32_t now_ms)
{
    dm_app_status_t left;
    dm_app_status_t right;

    if (!DmApp_GetStatus(&dm_app, DM_MOTOR_L_ID, now_ms, &left) ||
        !DmApp_GetStatus(&dm_app, DM_MOTOR_R_ID, now_ms, &right))
    {
        return false;
    }

    return left.online && right.online && left.active && right.active &&
           (left.feedback.fault == DM_FAULT_NONE) &&
           (right.feedback.fault == DM_FAULT_NONE);
}

static bool dm_pair_disabled(uint32_t now_ms)
{
    dm_app_status_t left;
    dm_app_status_t right;

    if (!DmApp_GetStatus(&dm_app, DM_MOTOR_L_ID, now_ms, &left) ||
        !DmApp_GetStatus(&dm_app, DM_MOTOR_R_ID, now_ms, &right))
    {
        return false;
    }

    /* During the second zero, active stays set until disabled feedback arrives. */
    return !left.active && !right.active;
}

static bool dm_pair_ready(uint32_t now_ms)
{
    dm_app_status_t left;
    dm_app_status_t right;

    if (!DmApp_GetStatus(&dm_app, DM_MOTOR_L_ID, now_ms, &left) ||
        !DmApp_GetStatus(&dm_app, DM_MOTOR_R_ID, now_ms, &right))
    {
        return false;
    }

    return left.online && right.online && left.active && right.active &&
           (left.feedback.fault == DM_FAULT_NONE) &&
           (right.feedback.fault == DM_FAULT_NONE) &&
           (abs_float(left.feedback.angle_deg) <= MOTOR_DONE_ERROR_DEG) &&
           (abs_float(right.feedback.angle_deg) <= MOTOR_DONE_ERROR_DEG);
}

static void reset_rs_coordinates(void)
{
    up_command_angles.rs_l_deg = 0.0f;
    up_command_angles.rs_r_deg = 0.0f;
    up_target_angles.rs_l_deg = 0.0f;
    up_target_angles.rs_r_deg = 0.0f;
    curve_start_angles.rs_l_deg = 0.0f;
    curve_start_angles.rs_r_deg = 0.0f;
}

static void reset_dm_coordinates(void)
{
    up_command_angles.dm_l_deg = 0.0f;
    up_command_angles.dm_r_deg = 0.0f;
    up_target_angles.dm_l_deg = 0.0f;
    up_target_angles.dm_r_deg = 0.0f;
    curve_start_angles.dm_l_deg = 0.0f;
    curve_start_angles.dm_r_deg = 0.0f;
}

static void start_all_motor_curve(float target_angle_deg, uint32_t now_ms)
{
    curve_start_angles = up_command_angles;
    up_target_angles.rs_l_deg = target_angle_deg;
    up_target_angles.rs_r_deg = target_angle_deg;
    up_target_angles.dm_l_deg = target_angle_deg;
    up_target_angles.dm_r_deg = target_angle_deg;
    curve_start_ms = now_ms;
    up_rs_feedforward_nm = 0.0f;
    up_dm_feedforward_nm = 0.0f;
    up_curve_running = true;
    up_last_result = HAL_OK;
}

static bool all_motors_at_angle(uint32_t now_ms, float angle_deg)
{
    rs_app_status_t rs_left;
    rs_app_status_t rs_right;
    dm_app_status_t dm_left;
    dm_app_status_t dm_right;

    if (!RsApp_GetStatus(&rs_app, RS_MOTOR_L_ID, now_ms, &rs_left) ||
        !RsApp_GetStatus(&rs_app, RS_MOTOR_R_ID, now_ms, &rs_right) ||
        !DmApp_GetStatus(&dm_app, DM_MOTOR_L_ID, now_ms, &dm_left) ||
        !DmApp_GetStatus(&dm_app, DM_MOTOR_R_ID, now_ms, &dm_right))
    {
        return false;
    }

    return rs_left.online && rs_right.online && dm_left.online &&
           dm_right.online && rs_left.active && rs_right.active &&
           dm_left.active && dm_right.active &&
           (rs_left.feedback.fault == 0U) &&
           (rs_right.feedback.fault == 0U) &&
           (dm_left.feedback.fault == DM_FAULT_NONE) &&
           (dm_right.feedback.fault == DM_FAULT_NONE) &&
           (abs_float(rs_left.feedback.angle_deg - angle_deg) <=
            MOTOR_DONE_ERROR_DEG) &&
           (abs_float(rs_right.feedback.angle_deg - angle_deg) <=
            MOTOR_DONE_ERROR_DEG) &&
           (abs_float(dm_left.feedback.angle_deg - angle_deg) <=
            MOTOR_DONE_ERROR_DEG) &&
           (abs_float(dm_right.feedback.angle_deg - angle_deg) <=
            MOTOR_DONE_ERROR_DEG);
}

static void set_zero_step(up_zero_step_t next_step, uint32_t now_ms)
{
    up_zero_step = next_step;
    start_motor_index = 0U;
    start_cmd_sent = false;
    start_due_ms = now_ms + MOTOR_START_DELAY_MS;
}

static void retry_zero_step(uint32_t now_ms)
{
    up_last_result = HAL_ERROR;
    start_cmd_sent = false;
    start_due_ms = now_ms + MOTOR_START_RETRY_MS;
}

static HAL_StatusTypeDef run_zero_sequence(uint32_t now_ms)
{
    HAL_StatusTypeDef rs_result;
    dm_result_t dm_result;

    if (!time_reached(now_ms, start_due_ms))
    {
        return HAL_BUSY;
    }

    switch (up_zero_step)
    {
    case UP_ZERO_DISABLE_RS:
        if (start_motor_index >= ARRAY_SIZE(rs_motor_config))
        {
            set_zero_step(UP_ZERO_DISABLE_DM, now_ms);
            break;
        }
        rs_result = RsApp_Enable(
            &rs_app, rs_motor_config[start_motor_index].id, false);
        if (rs_result == HAL_OK)
        {
            start_motor_index++;
            start_due_ms = now_ms + MOTOR_START_DELAY_MS;
        }
        else
        {
            retry_zero_step(now_ms);
        }
        break;

    case UP_ZERO_DISABLE_DM:
        if (start_motor_index >= ARRAY_SIZE(dm_motor_config))
        {
            set_zero_step(UP_ZERO_WAIT_DISABLED, now_ms);
            break;
        }
        dm_result = DmApp_Enable(
            &dm_app, dm_motor_config[start_motor_index].tx_id, false);
        if (dm_result == DM_OK)
        {
            start_motor_index++;
            start_due_ms = now_ms + MOTOR_START_DELAY_MS;
        }
        else
        {
            retry_zero_step(now_ms);
        }
        break;

    case UP_ZERO_WAIT_DISABLED:
        if (rs_pair_disabled(now_ms) && dm_pair_disabled(now_ms))
        {
            set_zero_step(UP_ZERO_CLEAR_RS_FAULT, now_ms);
        }
        break;

    case UP_ZERO_CLEAR_RS_FAULT:
        if (start_motor_index >= ARRAY_SIZE(rs_motor_config))
        {
            set_zero_step(UP_ZERO_SET_RS, now_ms);
            break;
        }
        rs_result = RsApp_ClearFault(
            &rs_app, rs_motor_config[start_motor_index].id);
        if (rs_result == HAL_OK)
        {
            start_motor_index++;
            start_due_ms = now_ms + MOTOR_START_DELAY_MS;
        }
        else
        {
            retry_zero_step(now_ms);
        }
        break;

    case UP_ZERO_SET_RS:
        if (start_motor_index >= ARRAY_SIZE(rs_motor_config))
        {
            reset_rs_coordinates();
            set_zero_step(UP_ZERO_SET_DM, now_ms);
            break;
        }
        if (!start_cmd_sent)
        {
            rs_result = RsApp_SetZero(
                &rs_app, rs_motor_config[start_motor_index].id);
            if (rs_result == HAL_OK)
            {
                start_cmd_sent = true;
            }
            else if (rs_result != HAL_BUSY)
            {
                retry_zero_step(now_ms);
            }
            break;
        }
        rs_result = RsApp_GetZeroStatus(
            &rs_app, rs_motor_config[start_motor_index].id);
        if (rs_result == HAL_OK)
        {
            start_motor_index++;
            start_cmd_sent = false;
            start_due_ms = now_ms + MOTOR_START_DELAY_MS;
        }
        else if (rs_result != HAL_BUSY)
        {
            retry_zero_step(now_ms);
        }
        break;

    case UP_ZERO_SET_DM:
        if (start_motor_index >= ARRAY_SIZE(dm_motor_config))
        {
            reset_dm_coordinates();
            set_zero_step(UP_ZERO_ENABLE_DM, now_ms);
            break;
        }
        if (!start_cmd_sent)
        {
            dm_result = DmApp_SetZero(
                &dm_app, dm_motor_config[start_motor_index].tx_id);
            if (dm_result == DM_OK)
            {
                start_cmd_sent = true;
            }
            else if (dm_result != DM_BUSY)
            {
                retry_zero_step(now_ms);
            }
            break;
        }
        dm_result = DmApp_GetZeroStatus(
            &dm_app, dm_motor_config[start_motor_index].tx_id);
        if (dm_result == DM_OK)
        {
            start_motor_index++;
            start_cmd_sent = false;
            start_due_ms = now_ms + MOTOR_START_DELAY_MS;
        }
        else if (dm_result != DM_BUSY)
        {
            retry_zero_step(now_ms);
        }
        break;

    case UP_ZERO_ENABLE_DM:
        if (start_motor_index < ARRAY_SIZE(dm_motor_config))
        {
            if (start_dm_motor(start_motor_index) != HAL_OK)
            {
                retry_zero_step(now_ms);
                break;
            }
            start_motor_index++;
            start_due_ms = now_ms + MOTOR_START_DELAY_MS;
            break;
        }
        if (dm_pair_active(now_ms))
        {
            set_zero_step(UP_ZERO_ENABLE_RS, now_ms);
        }
        break;

    case UP_ZERO_ENABLE_RS:
        if (start_motor_index < ARRAY_SIZE(rs_motor_config))
        {
            if (start_rs_motor(start_motor_index) != HAL_OK)
            {
                retry_zero_step(now_ms);
                break;
            }
            start_motor_index++;
            start_due_ms = now_ms + MOTOR_START_DELAY_MS;
            break;
        }
        set_zero_step(UP_ZERO_WAIT_READY, now_ms);
        break;

    case UP_ZERO_WAIT_READY:
        if (rs_pair_ready(now_ms) && dm_pair_ready(now_ms))
        {
            return HAL_OK;
        }
        break;

    default:
        return HAL_ERROR;
    }

    return HAL_BUSY;
}

static void fail_motor_state(void)
{
    up_last_result = HAL_TIMEOUT;
    stop_outputs();
    up_state = UP_STATE_ERROR;
}

static void run_motor_state(uint32_t now_ms)
{
    HAL_StatusTypeDef zero_result;

    if ((up_state == UP_STATE_READY) || (up_state == UP_STATE_STOPPED) ||
        (up_state == UP_STATE_ERROR))
    {
        return;
    }
    switch (up_state)
    {
    case UP_STATE_FIRST_ZERO:
        /* 在上电位置完成第一次标零。 */
        zero_result = run_zero_sequence(now_ms);
        if (zero_result == HAL_OK)
        {
            start_all_motor_curve(MOTOR_WORK_ZERO_DEG, now_ms);
            up_state = UP_STATE_MOVE_SECOND_ZERO;
            state_start_ms = now_ms;
        }
        else if (zero_result != HAL_BUSY)
        {
            fail_motor_state();
        }
        break;

    case UP_STATE_MOVE_SECOND_ZERO:
        /* 四台电机同步转到带偏移的待机位置。 */
        update_motor_curve(now_ms);
        if (!up_curve_running &&
            all_motors_at_angle(now_ms, MOTOR_WORK_ZERO_DEG))
        {
            set_zero_step(UP_ZERO_DISABLE_RS, now_ms);
            up_state = UP_STATE_SECOND_ZERO;
            state_start_ms = now_ms;
        }
        break;

    case UP_STATE_SECOND_ZERO:
        /* 将偏移后的待机位置重新定义为工作零点。 */
        zero_result = run_zero_sequence(now_ms);
        if (zero_result == HAL_OK)
        {
            up_state = UP_STATE_READY;
            up_last_result = HAL_OK;
        }
        else if (zero_result != HAL_BUSY)
        {
            fail_motor_state();
        }
        break;

    case UP_STATE_INIT:
    default:
        fail_motor_state();
        break;
    }

    if ((up_state != UP_STATE_READY) &&
        (up_state != UP_STATE_STOPPED) &&
        (up_state != UP_STATE_ERROR) &&
        ((uint32_t)(now_ms - state_start_ms) > MOTOR_START_TIMEOUT_MS))
    {
        fail_motor_state();
    }
}

static void fail_position_rebase(void)
{
    up_last_result = HAL_ERROR;
    stop_outputs();
    up_state = UP_STATE_ERROR;
}

static void update_position_rebase(uint32_t now_ms)
{
    rs_app_status_t rs_left;
    rs_app_status_t rs_right;
    dm_app_status_t dm_left;
    dm_app_status_t dm_right;

    if (rebase_state == UP_REBASE_IDLE)
    {
        return;
    }
    if ((uint32_t)(now_ms - rebase_start_ms) > MOTOR_REBASE_TIMEOUT_MS)
    {
        fail_position_rebase();
        return;
    }

    switch (rebase_state)
    {
    case UP_REBASE_RS_WAIT_DISABLED:
        if (!RsApp_GetStatus(&rs_app, RS_MOTOR_L_ID, now_ms, &rs_left) ||
            !RsApp_GetStatus(&rs_app, RS_MOTOR_R_ID, now_ms, &rs_right))
        {
            fail_position_rebase();
            return;
        }
        if (!rs_left.active && !rs_right.active &&
            time_reached(now_ms, rebase_due_ms))
        {
            start_motor_index = 0U;
            start_cmd_sent = false;
            rebase_state = UP_REBASE_RS_WAIT_ZERO;
        }
        break;

    case UP_REBASE_RS_WAIT_ZERO:
        if (start_motor_index >= ARRAY_SIZE(rs_motor_config))
        {
            reset_rs_coordinates();
            start_motor_index = 0U;
            rebase_state = UP_REBASE_RS_WAIT_READY;
            break;
        }
        if (!start_cmd_sent)
        {
            if (RsApp_SetZero(&rs_app,
                              rs_motor_config[start_motor_index].id) == HAL_OK)
            {
                start_cmd_sent = true;
            }
            break;
        }
        if (RsApp_GetZeroStatus(&rs_app,
                rs_motor_config[start_motor_index].id) == HAL_OK)
        {
            start_motor_index++;
            start_cmd_sent = false;
        }
        break;

    case UP_REBASE_RS_WAIT_READY:
        if (start_motor_index < ARRAY_SIZE(rs_motor_config))
        {
            if (start_rs_motor(start_motor_index) == HAL_OK)
            {
                start_motor_index++;
            }
        }
        else if (rs_pair_ready(now_ms))
        {
            rebase_state = UP_REBASE_IDLE;
            up_last_result = HAL_OK;
        }
        break;

    case UP_REBASE_DM_WAIT_DISABLED:
        if (!DmApp_GetStatus(&dm_app, DM_MOTOR_L_ID, now_ms, &dm_left) ||
            !DmApp_GetStatus(&dm_app, DM_MOTOR_R_ID, now_ms, &dm_right))
        {
            fail_position_rebase();
            return;
        }
        if (!dm_left.active && !dm_right.active &&
            time_reached(now_ms, rebase_due_ms))
        {
            start_motor_index = 0U;
            start_cmd_sent = false;
            rebase_state = UP_REBASE_DM_WAIT_ZERO;
        }
        break;

    case UP_REBASE_DM_WAIT_ZERO:
        if (start_motor_index >= ARRAY_SIZE(dm_motor_config))
        {
            reset_dm_coordinates();
            start_motor_index = 0U;
            rebase_state = UP_REBASE_DM_WAIT_READY;
            break;
        }
        if (!start_cmd_sent)
        {
            if (DmApp_SetZero(&dm_app,
                    dm_motor_config[start_motor_index].tx_id) == DM_OK)
            {
                start_cmd_sent = true;
            }
            break;
        }
        if (DmApp_GetZeroStatus(&dm_app,
                dm_motor_config[start_motor_index].tx_id) == DM_OK)
        {
            start_motor_index++;
            start_cmd_sent = false;
        }
        break;

    case UP_REBASE_DM_WAIT_READY:
        if (start_motor_index < ARRAY_SIZE(dm_motor_config))
        {
            if (start_dm_motor(start_motor_index) == HAL_OK)
            {
                start_motor_index++;
            }
        }
        else if (dm_pair_ready(now_ms))
        {
            rebase_state = UP_REBASE_IDLE;
            up_last_result = HAL_OK;
        }
        break;

    case UP_REBASE_IDLE:
    default:
        break;
    }
}

static bool rs_motors_at_target(uint32_t now_ms)
{
    rs_app_status_t left;
    rs_app_status_t right;

    if (!RsApp_GetStatus(&rs_app, RS_MOTOR_L_ID, now_ms, &left) ||
        !RsApp_GetStatus(&rs_app, RS_MOTOR_R_ID, now_ms, &right))
    {
        return false;
    }

    return left.online && right.online &&
           (left.feedback.fault == 0U) && (right.feedback.fault == 0U) &&
           ((left.feedback.valid & RS_FDB_POSITION) != 0U) &&
           ((right.feedback.valid & RS_FDB_POSITION) != 0U) &&
           (abs_float(left.feedback.angle_deg - up_target_angles.rs_l_deg) <=
            MOTOR_DONE_ERROR_DEG) &&
           (abs_float(right.feedback.angle_deg - up_target_angles.rs_r_deg) <=
             MOTOR_DONE_ERROR_DEG);
}

static bool dm_motors_at_target(uint32_t now_ms)
{
    dm_app_status_t left;
    dm_app_status_t right;

    if (!DmApp_GetStatus(&dm_app, DM_MOTOR_L_ID, now_ms, &left) ||
        !DmApp_GetStatus(&dm_app, DM_MOTOR_R_ID, now_ms, &right))
    {
        return false;
    }

    return left.online && right.online &&
           (left.feedback.fault == DM_FAULT_NONE) &&
           (right.feedback.fault == DM_FAULT_NONE) &&
           (abs_float(left.feedback.angle_deg - up_target_angles.dm_l_deg) <=
            MOTOR_DONE_ERROR_DEG) &&
           (abs_float(right.feedback.angle_deg - up_target_angles.dm_r_deg) <=
             MOTOR_DONE_ERROR_DEG);
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
    up_curve_running = false;
    up_rs_feedforward_nm = 0.0f;
    up_dm_feedforward_nm = 0.0f;
    rebase_state = UP_REBASE_IDLE;
    up_state = UP_STATE_FIRST_ZERO;
    state_start_ms = HAL_GetTick();
    set_zero_step(UP_ZERO_DISABLE_RS, state_start_ms);
    up_last_result = HAL_OK;
    return HAL_OK;
}

HAL_StatusTypeDef Up_HomeMotors(void)
{
    if (!app_ready || Fdcan_BusOff())
    {
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }

    stop_outputs();
    reset_rs_coordinates();
    reset_dm_coordinates();
    up_state = UP_STATE_FIRST_ZERO;
    state_start_ms = HAL_GetTick();
    set_zero_step(UP_ZERO_DISABLE_RS, state_start_ms);
    up_last_result = HAL_BUSY;
    return HAL_OK;
}

bool Up_IsReady(void)
{
    return app_ready && !Fdcan_BusOff() &&
           (up_state == UP_STATE_READY);
}

static bool m2006_control_ready(void)
{
    return app_ready && !Fdcan_StdBusOff();
}

bool Up_IsM2006Ready(void)
{
    m2006_status_t left;
    m2006_status_t right;

    return m2006_control_ready() &&
           C610_GetStatus(&c610_bus, M2006_MOTOR_L_ID, &left) &&
           C610_GetStatus(&c610_bus, M2006_MOTOR_R_ID, &right) &&
           left.online && right.online;
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

HAL_StatusTypeDef Up_SetFeedforward(float rs_torque_nm, float dm_torque_nm)
{
    if (!Up_IsReady() || !finite_float(rs_torque_nm) ||
        !finite_float(dm_torque_nm))
    {
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }

    up_rs_feedforward_nm = rs_torque_nm;
    up_dm_feedforward_nm = dm_torque_nm;
    up_last_result = set_motor_angles(&up_command_angles,
                                      UP_MOTOR_GROUP_ALL);
    return up_last_result;
}

bool Up_GetMotorAngles(up_motor_angles_t *angles)
{
    rs_app_status_t rs_left;
    rs_app_status_t rs_right;
    dm_app_status_t dm_left;
    dm_app_status_t dm_right;
    uint32_t now_ms = HAL_GetTick();

    if ((angles == NULL) ||
        !RsApp_GetStatus(&rs_app, RS_MOTOR_L_ID, now_ms, &rs_left) ||
        !RsApp_GetStatus(&rs_app, RS_MOTOR_R_ID, now_ms, &rs_right) ||
        !DmApp_GetStatus(&dm_app, DM_MOTOR_L_ID, now_ms, &dm_left) ||
        !DmApp_GetStatus(&dm_app, DM_MOTOR_R_ID, now_ms, &dm_right))
    {
        return false;
    }

    angles->rs_l_deg = rs_left.feedback.angle_deg;
    angles->rs_r_deg = rs_right.feedback.angle_deg;
    angles->dm_l_deg = dm_left.feedback.angle_deg;
    angles->dm_r_deg = dm_right.feedback.angle_deg;

    return rs_left.online && rs_right.online && dm_left.online &&
           dm_right.online && rs_left.active && rs_right.active &&
           dm_left.active && dm_right.active &&
           ((rs_left.feedback.valid & RS_FDB_POSITION) != 0U) &&
           ((rs_right.feedback.valid & RS_FDB_POSITION) != 0U) &&
           (rs_left.feedback.fault == 0U) &&
           (rs_right.feedback.fault == 0U) &&
           (dm_left.feedback.fault == DM_FAULT_NONE) &&
           (dm_right.feedback.fault == DM_FAULT_NONE);
}

bool Up_MotorMoveDone(void)
{
    uint32_t now_ms = HAL_GetTick();

    return Up_IsReady() && !up_curve_running &&
           (rebase_state == UP_REBASE_IDLE) &&
           rs_motors_at_target(now_ms) && dm_motors_at_target(now_ms);
}

bool Up_RsMoveDone(void)
{
    return Up_IsReady() && !up_curve_running &&
           (rebase_state == UP_REBASE_IDLE) &&
           rs_motors_at_target(HAL_GetTick());
}

bool Up_DmMoveDone(void)
{
    return Up_IsReady() && !up_curve_running &&
           (rebase_state == UP_REBASE_IDLE) &&
           dm_motors_at_target(HAL_GetTick());
}

/**
 * @brief 设置两台 RS00 的输出轴目标角度
 * @param left_deg 左侧 RS00 输出轴目标角度(deg)
 * @param right_deg 右侧 RS00 输出轴目标角度(deg)
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_SetRsPos(float left_deg, float right_deg)
{
    up_motor_angles_t target = up_target_angles;

    target.rs_l_deg = left_deg;
    target.rs_r_deg = right_deg;
    return start_motor_curve(&target);
}

/**
 * @brief 设置两台达妙电机的输出轴目标角度
 * @param left_deg 左侧达妙输出轴目标角度(deg)
 * @param right_deg 右侧达妙输出轴目标角度(deg)
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_SetDmPos(float left_deg, float right_deg)
{
    up_motor_angles_t target = up_target_angles;

    target.dm_l_deg = left_deg;
    target.dm_r_deg = right_deg;
    return start_motor_curve(&target);
}

HAL_StatusTypeDef Up_RebaseRsPosition(void)
{
    if (!Up_IsReady() || up_curve_running ||
        (rebase_state != UP_REBASE_IDLE) ||
        (RsApp_Enable(&rs_app, RS_MOTOR_L_ID, false) != HAL_OK) ||
        (RsApp_Enable(&rs_app, RS_MOTOR_R_ID, false) != HAL_OK))
    {
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }

    rebase_start_ms = HAL_GetTick();
    rebase_due_ms = rebase_start_ms + MOTOR_REBASE_DELAY_MS;
    rebase_state = UP_REBASE_RS_WAIT_DISABLED;
    up_last_result = HAL_OK;
    return HAL_OK;
}

HAL_StatusTypeDef Up_RebaseDmPosition(void)
{
    if (!Up_IsReady() || up_curve_running ||
        (rebase_state != UP_REBASE_IDLE) ||
        (DmApp_Enable(&dm_app, DM_MOTOR_L_ID, false) != DM_OK) ||
        (DmApp_Enable(&dm_app, DM_MOTOR_R_ID, false) != DM_OK))
    {
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }

    rebase_start_ms = HAL_GetTick();
    rebase_due_ms = rebase_start_ms + MOTOR_REBASE_DELAY_MS;
    rebase_state = UP_REBASE_DM_WAIT_DISABLED;
    up_last_result = HAL_OK;
    return HAL_OK;
}

HAL_StatusTypeDef Up_GetPositionRebaseStatus(void)
{
    if (!Up_IsReady())
    {
        return HAL_ERROR;
    }
    if (rebase_state != UP_REBASE_IDLE)
    {
        return HAL_BUSY;
    }
    return up_last_result;
}

HAL_StatusTypeDef Up_SetM2006Pos(uint8_t id, float position_deg)
{
    HAL_StatusTypeDef status;

    if (!m2006_control_ready())
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

    if (!Up_IsM2006Ready() || (offset_deg != offset_deg) ||
        (offset_deg > FLT_MAX) || (offset_deg < -FLT_MAX))
    {
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }
    if (!C610_GetStatus(&c610_bus, M2006_MOTOR_L_ID, &left_status) ||
        !C610_GetStatus(&c610_bus, M2006_MOTOR_R_ID, &right_status))
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

/**
 * @brief 关闭两台 M2006 的位置控制并输出零电流
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_CoastM2006(void)
{
    if (!app_ready)
    {
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }

    up_last_result = C610_CoastAll(&c610_bus);
    return up_last_result;
}

void Up_Run1ms(void)
{
    uint32_t now_ms;
    bool rs_bus_off;
    bool std_bus_off;

    if (!app_ready)
    {
        return;
    }

    now_ms = HAL_GetTick();
    Fdcan_Run1ms();
    rs_bus_off = Fdcan_RsBusOff();
    std_bus_off = Fdcan_StdBusOff();
    if (rs_bus_off || std_bus_off)
    {
        if (!bus_off_handled)
        {
            up_last_result = HAL_ERROR;
            bus_off_handled = true;
        }
    }
    else
    {
        bus_off_handled = false;
    }

    /* 先处理反馈和周期发送，再推进启动、标零及运动状态机。 */
    if (!rs_bus_off)
    {
        RsApp_Run(&rs_app, now_ms);
    }
    if (!std_bus_off)
    {
        DmApp_Run(&dm_app, now_ms);
        C610_Run(&c610_bus, now_ms);
    }
    if (rs_bus_off || std_bus_off)
    {
        return;
    }
    if (up_state != UP_STATE_READY)
    {
        run_motor_state(now_ms);
        return;
    }
    update_position_rebase(now_ms);
    update_motor_curve(now_ms);
}

void Up_StopAll(void)
{
    stop_outputs();
    up_state = UP_STATE_STOPPED;
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
