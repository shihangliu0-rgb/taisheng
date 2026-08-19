#include "up_main.h"

#include "fdcan_task.h"

#include <float.h>

#define RS_COUNT                 2U
#define RS_HOST_ID               0xFDU
#define RS_MOTOR_L_ID            39U
#define RS_MOTOR_R_ID            40U
#define RS_CONTROL_KP            80.0f
#define RS_CONTROL_KD            2.0f
#define MOTOR_WORK_ZERO_DEG      (-UP_SECOND_ZERO_OFFSET_DEG)
#define MOTOR_MOVE_TIME_MS       2000U
#define MOTOR_DONE_ERROR_DEG     3.0f
#define MOTOR_START_DELAY_MS     20U
#define MOTOR_START_RETRY_MS     100U
#define MOTOR_START_TIMEOUT_MS   15000U
#define MOTOR_REBASE_DELAY_MS    5U
#define MOTOR_REBASE_TIMEOUT_MS  5000U

static const rs_app_motor_config_t rs_motor_config[RS_COUNT] = {
    {
        .id = RS_MOTOR_L_ID,
        .direction = 1,
        .period_ms = RS_APP_CONTROL_PERIOD_MS,
        .command = {
            .mode = RS_MOTION,
            .data.motion = {
                .angle_deg = 0.0f,
                .speed_rad_s = 0.0f,
                .torque_nm = 0.0f,
                .kp = RS_CONTROL_KP,
                .kd = RS_CONTROL_KD
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
                .angle_deg = 0.0f,
                .speed_rad_s = 0.0f,
                .torque_nm = 0.0f,
                .kp = RS_CONTROL_KP,
                .kd = RS_CONTROL_KD
            }
        }
    }
};

static const fdcan_config_t fdcan_config = { .rs_host_id = RS_HOST_ID };
static rs_app_t rs_app;
static bool app_ready;
static bool bus_off_handled;
static uint32_t curve_start_ms;
static float curve_start_left_deg;
static float curve_start_right_deg;
static uint32_t start_due_ms;
static uint32_t state_start_ms;
static uint32_t rebase_start_ms;
static uint32_t rebase_due_ms;
static uint8_t start_motor_index;
static bool start_cmd_sent;

typedef enum
{
    REBASE_IDLE,
    REBASE_WAIT_DISABLED,
    REBASE_WAIT_ZERO,
    REBASE_WAIT_READY
} rebase_state_t;

static rebase_state_t rebase_state;

up_motor_angles_t up_target_angles;
up_motor_angles_t up_command_angles;
HAL_StatusTypeDef up_last_result = HAL_OK;
bool up_curve_running;
up_state_t up_state = UP_STATE_INIT;
up_zero_step_t up_zero_step = UP_ZERO_DISABLE_RS;

static bool time_reached(uint32_t now_ms, uint32_t due_ms)
{
    return (int32_t)(now_ms - due_ms) >= 0;
}

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool finite_float(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static float curve_value(float start, float target, float progress)
{
    float p2 = progress * progress;
    float p3 = p2 * progress;
    float blend = p3 * (10.0f + progress * (-15.0f + 6.0f * progress));

    return start + (target - start) * blend;
}

static void reset_coordinates(void)
{
    up_target_angles.rs_l_deg = 0.0f;
    up_target_angles.rs_r_deg = 0.0f;
    up_command_angles.rs_l_deg = 0.0f;
    up_command_angles.rs_r_deg = 0.0f;
}

static HAL_StatusTypeDef set_motor_angles(const up_motor_angles_t *angles)
{
    rs_command_t command = rs_motor_config[0].command;

    command.data.motion.angle_deg = angles->rs_l_deg;
    if (RsApp_SetCmd(&rs_app, RS_MOTOR_L_ID, &command) != HAL_OK)
    {
        return HAL_ERROR;
    }
    command = rs_motor_config[1].command;
    command.data.motion.angle_deg = angles->rs_r_deg;
    return (RsApp_SetCmd(&rs_app, RS_MOTOR_R_ID, &command) == HAL_OK) ?
           HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef start_rs_motor(uint8_t index)
{
    rs_command_t command = rs_motor_config[index].command;
    uint8_t id = rs_motor_config[index].id;

    command.data.motion.angle_deg = (index == 0U) ?
                                     up_command_angles.rs_l_deg :
                                     up_command_angles.rs_r_deg;
    if (RsApp_SetCmd(&rs_app, id, &command) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return (RsApp_Enable(&rs_app, id, true) == HAL_OK) ?
           HAL_OK : HAL_ERROR;
}

static bool rs_pair_status(uint32_t now_ms, rs_app_status_t *left,
                           rs_app_status_t *right)
{
    return RsApp_GetStatus(&rs_app, RS_MOTOR_L_ID, now_ms, left) &&
           RsApp_GetStatus(&rs_app, RS_MOTOR_R_ID, now_ms, right);
}

static bool rs_pair_disabled(uint32_t now_ms)
{
    rs_app_status_t left;
    rs_app_status_t right;

    return rs_pair_status(now_ms, &left, &right) &&
           !left.active && !right.active;
}

static bool rs_pair_ready(uint32_t now_ms)
{
    rs_app_status_t left;
    rs_app_status_t right;

    if (!rs_pair_status(now_ms, &left, &right))
    {
        return false;
    }
    return left.active && right.active &&
           (left.feedback.fault == 0U) && (right.feedback.fault == 0U);
}

static bool rs_pair_at_target(uint32_t now_ms, float left_target,
                              float right_target)
{
    rs_app_status_t left;
    rs_app_status_t right;

    if (!rs_pair_status(now_ms, &left, &right))
    {
        return false;
    }
    if ((left.feedback.valid & RS_FDB_POSITION) == 0U ||
        (right.feedback.valid & RS_FDB_POSITION) == 0U)
    {
        return true;
    }
    return (left.feedback.fault == 0U) && (right.feedback.fault == 0U) &&
           (abs_float(left.feedback.angle_deg - left_target) <=
            MOTOR_DONE_ERROR_DEG) &&
           (abs_float(right.feedback.angle_deg - right_target) <=
            MOTOR_DONE_ERROR_DEG);
}

static void set_zero_step(up_zero_step_t step, uint32_t now_ms)
{
    up_zero_step = step;
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
    HAL_StatusTypeDef result;

    if (!time_reached(now_ms, start_due_ms))
    {
        return HAL_BUSY;
    }
    switch (up_zero_step)
    {
    case UP_ZERO_DISABLE_RS:
        if (start_motor_index >= RS_COUNT)
        {
            set_zero_step(UP_ZERO_WAIT_DISABLED, now_ms);
            break;
        }
        result = RsApp_Enable(&rs_app,
                              rs_motor_config[start_motor_index].id, false);
        if (result == HAL_OK)
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
        if (rs_pair_disabled(now_ms))
        {
            set_zero_step(UP_ZERO_CLEAR_RS_FAULT, now_ms);
        }
        break;

    case UP_ZERO_CLEAR_RS_FAULT:
        if (start_motor_index >= RS_COUNT)
        {
            set_zero_step(UP_ZERO_SET_RS, now_ms);
            break;
        }
        result = RsApp_ClearFault(&rs_app,
                                  rs_motor_config[start_motor_index].id);
        if (result == HAL_OK)
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
        if (start_motor_index >= RS_COUNT)
        {
            reset_coordinates();
            set_zero_step(UP_ZERO_ENABLE_RS, now_ms);
            break;
        }
        if (!start_cmd_sent)
        {
            result = RsApp_SetZero(&rs_app,
                                   rs_motor_config[start_motor_index].id);
            if (result == HAL_OK)
            {
                start_cmd_sent = true;
            }
            else if (result != HAL_BUSY)
            {
                retry_zero_step(now_ms);
            }
            break;
        }
        result = RsApp_GetZeroStatus(
            &rs_app, rs_motor_config[start_motor_index].id);
        if (result == HAL_OK)
        {
            start_motor_index++;
            start_cmd_sent = false;
            start_due_ms = now_ms + MOTOR_START_DELAY_MS;
        }
        else if (result == HAL_TIMEOUT)
        {
            /* Continue after the documented zero command timeout; the
               motor can still be enabled and controlled without feedback. */
            start_motor_index++;
            start_cmd_sent = false;
            start_due_ms = now_ms + MOTOR_START_DELAY_MS;
        }
        else if (result != HAL_BUSY)
        {
            retry_zero_step(now_ms);
        }
        break;

    case UP_ZERO_ENABLE_RS:
        if (start_motor_index < RS_COUNT)
        {
            if (start_rs_motor(start_motor_index) != HAL_OK)
            {
                retry_zero_step(now_ms);
                break;
            }
            start_motor_index++;
            start_due_ms = now_ms + MOTOR_START_DELAY_MS;
        }
        else
        {
            set_zero_step(UP_ZERO_WAIT_READY, now_ms);
        }
        break;

    case UP_ZERO_WAIT_READY:
        if (rs_pair_ready(now_ms))
        {
            return HAL_OK;
        }
        break;

    default:
        return HAL_ERROR;
    }
    return HAL_BUSY;
}

static void stop_outputs(void)
{
    if (!app_ready)
    {
        return;
    }
    up_curve_running = false;
    rebase_state = REBASE_IDLE;
    RsApp_StopAll(&rs_app);
    RsApp_Run(&rs_app, HAL_GetTick());
}

static void fail_motor_state(void)
{
    up_last_result = HAL_TIMEOUT;
    stop_outputs();
    up_state = UP_STATE_ERROR;
}

static void update_motor_curve(uint32_t now_ms)
{
    uint32_t elapsed_ms;
    float progress;
    float start_left;
    float start_right;

    if (!up_curve_running)
    {
        return;
    }
    elapsed_ms = now_ms - curve_start_ms;
    progress = (elapsed_ms >= MOTOR_MOVE_TIME_MS) ? 1.0f :
               (float)elapsed_ms / (float)MOTOR_MOVE_TIME_MS;
    start_left = curve_start_left_deg;
    start_right = curve_start_right_deg;
    up_command_angles.rs_l_deg = curve_value(start_left,
                                             up_target_angles.rs_l_deg,
                                             progress);
    up_command_angles.rs_r_deg = curve_value(start_right,
                                             up_target_angles.rs_r_deg,
                                             progress);
    up_last_result = set_motor_angles(&up_command_angles);
    if (up_last_result != HAL_OK)
    {
        stop_outputs();
        up_state = UP_STATE_ERROR;
    }
    else if (progress >= 1.0f)
    {
        up_curve_running = false;
    }
}

static HAL_StatusTypeDef start_motor_curve(float left_deg, float right_deg)
{
    if (!Up_IsReady() || (rebase_state != REBASE_IDLE) ||
        !finite_float(left_deg) || !finite_float(right_deg))
    {
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }
    up_target_angles.rs_l_deg = left_deg;
    up_target_angles.rs_r_deg = right_deg;
    curve_start_left_deg = up_command_angles.rs_l_deg;
    curve_start_right_deg = up_command_angles.rs_r_deg;
    curve_start_ms = HAL_GetTick();
    up_curve_running = true;
    up_last_result = HAL_OK;
    return HAL_OK;
}

static void run_motor_state(uint32_t now_ms)
{
    HAL_StatusTypeDef result;

    if ((up_state == UP_STATE_READY) || (up_state == UP_STATE_STOPPED) ||
        (up_state == UP_STATE_ERROR))
    {
        return;
    }
    switch (up_state)
    {
    case UP_STATE_FIRST_ZERO:
        result = run_zero_sequence(now_ms);
        if (result == HAL_OK)
        {
            up_target_angles.rs_l_deg = MOTOR_WORK_ZERO_DEG;
            up_target_angles.rs_r_deg = MOTOR_WORK_ZERO_DEG;
            curve_start_left_deg = up_command_angles.rs_l_deg;
            curve_start_right_deg = up_command_angles.rs_r_deg;
            curve_start_ms = now_ms;
            up_curve_running = true;
            up_state = UP_STATE_MOVE_SECOND_ZERO;
            state_start_ms = now_ms;
        }
        else if (result != HAL_BUSY)
        {
            fail_motor_state();
        }
        break;

    case UP_STATE_MOVE_SECOND_ZERO:
        update_motor_curve(now_ms);
        if (!up_curve_running &&
            rs_pair_at_target(now_ms, MOTOR_WORK_ZERO_DEG,
                              MOTOR_WORK_ZERO_DEG))
        {
            set_zero_step(UP_ZERO_DISABLE_RS, now_ms);
            up_state = UP_STATE_SECOND_ZERO;
            state_start_ms = now_ms;
        }
        break;

    case UP_STATE_SECOND_ZERO:
        result = run_zero_sequence(now_ms);
        if (result == HAL_OK)
        {
            up_state = UP_STATE_READY;
            up_last_result = HAL_OK;
        }
        else if (result != HAL_BUSY)
        {
            fail_motor_state();
        }
        break;

    default:
        fail_motor_state();
        break;
    }
    if ((up_state != UP_STATE_READY) && (up_state != UP_STATE_ERROR) &&
        ((uint32_t)(now_ms - state_start_ms) > MOTOR_START_TIMEOUT_MS))
    {
        fail_motor_state();
    }
}

static void update_rebase(uint32_t now_ms)
{
    rs_app_status_t left;
    rs_app_status_t right;
    HAL_StatusTypeDef result;

    if (rebase_state == REBASE_IDLE)
    {
        return;
    }
    if ((uint32_t)(now_ms - rebase_start_ms) > MOTOR_REBASE_TIMEOUT_MS)
    {
        up_last_result = HAL_TIMEOUT;
        rebase_state = REBASE_IDLE;
        return;
    }
    switch (rebase_state)
    {
    case REBASE_WAIT_DISABLED:
        if (!rs_pair_status(now_ms, &left, &right))
        {
            up_last_result = HAL_ERROR;
            rebase_state = REBASE_IDLE;
            return;
        }
        if (!left.active && !right.active &&
            time_reached(now_ms, rebase_due_ms))
        {
            start_motor_index = 0U;
            start_cmd_sent = false;
            rebase_state = REBASE_WAIT_ZERO;
        }
        break;

    case REBASE_WAIT_ZERO:
        if (start_motor_index >= RS_COUNT)
        {
            reset_coordinates();
            start_motor_index = 0U;
            rebase_state = REBASE_WAIT_READY;
            break;
        }
        if (!start_cmd_sent)
        {
            result = RsApp_SetZero(&rs_app,
                                   rs_motor_config[start_motor_index].id);
            if (result == HAL_OK)
            {
                start_cmd_sent = true;
            }
            break;
        }
        result = RsApp_GetZeroStatus(
            &rs_app, rs_motor_config[start_motor_index].id);
        if ((result == HAL_OK) || (result == HAL_TIMEOUT))
        {
            start_motor_index++;
            start_cmd_sent = false;
        }
        break;

    case REBASE_WAIT_READY:
        if (start_motor_index < RS_COUNT)
        {
            if (start_rs_motor(start_motor_index) == HAL_OK)
            {
                start_motor_index++;
            }
        }
        else if (rs_pair_ready(now_ms))
        {
            rebase_state = REBASE_IDLE;
            up_last_result = HAL_OK;
        }
        break;

    default:
        rebase_state = REBASE_IDLE;
        break;
    }
}

HAL_StatusTypeDef Up_Init(void)
{
    uint32_t now_ms;

    if (app_ready)
    {
        return HAL_OK;
    }
    if (Fdcan_Init(&fdcan_config) != HAL_OK ||
        RsApp_Init(&rs_app, Fdcan_GetRsBus(), rs_motor_config,
                   RS_COUNT) != HAL_OK)
    {
        Fdcan_Stop();
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }
    now_ms = HAL_GetTick();
    app_ready = true;
    bus_off_handled = false;
    reset_coordinates();
    rebase_state = REBASE_IDLE;
    up_state = UP_STATE_FIRST_ZERO;
    state_start_ms = now_ms;
    set_zero_step(UP_ZERO_DISABLE_RS, now_ms);
    up_last_result = HAL_OK;
    return HAL_OK;
}

HAL_StatusTypeDef Up_HomeMotors(void)
{
    uint32_t now_ms;

    if (!app_ready || Fdcan_BusOff())
    {
        return HAL_ERROR;
    }
    stop_outputs();
    reset_coordinates();
    now_ms = HAL_GetTick();
    up_state = UP_STATE_FIRST_ZERO;
    state_start_ms = now_ms;
    set_zero_step(UP_ZERO_DISABLE_RS, now_ms);
    up_last_result = HAL_BUSY;
    return HAL_OK;
}

bool Up_IsReady(void)
{
    return app_ready && !Fdcan_BusOff() && (up_state == UP_STATE_READY);
}

HAL_StatusTypeDef Up_SetRsPos(float left_deg, float right_deg)
{
    return start_motor_curve(left_deg, right_deg);
}

bool Up_GetMotorAngles(up_motor_angles_t *angles)
{
    rs_app_status_t left;
    rs_app_status_t right;
    uint32_t now_ms = HAL_GetTick();

    if ((angles == NULL) || !rs_pair_status(now_ms, &left, &right))
    {
        return false;
    }
    angles->rs_l_deg = left.feedback.angle_deg;
    angles->rs_r_deg = right.feedback.angle_deg;
    return left.online && right.online && left.active && right.active &&
           (left.feedback.fault == 0U) && (right.feedback.fault == 0U);
}

bool Up_RsMoveDone(void)
{
    return Up_IsReady() && !up_curve_running &&
           (rebase_state == REBASE_IDLE) &&
           rs_pair_at_target(HAL_GetTick(), up_target_angles.rs_l_deg,
                             up_target_angles.rs_r_deg);
}

bool Up_MotorMoveDone(void)
{
    return Up_RsMoveDone();
}

HAL_StatusTypeDef Up_RebaseRsPosition(void)
{
    if (!Up_IsReady() || up_curve_running || (rebase_state != REBASE_IDLE) ||
        (RsApp_Enable(&rs_app, RS_MOTOR_L_ID, false) != HAL_OK) ||
        (RsApp_Enable(&rs_app, RS_MOTOR_R_ID, false) != HAL_OK))
    {
        up_last_result = HAL_ERROR;
        return HAL_ERROR;
    }
    rebase_start_ms = HAL_GetTick();
    rebase_due_ms = rebase_start_ms + MOTOR_REBASE_DELAY_MS;
    rebase_state = REBASE_WAIT_DISABLED;
    up_last_result = HAL_OK;
    return HAL_OK;
}

HAL_StatusTypeDef Up_GetPositionRebaseStatus(void)
{
    if (!Up_IsReady())
    {
        return HAL_ERROR;
    }
    return (rebase_state == REBASE_IDLE) ? up_last_result : HAL_BUSY;
}

void Up_Run1ms(void)
{
    uint32_t now_ms;

    if (!app_ready)
    {
        return;
    }
    now_ms = HAL_GetTick();
    Fdcan_Run1ms();
    if (Fdcan_BusOff())
    {
        if (!bus_off_handled)
        {
            up_last_result = HAL_ERROR;
            bus_off_handled = true;
        }
        return;
    }
    bus_off_handled = false;
    RsApp_Run(&rs_app, now_ms);
    if (up_state != UP_STATE_READY)
    {
        run_motor_state(now_ms);
        return;
    }
    update_rebase(now_ms);
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
