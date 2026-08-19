#include "auto_chassis.h"

#include "chassis_main.h"
#include "sc_link.h"

#include <math.h>
#include <stdbool.h>

#define AUTO_CONTROL_PERIOD_MS       10U
#define AUTO_COMMAND_TIMEOUT_MS      50U
#define AUTO_INPUT_TIMEOUT_MS        250U

#define AUTO_FIELD_LINEAR_KP         150.0f
#define AUTO_ALIGN_LINEAR_KP         300.0f
#define AUTO_YAW_KP                  8.0f
#define AUTO_FIELD_MAX_CMD           150.0f
#define AUTO_ALIGN_MAX_CMD           75.0f
#define AUTO_YAW_MAX_CMD             10.0f
#define AUTO_YAW_MIN_CMD             4.0f
#define AUTO_COORD_LIMIT_M           100.0f
#define AUTO_STOP_DISTANCE_LIMIT_M   10.0f

#define AUTO_POSITION_TOLERANCE_M    0.05f
#define AUTO_ALIGN_TOLERANCE_M       0.03f
#define AUTO_YAW_TOLERANCE_RAD       0.08f
#define AUTO_PI                      3.14159265358979323846f
#define AUTO_TWO_PI                  (2.0f * AUTO_PI)

typedef struct
{
    float field_x_m;
    float field_y_m;
    float field_yaw;
    float stop_distance_m;
} auto_chassis_target_t;

static auto_chassis_target_t auto_target;
static uint32_t auto_last_run_ms;

volatile auto_chassis_state_t auto_chassis_state = AUTO_CHASSIS_IDLE;
volatile auto_chassis_error_t auto_chassis_error = AUTO_CHASSIS_ERROR_NONE;

static float AutoChassis_Clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static int16_t AutoChassis_ToCommand(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value + 0.5f);
    }
    return (int16_t)(value - 0.5f);
}

static float AutoChassis_WrapAngle(float angle)
{
    angle = fmodf(angle + AUTO_PI, AUTO_TWO_PI);
    if (angle < 0.0f)
    {
        angle += AUTO_TWO_PI;
    }
    return angle - AUTO_PI;
}

static void AutoChassis_LimitVector(float *x, float *y, float limit)
{
    float magnitude = sqrtf((*x * *x) + (*y * *y));

    if (magnitude > limit)
    {
        *x *= limit / magnitude;
        *y *= limit / magnitude;
    }
}

static void AutoChassis_SetState(auto_chassis_state_t state,
                                 auto_chassis_error_t error)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    auto_chassis_state = state;
    auto_chassis_error = error;
    if (primask == 0U)
    {
        __enable_irq();
    }
    ScLink_SetStatus((uint8_t)state, (uint8_t)error);
}

static void AutoChassis_Fault(auto_chassis_error_t error)
{
    Chassis_ReleaseVelocity(CHASSIS_CMD_SOURCE_AUTONOMOUS);
    AutoChassis_SetState(AUTO_CHASSIS_FAULT, error);
}

static HAL_StatusTypeDef AutoChassis_Request(float right_cmd,
                                             float forward_cmd,
                                             float yaw_cmd)
{
    HAL_StatusTypeDef result;

    result = Chassis_RequestVelocity(
        CHASSIS_CMD_SOURCE_AUTONOMOUS,
        AutoChassis_ToCommand(right_cmd),
        AutoChassis_ToCommand(forward_cmd),
        AutoChassis_ToCommand(yaw_cmd),
        AUTO_COMMAND_TIMEOUT_MS);
    if ((result != HAL_OK) &&
        (Chassis_GetControlMode() == CHASSIS_CONTROL_AUTONOMOUS))
    {
        AutoChassis_Fault(AUTO_CHASSIS_ERROR_CHASSIS);
    }
    return result;
}

static void AutoChassis_RunField(uint32_t now_ms)
{
    sc_link_pose_t pose;
    auto_chassis_target_t target;
    uint32_t primask;
    float dx;
    float dy;
    float distance;
    float right_error;
    float forward_error;
    float right_cmd;
    float forward_cmd;
    float yaw_error;
    float yaw_cmd;

    if (!ScLink_GetPose(&pose) || !pose.valid ||
        ((now_ms - pose.received_ms) > AUTO_INPUT_TIMEOUT_MS) ||
        !isfinite(pose.field_x_m) || !isfinite(pose.field_y_m) ||
        !isfinite(pose.field_yaw) ||
        (fabsf(pose.field_x_m) > AUTO_COORD_LIMIT_M) ||
        (fabsf(pose.field_y_m) > AUTO_COORD_LIMIT_M))
    {
        AutoChassis_Fault(AUTO_CHASSIS_ERROR_POSE_INVALID);
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    target = auto_target;
    if (primask == 0U)
    {
        __enable_irq();
    }

    pose.field_yaw = AutoChassis_WrapAngle(pose.field_yaw);
    dx = target.field_x_m - pose.field_x_m;
    dy = target.field_y_m - pose.field_y_m;
    distance = sqrtf((dx * dx) + (dy * dy));
    yaw_error = AutoChassis_WrapAngle(target.field_yaw -
                                      pose.field_yaw);
    if ((distance <= AUTO_POSITION_TOLERANCE_M) &&
        (fabsf(yaw_error) <= AUTO_YAW_TOLERANCE_RAD))
    {
        Chassis_ReleaseVelocity(CHASSIS_CMD_SOURCE_AUTONOMOUS);
        AutoChassis_SetState(AUTO_CHASSIS_ARRIVED,
                             AUTO_CHASSIS_ERROR_NONE);
        return;
    }

    /* Convert global field error to X-right/Y-forward chassis commands. */
    forward_error = cosf(pose.field_yaw) * dx +
                    sinf(pose.field_yaw) * dy;
    right_error = sinf(pose.field_yaw) * dx -
                  cosf(pose.field_yaw) * dy;
    right_cmd = AUTO_FIELD_LINEAR_KP * right_error;
    forward_cmd = AUTO_FIELD_LINEAR_KP * forward_error;
    AutoChassis_LimitVector(&right_cmd, &forward_cmd,
                            AUTO_FIELD_MAX_CMD);

    yaw_cmd = AutoChassis_Clamp(AUTO_YAW_KP * yaw_error,
                                AUTO_YAW_MAX_CMD);
    if (fabsf(yaw_error) <= AUTO_YAW_TOLERANCE_RAD)
    {
        yaw_cmd = 0.0f;
    }
    else if (fabsf(yaw_cmd) < AUTO_YAW_MIN_CMD)
    {
        yaw_cmd = (yaw_cmd >= 0.0f) ? AUTO_YAW_MIN_CMD :
                                      -AUTO_YAW_MIN_CMD;
    }
    (void)AutoChassis_Request(right_cmd, forward_cmd, yaw_cmd);
}

static void AutoChassis_RunAlignment(uint32_t now_ms,
                                     auto_chassis_state_t state)
{
    sc_link_perception_t perception;
    auto_chassis_target_t target;
    uint32_t primask;
    float target_x;
    float target_y;
    float forward_error;
    float right_error;
    float right_cmd;
    float forward_cmd;
    bool valid;

    if (!ScLink_GetPerception(&perception) ||
        ((now_ms - perception.received_ms) > AUTO_INPUT_TIMEOUT_MS))
    {
        AutoChassis_Fault((state == AUTO_CHASSIS_ALIGN_BLOCK) ?
                          AUTO_CHASSIS_ERROR_BLOCK_INVALID :
                          AUTO_CHASSIS_ERROR_BALL_INVALID);
        return;
    }

    if (state == AUTO_CHASSIS_ALIGN_BLOCK)
    {
        valid = perception.block_valid;
        target_x = perception.block_x_m;
        target_y = perception.block_y_m;
    }
    else
    {
        valid = perception.ball_valid;
        target_x = perception.ball_x_m;
        target_y = perception.ball_y_m;
    }
    if (!valid || !isfinite(target_x) || !isfinite(target_y) ||
        (fabsf(target_x) > AUTO_COORD_LIMIT_M) ||
        (fabsf(target_y) > AUTO_COORD_LIMIT_M))
    {
        AutoChassis_Fault((state == AUTO_CHASSIS_ALIGN_BLOCK) ?
                          AUTO_CHASSIS_ERROR_BLOCK_INVALID :
                          AUTO_CHASSIS_ERROR_BALL_INVALID);
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    target = auto_target;
    if (primask == 0U)
    {
        __enable_irq();
    }

    forward_error = target_x - target.stop_distance_m;
    right_error = -target_y;
    if ((fabsf(forward_error) <= AUTO_ALIGN_TOLERANCE_M) &&
        (fabsf(right_error) <= AUTO_ALIGN_TOLERANCE_M))
    {
        Chassis_ReleaseVelocity(CHASSIS_CMD_SOURCE_AUTONOMOUS);
        AutoChassis_SetState(AUTO_CHASSIS_ARRIVED,
                             AUTO_CHASSIS_ERROR_NONE);
        return;
    }

    right_cmd = AUTO_ALIGN_LINEAR_KP * right_error;
    forward_cmd = AUTO_ALIGN_LINEAR_KP * forward_error;
    AutoChassis_LimitVector(&right_cmd, &forward_cmd,
                            AUTO_ALIGN_MAX_CMD);
    (void)AutoChassis_Request(right_cmd, forward_cmd, 0.0f);
}

void AutoChassis_Init(void)
{
    Chassis_SetControlMode(CHASSIS_CONTROL_MANUAL);
    Chassis_ReleaseVelocity(CHASSIS_CMD_SOURCE_AUTONOMOUS);
    auto_last_run_ms = HAL_GetTick();
    AutoChassis_SetState(AUTO_CHASSIS_MANUAL, AUTO_CHASSIS_ERROR_NONE);
}

void AutoChassis_Run(void)
{
    uint32_t now_ms = HAL_GetTick();
    auto_chassis_state_t state;

    if ((now_ms - auto_last_run_ms) < AUTO_CONTROL_PERIOD_MS)
    {
        return;
    }
    auto_last_run_ms = now_ms;

    if (Chassis_GetControlMode() == CHASSIS_CONTROL_MANUAL)
    {
        Chassis_ReleaseVelocity(CHASSIS_CMD_SOURCE_AUTONOMOUS);
        if (auto_chassis_state != AUTO_CHASSIS_MANUAL)
        {
            AutoChassis_SetState(AUTO_CHASSIS_MANUAL,
                                 AUTO_CHASSIS_ERROR_NONE);
        }
        return;
    }

    state = auto_chassis_state;
    if (state == AUTO_CHASSIS_NAVIGATING)
    {
        AutoChassis_RunField(now_ms);
    }
    else if ((state == AUTO_CHASSIS_ALIGN_BLOCK) ||
             (state == AUTO_CHASSIS_ALIGN_BALL))
    {
        AutoChassis_RunAlignment(now_ms, state);
    }
    else if (state != AUTO_CHASSIS_MANUAL)
    {
        Chassis_ReleaseVelocity(CHASSIS_CMD_SOURCE_AUTONOMOUS);
    }
}

HAL_StatusTypeDef AutoChassis_SetFieldTarget(float field_x_m,
                                             float field_y_m,
                                             float field_yaw)
{
    uint32_t primask;

    if (!isfinite(field_x_m) || !isfinite(field_y_m) ||
        !isfinite(field_yaw) ||
        (fabsf(field_x_m) > AUTO_COORD_LIMIT_M) ||
        (fabsf(field_y_m) > AUTO_COORD_LIMIT_M))
    {
        AutoChassis_SetState(AUTO_CHASSIS_FAULT,
                             AUTO_CHASSIS_ERROR_BAD_TARGET);
        return HAL_ERROR;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    auto_target.field_x_m = field_x_m;
    auto_target.field_y_m = field_y_m;
    auto_target.field_yaw = AutoChassis_WrapAngle(field_yaw);
    if (primask == 0U)
    {
        __enable_irq();
    }
    Chassis_SetControlMode(CHASSIS_CONTROL_AUTONOMOUS);
    AutoChassis_SetState(AUTO_CHASSIS_NAVIGATING,
                         AUTO_CHASSIS_ERROR_NONE);
    return HAL_OK;
}

static HAL_StatusTypeDef AutoChassis_SetAlignment(
    auto_chassis_state_t state, float stop_distance_m)
{
    uint32_t primask;

    if (!isfinite(stop_distance_m) || (stop_distance_m < 0.0f) ||
        (stop_distance_m > AUTO_STOP_DISTANCE_LIMIT_M))
    {
        AutoChassis_SetState(AUTO_CHASSIS_FAULT,
                             AUTO_CHASSIS_ERROR_BAD_TARGET);
        return HAL_ERROR;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    auto_target.stop_distance_m = stop_distance_m;
    if (primask == 0U)
    {
        __enable_irq();
    }
    Chassis_SetControlMode(CHASSIS_CONTROL_AUTONOMOUS);
    AutoChassis_SetState(state, AUTO_CHASSIS_ERROR_NONE);
    return HAL_OK;
}

HAL_StatusTypeDef AutoChassis_AlignBlock(float stop_distance_m)
{
    return AutoChassis_SetAlignment(AUTO_CHASSIS_ALIGN_BLOCK,
                                    stop_distance_m);
}

HAL_StatusTypeDef AutoChassis_AlignBall(float stop_distance_m)
{
    return AutoChassis_SetAlignment(AUTO_CHASSIS_ALIGN_BALL,
                                    stop_distance_m);
}

void AutoChassis_Stop(void)
{
    Chassis_ReleaseVelocity(CHASSIS_CMD_SOURCE_AUTONOMOUS);
    Chassis_SetControlMode(CHASSIS_CONTROL_MANUAL);
    AutoChassis_SetState(AUTO_CHASSIS_MANUAL, AUTO_CHASSIS_ERROR_NONE);
}

auto_chassis_state_t AutoChassis_GetState(void)
{
    return auto_chassis_state;
}
