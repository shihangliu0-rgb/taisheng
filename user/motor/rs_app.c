#include "rs_app.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

#define RS_APP_RAD_PER_DEG 0.01745329251994329577f
/* 5:9 减速齿轮，应用层量为输出轴量。 */
#define RS_APP_OUTPUT_TO_MOTOR_RATIO (9.0f / 5.0f)
#define RS_APP_MOTOR_TO_OUTPUT_RATIO (5.0f / 9.0f)

static bool time_reached(uint32_t now_ms, uint32_t due_ms)
{
    return (int32_t)(now_ms - due_ms) >= 0;
}

static bool finite_float(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static bool command_valid(const rs_command_t *command)
{
    if (command == NULL)
    {
        return false;
    }

    switch (command->mode)
    {
    case RS_MOTION:
        return finite_float(command->data.motion.angle_deg) &&
               finite_float(command->data.motion.speed_rad_s) &&
               finite_float(command->data.motion.torque_nm) &&
               finite_float(command->data.motion.kp) &&
               finite_float(command->data.motion.kd);

    case RS_IQ:
        return finite_float(command->data.iq.iq);

    case RS_SPD:
        return finite_float(command->data.speed.speed_rad_s) &&
               finite_float(command->data.speed.max_iq) &&
               (command->data.speed.max_iq >= 0.0f);

    case RS_CSP:
        return finite_float(command->data.csp.angle_deg) &&
               finite_float(command->data.csp.max_speed_rad_s) &&
               (command->data.csp.max_speed_rad_s >= 0.0f);

    case RS_PP:
        return finite_float(command->data.pp.angle_deg) &&
               finite_float(command->data.pp.max_speed_rad_s) &&
               finite_float(command->data.pp.acceleration_rad_s2) &&
               (command->data.pp.max_speed_rad_s >= 0.0f) &&
               (command->data.pp.acceleration_rad_s2 >= 0.0f);

    default:
        return false;
    }
}

static rs_app_motor_t *find_motor(rs_app_t *app, uint8_t id)
{
    uint8_t i;

    if (app == NULL)
    {
        return NULL;
    }
    for (i = 0U; i < app->motor_count; i++)
    {
        if (app->motors[i].motor.id == id)
        {
            return &app->motors[i];
        }
    }
    return NULL;
}

static const rs_app_motor_t *find_const_motor(const rs_app_t *app, uint8_t id)
{
    uint8_t i;

    if (app == NULL)
    {
        return NULL;
    }
    for (i = 0U; i < app->motor_count; i++)
    {
        if (app->motors[i].motor.id == id)
        {
            return &app->motors[i];
        }
    }
    return NULL;
}

static void handle_rx(void *context, uint32_t id, const uint8_t data[8],
                      uint32_t tick_ms)
{
    rs_app_t *app = context;
    rs_app_motor_t *motor;
    uint32_t previous_sequence;

    if ((app == NULL) || (data == NULL))
    {
        return;
    }

    motor = find_motor(app, (uint8_t)(id >> 8));
    if (motor == NULL)
    {
        return;
    }

    previous_sequence = motor->motor.feedback.sequence;
    RsMotor_Parse(&motor->motor, id, data, tick_ms);
    if (motor->motor.feedback.sequence == previous_sequence)
    {
        return;
    }

    if (((motor->motor.feedback.valid & RS_FDB_FAULT) != 0U) &&
        (motor->motor.feedback.fault != 0U))
    {
        /* 灵足故障时立即撤销使能请求，避免继续发送控制帧。 */
        motor->enable_requested = false;
        motor->last_result = HAL_ERROR;
    }
}

HAL_StatusTypeDef RsApp_Init(rs_app_t *app, rs_bus_t *bus,
                             const rs_app_motor_config_t *config,
                             uint8_t count)
{
    uint8_t i;
    uint8_t j;

    if ((app == NULL) || (bus == NULL) || !bus->ready ||
        (config == NULL) || (count == 0U) || (count > RS_APP_MAX_MOTORS))
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < count; i++)
    {
        if ((config[i].id > RS_MOTOR_ID_MAX) ||
            ((config[i].direction != 1) && (config[i].direction != -1)) ||
            !command_valid(&config[i].command))
        {
            return HAL_ERROR;
        }
        for (j = (uint8_t)(i + 1U); j < count; j++)
        {
            if (config[i].id == config[j].id)
            {
                return HAL_ERROR;
            }
        }
    }

    memset(app, 0, sizeof(*app));
    app->bus = bus;
    app->motor_count = count;

    for (i = 0U; i < count; i++)
    {
        rs_app_motor_t *motor = &app->motors[i];

        if (RsMotor_Init(&motor->motor, bus, config[i].id) != HAL_OK)
        {
            return HAL_ERROR;
        }
        motor->command = config[i].command;
        motor->direction = config[i].direction;
        motor->period_ms = (config[i].period_ms == 0U)
                               ? RS_APP_CONTROL_PERIOD_MS
                               : config[i].period_ms;
    }

    if (RsBus_SetHandler(bus, handle_rx, app) != HAL_OK)
    {
        return HAL_ERROR;
    }

    app->ready = true;
    return HAL_OK;
}

void RsApp_Run(rs_app_t *app, uint32_t now_ms)
{
    uint8_t i;

    if ((app == NULL) || !app->ready)
    {
        return;
    }

    for (i = 0U; i < app->motor_count; i++)
    {
        rs_app_motor_t *motor = &app->motors[i];
        const rs_command_t *command = &motor->command;
        float direction = (float)motor->direction;

        if (!motor->enable_requested)
        {
            if (motor->motor.active ||
                (motor->motor.start_step != RS_START_IDLE))
            {
                motor->last_result = RsMotor_Stop(&motor->motor);
            }
            continue;
        }
        if (!time_reached(now_ms, motor->next_control_ms))
        {
            continue;
        }
        if (!motor->motor.active ||
            (motor->motor.mode != (uint8_t)motor->command.mode) ||
            (motor->motor.start_step != RS_START_IDLE))
        {
            motor->last_result =
                RsMotor_Start(&motor->motor, motor->command.mode, now_ms);
            if (motor->last_result != HAL_OK)
            {
                continue;
            }
        }

        switch (command->mode)
        {
        case RS_MOTION:
            motor->last_result = RsMotor_SetMotion(
                &motor->motor,
                command->data.motion.angle_deg * direction *
                    RS_APP_RAD_PER_DEG * RS_APP_OUTPUT_TO_MOTOR_RATIO,
                command->data.motion.speed_rad_s * direction *
                    RS_APP_OUTPUT_TO_MOTOR_RATIO,
                command->data.motion.torque_nm * direction *
                    RS_APP_MOTOR_TO_OUTPUT_RATIO,
                command->data.motion.kp, command->data.motion.kd);
            break;

        case RS_IQ:
            motor->last_result = RsMotor_SetIq(
                &motor->motor, command->data.iq.iq * direction);
            break;

        case RS_SPD:
            motor->last_result = RsMotor_SetSpeed(
                &motor->motor,
                command->data.speed.speed_rad_s * direction *
                    RS_APP_OUTPUT_TO_MOTOR_RATIO,
                command->data.speed.max_iq);
            break;

        case RS_CSP:
            motor->last_result = RsMotor_SetCsp(
                &motor->motor,
                command->data.csp.angle_deg * direction *
                    RS_APP_RAD_PER_DEG * RS_APP_OUTPUT_TO_MOTOR_RATIO,
                command->data.csp.max_speed_rad_s *
                    RS_APP_OUTPUT_TO_MOTOR_RATIO);
            break;

        case RS_PP:
            motor->last_result = RsMotor_SetPp(
                &motor->motor,
                command->data.pp.angle_deg * direction *
                    RS_APP_RAD_PER_DEG * RS_APP_OUTPUT_TO_MOTOR_RATIO,
                command->data.pp.max_speed_rad_s *
                    RS_APP_OUTPUT_TO_MOTOR_RATIO,
                command->data.pp.acceleration_rad_s2 *
                    RS_APP_OUTPUT_TO_MOTOR_RATIO);
            break;

        default:
            motor->last_result = HAL_ERROR;
            break;
        }
        if (motor->last_result == HAL_OK)
        {
            motor->next_control_ms = now_ms + motor->period_ms;
        }
    }
}

HAL_StatusTypeDef RsApp_SetCmd(rs_app_t *app, uint8_t id,
                               const rs_command_t *command)
{
    rs_app_motor_t *motor;

    if ((app == NULL) || !app->ready || !command_valid(command))
    {
        return HAL_ERROR;
    }
    motor = find_motor(app, id);
    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    motor->command = *command;
    motor->next_control_ms = 0U;
    return HAL_OK;
}

HAL_StatusTypeDef RsApp_Enable(rs_app_t *app, uint8_t id, bool enabled)
{
    rs_app_motor_t *motor;

    if ((app == NULL) || !app->ready)
    {
        return HAL_ERROR;
    }
    motor = find_motor(app, id);
    if (motor == NULL)
    {
        return HAL_ERROR;
    }
    if (!enabled)
    {
        motor->enable_requested = false;
        motor->last_result = RsMotor_Stop(&motor->motor);
        return motor->last_result;
    }

    motor->enable_requested = true;
    motor->next_control_ms = 0U;
    return HAL_OK;
}

HAL_StatusTypeDef RsApp_Restart(rs_app_t *app, uint8_t id)
{
    rs_app_motor_t *motor;

    if ((app == NULL) || !app->ready)
    {
        return HAL_ERROR;
    }
    motor = find_motor(app, id);
    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    motor->enable_requested = true;
    motor->next_control_ms = 0U;
    /* 先发送 OFF，再重新写入模式，兼容电机掉电后模式复位。 */
    motor->motor.active = true;
    motor->motor.start_step = RS_START_IDLE;
    motor->motor.mode = UINT8_MAX;
    motor->motor.requested_mode = UINT8_MAX;
    return HAL_OK;
}

HAL_StatusTypeDef RsApp_SetZero(rs_app_t *app, uint8_t id)
{
    rs_app_motor_t *motor;

    if ((app == NULL) || !app->ready)
    {
        return HAL_ERROR;
    }
    motor = find_motor(app, id);
    if ((motor == NULL) || motor->enable_requested)
    {
        return HAL_ERROR;
    }
    motor->last_result = RsMotor_SetZero(&motor->motor);
    return motor->last_result;
}

HAL_StatusTypeDef RsApp_GetZeroStatus(rs_app_t *app, uint8_t id)
{
    rs_app_motor_t *motor;

    if ((app == NULL) || !app->ready)
    {
        return HAL_ERROR;
    }
    motor = find_motor(app, id);
    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    motor->last_result = RsMotor_GetZeroStatus(&motor->motor);
    return motor->last_result;
}

HAL_StatusTypeDef RsApp_ClearFault(rs_app_t *app, uint8_t id)
{
    rs_app_motor_t *motor;

    if ((app == NULL) || !app->ready)
    {
        return HAL_ERROR;
    }
    motor = find_motor(app, id);
    if (motor == NULL)
    {
        return HAL_ERROR;
    }
    motor->last_result = RsMotor_ClearFault(&motor->motor);
    return motor->last_result;
}

void RsApp_StopAll(rs_app_t *app)
{
    uint8_t i;

    if (app == NULL)
    {
        return;
    }
    for (i = 0U; i < app->motor_count; i++)
    {
        app->motors[i].enable_requested = false;
    }
}

bool RsApp_GetStatus(const rs_app_t *app, uint8_t id, uint32_t now_ms,
                     rs_app_status_t *status)
{
    const rs_app_motor_t *motor;
    rs_feedback_t driver_feedback;

    if ((app == NULL) || !app->ready || (status == NULL))
    {
        return false;
    }
    motor = find_const_motor(app, id);
    if (motor == NULL)
    {
        return false;
    }

    memset(status, 0, sizeof(*status));
    (void)RsMotor_GetFeedback(&motor->motor, &driver_feedback);
    status->feedback.angle_deg =
        driver_feedback.position_rad / RS_APP_RAD_PER_DEG *
        RS_APP_MOTOR_TO_OUTPUT_RATIO * (float)motor->direction;
    status->feedback.speed_rad_s = driver_feedback.velocity_rad_s *
                                   RS_APP_MOTOR_TO_OUTPUT_RATIO *
                                   (float)motor->direction;
    status->feedback.torque_nm = driver_feedback.torque_nm *
                                  RS_APP_OUTPUT_TO_MOTOR_RATIO *
                                  (float)motor->direction;
    status->feedback.temperature_c = driver_feedback.temperature_c;
    status->feedback.fault = driver_feedback.fault;
    status->feedback.warning = driver_feedback.warning;
    status->feedback.valid = driver_feedback.valid;
    status->feedback.sequence = driver_feedback.sequence;
    status->feedback.state = driver_feedback.state;
    status->has_feedback = driver_feedback.sequence != 0U;
    status->online = status->has_feedback &&
                     ((uint32_t)(now_ms - motor->motor.last_rx_ms) <=
                      RS_APP_FEEDBACK_TIMEOUT_MS);
    status->enable_requested = motor->enable_requested;
    status->active = motor->motor.active;
    status->last_result = motor->last_result;
    return true;
}
