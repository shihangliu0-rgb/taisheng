#include "rs_app.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

#define RS_APP_ID_MAX      0x7FU
#define RS_APP_RAD_PER_DEG 0.01745329251994329577f

static bool time_reached(uint32_t now_ms, uint32_t due_ms)
{
    return (int32_t)(now_ms - due_ms) >= 0;
}

static bool finite_float(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static bool mode_valid(rs_mode_t mode)
{
    return (mode == RS_MOTION) || (mode == RS_IQ) || (mode == RS_SPD) ||
           (mode == RS_CSP) || (mode == RS_PP);
}

static bool command_valid(const rs_command_t *command)
{
    if ((command == NULL) || !mode_valid(command->mode))
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

static HAL_StatusTypeDef send_control(rs_app_motor_t *motor)
{
    const rs_command_t *command = &motor->command;

    switch (command->mode)
    {
    case RS_MOTION:
        return RsMotor_SetMotion(
            &motor->motor,
            command->data.motion.angle_deg * RS_APP_RAD_PER_DEG,
            command->data.motion.speed_rad_s,
            command->data.motion.torque_nm,
            command->data.motion.kp, command->data.motion.kd);

    case RS_IQ:
        return RsMotor_SetIq(&motor->motor, command->data.iq.iq);

    case RS_SPD:
        return RsMotor_SetSpeed(&motor->motor,
                                command->data.speed.speed_rad_s,
                                command->data.speed.max_iq);

    case RS_CSP:
        return RsMotor_SetCsp(
            &motor->motor,
            command->data.csp.angle_deg * RS_APP_RAD_PER_DEG,
            command->data.csp.max_speed_rad_s);

    case RS_PP:
        return RsMotor_SetPp(
            &motor->motor,
            command->data.pp.angle_deg * RS_APP_RAD_PER_DEG,
            command->data.pp.max_speed_rad_s,
            command->data.pp.acceleration_rad_s2);

    default:
        return HAL_ERROR;
    }
}

static void handle_rx(void *context, uint32_t id, const uint8_t data[8],
                      uint32_t tick_ms)
{
    rs_app_t *app = context;
    uint8_t i;

    if ((app == NULL) || (data == NULL))
    {
        return;
    }

    for (i = 0U; i < app->motor_count; i++)
    {
        rs_app_motor_t *motor = &app->motors[i];
        uint32_t previous_sequence = motor->motor.feedback.sequence;

        RsMotor_Parse(&motor->motor, id, data, tick_ms);
        if ((motor->motor.feedback.sequence != previous_sequence) &&
            ((motor->motor.feedback.valid & RS_FDB_FAULT) != 0U) &&
            (motor->motor.feedback.fault != 0U))
        {
            motor->enable_requested = false;
            break;
        }
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
        if ((config[i].id > RS_APP_ID_MAX) ||
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

        if (!motor->enable_requested)
        {
            if (motor->motor.active || (motor->motor.start_step != 0U))
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
            (motor->motor.start_step != 0U))
        {
            motor->last_result =
                RsMotor_Start(&motor->motor, motor->command.mode, now_ms);
            if (motor->last_result != HAL_OK)
            {
                continue;
            }
        }

        motor->last_result = send_control(motor);
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

    if (!command_valid(command))
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
    rs_app_motor_t *motor = find_motor(app, id);

    if (motor == NULL)
    {
        return HAL_ERROR;
    }
    motor->enable_requested = enabled;
    motor->next_control_ms = 0U;
    return HAL_OK;
}

HAL_StatusTypeDef RsApp_Restart(rs_app_t *app, uint8_t id)
{
    rs_app_motor_t *motor = find_motor(app, id);

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    motor->enable_requested = true;
    motor->next_control_ms = 0U;
    motor->motor.active = true;
    motor->motor.start_step = 0U;
    motor->motor.mode = UINT8_MAX;
    return HAL_OK;
}

HAL_StatusTypeDef RsApp_SetZero(rs_app_t *app, uint8_t id)
{
    rs_app_motor_t *motor = find_motor(app, id);

    if ((motor == NULL) || motor->enable_requested)
    {
        return HAL_ERROR;
    }
    motor->last_result = RsMotor_SetZero(&motor->motor);
    return motor->last_result;
}

HAL_StatusTypeDef RsApp_ClearFault(rs_app_t *app, uint8_t id)
{
    rs_app_motor_t *motor = find_motor(app, id);

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
        driver_feedback.position_rad / RS_APP_RAD_PER_DEG;
    status->feedback.speed_rad_s = driver_feedback.velocity_rad_s;
    status->feedback.torque_nm = driver_feedback.torque_nm;
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
