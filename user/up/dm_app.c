#include "dm_app.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

#define DM_APP_ENABLE_INTERVAL_MS 100U
#define DM_APP_ZERO_TIMEOUT_MS    200U
#define DM_APP_ZERO_ERROR_RAD     0.05f

#define DM_APP_RAD_PER_DEG 0.01745329251994329577f

static bool time_reached(uint32_t now_ms, uint32_t due_ms)
{
    return (int32_t)(now_ms - due_ms) >= 0;
}

static bool finite_float(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static bool command_valid(const dm_app_mit_command_t *command)
{
    return (command != NULL) && finite_float(command->angle_deg) &&
           finite_float(command->speed_rad_s) &&
           finite_float(command->torque_nm) && finite_float(command->kp) &&
           finite_float(command->kd);
}

static dm_mit_cmd_t command_to_driver(const dm_app_mit_command_t *command,
                                      int8_t direction)
{
    dm_mit_cmd_t driver_command;

    driver_command.position_rad = command->angle_deg * (float)direction *
                                  DM_APP_RAD_PER_DEG *
                                  DM_APP_OUTPUT_TO_MOTOR_RATIO;
    driver_command.velocity_rad_s = command->speed_rad_s * (float)direction *
                                    DM_APP_OUTPUT_TO_MOTOR_RATIO;
    driver_command.torque_nm = command->torque_nm * (float)direction *
                               DM_APP_MOTOR_TO_OUTPUT_RATIO;
    driver_command.kp = command->kp;
    driver_command.kd = command->kd;
    return driver_command;
}

static void feedback_from_driver(const dm_state_t *state,
                                 dm_app_feedback_t *feedback,
                                 int8_t direction)
{
    feedback->angle_deg = state->position_rad / DM_APP_RAD_PER_DEG *
                          DM_APP_MOTOR_TO_OUTPUT_RATIO * (float)direction;
    feedback->speed_rad_s = state->velocity_rad_s *
                            DM_APP_MOTOR_TO_OUTPUT_RATIO * (float)direction;
    feedback->torque_nm = state->torque_nm * DM_APP_OUTPUT_TO_MOTOR_RATIO *
                          (float)direction;
    feedback->mos_temp_c = state->mos_temp_c;
    feedback->rotor_temp_c = state->rotor_temp_c;
    feedback->operating_state = state->operating_state;
    feedback->fault = state->fault;
    feedback->rx_tick_ms = state->rx_tick_ms;
    feedback->rx_count = state->rx_count;
}

static dm_app_motor_t *find_motor(dm_app_t *app, uint16_t id)
{
    uint8_t i;

    if (app == NULL)
    {
        return NULL;
    }
    for (i = 0U; i < app->motor_count; i++)
    {
        if (app->motors[i].motor.tx_id == id)
        {
            return &app->motors[i];
        }
    }
    return NULL;
}

static const dm_app_motor_t *find_const_motor(const dm_app_t *app, uint16_t id)
{
    uint8_t i;

    if (app == NULL)
    {
        return NULL;
    }
    for (i = 0U; i < app->motor_count; i++)
    {
        if (app->motors[i].motor.tx_id == id)
        {
            return &app->motors[i];
        }
    }
    return NULL;
}

static dm_result_t send_frame(dm_app_t *app, const dm_frame_t *frame)
{
    HAL_StatusTypeDef status =
        StdCan_Send(app->bus, frame->id, frame->data);

    if (status == HAL_OK)
    {
        return DM_OK;
    }
    return (status == HAL_BUSY) ? DM_BUSY : DM_IO_ERROR;
}

static void handle_rx(void *context, uint16_t id, const uint8_t data[8],
                      uint32_t tick_ms)
{
    dm_app_t *app = context;
    uint8_t i;

    if ((app == NULL) || !app->ready || (data == NULL))
    {
        return;
    }

    for (i = 0U; i < app->motor_count; i++)
    {
        dm_app_motor_t *motor = &app->motors[i];
        uint32_t previous_count = motor->motor.state.rx_count;

        if (DmMotor_Parse(&motor->motor, id, data, 8U, tick_ms))
        {
            motor->active =
                motor->motor.state.operating_state == DM_STATE_ENABLED;
            if (motor->motor.state.fault != DM_FAULT_NONE)
            {
                motor->last_result = DM_IO_ERROR;
            }
            if (motor->zero_pending &&
                (motor->motor.state.rx_count != previous_count) &&
                (motor->motor.state.rx_count != motor->zero_rx_count) &&
                !motor->active &&
                (motor->motor.state.fault == DM_FAULT_NONE) &&
                (motor->motor.state.position_rad >=
                 -DM_APP_ZERO_ERROR_RAD) &&
                (motor->motor.state.position_rad <= DM_APP_ZERO_ERROR_RAD))
            {
                motor->zero_pending = false;
                motor->zero_completed = true;
            }
            break;
        }
    }
}

dm_result_t DmApp_Init(dm_app_t *app, std_can_t *bus,
                       const dm_app_motor_config_t *config, uint8_t count)
{
    uint8_t i;
    uint8_t j;

    if ((app == NULL) || (bus == NULL) || !bus->ready ||
        (config == NULL) || (count == 0U) || (count > DM_APP_MAX_MOTORS))
    {
        return DM_BAD_ARG;
    }

    for (i = 0U; i < count; i++)
    {
        if (((config[i].direction != 1) && (config[i].direction != -1)) ||
            !command_valid(&config[i].command))
        {
            return DM_BAD_ARG;
        }
        for (j = (uint8_t)(i + 1U); j < count; j++)
        {
            if (config[i].tx_id == config[j].tx_id)
            {
                return DM_BAD_ARG;
            }
        }
    }

    memset(app, 0, sizeof(*app));
    app->bus = bus;
    app->motor_count = count;

    for (i = 0U; i < count; i++)
    {
        dm_result_t result =
            DmMotor_Init(&app->motors[i].motor, config[i].tx_id,
                         config[i].master_id, config[i].feedback_id);
        if (result != DM_OK)
        {
            return result;
        }
        app->motors[i].command = config[i].command;
        app->motors[i].direction = config[i].direction;
    }

    if (StdCan_AddHandler(bus, handle_rx, app) != HAL_OK)
    {
        return DM_IO_ERROR;
    }
    app->ready = true;
    return DM_OK;
}

void DmApp_Run(dm_app_t *app, uint32_t now_ms)
{
    uint8_t i;

    if ((app == NULL) || !app->ready)
    {
        return;
    }

    for (i = 0U; i < app->motor_count; i++)
    {
        dm_app_motor_t *motor = &app->motors[i];
        dm_frame_t frame;
        dm_mit_cmd_t driver_command;
        dm_result_t result;

        if (!motor->enable_requested)
        {
            if (!motor->active && (motor->enable_attempts == 0U))
            {
                continue;
            }
            if (!time_reached(now_ms, motor->next_control_ms))
            {
                continue;
            }

            result = DmMotor_BuildDisable(&motor->motor, &frame);
            if (result == DM_OK)
            {
                result = send_frame(app, &frame);
            }
            motor->last_result = result;
            if (result == DM_OK)
            {
                motor->enable_attempts = 0U;
                motor->next_control_ms =
                    now_ms + DM_APP_ENABLE_INTERVAL_MS;
            }
            continue;
        }

        if (!motor->active)
        {
            if (!time_reached(now_ms, motor->next_control_ms))
            {
                continue;
            }

            result = DmMotor_BuildEnable(&motor->motor, &frame);
            if (result == DM_OK)
            {
                result = send_frame(app, &frame);
            }
            motor->last_result = result;
            if (result == DM_OK)
            {
                if (motor->enable_attempts < UINT8_MAX)
                {
                    motor->enable_attempts++;
                }
                motor->next_control_ms =
                    now_ms + DM_APP_ENABLE_INTERVAL_MS;
            }
            continue;
        }
        if (!motor->active || !time_reached(now_ms, motor->next_control_ms))
        {
            continue;
        }
        if (!motor->zero_completed)
        {
            continue;
        }
        driver_command =
            command_to_driver(&motor->command, motor->direction);
        result = DmMotor_BuildMit(&motor->motor, &driver_command, &frame);
        if (result == DM_OK)
        {
            result = send_frame(app, &frame);
        }
        motor->last_result = result;
        if (motor->last_result == DM_OK)
        {
            motor->next_control_ms = now_ms + DM_APP_CONTROL_PERIOD_MS;
        }
    }
}

dm_result_t DmApp_SetMitCmd(dm_app_t *app, uint16_t id,
                            const dm_app_mit_command_t *command)
{
    dm_app_motor_t *motor;
    dm_frame_t frame;
    dm_mit_cmd_t driver_command;
    dm_result_t result;

    if ((app == NULL) || !app->ready || !command_valid(command))
    {
        return DM_BAD_ARG;
    }
    motor = find_motor(app, id);
    if (motor == NULL)
    {
        return DM_BAD_ARG;
    }

    driver_command = command_to_driver(command, motor->direction);
    result = DmMotor_BuildMit(&motor->motor, &driver_command, &frame);
    if (result == DM_OK)
    {
        motor->command = *command;
        motor->next_control_ms = 0U;
    }
    return result;
}

dm_result_t DmApp_Enable(dm_app_t *app, uint16_t id, bool enabled)
{
    dm_app_motor_t *motor;
    dm_frame_t frame;
    dm_result_t result;

    if ((app == NULL) || !app->ready)
    {
        return DM_BAD_ARG;
    }
    motor = find_motor(app, id);
    if (motor == NULL)
    {
        return DM_BAD_ARG;
    }
    if (!enabled)
    {
        motor->enable_requested = false;
        result = DmMotor_BuildDisable(&motor->motor, &frame);
        if (result == DM_OK)
        {
            result = send_frame(app, &frame);
        }
        motor->last_result = result;
        if (result == DM_OK)
        {
            motor->enable_attempts = 0U;
            motor->next_control_ms =
                HAL_GetTick() + DM_APP_ENABLE_INTERVAL_MS;
        }
        return result;
    }

    motor->enable_requested = true;
    if (!motor->active)
    {
        motor->enable_attempts = 0U;
    }
    motor->next_control_ms = 0U;
    return DM_OK;
}

dm_result_t DmApp_Restart(dm_app_t *app, uint16_t id)
{
    dm_app_motor_t *motor;

    if ((app == NULL) || !app->ready)
    {
        return DM_BAD_ARG;
    }
    motor = find_motor(app, id);
    if (motor == NULL)
    {
        return DM_BAD_ARG;
    }

    motor->enable_requested = true;
    motor->enable_attempts = 0U;
    motor->next_control_ms = 0U;
    return DM_OK;
}

dm_result_t DmApp_SetZero(dm_app_t *app, uint16_t id)
{
    dm_app_motor_t *motor;
    dm_frame_t frame;
    dm_result_t result;

    if ((app == NULL) || !app->ready)
    {
        return DM_BAD_ARG;
    }
    motor = find_motor(app, id);
    if (motor == NULL)
    {
        return DM_BAD_ARG;
    }
    if (motor->active || motor->enable_requested || motor->zero_pending)
    {
        return DM_BUSY;
    }

    result = DmMotor_BuildZero(&motor->motor, &frame);
    if (result == DM_OK)
    {
        motor->command.angle_deg = 0.0f;
        motor->zero_rx_count = motor->motor.state.rx_count;
        motor->zero_sent_ms = HAL_GetTick();
        motor->zero_pending = true;
        motor->zero_completed = false;
        result = send_frame(app, &frame);
    }
    if (result != DM_OK)
    {
        motor->zero_pending = false;
    }
    motor->last_result = result;
    return result;
}

dm_result_t DmApp_GetZeroStatus(dm_app_t *app, uint16_t id)
{
    dm_app_motor_t *motor;

    if ((app == NULL) || !app->ready)
    {
        return DM_BAD_ARG;
    }
    motor = find_motor(app, id);
    if (motor == NULL)
    {
        return DM_BAD_ARG;
    }
    if (motor->zero_pending)
    {
        if ((uint32_t)(HAL_GetTick() - motor->zero_sent_ms) >=
            DM_APP_ZERO_TIMEOUT_MS)
        {
            motor->zero_pending = false;
            motor->last_result = DM_IO_ERROR;
            return DM_IO_ERROR;
        }
        return DM_BUSY;
    }
    return motor->zero_completed ? DM_OK : motor->last_result;
}

void DmApp_StopAll(dm_app_t *app)
{
    uint8_t i;

    if (app == NULL)
    {
        return;
    }
    for (i = 0U; i < app->motor_count; i++)
    {
        app->motors[i].enable_requested = false;
        app->motors[i].next_control_ms = 0U;
    }
}

bool DmApp_GetStatus(const dm_app_t *app, uint16_t id, uint32_t now_ms,
                     dm_app_status_t *status)
{
    const dm_app_motor_t *motor;
    dm_state_t driver_feedback;

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
    status->has_feedback = DmMotor_GetState(&motor->motor, &driver_feedback);
    if (status->has_feedback)
    {
        feedback_from_driver(&driver_feedback, &status->feedback,
                             motor->direction);
    }
    status->online = status->has_feedback &&
                     ((uint32_t)(now_ms - status->feedback.rx_tick_ms) <=
                      DM_APP_FEEDBACK_TIMEOUT_MS);
    status->enable_requested = motor->enable_requested;
    status->active = motor->active;
    status->last_result = motor->last_result;
    return true;
}
