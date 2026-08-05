#include "dm_app.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

#define DM_APP_RAD_PER_DEG 0.01745329251994329577f

static void handle_rx(void *context, uint16_t id, const uint8_t data[8],
                      uint32_t tick_ms);

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

static dm_mit_cmd_t command_to_driver(const dm_app_mit_command_t *command)
{
    dm_mit_cmd_t driver_command;

    driver_command.position_rad = command->angle_deg * DM_APP_RAD_PER_DEG;
    driver_command.velocity_rad_s = command->speed_rad_s;
    driver_command.torque_nm = command->torque_nm;
    driver_command.kp = command->kp;
    driver_command.kd = command->kd;
    return driver_command;
}

static void feedback_from_driver(const dm_state_t *state,
                                 dm_app_feedback_t *feedback)
{
    feedback->angle_deg = state->position_rad / DM_APP_RAD_PER_DEG;
    feedback->speed_rad_s = state->velocity_rad_s;
    feedback->torque_nm = state->torque_nm;
    feedback->mos_temp_c = state->mos_temp_c;
    feedback->rotor_temp_c = state->rotor_temp_c;
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

static dm_result_t send_mode(dm_app_t *app, dm_app_motor_t *motor)
{
    dm_frame_t frame;
    dm_result_t result;

    result = motor->enable_requested
                 ? DmMotor_BuildEnable(&motor->motor, &frame)
                 : DmMotor_BuildDisable(&motor->motor, &frame);
    if (result == DM_OK)
    {
        result = send_frame(app, &frame);
    }
    if (result == DM_OK)
    {
        motor->active = motor->enable_requested;
    }
    return result;
}

static dm_result_t send_control(dm_app_t *app, dm_app_motor_t *motor)
{
    dm_frame_t frame;
    dm_mit_cmd_t driver_command = command_to_driver(&motor->command);
    dm_result_t result;

    result = DmMotor_BuildMit(&motor->motor, &driver_command, &frame);
    if (result == DM_OK)
    {
        result = send_frame(app, &frame);
    }
    return result;
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
        if (!command_valid(&config[i].command))
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
    }

    if (StdCan_AddHandler(bus, handle_rx, app) != HAL_OK)
    {
        return DM_IO_ERROR;
    }
    app->ready = true;
    return DM_OK;
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

        if (DmMotor_Parse(&motor->motor, id, data, 8U, tick_ms))
        {
            dm_state_t state;

            if (DmMotor_GetState(&motor->motor, &state) &&
                (state.fault != DM_FAULT_NONE))
            {
                motor->enable_requested = false;
            }
            break;
        }
    }
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

        if (motor->active != motor->enable_requested)
        {
            motor->last_result = send_mode(app, motor);
            continue;
        }
        if (!motor->active || !time_reached(now_ms, motor->next_control_ms))
        {
            continue;
        }

        motor->last_result = send_control(app, motor);
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

    driver_command = command_to_driver(command);
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
    dm_app_motor_t *motor = find_motor(app, id);

    if ((app == NULL) || !app->ready || (motor == NULL))
    {
        return DM_BAD_ARG;
    }
    motor->enable_requested = enabled;
    motor->next_control_ms = 0U;
    return DM_OK;
}

dm_result_t DmApp_Restart(dm_app_t *app, uint16_t id)
{
    dm_app_motor_t *motor = find_motor(app, id);

    if ((app == NULL) || !app->ready || (motor == NULL))
    {
        return DM_BAD_ARG;
    }

    motor->enable_requested = true;
    motor->active = false;
    motor->next_control_ms = 0U;
    return DM_OK;
}

dm_result_t DmApp_SetZero(dm_app_t *app, uint16_t id)
{
    dm_app_motor_t *motor = find_motor(app, id);
    dm_frame_t frame;
    dm_result_t result;

    if ((app == NULL) || !app->ready || (motor == NULL))
    {
        return DM_BAD_ARG;
    }
    if (!motor->active)
    {
        return DM_BUSY;
    }

    result = DmMotor_BuildZero(&motor->motor, &frame);
    if (result == DM_OK)
    {
        result = send_frame(app, &frame);
    }
    motor->last_result = result;
    return result;
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
        feedback_from_driver(&driver_feedback, &status->feedback);
    }
    status->online = status->has_feedback &&
                     ((uint32_t)(now_ms - status->feedback.rx_tick_ms) <=
                      DM_APP_FEEDBACK_TIMEOUT_MS);
    status->enable_requested = motor->enable_requested;
    status->active = motor->active;
    status->last_result = motor->last_result;
    return true;
}
