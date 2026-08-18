#include "c610_2006.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

/* C610 帧 ID、编码器分辨率和反馈超时。 */
#define C610_FEEDBACK_TIMEOUT_MS 20U
#define C610_COMMAND_ID_1_TO_4   0x200U
#define C610_COMMAND_ID_5_TO_8   0x1FFU
#define C610_ENCODER_COUNTS      8192
#define C610_ENCODER_HALF        (C610_ENCODER_COUNTS / 2)

#define M2006_GEAR_RATIO            36.0f
#define M2006_CURRENT_LIMIT_A       3.0f
#define M2006_POSITION_DEADBAND_DEG 0.2f
#define M2006_PID_GAIN_MAX          10000.0f
#define M2006_DT_S                  (C610_CONTROL_PERIOD_MS / 1000.0f)

static float clamp_symmetric(float value, float limit)
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

static bool finite_float(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static bool pid_gains_valid(const m2006_pid_gains_t *gains)
{
    return (gains != NULL) && finite_float(gains->kp) &&
           finite_float(gains->ki) && finite_float(gains->kd) &&
           (gains->kp >= 0.0f) && (gains->ki >= 0.0f) &&
           (gains->kd >= 0.0f) && (gains->kp <= M2006_PID_GAIN_MAX) &&
           (gains->ki <= M2006_PID_GAIN_MAX) &&
           (gains->kd <= M2006_PID_GAIN_MAX);
}

static bool config_valid(const m2006_config_t *config)
{
    return (config != NULL) && (config->id >= 1U) &&
           (config->id <= C610_MAX_MOTORS) &&
           ((config->direction == 1) || (config->direction == -1)) &&
           finite_float(config->target_position_deg) &&
           pid_gains_valid(&config->pid);
}

static void pid_reset(m2006_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->started = false;
}

static float pid_run(m2006_pid_t *pid, float error)
{
    float derivative = 0.0f;
    float output;

    if (pid->started)
    {
        derivative = (error - pid->last_error) / M2006_DT_S;
    }
    else
    {
        pid->started = true;
    }

    pid->integral = clamp_symmetric(
        pid->integral + pid->gains.ki * error * M2006_DT_S,
        pid->integral_limit);
    pid->last_error = error;
    output = pid->gains.kp * error + pid->integral +
             pid->gains.kd * derivative;

    return clamp_symmetric(output, pid->output_limit);
}

static m2006_motor_t *find_motor(c610_bus_t *bus, uint8_t id)
{
    if ((bus == NULL) || (id < 1U) || (id > C610_MAX_MOTORS))
    {
        return NULL;
    }
    return bus->motors[id - 1U];
}

static const m2006_motor_t *find_const_motor(const c610_bus_t *bus,
                                             uint8_t id)
{
    if ((bus == NULL) || (id < 1U) || (id > C610_MAX_MOTORS))
    {
        return NULL;
    }
    return bus->motors[id - 1U];
}

static bool attach_motor(c610_bus_t *bus, m2006_motor_t *motor,
                         const m2006_config_t *config)
{
    uint8_t slot = config->id - 1U;
    float current_limit = M2006_CURRENT_LIMIT_A * 1000.0f;

    if (bus->motors[slot] != NULL)
    {
        return false;
    }

    motor->id = config->id;
    motor->direction = config->direction;
    motor->pid.gains = config->pid;
    motor->pid.integral_limit = current_limit;
    motor->pid.output_limit = current_limit;
    motor->target_position_deg = config->target_position_deg;
    motor->control_mode = M2006_CONTROL_COAST;

    bus->motors[slot] = motor;
    bus->group_used[slot / 4U] = true;
    return true;
}

static void update_feedback(m2006_motor_t *motor, const uint8_t data[8],
                            uint32_t now_ms)
{
    uint16_t angle = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);

    if (motor->feedback_seen)
    {
        int32_t delta = (int32_t)angle - motor->rotor_angle;

        // 对编码器回绕进行补偿，保留多圈累计位置。
        if (delta > C610_ENCODER_HALF)
        {
            delta -= C610_ENCODER_COUNTS;
        }
        else if (delta < -C610_ENCODER_HALF)
        {
            delta += C610_ENCODER_COUNTS;
        }
        motor->rotor_total += delta;
    }
    else
    {
        motor->rotor_total = 0;
        motor->feedback_seen = true;
    }

    motor->rotor_angle = angle;
    motor->rotor_rpm =
        (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    motor->current_feedback =
        (int16_t)((int16_t)(((uint16_t)data[4] << 8) | data[5]) *
                  motor->direction);
    motor->last_feedback_ms = now_ms;
}

static void handle_rx(void *context, uint16_t id, const uint8_t data[8],
                      uint32_t tick_ms)
{
    c610_bus_t *bus = context;
    m2006_motor_t *motor;

    if ((bus == NULL) || !bus->ready || (data == NULL) ||
        (id < C610_FEEDBACK_ID_BASE) ||
        (id >= C610_FEEDBACK_ID_BASE + C610_MAX_MOTORS))
    {
        return;
    }

    motor = bus->motors[id - C610_FEEDBACK_ID_BASE];
    if (motor != NULL)
    {
        update_feedback(motor, data, tick_ms);
    }
}

static void update_motor(m2006_motor_t *motor, uint32_t now_ms)
{
    float current = 0.0f;

    motor->output_speed_rpm = (float)motor->rotor_rpm /
                              M2006_GEAR_RATIO *
                              (float)motor->direction;
    motor->output_angle_deg = (float)motor->rotor_total *
                              (360.0f / C610_ENCODER_COUNTS) /
                              M2006_GEAR_RATIO *
                              (float)motor->direction;
    motor->online = motor->feedback_seen &&
                    ((uint32_t)(now_ms - motor->last_feedback_ms) <=
                     C610_FEEDBACK_TIMEOUT_MS);

    // 未使能或反馈超时后立即清 PID，并在组控制帧中发送零电流。
    if (!motor->online ||
        (motor->control_mode != M2006_CONTROL_POSITION))
    {
        pid_reset(&motor->pid);
    }
    else
    {
        float error_deg = motor->target_position_deg -
                          motor->output_angle_deg;

        if ((error_deg <= -M2006_POSITION_DEADBAND_DEG) ||
            (error_deg >= M2006_POSITION_DEADBAND_DEG))
        {
            current = pid_run(&motor->pid, error_deg) *
                      (float)motor->direction;
        }
        else
        {
            pid_reset(&motor->pid);
        }
    }

    motor->current_command =
        (int16_t)clamp_symmetric(current, motor->pid.output_limit);
}

static HAL_StatusTypeDef send_group(c610_bus_t *bus, uint8_t group)
{
    uint8_t data[8] = {0};
    uint8_t i;
    uint16_t tx_id = (group == 0U) ? C610_COMMAND_ID_1_TO_4
                                   : C610_COMMAND_ID_5_TO_8;

    // C610 每帧同时控制连续的四个电机 ID，未使用槽位填零。
    for (i = 0U; i < 4U; i++)
    {
        m2006_motor_t *motor = bus->motors[group * 4U + i];
        int16_t current = (motor == NULL) ? 0 : motor->current_command;

        data[i * 2U] = (uint8_t)((uint16_t)current >> 8);
        data[i * 2U + 1U] = (uint8_t)current;
    }

    return StdCan_Send(bus->can, tx_id, data);
}

HAL_StatusTypeDef C610_Init(c610_bus_t *bus, std_can_t *can,
                            m2006_motor_t *motors,
                            const m2006_config_t *configs,
                            uint8_t motor_count)
{
    uint8_t i;

    if ((bus == NULL) || (can == NULL) || !can->ready ||
        (motors == NULL) || (configs == NULL) || (motor_count == 0U) ||
        (motor_count > C610_MAX_MOTORS))
    {
        return HAL_ERROR;
    }

    memset(bus, 0, sizeof(*bus));
    memset(motors, 0, sizeof(*motors) * motor_count);
    bus->can = can;
    bus->motor_list = motors;
    bus->motor_count = motor_count;
    bus->last_control_ms = 0U;

    for (i = 0U; i < motor_count; i++)
    {
        if (!config_valid(&configs[i]) ||
            !attach_motor(bus, &motors[i], &configs[i]))
        {
            return HAL_ERROR;
        }
    }

    if (StdCan_AddHandler(can, handle_rx, bus) != HAL_OK)
    {
        return HAL_ERROR;
    }
    bus->ready = true;
    return HAL_OK;
}

void C610_Run(c610_bus_t *bus, uint32_t now_ms)
{
    uint8_t i;

    if ((bus == NULL) || !bus->ready)
    {
        return;
    }
    if ((uint32_t)(now_ms - bus->last_control_ms) <
        C610_CONTROL_PERIOD_MS)
    {
        return;
    }
    bus->last_control_ms = now_ms;

    for (i = 0U; i < bus->motor_count; i++)
    {
        update_motor(&bus->motor_list[i], now_ms);
    }
    for (i = 0U; i < 2U; i++)
    {
        if (bus->group_used[i])
        {
            (void)send_group(bus, i);
        }
    }
}

HAL_StatusTypeDef C610_SetPos(c610_bus_t *bus, uint8_t id,
                              float target_position_deg)
{
    m2006_motor_t *motor = find_motor(bus, id);

    if ((motor == NULL) || !finite_float(target_position_deg))
    {
        return HAL_ERROR;
    }

    motor->target_position_deg = target_position_deg;
    motor->control_mode = M2006_CONTROL_POSITION;
    return HAL_OK;
}

HAL_StatusTypeDef C610_SetPid(c610_bus_t *bus, uint8_t id,
                              m2006_pid_gains_t gains)
{
    m2006_motor_t *motor = find_motor(bus, id);

    if ((motor == NULL) || !pid_gains_valid(&gains))
    {
        return HAL_ERROR;
    }

    pid_reset(&motor->pid);
    motor->pid.gains = gains;
    return HAL_OK;
}

void C610_Enable(c610_bus_t *bus, uint8_t id, bool enabled)
{
    m2006_motor_t *motor = find_motor(bus, id);

    if (motor == NULL)
    {
        return;
    }

    if (!enabled)
    {
        pid_reset(&motor->pid);
        motor->current_command = 0;
    }
    motor->control_mode = enabled ? M2006_CONTROL_POSITION
                                  : M2006_CONTROL_COAST;
}

HAL_StatusTypeDef C610_CoastAll(c610_bus_t *bus)
{
    uint8_t i;
    HAL_StatusTypeDef result = HAL_OK;

    if ((bus == NULL) || !bus->ready)
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < bus->motor_count; i++)
    {
        C610_Enable(bus, bus->motor_list[i].id, false);
    }
    for (i = 0U; i < 2U; i++)
    {
        if (bus->group_used[i] && (send_group(bus, i) != HAL_OK))
        {
            result = HAL_ERROR;
        }
    }
    bus->last_control_ms = HAL_GetTick();
    return result;
}

void C610_StopAll(c610_bus_t *bus)
{
    uint8_t i;

    if ((bus == NULL) || !bus->ready)
    {
        return;
    }
    for (i = 0U; i < bus->motor_count; i++)
    {
        C610_Enable(bus, bus->motor_list[i].id, false);
    }
}

bool C610_GetStatus(const c610_bus_t *bus, uint8_t id,
                    m2006_status_t *status)
{
    const m2006_motor_t *motor = find_const_motor(bus, id);

    if ((motor == NULL) || (status == NULL))
    {
        return false;
    }

    status->output_angle_deg = motor->output_angle_deg;
    status->output_speed_rpm = motor->output_speed_rpm;
    status->current_feedback = motor->current_feedback;
    status->current_command = motor->current_command;
    status->last_feedback_ms = motor->last_feedback_ms;
    status->feedback_seen = motor->feedback_seen;
    status->online = motor->online;
    status->control_mode = motor->control_mode;
    status->enabled = motor->control_mode == M2006_CONTROL_POSITION;
    return true;
}
