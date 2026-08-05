#include "vesc_motor.h"

#include <limits.h>
#include <stddef.h>

#define VESC_PACKET_SET_BRAKE 2U
#define VESC_PACKET_SET_RPM   3U
#define VESC_PACKET_STATUS    9U
#define VESC_CURRENT_SCALE    1000.0f

static void put_be32(uint8_t *data, int32_t value)
{
    data[0] = (uint8_t)((uint32_t)value >> 24);
    data[1] = (uint8_t)((uint32_t)value >> 16);
    data[2] = (uint8_t)((uint32_t)value >> 8);
    data[3] = (uint8_t)value;
}

static int16_t get_be16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int32_t get_be32(const uint8_t *data)
{
    uint32_t value;

    value = ((uint32_t)data[0] << 24) |
            ((uint32_t)data[1] << 16) |
            ((uint32_t)data[2] << 8) |
            data[3];
    return (int32_t)value;
}

static int32_t rpm_to_erpm(const vesc_motor_t *motor, int32_t rpm)
{
    int64_t erpm = (int64_t)rpm * motor->config.pole_pairs;

    if (erpm > INT32_MAX)
    {
        return INT32_MAX;
    }
    if (erpm < INT32_MIN)
    {
        return INT32_MIN;
    }

    return (int32_t)erpm;
}

static int32_t erpm_to_rpm(const vesc_motor_t *motor, int32_t erpm)
{
    int64_t value = erpm;
    int64_t divisor = motor->config.pole_pairs;

    if (value >= 0)
    {
        return (int32_t)((value + divisor / 2) / divisor);
    }

    return (int32_t)(-((-value + divisor / 2) / divisor));
}

static int32_t limit_rpm(const vesc_motor_t *motor, int32_t rpm)
{
    int64_t magnitude = rpm;

    if (magnitude < 0)
    {
        magnitude = -magnitude;
    }
    if (magnitude < motor->config.min_rpm)
    {
        return 0;
    }
    if (magnitude > motor->config.max_rpm)
    {
        return (rpm < 0) ? -motor->config.max_rpm
                         : motor->config.max_rpm;
    }

    return rpm;
}

HAL_StatusTypeDef VescMotor_Init(vesc_motor_t *motor, vesc_can_t *bus,
                                 const vesc_motor_config_t *config)
{
    if ((motor == NULL) || (bus == NULL) || (config == NULL) ||
        (config->pole_pairs == 0U) || (config->min_rpm < 0) ||
        (config->max_rpm < config->min_rpm) ||
        (config->brake_current_a < 0.0f))
    {
        return HAL_ERROR;
    }

    motor->bus = bus;
    motor->config = *config;
    motor->status.target_rpm = 0;
    motor->status.actual_rpm = 0;
    motor->status.current_a = 0.0f;
    motor->status.duty = 0.0f;
    motor->status.online = false;
    motor->status.last_rx_ms = 0U;
    return HAL_OK;
}

HAL_StatusTypeDef VescMotor_SetRpm(vesc_motor_t *motor,
                                   int32_t target_rpm)
{
    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    motor->status.target_rpm = limit_rpm(motor, target_rpm);
    return HAL_OK;
}

HAL_StatusTypeDef VescMotor_SendRpm(vesc_motor_t *motor)
{
    uint8_t data[4];

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return HAL_ERROR;
    }

    put_be32(data, rpm_to_erpm(motor, motor->status.target_rpm));
    return VescCan_Send(motor->bus, motor->config.id,
                        VESC_PACKET_SET_RPM, data, sizeof(data));
}

HAL_StatusTypeDef VescMotor_Brake(vesc_motor_t *motor)
{
    uint8_t data[4];
    int32_t brake_ma;

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return HAL_ERROR;
    }

    motor->status.target_rpm = 0;
    brake_ma = (int32_t)(motor->config.brake_current_a *
                         VESC_CURRENT_SCALE);
    put_be32(data, brake_ma);
    return VescCan_Send(motor->bus, motor->config.id,
                        VESC_PACKET_SET_BRAKE, data, sizeof(data));
}

bool VescMotor_Parse(vesc_motor_t *motor, const vesc_can_msg_t *msg,
                     uint32_t now_ms)
{
    if ((motor == NULL) || (msg == NULL) ||
        (msg->motor_id != motor->config.id) ||
        (msg->packet_id != VESC_PACKET_STATUS) ||
        (msg->length != 8U))
    {
        return false;
    }

    motor->status.actual_rpm = erpm_to_rpm(motor,
                                            get_be32(&msg->data[0]));
    motor->status.current_a = (float)get_be16(&msg->data[4]) / 10.0f;
    motor->status.duty = (float)get_be16(&msg->data[6]) / 1000.0f;
    motor->status.online = true;
    motor->status.last_rx_ms = now_ms;
    return true;
}

void VescMotor_Update(vesc_motor_t *motor, uint32_t now_ms)
{
    if (motor == NULL)
    {
        return;
    }

    if ((motor->status.last_rx_ms == 0U) ||
        ((now_ms - motor->status.last_rx_ms) > VESC_STATUS_TIMEOUT_MS))
    {
        motor->status.online = false;
    }
}

bool VescMotor_GetStatus(const vesc_motor_t *motor,
                         vesc_motor_status_t *status)
{
    if ((motor == NULL) || (status == NULL))
    {
        return false;
    }

    *status = motor->status;
    return true;
}
