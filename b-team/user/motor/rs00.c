#include "rs00.h"

#include <float.h>
#include <string.h>

/* RS00 协议范围和数据缩放。 */
#define RS_MODE_DELAY_MS        3U
#define RS_ENABLE_TIMEOUT_MS    100U
#define RS_ZERO_TIMEOUT_MS      1500U
#define RS_ZERO_ERROR_RAD       0.05f
#define RS_LIMIT_IQ_VALID       (1U << 0)
#define RS_LIMIT_SPEED_VALID    (1U << 1)

#define RS_PI            3.14159265358979323846f
#define RS_P_MIN         (-4.0f * RS_PI)
#define RS_P_MAX         (4.0f * RS_PI)
#define RS_P_PERIOD      (8.0f * RS_PI)
#define RS_V_MIN         (-33.0f)
#define RS_V_MAX         33.0f
#define RS_T_MIN         (-14.0f)
#define RS_T_MAX         14.0f
#define RS_IQ_MIN        (-16.0f)
#define RS_IQ_MAX        16.0f
#define RS_UINT16_MAX    65535.0f
#define RS_MOTION_KP_MAX 500.0f
#define RS_MOTION_KD_MAX 5.0f
#define RS_TEMP_SCALE_C  0.1f

enum
{
    RS_CMD_MOTION = 0x01,
    RS_CMD_FDB = 0x02,
    RS_CMD_ON = 0x03,
    RS_CMD_OFF = 0x04,
    RS_CMD_ZERO = 0x06,
    RS_CMD_READ = 0x11,
    RS_CMD_WRITE = 0x12,
    RS_CMD_FAULT = 0x15
};

enum
{
    RS_PARAM_MODE = 0x7005,
    RS_PARAM_IQ_REF = 0x7006,
    RS_PARAM_SPEED_REF = 0x700A,
    RS_PARAM_POS_REF = 0x7016,
    RS_PARAM_SPEED_MAX = 0x7017,
    RS_PARAM_IQ_MAX = 0x7018,
    RS_PARAM_PP_SPEED = 0x7024,
    RS_PARAM_PP_ACCEL = 0x7025
};

typedef struct
{
    uint32_t id;
    uint8_t data[8];
} rs_packet_t;

static bool finite_float(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static float clampf(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

static float wrap_position(float radian)
{
    while (radian > RS_P_MAX)
    {
        radian -= RS_P_PERIOD;
    }
    while (radian < RS_P_MIN)
    {
        radian += RS_P_PERIOD;
    }
    return radian;
}

static void pack_u16_le(uint8_t data[2], uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static uint16_t scale_u16(float value, float min, float max)
{
    value = clampf(value, min, max);
    return (uint16_t)((value - min) * RS_UINT16_MAX / (max - min));
}

static void pack_scaled_u16(uint8_t data[2], float value, float min, float max)
{
    uint16_t raw = scale_u16(value, min, max);

    data[0] = (uint8_t)(raw >> 8);
    data[1] = (uint8_t)raw;
}

static float unpack_scaled_u16(const uint8_t data[2], float min, float max)
{
    uint16_t raw = ((uint16_t)data[0] << 8) | (uint16_t)data[1];

    return ((float)raw * (max - min) / RS_UINT16_MAX) + min;
}

static void pack_float_le(uint8_t data[4], float value)
{
    uint32_t raw;

    memcpy(&raw, &value, sizeof(raw));
    data[0] = (uint8_t)raw;
    data[1] = (uint8_t)(raw >> 8);
    data[2] = (uint8_t)(raw >> 16);
    data[3] = (uint8_t)(raw >> 24);
}

static uint32_t unpack_u32_le(const uint8_t data[4])
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void pack_command(rs_packet_t *packet, const rs_motor_t *motor,
                         uint8_t command,
                         uint16_t extra)
{
    memset(packet, 0, sizeof(*packet));
    packet->id =
        ((uint32_t)command << 24) | ((uint32_t)extra << 8) |
        (uint32_t)motor->id;
}

static bool mode_ok(rs_mode_t mode)
{
    return (mode == RS_MOTION) || (mode == RS_PP) || (mode == RS_SPD) ||
           (mode == RS_IQ) || (mode == RS_CSP);
}

static bool motor_ok(const rs_motor_t *motor)
{
    return (motor != NULL) && (motor->bus != NULL) && motor->bus->ready &&
           (motor->id <= RS_MOTOR_ID_MAX);
}

static HAL_StatusTypeDef require_mode(rs_motor_t *motor, rs_mode_t mode)
{
    if (!motor_ok(motor) || !motor->active ||
        (motor->mode != (uint8_t)mode))
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef send_command(rs_motor_t *motor, uint8_t command,
                                      uint8_t first_byte)
{
    rs_packet_t packet;

    if (!motor_ok(motor))
    {
        return HAL_ERROR;
    }

    pack_command(&packet, motor, command, motor->bus->host_id);
    packet.data[0] = first_byte;
    return RsBus_Send(motor->bus, packet.id, packet.data);
}

static HAL_StatusTypeDef write_u8(rs_motor_t *motor, uint16_t index,
                                  uint8_t value)
{
    rs_packet_t packet;

    if (!motor_ok(motor))
    {
        return HAL_ERROR;
    }

    pack_command(&packet, motor, RS_CMD_WRITE, motor->bus->host_id);
    pack_u16_le(packet.data, index);
    packet.data[4] = value;
    return RsBus_Send(motor->bus, packet.id, packet.data);
}

static HAL_StatusTypeDef write_float(rs_motor_t *motor, uint16_t index,
                                     float value)
{
    rs_packet_t packet;

    if (!motor_ok(motor) || !finite_float(value))
    {
        return HAL_ERROR;
    }

    pack_command(&packet, motor, RS_CMD_WRITE, motor->bus->host_id);
    pack_u16_le(packet.data, index);
    pack_float_le(&packet.data[4], value);
    return RsBus_Send(motor->bus, packet.id, packet.data);
}

static bool time_reached(uint32_t now_ms, uint32_t due_ms)
{
    return (int32_t)(now_ms - due_ms) >= 0;
}

HAL_StatusTypeDef RsMotor_Init(rs_motor_t *motor, rs_bus_t *bus, uint8_t id)
{
    if ((motor == NULL) || (bus == NULL) || !bus->ready ||
        (id > RS_MOTOR_ID_MAX))
    {
        return HAL_ERROR;
    }
    memset(motor, 0, sizeof(*motor));
    motor->bus = bus;
    motor->id = id;
    motor->mode = UINT8_MAX;
    motor->requested_mode = UINT8_MAX;
    return HAL_OK;
}

HAL_StatusTypeDef RsMotor_SetZero(rs_motor_t *motor)
{
    HAL_StatusTypeDef status;

    if (!motor_ok(motor) || motor->active)
    {
        return HAL_ERROR;
    }
    if (motor->zero.pending)
    {
        return HAL_BUSY;
    }

    motor->zero.sent_ms = HAL_GetTick();
    motor->zero.feedback_sequence = motor->feedback.sequence;
    motor->zero.result = HAL_BUSY;
    motor->zero.pending = true;
    motor->zero.completed = false;

    status = send_command(motor, RS_CMD_ZERO, 1U);
    if (status != HAL_OK)
    {
        motor->zero.pending = false;
        motor->zero.result = status;
    }
    return status;
}

HAL_StatusTypeDef RsMotor_GetZeroStatus(rs_motor_t *motor)
{
    if (!motor_ok(motor))
    {
        return HAL_ERROR;
    }
    if (motor->zero.pending)
    {
        if ((uint32_t)(HAL_GetTick() - motor->zero.sent_ms) >=
            RS_ZERO_TIMEOUT_MS)
        {
            motor->zero.pending = false;
            motor->zero.result = HAL_TIMEOUT;
            return HAL_TIMEOUT;
        }
        return HAL_BUSY;
    }
    if (!motor->zero.completed)
    {
        return HAL_ERROR;
    }
    return motor->zero.result;
}

HAL_StatusTypeDef RsMotor_Start(rs_motor_t *motor, rs_mode_t mode,
                                uint32_t now_ms)
{
    HAL_StatusTypeDef status;

    if (!motor_ok(motor) || !mode_ok(mode))
    {
        return HAL_ERROR;
    }

    if ((motor->start_step != RS_START_IDLE) &&
        (motor->requested_mode != (uint8_t)mode))
    {
        motor->start_step = RS_START_IDLE;
    }

    motor->requested_mode = (uint8_t)mode;

    /* 停止、写模式、使能分多次调用推进，避免阻塞 1 ms 任务。 */
    if (motor->start_step == RS_START_IDLE)
    {
        if (motor->active)
        {
            status = send_command(motor, RS_CMD_OFF, 0U);
            if (status != HAL_OK)
            {
                return status;
            }
            motor->active = false;
            motor->transition_due_ms = now_ms + RS_MODE_DELAY_MS;
            motor->start_step = RS_START_MODE;
            return HAL_BUSY;
        }

        motor->start_step = RS_START_MODE;
        motor->transition_due_ms = now_ms;
    }

    if (!time_reached(now_ms, motor->transition_due_ms))
    {
        return HAL_BUSY;
    }

    if (motor->start_step == RS_START_MODE)
    {
        if (motor->mode != (uint8_t)mode)
        {
            status = write_u8(motor, RS_PARAM_MODE, (uint8_t)mode);
            if (status != HAL_OK)
            {
                motor->start_step = RS_START_IDLE;
                return status;
            }
            motor->mode = (uint8_t)mode;
            motor->transition_due_ms = now_ms + RS_MODE_DELAY_MS;
            motor->start_step = RS_START_ENABLE;
            return HAL_BUSY;
        }
        motor->start_step = RS_START_ENABLE;
    }

    if (motor->start_step == RS_START_ENABLE)
    {
        status = send_command(motor, RS_CMD_ON, 0U);
        if (status != HAL_OK)
        {
            motor->start_step = RS_START_IDLE;
            return status;
        }
        motor->transition_due_ms = now_ms + RS_ENABLE_TIMEOUT_MS;
        motor->start_step = RS_START_WAIT;
        return HAL_BUSY;
    }

    if (motor->start_step != RS_START_WAIT)
    {
        motor->start_step = RS_START_IDLE;
        return HAL_ERROR;
    }
    if (!motor->active)
    {
        if (time_reached(now_ms, motor->transition_due_ms))
        {
            motor->mode = UINT8_MAX;
            motor->requested_mode = UINT8_MAX;
            motor->start_step = RS_START_IDLE;
            return HAL_TIMEOUT;
        }
        return HAL_BUSY;
    }

    motor->pp_configured = false;
    motor->limit_valid = 0U;
    motor->start_step = RS_START_IDLE;
    return HAL_OK;
}

HAL_StatusTypeDef RsMotor_Stop(rs_motor_t *motor)
{
    HAL_StatusTypeDef status;

    if (!motor_ok(motor))
    {
        return HAL_ERROR;
    }

    motor->start_step = RS_START_IDLE;
    status = send_command(motor, RS_CMD_OFF, 0U);
    if (status == HAL_OK)
    {
        motor->active = false;
    }
    return status;
}

HAL_StatusTypeDef RsMotor_ClearFault(rs_motor_t *motor)
{
    HAL_StatusTypeDef status;

    if (!motor_ok(motor))
    {
        return HAL_ERROR;
    }

    motor->start_step = RS_START_IDLE;
    status = send_command(motor, RS_CMD_OFF, 1U);
    if (status == HAL_OK)
    {
        motor->active = false;
    }
    return status;
}

HAL_StatusTypeDef RsMotor_SetMotion(rs_motor_t *motor, float position_rad,
                                    float velocity_rad_s, float torque_nm,
                                    float kp, float kd)
{
    rs_packet_t packet;
    HAL_StatusTypeDef status;

    if (!finite_float(position_rad) || !finite_float(velocity_rad_s) ||
        !finite_float(torque_nm) || !finite_float(kp) || !finite_float(kd))
    {
        return HAL_ERROR;
    }

    status = require_mode(motor, RS_MOTION);
    if (status != HAL_OK)
    {
        return status;
    }

    position_rad = wrap_position(clampf(position_rad, RS_P_MIN, RS_P_MAX));
    pack_command(&packet, motor, RS_CMD_MOTION,
                 scale_u16(torque_nm, RS_T_MIN, RS_T_MAX));
    pack_scaled_u16(&packet.data[0], position_rad, RS_P_MIN, RS_P_MAX);
    pack_scaled_u16(&packet.data[2], velocity_rad_s, RS_V_MIN, RS_V_MAX);
    pack_scaled_u16(&packet.data[4], kp, 0.0f, RS_MOTION_KP_MAX);
    pack_scaled_u16(&packet.data[6], kd, 0.0f, RS_MOTION_KD_MAX);
    return RsBus_Send(motor->bus, packet.id, packet.data);
}

HAL_StatusTypeDef RsMotor_SetIq(rs_motor_t *motor, float iq)
{
    HAL_StatusTypeDef status;

    if (!finite_float(iq))
    {
        return HAL_ERROR;
    }
    status = require_mode(motor, RS_IQ);
    if (status != HAL_OK)
    {
        return status;
    }

    return write_float(motor, RS_PARAM_IQ_REF,
                       clampf(iq, RS_IQ_MIN, RS_IQ_MAX));
}

HAL_StatusTypeDef RsMotor_SetSpeed(rs_motor_t *motor, float velocity_rad_s,
                                   float max_iq)
{
    float iq_limit;
    HAL_StatusTypeDef status;

    if (!finite_float(velocity_rad_s) || !finite_float(max_iq) ||
        (max_iq < 0.0f))
    {
        return HAL_ERROR;
    }

    status = require_mode(motor, RS_SPD);
    if (status != HAL_OK)
    {
        return status;
    }

    iq_limit = clampf(max_iq, 0.0f, RS_IQ_MAX);
    if (((motor->limit_valid & RS_LIMIT_IQ_VALID) == 0U) ||
        (motor->limit_iq != iq_limit))
    {
        status = write_float(motor, RS_PARAM_IQ_MAX, iq_limit);
        if (status != HAL_OK)
        {
            return status;
        }
        motor->limit_iq = iq_limit;
        motor->limit_valid |= RS_LIMIT_IQ_VALID;
    }

    return write_float(motor, RS_PARAM_SPEED_REF,
                       clampf(velocity_rad_s, RS_V_MIN, RS_V_MAX));
}

HAL_StatusTypeDef RsMotor_SetCsp(rs_motor_t *motor, float position_rad,
                                 float max_velocity_rad_s)
{
    HAL_StatusTypeDef status;

    if (!finite_float(position_rad) || !finite_float(max_velocity_rad_s) ||
        (max_velocity_rad_s < 0.0f))
    {
        return HAL_ERROR;
    }
    status = require_mode(motor, RS_CSP);
    if (status != HAL_OK)
    {
        return status;
    }

    max_velocity_rad_s = clampf(max_velocity_rad_s, 0.0f, RS_V_MAX);
    if (((motor->limit_valid & RS_LIMIT_SPEED_VALID) == 0U) ||
        (motor->limit_speed_rad_s != max_velocity_rad_s))
    {
        status = write_float(motor, RS_PARAM_SPEED_MAX,
                             max_velocity_rad_s);
        if (status != HAL_OK)
        {
            return status;
        }
        motor->limit_speed_rad_s = max_velocity_rad_s;
        motor->limit_valid |= RS_LIMIT_SPEED_VALID;
    }

    return write_float(motor, RS_PARAM_POS_REF, position_rad);
}

HAL_StatusTypeDef RsMotor_SetPp(rs_motor_t *motor, float position_rad,
                                float max_velocity_rad_s,
                                float acceleration_rad_s2)
{
    HAL_StatusTypeDef status;

    if (!finite_float(position_rad) || !finite_float(max_velocity_rad_s) ||
        !finite_float(acceleration_rad_s2) || (max_velocity_rad_s < 0.0f) ||
        (acceleration_rad_s2 < 0.0f))
    {
        return HAL_ERROR;
    }

    status = require_mode(motor, RS_PP);
    if (status != HAL_OK)
    {
        return status;
    }

    max_velocity_rad_s = clampf(max_velocity_rad_s, 0.0f, RS_V_MAX);

    if (!motor->pp_configured)
    {
        status = write_float(motor, RS_PARAM_PP_SPEED, max_velocity_rad_s);
        if (status != HAL_OK)
        {
            return status;
        }
        status = write_float(motor, RS_PARAM_PP_ACCEL, acceleration_rad_s2);
        if (status != HAL_OK)
        {
            return status;
        }

        motor->pp_speed_rad_s = max_velocity_rad_s;
        motor->pp_acceleration_rad_s2 = acceleration_rad_s2;
        motor->pp_configured = true;
    }
    else if ((motor->pp_speed_rad_s != max_velocity_rad_s) ||
             (motor->pp_acceleration_rad_s2 != acceleration_rad_s2))
    {
        return HAL_BUSY;
    }

    return write_float(motor, RS_PARAM_POS_REF, position_rad);
}

HAL_StatusTypeDef RsMotor_HaltPp(rs_motor_t *motor)
{
    HAL_StatusTypeDef status = require_mode(motor, RS_PP);

    if (status != HAL_OK)
    {
        return status;
    }

    status = write_float(motor, RS_PARAM_PP_SPEED, 0.0f);
    if (status == HAL_OK)
    {
        motor->pp_speed_rad_s = 0.0f;
    }
    return status;
}

HAL_StatusTypeDef RsMotor_GetFeedback(const rs_motor_t *motor,
                                      rs_feedback_t *feedback)
{
    if ((motor == NULL) || (feedback == NULL))
    {
        return HAL_ERROR;
    }

    *feedback = motor->feedback;
    return HAL_OK;
}

static void parse_feedback(rs_motor_t *motor, uint32_t id,
                           const uint8_t data[8])
{
    motor->feedback.position_rad =
        unpack_scaled_u16(&data[0], RS_P_MIN, RS_P_MAX);
    motor->feedback.valid |= RS_FDB_POSITION;

    motor->feedback.velocity_rad_s =
        unpack_scaled_u16(&data[2], RS_V_MIN, RS_V_MAX);
    motor->feedback.torque_nm =
        unpack_scaled_u16(&data[4], RS_T_MIN, RS_T_MAX);
    motor->feedback.temperature_c =
        (float)(((uint16_t)data[6] << 8) | data[7]) * RS_TEMP_SCALE_C;
    motor->feedback.fault = (id >> 16) & 0x3FU;
    motor->feedback.state = (uint8_t)((id >> 22) & 0x03U);
    motor->active =
        motor->feedback.state == (uint8_t)RS_RUN;
    if (!motor->active && (motor->start_step == RS_START_IDLE) &&
        (motor->mode != UINT8_MAX))
    {
        /* Reset feedback invalidates the run mode cached by the MCU. */
        motor->mode = UINT8_MAX;
        motor->requested_mode = UINT8_MAX;
    }
    motor->feedback.valid |= RS_FDB_SPEED | RS_FDB_TORQUE | RS_FDB_TEMP |
                             RS_FDB_STATE | RS_FDB_FAULT;
    motor->feedback.sequence++;

    if (motor->zero.pending &&
        (motor->feedback.sequence != motor->zero.feedback_sequence) &&
        (motor->feedback.position_rad >= -RS_ZERO_ERROR_RAD) &&
        (motor->feedback.position_rad <= RS_ZERO_ERROR_RAD))
    {
        motor->zero.pending = false;
        motor->zero.completed = true;
        motor->zero.result = HAL_OK;
    }
}

static void parse_fault(rs_motor_t *motor, const uint8_t data[8])
{
    motor->feedback.fault = unpack_u32_le(&data[0]);
    motor->feedback.warning = unpack_u32_le(&data[4]);
    motor->feedback.valid |= RS_FDB_FAULT;
    motor->feedback.sequence++;
}

void RsMotor_Parse(rs_motor_t *motor, uint32_t id, const uint8_t data[8],
                   uint32_t tick_ms)
{
    uint8_t type;
    uint8_t motor_id;

    if (!motor_ok(motor) || (data == NULL) ||
        ((uint8_t)id != motor->bus->host_id))
    {
        return;
    }

    type = (uint8_t)((id >> 24) & 0x1FU);
    motor_id = (uint8_t)((id >> 8) & 0xFFU);
    if (motor_id != motor->id)
    {
        return;
    }

    switch (type)
    {
    case RS_CMD_FDB:
        parse_feedback(motor, id, data);
        break;

    case RS_CMD_READ:
        break;

    case RS_CMD_WRITE:
        break;

    case RS_CMD_FAULT:
        parse_fault(motor, data);
        break;

    default:
        return;
    }

    motor->last_rx_ms = tick_ms;
}
