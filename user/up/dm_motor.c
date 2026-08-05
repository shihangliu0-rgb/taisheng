#include "dm_motor.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

#define DM_STD_ID_MAX      0x7FFU
#define DM_FEEDBACK_ID_MAX 0x0FU
#define DM_SPECIAL_DATA    0xFFU
#define DM_CMD_ENABLE      0xFCU
#define DM_CMD_DISABLE     0xFDU
#define DM_CMD_ZERO        0xFEU
#define DM_POSITION_BITS   16U      // 位置用16位表示
#define DM_PARAMETER_BITS  12U      // 速度/力矩/KP/KD用12位表示


static bool valid_float(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static bool value_in_range(float value, float min, float max)
{
    return valid_float(value) && (value >= min) && (value <= max);
}

static float clamp(float value, float min, float max)
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

static uint16_t float_to_uint(float value, float min, float max, uint8_t bits)
{
    const uint32_t scale = (1UL << bits) - 1UL;

    value = clamp(value, min, max);
    return (uint16_t)((value - min) * (float)scale / (max - min));
}

static float uint_to_float(uint16_t value, float min, float max, uint8_t bits)
{
    const uint32_t scale = (1UL << bits) - 1UL;

    return ((float)value * (max - min) / (float)scale) + min;
}

static dm_result_t prepare_frame(const dm_motor_t *motor, dm_frame_t *frame)
{
    if ((motor == NULL) || (frame == NULL) || (motor->tx_id > DM_STD_ID_MAX))
    {
        return DM_BAD_ARG;
    }

    memset(frame, 0, sizeof(*frame));
    frame->id = motor->tx_id;
    frame->length = 8U;
    return DM_OK;
}

static dm_result_t build_special_frame(const dm_motor_t *motor, uint8_t command,
                                       dm_frame_t *frame)
{
    dm_result_t result = prepare_frame(motor, frame);

    if (result != DM_OK)
    {
        return result;
    }

    memset(frame->data, DM_SPECIAL_DATA, 7U);   // 前7字节填0xFF
    frame->data[7] = command;                   // 第8字节填命令码
    return DM_OK;
}

dm_result_t DmMotor_Init(dm_motor_t *motor, uint16_t tx_id, uint16_t master_id,
                         uint8_t feedback_id)
{
    if ((motor == NULL) || (tx_id > DM_STD_ID_MAX) ||
        (master_id > DM_STD_ID_MAX) || (feedback_id > DM_FEEDBACK_ID_MAX))
    {
        return DM_BAD_ARG;
    }

    memset(motor, 0, sizeof(*motor));
    motor->tx_id = tx_id;
    motor->master_id = master_id;
    motor->feedback_id = feedback_id;
    motor->limits.position_rad = DM_J4310_P_MAX;
    motor->limits.velocity_rad_s = DM_J4310_V_MAX;
    motor->limits.torque_nm = DM_J4310_T_MAX;
    return DM_OK;
}

dm_result_t DmMotor_SetLimits(dm_motor_t *motor, float p_max, float v_max,
                              float t_max)
{
    if ((motor == NULL) || !valid_float(p_max) || !valid_float(v_max) ||
        !valid_float(t_max) || (p_max <= 0.0f) || (v_max <= 0.0f) ||
        (t_max <= 0.0f))
    {
        return DM_BAD_ARG;
    }

    motor->limits.position_rad = p_max;
    motor->limits.velocity_rad_s = v_max;
    motor->limits.torque_nm = t_max;
    return DM_OK;
}

dm_result_t DmMotor_BuildEnable(const dm_motor_t *motor, dm_frame_t *frame)
{
    return build_special_frame(motor, DM_CMD_ENABLE, frame);
}

dm_result_t DmMotor_BuildDisable(const dm_motor_t *motor, dm_frame_t *frame)
{
    return build_special_frame(motor, DM_CMD_DISABLE, frame);
}

dm_result_t DmMotor_BuildZero(const dm_motor_t *motor, dm_frame_t *frame)
{
    return build_special_frame(motor, DM_CMD_ZERO, frame);
}

dm_result_t DmMotor_BuildMit(const dm_motor_t *motor, const dm_mit_cmd_t *cmd,
                             dm_frame_t *frame)
{
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    uint16_t kp;
    uint16_t kd;
    dm_result_t result;

    if ((motor == NULL) || (cmd == NULL))
    {
        return DM_BAD_ARG;
    }
    if (!value_in_range(cmd->position_rad, -motor->limits.position_rad,
                        motor->limits.position_rad) ||
        !value_in_range(cmd->velocity_rad_s, -motor->limits.velocity_rad_s,
                        motor->limits.velocity_rad_s) ||
        !value_in_range(cmd->torque_nm, -motor->limits.torque_nm,
                        motor->limits.torque_nm) ||
        !value_in_range(cmd->kp, 0.0f, DM_MIT_KP_MAX) ||
        !value_in_range(cmd->kd, 0.0f, DM_MIT_KD_MAX))
    {
        return DM_BAD_ARG;
    }

    result = prepare_frame(motor, frame);
    if (result != DM_OK)
    {
        return result;
    }

    position = float_to_uint(cmd->position_rad, -motor->limits.position_rad,
                             motor->limits.position_rad, DM_POSITION_BITS);
    velocity = float_to_uint(cmd->velocity_rad_s, -motor->limits.velocity_rad_s,
                             motor->limits.velocity_rad_s, DM_PARAMETER_BITS);
    torque = float_to_uint(cmd->torque_nm, -motor->limits.torque_nm,
                           motor->limits.torque_nm, DM_PARAMETER_BITS);
    kp = float_to_uint(cmd->kp, 0.0f, DM_MIT_KP_MAX, DM_PARAMETER_BITS);
    kd = float_to_uint(cmd->kd, 0.0f, DM_MIT_KD_MAX, DM_PARAMETER_BITS);

    frame->data[0] = (uint8_t)(position >> 8);
    frame->data[1] = (uint8_t)position;
    frame->data[2] = (uint8_t)(velocity >> 4);
    frame->data[3] = (uint8_t)((velocity << 4) | (kp >> 8));
    frame->data[4] = (uint8_t)kp;
    frame->data[5] = (uint8_t)(kd >> 4);
    frame->data[6] = (uint8_t)((kd << 4) | (torque >> 8));
    frame->data[7] = (uint8_t)torque;
    return DM_OK;
}

bool DmMotor_Parse(dm_motor_t *motor, uint16_t id, const uint8_t *data,
                   uint8_t length, uint32_t tick_ms)
{
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    uint32_t seq;

    if ((motor == NULL) || (data == NULL) || (length != 8U) ||
        (id != motor->master_id) ||
        ((data[0] & DM_FEEDBACK_ID_MAX) != motor->feedback_id))
    {
        return false;
    }

    position = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
    velocity = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
    torque = (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8) | data[5]);

    // 奇数序号表示正在写入，偶数序号表示反馈完整。
    seq = motor->state_seq;
    motor->state_seq = seq + 1U;
    motor->state.position_rad =
        uint_to_float(position, -motor->limits.position_rad,
                      motor->limits.position_rad, DM_POSITION_BITS);
    motor->state.velocity_rad_s =
        uint_to_float(velocity, -motor->limits.velocity_rad_s,
                      motor->limits.velocity_rad_s, DM_PARAMETER_BITS);
    motor->state.torque_nm =
        uint_to_float(torque, -motor->limits.torque_nm, motor->limits.torque_nm,
                      DM_PARAMETER_BITS);
    motor->state.fault = (dm_fault_t)(data[0] >> 4);
    motor->state.mos_temp_c = data[6];
    motor->state.rotor_temp_c = data[7];
    motor->state.rx_tick_ms = tick_ms;
    motor->state.rx_count++;
    motor->state_seq = seq + 2U;
    return true;
}

bool DmMotor_GetState(const dm_motor_t *motor, dm_state_t *state)
{
    uint32_t before;
    uint32_t after;

    if ((motor == NULL) || (state == NULL))
    {
        return false;
    }

    // 若读取期间收到新反馈，则重新复制一次完整快照。
    for (;;)
    {
        before = motor->state_seq;
        if ((before & 1U) != 0U)
        {
            continue;
        }
        *state = motor->state;
        after = motor->state_seq;
        if (before == after)
        {
            break;
        }
    }

    return state->rx_count != 0U;
}
