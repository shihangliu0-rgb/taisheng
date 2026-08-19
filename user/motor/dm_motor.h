#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

/* MIT 指令校验使用的 J4310 物理范围。 */
#define DM_J4310_P_MAX 12.5f
#define DM_J4310_V_MAX 30.0f
#define DM_J4310_T_MAX 10.0f
#define DM_MIT_KP_MAX  500.0f
#define DM_MIT_KD_MAX  5.0f

typedef enum
{
    DM_OK = 0,
    DM_BAD_ARG,
    DM_BUSY,
    DM_IO_ERROR
} dm_result_t;

typedef enum
{
    DM_FAULT_NONE = 0x0,
    DM_FAULT_OVER_VOLTAGE = 0x8,
    DM_FAULT_UNDER_VOLTAGE = 0x9,
    DM_FAULT_OVER_CURRENT = 0xA,
    DM_FAULT_MOS_OVER_TEMP = 0xB,
    DM_FAULT_COIL_OVER_TEMP = 0xC,
    DM_FAULT_COMM_LOST = 0xD,
    DM_FAULT_OVERLOAD = 0xE
} dm_fault_t;

typedef enum
{
    DM_STATE_DISABLED = 0x0,
    DM_STATE_ENABLED = 0x1
} dm_operating_state_t;

typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float kp;
    float kd;
} dm_mit_cmd_t;

typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
} dm_limits_t;

typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    uint8_t mos_temp_c;
    uint8_t rotor_temp_c;
    uint8_t operating_state;
    dm_fault_t fault;
    uint32_t rx_tick_ms;
    uint32_t rx_count;
} dm_state_t;

typedef struct
{
    uint16_t id;
    uint8_t length;
    uint8_t data[8];
} dm_frame_t;

typedef struct
{
    uint16_t tx_id;
    uint16_t master_id;
    uint8_t feedback_id;
    dm_limits_t limits;
    dm_state_t state;
} dm_motor_t;

/** @brief 初始化一台 DM 电机及其反馈地址。 */
dm_result_t DmMotor_Init(dm_motor_t *motor, uint16_t tx_id, uint16_t master_id,
                         uint8_t feedback_id);
/** @brief 设置 MIT 指令的物理限幅。 */
dm_result_t DmMotor_SetLimits(dm_motor_t *motor, float p_max, float v_max,
                              float t_max);
/** @brief 构建使能帧。 */
dm_result_t DmMotor_BuildEnable(const dm_motor_t *motor, dm_frame_t *frame);
/** @brief 构建失能帧。 */
dm_result_t DmMotor_BuildDisable(const dm_motor_t *motor, dm_frame_t *frame);
/** @brief 构建机械零点写入帧，不用于普通位置回零 */
dm_result_t DmMotor_BuildZero(const dm_motor_t *motor, dm_frame_t *frame);
/** @brief 将 MIT 控制量编码为一帧标准 CAN 数据。 */
dm_result_t DmMotor_BuildMit(const dm_motor_t *motor, const dm_mit_cmd_t *cmd,
                             dm_frame_t *frame);
/** @brief 解析一帧 DM 反馈并更新状态。 */
bool DmMotor_Parse(dm_motor_t *motor, uint16_t id, const uint8_t *data,
                   uint8_t length, uint32_t tick_ms);
/** @brief 复制最近一次 DM 反馈。 */
bool DmMotor_GetState(const dm_motor_t *motor, dm_state_t *state);
#endif
