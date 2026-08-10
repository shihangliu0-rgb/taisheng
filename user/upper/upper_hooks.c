/**
 * @file upper_hooks.c
 * @brief 上位机协议钩子: 把 flash_params(调参/保存) + DT35(激光) + IMU/融合(遥测)
 *        接进 upper 模块。
 *        覆盖 upper_protocol.c 里的 __weak 钩子(只在 UPPER_DEBUG 开启时编译)。
 */

#include "upper_protocol.h"

#ifdef UPPER_DEBUG

#include "flash_params.h"
#include "dt35_link.h"
#include "imu.h"
#include "imu_fusion.h"

#include <string.h>

/* ============================ PID 调参(上位机 PID_READ/WRITE/SAVE) ============================ */

void upper_on_pid_write(uint8_t pid_id, const float *param)
{
    /* 写入 RAM(带范围限制)，立即生效 */
    (void)Config_SetPID(pid_id, param);
}

void upper_get_pid(uint8_t pid_id, float *out_param)
{
    if (!Config_GetPID(pid_id, out_param))
    {
        memset(out_param, 0, 5U * sizeof(float));
    }
}

void upper_on_pid_save(uint8_t pid_id)
{
    (void)pid_id;
    /* 把 RAM 配置写入 W25Q128(双备份) */
    (void)FlashParams_Save();
}

/* ============================ 通用参数(PARAM_WRITE) ============================ */

void upper_on_param(uint16_t param_id, float value, uint8_t is_write)
{
    if (is_write != 0U)
    {
        /* 上位机下发参数值 → 写入 RAM(带范围限制) */
        Config_SetParam((Param_Id_t)param_id, value);
    }
    /* 读取走 STREAM_DATA 通道(见 upper_read_channel) */
}

/* ============================ 数据流(上位机订阅通道) ============================ */
/* 把 IMU/融合/激光/配置参数都映射到通道 ID，上位机勾选即可实时看曲线。
 *
 * 0x03 imu_yaw         Imu_GetYaw()            deg
 * 0x12 gyro_z          Imu_GetGyroZ()          deg/s
 * 0x40 fusion_pos_x    ImuFusion_GetPosX()     m
 * 0x41 fusion_pos_y    ImuFusion_GetPosY()     m
 * 0x44 fusion_vel_x    ImuFusion_GetVelX()     m/s
 * 0x45 fusion_vel_y    ImuFusion_GetVelY()     m/s
 * 0x70 dt35_front      dt35_distance_40_cm     mm(原始 cm×10)
 * 0x71 dt35_left       dt35_distance_41_cm     mm
 * 0x72 fusion_weight   ImuFusion_GetEncoderWeight()  [0,0.5]
 * 0x73 fusion_age      ImuFusion_GetEncoderAgeMs()   ms
 */

float upper_read_channel(uint8_t channel_id)
{
    switch (channel_id)
    {
    /* IMU */
    case 0x03U: return Imu_GetYaw();
    case 0x12U: return Imu_GetGyroZ();

    /* 融合速度/位置 */
    case 0x40U: return ImuFusion_GetPosX();
    case 0x41U: return ImuFusion_GetPosY();
    case 0x44U: return ImuFusion_GetVelX();
    case 0x45U: return ImuFusion_GetVelY();

    /* DT35 激光(单位 cm，上位机按 cm 显示) */
    case 0x70U: return (float)dt35_distance_40_cm;
    case 0x71U: return (float)dt35_distance_41_cm;

    /* 融合诊断 */
    case 0x72U: return ImuFusion_GetEncoderWeight();
    case 0x73U: return (float)ImuFusion_GetEncoderAgeMs();

    default: return 0.0f;
    }
}

#endif /* UPPER_DEBUG */
