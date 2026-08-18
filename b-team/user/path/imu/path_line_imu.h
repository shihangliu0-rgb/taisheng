/**
 * @file path_line_imu.h
 * @brief 独立的二维 IMU/四轮 VESC 融合里程计。
 *
 * 坐标约定与 chassis_main 一致：车体系 X 向右、Y 向前；世界系由
 * ImuMain 的右手系 yaw 将车体系旋转得到。所有速度、加速度和位置均
 * 使用 SI 单位。
 */
#ifndef PATH_LINE_IMU_H
#define PATH_LINE_IMU_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 实车底盘参数：152 mm 轮径、1:1 直驱。 */
#define PATH_LINE_IMU_WHEEL_DIAMETER_M 0.152f
#define PATH_LINE_IMU_WHEEL_RADIUS_M   \
    (PATH_LINE_IMU_WHEEL_DIAMETER_M * 0.5f)
#define PATH_LINE_IMU_GEAR_RATIO       1.0f

/* 无有效反馈时返回的年龄。 */
#define PATH_LINE_IMU_AGE_INVALID UINT32_MAX

typedef struct
{
    /* DM-IMU 0x01 帧原始值；用户已确认单位为 m/s^2。 */
    float sensor_accel_x_mps2;
    float sensor_accel_y_mps2;
    float sensor_accel_z_mps2;

    /* 滤波、去零偏后的车体系和世界系水平加速度。 */
    float body_accel_x_mps2;
    float body_accel_y_mps2;
    float world_accel_x_mps2;
    float world_accel_y_mps2;
    float accel_bias_x_mps2;
    float accel_bias_y_mps2;

    /* 仅由 IMU 加速度积分得到的世界系里程计。 */
    float imu_velocity_x_mps;
    float imu_velocity_y_mps;
    float imu_position_x_m;
    float imu_position_y_m;
    float imu_distance_m; /* 速度模长对时间积分得到的累计路程。 */

    /* 四轮 VESC 逆运动学得到的速度、位置和累计路程。 */
    float encoder_body_velocity_x_mps;
    float encoder_body_velocity_y_mps;
    float encoder_world_velocity_x_mps;
    float encoder_world_velocity_y_mps;
    float encoder_position_x_m;
    float encoder_position_y_m;
    float encoder_distance_m;

    /* 最终融合里程计；distance 是累计路程，不是位移模长。 */
    float fused_velocity_x_mps;
    float fused_velocity_y_mps;
    float fused_position_x_m;
    float fused_position_y_m;
    float fused_distance_m;
    float encoder_weight;

    uint32_t last_accel_ms;
    uint32_t accel_age_ms;
    uint32_t encoder_age_ms;
    uint32_t accel_frame_count;
    uint32_t accel_invalid_count;
    uint32_t accel_queue_overflow_count;

    bool accel_bias_ready;
    bool imu_solution_valid;
    bool encoder_solution_valid;
    bool encoder_offline;
    bool zupt_active;
} path_line_imu_data_t;

/** 初始化独立里程计状态；不改动原 IMU 或底盘模块内部状态。 */
void PathLineImu_Init(void);

/**
 * 接收原 IMU 驱动已校验帧尾和长度的 0x01 加速度帧。
 * 该接口只由 user/imu/imu.c 转发调用，不应在中断中调用。
 */
void PathLineImu_OnAccelerationFrame(const uint8_t *frame,
                                     uint8_t length,
                                     uint32_t now_ms);

/** 在底盘任务中每 1 ms 调用一次。 */
void PathLineImu_Run1ms(uint32_t now_ms);

/** 读取完整诊断和里程计输出。 */
bool PathLineImu_GetData(path_line_imu_data_t *data);

/** 清零三套位置坐标和累计路程，保留速度、零偏和权重状态。 */
void PathLineImu_ResetPosition(void);

#ifdef __cplusplus
}
#endif

#endif /* PATH_LINE_IMU_H */
