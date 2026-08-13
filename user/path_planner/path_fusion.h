/**
 ******************************************************************************
 * @file    path_fusion.h
 * @brief   IMU 陀螺 + 上位机绝对位姿互补融合(对应 imu_fusion.py,非 EKF)
 *
 * 依赖: path_config.h
 * 关键算法:
 *   - predict:yaw 由 gyro_z 高频积分(100Hz 数据,5ms 控制周期);
 *   - update_upper:xy 先过 3 点中值滤波,再与融合值比较,跳变 >30cm
 *     整帧拒绝;yaw 与积分预测差 >20° 时只拒绝 yaw 分量;
 *   - xy 直接覆盖,yaw 用 0.15 增益低通拉回;
 *   - 超过 500ms 无上位机数据判定 lost。
 ******************************************************************************
 */
#ifndef PATH_FUSION_H
#define PATH_FUSION_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 复位融合器
 */
void PathFusion_Init(void);

/**
 * @brief 静止零偏标定:采集 N 帧陀螺求均值,作为附加零偏
 * @param gyro_z_deg_s 当前陀螺读数(deg/s)
 * @retval true 标定完成(已可开始 predict)
 */
bool PathFusion_CalibrateSample(float gyro_z_deg_s);

/**
 * @brief 陀螺积分预测 yaw(每次控制周期调用)
 * @param gyro_z_deg_s 陀螺读数(deg/s)
 * @param dt_s 周期(秒)
 */
void PathFusion_Predict(float gyro_z_deg_s, float dt_s);

/**
 * @brief 融合上位机绝对位姿(50Hz 帧到达时调用)
 * @param x_m, y_m 上位机场地坐标(m)
 * @param yaw_rad 上位机 yaw(rad,用户约定:0 朝 +y)
 * @param now_ms  当前系统时间
 * @retval true 本次数据通过门限并参与融合
 */
bool PathFusion_UpdateUpper(float x_m, float y_m, float yaw_rad,
                            uint32_t now_ms);

/**
 * @brief 上位机是否丢失(超过 PATH_FUSION_UPPER_TIMEOUT_MS 无数据)
 */
bool PathFusion_IsUpperLost(uint32_t now_ms);

/**
 * @brief 读取融合位姿
 */
void PathFusion_Get(float *x_m, float *y_m, float *yaw_rad);

/**
 * @brief 读取统计:xy/yaw 拒绝次数、累计上位机帧数
 */
void PathFusion_GetStats(uint32_t *xy_rejects, uint32_t *yaw_rejects,
                         uint32_t *upper_frames);

#endif /* PATH_FUSION_H */
