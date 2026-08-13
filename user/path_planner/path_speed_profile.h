/**
 ******************************************************************************
 * @file    path_speed_profile.h
 * @brief   离线速度剖面:曲率限速 + 双向扫描,及期望激光查表(对应 speed_profile.py)
 *
 * 依赖: path_config.h / path_types.h / path_geometry.h / math.h
 * 关键算法:
 *   1. 曲率限速 v <= sqrt(a_lat / |kappa|),与 v_max 取小;
 *   2. 前向扫描(加速能力)  v_{i+1}^2 <= v_i^2 + 2*a_accel*ds;
 *   3. 反向扫描(刹车能力)  v_i^2 <= v_{i+1}^2 + 2*a_brake*ds;
 *   4. 每点做前/左激光期望距离射线投射(yaw 锁定 0,车头朝 +y)。
 ******************************************************************************
 */
#ifndef PATH_SPEED_PROFILE_H
#define PATH_SPEED_PROFILE_H

#include "path_geometry.h"
#include "path_types.h"
#include "stm32h7xx_hal.h"

/**
 * @brief 为采样轨迹生成速度剖面与期望激光表
 * @param points       B 样条采样点(输入 x/y,输出 v_ref/exp_laser_*)
 * @param count        点数
 * @param real_map     真实墙地图(算期望激光)
 * @retval true 成功
 */
bool PathSpeedProfile_Build(path_point_t *points, uint16_t count,
                            const path_gridmap_t *real_map);

/**
 * @brief 前向窗口搜索离 (x,y) 最近的轨迹点索引
 * @param hint 上次索引,从此处起只向前搜索(防止走回头)
 * @retval 最近点索引
 */
uint16_t PathSpeedProfile_Nearest(const path_point_t *points, uint16_t count,
                                  float x, float y, uint16_t hint);

/**
 * @brief 通过串口输出 CSV(表头 + 每行 s,x,y,kappa,v_ref,exp_f,exp_l),
 *        可在主机侧保存为 speed_profile.csv / expected_laser.csv 人工检查
 */
void PathSpeedProfile_DumpCsv(const path_point_t *points, uint16_t count,
                              UART_HandleTypeDef *uart);

#endif /* PATH_SPEED_PROFILE_H */
