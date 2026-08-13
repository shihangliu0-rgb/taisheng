/**
 ******************************************************************************
 * @file    path_pure_pursuit.h
 * @brief   纯追踪前视目标(对应 pure_pursuit.py)
 *
 * 依赖: path_config.h / path_types.h
 * 关键算法: Ld = lookahead_min + lookahead_k * v_ref 自适应前视;
 *           从最近点起沿轨迹方向找第一个距机器人 >= Ld 的点。
 ******************************************************************************
 */
#ifndef PATH_PURE_PURSUIT_H
#define PATH_PURE_PURSUIT_H

#include "path_types.h"

/**
 * @brief 在轨迹上寻找前视目标点
 * @param points    参考轨迹
 * @param count     轨迹点数
 * @param x, y      当前位姿(融合)
 * @param v_ref     当前速度参考(m/s),决定前视距离
 * @param kappa     最近点曲率(1/m):急弯处缩短前视,防止抄近道
 * @param i_near    最近点索引(由 PathSpeedProfile_Nearest 得到)
 * @param i_target  输出:目标点索引
 * @param tx, ty    输出:目标点坐标
 */
void PathPurePursuit_Find(const path_point_t *points, uint16_t count,
                          float x, float y, float v_ref, float kappa,
                          uint16_t i_near, uint16_t *i_target,
                          float *tx, float *ty);

#endif /* PATH_PURE_PURSUIT_H */
