/**
 ******************************************************************************
 * @file    path_spline.h
 * @brief   三次 B 样条平滑 + 弧长/曲率 + 离墙外推(对应 bspline_smoother.py)
 *
 * 依赖: path_config.h / path_types.h / path_geometry.h
 * 关键算法: clamped 弦长参数化 B 样条(de Boor 递推),插值端点;
 *          曲率 = dθ/ds(中心差分 + 3 点平滑)。
 ******************************************************************************
 */
#ifndef PATH_SPLINE_H
#define PATH_SPLINE_H

#include "path_geometry.h"
#include "path_types.h"

/**
 * @brief 由路点生成 B 样条采样轨迹
 * @param waypoints  路点数组(起点已按需要覆盖)
 * @param n_wp       路点数(>= 4)
 * @param out        输出采样点数组(x/y/yaw_tangent/s_m/kappa 填充)
 * @param max_out    输出数组容量(PATH_SPLINE_SAMPLES)
 * @param out_count  实际输出点数
 * @retval true 成功
 */
bool PathSpline_Build(const path_waypoint_t *waypoints, uint8_t n_wp,
                      path_point_t *out, uint16_t max_out,
                      uint16_t *out_count);

/**
 * @brief 把落入膨胀墙的采样点沿最小穿透方向外推(对应 push_away_from_walls)
 * @retval 成功外推出墙外的点数
 */
uint16_t PathSpline_PushAwayFromWalls(path_point_t *points, uint16_t count,
                                      const path_gridmap_t *inflated_map);

/**
 * @brief 推离 + 平滑迭代,并重新计算切线/弧长/曲率(必须在推离后调用)
 * @note  单次点状外推会把拐角撕出跳变,交替"推离 -> 移动平均"PATH_PUSH_SMOOTH_ROUNDS
 *        轮,使路径收敛为绕膨胀墙的圆角,再重算 s/kappa
 */
void PathSpline_Finalize(path_point_t *points, uint16_t count,
                         const path_gridmap_t *inflated_map);

#endif /* PATH_SPLINE_H */
