/**
 ******************************************************************************
 * @file    path_geometry.h
 * @brief   几何工具:墙(AABB)、距离场、射线投射(Slab 法)、坐标系变换
 *
 * 依赖: path_config.h / path_types.h / math.h
 *
 * 坐标系约定(与 field.yaml frame_convention 一致,并注明与底盘的关系):
 *   世界系:原点场地左下角,+x 向右,+y 向前(远离操作手)。
 *   用户 yaw:上电清零,yaw=0 时车头朝世界 +y,正方向为逆时针。
 *   车体系(body):+x = 车头正前(= world +y 当 yaw=0),
 *                +y = 车体正左(= world -x 当 yaw=0)。
 *   底盘系(chassis,对应 chassis_main.c 的 Chassis_SetVelocity):
 *                +x = 向右平移,+y = 向前,z = 逆时针。
 *
 *   经典 ROS 坐标系里 yaw=0 朝 +x,本工程 yaw=0 朝 +y,差一个 π/2。
 *   本模块的取舍:
 *     - 车体系(前/左)变换使用 R(yaw + π/2),满足原始规格要求;
 *     - 底盘系(右/前)变换直接使用 R(yaw),与 Chassis_SetVelocity
 *       的坐标系完全一致,等价于先做 R(yaw+π/2) 再乘 90° 旋转。
 ******************************************************************************
 */
#ifndef PATH_GEOMETRY_H
#define PATH_GEOMETRY_H

#include "path_config.h"
#include "path_types.h"

#include <stdbool.h>

/* 轴对齐矩形墙 */
typedef struct
{
    float xmin;
    float ymin;
    float xmax;
    float ymax;
} path_wall_t;

/* 静态墙集合 */
typedef struct
{
    const path_wall_t *walls;
    uint8_t count;
} path_gridmap_t;

/**
 * @brief 构建真实墙地图(未膨胀,PATH_WALLS_TABLE)
 */
void PathGridMap_BuildReal(path_gridmap_t *map);

/**
 * @brief 构建膨胀墙地图(每个墙向外扩 PATH_INFLATE_RADIUS_M)
 */
void PathGridMap_BuildInflated(path_gridmap_t *map);

/**
 * @brief 点是否在膨胀墙内(碰撞判定用)
 */
bool PathGridMap_Contains(const path_gridmap_t *map, float x, float y);

/**
 * @brief 点到最近墙的距离(m,点在墙内返回 0)
 */
float PathGridMap_DistTo(const path_gridmap_t *map, float x, float y);

/**
 * @brief 射线投射:返回沿方向 (dx,dy)(单位向量)到最近墙交点的距离;
 *        超过 max_range 或无交点返回 max_range。Slab 法。
 */
float PathGridMap_RayCast(const path_gridmap_t *map,
                          float ox, float oy,
                          float dx, float dy,
                          float max_range);

/**
 * @brief 把落进膨胀墙的点沿最小穿透方向推出墙外
 * @param map  膨胀墙地图
 * @param x,y  输入输出坐标
 * @param step_m 单步步长
 * @param max_iters 最大迭代次数
 * @retval 是否已推出墙外
 */
bool PathGridMap_PushOut(const path_gridmap_t *map, float *x, float *y,
                         float step_m, uint8_t max_iters);

/**
 * @brief 角度归一化到 [-pi, pi)
 */
float PathWrapAngle(float angle_rad);

/**
 * @brief 车体系 -> 世界系:R(yaw + pi/2)
 * @param yaw_user 用户约定 yaw(yaw=0 朝世界 +y)
 */
void PathBodyToWorld(float vx_b, float vy_b, float yaw_user,
                     float *vx_w, float *vy_w);

/**
 * @brief 世界系 -> 车体系:R(-(yaw + pi/2))
 */
void PathWorldToBody(float vx_w, float vy_w, float yaw_user,
                     float *vx_b, float *vy_b);

/**
 * @brief 世界系 -> 底盘系(Chassis_SetVelocity 坐标系):R(yaw)
 * @note  底盘系 x=向右、y=向前,因此直接使用 R(yaw) 而非 R(yaw+pi/2);
 *        与车体系之间差一个 90° 旋转,已在注释中说明。
 */
void PathWorldToChassis(float vx_w, float vy_w, float yaw_user,
                        float *vx_c, float *vy_c);

/**
 * @brief 底盘系 -> 世界系:R(-yaw)
 */
void PathChassisToWorld(float vx_c, float vy_c, float yaw_user,
                        float *vx_w, float *vy_w);

/**
 * @brief 前激光射线:给出世界系射线原点和单位方向
 * @param mount_body_x/y 激光在车体系挂载坐标(PATH_LASER_FRONT_*)
 */
void PathLaserRay(float robot_x, float robot_y, float yaw_user,
                  float mount_body_x, float mount_body_y,
                  float dir_body_x, float dir_body_y,
                  float *ox, float *oy, float *dx, float *dy);

#endif /* PATH_GEOMETRY_H */
