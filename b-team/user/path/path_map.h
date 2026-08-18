#ifndef PATH_MAP_H
#define PATH_MAP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 地图坐标：+X 向右，+Y 向前；正常控制锁 yaw=0，初始定点用实测 yaw。 */
#define PATH_MAP_FIELD_WIDTH_M       3.0000f
#define PATH_MAP_FIELD_HEIGHT_M      6.0000f
#define PATH_MAP_WALL_THICKNESS_M       0.0490f
#define PATH_MAP_ROBOT_LENGTH_M         0.6170f
#define PATH_MAP_ROBOT_WIDTH_M          0.4400f
#define PATH_MAP_INITIAL_CENTER_Y_M     (0.5f * PATH_MAP_ROBOT_LENGTH_M)
/* 上电假定摆位：贴西墙、左光约 10.5 cm。后续加回激光定点后再改。 */
#define PATH_MAP_START_X_M              0.3740f
#define PATH_MAP_START_Y_M              PATH_MAP_INITIAL_CENTER_Y_M
#define PATH_MAP_BOUNDARY_MARGIN_X_M    0.2700f
#define PATH_MAP_BOUNDARY_MARGIN_Y_M    0.3285f
#define PATH_MAP_LOCK_YAW_DEG           0.0f
#define PATH_MAP_WEST_INNER_X_M         0.0490f
#define PATH_MAP_EAST_INNER_X_M         2.9510f
#define PATH_MAP_SOUTH_INNER_Y_M        0.0490f

#define PATH_MAP_WALL_B_X_MIN_M         0.0000f
#define PATH_MAP_WALL_B_X_MAX_M         2.0000f
#define PATH_MAP_WALL_B_SOUTH_Y_M       2.0750f
#define PATH_MAP_WALL_COUNT             7U
#define PATH_MAP_ROUTE_SEGMENT_COUNT 6U
#define PATH_MAP_CLEARANCE_NONE_M    1000.0f

/*
 * 对抗镜像侧（场地左右互换）：
 * 机器人贴东墙起步，车头仍朝 +Y。镜像场地把 3 段内墙整体左右对调
 * （墙 1 贴西墙、墙 B 贴东墙、墙 C 贴西墙），路线也随之左右镜像。
 */
#define PATH_MAP_MIRRORED_START_X_M     2.6260f /* 3.000-0.049-0.175-0.150 */
#define PATH_MAP_MIRRORED_FIRST_WALL_Y_M PATH_MAP_WALL_B_SOUTH_Y_M

typedef enum
{
    PATH_MAP_AXIS_X = 0,
    PATH_MAP_AXIS_Y = 1
} path_map_axis_t;

typedef struct
{
    float x_min_m;
    float y_min_m;
    float x_max_m;
    float y_max_m;
} path_map_wall_t;

typedef struct
{
    path_map_axis_t axis;
    int8_t direction;
    float target_m;
} path_map_route_segment_t;

/** 返回由参考分支 path_config.h 提取的 4 面边界墙和 3 段内墙。 */
const path_map_wall_t *PathMap_GetWalls(uint8_t *count);

/** 返回将参考地图曲线路线改为 yaw=0 人工单轴通行的直线分段。 */
const path_map_route_segment_t *PathMap_GetRoute(uint8_t *count);

/**
 * 选择地图侧别：
 * - false：常规侧，机器人贴西墙起步，左 DT35 扫描西边界锚定 X；
 * - true ：对抗镜像侧，机器人贴东墙起步，左 DT35 无目标，
 *          由 path.c 在前光首次撞墙时自动选择并锚定。
 * 默认 false；选择后 GetWalls/GetRoute/SegmentReached/RayClearance
 * 全部使用对应侧的地图数据。
 */
void PathMap_SetMirrored(bool mirrored);

/** 查询当前是否使用对抗镜像地图。 */
bool PathMap_IsMirrored(void);



/** 判断融合里程计地图位姿是否已经越过当前直线段终点。 */
bool PathMap_SegmentReached(uint8_t segment_index,
                            float map_x_m,
                            float map_y_m);

/**
 * 计算机器人中心沿给定方向到膨胀后静态墙面的剩余直线距离。
 * dir_x/dir_y 可以不归一化；返回值是沿单位方向的米数。
 */
float PathMap_RayClearance(float map_x_m, float map_y_m,
                           float dir_x, float dir_y);

#ifdef __cplusplus
}
#endif

#endif /* PATH_MAP_H */
