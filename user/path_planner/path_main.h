/* path_main.h - 跑曲线:类型与接口(说明见 README.md) */
#ifndef PATH_MAIN_H
#define PATH_MAIN_H

#include "path_config.h"

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

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

/* 路点(世界系) */
typedef struct
{
    float x_m;
    float y_m;
} path_waypoint_t;

/* 参考轨迹采样点(离线生成) */
typedef struct
{
    float x_m;
    float y_m;
    float yaw_tangent;
    float s_m;
    float kappa;
    float v_ref;
    float exp_laser_front_m;
    float exp_laser_left_m;
} path_point_t;

/* 运行状态机 */
typedef enum
{
    PATH_STATE_INIT = 0,      /* 上电,关闭 IMU 自带航向保持 */
    PATH_STATE_CALIB,         /* 静止采集陀螺零偏 */
    PATH_STATE_WAIT_START,    /* 等待上位机位姿确定起点 */
    PATH_STATE_BUILD,         /* 离线生成 B 样条 + 速度剖面 */
    PATH_STATE_RUN,           /* 在线跟踪 */
    PATH_STATE_ARRIVED,       /* 到达终点 */
    PATH_STATE_STOPPED        /* 故障停止 */
} path_state_t;

/* 运行原因(调试/日志) */
typedef enum
{
    PATH_REASON_BOOT = 0,
    PATH_REASON_CALIB,
    PATH_REASON_WAIT_START,
    PATH_REASON_RUN,
    PATH_REASON_LASER_SLOW,        /* 前激光兜底降速 */
    PATH_REASON_ARRIVED,
    PATH_REASON_STOP_LASER_FRONT,  /* 前激光 < 12cm 强制停 */
    PATH_REASON_STOP_UPPER_LOST,   /* 上位机位姿丢失 > 500ms */
    PATH_REASON_STOP_IMU_LOST,     /* IMU 离线 */
    PATH_REASON_STOP_LASER_LOST,   /* 前激光离线 */
    PATH_REASON_STOP_BUILD,        /* 离线轨迹生成失败 */
    PATH_REASON_STOP_NUMERIC,      /* 数值异常(NaN/Inf)防护停车 */
    PATH_REASON_STOP_MOTOR_LOST,   /* 任一底盘电机离线 */
    PATH_REASON_STOP_HEADING,      /* 起步朝向超出 ±30 度硬约束 */
    PATH_REASON_STOP_TIMEOUT       /* 全程超时 */
} path_reason_t;

/* 调试信息(每 400ms 采样一次) */
typedef struct
{
    path_state_t state;
    path_reason_t reason;
    uint16_t i_near;
    uint16_t i_target;
    float v_ref;
    float v_used;
    float laser_f_m;
    float laser_l_m;
    float exp_laser_l_m;
    float fused_x;
    float fused_y;
    float fused_yaw_rad;
    float cmd_vx_ch;
    float cmd_vy_ch;
    float cmd_w;
    uint32_t fusion_xy_rejects;
    uint32_t fusion_yaw_rejects;
    uint32_t upper_frames;
    uint32_t pc_frames;
    uint32_t crc_errors;
    uint32_t run_ms;
} path_debug_t;

void PathRunner_Init(void);
void PathRunner_Run(void);
void PathRunner_GetDebug(path_debug_t *debug);
const path_point_t *PathRunner_GetTrajectory(uint16_t *count);

/* 指令仲裁:规划器 RUN 期间返回 true */
bool PathPlanner_OwnsChassis(void);

void PathGridMap_BuildReal(path_gridmap_t *map);
void PathGridMap_BuildInflated(path_gridmap_t *map);
void PathGridMap_BuildHardInflated(path_gridmap_t *map);   /* 验收用硬膨胀 */
float PathGridMap_RayCast(const path_gridmap_t *map, float ox, float oy,
                          float dx, float dy, float max_range);
float PathGridMap_DistTo(const path_gridmap_t *map, float x, float y);
void PathChassisToWorld(float vx_c, float vy_c, float yaw_user,
                        float *vx_w, float *vy_w);
void PathLaserRay(float robot_x, float robot_y, float yaw_user,
                  float mount_body_x, float mount_body_y,
                  float dir_body_x, float dir_body_y,
                  float *ox, float *oy, float *dx, float *dy);
float PathWrapAngle(float angle_rad);

void PathSpeedProfile_DumpCsv(const path_point_t *points, uint16_t count,
                              UART_HandleTypeDef *uart);

#if PATH_DEBUG && defined(PATH_DEBUG_UART_HANDLE)
/* 每 400ms 向调试串口输出一行运行状态 */
void PathRunner_DebugDump(UART_HandleTypeDef *uart);
#endif

#endif /* PATH_MAIN_H */
