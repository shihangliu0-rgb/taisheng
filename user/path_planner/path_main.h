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
    PATH_STATE_INIT = 0,        /* 上电,关闭 IMU 自带航向保持 */
    PATH_STATE_CALIB,           /* 静止采集陀螺零偏 */
    PATH_STATE_WAIT_START,      /* 等待位姿 + 朝向确认(每次布防重新确认) */
    PATH_STATE_BUILD,           /* 离线生成 B 样条 + 速度剖面 + 验收 */
    PATH_STATE_RUN,             /* 在线跟踪 */
    PATH_STATE_ARRIVED,         /* 到达终点(锁存) */
    PATH_STATE_SAFE_STOP,       /* 瞬态故障安全停车(可自动恢复) */
    PATH_STATE_RECOVER_CHECK,   /* 恢复前健康校验 + 朝向复查 */
    PATH_STATE_FAULT_LATCH,     /* 确定性/永久故障,人工复位 */
    PATH_STATE_MANUAL_OVERRIDE, /* 人工接管,自主挂起 */
    PATH_STATE_SELF_CHECK       /* 复位后自检(通过才重新标定/布防) */
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
    PATH_REASON_STOP_LASER_FRONT,  /* 6  前激光障碍(瞬态) */
    PATH_REASON_STOP_UPPER_LOST,   /* 7  位姿丢失(瞬态) */
    PATH_REASON_STOP_IMU_LOST,     /* 8  IMU 故障(瞬态,窗口内可恢复) */
    PATH_REASON_STOP_LASER_LOST,   /* 9  前激光离线(瞬态) */
    PATH_REASON_STOP_BUILD,        /* 10 建轨迹失败-瞬态数据错误(可重 BUIL D) */
    PATH_REASON_STOP_NUMERIC,      /* 11 数值异常(NaN/Inf),永久 */
    PATH_REASON_STOP_MOTOR_LOST,   /* 12 电机离线(瞬态) */
    PATH_REASON_STOP_HEADING,      /* 13 朝向超限(恢复期复查) */
    PATH_REASON_STOP_TIMEOUT,      /* 14 单次 RUN 超时(瞬态) */
    PATH_REASON_STOP_BUILD_PARAM,  /* 15 建轨迹失败-参数错误,永久(人工复位) */
    PATH_REASON_STOP_ESTOP,        /* 16 遥控急停,永久(人工复位) */
    PATH_REASON_STOP_RECOVER_FAIL, /* 17 恢复预算耗尽,永久(人工复位) */
    PATH_REASON_STOP_MANUAL_LINK,  /* 18 人工链路丢失(瞬态,等待人工) */
    PATH_REASON_MANUAL_OVERRIDE,   /* 19 人工接管(状态码) */
    PATH_REASON_STOP_RECOVER_LOOP, /* 20 同因同位自动循环,升级锁存 */
    PATH_REASON_STOP_SELFCHECK,    /* 21 复位自检失败/超时 */
    PATH_REASON_SELFCHECK          /* 22 复位自检中(状态码) */
} path_reason_t;

/* 故障上下文:state 轴与 fault_code 轴正交,避免状态爆炸 */
typedef struct
{
    path_reason_t code;       /* fault_code:最近一次故障原因 */
    float x_m;                /* fault_pose:故障时融合位姿 */
    float y_m;
    uint32_t timestamp_ms;    /* fault_timestamp */
} path_fault_ctx_t;

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
    path_reason_t fault_code;      /* 最近一次故障码 */
    float fault_x_m;               /* 最近一次故障位姿 */
    float fault_y_m;
    uint32_t fault_timestamp_ms;   /* 最近一次故障时间 */
} path_debug_t;

void PathRunner_Init(void);
void PathRunner_Run(void);
void PathRunner_GetDebug(path_debug_t *debug);
const path_point_t *PathRunner_GetTrajectory(uint16_t *count);

/* 控制仲裁优先级:急停 > 人工 > 自主 */
typedef enum
{
    PATH_ARB_ESTOP = 0,
    PATH_ARB_MANUAL = 1,
    PATH_ARB_AUTONOMOUS = 2
} path_arb_t;

/* commTask 每 1ms 调用:执行急停/人工/自主底盘指令仲裁(唯一写底盘入口) */
void PathRunner_Arbitrate(void);

/* 当前控制源(急停/人工/自主) */
path_arb_t PathPlanner_Arbiter(void);

/* 兼容旧接口:仅"自主且处于 RUN"时视为规划器独占底盘 */
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
