/**
 ******************************************************************************
 * @file    path_types.h
 * @brief   路径规划与控制模块公共类型定义
 ******************************************************************************
 */
#ifndef PATH_TYPES_H
#define PATH_TYPES_H

#include <stdint.h>

/* 轨迹点:离线生成的整条参考轨迹上的一个采样点 */
typedef struct
{
    float x_m;               /* 世界系 X */
    float y_m;               /* 世界系 Y */
    float yaw_tangent;       /* 切向角(rad,世界系) */
    float s_m;               /* 弧长(m) */
    float kappa;             /* 曲率(1/m,平滑后) */
    float v_ref;             /* 离线速度剖面参考(m/s) */
    float exp_laser_front_m; /* 该点前方激光期望距离(m) */
    float exp_laser_left_m;  /* 该点左方激光期望距离(m) */
} path_point_t;

/* 路点(世界系) */
typedef struct
{
    float x_m;
    float y_m;
} path_waypoint_t;

/* 运行原因(调试/日志用) */
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
    PATH_REASON_STOP_TIMEOUT       /* 全程超时 */
} path_reason_t;

/* 规划器运行状态机 */
typedef enum
{
    PATH_STATE_INIT = 0,      /* 上电,关闭 IMU 自带航向保持等 */
    PATH_STATE_CALIB,         /* 静止采集陀螺零偏 */
    PATH_STATE_WAIT_START,    /* 等待上位机位姿确定起点 */
    PATH_STATE_BUILD,         /* 离线生成 B 样条 + 速度剖面 */
    PATH_STATE_RUN,           /* 在线跟踪 */
    PATH_STATE_ARRIVED,       /* 到达终点 */
    PATH_STATE_STOPPED        /* 故障停止 */
} path_state_t;

/* 调试信息(400ms 一刷,真车可选串口打印) */
typedef struct
{
    path_state_t state;
    path_reason_t reason;
    uint16_t i_near;
    uint16_t i_target;
    float v_ref;              /* 查表速度(m/s) */
    float v_used;             /* 实际使用速度(m/s) */
    float laser_f_m;
    float laser_l_m;
    float exp_laser_l_m;
    float fused_x;
    float fused_y;
    float fused_yaw_rad;
    float cmd_vx_ch;          /* 底盘系 x(向右,m/s) */
    float cmd_vy_ch;          /* 底盘系 y(向前,m/s) */
    float cmd_w;              /* 旋转(rad/s) */
    uint32_t fusion_xy_rejects;
    uint32_t fusion_yaw_rejects;
    uint32_t upper_frames;    /* 参与融合的上位机帧数 */
    uint32_t pc_frames;       /* pc_link 收到的好帧数 */
    uint32_t crc_errors;      /* pc_link 校验错误数 */
    uint32_t run_ms;          /* 运行阶段累计时间 */
} path_debug_t;

#endif /* PATH_TYPES_H */
