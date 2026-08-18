#ifndef PATH_H
#define PATH_H

#include "path_map.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 遥控器六键（LoRa 本机控制帧 payload[4] 低 6 位；bit6/7 是肩键
 * 旋转，行驶阶段强制 z=0，到点交接后放开）。各键语义：
 *   按键 1~2：预留
 *   按键 3：自动行驶启动接口（按下沿触发 Path_AutoStartTrigger；
 *           当前默认上电延时 5 s 自动启动，把
 *           PATH_AUTO_START_ON_BUTTON 置 1 即切换为按键启动）
 *   按键 4~6：预留
 */
#define PATH_REMOTE_BUTTON_1_BIT   (1U << 0U)
#define PATH_REMOTE_BUTTON_2_BIT   (1U << 1U)
#define PATH_REMOTE_BUTTON_3_BIT   (1U << 2U)
#define PATH_REMOTE_BUTTON_4_BIT   (1U << 3U)
#define PATH_REMOTE_BUTTON_5_BIT   (1U << 4U)
#define PATH_REMOTE_BUTTON_6_BIT   (1U << 5U)

/* 语义别名 */
#define PATH_REMOTE_AUTO_BUTTON_BIT   PATH_REMOTE_BUTTON_3_BIT

/*
 * 自动行驶（无遥控器）：上电延时 PATH_AUTO_START_DELAY_MS 后自动
 * 沿去程 6 段行驶到终点，自动停车并鸣笛一声，随后交人工接手。
 * 自动指令与遥控指令走同一条 1 ms 安全管线（激光/地图限速、硬停、
 * 分段推进全部生效）。遥控任意摇杆/按键输入立即取消自动（人工
 * 接管优先）。把 PATH_AUTO_START_ON_BUTTON 置 1 则改为按键 3
 * （或直接调用 Path_AutoStartTrigger()）启动，延时不再生效。
 */
#define PATH_AUTO_START_ON_BUTTON     0
#define PATH_AUTO_START_DELAY_MS      5000U

/* 自动行驶状态（diagnostics.auto_state） */
#define PATH_AUTO_STATE_WAIT          0U  /* 等待启动（延时/按键） */
#define PATH_AUTO_STATE_READY_WAIT    1U  /* 等待 IMU/里程计就绪 */
#define PATH_AUTO_STATE_ANCHOR_RUN    2U  /* 镜像侧未锚定，慢速前出找墙 */
#define PATH_AUTO_STATE_DRIVE         3U  /* 逐段自动行驶 */
#define PATH_AUTO_STATE_DONE          4U  /* 到达终点，已停车鸣笛 */
#define PATH_AUTO_STATE_OFF           5U  /* 人工接管/取消 */

typedef struct
{
    bool initialized;
    bool remote_online;
    bool odometry_valid;
    bool initial_position_valid;
    bool route_complete;
    bool front_laser_online;
    bool left_laser_online;
    bool front_hard_blocked;
    bool left_hard_blocked;
    bool map_speed_limited;
    bool front_speed_limited;
    bool left_speed_limited;
    bool yaw_zero_lock_ready;
    bool map_mirrored;
    /* 自动行驶状态，见 PATH_AUTO_STATE_*。 */
    uint8_t auto_state;
    path_map_axis_t active_axis;
    uint8_t segment_index;
    uint8_t segment_count;
    /* 前初始定位字段为接口兼容保留且恒为 0；初始锚定只等待左光。 */
    uint8_t front_initial_sample_count;
    uint8_t left_initial_sample_count;
    uint16_t front_distance_cm;
    uint16_t left_distance_cm;
    uint32_t last_remote_ms;
    uint32_t initial_position_reject_count;
    float map_x_m;
    float map_y_m;
    float initial_map_x_m;
    float initial_map_y_m;
    float initial_yaw_deg;
    /* 前距离和交点为接口兼容保留且恒为 0。 */
    float front_initial_distance_m;
    float left_initial_distance_m;
    float front_wall_hit_x_m;
    float left_wall_hit_y_m;
    float encoder_velocity_x_mps;
    float encoder_velocity_y_mps;
    float map_clearance_m;
    float front_required_distance_m;
    float left_required_distance_m;
    float front_allowed_speed_mps;
    float left_allowed_speed_mps;
    int16_t raw_vx;
    int16_t raw_vy;
    int16_t output_vx;
    int16_t output_vy;
    int16_t output_z;
} path_diagnostics_t;

/** 在任务启动前初始化人工路径辅助层。 */
void Path_Init(void);

/**
 * 由 LoRa 本机控制帧提交人工指令和六键状态。
 * 函数保存原始摇杆指令，并把参数替换成最近一次 1 ms 安全计算结果；
 * 行驶阶段 z 恒为 0（肩键锁定），到点交接后放开直通。
 */
void Path_SubmitRemoteCommand(int16_t *vx, int16_t *vy, int16_t *z,
                              uint8_t six_buttons, uint32_t now_ms);

/** LoRa 200 ms 超时时立即撤销待执行命令和单轴约束。 */
void Path_NotifyRemoteOffline(uint32_t now_ms);

/**
 * 无小电脑模式下，把其他来源的底盘速度替换为最近的安全人工输出，
 * 避免已有上位机协议旁路本安全层。
 */
void Path_ReplaceNonRemoteCommand(int16_t *vx, int16_t *vy, int16_t *z);

/** 在融合里程计更新后每 1 ms 调用。 */
void Path_Run1ms(uint32_t now_ms);

/** 获取人工分段、激光、里程计和限速状态。 */
bool Path_GetDiagnostics(path_diagnostics_t *diagnostics);

/**
 * 自动行驶启动触发（按键接口）：默认延时模式下也可用它提前启动；
 * PATH_AUTO_START_ON_BUTTON=1 时为唯一启动方式。已接到遥控按键 3
 * 的按下沿，也可由其他模块直接调用。
 */
void Path_AutoStartTrigger(void);

/**
 * 到达终点提示音查询：由 1 ms 底盘任务在 Path_Run1ms 之后调用，
 * 返回 true 表示蜂鸣器输出有效，*level 为本毫秒电平（1 ms 交替
 * ≈500 Hz，与开机提示音一致），结束时最后一拍为低电平。
 */
bool Path_ArrivalBeep(bool *level);

/**
 * 到点交接查询：自动行驶到达终点后恒为 true（一次性测试语义）。
 * 置位后融合里程计不再需要——freertos 底盘任务据此停跑
 * PathLineImu_Run1ms；本层同时关闭地图净空保护与分段功能、放开
 * 肩键，仅保留 DT35 动态激光限速，遥控完全人工驾驶。
 */
bool Path_OdometryReleased(void);

#ifdef __cplusplus
}
#endif

#endif /* PATH_H */
