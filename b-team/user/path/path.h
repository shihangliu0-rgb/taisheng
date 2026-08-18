#ifndef PATH_H
#define PATH_H

#include "path_map.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DT35 全局路径：不用 IMU。上电等激光有数据，用左光判镜像，
 * 再载入该场地在本头文件里的那一套宏。自动速度由「到目标的
 * 剩余厘米」做 PID：远了顶满 150，近了减速。轮速闭环仍在 VESC。
 *
 * 遥控器六键 payload[4] 低 6 位：
 *   按键 3：自动启动接口（PATH_AUTO_START_ON_BUTTON=1 时必用）
 *   其余预留。行驶阶段 z=0。
 */
#define PATH_REMOTE_BUTTON_1_BIT   (1U << 0U)
#define PATH_REMOTE_BUTTON_2_BIT   (1U << 1U)
#define PATH_REMOTE_BUTTON_3_BIT   (1U << 2U)
#define PATH_REMOTE_BUTTON_4_BIT   (1U << 3U)
#define PATH_REMOTE_BUTTON_5_BIT   (1U << 4U)
#define PATH_REMOTE_BUTTON_6_BIT   (1U << 5U)
#define PATH_REMOTE_AUTO_BUTTON_BIT   PATH_REMOTE_BUTTON_3_BIT

#define PATH_AUTO_START_ON_BUTTON     0
#define PATH_AUTO_START_DELAY_MS      5000U

#define PATH_LASER_MIN_CM             5U
#define PATH_FRONT_LASER_MAX_CM       140U
#define PATH_LEFT_LASER_MAX_CM        240U
/* 启动时左光 >= 该值判镜像（贴东墙、左光朝西看空地）。 */
#define PATH_MIRROR_LEFT_CM           100U

#define PATH_DT35_SEGMENT_COUNT       4U
#define PATH_PID_DT_S                 0.001f

/* ========== 常规场地（左光 < PATH_MIRROR_LEFT_CM）========== */
#define PATH_FRONT_ARRIVE_CM          26U
#define PATH_LEFT_NEAR_CM             32U
#define PATH_LEFT_FAR_CM              222U
#define PATH_LASER_STOP_CM            10U
#define PATH_AUTO_FAST_COMMAND        150
#define PATH_AUTO_SETTLE_MS           400U
#define PATH_PID_KP                   4.0f
#define PATH_PID_KI                   1.2f
#define PATH_PID_KD                   0.05f
#define PATH_PID_I_LIMIT              30.0f

/* ========== 镜像场地（左光 >= PATH_MIRROR_LEFT_CM）========== */
#define PATH_MIRROR_FRONT_ARRIVE_CM   26U
#define PATH_MIRROR_LEFT_NEAR_CM      32U
#define PATH_MIRROR_LEFT_FAR_CM       222U
#define PATH_MIRROR_LASER_STOP_CM     10U
#define PATH_MIRROR_FAST_COMMAND      150
#define PATH_MIRROR_SETTLE_MS         400U
#define PATH_MIRROR_PID_KP            4.0f
#define PATH_MIRROR_PID_KI            1.2f
#define PATH_MIRROR_PID_KD            0.05f
#define PATH_MIRROR_PID_I_LIMIT       30.0f

#define PATH_AUTO_STATE_WAIT          0U
#define PATH_AUTO_STATE_READY_WAIT    1U
#define PATH_AUTO_STATE_DRIVE         3U
#define PATH_AUTO_STATE_DONE          4U
#define PATH_AUTO_STATE_OFF           5U

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
    uint8_t auto_state;
    path_map_axis_t active_axis;
    uint8_t segment_index;
    uint8_t segment_count;
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
    float pid_error_cm;
    float pid_output;
    int16_t raw_vx;
    int16_t raw_vy;
    int16_t output_vx;
    int16_t output_vy;
    int16_t output_z;
} path_diagnostics_t;

void Path_Init(void);
void Path_SubmitRemoteCommand(int16_t *vx, int16_t *vy, int16_t *z,
                              uint8_t six_buttons, uint32_t now_ms);
void Path_NotifyRemoteOffline(uint32_t now_ms);
void Path_ReplaceNonRemoteCommand(int16_t *vx, int16_t *vy, int16_t *z);
void Path_Run1ms(uint32_t now_ms);
bool Path_GetDiagnostics(path_diagnostics_t *diagnostics);
void Path_AutoStartTrigger(void);
bool Path_ArrivalBeep(bool *level);
bool Path_OdometryReleased(void);

#ifdef __cplusplus
}
#endif

#endif /* PATH_H */
