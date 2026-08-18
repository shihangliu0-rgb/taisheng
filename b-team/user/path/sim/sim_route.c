/**
 * @file sim_route.c
 * @brief 全路线物理闭环仿真：编译工程真实 path.c / path_map.c /
 *        path_localization.c / path_safety.c / path_line_imu.c，
 *        验证机器人能否按 user/path 走完去程 6 段。
 *
 * 禁止把真值位姿写回控制层。path.c 看到的地图坐标只来自真实融合
 * 里程计 PathLineImu（四轮 RPM + IMU 加速度帧 + yaw），与实车
 * FreeRTOS 底盘任务同一条数据链。
 *
 * 控制侧数据全部取自本工程源码：
 *  - 遥控映射/帧率/超时:  lora_link.c、遥控器对接说明
 *  - 底盘混控/限幅/斜坡:  chassis_main.c
 *  - 轮速换算/融合:       path_line_imu.c（本文件直接链接，不复刻）
 *  - IMU yaw 闭环:        imu_main.c
 *  - DT35 子板:           sensor/user/dt35.c（50 ms、整 cm、双量程）
 *  - 传感器安装:          path_localization.h（面装：前 0.3085 / 左 0.220）
 *  - 场地/机器人几何:     path_map.h / path_map.c
 *
 * 仿真自身仅有两个非工程来源的物理假设：
 *  1. 轮速跟踪按 PATH_BRAKE_DECELERATION_MPS2 = 2.0 m/s^2 限幅；
 *  2. 旋转等效半径 (长+宽)/2 = 0.5285 m。
 *
 * 传感器流全部由物理积分派生，不注入虚拟融合坐标：
 *  - VESC actual_rpm = 植物轮速，经 Chassis_GetStatus 被真实里程计读取；
 *  - IMU 0x01 加速度帧 = 车体速度差分 + 可标定常值零偏；
 *  - IMU yaw/gyro = 植物姿态积分；
 *  - DT35 = 真值位姿射线 + 子板量化。
 */
#include "path.h"

#include "chassis_main.h"
#include "dt35_pnp_link.h"
#include "imu_main.h"
#include "path_line_imu.h"
#include "path_localization.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* 工程常量                                                            */
/* ------------------------------------------------------------------ */
#define SIM_PI                    3.14159265358979323846
#define SIM_RPM_TO_MPS            (2.0 * SIM_PI * 0.076 / 60.0)
#define SIM_REMOTE_FAST           150
#define SIM_REMOTE_FINE           75
#define SIM_REMOTE_PERIOD_MS      50
#define SIM_CHASSIS_MAX_RPM       4000
#define SIM_START_RAMP_MS         300
#define SIM_STOP_RAMP_MS          200
#define SIM_ROTATION_SCALE        (3.5 + 3.30)
#define SIM_DT35_PERIOD_MS        50
#define SIM_DT35_MIN_CM           5
#define SIM_DT35_FRONT_MAX_CM     140
#define SIM_DT35_LEFT_MAX_CM      240
#define SIM_FRONT_OFFSET_M        0.3085
#define SIM_LEFT_OFFSET_M         0.2200
#define SIM_ROBOT_LEN_M           0.617
#define SIM_ROBOT_WID_M           0.440
#define SIM_HALF_LEN_M            (0.5 * SIM_ROBOT_LEN_M)
#define SIM_HALF_WID_M            (0.5 * SIM_ROBOT_WID_M)
#define SIM_VESC_STATUS_PERIOD_MS 10
#define SIM_ACCEL_PERIOD_MS       5
#define SIM_IMU_READY_MS          5350
#define SIM_YAW_KP_MOVE           0.4
#define SIM_YAW_OUTMAX_MOVE       800.0
#define SIM_YAW_KP_STOP           0.8
#define SIM_YAW_OUTMAX_STOP       100.0
#define SIM_YAW_GYRO_K            5.0
#define SIM_YAW_DEADZONE_DEG      0.17
#define SIM_YAW_PERIOD_MS         5
#define SIM_YAW_LINEAR_THRESHOLD  2
#define SIM_GYRO_KALMAN_Q         0.1
#define SIM_GYRO_KALMAN_R         2.0
#define SIM_ACCEL_LIMIT_MPS2      2.0
#define SIM_LXY_M                 ((SIM_ROBOT_LEN_M + SIM_ROBOT_WID_M) / 2.0)
#define SIM_ACCEL_BIAS_X          0.030
#define SIM_ACCEL_BIAS_Y          (-0.020)
#define SIM_GRAVITY_Z             9.80665

#define SIM_MAX_TIME_MS           240000
#define SIM_SEGMENT_TIMEOUT_MS    90000
#define SIM_LOG_PERIOD_MS         10

/* ------------------------------------------------------------------ */
volatile dt35_link_t dt35_link[SENSOR_LINK_COUNT];

typedef struct
{
    double x_min, y_min, x_max, y_max;
    bool solid;
} sim_wall_t;

static sim_wall_t sim_walls[16];
static int sim_wall_count;

typedef struct
{
    double px, py;
    double yaw_deg;
    double wheel_rpm[4];
    int16_t tgt_vx, tgt_vy, tgt_z;
    bool emergency;
    int state;
    uint32_t ramp_begin_ms;
    double motor_target_rpm[4];
    double stop_start_rpm[4];
    uint32_t vesc_last_rx_ms;
    bool imu_ready;
    double imu_target_yaw;
    bool imu_target_valid;
    bool yaw_hold_enabled;
    int yaw_mode;
    uint32_t yaw_last_ms;
    bool yaw_time_valid;
    double gyro_est, gyro_cov;
    int16_t omega_output;
    double prev_body_vx;
    double prev_body_vy;
    double body_ax;
    double body_ay;
    bool have_prev_body;
    double max_contact_speed;
    int contact_events;
    double first_contact_speed;
    uint32_t first_contact_ms;
    char first_contact_wall[32];
    uint32_t wall_contact_ms[16];
    double slip_distance;
    double max_odom_err_m;
    uint32_t max_odom_err_ms;
} sim_plant_t;

static sim_plant_t plant;
static double sim_gyro_deg_s;

/* ------------------------------------------------------------------ */
HAL_StatusTypeDef Chassis_SetVelocity(int16_t vx, int16_t vy, int16_t z)
{
    plant.tgt_vx = vx;
    plant.tgt_vy = vy;
    plant.tgt_z = z;
    if ((vx != 0) || (vy != 0) || (z != 0))
    {
        plant.emergency = false;
    }
    return HAL_OK;
}

void Chassis_StopAll(void)
{
    plant.tgt_vx = 0;
    plant.tgt_vy = 0;
    plant.tgt_z = 0;
    plant.emergency = true;
}

bool Chassis_GetStatus(chassis_wheel_t wheel, vesc_motor_status_t *status)
{
    if ((status == NULL) || (wheel >= CHASSIS_WHEEL_COUNT))
    {
        return false;
    }
    memset(status, 0, sizeof(*status));
    status->actual_rpm = (int32_t)lround(plant.wheel_rpm[wheel]);
    status->target_rpm = (int32_t)lround(plant.motor_target_rpm[wheel]);
    status->online = true;
    status->last_rx_ms = plant.vesc_last_rx_ms;
    return true;
}

bool ImuMain_GetData(imu_data_t *data)
{
    if (data == NULL)
    {
        return false;
    }
    memset(data, 0, sizeof(*data));
    data->yaw_deg = (float)plant.yaw_deg;
    data->gyro_z_deg_s = (float)sim_gyro_deg_s;
    data->state = plant.imu_ready ? IMU_STATE_READY : IMU_STATE_CALIBRATING;
    data->target_yaw_deg = (float)plant.imu_target_yaw;
    data->yaw_valid = plant.imu_ready;
    data->gyro_valid = plant.imu_ready;
    data->online = plant.imu_ready;
    data->yaw_hold_enabled = plant.yaw_hold_enabled;
    data->last_rx_ms = plant.imu_ready ? plant.yaw_last_ms : 0U;
    return true;
}

void ImuMain_EnableYawHold(bool enabled)
{
    plant.yaw_hold_enabled = enabled;
}

HAL_StatusTypeDef ImuMain_SetTargetYaw(float target_yaw_deg)
{
    plant.imu_target_yaw = target_yaw_deg;
    plant.imu_target_valid = true;
    return HAL_OK;
}

/* ------------------------------------------------------------------ */
static double sim_normalize_angle(double a)
{
    while (a > 180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}

static double sim_filter_gyro(double g)
{
    double gain;
    plant.gyro_cov += SIM_GYRO_KALMAN_Q;
    gain = plant.gyro_cov / (plant.gyro_cov + SIM_GYRO_KALMAN_R);
    plant.gyro_est += gain * (g - plant.gyro_est);
    plant.gyro_cov *= 1.0 - gain;
    return round(plant.gyro_est * 10.0) / 10.0;
}

static int16_t sim_calc_omega(int16_t vx, int16_t vy, int16_t omega,
                              uint32_t now_ms)
{
    bool stopped;
    int hold_mode;
    double err, out, kp, out_max;

    if (!plant.imu_ready || !plant.yaw_hold_enabled)
    {
        plant.yaw_mode = 0;
        plant.omega_output = omega;
        return omega;
    }
    if ((omega > 3) || (omega < -3))
    {
        plant.imu_target_yaw = plant.yaw_deg;
        plant.imu_target_valid = true;
        plant.yaw_mode = 3;
        plant.omega_output = omega;
        return omega;
    }
    stopped = (abs(vx) <= SIM_YAW_LINEAR_THRESHOLD) &&
              (abs(vy) <= SIM_YAW_LINEAR_THRESHOLD);
    hold_mode = stopped ? 2 : 1;
    if (!plant.imu_target_valid)
    {
        plant.imu_target_yaw = plant.yaw_deg;
        plant.imu_target_valid = true;
    }
    if (plant.yaw_mode != hold_mode)
    {
        plant.gyro_est = 0.0;
        plant.gyro_cov = 0.0;
        plant.yaw_time_valid = false;
        plant.yaw_mode = hold_mode;
    }
    if (plant.yaw_time_valid &&
        ((uint32_t)(now_ms - plant.yaw_last_ms) < SIM_YAW_PERIOD_MS))
    {
        return plant.omega_output;
    }
    plant.yaw_last_ms = now_ms;
    plant.yaw_time_valid = true;

    err = sim_normalize_angle(plant.imu_target_yaw - plant.yaw_deg);
    if (fabs(err) <= SIM_YAW_DEADZONE_DEG)
    {
        plant.omega_output = 0;
        return 0;
    }
    kp = stopped ? SIM_YAW_KP_STOP : SIM_YAW_KP_MOVE;
    out_max = stopped ? SIM_YAW_OUTMAX_STOP : SIM_YAW_OUTMAX_MOVE;
    out = sim_filter_gyro(sim_gyro_deg_s) * SIM_YAW_GYRO_K - kp * err;
    if (out > out_max) out = out_max;
    if (out < -out_max) out = -out_max;
    plant.omega_output = (int16_t)out;
    return plant.omega_output;
}

static double sim_scale_rpm(double value, uint32_t num, uint32_t den)
{
    if (num >= den) return value;
    return value * (double)num / (double)den;
}

static void sim_apply_motion(int16_t vx, int16_t vy, int16_t z,
                             uint32_t ramp_ms)
{
    double rotation = (double)z * SIM_ROTATION_SCALE;
    double w[4];
    double max_mag = 0.0;
    int i;

    w[0] = (double)vx + (double)vy + rotation;
    w[1] = (double)vx - (double)vy + rotation;
    w[2] = (double)vx - (double)vy - rotation;
    w[3] = (double)vx + (double)vy - rotation;
    for (i = 0; i < 4; i++)
    {
        if (fabs(w[i]) > max_mag) max_mag = fabs(w[i]);
    }
    if (max_mag > SIM_CHASSIS_MAX_RPM)
    {
        for (i = 0; i < 4; i++)
        {
            w[i] = w[i] * SIM_CHASSIS_MAX_RPM / max_mag;
        }
    }
    for (i = 0; i < 4; i++)
    {
        plant.motor_target_rpm[i] =
            sim_scale_rpm(w[i], ramp_ms, SIM_START_RAMP_MS);
    }
}

static void sim_chassis_run_1ms(uint32_t now_ms)
{
    int16_t vx = plant.tgt_vx;
    int16_t vy = plant.tgt_vy;
    int16_t z = plant.tgt_z;
    bool motion = (vx != 0) || (vy != 0) || (z != 0);
    uint32_t elapsed, remaining;
    int i;

    if (plant.emergency)
    {
        if (plant.state != 4)
        {
            for (i = 0; i < 4; i++) plant.motor_target_rpm[i] = 0.0;
            plant.state = 4;
        }
        else
        {
            if ((plant.tgt_vx == 0) && (plant.tgt_vy == 0) &&
                (plant.tgt_z == 0))
            {
                plant.emergency = false;
            }
            plant.state = 0;
        }
        return;
    }
    switch (plant.state)
    {
    case 0:
        if (motion)
        {
            plant.ramp_begin_ms = now_ms;
            plant.state = 1;
        }
        else
        {
            z = sim_calc_omega(0, 0, 0, now_ms);
            sim_apply_motion(0, 0, z, SIM_START_RAMP_MS);
        }
        break;
    case 1:
        if (!motion)
        {
            plant.ramp_begin_ms = now_ms;
            for (i = 0; i < 4; i++)
                plant.stop_start_rpm[i] = plant.motor_target_rpm[i];
            plant.state = 3;
            break;
        }
        elapsed = now_ms - plant.ramp_begin_ms;
        z = sim_calc_omega(vx, vy, z, now_ms);
        if (elapsed >= SIM_START_RAMP_MS)
        {
            elapsed = SIM_START_RAMP_MS;
            plant.state = 2;
        }
        sim_apply_motion(vx, vy, z, elapsed);
        break;
    case 2:
        if (!motion)
        {
            plant.ramp_begin_ms = now_ms;
            for (i = 0; i < 4; i++)
                plant.stop_start_rpm[i] = plant.motor_target_rpm[i];
            plant.state = 3;
        }
        else
        {
            z = sim_calc_omega(vx, vy, z, now_ms);
            sim_apply_motion(vx, vy, z, SIM_START_RAMP_MS);
        }
        break;
    case 3:
        if (motion)
        {
            plant.ramp_begin_ms = now_ms;
            plant.state = 1;
            z = sim_calc_omega(vx, vy, z, now_ms);
            sim_apply_motion(vx, vy, z, 0);
            break;
        }
        elapsed = now_ms - plant.ramp_begin_ms;
        if (elapsed >= SIM_STOP_RAMP_MS)
        {
            remaining = 0;
            plant.state = 0;
        }
        else
        {
            remaining = SIM_STOP_RAMP_MS - elapsed;
        }
        for (i = 0; i < 4; i++)
        {
            plant.motor_target_rpm[i] =
                sim_scale_rpm(plant.stop_start_rpm[i], remaining,
                              SIM_STOP_RAMP_MS);
        }
        break;
    case 4:
        if (motion)
        {
            plant.ramp_begin_ms = now_ms;
            plant.state = 1;
        }
        else
        {
            plant.state = 0;
        }
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
static void sim_plant_step(uint32_t now_ms, double dt)
{
    double slew = SIM_ACCEL_LIMIT_MPS2 / SIM_RPM_TO_MPS * dt;
    double lf, rf, lr, rr, bvx, bvy, rot_mps, c, s, wvx, wvy;
    int i;

    for (i = 0; i < 4; i++)
    {
        double d = plant.motor_target_rpm[i] - plant.wheel_rpm[i];
        if (d > slew) d = slew;
        if (d < -slew) d = -slew;
        plant.wheel_rpm[i] += d;
    }
    lf = plant.wheel_rpm[0] * SIM_RPM_TO_MPS;
    rf = plant.wheel_rpm[1] * SIM_RPM_TO_MPS;
    lr = plant.wheel_rpm[2] * SIM_RPM_TO_MPS;
    rr = plant.wheel_rpm[3] * SIM_RPM_TO_MPS;
    bvx = 0.25 * (lf + rf + lr + rr);
    bvy = 0.25 * (lf - rf - lr + rr);
    rot_mps = 0.25 * (lf + rf - lr - rr);
    plant.yaw_deg += -(rot_mps / SIM_LXY_M) * (180.0 / SIM_PI) * dt;
    plant.yaw_deg = sim_normalize_angle(plant.yaw_deg);
    sim_gyro_deg_s = -(rot_mps / SIM_LXY_M) * (180.0 / SIM_PI);

    if (plant.have_prev_body)
    {
        plant.body_ax = (bvx - plant.prev_body_vx) / dt;
        plant.body_ay = (bvy - plant.prev_body_vy) / dt;
    }
    else
    {
        plant.body_ax = 0.0;
        plant.body_ay = 0.0;
        plant.have_prev_body = true;
    }
    plant.prev_body_vx = bvx;
    plant.prev_body_vy = bvy;

    c = cos(plant.yaw_deg * SIM_PI / 180.0);
    s = sin(plant.yaw_deg * SIM_PI / 180.0);
    wvx = c * bvx - s * bvy;
    wvy = s * bvx + c * bvy;
    plant.px += wvx * dt;
    plant.py += wvy * dt;

    {
        double ca = fabs(c), sa = fabs(s);
        double hx = ca * SIM_HALF_WID_M + sa * SIM_HALF_LEN_M;
        double hy = sa * SIM_HALF_WID_M + ca * SIM_HALF_LEN_M;
        int w;
        for (w = 0; w < sim_wall_count; w++)
        {
            const sim_wall_t *wall = &sim_walls[w];
            double ox, oy;
            if (!wall->solid) continue;
            if (plant.px + hx <= wall->x_min || plant.px - hx >= wall->x_max ||
                plant.py + hy <= wall->y_min || plant.py - hy >= wall->y_max)
                continue;
            {
                double pen_left = (plant.px + hx) - wall->x_min;
                double pen_right = wall->x_max - (plant.px - hx);
                double pen_down = (plant.py + hy) - wall->y_min;
                double pen_up = wall->y_max - (plant.py - hy);
                double m = pen_left; ox = -pen_left; oy = 0.0;
                if (pen_right < m) { m = pen_right; ox = pen_right; oy = 0.0; }
                if (pen_down < m) { m = pen_down; ox = 0.0; oy = -pen_down; }
                if (pen_up < m) { m = pen_up; ox = 0.0; oy = pen_up; }
                {
                    double impact = (ox != 0.0) ? fabs(wvx) : fabs(wvy);
                    bool new_contact =
                        (now_ms - plant.wall_contact_ms[w]) > 200;
                    plant.wall_contact_ms[w] = now_ms;
                    if (new_contact && impact > 0.02)
                    {
                        plant.contact_events++;
                        if (plant.contact_events == 1)
                        {
                            plant.first_contact_speed = impact;
                            plant.first_contact_ms = now_ms;
                            snprintf(plant.first_contact_wall,
                                     sizeof(plant.first_contact_wall),
                                     "wall[%d]", w);
                        }
                        if (impact > plant.max_contact_speed)
                            plant.max_contact_speed = impact;
                        printf("[%.3f s] !! 撞墙 wall[%d]: 法向速度 "
                               "%.2f m/s @ (%.3f, %.3f)\n",
                               now_ms / 1000.0, w, impact,
                               plant.px, plant.py);
                    }
                    plant.slip_distance += m;
                }
                plant.px += ox;
                plant.py += oy;
            }
        }
    }
}

static void sim_write_float_le(uint8_t *bytes, float value)
{
    memcpy(bytes, &value, sizeof(value));
}

static void sim_send_accel_frame(uint32_t now_ms)
{
    uint8_t frame[19];
    memset(frame, 0, sizeof(frame));
    frame[3] = 0x01U;
    sim_write_float_le(&frame[4],
                       (float)(plant.body_ax + SIM_ACCEL_BIAS_X));
    sim_write_float_le(&frame[8],
                       (float)(plant.body_ay + SIM_ACCEL_BIAS_Y));
    sim_write_float_le(&frame[12], (float)SIM_GRAVITY_Z);
    PathLineImu_OnAccelerationFrame(frame, (uint8_t)sizeof(frame), now_ms);
}

/* ------------------------------------------------------------------ */
static double sim_raycast(double ox, double oy, double dx, double dy)
{
    double best = 1e9;
    int w;
    for (w = 0; w < sim_wall_count; w++)
    {
        const sim_wall_t *wall = &sim_walls[w];
        double t_min = 0.0, t_max = 1e9, t1, t2, tmp;
        if (!wall->solid) continue;
        if (fabs(dx) < 1e-9)
        {
            if (ox < wall->x_min || ox > wall->x_max) continue;
        }
        else
        {
            t1 = (wall->x_min - ox) / dx;
            t2 = (wall->x_max - ox) / dx;
            if (t1 > t2) { tmp = t1; t1 = t2; t2 = tmp; }
            if (t1 > t_min) t_min = t1;
            if (t2 < t_max) t_max = t2;
            if (t_min > t_max) continue;
        }
        if (fabs(dy) < 1e-9)
        {
            if (oy < wall->y_min || oy > wall->y_max) continue;
        }
        else
        {
            t1 = (wall->y_min - oy) / dy;
            t2 = (wall->y_max - oy) / dy;
            if (t1 > t2) { tmp = t1; t1 = t2; t2 = tmp; }
            if (t1 > t_min) t_min = t1;
            if (t2 < t_max) t_max = t2;
            if (t_min > t_max) continue;
        }
        if (t_max < 0.0) continue;
        if (t_min < best && t_min >= 0.0) best = t_min;
    }
    return best;
}

static uint16_t sim_dt35_cm(double dist_m, int max_cm)
{
    double cm = dist_m * 100.0;
    if (cm <= SIM_DT35_MIN_CM) return SIM_DT35_MIN_CM;
    if (cm >= max_cm) return (uint16_t)max_cm;
    return (uint16_t)cm;
}

static void sim_dt35_sample(uint32_t now_ms)
{
    double c = cos(plant.yaw_deg * SIM_PI / 180.0);
    double s = sin(plant.yaw_deg * SIM_PI / 180.0);
    double fx = plant.px + (-s) * SIM_FRONT_OFFSET_M;
    double fy = plant.py + (c) * SIM_FRONT_OFFSET_M;
    double fdx = -s, fdy = c;
    double lx = plant.px + (-c) * SIM_LEFT_OFFSET_M;
    double ly = plant.py + (-s) * SIM_LEFT_OFFSET_M;
    double ldx = -c, ldy = -s;

    dt35_link[SENSOR_LINK_F_INDEX].distance_cm =
        sim_dt35_cm(sim_raycast(fx, fy, fdx, fdy), SIM_DT35_FRONT_MAX_CM);
    dt35_link[SENSOR_LINK_F_INDEX].last_rx_ms = now_ms;
    dt35_link[SENSOR_LINK_F_INDEX].online = 1U;
    dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm =
        sim_dt35_cm(sim_raycast(lx, ly, ldx, ldy), SIM_DT35_LEFT_MAX_CM);
    dt35_link[SENSOR_LINK_L_B_INDEX].last_rx_ms = now_ms;
    dt35_link[SENSOR_LINK_L_B_INDEX].online = 1U;
}

static void sim_add_wall(double x0, double y0, double x1, double y1)
{
    sim_walls[sim_wall_count].x_min = x0;
    sim_walls[sim_wall_count].y_min = y0;
    sim_walls[sim_wall_count].x_max = x1;
    sim_walls[sim_wall_count].y_max = y1;
    sim_walls[sim_wall_count].solid = true;
    sim_wall_count++;
}

static void sim_build_field(bool mirrored)
{
    sim_wall_count = 0;
    sim_add_wall(-1.0, -1.0, 3.0 + 1.0, 0.0);
    sim_add_wall(-1.0, 5.951, 4.0, 7.0);
    if (!mirrored)
    {
        sim_add_wall(-1.0, -1.0, 0.049, 7.0);
    }
    else
    {
        sim_add_wall(-1.0, 1.070, 0.049, 7.0);
    }
    sim_add_wall(2.951, -1.0, 4.0, 7.0);
    if (!mirrored)
    {
        sim_add_wall(1.050, 1.070, 3.000, 1.120);
        sim_add_wall(0.000, 2.075, 2.000, 2.125);
        sim_add_wall(1.050, 3.075, 3.000, 3.125);
    }
    else
    {
        sim_add_wall(0.000, 1.070, 1.950, 1.120);
        sim_add_wall(1.000, 2.075, 3.000, 2.125);
        sim_add_wall(0.000, 3.075, 1.950, 3.125);
    }
}

/* ------------------------------------------------------------------ */
typedef enum
{
    DRIVER_FULL = 0,
    DRIVER_CAREFUL = 1
} driver_profile_t;

typedef struct
{
    driver_profile_t profile;
    uint32_t route_done_ms;
    int phase;
    int last_segment;
    uint32_t segment_change_ms;
    double stall_x, stall_y;
    uint32_t stall_since_ms;
    bool recovering;
    double recover_dir_x, recover_dir_y;
    double recover_from_x, recover_from_y;
    int recover_count;
} driver_t;

static driver_t driver;

static void driver_body_from_map(double mx, double my,
                                 int16_t *vx, int16_t *vy)
{
    double c = cos(plant.yaw_deg * SIM_PI / 180.0);
    double s = sin(plant.yaw_deg * SIM_PI / 180.0);
    double bx = c * mx + s * my;
    double by = -s * mx + c * my;
    *vx = (int16_t)lround(bx);
    *vy = (int16_t)lround(by);
}

static void driver_frame(uint32_t now_ms, const path_diagnostics_t *diag)
{
    int16_t vx = 0, vy = 0, z = 0;
    uint8_t buttons = 0;

    switch (driver.phase)
    {
    case 0:
        if (diag->initial_position_valid)
        {
            driver.phase = 1;
        }
        else if (now_ms > 6500)
        {
            driver_body_from_map(0.0, SIM_REMOTE_FAST, &vx, &vy);
        }
        break;
    case 1:
        if (diag->route_complete)
        {
            driver.phase = 2;
            driver.route_done_ms = now_ms;
            break;
        }
        if ((int)diag->segment_index != driver.last_segment)
        {
            driver.last_segment = diag->segment_index;
            driver.segment_change_ms = now_ms;
        }
        if ((now_ms - driver.segment_change_ms) < 400U)
        {
            break;
        }
        if (driver.recovering)
        {
            double moved = hypot(diag->map_x_m - driver.recover_from_x,
                                 diag->map_y_m - driver.recover_from_y);
            if (moved >= 0.08)
            {
                driver.recovering = false;
                driver.stall_since_ms = now_ms;
                break;
            }
            driver_body_from_map(driver.recover_dir_x * SIM_REMOTE_FINE,
                                 driver.recover_dir_y * SIM_REMOTE_FINE,
                                 &vx, &vy);
            break;
        }
        {
            uint8_t count;
            const path_map_route_segment_t *route = PathMap_GetRoute(&count);

            if (diag->segment_index < count)
            {
                const path_map_route_segment_t *seg =
                    &route[diag->segment_index];
                double coord = (seg->axis == PATH_MAP_AXIS_X) ?
                               diag->map_x_m : diag->map_y_m;
                double remaining = fabs(seg->target_m - coord);
                int speed = SIM_REMOTE_FAST;
                {
                    double moved = hypot(diag->map_x_m - driver.stall_x,
                                         diag->map_y_m - driver.stall_y);
                    if (moved > 0.01)
                    {
                        driver.stall_x = diag->map_x_m;
                        driver.stall_y = diag->map_y_m;
                        driver.stall_since_ms = now_ms;
                    }
                    else if ((now_ms - driver.stall_since_ms) > 2000)
                    {
                        static const double dirs[4][2] =
                            { {1,0}, {-1,0}, {0,1}, {0,-1} };
                        double best = -1.0;
                        int k, best_k = 0;
                        for (k = 0; k < 4; k++)
                        {
                            double clr = PathMap_RayClearance(
                                diag->map_x_m, diag->map_y_m,
                                (float)dirs[k][0], (float)dirs[k][1]);
                            if (clr > best)
                            {
                                best = clr;
                                best_k = k;
                            }
                        }
                        driver.recovering = true;
                        driver.recover_dir_x = dirs[best_k][0];
                        driver.recover_dir_y = dirs[best_k][1];
                        driver.recover_from_x = diag->map_x_m;
                        driver.recover_from_y = diag->map_y_m;
                        driver.recover_count++;
                        printf("[%.3f s] 驾驶员脱困 #%d: 位置(%.3f,%.3f) "
                               "朝(%g,%g)退 8 cm\n", now_ms / 1000.0,
                               driver.recover_count,
                               diag->map_x_m, diag->map_y_m,
                               driver.recover_dir_x, driver.recover_dir_y);
                        break;
                    }
                }
                if (driver.profile == DRIVER_CAREFUL && remaining < 0.5)
                {
                    speed = SIM_REMOTE_FINE;
                }
                if (seg->axis == PATH_MAP_AXIS_X)
                {
                    driver_body_from_map(seg->direction * speed, 0.0,
                                         &vx, &vy);
                }
                else
                {
                    driver_body_from_map(0.0, seg->direction * speed,
                                         &vx, &vy);
                }
            }
        }
        break;
    default:
        break;
    }

    Path_SubmitRemoteCommand(&vx, &vy, &z, buttons, now_ms);
    (void)Chassis_SetVelocity(vx, vy, z);
}

/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    bool mirrored = false;
    bool autonomous = false;
    driver_profile_t profile = DRIVER_FULL;
    const char *csv_path = "sim_log.csv";
    FILE *csv;
    path_diagnostics_t diag;
    path_line_imu_data_t odom;
    uint32_t now, next_remote = 0, next_dt35 = 0, next_log = 0;
    uint32_t next_vesc = 0, next_accel = 0;
    uint32_t seg_change_ms = 0;
    int last_seg = -1;
    uint32_t anchored_ms = 0, route_done_ms = 0;
    uint32_t beep_start_ms = 0, auto_done_ms = 0, imu_ready_ms = 0;
    uint32_t bias_ready_ms = 0;
    bool aborted = false;
    char abort_reason[128] = "";
    int i;

    for (i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--mirrored") == 0) mirrored = true;
        else if (strcmp(argv[i], "--careful") == 0) profile = DRIVER_CAREFUL;
        else if (strcmp(argv[i], "--auto") == 0) autonomous = true;
        else if (strncmp(argv[i], "--csv=", 6) == 0) csv_path = argv[i] + 6;
    }

    memset(&plant, 0, sizeof(plant));
    memset(&driver, 0, sizeof(driver));
    memset((void *)dt35_link, 0, sizeof(dt35_link));
    driver.profile = profile;
    sim_build_field(mirrored);

    /*
     * 起始摆位：常规侧左光约 10.5 cm → x = 0.049+0.220+0.105 = 0.374；
     * 镜像侧贴东墙 x = 2.626；中心 y = 半车长 0.3085。
     */
    plant.px = mirrored ? 2.626 : 0.374;
    plant.py = SIM_HALF_LEN_M;
    plant.yaw_deg = 0.0;

    Path_Init();
    PathLineImu_Init();

    csv = fopen(csv_path, "w");
    if (csv == NULL)
    {
        fprintf(stderr, "cannot open %s\n", csv_path);
        return 1;
    }
    fprintf(csv, "t_ms,true_x,true_y,yaw_deg,map_x,map_y,fused_x,fused_y,"
                 "enc_x,enc_y,enc_w,imu_ok,seg,auto_state,route_complete,"
                 "front_cm,left_cm,out_vx,out_vy,map_lim,front_lim,left_lim,"
                 "clearance,odom_err\n");

    for (now = 0; now <= SIM_MAX_TIME_MS; now++)
    {
        if (now >= SIM_IMU_READY_MS)
        {
            if (!plant.imu_ready)
            {
                plant.imu_ready = true;
                imu_ready_ms = now;
                printf("[%.3f s] IMU READY，开始水平加速度零偏窗口\n",
                       now / 1000.0);
            }
        }
        if (now >= next_dt35)
        {
            sim_dt35_sample(now);
            next_dt35 += SIM_DT35_PERIOD_MS;
        }
        if (now >= next_vesc)
        {
            plant.vesc_last_rx_ms = now;
            next_vesc += SIM_VESC_STATUS_PERIOD_MS;
        }

        (void)Path_GetDiagnostics(&diag);
        if (!autonomous && (now >= next_remote))
        {
            driver_frame(now, &diag);
            next_remote += SIM_REMOTE_PERIOD_MS;
        }

        /* 与 freertos.c 同序：底盘 → 植物积分 → 融合里程计 → 路径层 */
        sim_chassis_run_1ms(now);
        sim_plant_step(now, 0.001);
        if (now >= next_accel)
        {
            sim_send_accel_frame(now);
            next_accel += SIM_ACCEL_PERIOD_MS;
        }
        PathLineImu_Run1ms(now);
        Path_Run1ms(now);
        (void)Path_GetDiagnostics(&diag);
        (void)PathLineImu_GetData(&odom);

        if (odom.imu_solution_valid && bias_ready_ms == 0)
        {
            bias_ready_ms = now;
            printf("[%.3f s] 融合里程计 IMU 解就绪 (bias_x=%.4f "
                   "bias_y=%.4f weight=%.2f)\n",
                   now / 1000.0, odom.accel_bias_x_mps2,
                   odom.accel_bias_y_mps2, odom.encoder_weight);
        }
        if (diag.initial_position_valid && anchored_ms == 0)
        {
            anchored_ms = now;
            printf("[%.3f s] 锚定: map=(%.4f, %.4f) 真值=(%.4f, %.4f) "
                   "融合位移=(%.4f, %.4f) 镜像=%d\n", now / 1000.0,
                   diag.initial_map_x_m, diag.initial_map_y_m,
                   plant.px, plant.py,
                   odom.fused_position_x_m, odom.fused_position_y_m,
                   diag.map_mirrored);
        }
        if ((int)diag.segment_index != last_seg)
        {
            if (last_seg >= 0 && (int)diag.segment_index > last_seg)
            {
                printf("[%.3f s] 去程段 %d 完成: 真值=(%.4f, %.4f) "
                       "map=(%.4f, %.4f) 融合=(%.4f, %.4f)\n",
                       now / 1000.0, last_seg,
                       plant.px, plant.py, diag.map_x_m, diag.map_y_m,
                       odom.fused_position_x_m, odom.fused_position_y_m);
            }
            last_seg = diag.segment_index;
            seg_change_ms = now;
        }
        if (diag.route_complete && route_done_ms == 0)
        {
            route_done_ms = now;
            printf("[%.3f s] 去程 6 段全部完成 (route_complete)\n",
                   now / 1000.0);
        }
        {
            bool beep_level;
            if (Path_ArrivalBeep(&beep_level) && (beep_start_ms == 0))
            {
                beep_start_ms = now;
                printf("[%.3f s] 到达终点：自动停车，蜂鸣器鸣响 "
                       "(auto_state=%u)\n", now / 1000.0,
                       diag.auto_state);
            }
        }
        if (autonomous && (diag.auto_state == PATH_AUTO_STATE_DONE) &&
            auto_done_ms == 0)
        {
            auto_done_ms = now;
        }

        {
            double odom_err = 0.0;
            if (diag.initial_position_valid)
            {
                odom_err = hypot(diag.map_x_m - plant.px,
                                 diag.map_y_m - plant.py);
                if (odom_err > plant.max_odom_err_m)
                {
                    plant.max_odom_err_m = odom_err;
                    plant.max_odom_err_ms = now;
                }
            }
            if (now >= next_log)
            {
                fprintf(csv, "%u,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%.4f,"
                        "%.4f,%.4f,%.3f,%d,%u,%u,%d,%u,%u,%d,%d,%d,%d,%d,"
                        "%.3f,%.4f\n",
                        now, plant.px, plant.py, plant.yaw_deg,
                        diag.map_x_m, diag.map_y_m,
                        odom.fused_position_x_m, odom.fused_position_y_m,
                        odom.encoder_position_x_m, odom.encoder_position_y_m,
                        odom.encoder_weight,
                        odom.imu_solution_valid ? 1 : 0,
                        diag.segment_index, diag.auto_state,
                        diag.route_complete,
                        diag.front_distance_cm, diag.left_distance_cm,
                        diag.output_vx, diag.output_vy,
                        diag.map_speed_limited, diag.front_speed_limited,
                        diag.left_speed_limited, diag.map_clearance_m,
                        odom_err);
                next_log += SIM_LOG_PERIOD_MS;
            }
        }

        if (!autonomous && (driver.phase == 2) &&
            (now > driver.route_done_ms + 1000))
        {
            break;
        }
        if (autonomous && auto_done_ms != 0 && now > auto_done_ms + 1500)
        {
            break;
        }
        if ((now - seg_change_ms) > SIM_SEGMENT_TIMEOUT_MS &&
            anchored_ms != 0)
        {
            aborted = true;
            snprintf(abort_reason, sizeof(abort_reason),
                     "段超时: 去程段 %d 卡住 (真值 y=%.4f x=%.4f, "
                     "map=(%.4f,%.4f), 前光 %u cm)",
                     diag.segment_index, plant.py, plant.px,
                     diag.map_x_m, diag.map_y_m, diag.front_distance_cm);
            break;
        }
        if (anchored_ms == 0 && now > 30000)
        {
            aborted = true;
            snprintf(abort_reason, sizeof(abort_reason), "30 s 未锚定");
            break;
        }
    }
    fclose(csv);

    (void)Path_GetDiagnostics(&diag);
    (void)PathLineImu_GetData(&odom);
    printf("\n===== 仿真结果 (%s侧, %s) =====\n",
           mirrored ? "镜像" : "常规",
           autonomous ? "全自动(无遥控)" :
           (profile == DRIVER_FULL ? "全速驾驶" : "谨慎驾驶"));
    printf("IMU READY:        t=%.2f s\n", imu_ready_ms / 1000.0);
    printf("融合 IMU 解:      %s (t=%.2f s, weight=%.2f)\n",
           bias_ready_ms ? "就绪" : "未就绪", bias_ready_ms / 1000.0,
           odom.encoder_weight);
    printf("锚定:            %s (t=%.2f s)\n",
           anchored_ms ? "成功" : "失败", anchored_ms / 1000.0);
    printf("去程 6 段:       %s (t=%.2f s)\n",
           route_done_ms ? "完成" : "未完成", route_done_ms / 1000.0);
    printf("最终真值位姿:    (%.4f, %.4f) yaw=%.1f°\n",
           plant.px, plant.py, plant.yaw_deg);
    printf("最终地图坐标:    (%.4f, %.4f) 段=%u\n",
           diag.map_x_m, diag.map_y_m, diag.segment_index);
    printf("最终融合位移:    (%.4f, %.4f) 编码器=(%.4f, %.4f)\n",
           odom.fused_position_x_m, odom.fused_position_y_m,
           odom.encoder_position_x_m, odom.encoder_position_y_m);
    printf("里程计最大误差:  %.1f mm @ t=%.2f s\n",
           plant.max_odom_err_m * 1000.0, plant.max_odom_err_ms / 1000.0);
    printf("撞墙接触:        %d 次, 最大接触速度 %.2f m/s, 压墙滑移合计 %.3f m\n",
           plant.contact_events, plant.max_contact_speed,
           plant.slip_distance);
    if (plant.contact_events)
    {
        printf("  首次接触: t=%.2f s, %s, 速度 %.2f m/s\n",
               plant.first_contact_ms / 1000.0, plant.first_contact_wall,
               plant.first_contact_speed);
    }
    if (autonomous)
    {
        printf("自动模式:        auto_state=%u, 到点停车%s, 蜂鸣%s "
               "(t=%.2f s)\n", diag.auto_state,
               auto_done_ms ? "完成" : "未完成",
               beep_start_ms ? "已响" : "未响", beep_start_ms / 1000.0);
    }
    if (aborted)
    {
        printf("中止: %s\n", abort_reason);
    }
    if (autonomous)
    {
        if (route_done_ms && auto_done_ms && beep_start_ms &&
            !plant.contact_events)
        {
            printf("结论: 全自动到点停车+鸣笛完成（无接触，真实融合里程计）\n");
        }
        else if (route_done_ms && auto_done_ms && beep_start_ms)
        {
            printf("结论: 全自动到点完成（%d 次接触, 最大 %.2f m/s）\n",
                   plant.contact_events, plant.max_contact_speed);
        }
        else
        {
            printf("结论: 全自动路线未完成\n");
        }
        return route_done_ms && auto_done_ms ? 0 : 1;
    }
    if (route_done_ms)
    {
        if (!plant.contact_events)
        {
            printf("结论: 起始区到终点完成（无接触，真实融合里程计）\n");
        }
        else if (plant.max_contact_speed <= 0.5)
        {
            printf("结论: 起始区到终点完成（%d 次轻微刮擦, "
                   "最大法向 %.2f m/s）\n",
                   plant.contact_events, plant.max_contact_speed);
        }
        else
        {
            printf("结论: 到达终点但存在明显碰撞（最大 %.2f m/s）\n",
                   plant.max_contact_speed);
        }
        return 0;
    }
    printf("结论: 路线未完成\n");
    return 1;
}
