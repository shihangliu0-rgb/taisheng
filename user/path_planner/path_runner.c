/**
 ******************************************************************************
 * @file    path_runner.c
 * @brief   在线跟踪总控实现(离线剖面查表 + 纯追踪 + 激光兜底 + 航向锁)
 ******************************************************************************
 */
#include "path_runner.h"

#include "path_config.h"
#include "path_fusion.h"
#include "path_geometry.h"
#include "path_pure_pursuit.h"
#include "path_speed_profile.h"
#include "path_spline.h"
#include "path_yaw_lock.h"

/* 仓库已有模块(全部真实外设数据) */
#include "chassis_main.h"
#include "dt35_pnp_link.h"
#include "imu_main.h"
#include "pc_link.h"

#include <math.h>
#include <string.h>

#if PATH_DEBUG
#include "usart.h"   /* PATH_DEBUG_UART_HANDLE 引用(如 huart8) */
#endif

/* ------------------------------------------------------------------ */
static path_point_t trajectory[PATH_SPLINE_SAMPLES];
static uint16_t trajectory_count;
static path_gridmap_t real_map;
static path_gridmap_t inflated_map;
static path_waypoint_t waypoints[PATH_WAYPOINT_COUNT] =
{
    PATH_WAYPOINTS_TABLE
};

static path_state_t state = PATH_STATE_INIT;
static path_reason_t reason = PATH_REASON_BOOT;
static path_debug_t debug;

static uint32_t state_start_ms;
static uint32_t run_start_ms;
static uint32_t last_step_ms;
#if PATH_DEBUG && defined(PATH_DEBUG_UART_HANDLE)
static uint32_t last_debug_ms;
#endif
static uint16_t last_i_near;
static uint32_t last_pc_frame_count;   /* 上位机帧去重:每帧只融合一次 */
static float last_cmd_vx;
static float last_cmd_vy;
static float last_cmd_w;

/* ------------------------------------------------------------------ */
static void runner_stop(path_reason_t why)
{
    state = PATH_STATE_STOPPED;
    reason = why;
    last_cmd_vx = 0.0f;
    last_cmd_vy = 0.0f;
    last_cmd_w = 0.0f;
    Chassis_StopAll();
}

/* 数值健康检查:NaN/Inf/量级异常一律判非法(v == v 可移植地判 NaN) */
static bool num_ok(float v)
{
    return (v == v) && (fabsf(v) < 1e6f);
}

/* 每个控制周期读一次真实传感器并做融合 */
static bool runner_read_and_fuse(uint32_t now_ms, float dt_s,
                                 imu_data_t *imu,
                                 float *laser_f_m, bool *laser_f_ok,
                                 float *laser_l_m, bool *laser_l_ok)
{
    bool imu_ok;
    pc_position_t upper;

    imu_ok = ImuMain_GetData(imu) &&
             imu->online && imu->yaw_valid && imu->gyro_valid &&
             (imu->state == IMU_STATE_READY);

    /* DT35 前/左激光(串口帧解析值,单位 cm -> m) */
    *laser_f_m = (float)dt35_link[SENSOR_LINK_F_INDEX].distance_cm * 0.01f;
    *laser_f_ok = (dt35_link[SENSOR_LINK_F_INDEX].online != 0U) &&
                  ((uint32_t)(now_ms -
                   dt35_link[SENSOR_LINK_F_INDEX].last_rx_ms) <
                   PATH_LASER_TIMEOUT_MS);
    *laser_l_m = (float)dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm * 0.01f;
    *laser_l_ok = (dt35_link[SENSOR_LINK_L_B_INDEX].online != 0U) &&
                  ((uint32_t)(now_ms -
                   dt35_link[SENSOR_LINK_L_B_INDEX].last_rx_ms) <
                   PATH_LASER_TIMEOUT_MS);

    if (imu_ok)
    {
        PathFusion_Predict(imu->gyro_z_deg_s, dt_s);
    }

    /* 上位机位姿(0x11 位置帧,field_w 按已修复的 yaw_rad 理解,
     * 见 pc_link 模块注释:小电脑当前传四元数 W 是已知 bug)。
     * 去重:pc_link 保存最近一帧,本函数 5ms 调一次,同一帧会被读多次,
     * 若不去重,中值滤波窗口会被同一个跳变值填满而失去滤波意义,
     * 因此只在"好帧计数变化"即真正收到新帧时才融合一次。 */
    {
        uint32_t pos_seq = PcLink_GetPositionSeq();
        if (PcLink_GetPosition(&upper) &&
            ((upper.flags & PC_LINK_FLAG_FIELD_VALID) != 0U) &&
            (pos_seq != last_pc_frame_count))
        {
            last_pc_frame_count = pos_seq;
            (void)PathFusion_UpdateUpper(upper.field_x_m, upper.field_y_m,
                                         upper.field_w, now_ms);
        }
    }

    return imu_ok;
}

/* ------------------------------------------------------------------ */
static void runner_step(uint32_t now_ms, float dt_s)
{
    imu_data_t imu;
    float laser_f;
    float laser_l;
    bool laser_f_ok;
    bool laser_l_ok;
    bool imu_ok;
    float fx;
    float fy;
    float fyaw;
    float lf;
    float v_laser;
    float v_ref;
    float v_used;
    float exp_l;
    float tx;
    float ty;
    float L;
    float vx_w;
    float vy_w;
    float vx_c;
    float vy_c;
    float w_cmd;
    float dv_max;
    float dw_max;
    uint16_t i_near;
    uint16_t i_target;
    int16_t rpm_x;
    int16_t rpm_y;
    int16_t z;

    imu_ok = runner_read_and_fuse(now_ms, dt_s, &imu,
                                  &laser_f, &laser_f_ok,
                                  &laser_l, &laser_l_ok);
    PathFusion_Get(&fx, &fy, &fyaw);

    /* ---- 安全检查(顺序即优先级) ---- */
    if (PathFusion_IsUpperLost(now_ms))
    {
        runner_stop(PATH_REASON_STOP_UPPER_LOST);
        return;
    }
    if (!imu_ok)
    {
        runner_stop(PATH_REASON_STOP_IMU_LOST);
        return;
    }
    if ((PATH_STOP_ON_LASER_LOSS != 0U) && !laser_f_ok)
    {
        runner_stop(PATH_REASON_STOP_LASER_LOST);
        return;
    }
    if ((uint32_t)(now_ms - run_start_ms) > PATH_MAX_RUN_MS)
    {
        runner_stop(PATH_REASON_STOP_TIMEOUT);
        return;
    }

    /* ---- 到达判定(只用融合位姿) ---- */
    {
        float dx = fx - PATH_GOAL_X_M;
        float dy = fy - PATH_GOAL_Y_M;
        if (sqrtf(dx * dx + dy * dy) <= PATH_ARRIVE_TOL_M)
        {
            state = PATH_STATE_ARRIVED;
            reason = PATH_REASON_ARRIVED;
            last_cmd_vx = 0.0f;
            last_cmd_vy = 0.0f;
            last_cmd_w = 0.0f;
            Chassis_StopAll();
            return;
        }
    }

    /* ---- 查表(只读,不修改离线剖面) ---- */
    i_near = PathSpeedProfile_Nearest(trajectory, trajectory_count,
                                      fx, fy, last_i_near);
    last_i_near = i_near;
    v_ref = trajectory[i_near].v_ref;
    exp_l = trajectory[i_near].exp_laser_left_m;

    /* ---- 纯追踪:目标点(先于激光兜底计算,激光停车时需要目标方向) ---- */
    PathPurePursuit_Find(trajectory, trajectory_count, fx, fy, v_ref,
                         trajectory[i_near].kappa,
                         i_near, &i_target, &tx, &ty);
    L = sqrtf((tx - fx) * (tx - fx) + (ty - fy) * (ty - fy));
    if (L <= 1e-3f)
    {
        L = 1e-3f;
    }

    /* ---- 前激光兜底 ---- */
    lf = laser_f;
    if (lf > PATH_LASER_MAX_RANGE_M)
    {
        lf = PATH_LASER_MAX_RANGE_M;
    }
    if (lf <= PATH_LASER_STOP_DIST_M)
    {
        /* 强制 vx=0:保留目标方向的横向分量做脱困平移(全向轮可侧移)。
         * 若目标几乎在正前方(横向分量过小)则原地等待,避免顶墙。 */
        float dir_bx;
        float dir_by;
        float lat_sign;

        PathWorldToBody((tx - fx) / L, (ty - fy) / L, fyaw,
                        &dir_bx, &dir_by);
        lat_sign = (dir_by > 0.05f) ? 1.0f :
                   ((dir_by < -0.05f) ? -1.0f : 0.0f);
        PathBodyToWorld(0.0f, lat_sign * PATH_LASER_STOP_LAT_MS, fyaw,
                        &vx_w, &vy_w);
        v_used = 0.0f;
        reason = PATH_REASON_STOP_LASER_FRONT;
    }
    else
    {
        v_laser = sqrtf(2.0f * PATH_A_LON_BRAKE *
                        (lf - PATH_LASER_STOP_DIST_M));
        v_used = (v_ref < v_laser) ? v_ref : v_laser;
        vx_w = v_used * (tx - fx) / L;
        vy_w = v_used * (ty - fy) / L;
        reason = (v_laser < (v_ref - 1e-3f)) ?
                 PATH_REASON_LASER_SLOW : PATH_REASON_RUN;
    }

    /* ---- 世界系 -> 底盘系(右/前),直接对应 Chassis_SetVelocity ---- */
    PathWorldToChassis(vx_w, vy_w, fyaw, &vx_c, &vy_c);

    /* ---- 左激光横向微调(err = laser_left - expected_left) ---- */
    if ((v_used > 0.0f) && laser_l_ok)
    {
        float err = laser_l - exp_l;
        float trim = PATH_LAT_TRIM_SIGN * PATH_LAT_TRIM_KP * err;
        if (trim > PATH_LAT_TRIM_MAX_MS)
        {
            trim = PATH_LAT_TRIM_MAX_MS;
        }
        if (trim < -PATH_LAT_TRIM_MAX_MS)
        {
            trim = -PATH_LAT_TRIM_MAX_MS;
        }
        /* 左墙太近:强制向右离开(底盘 +x = 向右) */
        if (laser_l < PATH_LAT_SAFE_M)
        {
            trim = PATH_LAT_TRIM_MAX_MS;
        }
        vx_c += trim;
    }

    /* ---- 航向锁 ---- */
    w_cmd = PathYawLock_Step(fyaw, v_used);

    /* ---- slew-rate 限幅(每周期变化不超过 max_accel * dt) ---- */
    dv_max = PATH_SLEW_XY_ACCEL_MS2 * dt_s;
    dw_max = PATH_SLEW_W_ACCEL_RADS2 * dt_s;
    if ((vx_c - last_cmd_vx) > dv_max) { vx_c = last_cmd_vx + dv_max; }
    if ((vx_c - last_cmd_vx) < -dv_max) { vx_c = last_cmd_vx - dv_max; }
    if ((vy_c - last_cmd_vy) > dv_max) { vy_c = last_cmd_vy + dv_max; }
    if ((vy_c - last_cmd_vy) < -dv_max) { vy_c = last_cmd_vy - dv_max; }
    if ((w_cmd - last_cmd_w) > dw_max) { w_cmd = last_cmd_w + dw_max; }
    if ((w_cmd - last_cmd_w) < -dw_max) { w_cmd = last_cmd_w - dw_max; }
    last_cmd_vx = vx_c;
    last_cmd_vy = vy_c;
    last_cmd_w = w_cmd;

    /* ---- 数值防护:任何环节产生 NaN/Inf 立即安全停车 ---- */
    if (!num_ok(vx_c) || !num_ok(vy_c) || !num_ok(w_cmd) ||
        !num_ok(v_ref) || !num_ok(v_used))
    {
        runner_stop(PATH_REASON_STOP_NUMERIC);
        return;
    }

    /* ---- 输出:底盘系(右/前) -> Chassis_SetVelocity(vx=右, vy=前, z=CCW) ---- */
    rpm_x = (int16_t)roundf(vx_c * PATH_RPM_PER_M_S);
    rpm_y = (int16_t)roundf(vy_c * PATH_RPM_PER_M_S);
    z = (int16_t)roundf(w_cmd * PATH_Z_PER_RAD_S);
    (void)Chassis_SetVelocity(rpm_x, rpm_y, z);

    /* ---- 调试信息 ---- */
    debug.i_near = i_near;
    debug.i_target = i_target;
    debug.v_ref = v_ref;
    debug.v_used = v_used;
    debug.laser_f_m = laser_f;
    debug.laser_l_m = laser_l;
    debug.exp_laser_l_m = exp_l;
    debug.fused_x = fx;
    debug.fused_y = fy;
    debug.fused_yaw_rad = fyaw;
    debug.cmd_vx_ch = vx_c;
    debug.cmd_vy_ch = vy_c;
    debug.cmd_w = w_cmd;
    debug.run_ms = now_ms - run_start_ms;
}

/* ------------------------------------------------------------------ */
void PathRunner_Init(void)
{
    (void)memset(&debug, 0, sizeof(debug));
    state = PATH_STATE_INIT;
    reason = PATH_REASON_BOOT;
    state_start_ms = HAL_GetTick();
    last_i_near = 0U;
    last_cmd_vx = 0.0f;
    last_cmd_vy = 0.0f;
    last_cmd_w = 0.0f;
    PathFusion_Init();
}

void PathRunner_Run(void)
{
    uint32_t now_ms = HAL_GetTick();
    float dt_s;

    /* 控制周期分频(commTask 1ms 调用) */
    if ((uint32_t)(now_ms - last_step_ms) < PATH_CONTROL_PERIOD_MS)
    {
        return;
    }
    dt_s = (float)(now_ms - last_step_ms) / 1000.0f;
    if (dt_s > 0.02f)
    {
        dt_s = 0.02f;
    }
    last_step_ms = now_ms;

    switch (state)
    {
    case PATH_STATE_INIT:
        /* 关闭 IMU 模块自带航向保持,避免它覆盖本模块的 z 指令 */
        ImuMain_EnableYawHold(false);
        state = PATH_STATE_CALIB;
        state_start_ms = now_ms;
        reason = PATH_REASON_CALIB;
        break;

    case PATH_STATE_CALIB:
    {
        imu_data_t imu;
        ImuMain_EnableYawHold(false);
        Chassis_StopAll();

        if (ImuMain_GetData(&imu) && imu.online &&
            imu.gyro_valid && (imu.state == IMU_STATE_READY))
        {
            if (PathFusion_CalibrateSample(imu.gyro_z_deg_s))
            {
                state = PATH_STATE_WAIT_START;
                state_start_ms = now_ms;
                reason = PATH_REASON_WAIT_START;
            }
        }
        /* IMU 异常兜底:标定超时也继续(零偏按 0 处理) */
        if ((uint32_t)(now_ms - state_start_ms) > 5000U)
        {
            state = PATH_STATE_WAIT_START;
            state_start_ms = now_ms;
            reason = PATH_REASON_WAIT_START;
        }
        break;
    }

    case PATH_STATE_WAIT_START:
    {
        imu_data_t imu;
        float laser_f;
        float laser_l;
        bool lf_ok;
        bool ll_ok;

        Chassis_StopAll();
        (void)runner_read_and_fuse(now_ms, dt_s, &imu,
                                   &laser_f, &lf_ok, &laser_l, &ll_ok);

        /* 起点由上电位姿覆盖(与 yaml 起点差 1m 以内才接受) */
        {
            pc_position_t upper;
            if (PcLink_GetPosition(&upper) &&
                ((upper.flags & PC_LINK_FLAG_FIELD_VALID) != 0U))
            {
                float dx = upper.field_x_m - waypoints[0].x_m;
                float dy = upper.field_y_m - waypoints[0].y_m;
                if (sqrtf(dx * dx + dy * dy) <= PATH_START_OVERRIDE_MAX_M)
                {
                    waypoints[0].x_m = upper.field_x_m;
                    waypoints[0].y_m = upper.field_y_m;
                }
                state = PATH_STATE_BUILD;
                state_start_ms = now_ms;
                break;
            }
        }
        if ((uint32_t)(now_ms - state_start_ms) > PATH_WAIT_START_MS)
        {
            /* 超时:用 yaml 占位起点 */
            state = PATH_STATE_BUILD;
            state_start_ms = now_ms;
        }
        break;
    }

    case PATH_STATE_BUILD:
    {
        PathGridMap_BuildReal(&real_map);
        PathGridMap_BuildInflated(&inflated_map);

        if (!PathSpline_Build(waypoints, PATH_WAYPOINT_COUNT,
                              trajectory, PATH_SPLINE_SAMPLES,
                              &trajectory_count))
        {
            runner_stop(PATH_REASON_STOP_BUILD);
            break;
        }
        /* 推离 + 平滑迭代,并重算弧长/曲率(避免点状外推撕裂拐角) */
        PathSpline_Finalize(trajectory, trajectory_count, &inflated_map);
        if (!PathSpeedProfile_Build(trajectory, trajectory_count, &real_map))
        {
            runner_stop(PATH_REASON_STOP_BUILD);
            break;
        }

        state = PATH_STATE_RUN;
        reason = PATH_REASON_RUN;
        run_start_ms = now_ms;
        last_i_near = 0U;
        break;
    }

    case PATH_STATE_RUN:
        runner_step(now_ms, dt_s);
        break;

    case PATH_STATE_ARRIVED:
    case PATH_STATE_STOPPED:
    default:
        Chassis_StopAll();
        break;
    }

    /* 调试信息公共部分 */
    debug.state = state;
    debug.reason = reason;
    PathFusion_GetStats(&debug.fusion_xy_rejects, &debug.fusion_yaw_rejects,
                        &debug.upper_frames);
    PcLink_GetStats(&debug.pc_frames, &debug.crc_errors);

    /* 回传小电脑状态帧(55 AA 20 | state | error):
     * state = 规划器状态机;error = 停止原因(STOPPED 时),正常为 0。
     * 上位机竞争网关用该帧判断控制器在线(500ms 未收到判离线)。 */
    PcLink_SetStatus((uint8_t)state,
                     (state == PATH_STATE_STOPPED) ? (uint8_t)reason : 0U);

#if PATH_DEBUG && defined(PATH_DEBUG_UART_HANDLE)
    if ((uint32_t)(now_ms - last_debug_ms) >= PATH_DEBUG_PERIOD_MS)
    {
        last_debug_ms = now_ms;
        PathRunner_DebugDump(&PATH_DEBUG_UART_HANDLE);
    }
#endif
}

void PathRunner_GetDebug(path_debug_t *out)
{
    if (out != NULL)
    {
        *out = debug;
    }
}

const path_point_t *PathRunner_GetTrajectory(uint16_t *count)
{
    /* 只要离线轨迹构建完成就返回(BUILD/RUN/ARRIVED/STOPPED 均可) */
    if (trajectory_count > 0U)
    {
        if (count != NULL)
        {
            *count = trajectory_count;
        }
        return trajectory;
    }
    if (count != NULL)
    {
        *count = 0U;
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
#if PATH_DEBUG && defined(PATH_DEBUG_UART_HANDLE)

#include <string.h>

static void dbg_puts(UART_HandleTypeDef *uart, const char *s)
{
    (void)HAL_UART_Transmit(uart, (uint8_t *)s, (uint16_t)strlen(s), 10U);
}

static void dbg_putf(UART_HandleTypeDef *uart, float v)
{
    char buf[24];
    uint8_t pos = 0U;
    char rev[12];
    uint8_t rpos = 0U;
    int32_t ip;
    uint32_t frac;

    if (v < 0.0f) { buf[pos++] = '-'; v = -v; }
    ip = (int32_t)v;
    frac = (uint32_t)((v - (float)ip) * 100.0f + 0.5f);
    if (frac >= 100U) { ip++; frac = 0U; }
    if (ip == 0) { buf[pos++] = '0'; }
    else
    {
        while (ip > 0) { rev[rpos++] = (char)('0' + (ip % 10)); ip /= 10; }
        while (rpos > 0U) { buf[pos++] = rev[--rpos]; }
    }
    buf[pos++] = '.';
    buf[pos++] = (char)('0' + (frac / 10U) % 10U);
    buf[pos++] = (char)('0' + frac % 10U);
    buf[pos] = '\0';
    dbg_puts(uart, buf);
}

/* 每 400ms 一行:
 * t_ms,state,reason,x,y,yaw,vx,vy,w,v_ref,v_used,laserF,laserL,crc */
void PathRunner_DebugDump(UART_HandleTypeDef *uart)
{
    dbg_puts(uart, "t=");
    dbg_putf(uart, (float)debug.run_ms);
    dbg_puts(uart, " st=");
    dbg_putf(uart, (float)debug.state);
    dbg_puts(uart, " rsn=");
    dbg_putf(uart, (float)debug.reason);
    dbg_puts(uart, " x=");
    dbg_putf(uart, debug.fused_x);
    dbg_puts(uart, " y=");
    dbg_putf(uart, debug.fused_y);
    dbg_puts(uart, " yaw=");
    dbg_putf(uart, debug.fused_yaw_rad * 57.29578f);
    dbg_puts(uart, " cmd=");
    dbg_putf(uart, debug.cmd_vx_ch);
    dbg_puts(uart, ",");
    dbg_putf(uart, debug.cmd_vy_ch);
    dbg_puts(uart, ",");
    dbg_putf(uart, debug.cmd_w);
    dbg_puts(uart, " vref=");
    dbg_putf(uart, debug.v_ref);
    dbg_puts(uart, " vused=");
    dbg_putf(uart, debug.v_used);
    dbg_puts(uart, " lf=");
    dbg_putf(uart, debug.laser_f_m);
    dbg_puts(uart, " ll=");
    dbg_putf(uart, debug.laser_l_m);
    dbg_puts(uart, " crc=");
    dbg_putf(uart, (float)debug.crc_errors);
    dbg_puts(uart, "\r\n");
}

#endif /* PATH_DEBUG && PATH_DEBUG_UART_HANDLE */
