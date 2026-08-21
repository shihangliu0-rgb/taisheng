#include "path_main.h"

#include "auto_chassis.h"
#include "chassis_main.h"
#include "dt35_pnp_link.h"
#include "imu_main.h"
#include "laser_safety.h"

/* 调度与通信。 */
#define PATH_CONTROL_PERIOD_MS       10U
#define PATH_COMMAND_TIMEOUT_MS      50U
#define PATH_DT35_TIMEOUT_MS         500U
#define PATH_SEGMENT_SETTLE_MS       50U
#define PATH_REMOTE_PA0_MASK         (1U << 0U)
#define PATH_FRONT_SENSOR_INDEX      SENSOR_LINK_L_B_INDEX
#define PATH_LEFT_SENSOR_INDEX       SENSOR_LINK_F_INDEX

/* 场地距离，单位 cm。 */
#define PATH_LASER_STOP_CM           15U
#define PATH_FRONT_ARRIVE_CM         70U
#define PATH_LEFT_NEAR_CM            75U
#define PATH_LEFT_FAR_CM             200U
#define PATH_MIRROR_LEFT_CM          100U

/* 画弧：横移到触发线后进通道，前激光明显看开后再计算弧线进度。 */
#define PATH_ARC_ENABLE              1
#define PATH_LEFT_NEAR_ARC_CM        80U
#define PATH_LEFT_FAR_ARC_CM         160U
#define PATH_ARC_OPEN_CM             (PATH_FRONT_ARRIVE_CM + 20U)
#define PATH_ARC_FRONT_DROP_MAX_CM   15U

/* 平移控制。PID 周期直接由控制周期换算，不再单独维护参数。 */
#define PATH_SEGMENT_COUNT           4U
#define PATH_SPEED_MAX               190.0f
#define PATH_SPEED_MIN               50.0f
#define PATH_PID_KP                  2.9f
#define PATH_PID_KI                  1.2f
#define PATH_PID_KD                  0.05f
#define PATH_PID_I_LIMIT             30.0f
#define PATH_PID_DT_S                ((float)PATH_CONTROL_PERIOD_MS / 1000.0f)

/* 航向对准。 */
#define PATH_ALIGN_DONE_DEG          2.0f
#define PATH_ALIGN_TIMEOUT_MS        4000U
#define PATH_WP_ALIGN_DONE_DEG       3.0f
#define PATH_WP_ALIGN_TIMEOUT_MS     700U

typedef struct
{
    uint32_t last_rx_ms;
    uint16_t distance_cm;
    uint8_t online;
} path_dt35_t;

typedef struct
{
    float integral;
    float last_error;
    bool started;
} path_pid_t;

typedef enum
{
    PATH_ARC_SHIFT_TO_OPENING,
    PATH_ARC_ENTER_CHANNEL,
    PATH_ARC_RETURN_TO_LANE
} path_arc_phase_t;

typedef struct
{
    path_arc_phase_t phase;
    uint16_t front_peak_cm;
    uint16_t front_last_cm;
} path_arc_t;

static uint8_t path_pa0_previous;
static bool path_pa0_initialized;
static uint32_t path_last_control_ms;
static uint32_t path_segment_change_ms;
static path_pid_t path_pid;
static path_arc_t path_arc;
static uint8_t path_auto_start_count;
static bool path_yaw_aligning;
static float path_hold_yaw_deg;
static uint32_t path_align_start_ms;
static bool path_wp_yaw_pending;
static uint8_t path_wp_next_seg;

volatile path_state_t path_state = PATH_STATE_IDLE;
volatile path_error_t path_error = PATH_ERROR_NONE;
volatile uint8_t path_segment_index;
volatile bool path_mirrored;

static void PathMain_SetState(path_state_t state, path_error_t error)
{
    path_state = state;
    path_error = error;
}

static void PathMain_ResetPid(void)
{
    path_pid.integral = 0.0f;
    path_pid.last_error = 0.0f;
    path_pid.started = false;
}

static void PathMain_ResetArc(void)
{
    path_arc.phase = PATH_ARC_SHIFT_TO_OPENING;
    path_arc.front_peak_cm = 0U;
    path_arc.front_last_cm = 0U;
}

static void PathMain_LeaveAutomatic(path_state_t state,
                                    path_error_t error)
{
    path_yaw_aligning = false;
    path_wp_yaw_pending = false;
    PathMain_ResetArc();
    Chassis_ReleaseVelocity(CHASSIS_CMD_SOURCE_AUTONOMOUS);
    Chassis_SetControlMode(CHASSIS_CONTROL_MANUAL);
    (void)ImuMain_CaptureCurrentYaw();
    PathMain_SetState(state, error);
}

static void PathMain_Finish(void)
{
    path_yaw_aligning = false;
    path_wp_yaw_pending = false;
    PathMain_ResetArc();
    /* 终点不走200 ms停车斜坡，最迟1 ms内整组轮速清零。
     * 保持自动模式，防止遥控摇杆偏置在下一帧立即接管。 */
    Chassis_StopAll();
    (void)ImuMain_CaptureCurrentYaw();
    PathMain_SetState(PATH_STATE_FINISHED, PATH_ERROR_NONE);
}

static float PathMain_Absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float PathMain_WrapDeg(float angle_deg)
{
    while (angle_deg >= 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

/* 自动识别当前航向更靠近 0 还是 180，后续闭环都锁这个目标。 */
static float PathMain_SelectHoldYaw(float yaw_deg)
{
    float to_zero = PathMain_Absf(PathMain_WrapDeg(yaw_deg));
    float to_180 = PathMain_Absf(PathMain_WrapDeg(yaw_deg - 180.0f));

    return (to_zero <= to_180) ? 0.0f : 180.0f;
}

static void PathMain_HoldSelectedYaw(void)
{
    imu_data_t imu;

    if (!ImuMain_GetData(&imu))
    {
        return;
    }
    if (PathMain_Absf(PathMain_WrapDeg(imu.target_yaw_deg -
                                       path_hold_yaw_deg)) > 0.5f)
    {
        (void)ImuMain_SetTargetYaw(path_hold_yaw_deg);
    }
}

static bool PathMain_ImuReady(void)
{
    imu_data_t imu;

    return ImuMain_GetData(&imu) && imu.online && imu.yaw_valid &&
           imu.gyro_valid && (imu.state == IMU_STATE_READY);
}

static path_dt35_t PathMain_ReadDt35(uint8_t index)
{
    path_dt35_t sensor;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    sensor.last_rx_ms = dt35_link[index].last_rx_ms;
    sensor.distance_cm = dt35_link[index].distance_cm;
    sensor.online = dt35_link[index].online;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return sensor;
}

static bool PathMain_Dt35Fresh(const path_dt35_t *sensor,
                               uint32_t now_ms)
{
    return (sensor->online != 0U) &&
           ((uint32_t)(now_ms - sensor->last_rx_ms) <=
            PATH_DT35_TIMEOUT_MS);
}

static float PathMain_Clamp(float value, float low, float high)
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

/* 开关打开且提前量不为 0 才画弧，否则段 1 仍按原文件走。 */
static bool PathMain_ArcActive(void)
{
#if PATH_ARC_ENABLE
    if (path_mirrored)
    {
        return PATH_LEFT_NEAR_ARC_CM > PATH_LEFT_NEAR_CM;
    }
    return PATH_LEFT_FAR_ARC_CM < PATH_LEFT_FAR_CM;
#else
    return false;
#endif
}

static bool PathMain_ArcTrigger(const path_dt35_t *left)
{
    if (!PathMain_ArcActive())
    {
        return false;
    }
    if (path_mirrored)
    {
        return left->distance_cm <= PATH_LEFT_NEAR_ARC_CM;
    }
    return left->distance_cm >= PATH_LEFT_FAR_ARC_CM;
}

static int16_t PathMain_ArcAxis(float error_cm)
{
    float output;

    if (PathMain_Absf(error_cm) < 1.0f)
    {
        return 0;
    }
    output = PATH_PID_KP * error_cm;
    output = PathMain_Clamp(output, -PATH_SPEED_MAX, PATH_SPEED_MAX);
    return (int16_t)(output + ((output >= 0.0f) ? 0.5f : -0.5f));
}

static void PathMain_ArcCommand(const path_dt35_t *front,
                                const path_dt35_t *left,
                                int16_t *vx, int16_t *vy)
{
    float left_cm = (float)left->distance_cm;
    float front_cm = (float)front->distance_cm;
    float left_first = path_mirrored ?
                       (float)PATH_LEFT_NEAR_CM : (float)PATH_LEFT_FAR_CM;
    float left_final = path_mirrored ?
                       (float)PATH_LEFT_FAR_CM : (float)PATH_LEFT_NEAR_CM;
    float desired_left = left_first;
    float front_err = front_cm - (float)PATH_FRONT_ARRIVE_CM;
    float total_span;
    float clear_span;
    float return_span;
    float travelled;
    float alpha;

    *vx = 0;
    *vy = 0;

    /* 10 ms 内不可能真实缩短 15 cm。突降按遮挡处理：暂停且不推进弧线。 */
    if ((path_arc.front_last_cm != 0U) &&
        (path_arc.front_last_cm > front->distance_cm) &&
        ((uint16_t)(path_arc.front_last_cm - front->distance_cm) >
         PATH_ARC_FRONT_DROP_MAX_CM))
    {
        return;
    }
    path_arc.front_last_cm = front->distance_cm;

    if ((path_arc.phase == PATH_ARC_SHIFT_TO_OPENING) &&
        PathMain_ArcTrigger(left))
    {
        path_arc.phase = PATH_ARC_ENTER_CHANNEL;
    }

    if (path_arc.phase == PATH_ARC_SHIFT_TO_OPENING)
    {
        *vx = PathMain_ArcAxis(left_first - left_cm);
        return;
    }

    /* 开口只接受明显高于终点的读数，随后持续跟踪峰值。 */
    if (front->distance_cm >= PATH_ARC_OPEN_CM)
    {
        if (front->distance_cm > path_arc.front_peak_cm)
        {
            path_arc.front_peak_cm = front->distance_cm;
        }
    }
    if (path_arc.front_peak_cm == 0U)
    {
        *vx = PathMain_ArcAxis(left_first - left_cm);
        return;
    }

    total_span = (float)path_arc.front_peak_cm -
                 (float)PATH_FRONT_ARRIVE_CM;
    travelled = (float)path_arc.front_peak_cm - front_cm;
    if (travelled < 0.0f)
    {
        travelled = 0.0f;
    }

    /* 前1/3只进通道，后2/3平滑回到最终车道；不再依赖固定clear距离。 */
    clear_span = total_span / 3.0f;
    if ((path_arc.phase == PATH_ARC_ENTER_CHANNEL) &&
        (front->distance_cm > PATH_FRONT_ARRIVE_CM) &&
        (travelled >= clear_span))
    {
        path_arc.phase = PATH_ARC_RETURN_TO_LANE;
    }

    if (path_arc.phase == PATH_ARC_RETURN_TO_LANE)
    {
        return_span = total_span - clear_span;
        if (return_span < 1.0f)
        {
            return_span = 1.0f;
        }
        alpha = PathMain_Clamp((travelled - clear_span) / return_span,
                               0.0f, 1.0f);
        desired_left = left_first * (1.0f - alpha) + left_final * alpha;
    }

    *vx = PathMain_ArcAxis(desired_left - left_cm);
    if (front_err > 0.0f)
    {
        *vy = (int16_t)(PathMain_Clamp(PATH_PID_KP * front_err,
                                       0.0f, PATH_SPEED_MAX) + 0.5f);
    }
}

static int16_t PathMain_RunPid(float error_cm)
{
    float derivative = 0.0f;
    float output;

    if (error_cm <= 0.0f)
    {
        PathMain_ResetPid();
        return 0;
    }
    if (path_pid.started)
    {
        derivative = (error_cm - path_pid.last_error) / PATH_PID_DT_S;
    }
    else
    {
        path_pid.started = true;
    }
    path_pid.integral = PathMain_Clamp(
        path_pid.integral + PATH_PID_KI * error_cm * PATH_PID_DT_S,
        -PATH_PID_I_LIMIT, PATH_PID_I_LIMIT);
    path_pid.last_error = error_cm;
    output = PATH_PID_KP * error_cm + path_pid.integral +
             PATH_PID_KD * derivative;
    output = PathMain_Clamp(output, PATH_SPEED_MIN, PATH_SPEED_MAX);
    return (int16_t)(output + 0.5f);
}

static bool PathMain_SegmentArrived(const path_dt35_t *front,
                                    const path_dt35_t *left)
{
    switch (path_segment_index)
    {
    case 0U:
    case 2U:
        return front->distance_cm <= PATH_FRONT_ARRIVE_CM;

    case 1U:
    {
        bool first_done = path_mirrored ?
                          (left->distance_cm <= PATH_LEFT_NEAR_CM) :
                          (left->distance_cm >= PATH_LEFT_FAR_CM);
        bool final_done = path_mirrored ?
                          (left->distance_cm >= PATH_LEFT_FAR_CM) :
                          (left->distance_cm <= PATH_LEFT_NEAR_CM);

        /* 回到最终车道并到达通道终点，画弧路径才算完成。 */
        if (PathMain_ArcActive())
        {
            return (path_arc.phase == PATH_ARC_RETURN_TO_LANE) &&
                   final_done &&
                   (front->distance_cm <= PATH_FRONT_ARRIVE_CM);
        }
        return first_done;
    }

    case 3U:
        return path_mirrored ?
               (left->distance_cm >= PATH_LEFT_FAR_CM) :
               (left->distance_cm <= PATH_LEFT_NEAR_CM);

    default:
        return true;
    }
}

static void PathMain_GetSegmentCommand(const path_dt35_t *front,
                                       const path_dt35_t *left,
                                       int16_t *vx, int16_t *vy)
{
    float remaining_cm;

    *vx = 0;
    *vy = 0;
    switch (path_segment_index)
    {
    case 0U:
    case 2U:
        remaining_cm = (float)front->distance_cm -
                       (float)PATH_FRONT_ARRIVE_CM;
        *vy = PathMain_RunPid(remaining_cm);
        break;

    case 1U:
        if (PathMain_ArcActive())
        {
            PathMain_ArcCommand(front, left, vx, vy);
            break;
        }
        if (path_mirrored)
        {
            remaining_cm = (float)left->distance_cm -
                           (float)PATH_LEFT_NEAR_CM;
            *vx = (int16_t)-PathMain_RunPid(remaining_cm);
        }
        else
        {
            remaining_cm = (float)PATH_LEFT_FAR_CM -
                           (float)left->distance_cm;
            *vx = PathMain_RunPid(remaining_cm);
        }
        break;

    case 3U:
        if (path_mirrored)
        {
            remaining_cm = (float)PATH_LEFT_FAR_CM -
                           (float)left->distance_cm;
            *vx = PathMain_RunPid(remaining_cm);
        }
        else
        {
            remaining_cm = (float)left->distance_cm -
                           (float)PATH_LEFT_NEAR_CM;
            *vx = (int16_t)-PathMain_RunPid(remaining_cm);
        }
        break;

    default:
        break;
    }

    if ((front->distance_cm < PATH_LASER_STOP_CM) && (*vy > 0))
    {
        *vy = 0;
    }
    if ((left->distance_cm < PATH_LASER_STOP_CM) && (*vx < 0))
    {
        *vx = 0;
    }
}

static bool PathMain_Start(uint32_t now_ms)
{
    path_dt35_t front = PathMain_ReadDt35(PATH_FRONT_SENSOR_INDEX);
    path_dt35_t left = PathMain_ReadDt35(PATH_LEFT_SENSOR_INDEX);
    imu_data_t imu;

    if (!PathMain_ImuReady() || !ImuMain_GetData(&imu))
    {
        PathMain_SetState(PATH_STATE_FAULT, PATH_ERROR_IMU);
        return false;
    }
    if (!PathMain_Dt35Fresh(&front, now_ms) ||
        !PathMain_Dt35Fresh(&left, now_ms))
    {
        PathMain_SetState(PATH_STATE_FAULT, PATH_ERROR_DT35);
        return false;
    }

    path_hold_yaw_deg = PathMain_SelectHoldYaw(imu.yaw_deg);
    /* 先使能再设置目标，避免使能动作清掉本次选择的 0/180 航向。 */
    ImuMain_EnableYawHold(true);
    if (ImuMain_SetTargetYaw(path_hold_yaw_deg) != HAL_OK)
    {
        PathMain_SetState(PATH_STATE_FAULT, PATH_ERROR_IMU);
        return false;
    }

    path_mirrored = left.distance_cm >= PATH_MIRROR_LEFT_CM;
    path_segment_index = 0U;
    path_segment_change_ms = now_ms;
    path_last_control_ms = now_ms - PATH_CONTROL_PERIOD_MS;
    PathMain_ResetPid();

    /* 第一次上电默认已对准，再按才先原地转到 0/180，避免对准拖慢速度。 */
    path_auto_start_count++;
    path_yaw_aligning = (path_auto_start_count > 1U);
    path_align_start_ms = now_ms;
    path_wp_yaw_pending = false;
    PathMain_ResetArc();

    AutoChassis_Stop();
    Chassis_SetControlMode(CHASSIS_CONTROL_AUTONOMOUS);
    PathMain_SetState(PATH_STATE_RUNNING, PATH_ERROR_NONE);
    return true;
}

static void PathMain_RunAlign(uint32_t now_ms)
{
    imu_data_t imu;
    float error_deg;

    if (!PathMain_ImuReady() || !ImuMain_GetData(&imu))
    {
        PathMain_LeaveAutomatic(PATH_STATE_FAULT, PATH_ERROR_IMU);
        return;
    }

    PathMain_HoldSelectedYaw();
    /* z=0，让 Chassis_Run1ms 里的 ImuMain_CalcOmega 原地转到目标航向。 */
    if (Chassis_RequestVelocity(CHASSIS_CMD_SOURCE_AUTONOMOUS,
                                0, 0, 0,
                                PATH_COMMAND_TIMEOUT_MS) != HAL_OK)
    {
        PathMain_LeaveAutomatic(PATH_STATE_FAULT, PATH_ERROR_CHASSIS);
        return;
    }

    error_deg = PathMain_WrapDeg(imu.yaw_deg - path_hold_yaw_deg);
    {
        float done_deg = path_wp_yaw_pending ? PATH_WP_ALIGN_DONE_DEG : PATH_ALIGN_DONE_DEG;
        uint32_t timeout = path_wp_yaw_pending ? PATH_WP_ALIGN_TIMEOUT_MS : PATH_ALIGN_TIMEOUT_MS;
        if ((PathMain_Absf(error_deg) <= done_deg) ||
            ((uint32_t)(now_ms - path_align_start_ms) >= timeout))
        {
            path_yaw_aligning = false;
            path_segment_change_ms = now_ms;
            PathMain_ResetPid();
            /* 拐点yaw纠正完成，再切到下一段 */
            if (path_wp_yaw_pending)
            {
                path_wp_yaw_pending = false;
                path_segment_index = path_wp_next_seg;
                PathMain_ResetArc();
                if (path_segment_index >= PATH_SEGMENT_COUNT)
                {
                    PathMain_Finish();
                }
            }
        }
    }
}

static void PathMain_RunControl(uint32_t now_ms)
{
    path_dt35_t front = PathMain_ReadDt35(PATH_FRONT_SENSOR_INDEX);
    path_dt35_t left = PathMain_ReadDt35(PATH_LEFT_SENSOR_INDEX);
    int16_t vx;
    int16_t vy;

    if (!PathMain_ImuReady())
    {
        PathMain_LeaveAutomatic(PATH_STATE_FAULT, PATH_ERROR_IMU);
        return;
    }
    if (!PathMain_Dt35Fresh(&front, now_ms) ||
        !PathMain_Dt35Fresh(&left, now_ms))
    {
        PathMain_LeaveAutomatic(PATH_STATE_FAULT, PATH_ERROR_DT35);
        return;
    }

    /* 全程锁 0/180，防止段间停车时底盘把目标改成当前角。 */
    PathMain_HoldSelectedYaw();

    if (PathMain_SegmentArrived(&front, &left))
    {
        Chassis_ReleaseVelocity(CHASSIS_CMD_SOURCE_AUTONOMOUS);
        /* 每到拐点先检查一边并原地yaw纠正，再进下一段 */
        if (!path_wp_yaw_pending && !path_yaw_aligning)
        {
            imu_data_t imu;
            if (ImuMain_GetData(&imu))
            {
                path_hold_yaw_deg = PathMain_SelectHoldYaw(imu.yaw_deg);
                (void)ImuMain_SetTargetYaw(path_hold_yaw_deg);
            }
            if ((path_segment_index == 1U) && PathMain_ArcActive())
            {
                path_wp_next_seg = PATH_SEGMENT_COUNT;
            }
            else
            {
                path_wp_next_seg = path_segment_index + 1U;
            }
            if (path_wp_next_seg >= PATH_SEGMENT_COUNT)
            {
                path_segment_index = PATH_SEGMENT_COUNT;
                PathMain_Finish();
                return;
            }
            /* 若已在3°内则不额外对准，直接切段 */
            {
                imu_data_t imu2;
                float err = 180.0f;
                if (ImuMain_GetData(&imu2))
                {
                    err = PathMain_Absf(PathMain_WrapDeg(imu2.yaw_deg - path_hold_yaw_deg));
                }
                if (err <= PATH_WP_ALIGN_DONE_DEG)
                {
                    path_segment_index = path_wp_next_seg;
                    PathMain_ResetArc();
                    path_segment_change_ms = now_ms;
                    PathMain_ResetPid();
                    if (path_segment_index >= PATH_SEGMENT_COUNT)
                    {
                        PathMain_Finish();
                    }
                    return;
                }
            }
            path_wp_yaw_pending = true;
            path_yaw_aligning = true;
            path_align_start_ms = now_ms;
            /* 下一周期进 RunAlign 原地纠正 */
            return;
        }
        return;
    }
    if ((uint32_t)(now_ms - path_segment_change_ms) <
        PATH_SEGMENT_SETTLE_MS)
    {
        Chassis_ReleaseVelocity(CHASSIS_CMD_SOURCE_AUTONOMOUS);
        return;
    }

    PathMain_GetSegmentCommand(&front, &left, &vx, &vy);
    /* z=0，始终走底盘 ImuMain_CalcOmega，把航向保持在 0/180。 */
    if (LaserSafety_RequestVelocity(CHASSIS_CMD_SOURCE_AUTONOMOUS,
                                    vx, vy, 0,
                                    PATH_COMMAND_TIMEOUT_MS) != HAL_OK)
    {
        PathMain_LeaveAutomatic(PATH_STATE_FAULT,
                                PATH_ERROR_CHASSIS);
    }
}

void PathMain_Init(void)
{
    path_segment_index = 0U;
    path_mirrored = false;
    path_pa0_previous = 0U;
    path_pa0_initialized = false;
    path_last_control_ms = HAL_GetTick();
    path_segment_change_ms = path_last_control_ms;
    path_auto_start_count = 0U;
    path_yaw_aligning = false;
    path_hold_yaw_deg = 0.0f;
    path_align_start_ms = path_last_control_ms;
    path_wp_yaw_pending = false;
    path_wp_next_seg = 0U;
    PathMain_ResetArc();
    PathMain_ResetPid();
    PathMain_SetState(PATH_STATE_IDLE, PATH_ERROR_NONE);
}

void PathMain_Run(uint8_t remote_buttons, uint8_t remote_online)
{
    uint32_t now_ms = HAL_GetTick();
    uint8_t pa0 = ((remote_buttons & PATH_REMOTE_PA0_MASK) != 0U) ?
                  1U : 0U;

    if (remote_online == 0U)
    {
        path_pa0_initialized = false;
        path_pa0_previous = 0U;
        if (path_state == PATH_STATE_RUNNING)
        {
            PathMain_LeaveAutomatic(PATH_STATE_FAULT,
                                    PATH_ERROR_REMOTE);
        }
        return;
    }

    if (!path_pa0_initialized)
    {
        path_pa0_previous = pa0;
        path_pa0_initialized = true;
    }
    else if ((pa0 != 0U) && (path_pa0_previous == 0U))
    {
        if (path_state == PATH_STATE_RUNNING)
        {
            PathMain_LeaveAutomatic(PATH_STATE_IDLE, PATH_ERROR_NONE);
        }
        else
        {
            (void)PathMain_Start(now_ms);
        }
    }
    path_pa0_previous = pa0;

    if (path_state != PATH_STATE_RUNNING)
    {
        return;
    }
    if (Chassis_GetControlMode() != CHASSIS_CONTROL_AUTONOMOUS)
    {
        PathMain_LeaveAutomatic(PATH_STATE_IDLE, PATH_ERROR_MANUAL);
        return;
    }
    if ((uint32_t)(now_ms - path_last_control_ms) <
        PATH_CONTROL_PERIOD_MS)
    {
        return;
    }
    path_last_control_ms = now_ms;
    if (path_yaw_aligning)
    {
        PathMain_RunAlign(now_ms);
        return;
    }
    PathMain_RunControl(now_ms);
}

void PathMain_Stop(void)
{
    PathMain_LeaveAutomatic(PATH_STATE_IDLE, PATH_ERROR_NONE);
}

bool PathMain_IsRunning(void)
{
    return path_state == PATH_STATE_RUNNING;
}
