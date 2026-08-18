#include "path.h"

#include "chassis_main.h"
#include "dt35_pnp_link.h"
#include "imu_main.h"
#include "path_line_imu.h"
#include "path_localization.h"
#include "path_safety.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PATH_REMOTE_TIMEOUT_MS          200U
#define PATH_NEUTRAL_COMMAND_THRESHOLD  3
#define PATH_PI                         3.14159265358979323846f
#define PATH_COMMAND_TO_MPS             \
    (PATH_PI * PATH_LINE_IMU_WHEEL_DIAMETER_M / 60.0f)
#define PATH_REACTION_TIME_S            0.080f
#define PATH_BRAKE_DECELERATION_MPS2    2.000f
#define PATH_MAP_BRAKE_DECELERATION_MPS2 1.600f
#define PATH_FRONT_BASE_DISTANCE_M      0.120f
/*
 * 左光基础净距：读数现从车体最左端面起算。旧值 0.10 m 是按传感器
 * 凹进车侧 4.5 cm 的安装标定的，隐含的"车侧面到墙"物理净距约
 * 5.5 cm；面装后直接取该物理净距，否则左光会把机器人挡在
 * x ≥ 0.324+0.045，去程段 3 终点 x=0.36 附近的贴西墙走廊无法到达。
 */
#define PATH_LEFT_BASE_DISTANCE_M       0.055f
#define PATH_BLOCK_RELEASE_HYSTERESIS_M 0.020f
#define PATH_LASER_MIN_CM               5U
/*
 * DT35 量程（u_dt35+PNP 子板示教窗口）：前激光 5–140 cm，
 * 左激光 5–240 cm；两只激光装在车体最前端/最左端，读数从车体
 * 边缘起算（path_localization.h 的偏移即半车长/半车宽）。
 * 左激光 2.4 m 量程覆盖整个场地宽度的大部分：常规侧与镜像侧
 * 都能在起点静止读到西墙，锚定与侧别识别统一由左光完成。
 */
#define PATH_FRONT_LASER_MAX_CM         140U
#define PATH_LEFT_LASER_MAX_CM          240U
#define PATH_HARD_STOP_SPEED_MPS        0.030f
#define PATH_MAP_HARD_STOP_DISTANCE_M   0.010f
#define PATH_LASER_FILTER_SIZE          3U
#define PATH_INITIAL_SAMPLE_COUNT       3U
/*
 * 自动行驶参数：速度档取 lora_link.c 遥控同款（快杆 150 / 精调 75），
 * 距段终点 0.5 m 内换精调档；任意遥控摇杆超过 10（约 0.08 m/s）或
 * 任意按键按下即判定人工接管。到点提示音 160 ms（与开机提示音
 * 相同的 1 ms 翻转驱动）。
 */
#define PATH_AUTO_FAST_COMMAND          150
#define PATH_AUTO_FINE_COMMAND          75
#define PATH_AUTO_FINE_DISTANCE_M       0.800f
#define PATH_AUTO_TAKEOVER_COMMAND      10
#define PATH_AUTO_BEEP_MS               160U
/*
 * 段终点滑停窗口：段号推进后自动指令清零 400 ms，让上一段的残余
 * 速度先刹掉再走下一段——段终点不再强停后，直角换向的动量会把
 * 机器人推向墙侧（DT35 大量程下巡航速度更高，过冲更明显）。
 */
#define PATH_AUTO_SETTLE_MS             400U

typedef struct
{
    int16_t vx;
    int16_t vy;
    int16_t vz;
    uint8_t buttons;
    uint32_t timestamp_ms;
    bool online;
    uint32_t sequence;
} path_remote_snapshot_t;

typedef struct
{
    uint16_t sample_cm[PATH_LASER_FILTER_SIZE];
    uint8_t count;
    uint8_t write_index;
    uint32_t last_rx_ms;
    uint16_t filtered_cm;
    bool online_previous;
} path_laser_filter_t;

typedef struct
{
    uint16_t sample_cm[PATH_INITIAL_SAMPLE_COUNT];
    uint8_t count;
    uint32_t last_rx_ms;
    bool online_previous;
} path_initial_sampler_t;

/* 单写者是通信任务；1 ms 底盘任务通过 sequence seqlock 读取。 */
static volatile int16_t path_remote_vx;
static volatile int16_t path_remote_vy;
static volatile int16_t path_remote_vz;
static volatile uint8_t path_remote_buttons;
static volatile uint32_t path_remote_timestamp_ms;
static volatile uint8_t path_remote_online;
static volatile uint32_t path_remote_sequence;

/* 供 LoRa 原有 Chassis_SetVelocity 语句和无小电脑拦截读取。 */
static volatile int16_t path_last_output_vx;
static volatile int16_t path_last_output_vy;
static volatile int16_t path_last_output_z;

static path_diagnostics_t path_diagnostics;
static path_laser_filter_t path_front_filter;
static path_laser_filter_t path_left_filter;
static path_initial_sampler_t path_front_initial_sampler;
static path_initial_sampler_t path_left_initial_sampler;
static uint32_t path_processed_remote_sequence;
static float path_map_origin_x_m;
static float path_map_origin_y_m;
static float path_localization_yaw_deg;
static bool path_localization_yaw_valid;
static bool path_yaw_was_ready;
static bool path_front_blocked;
static bool path_left_blocked;
static bool path_auto_button_armed;
static bool path_auto_triggered;
static uint8_t path_auto_last_segment;
static uint32_t path_auto_segment_change_ms;
static volatile uint16_t path_beep_counter_ms;
/*
 * 到点交接标志：自动行驶到达终点后置位且不再清除（一次性测试）。
 * 交接后本层退为透传——不再读融合里程计（freertos 任务同步停跑
 * PathLineImu_Run1ms）、关闭地图净空保护、放开肩键、按键 1/2 失效，
 * 仅保留 DT35 动态激光限速。
 */
static bool path_handover;

static int16_t Path_AbsCommand(int16_t value)
{
    if (value == INT16_MIN)
    {
        return INT16_MAX;
    }
    return (value < 0) ? (int16_t)-value : value;
}

static float Path_MaxFloat(float lhs, float rhs)
{
    return (lhs > rhs) ? lhs : rhs;
}

static float Path_AngleDiffDeg(float lhs_deg, float rhs_deg)
{
    float diff = fmodf(lhs_deg - rhs_deg, 360.0f);

    if (diff > 180.0f)
    {
        diff -= 360.0f;
    }
    else if (diff < -180.0f)
    {
        diff += 360.0f;
    }
    return diff;
}

/*
 * 车体系与地图系互转。Chassis_SetVelocity 的 vx/vy 是车体系
 * （chassis_main.c："机器人坐标系：X 向右、Y 向前"），掉头 180°
 * 后车体系相对地图系整体取反；所有涉及地图几何的计算都必须先用
 * 实测 yaw 把命令旋到地图系，反之亦然。yaw 无效时回退到锁定目标
 * 角（去程 0°/回程 180°）。
 */
static float Path_CommandYawDeg(void)
{
    if (path_localization_yaw_valid)
    {
        return path_localization_yaw_deg;
    }
    return PATH_MAP_LOCK_YAW_DEG;
}

static void Path_RotateVector(float x, float y, float yaw_deg,
                              float *out_x, float *out_y)
{
    float yaw_rad = yaw_deg * (PATH_PI / 180.0f);
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    *out_x = cos_yaw * x - sin_yaw * y;
    *out_y = sin_yaw * x + cos_yaw * y;
}

static uint16_t Path_ClampLaserCm(uint16_t distance_cm, uint16_t max_cm)
{
    if (distance_cm < PATH_LASER_MIN_CM)
    {
        return PATH_LASER_MIN_CM;
    }
    if (distance_cm > max_cm)
    {
        return max_cm;
    }
    return distance_cm;
}

static uint16_t Path_ConservativeLaser(const path_laser_filter_t *filter)
{
    uint16_t minimum;
    uint8_t index;

    if (filter->count == 0U)
    {
        return 0U;
    }

    minimum = filter->sample_cm[0];
    for (index = 1U; index < filter->count; index++)
    {
        if (filter->sample_cm[index] < minimum)
        {
            minimum = filter->sample_cm[index];
        }
    }
    return minimum;
}

static uint16_t Path_UpdateLaserFilter(path_laser_filter_t *filter,
                                       uint32_t last_rx_ms,
                                       uint16_t distance_cm,
                                       uint16_t max_cm,
                                       bool online)
{
    if (!online)
    {
        filter->online_previous = false;
        return filter->filtered_cm;
    }

    distance_cm = Path_ClampLaserCm(distance_cm, max_cm);
    if (!filter->online_previous)
    {
        filter->count = 0U;
        filter->write_index = 0U;
        filter->online_previous = true;
    }
    if ((filter->count == 0U) || (last_rx_ms != filter->last_rx_ms))
    {
        filter->sample_cm[filter->write_index] = distance_cm;
        filter->write_index = (uint8_t)((filter->write_index + 1U) %
                                        PATH_LASER_FILTER_SIZE);
        if (filter->count < PATH_LASER_FILTER_SIZE)
        {
            filter->count++;
        }
        filter->last_rx_ms = last_rx_ms;
        /* 距离减小时首帧生效；增大需连续覆盖旧样本，避免过早解除。 */
        filter->filtered_cm = Path_ConservativeLaser(filter);
    }
    return filter->filtered_cm;
}

static void Path_CollectInitialSample(path_initial_sampler_t *sampler,
                                      uint16_t distance_cm,
                                      uint32_t rx_ms,
                                      bool online)
{
    if (!online)
    {
        sampler->count = 0U;
        sampler->online_previous = false;
        return;
    }

    /*
     * DT35 子板超量程时输出钳在量程上限；饱和读数视为"没有目标"，
     * 不参与初始定点。左光 2.4 m 量程下，常规侧（贴西墙 ~10 cm）
     * 与镜像侧（贴东墙、到西墙 ~2.36 m）都能读到西墙。
     */
    if (Path_ClampLaserCm(distance_cm, PATH_LEFT_LASER_MAX_CM) >=
        PATH_LEFT_LASER_MAX_CM)
    {
        return;
    }

    if (!sampler->online_previous)
    {
        sampler->count = 0U;
        sampler->last_rx_ms = rx_ms;
        sampler->online_previous = true;
        sampler->sample_cm[sampler->count++] =
            Path_ClampLaserCm(distance_cm, PATH_LEFT_LASER_MAX_CM);
        return;
    }

    if ((sampler->count < PATH_INITIAL_SAMPLE_COUNT) &&
        (rx_ms != sampler->last_rx_ms))
    {
        sampler->last_rx_ms = rx_ms;
        sampler->sample_cm[sampler->count++] =
            Path_ClampLaserCm(distance_cm, PATH_LEFT_LASER_MAX_CM);
    }
}

static uint16_t Path_InitialMedian(const path_initial_sampler_t *sampler)
{
    uint16_t a = sampler->sample_cm[0];
    uint16_t b = sampler->sample_cm[1];
    uint16_t c = sampler->sample_cm[2];
    uint16_t swap;

    if (a > b)
    {
        swap = a;
        a = b;
        b = swap;
    }
    if (b > c)
    {
        swap = b;
        b = c;
        c = swap;
    }
    return (a > b) ? a : b;
}

static void Path_WriteRemoteMailbox(int16_t vx, int16_t vy, int16_t vz,
                                    uint8_t buttons, uint32_t now_ms,
                                    bool online)
{
    uint32_t sequence = path_remote_sequence;

    path_remote_sequence = sequence + 1U;
    __DMB();
    path_remote_vx = vx;
    path_remote_vy = vy;
    path_remote_vz = vz;
    path_remote_buttons = buttons;
    path_remote_timestamp_ms = now_ms;
    path_remote_online = online ? 1U : 0U;
    __DMB();
    path_remote_sequence = sequence + 2U;
}

static void Path_ReadRemoteMailbox(path_remote_snapshot_t *snapshot)
{
    uint32_t sequence_before;
    uint32_t sequence_after;

    do
    {
        sequence_before = path_remote_sequence;
        __DMB();
        snapshot->vx = path_remote_vx;
        snapshot->vy = path_remote_vy;
        snapshot->vz = path_remote_vz;
        snapshot->buttons = path_remote_buttons;
        snapshot->timestamp_ms = path_remote_timestamp_ms;
        snapshot->online = path_remote_online != 0U;
        __DMB();
        sequence_after = path_remote_sequence;
    } while (((sequence_before & 1U) != 0U) ||
             (sequence_before != sequence_after));
    snapshot->sequence = sequence_after;
}

/* 六键处理：仅保留按键 3（自动行驶启动接口），其余键预留。 */
static bool Path_ProcessModeButton(const path_remote_snapshot_t *remote)
{
    bool auto_pressed;

    if (remote->sequence == path_processed_remote_sequence)
    {
        return false;
    }
    path_processed_remote_sequence = remote->sequence;

    if (!remote->online)
    {
        path_auto_button_armed = false;
        return false;
    }
    /* 到点交接后按键功能关闭。 */
    if (path_handover)
    {
        return false;
    }
    auto_pressed = (remote->buttons & PATH_REMOTE_AUTO_BUTTON_BIT) != 0U;
    if (!auto_pressed)
    {
        path_auto_button_armed = true;
    }
    else if (path_auto_button_armed)
    {
        path_auto_button_armed = false;
        Path_AutoStartTrigger();
    }
    return false;
}

/*
 * 镜像侧锚定：镜像场地西边界在南墙与墙 1' 之间是通道开口，起始区
 * 左光朝西没有任何回波（在线但收不到样本）；而前光在向北行进约
 * 6 cm 后即可读到第一堵前墙（贴东墙的墙 B'，南面 y = 2.075，起点
 * 车头面距其约 1.458 m，略超 1.4 m 量程）。因此判定条件为：
 * 左光在线且无初始样本 + 前光读数进入量程（< 140 cm）→ 镜像侧。
 * X 用假定贴东墙起点 2.626 m，Y 用车头面位置反算：
 * y = 2.075 − 半车长 − d_front。常规侧左光必有近距样本，不会进入
 * 本分支；左光离线（故障）时不做镜像判定，避免误锚。
 */
static void Path_TrySetMirroredInitialPosition(
    const path_line_imu_data_t *odometry)
{
    float front_distance_m;
    float map_y_m;

    if (path_diagnostics.initial_position_valid ||
        !path_diagnostics.left_laser_online ||
        (path_left_initial_sampler.count > 0U) ||
        !path_diagnostics.front_laser_online ||
        (path_diagnostics.front_distance_cm >= PATH_FRONT_LASER_MAX_CM))
    {
        return;
    }

    front_distance_m = (float)path_diagnostics.front_distance_cm * 0.01f;
    map_y_m = PATH_MAP_MIRRORED_FIRST_WALL_Y_M -
              PATH_LOCALIZATION_FRONT_SENSOR_OFFSET_M -
              front_distance_m;

    PathMap_SetMirrored(true);
    path_map_origin_x_m = PATH_MAP_MIRRORED_START_X_M -
                          odometry->fused_position_x_m;
    path_map_origin_y_m = map_y_m - odometry->fused_position_y_m;
    path_diagnostics.initial_position_valid = true;
    path_diagnostics.initial_map_x_m = PATH_MAP_MIRRORED_START_X_M;
    path_diagnostics.initial_map_y_m = map_y_m;
    path_diagnostics.initial_yaw_deg = path_localization_yaw_valid ?
                                       path_localization_yaw_deg : 0.0f;
    /* 镜像侧左光无回波，前光距离成为有效初始信息。 */
    path_diagnostics.front_initial_distance_m = front_distance_m;
    path_diagnostics.left_initial_distance_m = 0.0f;
    path_diagnostics.front_wall_hit_x_m = 0.0f;
    path_diagnostics.left_wall_hit_y_m = 0.0f;
}

static void Path_TrySetInitialPosition(
    const path_line_imu_data_t *odometry)
{
    path_localization_result_t result;
    float left_distance_m;

    if (path_diagnostics.initial_position_valid ||
        !path_localization_yaw_valid ||
        (path_left_initial_sampler.count < PATH_INITIAL_SAMPLE_COUNT))
    {
        return;
    }

    left_distance_m =
        (float)Path_InitialMedian(&path_left_initial_sampler) * 0.01f;
    if (!PathLocalization_Calculate(0.0f, left_distance_m,
                                    path_localization_yaw_deg, &result))
    {
        path_diagnostics.initial_position_reject_count++;
        path_left_initial_sampler.count = 0U;
        return;
    }

    /*
     * 侧别兜底：常规侧贴西墙读数 ~10 cm → 锚定 x≈0.37。真实镜像
     * 场地起始区西边界是通道开口，左光无回波，走前光镜像锚定；
     * 但若场地为封闭变体（西侧有墙），左光读到 ~2.36 m 时锚定 X
     * 会越过半场，此处仍正确切换镜像侧。
     */
    PathMap_SetMirrored(result.map_x_m >
                        (0.5f * PATH_MAP_FIELD_WIDTH_M));

    /* 只锚定一次；之后地图坐标完全跟随现有融合里程计。 */
    path_map_origin_x_m = result.map_x_m - odometry->fused_position_x_m;
    path_map_origin_y_m = result.map_y_m - odometry->fused_position_y_m;
    path_diagnostics.initial_position_valid = true;
    path_diagnostics.initial_map_x_m = result.map_x_m;
    path_diagnostics.initial_map_y_m = result.map_y_m;
    path_diagnostics.initial_yaw_deg = path_localization_yaw_deg;
    path_diagnostics.front_initial_distance_m = 0.0f;
    path_diagnostics.left_initial_distance_m = left_distance_m;
    path_diagnostics.front_wall_hit_x_m = 0.0f;
    path_diagnostics.left_wall_hit_y_m = result.left_wall_hit_y_m;
}

static bool Path_UpdateOdometryAndRoute(void)
{
    path_line_imu_data_t odometry;
    const path_map_route_segment_t *route;
    uint8_t route_count;
    bool automatic_stop = false;

    path_diagnostics.odometry_valid =
        PathLineImu_GetData(&odometry) &&
        (odometry.imu_solution_valid || odometry.encoder_solution_valid);
    if (!path_diagnostics.odometry_valid)
    {
        return false;
    }

    path_diagnostics.encoder_velocity_x_mps =
        odometry.encoder_solution_valid ?
        odometry.encoder_body_velocity_x_mps :
        odometry.fused_velocity_x_mps;
    path_diagnostics.encoder_velocity_y_mps =
        odometry.encoder_solution_valid ?
        odometry.encoder_body_velocity_y_mps :
        odometry.fused_velocity_y_mps;

    Path_TrySetMirroredInitialPosition(&odometry);
    Path_TrySetInitialPosition(&odometry);
    if (!path_diagnostics.initial_position_valid)
    {
        return false;
    }

    path_diagnostics.map_x_m = path_map_origin_x_m +
                               odometry.fused_position_x_m;
    path_diagnostics.map_y_m = path_map_origin_y_m +
                               odometry.fused_position_y_m;

    route = PathMap_GetRoute(&route_count);
    while ((path_diagnostics.segment_index < route_count) &&
           PathMap_SegmentReached(path_diagnostics.segment_index,
                                  path_diagnostics.map_x_m,
                                  path_diagnostics.map_y_m))
    {
        path_diagnostics.segment_index++;
    }

    path_diagnostics.route_complete =
        path_diagnostics.segment_index >= route_count;

    if (!path_diagnostics.route_complete)
    {
        path_diagnostics.active_axis =
            route[path_diagnostics.segment_index].axis;
    }
    return automatic_stop;
}

static void Path_UpdateYawZeroLock(void)
{
    imu_data_t imu;
    bool assert_target;
    bool ready = ImuMain_GetData(&imu) &&
                 (imu.state == IMU_STATE_READY) && imu.online &&
                 imu.yaw_valid && !isnan(imu.yaw_deg) &&
                 !isinf(imu.yaw_deg);

    if (ready)
    {
        /* 初始定点使用锁零前这一周期实际测得的 yaw。 */
        path_localization_yaw_deg = imu.yaw_deg;
        path_localization_yaw_valid = true;
    }
    else
    {
        path_localization_yaw_valid = false;
    }

    /*
     * 行驶阶段持续锁 yaw = 0°；到点交接后不再干预航向保持
     * （肩键旋转由 CalcOmega 自行记录并保持在新航向）。
     */
    assert_target = !path_yaw_was_ready || !imu.yaw_hold_enabled ||
                    (fabsf(Path_AngleDiffDeg(imu.target_yaw_deg,
                                             PATH_MAP_LOCK_YAW_DEG)) >
                     0.5f);
    if (path_handover)
    {
        assert_target = false;
    }
    if (ready && assert_target)
    {
        ImuMain_EnableYawHold(true);
        if (ImuMain_SetTargetYaw(PATH_MAP_LOCK_YAW_DEG) != HAL_OK)
        {
            ready = false;
        }
    }
    path_yaw_was_ready = ready;
    path_diagnostics.yaw_zero_lock_ready = ready;
}

static void Path_UpdateLaserData(void)
{
    uint32_t front_last_rx;
    uint32_t left_last_rx;
    uint32_t primask;
    uint16_t front_cm;
    uint16_t left_cm;
    bool front_online;
    bool left_online;

    /* 数据由 UART 中断逐字段更新，短临界区避免拼出跨帧快照。 */
    primask = __get_PRIMASK();
    __disable_irq();
    front_last_rx = dt35_link[SENSOR_LINK_F_INDEX].last_rx_ms;
    left_last_rx = dt35_link[SENSOR_LINK_L_B_INDEX].last_rx_ms;
    front_cm = dt35_link[SENSOR_LINK_F_INDEX].distance_cm;
    left_cm = dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm;
    front_online = dt35_link[SENSOR_LINK_F_INDEX].online != 0U;
    left_online = dt35_link[SENSOR_LINK_L_B_INDEX].online != 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    path_diagnostics.front_laser_online = front_online;
    path_diagnostics.left_laser_online = left_online;
    if (!path_diagnostics.initial_position_valid)
    {
        /* 初始 Y 固定；前 DT35 只保留运行期 +Y 动态安全职责。 */
        Path_CollectInitialSample(&path_left_initial_sampler, left_cm,
                                  left_last_rx, left_online);
    }

    path_diagnostics.front_initial_sample_count = 0U;
    path_diagnostics.left_initial_sample_count =
        path_left_initial_sampler.count;
    path_diagnostics.front_distance_cm =
        Path_UpdateLaserFilter(&path_front_filter, front_last_rx,
                               front_cm, PATH_FRONT_LASER_MAX_CM,
                               front_online);
    path_diagnostics.left_distance_cm =
        Path_UpdateLaserFilter(&path_left_filter, left_last_rx,
                               left_cm, PATH_LEFT_LASER_MAX_CM,
                               left_online);

    if (front_online)
    {
        float front_distance_m =
            (float)path_diagnostics.front_distance_cm * 0.01f;
        if (front_distance_m <= PATH_FRONT_BASE_DISTANCE_M)
        {
            path_front_blocked = true;
        }
        else if (front_distance_m >=
                 (PATH_FRONT_BASE_DISTANCE_M +
                  PATH_BLOCK_RELEASE_HYSTERESIS_M))
        {
            path_front_blocked = false;
        }
    }
    else
    {
        /* 用户明确要求激光离线不停机；仅在诊断中保留离线状态。 */
        path_front_blocked = false;
    }

    if (left_online)
    {
        float left_distance_m =
            (float)path_diagnostics.left_distance_cm * 0.01f;
        if (left_distance_m <= PATH_LEFT_BASE_DISTANCE_M)
        {
            path_left_blocked = true;
        }
        else if (left_distance_m >=
                 (PATH_LEFT_BASE_DISTANCE_M +
                  PATH_BLOCK_RELEASE_HYSTERESIS_M))
        {
            path_left_blocked = false;
        }
    }
    else
    {
        path_left_blocked = false;
    }

    path_diagnostics.front_hard_blocked = path_front_blocked;
    path_diagnostics.left_hard_blocked = path_left_blocked;
}

static bool Path_ApplyMapLimit(int16_t *vx, int16_t *vy)
{
    float magnitude;
    float allowed_speed_mps;
    float map_dir_x;
    float map_dir_y;
    float command_yaw_deg;
    bool hard_stop = false;

    path_diagnostics.map_speed_limited = false;
    path_diagnostics.map_clearance_m = PATH_MAP_CLEARANCE_NONE_M;
    if (!path_diagnostics.odometry_valid ||
        !path_diagnostics.initial_position_valid ||
        ((*vx == 0) && (*vy == 0)))
    {
        return false;
    }

    magnitude = sqrtf((float)*vx * (float)*vx +
                      (float)*vy * (float)*vy);
    /*
     * vx/vy 是车体系命令，地图净空必须沿地图系方向查询：掉头 180°
     * 后车体系相对地图系取反，直接用车体方向会把净空查到运动的
     * 反方向——回程时对真实运动方向完全失去静态墙保护，反而在
     * 安全方向上误触发限速/硬停。
     */
    command_yaw_deg = Path_CommandYawDeg();
    Path_RotateVector((float)*vx / magnitude, (float)*vy / magnitude,
                      command_yaw_deg, &map_dir_x, &map_dir_y);
    path_diagnostics.map_clearance_m =
        PathMap_RayClearance(path_diagnostics.map_x_m,
                             path_diagnostics.map_y_m,
                             map_dir_x, map_dir_y);

    /*
     * 沿墙滑动：主方向被挡（例如中心已在膨胀带内，命令带有极小的
     * 向墙分量）时，把命令旋到地图系只保留主轴分量再查一次净空；
     * 可行则以投影后的命令继续（清除向墙分量），不可行才硬停。
     * 否则一次越界就会把机器人锁死在膨胀带内。
     */
    if (path_diagnostics.map_clearance_m <= PATH_MAP_HARD_STOP_DISTANCE_M)
    {
        float map_cmd_x;
        float map_cmd_y;
        float body_cmd_x;
        float body_cmd_y;
        float slide_clearance_m;

        Path_RotateVector((float)*vx, (float)*vy, command_yaw_deg,
                          &map_cmd_x, &map_cmd_y);
        if (fabsf(map_cmd_x) >= fabsf(map_cmd_y))
        {
            map_cmd_y = 0.0f;
        }
        else
        {
            map_cmd_x = 0.0f;
        }
        slide_clearance_m = PathMap_RayClearance(
            path_diagnostics.map_x_m, path_diagnostics.map_y_m,
            map_cmd_x, map_cmd_y);
        if (((map_cmd_x != 0.0f) || (map_cmd_y != 0.0f)) &&
            (slide_clearance_m > PATH_MAP_HARD_STOP_DISTANCE_M))
        {
            Path_RotateVector(map_cmd_x, map_cmd_y, -command_yaw_deg,
                              &body_cmd_x, &body_cmd_y);
            *vx = (int16_t)((body_cmd_x >= 0.0f) ? (body_cmd_x + 0.5f)
                                                 : (body_cmd_x - 0.5f));
            *vy = (int16_t)((body_cmd_y >= 0.0f) ? (body_cmd_y + 0.5f)
                                                 : (body_cmd_y - 0.5f));
            path_diagnostics.map_speed_limited = true;
            path_diagnostics.map_clearance_m = slide_clearance_m;
            magnitude = sqrtf((float)*vx * (float)*vx +
                              (float)*vy * (float)*vy);
            if (magnitude <= 0.0f)
            {
                return false;
            }
        }
    }
    if (path_diagnostics.map_clearance_m >=
        PATH_MAP_CLEARANCE_NONE_M)
    {
        return false;
    }

    allowed_speed_mps =
        PathSafety_MaxAllowedSpeed(path_diagnostics.map_clearance_m,
                                   0.0f, PATH_REACTION_TIME_S,
                                   PATH_MAP_BRAKE_DECELERATION_MPS2);
    if (PathSafety_LimitVectorCommand(vx, vy, allowed_speed_mps,
                                      PATH_COMMAND_TO_MPS) != 0)
    {
        path_diagnostics.map_speed_limited = true;
    }
    if (path_diagnostics.map_clearance_m <=
        PATH_MAP_HARD_STOP_DISTANCE_M)
    {
        hard_stop = true;
        *vx = 0;
        *vy = 0;
    }
    return hard_stop;
}

static bool Path_ApplyLaserLimits(int16_t *vx, int16_t *vy)
{
    /*
     * vx/vy 与 DT35 都在车体系：前光装在车头（车体 +Y），左光装在
     * 左侧（车体 -X），无论车头朝哪，前光永远只限制 vy>0、左光
     * 永远只限制 vx<0。回程"保护地图 -Y / +X"由车体系自然成立
     * （车头朝 -Y 后车体 +Y 即地图 -Y），不需要也不允许对命令取
     * 反——原实现在回程把命令按地图系取反，真实逼近墙时反而完全
     * 不限速。returning/镜像标志只用于判断左光当前物理上面向墙
     * 还是空旷场地（20 cm 饱和是否当真）。
     */
    int32_t front_command;
    int32_t left_command;
    int32_t limited;
    float requested_speed_mps;
    float measured_speed_mps;
    float safety_speed_mps;
    float distance_m;
    bool hard_stop = false;
    bool left_gate;

    path_diagnostics.front_speed_limited = false;
    path_diagnostics.left_speed_limited = false;
    path_diagnostics.front_required_distance_m = PATH_FRONT_BASE_DISTANCE_M;
    path_diagnostics.left_required_distance_m = PATH_LEFT_BASE_DISTANCE_M;
    path_diagnostics.front_allowed_speed_mps = 0.0f;
    path_diagnostics.left_allowed_speed_mps = 0.0f;

    front_command = (int32_t)(*vy);
    left_command = (int32_t)-(*vx);

    if (path_diagnostics.front_laser_online)
    {
        distance_m = (float)path_diagnostics.front_distance_cm * 0.01f;
        requested_speed_mps = (front_command > 0) ?
            (float)front_command * PATH_COMMAND_TO_MPS : 0.0f;
        /* 编码器速度同样是车体系：朝前光逼近恒为 +Y 分量。 */
        measured_speed_mps = Path_MaxFloat(
            path_diagnostics.encoder_velocity_y_mps, 0.0f);
        safety_speed_mps = Path_MaxFloat(requested_speed_mps,
                                         measured_speed_mps);
        path_diagnostics.front_required_distance_m =
            PathSafety_RequiredDistance(safety_speed_mps,
                                        PATH_FRONT_BASE_DISTANCE_M,
                                        PATH_REACTION_TIME_S,
                                        PATH_BRAKE_DECELERATION_MPS2);
        path_diagnostics.front_allowed_speed_mps =
            PathSafety_MaxAllowedSpeed(distance_m,
                                       PATH_FRONT_BASE_DISTANCE_M,
                                       PATH_REACTION_TIME_S,
                                       PATH_BRAKE_DECELERATION_MPS2);
        if (front_command > 0)
        {
            limited = PathSafety_LimitAxisCommand(
                (int16_t)front_command,
                path_diagnostics.front_allowed_speed_mps,
                PATH_COMMAND_TO_MPS);
            if (limited != (int16_t)front_command)
            {
                path_diagnostics.front_speed_limited = true;
                front_command = limited;
            }
        }
        if (path_front_blocked && (front_command > 0))
        {
            path_diagnostics.front_speed_limited = true;
            front_command = 0;
        }
        /*
         * 实测仍在逼近且已高于当前距离允许值时立即制动；若进入硬阻挡
         * 时上一安全输出仍朝传感器方向，即使里程计暂时无效也制动。
         * 反方向命令是主动远离障碍，保留恢复；纯横移不能掩盖惯性。
         */
        if ((front_command >= 0) &&
            ((measured_speed_mps >
              (path_diagnostics.front_allowed_speed_mps +
               PATH_HARD_STOP_SPEED_MPS)) ||
             (path_front_blocked && (path_last_output_vy > 0))))
        {
            hard_stop = true;
        }
    }

    /*
     * 左光门控：左光面向本侧近墙（常规侧=西墙、镜像侧=东墙）时，
     * 20 cm 饱和仍按“墙在量程边界”保守限速；面向空旷场地时只有
     * 出现真实目标（< 20 cm，例如对抗中靠近的另一台车）才启用
     * 左光限速。面向由车头朝向决定：车头靠近 +Y 时左光朝西，
     * 靠近 -Y 时朝东——倒车回程（车头 +Y）左光重新面向西墙，
     * 保守限速自动恢复。
     */
    {
        bool heading_north =
            cosf(Path_CommandYawDeg() * (PATH_PI / 180.0f)) >= 0.0f;
        bool left_faces_wall = PathMap_IsMirrored() ? !heading_north
                                                    : heading_north;

        left_gate = (path_diagnostics.left_distance_cm <
                     PATH_LEFT_LASER_MAX_CM) || left_faces_wall;
    }
    if (path_diagnostics.left_laser_online && left_gate)
    {
        distance_m = (float)path_diagnostics.left_distance_cm * 0.01f;
        requested_speed_mps = (left_command > 0) ?
            (float)left_command * PATH_COMMAND_TO_MPS : 0.0f;
        /* 朝左光逼近恒为车体 -X 分量。 */
        measured_speed_mps = Path_MaxFloat(
            -path_diagnostics.encoder_velocity_x_mps, 0.0f);
        safety_speed_mps = Path_MaxFloat(requested_speed_mps,
                                         measured_speed_mps);
        path_diagnostics.left_required_distance_m =
            PathSafety_RequiredDistance(safety_speed_mps,
                                        PATH_LEFT_BASE_DISTANCE_M,
                                        PATH_REACTION_TIME_S,
                                        PATH_BRAKE_DECELERATION_MPS2);
        path_diagnostics.left_allowed_speed_mps =
            PathSafety_MaxAllowedSpeed(distance_m,
                                       PATH_LEFT_BASE_DISTANCE_M,
                                       PATH_REACTION_TIME_S,
                                       PATH_BRAKE_DECELERATION_MPS2);
        if (left_command > 0)
        {
            limited = PathSafety_LimitAxisCommand(
                (int16_t)left_command,
                path_diagnostics.left_allowed_speed_mps,
                PATH_COMMAND_TO_MPS);
            if (limited != (int16_t)left_command)
            {
                path_diagnostics.left_speed_limited = true;
                left_command = limited;
            }
        }
        if (path_left_blocked && (left_command > 0))
        {
            path_diagnostics.left_speed_limited = true;
            left_command = 0;
        }
        if ((left_command >= 0) &&
            ((measured_speed_mps >
              (path_diagnostics.left_allowed_speed_mps +
               PATH_HARD_STOP_SPEED_MPS)) ||
             (path_left_blocked && (path_last_output_vx < 0))))
        {
            hard_stop = true;
        }
    }

    *vy = (int16_t)front_command;
    *vx = (int16_t)-left_command;
    return hard_stop;
}

/*
 * 自动行驶状态机：返回 true 时以 auto_vx/auto_vy（车体系）替换遥控
 * 平移命令进入同一条安全管线；到达终点的一拍置 *force_stop 触发
 * 整组制动并启动提示音。人工接管（任意摇杆/按键）立即永久退出。
 */
static bool Path_AutoUpdate(uint32_t now_ms,
                            const path_remote_snapshot_t *remote,
                            int16_t *auto_vx, int16_t *auto_vy,
                            bool *force_stop)
{
    uint8_t state = path_diagnostics.auto_state;
    float map_cmd_x = 0.0f;
    float map_cmd_y = 0.0f;
    float body_cmd_x;
    float body_cmd_y;

    *auto_vx = 0;
    *auto_vy = 0;

    if ((state == PATH_AUTO_STATE_DONE) || (state == PATH_AUTO_STATE_OFF))
    {
        return false;
    }

    /* 人工接管优先：遥控在线且有摇杆或按键输入即退出自动
     * （按键 3 是自动启动键，排除在接管判定之外）。 */
    if (remote->online &&
        ((Path_AbsCommand(remote->vx) > PATH_AUTO_TAKEOVER_COMMAND) ||
         (Path_AbsCommand(remote->vy) > PATH_AUTO_TAKEOVER_COMMAND) ||
         ((remote->buttons &
           (0x3FU & (uint8_t)~PATH_REMOTE_AUTO_BUTTON_BIT)) != 0U)))
    {
        path_diagnostics.auto_state = PATH_AUTO_STATE_OFF;
        return false;
    }

    if (state == PATH_AUTO_STATE_WAIT)
    {
#if PATH_AUTO_START_ON_BUTTON
        if (!path_auto_triggered)
        {
            return false;
        }
#else
        if (!path_auto_triggered && (now_ms < PATH_AUTO_START_DELAY_MS))
        {
            return false;
        }
#endif
        state = PATH_AUTO_STATE_READY_WAIT;
    }

    if (state == PATH_AUTO_STATE_READY_WAIT)
    {
        if (!path_diagnostics.yaw_zero_lock_ready ||
            !path_diagnostics.odometry_valid)
        {
            path_diagnostics.auto_state = state;
            return false;
        }
        state = path_diagnostics.initial_position_valid ?
                PATH_AUTO_STATE_DRIVE : PATH_AUTO_STATE_ANCHOR_RUN;
        /* 首次进入行驶不做段终点滑停。 */
        path_auto_last_segment = path_diagnostics.segment_index;
        path_auto_segment_change_ms = now_ms - PATH_AUTO_SETTLE_MS;
    }

    if (state == PATH_AUTO_STATE_ANCHOR_RUN)
    {
        if (path_diagnostics.initial_position_valid)
        {
            state = PATH_AUTO_STATE_DRIVE;
            path_auto_last_segment = path_diagnostics.segment_index;
            path_auto_segment_change_ms = now_ms - PATH_AUTO_SETTLE_MS;
        }
        else
        {
            /* 镜像侧贴东墙起步：起始区西边界是通道开口，左光无
             * 回波，慢速 +Y 前出约 6 cm 让前光读到墙 B' 后锚定。 */
            map_cmd_y = (float)PATH_AUTO_FINE_COMMAND;
        }
    }

    if (state == PATH_AUTO_STATE_DRIVE)
    {
        /* 段号推进：先滑停一个窗口再驶向下一段，消化换向动量。 */
        if (path_diagnostics.segment_index != path_auto_last_segment)
        {
            path_auto_last_segment = path_diagnostics.segment_index;
            path_auto_segment_change_ms = now_ms;
        }
        if (path_diagnostics.route_complete)
        {
            /* 到达终点：整组制动一次，鸣笛，交接并释放里程计/
             * 地图保护（一次性测试，之后纯人工）。 */
            *force_stop = true;
            path_beep_counter_ms = PATH_AUTO_BEEP_MS;
            path_diagnostics.auto_state = PATH_AUTO_STATE_DONE;
            path_handover = true;
            return true;
        }
        if ((uint32_t)(now_ms - path_auto_segment_change_ms) <
            PATH_AUTO_SETTLE_MS)
        {
            path_diagnostics.auto_state = state;
            return true; /* 滑停窗口：输出零速 */
        }
        {
            uint8_t route_count;
            const path_map_route_segment_t *route =
                PathMap_GetRoute(&route_count);

            if (path_diagnostics.segment_index < route_count)
            {
                const path_map_route_segment_t *segment =
                    &route[path_diagnostics.segment_index];
                float coordinate = (segment->axis == PATH_MAP_AXIS_X) ?
                                   path_diagnostics.map_x_m :
                                   path_diagnostics.map_y_m;
                float remaining_m = (segment->direction > 0) ?
                                    (segment->target_m - coordinate) :
                                    (coordinate - segment->target_m);
                int16_t speed = (remaining_m > PATH_AUTO_FINE_DISTANCE_M) ?
                                PATH_AUTO_FAST_COMMAND :
                                PATH_AUTO_FINE_COMMAND;

                if (segment->axis == PATH_MAP_AXIS_X)
                {
                    map_cmd_x = (float)(segment->direction * speed);
                }
                else
                {
                    map_cmd_y = (float)(segment->direction * speed);
                }
            }
        }
    }

    path_diagnostics.auto_state = state;
    /* 地图系目标方向旋回车体系（去程 yaw≈0，接近恒等变换）。 */
    Path_RotateVector(map_cmd_x, map_cmd_y, -Path_CommandYawDeg(),
                      &body_cmd_x, &body_cmd_y);
    *auto_vx = (int16_t)((body_cmd_x >= 0.0f) ? (body_cmd_x + 0.5f)
                                              : (body_cmd_x - 0.5f));
    *auto_vy = (int16_t)((body_cmd_y >= 0.0f) ? (body_cmd_y + 0.5f)
                                              : (body_cmd_y - 0.5f));
    return true;
}

static void Path_ApplyOutput(int16_t vx, int16_t vy, int16_t vz,
                             bool force_stop)
{
    if (force_stop)
    {
        Chassis_StopAll();
        vx = 0;
        vy = 0;
        vz = 0;
    }
    else if ((vx != path_last_output_vx) ||
             (vy != path_last_output_vy) ||
             (vz != path_last_output_z))
    {
        (void)Chassis_SetVelocity(vx, vy, vz);
    }

    path_last_output_vx = vx;
    path_last_output_vy = vy;
    path_last_output_z = vz;
    path_diagnostics.output_vx = vx;
    path_diagnostics.output_vy = vy;
    path_diagnostics.output_z = vz;
}

void Path_Init(void)
{
    (void)memset(&path_diagnostics, 0, sizeof(path_diagnostics));
    (void)memset(&path_front_filter, 0, sizeof(path_front_filter));
    (void)memset(&path_left_filter, 0, sizeof(path_left_filter));
    (void)memset(&path_front_initial_sampler, 0,
                 sizeof(path_front_initial_sampler));
    (void)memset(&path_left_initial_sampler, 0,
                 sizeof(path_left_initial_sampler));
    path_remote_vx = 0;
    path_remote_vy = 0;
    path_remote_vz = 0;
    path_remote_buttons = 0U;
    path_remote_timestamp_ms = 0U;
    path_remote_online = 0U;
    path_remote_sequence = 0U;
    path_last_output_vx = 0;
    path_last_output_vy = 0;
    path_last_output_z = 0;
    path_processed_remote_sequence = 0U;
    path_map_origin_x_m = 0.0f;
    path_map_origin_y_m = 0.0f;
    path_localization_yaw_deg = 0.0f;
    path_localization_yaw_valid = false;
    path_yaw_was_ready = false;
    path_front_blocked = false;
    path_left_blocked = false;
    path_auto_button_armed = true;
    path_auto_triggered = false;
    path_auto_last_segment = 0U;
    path_auto_segment_change_ms = 0U;
    path_beep_counter_ms = 0U;
    path_handover = false;
    PathMap_SetMirrored(false);

    path_diagnostics.initialized = true;
    path_diagnostics.segment_count = PATH_MAP_ROUTE_SEGMENT_COUNT;
    path_diagnostics.active_axis = PATH_MAP_AXIS_Y;
    path_diagnostics.auto_state = PATH_AUTO_STATE_WAIT;
    path_diagnostics.map_clearance_m = PATH_MAP_CLEARANCE_NONE_M;
}

void Path_SubmitRemoteCommand(int16_t *vx, int16_t *vy, int16_t *z,
                              uint8_t six_buttons, uint32_t now_ms)
{
    int16_t raw_vx;
    int16_t raw_vy;

    if ((vx == NULL) || (vy == NULL) || (z == NULL))
    {
        return;
    }

    raw_vx = *vx;
    raw_vy = *vy;
    Path_WriteRemoteMailbox(raw_vx, raw_vy, *z,
                            (uint8_t)(six_buttons & 0x3FU),
                            now_ms, true);

    /* 原 LoRa 调用保留，但只能重复最近一次已通过 1 ms 安全层的输出。 */
    *vx = path_last_output_vx;
    *vy = path_last_output_vy;
    *z = path_last_output_z;
}

void Path_NotifyRemoteOffline(uint32_t now_ms)
{
    Path_WriteRemoteMailbox(0, 0, 0, 0U, now_ms, false);
    path_last_output_vx = 0;
    path_last_output_vy = 0;
    path_last_output_z = 0;
}

void Path_ReplaceNonRemoteCommand(int16_t *vx, int16_t *vy, int16_t *z)
{
    if ((vx == NULL) || (vy == NULL) || (z == NULL))
    {
        return;
    }
    *vx = path_last_output_vx;
    *vy = path_last_output_vy;
    *z = path_last_output_z;
}

void Path_Run1ms(uint32_t now_ms)
{
    path_remote_snapshot_t remote;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    bool automatic_stop;
    bool hard_stop;

    if (!path_diagnostics.initialized)
    {
        return;
    }

    if (path_beep_counter_ms > 0U)
    {
        path_beep_counter_ms--;
    }

    Path_ReadRemoteMailbox(&remote);
    if (remote.online &&
        ((uint32_t)(now_ms - remote.timestamp_ms) > PATH_REMOTE_TIMEOUT_MS))
    {
        remote.online = false;
    }
    automatic_stop = Path_ProcessModeButton(&remote);
    Path_UpdateYawZeroLock();
    Path_UpdateLaserData();
    if (!path_handover)
    {
        automatic_stop = Path_UpdateOdometryAndRoute() || automatic_stop;
    }
    else
    {
        /* 交接后：融合里程计已停跑，地图坐标/净空保护全部退出；
         * 激光限速的实测速度项一并置零，只按请求速度限速。 */
        path_diagnostics.odometry_valid = false;
        path_diagnostics.encoder_velocity_x_mps = 0.0f;
        path_diagnostics.encoder_velocity_y_mps = 0.0f;
    }
    path_diagnostics.map_mirrored = PathMap_IsMirrored();

    path_diagnostics.remote_online = remote.online;
    path_diagnostics.last_remote_ms = remote.timestamp_ms;
    path_diagnostics.raw_vx = remote.vx;
    path_diagnostics.raw_vy = remote.vy;

    if (!remote.online)
    {
        remote.vx = 0;
        remote.vy = 0;
    }

    vx = remote.vx;
    vy = remote.vy;
    /*
     * 肩键旋转：行驶阶段保持锁 0（z 恒 0）；到点交接后放开肩键
     * （CalcOmega 手动旋转优先并跟随记录目标角，本层不再干预，
     * 见 Path_UpdateYawZeroLock）。
     */
    vz = (path_handover && remote.online) ? remote.vz : 0;

    /* 自动行驶：无遥控输入时按去程路线注入平移命令（同一安全管线）。 */
    {
        int16_t auto_vx;
        int16_t auto_vy;
        bool auto_stop = false;

        if (Path_AutoUpdate(now_ms, &remote, &auto_vx, &auto_vy,
                            &auto_stop))
        {
            vx = auto_vx;
            vy = auto_vy;
        }
        automatic_stop = automatic_stop || auto_stop;
    }

    hard_stop = Path_ApplyMapLimit(&vx, &vy);
    hard_stop = Path_ApplyLaserLimits(&vx, &vy) || hard_stop;

    /*
     * 高优先级通信任务若在本轮计算中更新了邮箱，不得回写旧命令。
     * 比较、底盘提交和 last_output 更新必须不可抢占，否则通信任务可能
     * 在比较后插入新帧，随后又被本轮旧快照覆盖，或读到一半的新输出。
     */
    {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        __DMB();
        if (path_remote_sequence != remote.sequence)
        {
            if (automatic_stop || hard_stop)
            {
                Path_ApplyOutput(0, 0, 0, true);
            }
            if (primask == 0U)
            {
                __enable_irq();
            }
            return;
        }
        Path_ApplyOutput(vx, vy, vz, automatic_stop || hard_stop);
        if (primask == 0U)
        {
            __enable_irq();
        }
    }
}

void Path_AutoStartTrigger(void)
{
    path_auto_triggered = true;
}

bool Path_OdometryReleased(void)
{
    return path_handover;
}

bool Path_ArrivalBeep(bool *level)
{
    uint16_t counter = path_beep_counter_ms;

    if ((level == NULL) || (counter == 0U))
    {
        return false;
    }
    /* 偶数拍高、奇数拍低：1 ms 交替 ≈500 Hz，最后一拍（1）为低。 */
    *level = (counter & 1U) == 0U;
    return true;
}

bool Path_GetDiagnostics(path_diagnostics_t *diagnostics)
{
    if (!path_diagnostics.initialized || (diagnostics == NULL))
    {
        return false;
    }
    *diagnostics = path_diagnostics;
    return true;
}
