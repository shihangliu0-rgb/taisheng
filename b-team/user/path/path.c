#include "path.h"

#include "chassis_main.h"
#include "imu_main.h"
#include "path_line_imu.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PATH_REMOTE_TIMEOUT_MS          200U
#define PATH_PI                         3.14159265358979323846f
#define PATH_AUTO_FAST_COMMAND          150
#define PATH_AUTO_FINE_COMMAND          75
#define PATH_AUTO_FINE_DISTANCE_M       0.800f
#define PATH_AUTO_TAKEOVER_COMMAND      10
#define PATH_AUTO_BEEP_MS               160U
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

static volatile int16_t path_remote_vx;
static volatile int16_t path_remote_vy;
static volatile int16_t path_remote_vz;
static volatile uint8_t path_remote_buttons;
static volatile uint32_t path_remote_timestamp_ms;
static volatile uint8_t path_remote_online;
static volatile uint32_t path_remote_sequence;

static volatile int16_t path_last_output_vx;
static volatile int16_t path_last_output_vy;
static volatile int16_t path_last_output_z;

static path_diagnostics_t path_diagnostics;
static uint32_t path_processed_remote_sequence;
static float path_map_origin_x_m;
static float path_map_origin_y_m;
static bool path_yaw_was_ready;
static bool path_auto_button_armed;
static bool path_auto_triggered;
static uint8_t path_auto_last_segment;
static uint32_t path_auto_segment_change_ms;
static volatile uint16_t path_beep_counter_ms;
static bool path_handover;

static int16_t Path_AbsCommand(int16_t value)
{
    if (value == INT16_MIN)
    {
        return INT16_MAX;
    }
    return (value < 0) ? (int16_t)-value : value;
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

static void Path_ProcessModeButton(const path_remote_snapshot_t *remote)
{
    bool auto_pressed;

    if (remote->sequence == path_processed_remote_sequence)
    {
        return;
    }
    path_processed_remote_sequence = remote->sequence;
    if (!remote->online || path_handover)
    {
        path_auto_button_armed = false;
        return;
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
}

static void Path_UpdateYawZeroLock(void)
{
    imu_data_t imu;
    bool assert_target;
    bool ready = ImuMain_GetData(&imu) &&
                 (imu.state == IMU_STATE_READY) && imu.online &&
                 imu.yaw_valid && !isnan(imu.yaw_deg) &&
                 !isinf(imu.yaw_deg);

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

static void Path_UpdateOdometryAndRoute(void)
{
    path_line_imu_data_t odometry;
    const path_map_route_segment_t *route;
    uint8_t route_count;

    path_diagnostics.odometry_valid =
        PathLineImu_GetData(&odometry) && odometry.imu_solution_valid;
    if (!path_diagnostics.odometry_valid)
    {
        return;
    }

    path_diagnostics.encoder_velocity_x_mps =
        odometry.imu_velocity_x_mps;
    path_diagnostics.encoder_velocity_y_mps =
        odometry.imu_velocity_y_mps;

    if (!path_diagnostics.initial_position_valid)
    {
        /*
         * 假定上电时车在固定起点，里程计从 (0,0) 积起。
         * 地图坐标 = 起点 + IMU 位移。
         */
        path_map_origin_x_m = PATH_MAP_START_X_M -
                              odometry.imu_position_x_m;
        path_map_origin_y_m = PATH_MAP_START_Y_M -
                              odometry.imu_position_y_m;
        path_diagnostics.initial_position_valid = true;
        path_diagnostics.initial_map_x_m = PATH_MAP_START_X_M;
        path_diagnostics.initial_map_y_m = PATH_MAP_START_Y_M;
        path_diagnostics.initial_yaw_deg = 0.0f;
    }

    path_diagnostics.map_x_m = path_map_origin_x_m +
                               odometry.imu_position_x_m;
    path_diagnostics.map_y_m = path_map_origin_y_m +
                               odometry.imu_position_y_m;

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
}

static bool Path_AutoUpdate(uint32_t now_ms,
                            const path_remote_snapshot_t *remote,
                            int16_t *auto_vx, int16_t *auto_vy,
                            bool *force_stop)
{
    uint8_t state = path_diagnostics.auto_state;
    uint8_t route_count;
    const path_map_route_segment_t *route;
    const path_map_route_segment_t *segment;
    float coordinate;
    float remaining_m;
    int16_t speed;

    *auto_vx = 0;
    *auto_vy = 0;

    if ((state == PATH_AUTO_STATE_DONE) || (state == PATH_AUTO_STATE_OFF))
    {
        return false;
    }

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
            !path_diagnostics.odometry_valid ||
            !path_diagnostics.initial_position_valid)
        {
            path_diagnostics.auto_state = state;
            return false;
        }
        state = PATH_AUTO_STATE_DRIVE;
        path_auto_last_segment = path_diagnostics.segment_index;
        path_auto_segment_change_ms = now_ms - PATH_AUTO_SETTLE_MS;
    }

    if (state == PATH_AUTO_STATE_DRIVE)
    {
        if (path_diagnostics.segment_index != path_auto_last_segment)
        {
            path_auto_last_segment = path_diagnostics.segment_index;
            path_auto_segment_change_ms = now_ms;
        }
        if (path_diagnostics.route_complete)
        {
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
            return true;
        }
        route = PathMap_GetRoute(&route_count);
        if (path_diagnostics.segment_index < route_count)
        {
            segment = &route[path_diagnostics.segment_index];
            coordinate = (segment->axis == PATH_MAP_AXIS_X) ?
                         path_diagnostics.map_x_m :
                         path_diagnostics.map_y_m;
            remaining_m = (segment->direction > 0) ?
                          (segment->target_m - coordinate) :
                          (coordinate - segment->target_m);
            speed = (remaining_m > PATH_AUTO_FINE_DISTANCE_M) ?
                    PATH_AUTO_FAST_COMMAND : PATH_AUTO_FINE_COMMAND;
            if (segment->axis == PATH_MAP_AXIS_X)
            {
                *auto_vx = (int16_t)(segment->direction * speed);
            }
            else
            {
                *auto_vy = (int16_t)(segment->direction * speed);
            }
        }
    }

    path_diagnostics.auto_state = state;
    return true;
}

static void Path_ApplyOutput(int16_t vx, int16_t vy, int16_t vz,
                             bool force_stop)
{
    if (force_stop)
    {
        if ((path_last_output_vx != 0) || (path_last_output_vy != 0) ||
            (path_last_output_z != 0))
        {
            Chassis_StopAll();
        }
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
    path_map_origin_x_m = PATH_MAP_START_X_M;
    path_map_origin_y_m = PATH_MAP_START_Y_M;
    path_yaw_was_ready = false;
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
    path_diagnostics.initial_map_x_m = PATH_MAP_START_X_M;
    path_diagnostics.initial_map_y_m = PATH_MAP_START_Y_M;
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
    int16_t auto_vx;
    int16_t auto_vy;
    bool auto_stop = false;
    uint32_t primask;

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
    Path_ProcessModeButton(&remote);
    Path_UpdateYawZeroLock();
    if (!path_handover)
    {
        Path_UpdateOdometryAndRoute();
    }
    else
    {
        path_diagnostics.odometry_valid = false;
    }

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
    vz = (path_handover && remote.online) ? remote.vz : 0;

    if (Path_AutoUpdate(now_ms, &remote, &auto_vx, &auto_vy, &auto_stop))
    {
        vx = auto_vx;
        vy = auto_vy;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    if (path_remote_sequence != remote.sequence)
    {
        if (auto_stop)
        {
            Path_ApplyOutput(0, 0, 0, true);
        }
        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }
    Path_ApplyOutput(vx, vy, vz, auto_stop);
    if (primask == 0U)
    {
        __enable_irq();
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
