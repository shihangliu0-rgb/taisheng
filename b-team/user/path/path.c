#include "path.h"

#include "chassis_main.h"
#include "dt35_pnp_link.h"

#include <stddef.h>
#include <string.h>

#define PATH_REMOTE_TIMEOUT_MS          200U
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
static bool path_auto_button_armed;
static bool path_auto_triggered;
static bool path_field_detected;
static bool path_front_armed;
static uint8_t path_auto_last_segment;
static uint32_t path_auto_segment_change_ms;
static volatile uint16_t path_beep_counter_ms;
static bool path_handover;

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

static void Path_UpdateLaserData(void)
{
    uint32_t primask;
    uint16_t front_cm;
    uint16_t left_cm;
    bool front_online;
    bool left_online;

    primask = __get_PRIMASK();
    __disable_irq();
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
    path_diagnostics.front_distance_cm =
        Path_ClampLaserCm(front_cm, PATH_FRONT_LASER_MAX_CM);
    path_diagnostics.left_distance_cm =
        Path_ClampLaserCm(left_cm, PATH_LEFT_LASER_MAX_CM);
    path_diagnostics.front_hard_blocked =
        front_online &&
        (path_diagnostics.front_distance_cm < PATH_LASER_STOP_CM);
    path_diagnostics.left_hard_blocked =
        left_online &&
        (path_diagnostics.left_distance_cm < PATH_LASER_STOP_CM);
}

static void Path_DetectFieldSide(void)
{
    bool mirrored;

    if (path_field_detected || !path_diagnostics.left_laser_online)
    {
        return;
    }

    mirrored = (path_diagnostics.left_distance_cm >= PATH_MIRROR_LEFT_CM);
    PathMap_SetMirrored(mirrored);
    path_diagnostics.map_mirrored = mirrored;
    path_diagnostics.left_initial_distance_m =
        (float)path_diagnostics.left_distance_cm * 0.01f;
    path_diagnostics.front_initial_distance_m =
        (float)path_diagnostics.front_distance_cm * 0.01f;
    path_field_detected = true;
    path_diagnostics.initial_position_valid = true;
}

static int16_t Path_AbsCommand(int16_t value)
{
    if (value == INT16_MIN)
    {
        return INT16_MAX;
    }
    return (value < 0) ? (int16_t)-value : value;
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

static void Path_AdvanceSegment(uint32_t now_ms)
{
    if (path_diagnostics.segment_index < PATH_DT35_SEGMENT_COUNT)
    {
        path_diagnostics.segment_index++;
    }
    path_diagnostics.route_complete =
        path_diagnostics.segment_index >= PATH_DT35_SEGMENT_COUNT;
    path_auto_last_segment = path_diagnostics.segment_index;
    path_auto_segment_change_ms = now_ms;
    path_front_armed = path_diagnostics.front_laser_online &&
                       (path_diagnostics.front_distance_cm >
                        PATH_FRONT_ARRIVE_CM);
    if (!path_diagnostics.route_complete)
    {
        path_diagnostics.active_axis =
            ((path_diagnostics.segment_index % 2U) == 0U) ?
            PATH_MAP_AXIS_Y : PATH_MAP_AXIS_X;
    }
}

static bool Path_SegmentArrived(void)
{
    uint16_t front_cm = path_diagnostics.front_distance_cm;
    uint16_t left_cm = path_diagnostics.left_distance_cm;
    bool front_ok = path_diagnostics.front_laser_online;
    bool left_ok = path_diagnostics.left_laser_online;
    bool mirrored = path_diagnostics.map_mirrored;

    switch (path_diagnostics.segment_index)
    {
    case 0U:
    case 2U:
        if (front_ok && (front_cm > PATH_FRONT_ARRIVE_CM))
        {
            path_front_armed = true;
        }
        return front_ok && path_front_armed &&
               (front_cm <= PATH_FRONT_ARRIVE_CM);
    case 1U:
        if (mirrored)
        {
            return left_ok && (left_cm <= PATH_LEFT_NEAR_CM);
        }
        return left_ok && (left_cm >= PATH_LEFT_FAR_CM);
    case 3U:
        if (mirrored)
        {
            return left_ok && (left_cm >= PATH_LEFT_FAR_CM);
        }
        return left_ok && (left_cm <= PATH_LEFT_NEAR_CM);
    default:
        return false;
    }
}

static void Path_SegmentCommand(int16_t *vx, int16_t *vy)
{
    bool mirrored = path_diagnostics.map_mirrored;

    *vx = 0;
    *vy = 0;
    switch (path_diagnostics.segment_index)
    {
    case 0U:
    case 2U:
        *vy = PATH_AUTO_FAST_COMMAND;
        break;
    case 1U:
        *vx = mirrored ? (int16_t)-PATH_AUTO_FAST_COMMAND
                       : PATH_AUTO_FAST_COMMAND;
        break;
    case 3U:
        *vx = mirrored ? PATH_AUTO_FAST_COMMAND
                       : (int16_t)-PATH_AUTO_FAST_COMMAND;
        break;
    default:
        break;
    }
}

static bool Path_AutoUpdate(uint32_t now_ms,
                            const path_remote_snapshot_t *remote,
                            int16_t *auto_vx, int16_t *auto_vy,
                            bool *force_stop)
{
    uint8_t state = path_diagnostics.auto_state;

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
        Path_DetectFieldSide();
        if (!path_diagnostics.front_laser_online ||
            !path_diagnostics.left_laser_online ||
            !path_field_detected)
        {
            path_diagnostics.auto_state = state;
            return false;
        }
        state = PATH_AUTO_STATE_DRIVE;
        path_auto_last_segment = path_diagnostics.segment_index;
        path_auto_segment_change_ms = now_ms - PATH_AUTO_SETTLE_MS;
        path_front_armed = path_diagnostics.front_laser_online &&
                           (path_diagnostics.front_distance_cm >
                            PATH_FRONT_ARRIVE_CM);
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

        Path_SegmentCommand(auto_vx, auto_vy);
        if (Path_SegmentArrived())
        {
            Path_AdvanceSegment(now_ms);
            *auto_vx = 0;
            *auto_vy = 0;
            *force_stop = true;
            if (path_diagnostics.route_complete)
            {
                path_beep_counter_ms = PATH_AUTO_BEEP_MS;
                path_diagnostics.auto_state = PATH_AUTO_STATE_DONE;
                path_handover = true;
            }
            else
            {
                path_diagnostics.auto_state = state;
            }
            return true;
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
    path_auto_button_armed = true;
    path_auto_triggered = false;
    path_field_detected = false;
    path_front_armed = false;
    path_auto_last_segment = 0U;
    path_auto_segment_change_ms = 0U;
    path_beep_counter_ms = 0U;
    path_handover = false;
    PathMap_SetMirrored(false);

    path_diagnostics.initialized = true;
    path_diagnostics.segment_count = PATH_DT35_SEGMENT_COUNT;
    path_diagnostics.active_axis = PATH_MAP_AXIS_Y;
    path_diagnostics.auto_state = PATH_AUTO_STATE_WAIT;
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
    bool auto_active;
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
    Path_UpdateLaserData();

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

    auto_active = Path_AutoUpdate(now_ms, &remote, &auto_vx, &auto_vy,
                                  &auto_stop);
    if (auto_active)
    {
        vx = auto_vx;
        vy = auto_vy;
    }
    else
    {
        if (path_diagnostics.front_hard_blocked && (vy > 0))
        {
            vy = 0;
        }
        if (path_diagnostics.left_hard_blocked && (vx < 0))
        {
            vx = 0;
        }
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
