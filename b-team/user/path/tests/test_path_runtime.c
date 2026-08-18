#include "path.h"

#include "chassis_main.h"
#include "dt35_pnp_link.h"
#include "imu_main.h"
#include "path_line_imu.h"

volatile dt35_link_t dt35_link[SENSOR_LINK_COUNT];

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static path_line_imu_data_t mock_odometry;
static imu_data_t mock_imu;
static int16_t mock_chassis_vx;
static int16_t mock_chassis_vy;
static int16_t mock_chassis_z;
static uint32_t mock_stop_count;
static uint32_t mock_yaw_target_count;

HAL_StatusTypeDef Chassis_SetVelocity(int16_t vx, int16_t vy, int16_t z)
{
    mock_chassis_vx = vx;
    mock_chassis_vy = vy;
    mock_chassis_z = z;
    return HAL_OK;
}

void Chassis_StopAll(void)
{
    mock_chassis_vx = 0;
    mock_chassis_vy = 0;
    mock_chassis_z = 0;
    mock_stop_count++;
}

bool PathLineImu_GetData(path_line_imu_data_t *data)
{
    *data = mock_odometry;
    return true;
}

bool ImuMain_GetData(imu_data_t *data)
{
    *data = mock_imu;
    return true;
}

void ImuMain_EnableYawHold(bool enabled)
{
    mock_imu.yaw_hold_enabled = enabled;
}

HAL_StatusTypeDef ImuMain_SetTargetYaw(float target_yaw_deg)
{
    mock_imu.target_yaw_deg = target_yaw_deg;
    mock_yaw_target_count++;
    return HAL_OK;
}

static void submit_and_run(int16_t raw_vx, int16_t raw_vy,
                           int16_t raw_z, uint8_t buttons,
                           uint32_t now_ms)
{
    Path_SubmitRemoteCommand(&raw_vx, &raw_vy, &raw_z, buttons, now_ms);
    Path_Run1ms(now_ms);
}

static void reset_mocks(void)
{
    (void)memset(&mock_odometry, 0, sizeof(mock_odometry));
    mock_odometry.imu_solution_valid = true;
    mock_imu.yaw_deg = 0.0f;
    mock_imu.state = IMU_STATE_READY;
    mock_imu.online = true;
    mock_imu.yaw_valid = true;
    mock_imu.yaw_hold_enabled = true;
    mock_chassis_vx = 0;
    mock_chassis_vy = 0;
    mock_chassis_z = 0;
    (void)memset((void *)dt35_link, 0, sizeof(dt35_link));
}

static void test_fixed_start_and_odom(void)
{
    path_diagnostics_t diagnostics;

    Path_Init();
    Path_Run1ms(1U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.initial_position_valid);
    assert(diagnostics.map_mirrored == (PATH_USE_MIRRORED != 0));
    assert(fabsf(diagnostics.initial_map_x_m - PATH_RUNTIME_START_X_M) <
           0.0002f);
    assert(fabsf(diagnostics.initial_map_y_m - PATH_RUNTIME_START_Y_M) <
           0.0002f);
    assert(fabsf(diagnostics.map_x_m - PATH_RUNTIME_START_X_M) < 0.0002f);
    assert(fabsf(diagnostics.map_y_m - PATH_RUNTIME_START_Y_M) < 0.0002f);

    mock_odometry.imu_position_x_m = 0.10f;
    mock_odometry.imu_position_y_m = 0.20f;
    Path_Run1ms(2U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(fabsf(diagnostics.map_x_m - (PATH_RUNTIME_START_X_M + 0.10f)) <
           0.0002f);
    assert(fabsf(diagnostics.map_y_m - (PATH_RUNTIME_START_Y_M + 0.20f)) <
           0.0002f);
    assert(diagnostics.segment_index == 0U);
}

static void test_auto_fixed_route(void)
{
    path_diagnostics_t diagnostics;
    bool beep_level;
    uint32_t t;

    Path_Init();
    reset_mocks();
    Path_Run1ms(100U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.initial_position_valid);
    Path_Run1ms(4999U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_WAIT);
    assert(mock_chassis_vy == 0);

    mock_odometry.imu_solution_valid = false;
    Path_Run1ms(5000U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_READY_WAIT);
    assert(mock_chassis_vy == 0);

    mock_odometry.imu_solution_valid = true;
    Path_Run1ms(5001U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_DRIVE);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 150);

    {
        uint8_t route_count;
        const path_map_route_segment_t *route = PathMap_GetRoute(&route_count);
        uint8_t i;
        uint32_t now = 5002U;
        float map_x = PATH_RUNTIME_START_X_M;
        float map_y = PATH_RUNTIME_START_Y_M;

        assert(route_count == PATH_MAP_ROUTE_SEGMENT_COUNT);
        for (i = 0U; i < route_count; i++)
        {
            if (route[i].axis == PATH_MAP_AXIS_Y)
            {
                map_y = route[i].target_m +
                        ((route[i].direction > 0) ? 0.001f : -0.001f);
            }
            else
            {
                map_x = route[i].target_m +
                        ((route[i].direction > 0) ? 0.001f : -0.001f);
            }
            mock_odometry.imu_position_x_m = map_x - PATH_RUNTIME_START_X_M;
            mock_odometry.imu_position_y_m = map_y - PATH_RUNTIME_START_Y_M;
            Path_Run1ms(now);
            now += 400U;
            if (i + 1U < route_count)
            {
                Path_Run1ms(now);
                now += 1U;
            }
        }
    }
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.route_complete);
    assert(diagnostics.auto_state == PATH_AUTO_STATE_DONE);
    assert(mock_stop_count >= 1U);
    assert(Path_ArrivalBeep(&beep_level));
    for (t = 7810U; t < 8010U; t++)
    {
        Path_Run1ms(t);
    }
    assert(!Path_ArrivalBeep(&beep_level));
    assert(Path_OdometryReleased());

    submit_and_run(-150, 0, 10, 0U, 5700U);
    assert(mock_chassis_vx == -150);
    assert(mock_chassis_z == 10);
}

static void test_auto_takeover(void)
{
    path_diagnostics_t diagnostics;

    Path_Init();
    reset_mocks();
    Path_Run1ms(200U);
    submit_and_run(0, 0, 0, 0U, 300U);
    submit_and_run(0, 0, 0, PATH_REMOTE_AUTO_BUTTON_BIT, 350U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_DRIVE);
    assert(mock_chassis_vy == 150);

    submit_and_run(50, 0, 0, 0U, 400U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_OFF);
    assert(mock_chassis_vx == 50);
    Path_Run1ms(6000U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_OFF);
}

static void set_laser(uint8_t index, uint16_t distance_cm, bool online)
{
    dt35_link[index].distance_cm = distance_cm;
    dt35_link[index].online = online ? 1U : 0U;
    dt35_link[index].last_rx_ms = 1U;
}

static void test_dt35_skip_to_next(void)
{
    path_diagnostics_t diagnostics;
    uint8_t route_count;
    const path_map_route_segment_t *route;
    int16_t next_vx;

    Path_Init();
    reset_mocks();
    Path_AutoStartTrigger();
    Path_Run1ms(100U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_DRIVE);
    assert(diagnostics.segment_index == 0U);
    assert(mock_chassis_vy == 150);
    assert(!diagnostics.front_hard_blocked);

    set_laser(SENSOR_LINK_F_INDEX, 9U, true);
    Path_Run1ms(101U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.front_hard_blocked);
    assert(diagnostics.front_distance_cm == 9U);
    assert(diagnostics.segment_index == 1U);
    assert(diagnostics.auto_state == PATH_AUTO_STATE_DRIVE);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 0);

    Path_Run1ms(200U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 1U);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 0);

    Path_Run1ms(502U);
    route = PathMap_GetRoute(&route_count);
    next_vx = (int16_t)(route[1].direction * 150);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 1U);
    assert(mock_chassis_vx == next_vx);
    assert(mock_chassis_vy == 0);

    set_laser(SENSOR_LINK_F_INDEX, 50U, true);
    set_laser(SENSOR_LINK_L_B_INDEX, 8U, true);
    Path_Run1ms(503U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.left_hard_blocked);
    assert(diagnostics.left_distance_cm == 8U);
    if (route[1].direction < 0)
    {
        assert(diagnostics.segment_index == 2U);
        assert(mock_chassis_vx == 0);
        assert(mock_chassis_vy == 0);
    }
    else
    {
        assert(diagnostics.segment_index == 1U);
        assert(mock_chassis_vx == next_vx);
    }

    set_laser(SENSOR_LINK_F_INDEX, 0U, true);
    set_laser(SENSOR_LINK_L_B_INDEX, 50U, true);
    Path_Run1ms(504U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.front_distance_cm == PATH_LASER_MIN_CM);
    assert(diagnostics.front_hard_blocked);
    if (route[1].direction > 0)
    {
        assert(diagnostics.segment_index == 1U);
        assert(mock_chassis_vx == next_vx);
    }

    set_laser(SENSOR_LINK_F_INDEX, 0U, false);
    set_laser(SENSOR_LINK_L_B_INDEX, 0U, false);
    Path_Run1ms(505U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.front_laser_online);
    assert(!diagnostics.left_hard_blocked);
    assert(diagnostics.auto_state == PATH_AUTO_STATE_DRIVE);
}

static void test_yaw_zero_and_timeout(void)
{
    path_diagnostics_t diagnostics;

    Path_Init();
    reset_mocks();
    Path_Run1ms(1U);
    assert(mock_yaw_target_count >= 1U);
    assert(mock_imu.target_yaw_deg == 0.0f);
    Path_Run1ms(401U);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 0);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.remote_online);
}

int main(void)
{
    (void)memset(&mock_odometry, 0, sizeof(mock_odometry));
    (void)memset(&mock_imu, 0, sizeof(mock_imu));
    mock_odometry.imu_solution_valid = true;
    mock_imu.state = IMU_STATE_READY;
    mock_imu.online = true;
    mock_imu.yaw_valid = true;
    mock_imu.yaw_hold_enabled = true;

    Path_Init();
    Path_Run1ms(0U);
    test_fixed_start_and_odom();
    test_auto_fixed_route();
    test_auto_takeover();
    test_dt35_skip_to_next();
    test_yaw_zero_and_timeout();
    puts("path runtime host tests: PASS");
    return 0;
}
