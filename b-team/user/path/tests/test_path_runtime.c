#include "path.h"

#include "chassis_main.h"
#include "dt35_pnp_link.h"

volatile dt35_link_t dt35_link[SENSOR_LINK_COUNT];

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int16_t mock_chassis_vx;
static int16_t mock_chassis_vy;
static int16_t mock_chassis_z;
static uint32_t mock_stop_count;

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

static void submit_and_run(int16_t raw_vx, int16_t raw_vy,
                           int16_t raw_z, uint8_t buttons,
                           uint32_t now_ms)
{
    Path_SubmitRemoteCommand(&raw_vx, &raw_vy, &raw_z, buttons, now_ms);
    Path_Run1ms(now_ms);
}

static void set_laser(uint8_t index, uint16_t distance_cm, bool online)
{
    dt35_link[index].distance_cm = distance_cm;
    dt35_link[index].online = online ? 1U : 0U;
    dt35_link[index].last_rx_ms = 1U;
}

static void set_both(uint16_t front_cm, uint16_t left_cm)
{
    set_laser(SENSOR_LINK_F_INDEX, front_cm, true);
    set_laser(SENSOR_LINK_L_B_INDEX, left_cm, true);
}

static void reset_mocks(void)
{
    mock_chassis_vx = 0;
    mock_chassis_vy = 0;
    mock_chassis_z = 0;
    mock_stop_count = 0U;
    (void)memset((void *)dt35_link, 0, sizeof(dt35_link));
}

static void test_wait_for_dt35(void)
{
    path_diagnostics_t diagnostics;

    Path_Init();
    reset_mocks();
    Path_Run1ms(4999U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_WAIT);
    assert(mock_chassis_vy == 0);

    Path_Run1ms(5000U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_READY_WAIT);
    assert(mock_chassis_vy == 0);
    assert(!diagnostics.map_mirrored);
}

static void test_normal_dt35_route(void)
{
    path_diagnostics_t diagnostics;
    bool beep_level;
    uint32_t t;

    Path_Init();
    reset_mocks();
    set_both(140U, 10U);
    Path_Run1ms(5000U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_DRIVE);
    assert(!diagnostics.map_mirrored);
    assert(diagnostics.segment_index == 0U);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 150);

    set_both(26U, 10U);
    Path_Run1ms(5001U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 1U);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 0);

    set_both(140U, 80U);
    Path_Run1ms(5402U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 1U);
    assert(mock_chassis_vx == 150);
    assert(mock_chassis_vy == 0);

    set_both(140U, 222U);
    Path_Run1ms(5403U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 2U);
    assert(mock_chassis_vx == 0);

    Path_Run1ms(5804U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 2U);
    assert(mock_chassis_vy == 150);

    set_both(26U, 222U);
    Path_Run1ms(5805U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 3U);
    assert(mock_chassis_vy == 0);

    set_both(140U, 80U);
    Path_Run1ms(6206U);
    assert(mock_chassis_vx == -150);

    set_both(140U, 32U);
    Path_Run1ms(6207U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.route_complete);
    assert(diagnostics.auto_state == PATH_AUTO_STATE_DONE);
    assert(mock_stop_count >= 1U);
    assert(Path_ArrivalBeep(&beep_level));
    for (t = 6208U; t < 6400U; t++)
    {
        Path_Run1ms(t);
    }
    assert(!Path_ArrivalBeep(&beep_level));
    assert(Path_OdometryReleased());
}

static void test_mirrored_dt35_route(void)
{
    path_diagnostics_t diagnostics;

    Path_Init();
    reset_mocks();
    set_both(140U, 200U);
    Path_AutoStartTrigger();
    Path_Run1ms(100U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.map_mirrored);
    assert(diagnostics.auto_state == PATH_AUTO_STATE_DRIVE);
    assert(mock_chassis_vy == 150);

    set_both(26U, 200U);
    Path_Run1ms(101U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 1U);

    set_both(140U, 80U);
    Path_Run1ms(502U);
    assert(mock_chassis_vx == -150);
    assert(mock_chassis_vy == 0);

    set_both(140U, 32U);
    Path_Run1ms(503U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 2U);

    Path_Run1ms(904U);
    assert(mock_chassis_vy == 150);

    set_both(26U, 32U);
    Path_Run1ms(905U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 3U);

    set_both(140U, 80U);
    Path_Run1ms(1306U);
    assert(mock_chassis_vx == 150);

    set_both(140U, 222U);
    Path_Run1ms(1307U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.route_complete);
    assert(diagnostics.auto_state == PATH_AUTO_STATE_DONE);
}

static void test_pid_slows_near_target(void)
{
    path_diagnostics_t diagnostics;

    Path_Init();
    reset_mocks();
    set_both(36U, 10U);
    Path_AutoStartTrigger();
    Path_Run1ms(10U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_DRIVE);
    assert(diagnostics.segment_index == 0U);
    assert(fabsf(diagnostics.pid_error_cm - 10.0f) < 0.01f);
    assert(mock_chassis_vy > 30);
    assert(mock_chassis_vy < 60);
    assert(mock_chassis_vx == 0);
}

static void test_close_front_does_not_drive(void)
{
    path_diagnostics_t diagnostics;

    Path_Init();
    reset_mocks();
    set_both(10U, 10U);
    Path_AutoStartTrigger();
    Path_Run1ms(10U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.front_distance_cm == 10U);
    assert(diagnostics.segment_index >= 1U);
    assert(mock_chassis_vy == 0);
}

static void test_global_10cm_blocks_any_phase(void)
{
    path_diagnostics_t diagnostics;

    Path_Init();
    reset_mocks();
    set_both(9U, 80U);
    submit_and_run(0, 150, 0, 0U, 20U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_OFF);
    assert(diagnostics.front_hard_blocked);
    assert(mock_chassis_vy == 0);

    set_both(80U, 9U);
    submit_and_run(-150, 0, 0, 0U, 22U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.left_hard_blocked);
    assert(mock_chassis_vx == 0);
}

static void test_auto_takeover(void)
{
    path_diagnostics_t diagnostics;

    Path_Init();
    reset_mocks();
    set_both(140U, 10U);
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

static void test_remote_timeout(void)
{
    path_diagnostics_t diagnostics;

    Path_Init();
    reset_mocks();
    Path_Run1ms(1U);
    Path_Run1ms(401U);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 0);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.remote_online);
}

int main(void)
{
    Path_Init();
    Path_Run1ms(0U);
    test_wait_for_dt35();
    test_normal_dt35_route();
    test_mirrored_dt35_route();
    test_pid_slows_near_target();
    test_close_front_does_not_drive();
    test_global_10cm_blocks_any_phase();
    test_auto_takeover();
    test_remote_timeout();
    puts("path runtime host tests: PASS");
    return 0;
}
