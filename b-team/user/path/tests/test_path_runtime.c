#include "path.h"

#include "chassis_main.h"
#include "dt35_pnp_link.h"
#include "imu_main.h"
#include "path_line_imu.h"
#include "path_localization.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

volatile dt35_link_t dt35_link[SENSOR_LINK_COUNT];

static path_line_imu_data_t mock_odometry;
static imu_data_t mock_imu;
static int16_t mock_chassis_vx;
static int16_t mock_chassis_vy;
static int16_t mock_chassis_z;
static uint32_t mock_set_count;
static uint32_t mock_stop_count;
static uint32_t mock_yaw_target_count;

HAL_StatusTypeDef Chassis_SetVelocity(int16_t vx, int16_t vy, int16_t z)
{
    mock_chassis_vx = vx;
    mock_chassis_vy = vy;
    mock_chassis_z = z;
    mock_set_count++;
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

static void set_front_laser(uint16_t distance_cm, uint32_t now_ms)
{
    dt35_link[SENSOR_LINK_F_INDEX].distance_cm = distance_cm;
    dt35_link[SENSOR_LINK_F_INDEX].last_rx_ms = now_ms;
    dt35_link[SENSOR_LINK_F_INDEX].online = 1U;
}

static void set_left_laser(uint16_t distance_cm, uint32_t now_ms)
{
    dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm = distance_cm;
    dt35_link[SENSOR_LINK_L_B_INDEX].last_rx_ms = now_ms;
    dt35_link[SENSOR_LINK_L_B_INDEX].online = 1U;
}

static void test_initial_position_median_and_anchor(void)
{
    path_diagnostics_t diagnostics;
    float initial_x_m;
    float initial_y_m;
    float yaw_rad = 10.0f * 3.14159265358979323846f / 180.0f;

    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.initial_position_valid);

    /* 前 DT35 保持无样本/离线；同一左光时间戳不能伪造 3 个样本。 */
    assert(dt35_link[SENSOR_LINK_F_INDEX].online == 0U);
    mock_imu.yaw_deg = 10.0f;
    set_left_laser(15U, 1U);
    Path_Run1ms(1U);
    Path_Run1ms(2U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.front_laser_online);
    assert(diagnostics.front_initial_sample_count == 0U);
    assert(diagnostics.left_initial_sample_count == 1U);

    set_left_laser(17U, 2U);
    Path_Run1ms(2U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.initial_position_valid);
    assert(diagnostics.front_initial_sample_count == 0U);
    assert(diagnostics.left_initial_sample_count == 2U);

    set_left_laser(16U, 3U);
    Path_Run1ms(3U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.initial_position_valid);
    assert(diagnostics.front_initial_sample_count == 0U);
    assert(diagnostics.left_initial_sample_count == 3U);
    assert(fabsf(diagnostics.front_initial_distance_m) < 0.0002f);
    assert(fabsf(diagnostics.left_initial_distance_m - 0.16f) < 0.0002f);
    assert(fabsf(diagnostics.initial_map_x_m -
                 (PATH_MAP_WEST_INNER_X_M + 0.38f * cosf(yaw_rad))) <
           0.0002f);
    assert(fabsf(diagnostics.initial_map_y_m - 0.3085f) < 0.0002f);
    assert(fabsf(diagnostics.front_wall_hit_x_m) < 0.0002f);
    assert(fabsf(diagnostics.initial_yaw_deg - 10.0f) < 0.0002f);
    assert(diagnostics.segment_index == 0U);
    assert(diagnostics.active_axis == PATH_MAP_AXIS_Y);
    initial_x_m = diagnostics.initial_map_x_m;
    initial_y_m = diagnostics.initial_map_y_m;

    /* 定点后新激光不再改原点，地图位置仅累加融合里程计位移。 */
    mock_imu.yaw_deg = 0.0f;
    mock_odometry.fused_position_x_m = 0.10f;
    mock_odometry.fused_position_y_m = 0.05f;
    set_front_laser(5U, 4U);
    set_left_laser(5U, 4U);
    Path_Run1ms(4U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(fabsf(diagnostics.map_x_m - (initial_x_m + 0.10f)) < 0.0002f);
    assert(fabsf(diagnostics.map_y_m - (initial_y_m + 0.05f)) < 0.0002f);
    assert(fabsf(diagnostics.initial_map_x_m - initial_x_m) < 0.0002f);
    assert(fabsf(diagnostics.initial_map_y_m - initial_y_m) < 0.0002f);

    mock_odometry.fused_position_x_m = 0.0f;
    mock_odometry.fused_position_y_m = 0.0f;
    set_front_laser(20U, 5U);
    set_left_laser(20U, 5U);
    Path_Run1ms(5U);
    set_front_laser(20U, 6U);
    set_left_laser(20U, 6U);
    Path_Run1ms(6U);
    set_front_laser(20U, 7U);
    set_left_laser(20U, 7U);
    Path_Run1ms(7U);
    dt35_link[SENSOR_LINK_F_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].online = 0U;
}

static void test_laser_direction_and_offline_policy(void)
{
    path_diagnostics_t diagnostics;
    int16_t vx;
    int16_t vy;
    int16_t z;

    /* 移到墙 B 右侧做独立激光安全测试（不影响初始锚点）。 */
    assert(Path_GetDiagnostics(&diagnostics));
    mock_odometry.fused_position_x_m =
        2.480f - diagnostics.initial_map_x_m;
    mock_odometry.fused_position_y_m =
        1.700f - diagnostics.initial_map_y_m;
    Path_Run1ms(105U);

    /* 前 DT35 读到 20 cm 真实距离时按动态制动距离限速。 */
    set_front_laser(20U, 110U);
    submit_and_run(0, 150, 0, 0U, 110U);
    assert(mock_chassis_vy > 0);
    assert(mock_chassis_vy < 150);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.front_speed_limited);
    assert(diagnostics.front_laser_online);

    /* 三点保守最小值对距离下降首帧生效；0 同样按 5 cm 处理。 */
    set_front_laser(10U, 120U);
    submit_and_run(0, 150, 0, 0U, 120U);
    assert(mock_chassis_vy == 0);
    set_front_laser(10U, 130U);
    submit_and_run(0, 150, 0, 0U, 130U);
    set_front_laser(10U, 140U);
    submit_and_run(0, 150, 0, 0U, 140U);
    assert(mock_chassis_vy == 0);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.front_hard_blocked);

    set_front_laser(0U, 150U);
    submit_and_run(0, 100, 0, 0U, 150U);
    set_front_laser(0U, 160U);
    submit_and_run(0, 100, 0, 0U, 160U);
    set_front_laser(0U, 170U);
    submit_and_run(0, 100, 0, 0U, 170U);
    assert(mock_chassis_vy == 0);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.front_distance_cm == 5U);

    /* 距离增大必须用三帧覆盖近距离历史后才释放。 */
    set_front_laser(20U, 171U);
    submit_and_run(0, 100, 0, 0U, 171U);
    assert(mock_chassis_vy == 0);
    set_front_laser(20U, 172U);
    submit_and_run(0, 100, 0, 0U, 172U);
    assert(mock_chassis_vy == 0);
    set_front_laser(20U, 173U);
    submit_and_run(0, 100, 0, 0U, 173U);
    assert(mock_chassis_vy > 0);
    assert(mock_chassis_vy < 100);

    /* 离线只更新诊断，不因离线停车。 */
    dt35_link[SENSOR_LINK_F_INDEX].online = 0U;
    submit_and_run(0, 100, 0, 0U, 180U);
    assert(mock_chassis_vy == 100);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.front_laser_online);

    /* 离线后重新在线的近障碍必须首帧生效，不能沿用旧的远距离。 */
    set_front_laser(5U, 185U);
    submit_and_run(0, 100, 0, 0U, 185U);
    assert(mock_chassis_vy == 0);

    /* 横移不能掩盖仍在逼近前障碍的实测惯性；后退恢复仍可用。 */
    mock_odometry.encoder_body_velocity_y_mps = 0.20f;
    submit_and_run(30, 0, 0, 0U, 186U);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 0);
    submit_and_run(0, -50, 0, 0U, 187U);
    assert(mock_chassis_vy < 0);
    mock_odometry.encoder_body_velocity_y_mps = 0.0f;
    dt35_link[SENSOR_LINK_F_INDEX].online = 0U;

    /* 左 DT35 只限制 -X；+X 逃离方向保持可用。 */
    set_left_laser(5U, 190U);
    submit_and_run(-100, 0, 0, 0U, 190U);
    assert(mock_chassis_vx == 0);
    submit_and_run(100, 0, 0, 0U, 200U);
    assert(mock_chassis_vx > 0);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.left_speed_limited);

    /* 上位机来源不能旁路人工安全输出，且旋转始终为 0。 */
    vx = -999;
    vy = -999;
    z = 999;
    Path_ReplaceNonRemoteCommand(&vx, &vy, &z);
    assert(vx == mock_chassis_vx);
    assert(vy == 0);
    assert(z == 0);
}

static void test_yaw_zero_and_remote_timeout(void)
{
    path_diagnostics_t diagnostics;

    assert(mock_yaw_target_count >= 1U);
    assert(mock_imu.target_yaw_deg == 0.0f);
    Path_Run1ms(401U);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 0);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.remote_online);
}

/*
 * 对抗镜像侧（真实场地）：西边界在南墙与墙 1' 之间是通道开口，
 * 起始区左光朝西没有回波（在线但收不到样本）；前进约 6 cm 后
 * 前光读到第一堵前墙（墙 B'，y=2.075）进入 1.4 m 量程 → 判定
 * 镜像侧并用前光锚定 Y、贴东墙起点 2.626 锚定 X。
 */
static void test_mirrored_side_detection_and_anchor(void)
{
    path_diagnostics_t diagnostics;

    /* 模拟镜像侧重新上电：重新初始化模块并复位外部模拟量。 */
    Path_Init();
    (void)memset(&mock_odometry, 0, sizeof(mock_odometry));
    mock_odometry.encoder_solution_valid = true;
    mock_imu.yaw_deg = 0.0f;
    mock_imu.state = IMU_STATE_READY;
    mock_imu.online = true;
    mock_imu.yaw_valid = true;
    mock_imu.yaw_hold_enabled = true;
    mock_chassis_vx = 0;
    mock_chassis_vy = 0;
    dt35_link[SENSOR_LINK_F_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_F_INDEX].distance_cm = 0U;
    dt35_link[SENSOR_LINK_F_INDEX].last_rx_ms = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].last_rx_ms = 0U;

    Path_Run1ms(500U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.initial_position_valid);
    assert(!diagnostics.map_mirrored);

    /* 起始区左光朝通道开口：在线但 240 cm 饱和，无初始样本。 */
    set_left_laser(240U, 501U);
    Path_Run1ms(501U);
    set_left_laser(240U, 502U);
    Path_Run1ms(502U);
    set_left_laser(240U, 503U);
    Path_Run1ms(503U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.initial_position_valid);
    assert(diagnostics.left_initial_sample_count == 0U);

    /* 起点前光距墙 B' 约 145.8 cm，超量程钳 140：不判定不锚定。 */
    set_front_laser(145U, 504U);
    Path_Run1ms(504U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.initial_position_valid);
    assert(!diagnostics.map_mirrored);

    /*
     * 慢速 +Y 前出约 7 cm：前光 139 cm 进入量程，左光仍无样本 →
     * 镜像侧，Y = 2.075 − 0.3085 − 1.39 = 0.3765，X = 2.626。
     */
    mock_odometry.fused_position_y_m = 0.10f;
    set_left_laser(240U, 505U);
    set_front_laser(139U, 505U);
    Path_Run1ms(505U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.initial_position_valid);
    assert(diagnostics.map_mirrored);
    assert(fabsf(diagnostics.initial_map_x_m - 2.626f) < 0.0002f);
    assert(fabsf(diagnostics.initial_map_y_m - 0.3765f) < 0.0002f);
    assert(fabsf(diagnostics.front_initial_distance_m - 1.39f) < 0.0002f);
    assert(diagnostics.segment_index == 0U);
    assert(diagnostics.active_axis == PATH_MAP_AXIS_Y);

    /* 镜像侧左光饱和（通道开口/空旷）跳过限速：-X 满杆可用。 */
    set_left_laser(240U, 510U);
    submit_and_run(-200, 0, 0, 0U, 510U);
    assert(mock_chassis_vx == -200);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.left_speed_limited);
    assert(!diagnostics.left_hard_blocked);

    /* 镜像侧左光出现真实近距目标（对抗车辆）时，限速恢复。 */
    set_left_laser(15U, 511U);
    submit_and_run(-200, 0, 0, 0U, 511U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.left_speed_limited);
    assert(mock_chassis_vx > -200);
    assert(mock_chassis_vx < 0);
    submit_and_run(0, 0, 0, 0U, 512U);

    /* 镜像段 0（Y+ 到 1.60）推进后进入镜像段 1（X− 到 0.52）。 */
    mock_odometry.fused_position_y_m = 1.601f - 0.2765f;
    Path_Run1ms(520U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 1U);
    assert(diagnostics.active_axis == PATH_MAP_AXIS_X);

    /* 镜像段 1 终点 x = 0.52 m。 */
    mock_odometry.fused_position_x_m = 0.53f - 2.626f;
    Path_Run1ms(521U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 1U);
    mock_odometry.fused_position_x_m = 0.52f - 2.626f;
    Path_Run1ms(522U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 2U);
    assert(diagnostics.active_axis == PATH_MAP_AXIS_Y);

    /*
     * 反例：贴西墙起点左光读数 15 cm → 锚定 x = 0.419 < 1.5，
     * 判定常规侧，不切换镜像地图。
     */
    Path_Init();
    (void)memset(&mock_odometry, 0, sizeof(mock_odometry));
    mock_odometry.encoder_solution_valid = true;
    mock_imu.state = IMU_STATE_READY;
    mock_imu.yaw_deg = 0.0f;
    mock_imu.online = true;
    mock_imu.yaw_valid = true;
    mock_imu.yaw_hold_enabled = true;
    mock_chassis_vx = 0;
    mock_chassis_vy = 0;
    dt35_link[SENSOR_LINK_F_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].online = 0U;

    Path_Run1ms(600U);
    set_left_laser(15U, 601U);
    Path_Run1ms(601U);
    set_left_laser(15U, 602U);
    Path_Run1ms(602U);
    set_left_laser(15U, 603U);
    Path_Run1ms(603U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.initial_position_valid);
    assert(!diagnostics.map_mirrored);
    assert(fabsf(diagnostics.initial_map_x_m - 0.419f) < 0.0002f);
    assert(fabsf(diagnostics.initial_map_y_m - 0.3085f) < 0.0002f);

    /*
     * 封闭场地变体兜底：若西侧在起始区有墙（左光读到 ~2.35 m），
     * 左光静止锚定 X 越过半场同样正确判定镜像侧。
     */
    Path_Init();
    (void)memset(&mock_odometry, 0, sizeof(mock_odometry));
    mock_odometry.encoder_solution_valid = true;
    mock_imu.state = IMU_STATE_READY;
    mock_imu.yaw_deg = 0.0f;
    mock_imu.online = true;
    mock_imu.yaw_valid = true;
    mock_imu.yaw_hold_enabled = true;
    dt35_link[SENSOR_LINK_F_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].online = 0U;

    Path_Run1ms(700U);
    set_left_laser(235U, 701U);
    Path_Run1ms(701U);
    set_left_laser(235U, 702U);
    Path_Run1ms(702U);
    set_left_laser(235U, 703U);
    Path_Run1ms(703U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.initial_position_valid);
    assert(diagnostics.map_mirrored);
    assert(fabsf(diagnostics.initial_map_x_m - 2.619f) < 0.0002f);
}

/*
 * 自动行驶：无遥控输入，上电延时 5 s 后自动沿去程 6 段行驶，
 * 近段终点换精调档，到终点自动停车 + 鸣笛 160 ms + 转人工接手。
 */
static void test_auto_route(void)
{
    path_diagnostics_t diagnostics;
    bool beep_level;
    uint32_t t;

    Path_Init();
    (void)memset(&mock_odometry, 0, sizeof(mock_odometry));
    mock_odometry.encoder_solution_valid = true;
    mock_imu.yaw_deg = 0.0f;
    mock_imu.state = IMU_STATE_READY;
    mock_imu.online = true;
    mock_imu.yaw_valid = true;
    mock_imu.yaw_hold_enabled = true;
    mock_chassis_vx = 0;
    mock_chassis_vy = 0;
    mock_chassis_z = 0;
    dt35_link[SENSOR_LINK_F_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_F_INDEX].distance_cm = 0U;
    dt35_link[SENSOR_LINK_F_INDEX].last_rx_ms = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].last_rx_ms = 0U;

    /* 锚定完成后仍在等待 5 s 延时，不动。 */
    Path_Run1ms(100U);
    set_left_laser(15U, 101U);
    Path_Run1ms(101U);
    set_left_laser(15U, 102U);
    Path_Run1ms(102U);
    set_left_laser(15U, 103U);
    Path_Run1ms(103U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.initial_position_valid);
    Path_Run1ms(4999U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_WAIT);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 0);

    /* 5 s 到：段 0（Y+）快杆自动前进。 */
    Path_Run1ms(5000U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_DRIVE);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 150);

    /* 段 0 越过 1.60 -> 段 1（X+）：先滑停 400 ms 消化换向动量。 */
    mock_odometry.fused_position_y_m = 1.601f - 0.3085f;
    Path_Run1ms(5001U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.segment_index == 1U);
    assert(mock_chassis_vy == 0);
    assert(mock_chassis_vx == 0);
    /* 滑停窗口结束后驶向段 1。 */
    Path_Run1ms(5402U);
    assert(mock_chassis_vx == 150);
    assert(mock_chassis_vy == 0);

    /* 距段终点 <0.8 m 自动换精调档。 */
    mock_odometry.fused_position_x_m = 2.300f - 0.419f;
    Path_Run1ms(5403U);
    assert(mock_chassis_vx == 75);

    /* 推进剩余段直到终点。 */
    mock_odometry.fused_position_x_m = 2.481f - 0.419f;
    Path_Run1ms(5404U);
    mock_odometry.fused_position_y_m = 2.601f - 0.3085f;
    Path_Run1ms(5405U);
    mock_odometry.fused_position_x_m = 0.359f - 0.419f;
    Path_Run1ms(5406U);
    mock_odometry.fused_position_y_m = 3.701f - 0.3085f;
    Path_Run1ms(5407U);
    mock_odometry.fused_position_x_m = 0.501f - 0.419f;
    Path_Run1ms(5408U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.route_complete);
    assert(diagnostics.auto_state == PATH_AUTO_STATE_DONE);
    assert(mock_stop_count >= 1U);
    assert(mock_chassis_vx == 0);
    assert(mock_chassis_vy == 0);

    /* 到点鸣笛 160 ms（1 ms 交替），结束后自动停止且不再触发。 */
    assert(Path_ArrivalBeep(&beep_level));
    for (t = 5409U; t < 5609U; t++)
    {
        Path_Run1ms(t);
    }
    assert(!Path_ArrivalBeep(&beep_level));
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_DONE);

    /*
     * 交接后（一次性测试语义）：融合里程计释放（freertos 停跑
     * PathLineImu_Run1ms）、地图净空保护关闭、肩键直通、
     * 按键 1/2 失效，遥控完全人工驾驶。
     */
    assert(Path_OdometryReleased());
    submit_and_run(0, 0, 0, 0U, 5300U);
    /* 左光仍在线时激光限速保留：向西命令仍被左光按 15 cm 限速。 */
    submit_and_run(-150, 0, 0, 0U, 5320U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.left_speed_limited);
    assert(mock_chassis_vx < 0);
    assert(mock_chassis_vx > -150);
    /* 左光离线后：交接前该位置(0.5,3.7)西向会被地图净空限到约 93，
       交接后地图保护已关闭，满杆 + 肩键旋转原样直通。 */
    dt35_link[SENSOR_LINK_L_B_INDEX].online = 0U;
    submit_and_run(-150, 0, 10, 0U, 5350U);
    assert(mock_chassis_vx == -150);
    assert(mock_chassis_z == 10);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(!diagnostics.odometry_valid);
    assert(!diagnostics.map_speed_limited);
    submit_and_run(0, 0, 0, 0U, 5400U);
    submit_and_run(0, 80, 0, 0U, 5450U);
    assert(mock_chassis_vy == 80);
}

/* 自动行驶中任意摇杆输入立即人工接管；按键 3 可提前启动。 */
static void test_auto_takeover_and_button(void)
{
    path_diagnostics_t diagnostics;

    Path_Init();
    (void)memset(&mock_odometry, 0, sizeof(mock_odometry));
    mock_odometry.encoder_solution_valid = true;
    mock_imu.yaw_deg = 0.0f;
    mock_imu.state = IMU_STATE_READY;
    mock_imu.online = true;
    mock_imu.yaw_valid = true;
    mock_imu.yaw_hold_enabled = true;
    mock_chassis_vx = 0;
    mock_chassis_vy = 0;
    dt35_link[SENSOR_LINK_F_INDEX].online = 0U;
    dt35_link[SENSOR_LINK_L_B_INDEX].online = 0U;

    Path_Run1ms(200U);
    set_left_laser(15U, 201U);
    Path_Run1ms(201U);
    set_left_laser(15U, 202U);
    Path_Run1ms(202U);
    set_left_laser(15U, 203U);
    Path_Run1ms(203U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.initial_position_valid);

    /* 按键 3 按下沿：延时未到也立即启动（按键启动接口）。 */
    submit_and_run(0, 0, 0, 0U, 300U);
    submit_and_run(0, 0, 0, PATH_REMOTE_AUTO_BUTTON_BIT, 350U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_DRIVE);
    assert(mock_chassis_vy == 150);

    /* 摇杆推杆 -> 立即人工接管，自动永久退出。 */
    submit_and_run(50, 0, 0, 0U, 400U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_OFF);
    assert(mock_chassis_vx == 50);
    assert(mock_chassis_vy == 0);
    submit_and_run(0, 0, 0, 0U, 450U);
    Path_Run1ms(6000U);
    assert(Path_GetDiagnostics(&diagnostics));
    assert(diagnostics.auto_state == PATH_AUTO_STATE_OFF);
}

int main(void)
{
    (void)memset((void *)dt35_link, 0, sizeof(dt35_link));
    (void)memset(&mock_odometry, 0, sizeof(mock_odometry));
    (void)memset(&mock_imu, 0, sizeof(mock_imu));
    mock_odometry.encoder_solution_valid = true;
    mock_imu.state = IMU_STATE_READY;
    mock_imu.online = true;
    mock_imu.yaw_valid = true;
    mock_imu.yaw_hold_enabled = true;

    Path_Init();
    Path_Run1ms(0U);
    test_initial_position_median_and_anchor();
    test_laser_direction_and_offline_policy();
    test_yaw_zero_and_remote_timeout();
    test_mirrored_side_detection_and_anchor();
    test_auto_route();
    test_auto_takeover_and_button();
    puts("path runtime host tests: PASS");
    return 0;
}
