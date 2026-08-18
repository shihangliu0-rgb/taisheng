#include "path_line_imu.h"

#include "chassis_main.h"
#include "imu_main.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define TEST_PI_F 3.14159265358979323846f

static imu_data_t test_imu;
static vesc_motor_status_t test_wheels[CHASSIS_WHEEL_COUNT];

bool ImuMain_GetData(imu_data_t *data)
{
    *data = test_imu;
    return true;
}

bool Chassis_GetStatus(chassis_wheel_t wheel,
                       vesc_motor_status_t *status)
{
    if ((wheel < CHASSIS_WHEEL_LF) || (wheel >= CHASSIS_WHEEL_COUNT))
    {
        return false;
    }
    *status = test_wheels[wheel];
    return true;
}

static void Test_SendAcceleration(uint32_t now_ms,
                                  float x,
                                  float y,
                                  float z)
{
    uint8_t frame[19] = {0x55U, 0xAAU, 0x00U, 0x01U};

    memcpy(&frame[4], &x, sizeof(x));
    memcpy(&frame[8], &y, sizeof(y));
    memcpy(&frame[12], &z, sizeof(z));
    frame[18] = 0x0AU;
    PathLineImu_OnAccelerationFrame(frame, (uint8_t)sizeof(frame), now_ms);
}

static void Test_Setup(void)
{
    uint8_t i;

    memset(&test_imu, 0, sizeof(test_imu));
    memset(test_wheels, 0, sizeof(test_wheels));
    test_imu.state = IMU_STATE_READY;
    test_imu.online = true;
    test_imu.yaw_valid = true;
    test_imu.gyro_valid = true;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        test_wheels[i].online = true;
    }
    PathLineImu_Init();
}

static void Test_RefreshWheels(uint32_t now_ms)
{
    uint8_t i;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        test_wheels[i].last_rx_ms = now_ms;
    }
}

static void Test_Calibrate(uint32_t *now_ms)
{
    path_line_imu_data_t data;

    for (; *now_ms <= 650U; ++(*now_ms))
    {
        Test_RefreshWheels(*now_ms);
        if ((*now_ms % 5U) == 0U)
        {
            Test_SendAcceleration(*now_ms, 0.20f, -0.10f, 9.81f);
        }
        PathLineImu_Run1ms(*now_ms);
    }

    assert(PathLineImu_GetData(&data));
    assert(data.accel_bias_ready);
    assert(data.zupt_active);
    assert(fabsf(data.accel_bias_x_mps2 - 0.20f) < 0.001f);
    assert(fabsf(data.accel_bias_y_mps2 + 0.10f) < 0.001f);
    assert(fabsf(data.imu_velocity_x_mps) < 0.000001f);
    assert(fabsf(data.imu_velocity_y_mps) < 0.000001f);
    assert(fabsf(data.encoder_weight - 0.50f) < 0.002f);
}

static void Test_EncoderDistanceScale(void)
{
    path_line_imu_data_t data;
    uint32_t now_ms = 1U;
    uint32_t end_ms;
    const float one_revolution_m =
        TEST_PI_F * PATH_LINE_IMU_WHEEL_DIAMETER_M;

    Test_Setup();
    Test_Calibrate(&now_ms);
    PathLineImu_ResetPosition();

    assert(fabsf(PATH_LINE_IMU_WHEEL_DIAMETER_M - 0.152f) < 0.000001f);
    assert(fabsf(PATH_LINE_IMU_WHEEL_RADIUS_M - 0.076f) < 0.000001f);
    assert(fabsf(PATH_LINE_IMU_GEAR_RATIO - 1.0f) < 0.000001f);

    /*
     * 60 mechanical RPM = 1 wheel revolution/s. 走 1 s 后，编码器路程
     * 应为一圈周长 pi*0.152；这同时锁定轮径、RPM 和秒的单位链。
     */
    test_wheels[CHASSIS_WHEEL_LF].actual_rpm = 60;
    test_wheels[CHASSIS_WHEEL_RF].actual_rpm = 60;
    test_wheels[CHASSIS_WHEEL_LR].actual_rpm = 60;
    test_wheels[CHASSIS_WHEEL_RR].actual_rpm = 60;
    end_ms = now_ms + 1000U;
    for (; now_ms < end_ms; ++now_ms)
    {
        Test_RefreshWheels(now_ms);
        PathLineImu_Run1ms(now_ms);
    }
    assert(PathLineImu_GetData(&data));
    assert(fabsf(data.encoder_body_velocity_x_mps - one_revolution_m) <
           0.00001f);
    assert(fabsf(data.encoder_position_x_m - one_revolution_m) < 0.001f);
    assert(fabsf(data.encoder_distance_m - one_revolution_m) < 0.001f);
    assert(fabsf(data.fused_distance_m - 0.5f * one_revolution_m) < 0.001f);

    /* 原路返回后位移接近零，但累计路程应接近两圈周长。 */
    test_wheels[CHASSIS_WHEEL_LF].actual_rpm = -60;
    test_wheels[CHASSIS_WHEEL_RF].actual_rpm = -60;
    test_wheels[CHASSIS_WHEEL_LR].actual_rpm = -60;
    test_wheels[CHASSIS_WHEEL_RR].actual_rpm = -60;
    end_ms = now_ms + 1000U;
    for (; now_ms < end_ms; ++now_ms)
    {
        Test_RefreshWheels(now_ms);
        PathLineImu_Run1ms(now_ms);
    }
    assert(PathLineImu_GetData(&data));
    assert(fabsf(data.encoder_position_x_m) < 0.001f);
    assert(fabsf(data.encoder_distance_m - 2.0f * one_revolution_m) <
           0.001f);
    assert(data.encoder_distance_m > fabsf(data.encoder_position_x_m));
    assert(data.fused_distance_m > fabsf(data.fused_position_x_m));

    PathLineImu_ResetPosition();
    assert(PathLineImu_GetData(&data));
    assert(data.imu_distance_m == 0.0f);
    assert(data.encoder_distance_m == 0.0f);
    assert(data.fused_distance_m == 0.0f);
}

static void Test_EncoderAndXIntegration(void)
{
    path_line_imu_data_t data;
    uint32_t now_ms = 1U;
    uint32_t end_ms;
    const float speed_at_60_rpm =
        TEST_PI_F * PATH_LINE_IMU_WHEEL_DIAMETER_M;

    Test_Setup();
    Test_Calibrate(&now_ms);

    /* 四轮均为 +60 mechanical RPM，应解算为底盘 +X。 */
    test_wheels[CHASSIS_WHEEL_LF].actual_rpm = 60;
    test_wheels[CHASSIS_WHEEL_RF].actual_rpm = 60;
    test_wheels[CHASSIS_WHEEL_LR].actual_rpm = 60;
    test_wheels[CHASSIS_WHEEL_RR].actual_rpm = 60;
    Test_RefreshWheels(now_ms);
    PathLineImu_Run1ms(now_ms++);
    assert(PathLineImu_GetData(&data));
    assert(fabsf(data.encoder_body_velocity_x_mps - speed_at_60_rpm) <
           0.00001f);
    assert(fabsf(data.encoder_body_velocity_y_mps) < 0.000001f);

    /*
     * 按原 Chassis_ApplyMotion 混控构造 vx=60、vy=30、rotation=100；
     * 逆解必须恢复平移量并严格消去 rotation。
     */
    test_wheels[CHASSIS_WHEEL_LF].actual_rpm = 190;
    test_wheels[CHASSIS_WHEEL_RF].actual_rpm = 130;
    test_wheels[CHASSIS_WHEEL_LR].actual_rpm = -70;
    test_wheels[CHASSIS_WHEEL_RR].actual_rpm = -10;
    Test_RefreshWheels(now_ms);
    PathLineImu_Run1ms(now_ms++);
    assert(PathLineImu_GetData(&data));
    assert(fabsf(data.encoder_body_velocity_x_mps - speed_at_60_rpm) <
           0.00001f);
    assert(fabsf(data.encoder_body_velocity_y_mps -
                 0.5f * speed_at_60_rpm) < 0.00001f);

    /* +Y 轮速组合在 yaw=90 度时应旋转为世界系 -X。 */
    test_imu.yaw_deg = 90.0f;
    test_wheels[CHASSIS_WHEEL_LF].actual_rpm = 60;
    test_wheels[CHASSIS_WHEEL_RF].actual_rpm = -60;
    test_wheels[CHASSIS_WHEEL_LR].actual_rpm = -60;
    test_wheels[CHASSIS_WHEEL_RR].actual_rpm = 60;
    Test_RefreshWheels(now_ms);
    PathLineImu_Run1ms(now_ms++);
    assert(PathLineImu_GetData(&data));
    assert(fabsf(data.encoder_body_velocity_x_mps) < 0.000001f);
    assert(fabsf(data.encoder_body_velocity_y_mps - speed_at_60_rpm) <
           0.00001f);
    assert(fabsf(data.encoder_world_velocity_x_mps + speed_at_60_rpm) <
           0.00001f);
    assert(fabsf(data.encoder_world_velocity_y_mps) < 0.00001f);

    /* 回到 yaw=0，持续 +X 加速度 1 秒，验证 X 积分和融合输出。 */
    test_imu.yaw_deg = 0.0f;
    test_wheels[CHASSIS_WHEEL_LF].actual_rpm = 60;
    test_wheels[CHASSIS_WHEEL_RF].actual_rpm = 60;
    test_wheels[CHASSIS_WHEEL_LR].actual_rpm = 60;
    test_wheels[CHASSIS_WHEEL_RR].actual_rpm = 60;
    end_ms = now_ms + 1000U;
    for (; now_ms < end_ms; ++now_ms)
    {
        Test_RefreshWheels(now_ms);
        if ((now_ms % 5U) == 0U)
        {
            Test_SendAcceleration(now_ms, 1.20f, -0.10f, 9.81f);
        }
        PathLineImu_Run1ms(now_ms);
    }
    assert(PathLineImu_GetData(&data));
    assert(data.imu_solution_valid);
    assert(!data.zupt_active);
    assert(data.imu_velocity_x_mps > 0.85f);
    assert(data.imu_velocity_x_mps < 1.10f);
    assert(fabsf(data.imu_velocity_y_mps) < 0.02f);
    assert(data.imu_position_x_m > 0.35f);
    assert(data.imu_distance_m >= fabsf(data.imu_position_x_m));
    assert(data.fused_velocity_x_mps > 0.60f);
    assert(data.fused_distance_m >= fabsf(data.fused_position_x_m));

    /* 停止刷新反馈，先检查 60~1000 ms 区间内的线性降权。 */
    end_ms = now_ms + 500U;
    for (; now_ms < end_ms; ++now_ms)
    {
        PathLineImu_Run1ms(now_ms);
    }
    assert(PathLineImu_GetData(&data));
    assert(data.encoder_age_ms >= 500U);
    assert(data.encoder_weight > 0.20f);
    assert(data.encoder_weight < 0.35f);

    /* 超过 1000 ms 并通过三周期确认后，应判离线且权重归零。 */
    end_ms = now_ms + 600U;
    for (; now_ms < end_ms; ++now_ms)
    {
        PathLineImu_Run1ms(now_ms);
    }
    assert(PathLineImu_GetData(&data));
    assert(data.encoder_age_ms > 1000U);
    assert(data.encoder_offline);
    assert(data.encoder_weight < 0.001f);
    assert(!data.encoder_solution_valid);

    /* 连续五次新鲜反馈后恢复，权重重新开始爬升。 */
    end_ms = now_ms + 5U;
    for (; now_ms < end_ms; ++now_ms)
    {
        Test_RefreshWheels(now_ms);
        PathLineImu_Run1ms(now_ms);
    }
    assert(PathLineImu_GetData(&data));
    assert(!data.encoder_offline);
    assert(data.encoder_solution_valid);
    assert(data.encoder_weight > 0.0f);

    /* NaN 不得进入积分队列。 */
    {
        const uint32_t invalid_before = data.accel_invalid_count;
        Test_SendAcceleration(now_ms, NAN, 0.0f, 0.0f);
        PathLineImu_Run1ms(now_ms);
        assert(PathLineImu_GetData(&data));
        assert(data.accel_invalid_count == invalid_before + 1U);
    }
}

static void Test_YIntegration(void)
{
    path_line_imu_data_t data;
    uint32_t now_ms = 1U;
    uint32_t end_ms;

    Test_Setup();
    Test_Calibrate(&now_ms);

    /* 非零编码器速度阻止 ZUPT，单独验证第二个平移轴的积分。 */
    test_wheels[CHASSIS_WHEEL_LF].actual_rpm = 60;
    test_wheels[CHASSIS_WHEEL_RF].actual_rpm = -60;
    test_wheels[CHASSIS_WHEEL_LR].actual_rpm = -60;
    test_wheels[CHASSIS_WHEEL_RR].actual_rpm = 60;
    end_ms = now_ms + 1000U;
    for (; now_ms < end_ms; ++now_ms)
    {
        Test_RefreshWheels(now_ms);
        if ((now_ms % 5U) == 0U)
        {
            Test_SendAcceleration(now_ms, 0.20f, 0.90f, 9.81f);
        }
        PathLineImu_Run1ms(now_ms);
    }

    assert(PathLineImu_GetData(&data));
    assert(data.imu_solution_valid);
    assert(!data.zupt_active);
    assert(fabsf(data.imu_velocity_x_mps) < 0.02f);
    assert(data.imu_velocity_y_mps > 0.85f);
    assert(data.imu_velocity_y_mps < 1.10f);
    assert(data.imu_position_y_m > 0.35f);
    assert(data.imu_distance_m >= fabsf(data.imu_position_y_m));
}

int main(void)
{
    Test_EncoderDistanceScale();
    Test_EncoderAndXIntegration();
    Test_YIntegration();
    puts("path_line_imu host tests: OK");
    return 0;
}
