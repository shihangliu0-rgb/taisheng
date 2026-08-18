/**
 * @file path_line_imu.c
 * @brief 二维 IMU/四轮 VESC 自适应权重融合里程计。
 */
#include "path_line_imu.h"

#include "chassis_main.h"
#include "imu_main.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PATH_LINE_IMU_PI_F                  3.14159265358979323846f
#define PATH_LINE_IMU_DEG_TO_RAD_F          (PATH_LINE_IMU_PI_F / 180.0f)

/* actual_rpm 已由 VESC 层换算为机械 RPM，此处不能再次除以极对数。 */
#define PATH_LINE_IMU_RPM_TO_MPS            \
    (2.0f * PATH_LINE_IMU_PI_F * PATH_LINE_IMU_WHEEL_RADIUS_M / \
     (60.0f * PATH_LINE_IMU_GEAR_RATIO))

#define PATH_LINE_IMU_ACCEL_FRAME_TYPE      0x01U
#define PATH_LINE_IMU_FRAME_LENGTH          19U
#define PATH_LINE_IMU_ACCEL_MAX_MPS2        100.0f
#define PATH_LINE_IMU_ACCEL_STALE_MS        100U
#define PATH_LINE_IMU_NOMINAL_ACCEL_DT_S    0.005f
#define PATH_LINE_IMU_MAX_ACCEL_DT_MS       20U
#define PATH_LINE_IMU_MAX_RUN_DT_MS         20U

/* 二阶 Butterworth 低通：采样 200 Hz、截止 25 Hz。 */
#define PATH_LINE_IMU_LPF_B0                0.09763107f
#define PATH_LINE_IMU_LPF_B1                0.19526215f
#define PATH_LINE_IMU_LPF_B2                0.09763107f
#define PATH_LINE_IMU_LPF_A1               (-0.94280904f)
#define PATH_LINE_IMU_LPF_A2                0.33333333f

#define PATH_LINE_IMU_STATIC_WINDOW         100U
#define PATH_LINE_IMU_ACCEL_STD_MAX_MPS2    0.08f
#define PATH_LINE_IMU_GYRO_STATIC_MAX_DPS   0.50f
#define PATH_LINE_IMU_ACCEL_BIAS_NEAR_MPS2  0.15f
#define PATH_LINE_IMU_ENCODER_STATIC_MPS    0.03f
#define PATH_LINE_IMU_BIAS_TRACK_ALPHA      0.001f

/* 仅提取参考实现中的编码器反馈年龄自适应权重参数。 */
#define PATH_LINE_IMU_ENCODER_FRESH_MS      60U
#define PATH_LINE_IMU_ENCODER_DROPOUT_MS    1000U
#define PATH_LINE_IMU_ENCODER_DROP_CONFIRM  3U
#define PATH_LINE_IMU_ENCODER_OK_CONFIRM    5U
#define PATH_LINE_IMU_ENCODER_WEIGHT_MAX    0.50f
#define PATH_LINE_IMU_WEIGHT_RATE_PER_S     1.0f

#define PATH_LINE_IMU_ACCEL_QUEUE_SIZE      16U

typedef struct
{
    float x;
    float y;
    float z;
    uint32_t time_ms;
} path_line_accel_sample_t;

typedef struct
{
    float x1;
    float x2;
    float y1;
    float y2;
    bool initialized;
} path_line_biquad_t;

typedef struct
{
    path_line_imu_data_t data;

    path_line_accel_sample_t accel_queue[PATH_LINE_IMU_ACCEL_QUEUE_SIZE];
    uint8_t queue_head;
    uint8_t queue_tail;
    uint8_t queue_count;

    path_line_biquad_t filter_x;
    path_line_biquad_t filter_y;
    float history_x[PATH_LINE_IMU_STATIC_WINDOW];
    float history_y[PATH_LINE_IMU_STATIC_WINDOW];
    uint16_t history_index;
    uint16_t history_count;

    float previous_world_accel_x;
    float previous_world_accel_y;
    float previous_encoder_world_velocity_x;
    float previous_encoder_world_velocity_y;
    float previous_encoder_speed;
    float previous_fused_velocity_x;
    float previous_fused_velocity_y;
    uint32_t previous_accel_ms;
    uint32_t previous_run_ms;
    bool have_previous_accel;
    bool have_previous_encoder_speed;
    bool have_previous_encoder_world_velocity;
    bool have_previous_run;

    uint8_t encoder_bad_count;
    uint8_t encoder_good_count;
    bool initialized;
} path_line_imu_context_t;

static path_line_imu_context_t s_odometry;

static float PathLineImu_ReadFloatLe(const uint8_t *bytes)
{
    float value;
    uint8_t raw[sizeof(float)];

    raw[0] = bytes[0];
    raw[1] = bytes[1];
    raw[2] = bytes[2];
    raw[3] = bytes[3];
    memcpy(&value, raw, sizeof(value));
    return value;
}

static bool PathLineImu_IsFinite(float value)
{
    return (isnan(value) == 0) && (isinf(value) == 0);
}

static float PathLineImu_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float PathLineImu_VectorNorm(float x, float y)
{
    return sqrtf(x * x + y * y);
}

static float PathLineImu_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float PathLineImu_Filter(path_line_biquad_t *filter, float input)
{
    float output;

    if (!filter->initialized)
    {
        /* 用首样本初始化稳态，避免启动瞬态被误认为运动。 */
        filter->x1 = input;
        filter->x2 = input;
        filter->y1 = input;
        filter->y2 = input;
        filter->initialized = true;
        return input;
    }

    output = PATH_LINE_IMU_LPF_B0 * input +
             PATH_LINE_IMU_LPF_B1 * filter->x1 +
             PATH_LINE_IMU_LPF_B2 * filter->x2 -
             PATH_LINE_IMU_LPF_A1 * filter->y1 -
             PATH_LINE_IMU_LPF_A2 * filter->y2;

    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;
    return output;
}

static void PathLineImu_RotateBodyToWorld(float body_x,
                                          float body_y,
                                          float yaw_deg,
                                          float *world_x,
                                          float *world_y)
{
    const float yaw_rad = yaw_deg * PATH_LINE_IMU_DEG_TO_RAD_F;
    const float cos_yaw = cosf(yaw_rad);
    const float sin_yaw = sinf(yaw_rad);

    *world_x = cos_yaw * body_x - sin_yaw * body_y;
    *world_y = sin_yaw * body_x + cos_yaw * body_y;
}

static void PathLineImu_AddHistory(float accel_x, float accel_y)
{
    s_odometry.history_x[s_odometry.history_index] = accel_x;
    s_odometry.history_y[s_odometry.history_index] = accel_y;
    s_odometry.history_index =
        (uint16_t)((s_odometry.history_index + 1U) % PATH_LINE_IMU_STATIC_WINDOW);
    if (s_odometry.history_count < PATH_LINE_IMU_STATIC_WINDOW)
    {
        s_odometry.history_count++;
    }
}

static bool PathLineImu_GetHistoryStatistics(float *mean_x,
                                             float *mean_y,
                                             float *std_x,
                                             float *std_y)
{
    uint16_t i;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float variance_x = 0.0f;
    float variance_y = 0.0f;

    if (s_odometry.history_count < PATH_LINE_IMU_STATIC_WINDOW)
    {
        return false;
    }

    for (i = 0U; i < PATH_LINE_IMU_STATIC_WINDOW; ++i)
    {
        sum_x += s_odometry.history_x[i];
        sum_y += s_odometry.history_y[i];
    }
    *mean_x = sum_x / (float)PATH_LINE_IMU_STATIC_WINDOW;
    *mean_y = sum_y / (float)PATH_LINE_IMU_STATIC_WINDOW;

    for (i = 0U; i < PATH_LINE_IMU_STATIC_WINDOW; ++i)
    {
        const float dx = s_odometry.history_x[i] - *mean_x;
        const float dy = s_odometry.history_y[i] - *mean_y;
        variance_x += dx * dx;
        variance_y += dy * dy;
    }

    *std_x = sqrtf(variance_x / (float)PATH_LINE_IMU_STATIC_WINDOW);
    *std_y = sqrtf(variance_y / (float)PATH_LINE_IMU_STATIC_WINDOW);
    return true;
}

static bool PathLineImu_PopAcceleration(path_line_accel_sample_t *sample)
{
    if ((sample == NULL) || (s_odometry.queue_count == 0U))
    {
        return false;
    }

    *sample = s_odometry.accel_queue[s_odometry.queue_tail];
    s_odometry.queue_tail =
        (uint8_t)((s_odometry.queue_tail + 1U) % PATH_LINE_IMU_ACCEL_QUEUE_SIZE);
    s_odometry.queue_count--;
    return true;
}

static bool PathLineImu_ReadEncoder(uint32_t now_ms,
                                    float *body_velocity_x,
                                    float *body_velocity_y,
                                    uint32_t *age_ms)
{
    vesc_motor_status_t left_front;
    vesc_motor_status_t right_front;
    vesc_motor_status_t left_rear;
    vesc_motor_status_t right_rear;
    float lf_mps;
    float rf_mps;
    float lr_mps;
    float rr_mps;
    uint32_t wheel_age;
    uint32_t oldest_age = 0U;

    if ((body_velocity_x == NULL) || (body_velocity_y == NULL) ||
        (age_ms == NULL))
    {
        return false;
    }

    if (!Chassis_GetStatus(CHASSIS_WHEEL_LF, &left_front) ||
        !Chassis_GetStatus(CHASSIS_WHEEL_RF, &right_front) ||
        !Chassis_GetStatus(CHASSIS_WHEEL_LR, &left_rear) ||
        !Chassis_GetStatus(CHASSIS_WHEEL_RR, &right_rear))
    {
        *age_ms = PATH_LINE_IMU_AGE_INVALID;
        return false;
    }

    /* 所有四轮均收到过数据后才计算，年龄取四轮中最旧的一帧。 */
    if ((left_front.last_rx_ms == 0U) || (right_front.last_rx_ms == 0U) ||
        (left_rear.last_rx_ms == 0U) || (right_rear.last_rx_ms == 0U))
    {
        *age_ms = PATH_LINE_IMU_AGE_INVALID;
        return false;
    }

    wheel_age = now_ms - left_front.last_rx_ms;
    if (wheel_age > oldest_age)
    {
        oldest_age = wheel_age;
    }
    wheel_age = now_ms - right_front.last_rx_ms;
    if (wheel_age > oldest_age)
    {
        oldest_age = wheel_age;
    }
    wheel_age = now_ms - left_rear.last_rx_ms;
    if (wheel_age > oldest_age)
    {
        oldest_age = wheel_age;
    }
    wheel_age = now_ms - right_rear.last_rx_ms;
    if (wheel_age > oldest_age)
    {
        oldest_age = wheel_age;
    }

    lf_mps = left_front.actual_rpm * PATH_LINE_IMU_RPM_TO_MPS;
    rf_mps = right_front.actual_rpm * PATH_LINE_IMU_RPM_TO_MPS;
    lr_mps = left_rear.actual_rpm * PATH_LINE_IMU_RPM_TO_MPS;
    rr_mps = right_rear.actual_rpm * PATH_LINE_IMU_RPM_TO_MPS;

    /* 与 Chassis_SetVelocity 的四轮混控矩阵严格互逆。 */
    *body_velocity_x = 0.25f * (lf_mps + rf_mps + lr_mps + rr_mps);
    *body_velocity_y = 0.25f * (lf_mps - rf_mps - lr_mps + rr_mps);
    *age_ms = oldest_age;
    return true;
}

static void PathLineImu_UpdateEncoderWeight(bool have_encoder,
                                            uint32_t encoder_age_ms,
                                            float dt_s)
{
    bool fresh = have_encoder &&
                 (encoder_age_ms <= PATH_LINE_IMU_ENCODER_FRESH_MS);
    bool dropped = !have_encoder ||
                   (encoder_age_ms > PATH_LINE_IMU_ENCODER_DROPOUT_MS);
    float target_weight;
    float max_step;

    if (dropped)
    {
        if (s_odometry.encoder_bad_count < PATH_LINE_IMU_ENCODER_DROP_CONFIRM)
        {
            s_odometry.encoder_bad_count++;
        }
        s_odometry.encoder_good_count = 0U;
        if (s_odometry.encoder_bad_count >= PATH_LINE_IMU_ENCODER_DROP_CONFIRM)
        {
            s_odometry.data.encoder_offline = true;
        }
    }
    else if (fresh)
    {
        if (s_odometry.encoder_good_count < PATH_LINE_IMU_ENCODER_OK_CONFIRM)
        {
            s_odometry.encoder_good_count++;
        }
        s_odometry.encoder_bad_count = 0U;
        if (s_odometry.encoder_good_count >= PATH_LINE_IMU_ENCODER_OK_CONFIRM)
        {
            s_odometry.data.encoder_offline = false;
        }
    }
    else
    {
        s_odometry.encoder_bad_count = 0U;
        s_odometry.encoder_good_count = 0U;
    }

    if (!have_encoder || s_odometry.data.encoder_offline)
    {
        target_weight = 0.0f;
    }
    else if (encoder_age_ms <= PATH_LINE_IMU_ENCODER_FRESH_MS)
    {
        target_weight = PATH_LINE_IMU_ENCODER_WEIGHT_MAX;
    }
    else if (encoder_age_ms >= PATH_LINE_IMU_ENCODER_DROPOUT_MS)
    {
        target_weight = 0.0f;
    }
    else
    {
        target_weight = PATH_LINE_IMU_ENCODER_WEIGHT_MAX *
            ((float)(PATH_LINE_IMU_ENCODER_DROPOUT_MS - encoder_age_ms) /
             (float)(PATH_LINE_IMU_ENCODER_DROPOUT_MS -
                     PATH_LINE_IMU_ENCODER_FRESH_MS));
    }

    max_step = PATH_LINE_IMU_WEIGHT_RATE_PER_S * dt_s;
    if (s_odometry.data.encoder_weight < target_weight)
    {
        s_odometry.data.encoder_weight = PathLineImu_Clamp(
            s_odometry.data.encoder_weight + max_step,
            0.0f,
            target_weight);
    }
    else if (s_odometry.data.encoder_weight > target_weight)
    {
        s_odometry.data.encoder_weight = PathLineImu_Clamp(
            s_odometry.data.encoder_weight - max_step,
            target_weight,
            PATH_LINE_IMU_ENCODER_WEIGHT_MAX);
    }
}

static void PathLineImu_ProcessAcceleration(const path_line_accel_sample_t *sample,
                                            const imu_data_t *imu,
                                            bool encoder_stationary)
{
    float filtered_x;
    float filtered_y;
    float mean_x = 0.0f;
    float mean_y = 0.0f;
    float std_x = 0.0f;
    float std_y = 0.0f;
    float corrected_x;
    float corrected_y;
    float world_x;
    float world_y;
    float dt_s = 0.0f;
    bool have_statistics;
    bool gyro_stationary;
    bool bias_near;
    bool static_candidate;
    bool imu_ready;

    filtered_x = PathLineImu_Filter(&s_odometry.filter_x, sample->x);
    filtered_y = PathLineImu_Filter(&s_odometry.filter_y, sample->y);
    PathLineImu_AddHistory(filtered_x, filtered_y);
    have_statistics = PathLineImu_GetHistoryStatistics(&mean_x,
                                                       &mean_y,
                                                       &std_x,
                                                       &std_y);

    imu_ready = (imu != NULL) && imu->online && imu->gyro_valid &&
                imu->yaw_valid && (imu->state == IMU_STATE_READY);
    gyro_stationary = imu_ready &&
                      (PathLineImu_Abs(imu->gyro_z_deg_s) <=
                       PATH_LINE_IMU_GYRO_STATIC_MAX_DPS);

    if (!s_odometry.data.accel_bias_ready)
    {
        bias_near = true;
    }
    else
    {
        bias_near = (PathLineImu_Abs(mean_x -
                                     s_odometry.data.accel_bias_x_mps2) <=
                     PATH_LINE_IMU_ACCEL_BIAS_NEAR_MPS2) &&
                    (PathLineImu_Abs(mean_y -
                                     s_odometry.data.accel_bias_y_mps2) <=
                     PATH_LINE_IMU_ACCEL_BIAS_NEAR_MPS2);
    }

    static_candidate = have_statistics && gyro_stationary &&
                       encoder_stationary && bias_near &&
                       (std_x <= PATH_LINE_IMU_ACCEL_STD_MAX_MPS2) &&
                       (std_y <= PATH_LINE_IMU_ACCEL_STD_MAX_MPS2);

    if (!s_odometry.data.accel_bias_ready && static_candidate)
    {
        s_odometry.data.accel_bias_x_mps2 = mean_x;
        s_odometry.data.accel_bias_y_mps2 = mean_y;
        s_odometry.data.accel_bias_ready = true;
        s_odometry.have_previous_accel = false;
        s_odometry.data.imu_velocity_x_mps = 0.0f;
        s_odometry.data.imu_velocity_y_mps = 0.0f;
    }
    else if (s_odometry.data.accel_bias_ready && static_candidate)
    {
        s_odometry.data.accel_bias_x_mps2 += PATH_LINE_IMU_BIAS_TRACK_ALPHA *
            (mean_x - s_odometry.data.accel_bias_x_mps2);
        s_odometry.data.accel_bias_y_mps2 += PATH_LINE_IMU_BIAS_TRACK_ALPHA *
            (mean_y - s_odometry.data.accel_bias_y_mps2);
    }

    s_odometry.data.zupt_active = s_odometry.data.accel_bias_ready &&
                                  static_candidate;
    corrected_x = filtered_x - s_odometry.data.accel_bias_x_mps2;
    corrected_y = filtered_y - s_odometry.data.accel_bias_y_mps2;

    if (imu_ready)
    {
        PathLineImu_RotateBodyToWorld(corrected_x,
                                      corrected_y,
                                      imu->yaw_deg,
                                      &world_x,
                                      &world_y);
    }
    else
    {
        world_x = 0.0f;
        world_y = 0.0f;
    }

    s_odometry.data.body_accel_x_mps2 = corrected_x;
    s_odometry.data.body_accel_y_mps2 = corrected_y;
    s_odometry.data.world_accel_x_mps2 = world_x;
    s_odometry.data.world_accel_y_mps2 = world_y;
    s_odometry.data.imu_solution_valid = imu_ready &&
                                         s_odometry.data.accel_bias_ready;

    if (s_odometry.have_previous_accel)
    {
        const uint32_t delta_ms = sample->time_ms - s_odometry.previous_accel_ms;
        if ((delta_ms > 0U) && (delta_ms <= PATH_LINE_IMU_MAX_ACCEL_DT_MS))
        {
            dt_s = (float)delta_ms * 0.001f;
        }
        else if (delta_ms == 0U)
        {
            /* 同一次 DMA 解析出的连续帧使用传感器标称 200 Hz 周期。 */
            dt_s = PATH_LINE_IMU_NOMINAL_ACCEL_DT_S;
        }
    }

    if (!s_odometry.data.imu_solution_valid)
    {
        s_odometry.have_previous_accel = false;
    }
    else if (s_odometry.data.zupt_active)
    {
        s_odometry.data.imu_velocity_x_mps = 0.0f;
        s_odometry.data.imu_velocity_y_mps = 0.0f;
        s_odometry.have_previous_accel = true;
    }
    else if (s_odometry.have_previous_accel && (dt_s > 0.0f))
    {
        const float old_velocity_x = s_odometry.data.imu_velocity_x_mps;
        const float old_velocity_y = s_odometry.data.imu_velocity_y_mps;
        const float old_speed = PathLineImu_VectorNorm(old_velocity_x,
                                                       old_velocity_y);
        float new_speed;

        s_odometry.data.imu_velocity_x_mps +=
            0.5f * (s_odometry.previous_world_accel_x + world_x) * dt_s;
        s_odometry.data.imu_velocity_y_mps +=
            0.5f * (s_odometry.previous_world_accel_y + world_y) * dt_s;
        new_speed = PathLineImu_VectorNorm(
            s_odometry.data.imu_velocity_x_mps,
            s_odometry.data.imu_velocity_y_mps);
        s_odometry.data.imu_position_x_m +=
            0.5f * (old_velocity_x + s_odometry.data.imu_velocity_x_mps) * dt_s;
        s_odometry.data.imu_position_y_m +=
            0.5f * (old_velocity_y + s_odometry.data.imu_velocity_y_mps) * dt_s;
        s_odometry.data.imu_distance_m +=
            0.5f * (old_speed + new_speed) * dt_s;
    }
    else
    {
        s_odometry.have_previous_accel = true;
    }

    s_odometry.previous_world_accel_x = world_x;
    s_odometry.previous_world_accel_y = world_y;
    s_odometry.previous_accel_ms = sample->time_ms;
}

void PathLineImu_Init(void)
{
    memset(&s_odometry, 0, sizeof(s_odometry));
    s_odometry.data.accel_age_ms = PATH_LINE_IMU_AGE_INVALID;
    s_odometry.data.encoder_age_ms = PATH_LINE_IMU_AGE_INVALID;
    s_odometry.data.encoder_offline = true;
    s_odometry.initialized = true;
}

void PathLineImu_OnAccelerationFrame(const uint8_t *frame,
                                     uint8_t length,
                                     uint32_t now_ms)
{
    path_line_accel_sample_t sample;
    uint8_t write_index;

    if (!s_odometry.initialized)
    {
        return;
    }

    if ((frame == NULL) || (length != PATH_LINE_IMU_FRAME_LENGTH) ||
        (frame[3] != PATH_LINE_IMU_ACCEL_FRAME_TYPE))
    {
        s_odometry.data.accel_invalid_count++;
        return;
    }

    sample.x = PathLineImu_ReadFloatLe(&frame[4]);
    sample.y = PathLineImu_ReadFloatLe(&frame[8]);
    sample.z = PathLineImu_ReadFloatLe(&frame[12]);
    sample.time_ms = now_ms;

    if (!PathLineImu_IsFinite(sample.x) ||
        !PathLineImu_IsFinite(sample.y) ||
        !PathLineImu_IsFinite(sample.z) ||
        (PathLineImu_Abs(sample.x) > PATH_LINE_IMU_ACCEL_MAX_MPS2) ||
        (PathLineImu_Abs(sample.y) > PATH_LINE_IMU_ACCEL_MAX_MPS2) ||
        (PathLineImu_Abs(sample.z) > PATH_LINE_IMU_ACCEL_MAX_MPS2))
    {
        s_odometry.data.accel_invalid_count++;
        return;
    }

    /* 用户指定的轴映射：传感器 +X -> 底盘 +X，+Y -> 底盘 +Y。 */
    if (s_odometry.queue_count >= PATH_LINE_IMU_ACCEL_QUEUE_SIZE)
    {
        s_odometry.queue_tail =
            (uint8_t)((s_odometry.queue_tail + 1U) %
                      PATH_LINE_IMU_ACCEL_QUEUE_SIZE);
        s_odometry.queue_count--;
        s_odometry.data.accel_queue_overflow_count++;
    }

    write_index = s_odometry.queue_head;
    s_odometry.accel_queue[write_index] = sample;
    s_odometry.queue_head =
        (uint8_t)((s_odometry.queue_head + 1U) % PATH_LINE_IMU_ACCEL_QUEUE_SIZE);
    s_odometry.queue_count++;

    s_odometry.data.sensor_accel_x_mps2 = sample.x;
    s_odometry.data.sensor_accel_y_mps2 = sample.y;
    s_odometry.data.sensor_accel_z_mps2 = sample.z;
    s_odometry.data.last_accel_ms = now_ms;
    s_odometry.data.accel_frame_count++;
}

void PathLineImu_Run1ms(uint32_t now_ms)
{
    imu_data_t imu_data;
    path_line_accel_sample_t sample;
    float encoder_body_x = 0.0f;
    float encoder_body_y = 0.0f;
    float encoder_world_x = 0.0f;
    float encoder_world_y = 0.0f;
    float encoder_speed;
    float dt_s = 0.0f;
    float imu_weight;
    bool have_imu_data;
    bool have_encoder;
    bool encoder_fresh;
    bool encoder_world_valid;
    bool encoder_stationary;

    if (!s_odometry.initialized)
    {
        return;
    }

    if (s_odometry.have_previous_run)
    {
        const uint32_t delta_ms = now_ms - s_odometry.previous_run_ms;
        if ((delta_ms > 0U) && (delta_ms <= PATH_LINE_IMU_MAX_RUN_DT_MS))
        {
            dt_s = (float)delta_ms * 0.001f;
        }
        else if (delta_ms > PATH_LINE_IMU_MAX_RUN_DT_MS)
        {
            dt_s = 0.0f;
        }
    }
    else
    {
        s_odometry.have_previous_run = true;
        dt_s = 0.0f;
    }
    s_odometry.previous_run_ms = now_ms;

    have_imu_data = ImuMain_GetData(&imu_data);
    if (!have_imu_data)
    {
        memset(&imu_data, 0, sizeof(imu_data));
    }

    have_encoder = PathLineImu_ReadEncoder(now_ms,
                                           &encoder_body_x,
                                           &encoder_body_y,
                                           &s_odometry.data.encoder_age_ms);
    encoder_speed = PathLineImu_VectorNorm(encoder_body_x, encoder_body_y);
    encoder_fresh = have_encoder &&
                    (s_odometry.data.encoder_age_ms <=
                     PATH_LINE_IMU_ENCODER_FRESH_MS);
    encoder_stationary = !have_encoder ||
                         (s_odometry.data.encoder_age_ms >
                          PATH_LINE_IMU_ENCODER_DROPOUT_MS) ||
                         (encoder_speed <= PATH_LINE_IMU_ENCODER_STATIC_MPS);
    encoder_world_valid = have_imu_data && imu_data.online &&
                          imu_data.yaw_valid;

    if (encoder_world_valid)
    {
        PathLineImu_RotateBodyToWorld(encoder_body_x,
                                      encoder_body_y,
                                      imu_data.yaw_deg,
                                      &encoder_world_x,
                                      &encoder_world_y);
    }

    s_odometry.data.encoder_body_velocity_x_mps = encoder_body_x;
    s_odometry.data.encoder_body_velocity_y_mps = encoder_body_y;
    s_odometry.data.encoder_world_velocity_x_mps = encoder_world_x;
    s_odometry.data.encoder_world_velocity_y_mps = encoder_world_y;

    /*
     * 仅在四轮反馈新鲜时积分编码器里程。累计路程积分速度模长，
     * 因而车辆返回起点后路程仍会保留，而位置坐标可以回到零附近。
     */
    if (encoder_fresh)
    {
        if ((dt_s > 0.0f) && s_odometry.have_previous_encoder_speed)
        {
            s_odometry.data.encoder_distance_m +=
                0.5f * (s_odometry.previous_encoder_speed + encoder_speed) *
                dt_s;
        }
        s_odometry.previous_encoder_speed = encoder_speed;
        s_odometry.have_previous_encoder_speed = true;

        if (encoder_world_valid)
        {
            if ((dt_s > 0.0f) &&
                s_odometry.have_previous_encoder_world_velocity)
            {
                s_odometry.data.encoder_position_x_m +=
                    0.5f *
                    (s_odometry.previous_encoder_world_velocity_x +
                     encoder_world_x) * dt_s;
                s_odometry.data.encoder_position_y_m +=
                    0.5f *
                    (s_odometry.previous_encoder_world_velocity_y +
                     encoder_world_y) * dt_s;
            }
            s_odometry.previous_encoder_world_velocity_x = encoder_world_x;
            s_odometry.previous_encoder_world_velocity_y = encoder_world_y;
            s_odometry.have_previous_encoder_world_velocity = true;
        }
        else
        {
            s_odometry.have_previous_encoder_world_velocity = false;
        }
    }
    else
    {
        /* 不跨反馈缺口做梯形积分，避免用断线前速度补整段缺口。 */
        s_odometry.have_previous_encoder_speed = false;
        s_odometry.have_previous_encoder_world_velocity = false;
    }

    PathLineImu_UpdateEncoderWeight(have_encoder,
                                    s_odometry.data.encoder_age_ms,
                                    dt_s);
    s_odometry.data.encoder_solution_valid = have_encoder &&
        !s_odometry.data.encoder_offline &&
        (s_odometry.data.encoder_age_ms <= PATH_LINE_IMU_ENCODER_DROPOUT_MS);

    while (PathLineImu_PopAcceleration(&sample))
    {
        PathLineImu_ProcessAcceleration(&sample,
                                        have_imu_data ? &imu_data : NULL,
                                        encoder_stationary);
    }

    if (s_odometry.data.accel_frame_count > 0U)
    {
        s_odometry.data.accel_age_ms =
            now_ms - s_odometry.data.last_accel_ms;
    }
    else
    {
        s_odometry.data.accel_age_ms = PATH_LINE_IMU_AGE_INVALID;
    }
    if (s_odometry.data.accel_age_ms > PATH_LINE_IMU_ACCEL_STALE_MS)
    {
        s_odometry.data.imu_solution_valid = false;
        s_odometry.data.zupt_active = false;
        s_odometry.have_previous_accel = false;
    }

    imu_weight = 1.0f - s_odometry.data.encoder_weight;
    s_odometry.data.fused_velocity_x_mps =
        imu_weight * s_odometry.data.imu_velocity_x_mps +
        s_odometry.data.encoder_weight * encoder_world_x;
    s_odometry.data.fused_velocity_y_mps =
        imu_weight * s_odometry.data.imu_velocity_y_mps +
        s_odometry.data.encoder_weight * encoder_world_y;

    if (dt_s > 0.0f)
    {
        const float previous_fused_speed = PathLineImu_VectorNorm(
            s_odometry.previous_fused_velocity_x,
            s_odometry.previous_fused_velocity_y);
        const float fused_speed = PathLineImu_VectorNorm(
            s_odometry.data.fused_velocity_x_mps,
            s_odometry.data.fused_velocity_y_mps);

        s_odometry.data.fused_position_x_m +=
            0.5f * (s_odometry.previous_fused_velocity_x +
                    s_odometry.data.fused_velocity_x_mps) * dt_s;
        s_odometry.data.fused_position_y_m +=
            0.5f * (s_odometry.previous_fused_velocity_y +
                    s_odometry.data.fused_velocity_y_mps) * dt_s;
        s_odometry.data.fused_distance_m +=
            0.5f * (previous_fused_speed + fused_speed) * dt_s;
    }
    s_odometry.previous_fused_velocity_x =
        s_odometry.data.fused_velocity_x_mps;
    s_odometry.previous_fused_velocity_y =
        s_odometry.data.fused_velocity_y_mps;
}

bool PathLineImu_GetData(path_line_imu_data_t *data)
{
    if ((data == NULL) || !s_odometry.initialized)
    {
        return false;
    }

    *data = s_odometry.data;
    return true;
}

void PathLineImu_ResetPosition(void)
{
    if (!s_odometry.initialized)
    {
        return;
    }

    s_odometry.data.imu_position_x_m = 0.0f;
    s_odometry.data.imu_position_y_m = 0.0f;
    s_odometry.data.imu_distance_m = 0.0f;
    s_odometry.data.encoder_position_x_m = 0.0f;
    s_odometry.data.encoder_position_y_m = 0.0f;
    s_odometry.data.encoder_distance_m = 0.0f;
    s_odometry.data.fused_position_x_m = 0.0f;
    s_odometry.data.fused_position_y_m = 0.0f;
    s_odometry.data.fused_distance_m = 0.0f;
}
