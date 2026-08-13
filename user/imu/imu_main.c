#include "imu_main.h"

#include "imu.h"
#include "usart.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* IMU 启动时序和偏航角上报协议。 */
#define IMU_YAW_FRAME_HEADER_0     0xA5U
#define IMU_YAW_FRAME_HEADER_1     0x5CU
#define IMU_YAW_FRAME_LENGTH       5U

/* 偏航保持控制器参数。 */
#define IMU_ANGLE_HALF_RANGE_DEG 180.0f
#define IMU_ANGLE_RANGE_DEG      360.0f

enum
{
    IMU_YAW_PID_MOVE,
    IMU_YAW_PID_STOP,
    IMU_YAW_PID_COUNT
};

static const imu_config_t imu_config = {
    .boot_delay_ms = 100U,
    .cal_cmd_delay_ms = 50U,
    .gyro_cal_wait_ms = 4000U,
    .config_start_delay_ms = 100U,
    .config_cmd_delay_ms = 20U,
    .gyro_bias_samples = 200U,
    .online_timeout_ms = 100U,
    .yaw_tx_period_ms = 50U,
    .yaw_control_period_ms = 5U,
    .yaw_cmd_threshold = 5,
    .yaw_linear_threshold = 5,
    .kalman_q = 0.02f,
    .kalman_r = 3.0f,
    .gyro_filter_q = 0.1f,
    .gyro_filter_r = 2.0f,
    .yaw_tx_scale = 100.0f,
    .yaw_deadzone_deg = 0.5f,
    .yaw_i_active_deg = 10.0f,
    .yaw_i_decay = 0.90f,
    .yaw_gyro_k = 5.0f
};

typedef enum
{
    IMU_INIT_WAIT_BOOT,
    IMU_INIT_SEND_CAL,
    IMU_INIT_WAIT_CAL,
    IMU_INIT_WAIT_CONFIG,
    IMU_INIT_SEND_CONFIG,
    IMU_INIT_SAMPLE_BIAS,
    IMU_INIT_COMPLETE,
    IMU_INIT_ERROR
} imu_init_step_t;

typedef struct
{
    imu_init_step_t step;       // 当前初始化步骤
    uint32_t next_action_ms;
    uint32_t last_gyro_sequence;   
    uint32_t last_yaw_sequence;
    float gyro_bias_sum_deg_s;
    uint16_t gyro_bias_sample_count;
    uint8_t config_index;
} imu_init_context_t;

enum
{
    IMU_COMMAND_MAX_LENGTH = 5U
};

typedef struct
{
    uint8_t data[IMU_COMMAND_MAX_LENGTH];
    uint8_t length;
} imu_command_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float i_max;
    float out_max;
    float integral;
    float last_error;
    bool first_run;
} imu_yaw_pid_t;

typedef struct
{
    float estimate;
    float covariance;
    bool valid;
} imu_gyro_filter_t;

enum
{
    IMU_CMD_ENTER_SETUP,
    IMU_CMD_GYRO_CAL,
    IMU_CMD_SET_OUTPUT_485,
    IMU_CMD_SET_BAUD_921600,
    IMU_CMD_ENABLE_ACTIVE,
    IMU_CMD_OPEN_GYRO,
    IMU_CMD_OPEN_EULER,
    IMU_CMD_CLOSE_QUAT,
    IMU_CMD_EXIT_SETUP,
    IMU_CMD_COUNT
};

static const imu_command_t imu_commands[IMU_CMD_COUNT] = {
    [IMU_CMD_ENTER_SETUP] = {{0xAAU, 0x06U, 0x01U, 0x0DU}, 4U},
    [IMU_CMD_GYRO_CAL] = {{0xAAU, 0x03U, 0x02U, 0x0DU}, 4U},
    [IMU_CMD_SET_OUTPUT_485] = {{0xAAU, 0x0AU, 0x01U, 0x0DU}, 4U},
    [IMU_CMD_SET_BAUD_921600] = {{0xAAU, 0x0DU, 0x01U, 0x05U, 0x0DU}, 5U},
    [IMU_CMD_ENABLE_ACTIVE] = {{0xAAU, 0x01U, 0x13U, 0x0DU}, 4U},
    [IMU_CMD_OPEN_GYRO] = {{0xAAU, 0x01U, 0x15U, 0x0DU}, 4U},
    [IMU_CMD_OPEN_EULER] = {{0xAAU, 0x01U, 0x16U, 0x0DU}, 4U},
    [IMU_CMD_CLOSE_QUAT] = {{0xAAU, 0x01U, 0x07U, 0x0DU}, 4U},
    [IMU_CMD_EXIT_SETUP] = {{0xAAU, 0x06U, 0x00U, 0x0DU}, 4U}
};

static const uint8_t config_command_ids[] = {
    IMU_CMD_ENTER_SETUP,
    IMU_CMD_SET_OUTPUT_485,
    IMU_CMD_SET_BAUD_921600,
    IMU_CMD_ENABLE_ACTIVE,
    IMU_CMD_OPEN_GYRO,
    IMU_CMD_OPEN_EULER,
    IMU_CMD_CLOSE_QUAT,
    IMU_CMD_EXIT_SETUP
};

static imu_data_t imu_data;
static imu_init_context_t imu_init;
static float yaw_zero_ref_deg;
static bool yaw_zero_ref_valid;
static float yaw_kalman_estimate_deg;
static float yaw_kalman_p;
static bool yaw_kalman_valid;
static bool initialized;
static imu_yaw_pid_t yaw_pid[IMU_YAW_PID_COUNT] = {
    {
        .kp = 1.8f, .ki = 0.25f, .kd = 1.8f,
        .i_max = 8.0f, .out_max = 1000.0f, .first_run = true
    },
    {
        .kp = 1.0f, .ki = 0.18f, .kd = 1.8f,
        .i_max = 12.0f, .out_max = 250.0f, .first_run = true
    }
};
static imu_gyro_filter_t gyro_filter;
static uint32_t last_yaw_control_ms;
static uint32_t last_yaw_tx_ms;
static bool yaw_target_valid;

static bool time_reached(uint32_t now_ms, uint32_t target_ms)
{
    return (int32_t)(now_ms - target_ms) >= 0;
}

static float normalize_angle(float angle_deg)
{
    while (angle_deg >= IMU_ANGLE_HALF_RANGE_DEG)
    {
        angle_deg -= IMU_ANGLE_RANGE_DEG;
    }
    while (angle_deg < -IMU_ANGLE_HALF_RANGE_DEG)
    {
        angle_deg += IMU_ANGLE_RANGE_DEG;
    }

    return angle_deg;
}

static float limit_float(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static void reset_yaw_pid(imu_yaw_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->first_run = true;
}

static void reset_yaw_control(void)
{
    reset_yaw_pid(&yaw_pid[IMU_YAW_PID_MOVE]);
    reset_yaw_pid(&yaw_pid[IMU_YAW_PID_STOP]);
    memset(&gyro_filter, 0, sizeof(gyro_filter));
    yaw_target_valid = false;
    last_yaw_control_ms = 0U;
    imu_data.target_yaw_deg = 0.0f;
    imu_data.yaw_error_deg = 0.0f;
    imu_data.omega_output = 0;
    imu_data.yaw_hold_active = false;
}

static float filter_gyro(float gyro_deg_s)   //一维卡尔曼滤波(角速度)
{
    float gain;     //卡尔曼增益k

    if (!gyro_filter.valid)
    {
        gyro_filter.estimate = gyro_deg_s;   //角速度估计值
        gyro_filter.covariance = 1.0f;       //估计协方差p
        gyro_filter.valid = true;
    }
    else
    {
        gyro_filter.covariance += imu_config.gyro_filter_q; //Q为过程噪声
        gain = gyro_filter.covariance /
               (gyro_filter.covariance + imu_config.gyro_filter_r);  //r为测量噪声
        gyro_filter.estimate += gain *
                                (gyro_deg_s - gyro_filter.estimate);
        gyro_filter.covariance *= 1.0f - gain;
    }

    return roundf(gyro_filter.estimate * 10.0f) / 10.0f;
}

static float calculate_yaw_pid(imu_yaw_pid_t *pid, float error_deg)
{
    float error_delta;
    float output;

    if (fabsf(error_deg) <= imu_config.yaw_i_active_deg)
    {
        pid->integral += pid->ki * error_deg;
        pid->integral = limit_float(pid->integral,
                                    -pid->i_max, pid->i_max);
    }
    else
    {
        pid->integral *= imu_config.yaw_i_decay;   //积分软释放 防止积分饱和
    }

    if (pid->first_run)
    {
        pid->last_error = error_deg;
        pid->first_run = false;
    }

    error_delta = normalize_angle(error_deg - pid->last_error);
    output = pid->kp * error_deg + pid->integral +
             pid->kd * error_delta;
    pid->last_error = error_deg;
    return limit_float(output, -pid->out_max, pid->out_max);
}

static float filter_yaw(float measured_yaw_deg)
{
    float innovation_deg; //观测值与预测值的差值
    float kalman_gain;    //卡尔曼增益k

    // 标量卡尔曼参数，并对跨越正负 180 度的误差归一化
    if (!yaw_kalman_valid)
    {
        yaw_kalman_estimate_deg = normalize_angle(measured_yaw_deg);
        yaw_kalman_p = 1.0f;
        yaw_kalman_valid = true;
        return yaw_kalman_estimate_deg;
    }

    // 更新卡尔曼滤波器的协方差
    yaw_kalman_p += imu_config.kalman_q;
    innovation_deg = normalize_angle(measured_yaw_deg -
                                     yaw_kalman_estimate_deg);
    kalman_gain = yaw_kalman_p / (yaw_kalman_p + imu_config.kalman_r);
    yaw_kalman_estimate_deg = normalize_angle(
        yaw_kalman_estimate_deg + kalman_gain * innovation_deg);
    yaw_kalman_p = (1.0f - kalman_gain) * yaw_kalman_p;
    return yaw_kalman_estimate_deg;
}

static void reset_yaw(void)
{
    yaw_zero_ref_deg = 0.0f;
    yaw_zero_ref_valid = false;
    yaw_kalman_estimate_deg = 0.0f;
    yaw_kalman_p = 1.0f;
    yaw_kalman_valid = false;
    imu_data.yaw_deg = 0.0f;
    imu_data.yaw_valid = false;
    reset_yaw_control();
}

static void set_error_state(void)
{
    imu_init.step = IMU_INIT_ERROR;
    imu_data.state = IMU_STATE_ERROR;
    imu_data.gyro_valid = false;
    imu_data.yaw_valid = false;
    reset_yaw_control();
}

static void start_bias_sampling(const imu_raw_data_t *raw_data)
{
    imu_init.gyro_bias_sum_deg_s = 0.0f;
    imu_init.gyro_bias_sample_count = 0U;
    imu_init.last_gyro_sequence = raw_data->gyro_sequence;
    imu_init.last_yaw_sequence = raw_data->yaw_sequence;
    imu_data.gyro_bias_deg_s = 0.0f;
    imu_data.gyro_z_deg_s = 0.0f;
    imu_data.gyro_valid = false;
    reset_yaw();
    imu_init.step = IMU_INIT_SAMPLE_BIAS;
    imu_data.state = IMU_STATE_BIAS_SAMPLING;
}

static void run_init_state(uint32_t now_ms, const imu_raw_data_t *raw_data)
{
    const imu_command_t *command;

    if (!time_reached(now_ms, imu_init.next_action_ms))
    {
        return;
    }

    // 固定顺序：硬件校准 -> 配置主动输出 -> 软件零偏采样
    switch (imu_init.step)
    {
    case IMU_INIT_WAIT_BOOT:
        command = &imu_commands[IMU_CMD_ENTER_SETUP];
        if (Imu_Send(command->data, command->length) != HAL_OK)
        {
            set_error_state();
            break;
        }
        imu_init.step = IMU_INIT_SEND_CAL;
        imu_init.next_action_ms = now_ms + imu_config.cal_cmd_delay_ms;
        break;

    case IMU_INIT_SEND_CAL:
        command = &imu_commands[IMU_CMD_GYRO_CAL];
        if (Imu_Send(command->data, command->length) != HAL_OK)
        {
            set_error_state();
            break;
        }
        imu_init.step = IMU_INIT_WAIT_CAL;
        imu_init.next_action_ms = now_ms + imu_config.gyro_cal_wait_ms;
        break;

    case IMU_INIT_WAIT_CAL:
        imu_init.step = IMU_INIT_WAIT_CONFIG;
        imu_data.state = IMU_STATE_CONFIGURING;
        imu_init.next_action_ms = now_ms + imu_config.config_start_delay_ms;
        break;

    case IMU_INIT_WAIT_CONFIG:
        imu_init.config_index = 0U;
        imu_init.step = IMU_INIT_SEND_CONFIG;
        imu_init.next_action_ms = now_ms;
        break;

    case IMU_INIT_SEND_CONFIG:
        command = &imu_commands[config_command_ids[imu_init.config_index]];
        if (Imu_Send(command->data, command->length) != HAL_OK)
        {
            set_error_state();
            break;
        }
        imu_init.config_index++;
        if (imu_init.config_index >=
            (uint8_t)(sizeof(config_command_ids) /
                       sizeof(config_command_ids[0])))
        {
            start_bias_sampling(raw_data);
        }
        else
        {
            imu_init.next_action_ms = now_ms + imu_config.config_cmd_delay_ms;
        }
        break;

    case IMU_INIT_SAMPLE_BIAS:
    case IMU_INIT_COMPLETE:
    case IMU_INIT_ERROR:
    default:
        break;
    }
}

static void process_gyro(const imu_raw_data_t *raw_data)
{
    if (!raw_data->gyro_valid ||
        (raw_data->gyro_sequence == imu_init.last_gyro_sequence))
    {
        return;
    }
    imu_init.last_gyro_sequence = raw_data->gyro_sequence;

    if (imu_init.step == IMU_INIT_SAMPLE_BIAS)
    {
        // 延续旧工程做法，静止采集 200 帧 Z 轴角速度作为软件零偏
        imu_init.gyro_bias_sum_deg_s += raw_data->gyro_z_deg_s;
        imu_init.gyro_bias_sample_count++;
        if (imu_init.gyro_bias_sample_count >= imu_config.gyro_bias_samples)
        {
            imu_data.gyro_bias_deg_s = imu_init.gyro_bias_sum_deg_s /
                                       (float)imu_init.gyro_bias_sample_count;
            imu_data.gyro_z_deg_s = 0.0f;
            imu_data.gyro_valid = true;
            imu_init.last_yaw_sequence = raw_data->yaw_sequence;
            reset_yaw();
            imu_init.step = IMU_INIT_COMPLETE;
            imu_data.state = IMU_STATE_READY;
        }
        return;
    }

    if (imu_init.step == IMU_INIT_COMPLETE)
    {
        imu_data.gyro_z_deg_s = -(raw_data->gyro_z_deg_s -
                                  imu_data.gyro_bias_deg_s);
        imu_data.gyro_valid = true;
    }
}

static void process_yaw(const imu_raw_data_t *raw_data)
{
    float signed_yaw_deg;
    float zeroed_yaw_deg;

    if ((imu_init.step != IMU_INIT_COMPLETE) || !raw_data->yaw_valid ||
        (raw_data->yaw_sequence == imu_init.last_yaw_sequence))
    {
        return;
    }
    imu_init.last_yaw_sequence = raw_data->yaw_sequence;
    signed_yaw_deg = normalize_angle(-raw_data->yaw_deg);

    if (!yaw_zero_ref_valid)
    {
        yaw_zero_ref_deg = signed_yaw_deg;
        yaw_zero_ref_valid = true;
        imu_data.yaw_deg = 0.0f;
        imu_data.yaw_valid = true;
        return;
    }

    zeroed_yaw_deg = normalize_angle(signed_yaw_deg - yaw_zero_ref_deg);
    imu_data.yaw_deg = filter_yaw(zeroed_yaw_deg);
    imu_data.yaw_valid = true;
}

static void update_stats(uint32_t now_ms)
{
    imu_stats_t stats;

    if (!Imu_GetStats(&stats))
    {
        return;
    }

    imu_data.last_rx_ms = stats.last_valid_ms;
    imu_data.valid_frame_count = stats.valid_frame_count;
    imu_data.invalid_frame_count = stats.invalid_frame_count;
    imu_data.rx_overflow_count = stats.rx_overflow_count;
    imu_data.uart_error_count = stats.uart_error_count;
    imu_data.online = (stats.last_valid_ms != 0U) &&
                      ((now_ms - stats.last_valid_ms) <=
                       imu_config.online_timeout_ms);
}

/**
 * @brief 初始化 USART1 上的 DM-IMU L1 上层逻辑
 * @retval HAL 状态
 */
HAL_StatusTypeDef ImuMain_Init(void)
{
    if (initialized)
    {
        return HAL_OK;
    }

    memset(&imu_data, 0, sizeof(imu_data));
    if (Imu_Init(&huart1) != HAL_OK)
    {
        imu_data.state = IMU_STATE_ERROR;
        return HAL_ERROR;
    }

    memset(&imu_init, 0, sizeof(imu_init));
    imu_init.step = IMU_INIT_WAIT_BOOT;
    imu_data.state = IMU_STATE_CALIBRATING;
    imu_data.yaw_hold_enabled = true;
    imu_init.next_action_ms = HAL_GetTick() + imu_config.boot_delay_ms;
    initialized = true;
    return HAL_OK;
}

/**
 * @brief 每 1 ms 执行数据解析、初始化状态机和零偏修正
 * @retval None
 */
void ImuMain_Run1ms(void)
{
    imu_raw_data_t raw_data;
    uint32_t now_ms;

    if (!initialized)
    {
        return;
    }

    Imu_Process();
    if (!Imu_GetRawData(&raw_data))
    {
        set_error_state();
        return;
    }

    now_ms = HAL_GetTick();
    run_init_state(now_ms, &raw_data);
    process_gyro(&raw_data);
    process_yaw(&raw_data);
    update_stats(now_ms);
}

/**
 * @brief 将下一帧欧拉角设为新的偏航角零点
 * @retval HAL 状态
 */
HAL_StatusTypeDef ImuMain_ZeroYaw(void)
{
    if (!initialized || (imu_init.step != IMU_INIT_COMPLETE))
    {
        return HAL_ERROR;
    }

    reset_yaw();
    return HAL_OK;
}

/**
 * @brief 根据当前航向计算底盘旋转输出
 * @param vx X 方向速度指令
 * @param vy Y 方向速度指令
 * @param omega 手动旋转指令
 * @retval 修正后的底盘旋转指令
 */
int16_t ImuMain_CalcOmega(int16_t vx, int16_t vy, int16_t omega)
{
    imu_yaw_pid_t *active_pid;
    float filtered_gyro_deg_s;
    float output;
    uint32_t now_ms;
    bool stopped;

    if (!initialized || !imu_data.yaw_hold_enabled ||
        (imu_data.state != IMU_STATE_READY) || !imu_data.online ||
        !imu_data.yaw_valid || !imu_data.gyro_valid)
    {
        reset_yaw_control();
        imu_data.omega_output = omega;
        return omega;
    }

    now_ms = HAL_GetTick();
    if ((last_yaw_control_ms != 0U) &&
        ((now_ms - last_yaw_control_ms) < imu_config.yaw_control_period_ms))
    {
        return imu_data.omega_output;
    }
    last_yaw_control_ms = now_ms;
    filtered_gyro_deg_s = filter_gyro(imu_data.gyro_z_deg_s);

    // 第一次进入闭环时保持当前位置，避免使能瞬间突然旋转
    if (!yaw_target_valid)
    {
        imu_data.target_yaw_deg = imu_data.yaw_deg;
        yaw_target_valid = true;
    }

    // 手动旋转优先，旋转过程中持续记录当前航向，松手后原地保持
    if ((omega > imu_config.yaw_cmd_threshold) ||
        (omega < -imu_config.yaw_cmd_threshold))
    {
        imu_data.target_yaw_deg = imu_data.yaw_deg;
        imu_data.yaw_error_deg = 0.0f;
        imu_data.omega_output = omega;
        imu_data.yaw_hold_active = false;
        reset_yaw_pid(&yaw_pid[IMU_YAW_PID_MOVE]);
        reset_yaw_pid(&yaw_pid[IMU_YAW_PID_STOP]);
        return omega;
    }

    stopped = (abs((int)vx) <= imu_config.yaw_linear_threshold) &&
              (abs((int)vy) <= imu_config.yaw_linear_threshold);
    active_pid = &yaw_pid[stopped ? IMU_YAW_PID_STOP : IMU_YAW_PID_MOVE];
    if (stopped)
    {
        reset_yaw_pid(&yaw_pid[IMU_YAW_PID_MOVE]);
    }
    else
    {
        reset_yaw_pid(&yaw_pid[IMU_YAW_PID_STOP]);
    }

    imu_data.yaw_error_deg = normalize_angle(imu_data.target_yaw_deg -
                                             imu_data.yaw_deg);
    imu_data.yaw_hold_active = true;
    if (fabsf(imu_data.yaw_error_deg) <= imu_config.yaw_deadzone_deg)
    {
        reset_yaw_pid(active_pid);
        imu_data.omega_output = 0;
        return 0;
    }

    output = calculate_yaw_pid(active_pid, imu_data.yaw_error_deg) -
             filtered_gyro_deg_s * imu_config.yaw_gyro_k;
    output = limit_float(output, -active_pid->out_max,
                         active_pid->out_max);
    imu_data.omega_output = (int16_t)output;
    return imu_data.omega_output;
}

/**
 * @brief 设置航向保持目标角
 * @param target_yaw_deg 目标偏航角，单位为 deg
 * @retval HAL 状态
 */
HAL_StatusTypeDef ImuMain_SetTargetYaw(float target_yaw_deg)
{
    if (!initialized || isnan(target_yaw_deg) || isinf(target_yaw_deg))
    {
        return HAL_ERROR;
    }

    imu_data.target_yaw_deg = normalize_angle(target_yaw_deg);
    imu_data.yaw_error_deg = 0.0f;
    yaw_target_valid = true;
    reset_yaw_pid(&yaw_pid[IMU_YAW_PID_MOVE]);
    reset_yaw_pid(&yaw_pid[IMU_YAW_PID_STOP]);
    return HAL_OK;
}

/**
 * @brief 设置航向保持使能状态
 * @param enabled 是否使能
 * @retval None
 */
void ImuMain_EnableYawHold(bool enabled)
{
    if (!initialized)
    {
        return;
    }

    imu_data.yaw_hold_enabled = enabled;
    reset_yaw_control();
}

bool ImuMain_GetData(imu_data_t *data)
{
    if (!initialized || (data == NULL))
    {
        return false;
    }

    *data = imu_data;
    return true;
}

/**
 * @brief 通过上位机串口回传当前 yaw 角
 * @param uart 上位机串口句柄
 * @retval HAL 状态
 */
HAL_StatusTypeDef ImuMain_SendYaw(UART_HandleTypeDef *uart)
{
    uint8_t frame[IMU_YAW_FRAME_LENGTH];
    int16_t yaw_cdeg;
    float scaled_yaw;
    uint32_t now_ms;

    if ((uart == NULL) || !initialized || !imu_data.online ||
        !imu_data.yaw_valid)
    {
        return HAL_ERROR;
    }

    now_ms = HAL_GetTick();
    if ((now_ms - last_yaw_tx_ms) < imu_config.yaw_tx_period_ms)
    {
        return HAL_BUSY;
    }
    last_yaw_tx_ms = now_ms;

    scaled_yaw = imu_data.yaw_deg * imu_config.yaw_tx_scale;
    yaw_cdeg = (int16_t)(scaled_yaw +
                        ((scaled_yaw >= 0.0f) ? 0.5f : -0.5f));
    frame[0] = IMU_YAW_FRAME_HEADER_0;
    frame[1] = IMU_YAW_FRAME_HEADER_1;
    frame[2] = (uint8_t)((uint16_t)yaw_cdeg & 0xFFU);
    frame[3] = (uint8_t)(((uint16_t)yaw_cdeg >> 8) & 0xFFU);
    frame[4] = frame[2] ^ frame[3];

    return HAL_UART_Transmit(uart, frame, sizeof(frame), 1U);
}

void ImuMain_HandleRxEvent(UART_HandleTypeDef *uart, uint16_t size)
{
    Imu_HandleRxEvent(uart, size);
}

void ImuMain_HandleUartError(UART_HandleTypeDef *uart)
{
    Imu_HandleUartError(uart);
}
