/**
 * @file    imu_main.c
 * @brief   IMU 在 723 主板上的任务接入层实现
 * @note    按 723 工程风格(Init + Run1ms + 回调)，内部调用 user/imu 驱动与算法；
 *          板级 main/日志/串口回调(F405 my_main)已剥离。
 *          含航向保持(Yaw Hold) PID 控制器 ImuMain_CalcOmega(底盘每周期调用)，
 *          移植自 b-team 旧 imu_main，数据源改用 F405 IMU 的 Imu_GetYaw/Imu_GetGyroZ。
 */

#include "imu_main.h"
#include "imu_fusion.h"
#include "usart.h"          /* 引用 H7 既有串口句柄声明，仅使用、不修改 usart.c */

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* USART3(PD8/PD9) 由用户在 CubeMX 开启后，huart3 的定义在 usart.c 中生成。
 * 在那之前用 extern 前向声明保证可编译；与 usart.h 的声明兼容(可重复 extern)。 */
extern UART_HandleTypeDef huart3;

/* ==========================================================================
 *  航向保持(Yaw Hold) PID 子系统 —— 移植自 b-team 旧 imu_main
 *  参数沿用旧工程 5ms 航向环；底盘未实车调参前不要改调用周期。
 * ========================================================================== */
#define IMU_ANGLE_HALF_RANGE_DEG   180.0f
#define IMU_ANGLE_RANGE_DEG        360.0f

#define IMU_YAW_PERIOD_MS          5U     /* 航向环周期 ms */
#define IMU_YAW_CMD_THRESHOLD      5      /* 手动旋转指令阈值，超过则跟随手动 */
#define IMU_YAW_LINEAR_THRESHOLD   5      /* 判定静止的线速度阈值 */
#define IMU_YAW_DEADZONE_DEG       0.5f   /* 航向误差死区 */
#define IMU_YAW_I_ACTIVE_DEG       10.0f  /* 积分激活误差范围 */
#define IMU_YAW_I_DECAY            0.90f  /* 积分衰减 */
#define IMU_YAW_GYRO_K             5.0f   /* 陀螺阻尼增益 */
#define IMU_GYRO_FILTER_Q          0.1f
#define IMU_GYRO_FILTER_R          2.0f

#define IMU_YAW_MOVE_KP            1.8f
#define IMU_YAW_MOVE_KI            0.25f
#define IMU_YAW_MOVE_KD            1.8f
#define IMU_YAW_MOVE_I_MAX         8.0f
#define IMU_YAW_MOVE_OUT_MAX       1000.0f

#define IMU_YAW_STOP_KP            1.0f
#define IMU_YAW_STOP_KI            0.18f
#define IMU_YAW_STOP_KD            1.8f
#define IMU_YAW_STOP_I_MAX         12.0f
#define IMU_YAW_STOP_OUT_MAX       250.0f

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

static imu_yaw_pid_t yaw_move_pid = {
    .kp = IMU_YAW_MOVE_KP, .ki = IMU_YAW_MOVE_KI, .kd = IMU_YAW_MOVE_KD,
    .i_max = IMU_YAW_MOVE_I_MAX, .out_max = IMU_YAW_MOVE_OUT_MAX, .first_run = true
};
static imu_yaw_pid_t yaw_stop_pid = {
    .kp = IMU_YAW_STOP_KP, .ki = IMU_YAW_STOP_KI, .kd = IMU_YAW_STOP_KD,
    .i_max = IMU_YAW_STOP_I_MAX, .out_max = IMU_YAW_STOP_OUT_MAX, .first_run = true
};
static imu_gyro_filter_t gyro_filter;
static uint32_t last_yaw_control_ms;
static bool yaw_target_valid;
static float target_yaw_deg;
static float yaw_error_deg;
static int16_t omega_output;
static bool yaw_hold_active;

static float normalize_angle(float angle_deg)
{
    while (angle_deg >= IMU_ANGLE_HALF_RANGE_DEG) { angle_deg -= IMU_ANGLE_RANGE_DEG; }
    while (angle_deg < -IMU_ANGLE_HALF_RANGE_DEG) { angle_deg += IMU_ANGLE_RANGE_DEG; }
    return angle_deg;
}

static float limit_float(float value, float min_value, float max_value)
{
    if (value < min_value) { return min_value; }
    if (value > max_value) { return max_value; }
    return value;
}

static void reset_yaw_pid(imu_yaw_pid_t *pid)
{
    if (pid == NULL) { return; }
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->first_run = true;
}

static void reset_yaw_control(void)
{
    reset_yaw_pid(&yaw_move_pid);
    reset_yaw_pid(&yaw_stop_pid);
    memset(&gyro_filter, 0, sizeof(gyro_filter));
    yaw_target_valid = false;
    last_yaw_control_ms = 0U;
    target_yaw_deg = 0.0f;
    yaw_error_deg = 0.0f;
    omega_output = 0;
    yaw_hold_active = false;
}

static float filter_gyro(float gyro_deg_s)
{
    float gain;

    if (!gyro_filter.valid)
    {
        gyro_filter.estimate = gyro_deg_s;
        gyro_filter.covariance = 1.0f;
        gyro_filter.valid = true;
    }
    else
    {
        gyro_filter.covariance += IMU_GYRO_FILTER_Q;
        gain = gyro_filter.covariance / (gyro_filter.covariance + IMU_GYRO_FILTER_R);
        gyro_filter.estimate += gain * (gyro_deg_s - gyro_filter.estimate);
        gyro_filter.covariance *= 1.0f - gain;
    }
    return roundf(gyro_filter.estimate * 10.0f) / 10.0f;
}

static float calculate_yaw_pid(imu_yaw_pid_t *pid, float error_deg)
{
    float error_delta;
    float output;

    if (fabsf(error_deg) <= IMU_YAW_I_ACTIVE_DEG)
    {
        pid->integral += pid->ki * error_deg;
        pid->integral = limit_float(pid->integral, -pid->i_max, pid->i_max);
    }
    else
    {
        pid->integral *= IMU_YAW_I_DECAY;
    }

    if (pid->first_run)
    {
        pid->last_error = error_deg;
        pid->first_run = false;
    }

    error_delta = normalize_angle(error_deg - pid->last_error);
    output = pid->kp * error_deg + pid->integral + pid->kd * error_delta;
    pid->last_error = error_deg;
    return limit_float(output, -pid->out_max, pid->out_max);
}

/* ==========================================================================
 *  任务接入层 API
 * ========================================================================== */
HAL_StatusTypeDef ImuMain_Init(void)
{
    Imu_AttachUart(&IMU_UART_HANDLE);   /* 绑定 IMU 串口(由 imu_main.h 的宏选择，不修改 usart.c) */
    Imu_Init();                         /* 算法初始化 + 启动 DMA 接收 + 零漂校准调度 */
    reset_yaw_control();
    ImuFusion_Reset();                  /* 编码器-IMU 融合器复位 */
    return HAL_OK;
}

void ImuMain_Run1ms(void)
{
    Imu_Update();                       /* 驱动调度器：IMU 指令序列 / 延时回调 */
    ImuFusion_Update(HAL_GetTick());    /* 编码器(VESC)与 IMU 速度融合 + 延迟自适应权重 */
}

int16_t ImuMain_CalcOmega(int16_t vx, int16_t vy, int16_t omega)
{
    imu_data_t d;
    imu_yaw_pid_t *active_pid;
    float filtered_gyro_deg_s;
    float yaw_deg;
    float output;
    uint32_t now_ms;
    bool stopped;

    /* IMU 未标定完成(陀螺零漂未就绪)时直通，与旧逻辑一致 */
    Imu_GetData(&d);
    if (!d.gyro_bias_ready)
    {
        reset_yaw_control();
        omega_output = omega;
        return omega;
    }

    now_ms = HAL_GetTick();
    if ((last_yaw_control_ms != 0U) && ((now_ms - last_yaw_control_ms) < IMU_YAW_PERIOD_MS))
    {
        return omega_output;
    }
    last_yaw_control_ms = now_ms;

    yaw_deg = Imu_GetYaw();
    filtered_gyro_deg_s = filter_gyro(Imu_GetGyroZ());

    /* 第一次进入闭环时保持当前航向，避免使能瞬间突然旋转 */
    if (!yaw_target_valid)
    {
        target_yaw_deg = yaw_deg;
        yaw_target_valid = true;
    }

    /* 手动旋转优先，旋转中持续记录当前航向，松手后原地保持 */
    if ((omega > IMU_YAW_CMD_THRESHOLD) || (omega < -IMU_YAW_CMD_THRESHOLD))
    {
        target_yaw_deg = yaw_deg;
        yaw_error_deg = 0.0f;
        omega_output = omega;
        yaw_hold_active = false;
        reset_yaw_pid(&yaw_move_pid);
        reset_yaw_pid(&yaw_stop_pid);
        return omega;
    }

    stopped = (abs((int)vx) <= IMU_YAW_LINEAR_THRESHOLD) &&
              (abs((int)vy) <= IMU_YAW_LINEAR_THRESHOLD);
    active_pid = stopped ? &yaw_stop_pid : &yaw_move_pid;
    if (stopped) { reset_yaw_pid(&yaw_move_pid); }
    else         { reset_yaw_pid(&yaw_stop_pid); }

    yaw_error_deg = normalize_angle(target_yaw_deg - yaw_deg);
    yaw_hold_active = true;
    if (fabsf(yaw_error_deg) <= IMU_YAW_DEADZONE_DEG)
    {
        reset_yaw_pid(active_pid);
        omega_output = 0;
        return 0;
    }

    output = calculate_yaw_pid(active_pid, yaw_error_deg) - filtered_gyro_deg_s * IMU_YAW_GYRO_K;
    output = limit_float(output, -active_pid->out_max, active_pid->out_max);
    omega_output = (int16_t)output;
    return omega_output;
}

void ImuMain_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == &IMU_UART_HANDLE)
    {
        Imu_ProcessRxData(Imu_GetRxBuffer(), size);
        Imu_StartReceive();
    }
}

void ImuMain_HandleUartError(UART_HandleTypeDef *huart)
{
    if (huart == &IMU_UART_HANDLE)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        Imu_StartReceive();
    }
}
