/**
 * @file    imu.c
 * @brief   IMU 传感器串口 6 DMA 接收驱动与底层控制实现
 * @note    纯硬件驱动与帧协议解析层，算法已抽离至独立的 imu_algo.c / imu_algo.h 模块
 */

#include "imu.h"
#include "imu_algo.h"
#include "my_main.h"
#include "usart.h"
#include <math.h>

#define IMU_BOOT_DELAY_MS           100U
#define IMU_GYRO_CAL_WAIT_MS        4000U
#define IMU_GYRO_BIAS_SAMPLE_TARGET 200U

#define IMU_MAX_DELAYED_CALLBACKS   4U
#define IMU_SEQ_BUF_SIZE            16U

#define IMU_MAX_GYRO_VALUE_DEG_S    2000.0f
#define IMU_MAX_YAW_VALUE_DEG       360.0f
#define NORMALIZE_MAX_DEG           180.0f
#define NORMALIZE_STEP_DEG          360.0f

/**
 * @brief  IMU 指令序列描述结构体
 */
typedef struct
{
    const uint8_t *cmd;         // 命令内容常量指针
    uint16_t len;               // 命令长度
    uint32_t delay_ms;          // 执行后的等待延时 (ms)
} imu_cmd_t;

/**
 * @brief  延时回调描述结构体
 */
typedef struct
{
    uint32_t execute_at;        // 执行的目标 Tick
    void (*cb)(void);           // 回调函数指针
    uint8_t active;             // 激活状态
} delayed_callback_t;

static uint8_t imu_rx_dma_buffer[IMU_RX_DMA_BUFFER_SIZE];
static uint16_t imu_gyro_bias_sample_count = 0U;
static float imu_gyro_bias_sum_deg_s = 0.0f;
static float imu_gyro_bias_deg_s = 0.0f;
static uint8_t imu_gyro_bias_ready = 0U;
static float imu_yaw_zero_ref_deg = 0.0f;
static uint8_t imu_yaw_zero_ref_valid = 0U;

static imu_algo_t imu_algo;

static imu_cmd_t imu_seq_buf[IMU_SEQ_BUF_SIZE];
static uint8_t imu_seq_len = 0U;
static uint8_t imu_seq_idx = 0U;
static uint32_t imu_seq_next_time = 0U;
static void (*imu_seq_done_cb)(void) = 0;

static delayed_callback_t delayed_callbacks[IMU_MAX_DELAYED_CALLBACKS];

/**
 * @brief  规范化角度到 [-180.0, 180.0] 范围内
 * @param  angle_deg 待转换的角度值(度)
 * @retval 规范化后的角度值(度)
 */
static float Imu_NormalizeAngleDeg(float angle_deg)
{
    while (angle_deg > NORMALIZE_MAX_DEG)
    {
        angle_deg -= NORMALIZE_STEP_DEG;
    }

    while (angle_deg <= -NORMALIZE_MAX_DEG)
    {
        angle_deg += NORMALIZE_STEP_DEG;
    }

    return angle_deg;
}

/**
 * @brief  向序列调度队列中压入新序列
 * @param  seq     序列结构体数组
 * @param  len     序列条目数
 * @param  done_cb 序列完成后的回调
 * @retval None
 */
static void Imu_SchedulerEnqueueSequence(const imu_cmd_t *seq, uint8_t len, void (*done_cb)(void))
{
    uint8_t i;

    if ((seq == 0) || (len == 0U) || (len > IMU_SEQ_BUF_SIZE))
    {
        return;
    }

    for (i = 0U; i < len; i++)
    {
        imu_seq_buf[i] = seq[i];
    }

    imu_seq_len = len;
    imu_seq_idx = 0U;
    imu_seq_done_cb = done_cb;
    imu_seq_next_time = HAL_GetTick();
}

/**
 * @brief  注册一个延迟毫秒触发的回调任务
 * @param  delay_ms 延迟毫秒数
 * @param  cb       执行的回调函数
 * @retval None
 */
static void Imu_ScheduleDelayedCallback(uint32_t delay_ms, void (*cb)(void))
{
    uint8_t i;
    uint32_t when;

    if (cb == 0)
    {
        return;
    }

    when = HAL_GetTick() + delay_ms;

    for (i = 0U; i < IMU_MAX_DELAYED_CALLBACKS; i++)
    {
        if (delayed_callbacks[i].active == 0U)
        {
            delayed_callbacks[i].execute_at = when;
            delayed_callbacks[i].cb = cb;
            delayed_callbacks[i].active = 1U;
            return;
        }
    }
}

/**
 * @brief  向 IMU 发送控制或设置命令字
 * @param  cmd 命令字节指针
 * @param  len 命令长度
 * @retval None
 */
static void Imu_SendCommand(const uint8_t *cmd, uint16_t len)
{
    if ((cmd == 0) || (len == 0U))
    {
        return;
    }

    HAL_UART_Transmit(&huart6, (uint8_t *)cmd, len, 50U);
}

/**
 * @brief  复位偏航角校正状态
 * @retval None
 */
static void Imu_ResetYawCorrectionState(void)
{
    imu_yaw_zero_ref_deg = 0.0f;
    imu_yaw_zero_ref_valid = 0U;
    imu_algo.x_yaw = 0.0f;
    imu_algo.kalman_yaw_inited = 0U;
}

/**
 * @brief  首帧相对零位计算与对齐
 * @param  raw_yaw_deg 传感器回传的绝对偏航角 (deg)
 * @retval 减去开机起始朝向后的相对偏航角 (deg)
 */
static float Imu_CorrectYawDeg(float raw_yaw_deg)
{
    float signed_raw_yaw_deg = Imu_NormalizeAngleDeg(raw_yaw_deg);
    float output_yaw_deg;

    if (imu_yaw_zero_ref_valid == 0U)
    {
        imu_yaw_zero_ref_deg = signed_raw_yaw_deg;
        imu_yaw_zero_ref_valid = 1U;
    }

    output_yaw_deg = Imu_NormalizeAngleDeg(signed_raw_yaw_deg - imu_yaw_zero_ref_deg);

    return output_yaw_deg;
}

/**
 * @brief  配置 IMU 为 485 主动输出高波特率模式序列
 * @retval None
 */
static void Imu_Configure485ActiveOutput(void)
{
    static const uint8_t cmd_enter_setup[] = {0xAA, 0x06, 0x01, 0x0D};
    static const uint8_t cmd_set_output_485[] = {0xAA, 0x0A, 0x01, 0x0D};
    static const uint8_t cmd_set_485_baud_921600[] = {0xAA, 0x0D, 0x01, 0x05, 0x0D};
    static const uint8_t cmd_enable_485_active[] = {0xAA, 0x01, 0x13, 0x0D};
    static const uint8_t cmd_open_gyro[] = {0xAA, 0x01, 0x15, 0x0D};
    static const uint8_t cmd_open_euler[] = {0xAA, 0x01, 0x16, 0x0D};
    static const uint8_t cmd_close_quat[] = {0xAA, 0x01, 0x07, 0x0D};
    static const uint8_t cmd_exit_setup[] = {0xAA, 0x06, 0x00, 0x0D};

    imu_cmd_t seq[8];
    uint8_t idx = 0U;

    seq[idx].cmd = cmd_enter_setup;
    seq[idx].len = sizeof(cmd_enter_setup);
    seq[idx].delay_ms = 50U;
    idx++;

    seq[idx].cmd = cmd_set_output_485;
    seq[idx].len = sizeof(cmd_set_output_485);
    seq[idx].delay_ms = 20U;
    idx++;

    seq[idx].cmd = cmd_set_485_baud_921600;
    seq[idx].len = sizeof(cmd_set_485_baud_921600);
    seq[idx].delay_ms = 20U;
    idx++;

    seq[idx].cmd = cmd_enable_485_active;
    seq[idx].len = sizeof(cmd_enable_485_active);
    seq[idx].delay_ms = 20U;
    idx++;

    seq[idx].cmd = cmd_open_gyro;
    seq[idx].len = sizeof(cmd_open_gyro);
    seq[idx].delay_ms = 20U;
    idx++;

    seq[idx].cmd = cmd_open_euler;
    seq[idx].len = sizeof(cmd_open_euler);
    seq[idx].delay_ms = 20U;
    idx++;

    seq[idx].cmd = cmd_close_quat;
    seq[idx].len = sizeof(cmd_close_quat);
    seq[idx].delay_ms = 20U;
    idx++;

    seq[idx].cmd = cmd_exit_setup;
    seq[idx].len = sizeof(cmd_exit_setup);
    seq[idx].delay_ms = 20U;
    idx++;

    Imu_SchedulerEnqueueSequence(seq, idx, 0);
    LOG_INFO("IMU configured 485 active output mode");
}

/**
 * @brief  陀螺仪零漂校准完成后的回调
 * @retval None
 */
static void Imu_GyroCalDone(void)
{
    Imu_ResetYawCorrectionState();
    Imu_StartReceive();
    Imu_ScheduleDelayedCallback(IMU_BOOT_DELAY_MS, Imu_Configure485ActiveOutput);
    LOG_INFO("IMU gyro bias calibration done: bias=%.2f deg/s", imu_gyro_bias_deg_s);
}

/**
 * @brief  单帧数据解析处理（接入二状态 Yaw-Bias 卡尔曼、在线零偏及自适应滤波）
 * @param  frame 帧内容数组指针
 * @param  len   帧长度
 * @retval None
 */
static void Imu_ProcessFrame(const uint8_t *frame, uint8_t len)
{
    union
    {
        uint8_t bytes[4];
        float value;
    } frame_data;

    uint8_t frame_type;

    if ((frame == 0) || (len < 19U) || (frame[len - 1U] != 0x0AU))
    {
        return;
    }

    frame_type = frame[3];
    frame_data.bytes[0] = frame[12];
    frame_data.bytes[1] = frame[13];
    frame_data.bytes[2] = frame[14];
    frame_data.bytes[3] = frame[15];

    if (frame_type == 0x01U)
    {
        float acc_filt_x, acc_filt_y;
        float dt;
        float yaw_deg;

        /* 第 4 层：异常线性加速度检测保护 */
        if (ImuAlgo_CheckAccValid(&imu_algo, frame_data.value, 0.0f) == 0U)
        {
            return;
        }

        /* 第 4 层：二阶 Butterworth 低通滤波 (强力抑制电机与车体机械高频噪声) */
        ImuAlgo_BiquadFilterAcc(&imu_algo, frame_data.value, 0.0f, &acc_filt_x, &acc_filt_y);

        /* 第 7 层 & 第 5/6/2 层：滑动窗口振动判断、ZUPT 零速驻停与加速度在线零漂估计 */
        ImuAlgo_UpdateZuptAndBias(&imu_algo, acc_filt_x, acc_filt_y);

        /* 第 3 层：姿态变换与重力补偿 (根据偏航角将机体坐标系加速度向世界坐标系正交投影) */
        yaw_deg = ImuAlgo_GetYaw(&imu_algo);
        ImuAlgo_RotateAndCompensateAcc(&imu_algo, acc_filt_x, acc_filt_y, yaw_deg);

        /* 第 11 层：地面差速/直行机器人侧滑阻尼运动学模型约束 */
        ImuAlgo_ApplyNonHolonomicConstraint(&imu_algo, yaw_deg);

        /* 第 8 层 & 第 9 层：梯形二次双重积分与 3 状态 [pos, vel, a_bias] 卡尔曼协方差预测 */
        dt = ImuAlgo_GetDtSeconds(&imu_algo);
        ImuAlgo_PredictDoubleIntegral(&imu_algo, dt);

        /* 第 5 / 9 层：如果处于 ZUPT 零速度驻停，以 z=0 闭环卡尔曼修正位置趋势与零偏 */
        if (ImuAlgo_IsZuptActive(&imu_algo) != 0U)
        {
            ImuAlgo_UpdateZuptKalman(&imu_algo);
        }
    }
    else if (frame_type == 0x02U)
    {
        float gyro_clean;

        /* 异常角速度检验保护 */
        if (ImuAlgo_CheckGyroValid(&imu_algo, frame_data.value) == 0U)
        {
            return;
        }

        /* 原工程精髓：开机 4秒采集 200次求静态均值锁定基础 bias，永不被慢速自转误伤 */
        if (imu_gyro_bias_ready == 0U)
        {
            imu_gyro_bias_sum_deg_s += frame_data.value;
            imu_gyro_bias_sample_count++;

            if (imu_gyro_bias_sample_count >= IMU_GYRO_BIAS_SAMPLE_TARGET)
            {
                imu_gyro_bias_deg_s = imu_gyro_bias_sum_deg_s / (float)imu_gyro_bias_sample_count;
                imu_gyro_bias_ready = 1U;
                Imu_ResetYawCorrectionState();
            }
        }

        /* 扣去静止零偏后，叠加上我们系统的角速度噪声自适应一阶低通去噪 */
        if (imu_gyro_bias_ready != 0U)
        {
            gyro_clean = -(frame_data.value - imu_gyro_bias_deg_s);
        }
        else
        {
            gyro_clean = frame_data.value;
        }

        ImuAlgo_AdaptiveFilterGyro(&imu_algo, gyro_clean);
    }
    else if (frame_type == 0x03U)
    {
        float yaw_rel;

        if (isnan(frame_data.value) || isinf(frame_data.value) || (fabsf(frame_data.value) > IMU_MAX_YAW_VALUE_DEG))
        {
            return;
        }

        /* 原工程精髓：首帧相对起始方向归零处理 */
        yaw_rel = Imu_CorrectYawDeg(frame_data.value);

        /* 在官方官方解算角度的基础上，进行标量一阶卡尔曼去杂波平滑 */
        ImuAlgo_ApplyYawKalmanDeg(&imu_algo, yaw_rel);
    }
    else
    {
        /* 未关心的报文帧类型 */
    }
}

/**
 * @brief  初始化 IMU 模块与算法，并启动 DMA 接收
 * @retval None
 */
void Imu_Init(void)
{
    uint8_t i;

    imu_gyro_bias_sample_count = 0U;
    imu_gyro_bias_sum_deg_s = 0.0f;
    imu_gyro_bias_deg_s = 0.0f;
    imu_gyro_bias_ready = 0U;
    imu_yaw_zero_ref_deg = 0.0f;
    imu_yaw_zero_ref_valid = 0U;

    for (i = 0U; i < IMU_MAX_DELAYED_CALLBACKS; i++)
    {
        delayed_callbacks[i].active = 0U;
    }

    ImuAlgo_Init(&imu_algo);
    Imu_ResetYawCorrectionState();
    Imu_StartReceive();
    Imu_ScheduleDelayedCallback(IMU_BOOT_DELAY_MS, Imu_CalibrateGyro);

    LOG_INFO("IMU driver and 8-core algorithm initialized");
}

/**
 * @brief  主循环中持续调用的 IMU 任务更新函数（执行指令序列调度等）
 * @retval None
 */
void Imu_Update(void)
{
    uint8_t i;
    uint32_t now = HAL_GetTick();

    if ((imu_seq_len > 0U) && ((int32_t)(now - imu_seq_next_time) >= 0))
    {
        Imu_SendCommand(imu_seq_buf[imu_seq_idx].cmd, imu_seq_buf[imu_seq_idx].len);
        imu_seq_next_time = now + imu_seq_buf[imu_seq_idx].delay_ms;
        imu_seq_idx++;

        if (imu_seq_idx >= imu_seq_len)
        {
            void (*done_cb)(void) = imu_seq_done_cb;

            imu_seq_len = 0U;
            imu_seq_idx = 0U;
            imu_seq_done_cb = 0;

            if (done_cb != 0)
            {
                done_cb();
            }
        }
    }

    for (i = 0U; i < IMU_MAX_DELAYED_CALLBACKS; i++)
    {
        if ((delayed_callbacks[i].active != 0U) && ((int32_t)(now - delayed_callbacks[i].execute_at) >= 0))
        {
            void (*cb)(void) = delayed_callbacks[i].cb;

            delayed_callbacks[i].active = 0U;

            if (cb != 0)
            {
                cb();
            }
        }
    }
}

/**
 * @brief  开启串口 6 DMA 接收监听
 * @retval None
 */
void Imu_StartReceive(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart6, imu_rx_dma_buffer, sizeof(imu_rx_dma_buffer)) == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
    }
}

/**
 * @brief  处理通过串口 6 接收到的原始数据帧
 * @param  data 接收缓冲区首地址
 * @param  len  当前收到的字节长度
 * @retval None
 */
void Imu_ProcessRxData(const uint8_t *data, uint16_t len)
{
    static uint8_t frame_buf[24];
    static uint8_t frame_idx = 0U;
    static uint8_t expected_len = 0U;
    uint16_t i;

    if ((data == 0) || (len == 0U))
    {
        return;
    }

    for (i = 0U; i < len; i++)
    {
        uint8_t byte = data[i];

        if (frame_idx == 0U)
        {
            if (byte == 0x55U)
            {
                frame_buf[0] = byte;
                frame_idx = 1U;
            }
            continue;
        }

        if (frame_idx == 1U)
        {
            if (byte == 0xAAU)
            {
                frame_buf[1] = byte;
                frame_idx = 2U;
            }
            else if (byte == 0x55U)
            {
                frame_buf[0] = byte;
                frame_idx = 1U;
            }
            else
            {
                frame_idx = 0U;
            }
            continue;
        }

        if (frame_idx >= sizeof(frame_buf))
        {
            frame_idx = 0U;
            expected_len = 0U;
            continue;
        }

        frame_buf[frame_idx] = byte;

        if (frame_idx == 2U)
        {
            frame_idx = 3U;
            continue;
        }

        if (frame_idx == 3U)
        {
            if ((byte == 0x01U) || (byte == 0x02U) || (byte == 0x03U))
            {
                expected_len = 19U;
                frame_idx = 4U;
            }
            else if (byte == 0x04U)
            {
                expected_len = 23U;
                frame_idx = 4U;
            }
            else
            {
                frame_idx = 0U;
                expected_len = 0U;
            }
            continue;
        }

        frame_idx++;
        if ((expected_len > 0U) && (frame_idx == expected_len))
        {
            Imu_ProcessFrame(frame_buf, expected_len);
            frame_idx = 0U;
            expected_len = 0U;
        }
    }
}

/**
 * @brief  获取修正滤波后的偏航角 (Yaw)
 * @retval 偏航角角度值(度)
 */
float Imu_GetYaw(void)
{
    return ImuAlgo_GetYaw(&imu_algo);
}

/**
 * @brief  获取校准零漂后的 Z 轴角速度 (Gyro Z)
 * @retval Z轴角速度(度/秒)
 */
float Imu_GetGyroZ(void)
{
    return imu_algo.gyro_filtered;
}

/**
 * @brief  获取实时在线跟踪评估的最优角速度零偏 (Bias)
 * @retval 零漂值(度/秒)
 */
float Imu_GetBias(void)
{
    return imu_gyro_bias_deg_s;
}

/**
 * @brief  获取姿态变换去偏后的世界坐标系 X 轴线性加速度
 * @retval 加速度值(m/s²)
 */
float Imu_GetAccWorldX(void)
{
    return ImuAlgo_GetAccWorldX(&imu_algo);
}

/**
 * @brief  获取姿态变换去偏后的世界坐标系 Y 轴线性加速度
 * @retval 加速度值(m/s²)
 */
float Imu_GetAccWorldY(void)
{
    return ImuAlgo_GetAccWorldY(&imu_algo);
}

/**
 * @brief  获取梯形积分与 ZUPT 修正后的世界坐标系 X 轴速度
 * @retval 速度值(m/s)
 */
float Imu_GetVelX(void)
{
    return ImuAlgo_GetVelX(&imu_algo);
}

/**
 * @brief  获取梯形积分与 ZUPT 修正后的世界坐标系 Y 轴速度
 * @retval 速度值(m/s)
 */
float Imu_GetVelY(void)
{
    return ImuAlgo_GetVelY(&imu_algo);
}

/**
 * @brief  获取梯形二次双重积分得到的世界坐标系 X 轴累计位置 (Position X)
 * @retval 位置值(m)
 */
float Imu_GetPosX(void)
{
    return ImuAlgo_GetPosX(&imu_algo);
}

/**
 * @brief  获取梯形二次双重积分得到的世界坐标系 Y 轴累计位置 (Position Y)
 * @retval 位置值(m)
 */
float Imu_GetPosY(void)
{
    return ImuAlgo_GetPosY(&imu_algo);
}

/**
 * @brief  获取实时在线跟踪评估的 X 轴加速度零漂
 * @retval 零漂值(m/s²)
 */
float Imu_GetAccBiasX(void)
{
    return ImuAlgo_GetAccBiasX(&imu_algo);
}

/**
 * @brief  获取实时在线跟踪评估的 Y 轴加速度零漂
 * @retval 零漂值(m/s²)
 */
float Imu_GetAccBiasY(void)
{
    return ImuAlgo_GetAccBiasY(&imu_algo);
}

/**
 * @brief  获取当前是否处于 ZUPT 零速度驻停模式
 * @retval 1:触发ZUPT, 0:运动
 */
uint8_t Imu_IsZuptActive(void)
{
    return ImuAlgo_IsZuptActive(&imu_algo);
}

/**
 * @brief  一键清空复位底盘二次积分积累的位置与速度坐标 (PosX/PosY/VelX/VelY 清零)
 * @retval None
 */
void Imu_ResetPosition(void)
{
    ImuAlgo_ResetPosition(&imu_algo);
}

/**
 * @brief  获取当前是否判定为车体静止
 * @retval 1:静止, 0:运动中
 */
uint8_t Imu_IsStationary(void)
{
    return ImuAlgo_IsZuptActive(&imu_algo);
}

/**
 * @brief  获取完整的 IMU 输出状态数据
 * @param  data 输出保存结构体指针
 * @retval None
 */
void Imu_GetData(imu_data_t *data)
{
    if (data == 0)
    {
        return;
    }

    data->yaw_deg = Imu_GetYaw();
    data->gyro_z_deg_s = Imu_GetGyroZ();
    data->gyro_bias_deg_s = Imu_GetBias();
    data->acc_world_x_mps2 = Imu_GetAccWorldX();
    data->acc_world_y_mps2 = Imu_GetAccWorldY();
    data->vel_world_x_mps = Imu_GetVelX();
    data->vel_world_y_mps = Imu_GetVelY();
    data->pos_world_x_m = Imu_GetPosX();
    data->pos_world_y_m = Imu_GetPosY();
    data->acc_bias_x_mps2 = Imu_GetAccBiasX();
    data->acc_bias_y_mps2 = Imu_GetAccBiasY();
    data->gyro_bias_ready = imu_gyro_bias_ready;
    data->is_connected = 1U;
    data->is_stationary = Imu_IsStationary();
    data->zupt_active = Imu_IsZuptActive();
}

/**
 * @brief  获取内部 DMA 接收缓冲区首地址
 * @retval 缓冲区首地址
 */
uint8_t *Imu_GetRxBuffer(void)
{
    return imu_rx_dma_buffer;
}

/**
 * @brief  启动陀螺仪零漂校准序列
 * @retval None
 */
void Imu_CalibrateGyro(void)
{
    static const uint8_t cmd_enter_setup[] = {0xAA, 0x06, 0x01, 0x0D};
    static const uint8_t cmd_gyro_cal[] = {0xAA, 0x03, 0x02, 0x0D};
    imu_cmd_t seq[2];

    seq[0].cmd = cmd_enter_setup;
    seq[0].len = sizeof(cmd_enter_setup);
    seq[0].delay_ms = 50U;

    seq[1].cmd = cmd_gyro_cal;
    seq[1].len = sizeof(cmd_gyro_cal);
    seq[1].delay_ms = IMU_GYRO_CAL_WAIT_MS;

    Imu_SchedulerEnqueueSequence(seq, 2U, Imu_GyroCalDone);
    LOG_INFO("IMU started gyro bias calibration sequence");
}
