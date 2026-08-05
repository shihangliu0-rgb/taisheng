/**
 * @file    imu_algo.c
 * @brief   IMU 混合体系算法实现：原工程官方姿态直读平滑 + 11层惯导加速度/速度/位置积分定位
 */

#include "imu_algo.h"
#include "imu.h"

#define DT_MIN_SECONDS          0.00001f
#define DT_MAX_SECONDS          0.02f
#define DT_DEFAULT_SECONDS      0.005f

#define NORMALIZE_MAX_DEG       180.0f
#define NORMALIZE_STEP_DEG      360.0f
#define MATH_DEG_TO_RAD         0.0174532925f

/**
 * @brief  内部静态工具：规范化角度到 [-180.0, 180.0] 范围内
 * @param  angle_deg 输入待规范角度 (deg)
 * @retval 规范化后的角度 (deg)
 */
static float ImuAlgo_NormalizeAngleDeg(float angle_deg)
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
 * @brief  初始化算法模块各个参数与结构体
 * @param  algo 算法实例指针
 * @retval None
 */
void ImuAlgo_Init(imu_algo_t *algo)
{
    uint16_t i;
    uint8_t r, c;

    if (algo == 0)
    {
        return;
    }

    /* 原工程官方偏航角直读与标量卡尔曼平滑 */
    algo->x_yaw = 0.0f;
    algo->p_yaw = 1.0f;
    algo->kalman_yaw_inited = 0U;

    algo->gyro_filtered = 0.0f;
    algo->gyro_last_raw = 0.0f;
    algo->has_last_raw = 0U;

    /* 加速度二阶 Butterworth 低通滤波器状态清零 */
    algo->lpf_acc_x.x1 = 0.0f;
    algo->lpf_acc_x.x2 = 0.0f;
    algo->lpf_acc_x.y1 = 0.0f;
    algo->lpf_acc_x.y2 = 0.0f;

    algo->lpf_acc_y.x1 = 0.0f;
    algo->lpf_acc_y.x2 = 0.0f;
    algo->lpf_acc_y.y1 = 0.0f;
    algo->lpf_acc_y.y2 = 0.0f;

    algo->acc_x_filt = 0.0f;
    algo->acc_y_filt = 0.0f;

    algo->acc_bias_x = 0.0f;
    algo->acc_bias_y = 0.0f;

    algo->acc_world_x = 0.0f;
    algo->acc_world_y = 0.0f;
    algo->acc_world_last_x = 0.0f;
    algo->acc_world_last_y = 0.0f;
    algo->has_last_acc_world = 0U;

    algo->acc_last_raw_x = 0.0f;
    algo->acc_last_raw_y = 0.0f;
    algo->has_last_acc_raw = 0U;

    algo->vel_world_x = 0.0f;
    algo->vel_world_y = 0.0f;
    algo->vel_world_last_x = 0.0f;
    algo->vel_world_last_y = 0.0f;
    algo->has_last_vel_world = 0U;
    algo->zupt_active = 0U;

    algo->pos_world_x = 0.0f;
    algo->pos_world_y = 0.0f;

    for (r = 0U; r < 3U; r++)
    {
        for (c = 0U; c < 3U; c++)
        {
            if (r == c)
            {
                algo->p_acc_x[r][c] = 1.0f;
                algo->p_acc_y[r][c] = 1.0f;
            }
            else
            {
                algo->p_acc_x[r][c] = 0.0f;
                algo->p_acc_y[r][c] = 0.0f;
            }
        }
    }
    algo->pos_kalman_inited = 0U;

    algo->acc_history_idx = 0U;
    algo->acc_history_count = 0U;

    for (i = 0U; i < IMU_ALGO_HISTORY_SIZE; i++)
    {
        algo->acc_history_x[i] = 0.0f;
        algo->acc_history_y[i] = 0.0f;
    }

    /* 开启 Cortex-M4 DWT 周期计数器实现真正微秒级 dt 测量 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    algo->last_dwt_cycles = DWT->CYCCNT;
    algo->dwt_inited = 1U;
}

/**
 * @brief  异常角速度检验保护 (超过绝对上限或单帧阶跃过大视为错帧)
 * @param  algo     算法实例指针
 * @param  gyro_raw 当前读到的原始角速度 (deg/s)
 * @retval 1:有效正常数据, 0:异常错帧需丢弃
 */
uint8_t ImuAlgo_CheckGyroValid(imu_algo_t *algo, float gyro_raw)
{
    if (algo == 0)
    {
        return 0U;
    }

    if ((isnan(gyro_raw)) || (isinf(gyro_raw)) || (fabsf(gyro_raw) > IMU_ALGO_GYRO_ABS_LIMIT))
    {
        return 0U;
    }

    if (algo->has_last_raw != 0U)
    {
        if (fabsf(gyro_raw - algo->gyro_last_raw) > IMU_ALGO_GYRO_STEP_LIMIT)
        {
            return 0U;
        }
    }

    algo->gyro_last_raw = gyro_raw;
    algo->has_last_raw = 1U;

    return 1U;
}

/**
 * @brief  角速度噪声自适应一阶滤波 (静止强滤波 alpha=0.02, 运动弱滤波 alpha=0.3)
 * @param  algo     算法实例指针
 * @param  gyro_raw 当前有效角速度 (deg/s)
 * @retval 滤波计算后得到的平滑角速度 (deg/s)
 */
float ImuAlgo_AdaptiveFilterGyro(imu_algo_t *algo, float gyro_raw)
{
    float alpha;

    if (algo == 0)
    {
        return gyro_raw;
    }

    if (fabsf(gyro_raw) < IMU_ALGO_GYRO_MOTION_THRESHOLD)
    {
        alpha = IMU_ALGO_GYRO_ALPHA_STATIC;
    }
    else
    {
        alpha = IMU_ALGO_GYRO_ALPHA_MOTION;
    }

    algo->gyro_filtered += alpha * (gyro_raw - algo->gyro_filtered);
    return algo->gyro_filtered;
}

/**
 * @brief  获取基于 DWT 微秒级时间计数的帧时间差 dt
 * @param  algo 算法实例指针
 * @retval dt (秒)，有效范围 [0.00001, 0.02]，超时默认 0.005
 */
float ImuAlgo_GetDtSeconds(imu_algo_t *algo)
{
    uint32_t now_cycles;
    uint32_t delta_cycles;
    float dt;

    if (algo == 0)
    {
        return DT_DEFAULT_SECONDS;
    }

    now_cycles = DWT->CYCCNT;
    delta_cycles = now_cycles - algo->last_dwt_cycles;
    algo->last_dwt_cycles = now_cycles;

    dt = (float)delta_cycles / (float)SystemCoreClock;

    if ((dt < DT_MIN_SECONDS) || (dt > DT_MAX_SECONDS))
    {
        dt = DT_DEFAULT_SECONDS;
    }

    return dt;
}

/**
 * @brief  原工程路线：对官方传回的对准后 Yaw 角度执行标量一阶卡尔曼去杂波平滑
 * @param  algo     算法实例指针
 * @param  yaw_meas 官方 0x03 帧去零参考后的角度 (deg)
 * @retval 平滑去杂波后的最优 Yaw 角度 (deg)
 */
float ImuAlgo_ApplyYawKalmanDeg(imu_algo_t *algo, float yaw_meas)
{
    float innovation_deg;
    float kalman_gain;

    if (algo == 0)
    {
        return yaw_meas;
    }

    if (algo->kalman_yaw_inited == 0U)
    {
        algo->x_yaw = ImuAlgo_NormalizeAngleDeg(yaw_meas);
        algo->p_yaw = 1.0f;
        algo->kalman_yaw_inited = 1U;
        return algo->x_yaw;
    }

    algo->p_yaw += IMU_ALGO_Q_YAW;
    innovation_deg = ImuAlgo_NormalizeAngleDeg(yaw_meas - algo->x_yaw);
    kalman_gain = algo->p_yaw / (algo->p_yaw + IMU_ALGO_R_YAW);
    algo->x_yaw = ImuAlgo_NormalizeAngleDeg(algo->x_yaw + kalman_gain * innovation_deg);
    algo->p_yaw = (1.0f - kalman_gain) * algo->p_yaw;

    return algo->x_yaw;
}

/**
 * @brief  获取最终最优偏航角输出 (Yaw)
 * @param  algo 算法实例指针
 * @retval 最优偏航角 (-180.0 ~ 180.0 deg)
 */
float ImuAlgo_GetYaw(const imu_algo_t *algo)
{
    if (algo == 0)
    {
        return 0.0f;
    }
    return algo->x_yaw;
}

/* ========================================================================== */
/*           IMU 加速度处理链：二阶 Butterworth、ZUPT 与梯形双重积分            */
/* ========================================================================== */

/**
 * @brief  第 4 层：异常线性加速度检测保护 (判断采样是否超过最大额定量程)
 * @param  algo      算法实例指针
 * @param  acc_raw_x 采样 X 加速度 (m/s²)
 * @param  acc_raw_y 采样 Y 加速度 (m/s²)
 * @retval 1:有效数据, 0:错报尖峰包需抛弃
 */
uint8_t ImuAlgo_CheckAccValid(imu_algo_t *algo, float acc_raw_x, float acc_raw_y)
{
    if (algo == 0)
    {
        return 0U;
    }

    if ((isnan(acc_raw_x)) || (isinf(acc_raw_x)) || (fabsf(acc_raw_x) > IMU_ALGO_ACC_ABS_LIMIT) ||
        (isnan(acc_raw_y)) || (isinf(acc_raw_y)) || (fabsf(acc_raw_y) > IMU_ALGO_ACC_ABS_LIMIT))
    {
        return 0U;
    }

    algo->acc_last_raw_x = acc_raw_x;
    algo->acc_last_raw_y = acc_raw_y;
    algo->has_last_acc_raw = 1U;
    return 1U;
}

/**
 * @brief  第 4 层：二阶 Butterworth 低通滤波 (20~30Hz 截止频率，强力滤除电机机械振动)
 * @param  algo       算法实例指针
 * @param  acc_raw_x  原始有效 X 轴加速度
 * @param  acc_raw_y  原始有效 Y 轴加速度
 * @param  acc_filt_x 输出二阶滤波后的 X 轴加速度
 * @param  acc_filt_y 输出二阶滤波后的 Y 轴加速度
 * @retval None
 */
void ImuAlgo_BiquadFilterAcc(imu_algo_t *algo, float acc_raw_x, float acc_raw_y, float *acc_filt_x, float *acc_filt_y)
{
    float y_new_x;
    float y_new_y;

    if (algo == 0)
    {
        return;
    }

    /* X 轴二阶 Butterworth 差分计算：y = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2 */
    y_new_x = IMU_ALGO_ACC_LPF_B0 * acc_raw_x +
              IMU_ALGO_ACC_LPF_B1 * algo->lpf_acc_x.x1 +
              IMU_ALGO_ACC_LPF_B2 * algo->lpf_acc_x.x2 -
              IMU_ALGO_ACC_LPF_A1 * algo->lpf_acc_x.y1 -
              IMU_ALGO_ACC_LPF_A2 * algo->lpf_acc_x.y2;

    algo->lpf_acc_x.x2 = algo->lpf_acc_x.x1;
    algo->lpf_acc_x.x1 = acc_raw_x;
    algo->lpf_acc_x.y2 = algo->lpf_acc_x.y1;
    algo->lpf_acc_x.y1 = y_new_x;
    algo->acc_x_filt = y_new_x;

    /* Y 轴二阶 Butterworth 差分计算 */
    y_new_y = IMU_ALGO_ACC_LPF_B0 * acc_raw_y +
              IMU_ALGO_ACC_LPF_B1 * algo->lpf_acc_y.x1 +
              IMU_ALGO_ACC_LPF_B2 * algo->lpf_acc_y.x2 -
              IMU_ALGO_ACC_LPF_A1 * algo->lpf_acc_y.y1 -
              IMU_ALGO_ACC_LPF_A2 * algo->lpf_acc_y.y2;

    algo->lpf_acc_y.x2 = algo->lpf_acc_y.x1;
    algo->lpf_acc_y.x1 = acc_raw_y;
    algo->lpf_acc_y.y2 = algo->lpf_acc_y.y1;
    algo->lpf_acc_y.y1 = y_new_y;
    algo->acc_y_filt = y_new_y;

    if (acc_filt_x != 0)
    {
        *acc_filt_x = y_new_x;
    }
    if (acc_filt_y != 0)
    {
        *acc_filt_y = y_new_y;
    }
}

/**
 * @brief  第 7 层 & 第 5/2 层：滑动窗口振动判断、ZUPT 零速驻停与在线加速度零漂收敛
 * @param  algo       算法实例指针
 * @param  acc_filt_x 滤波后 X 轴加速度 (m/s²)
 * @param  acc_filt_y 滤波后 Y 轴加速度 (m/s²)
 * @retval None
 */
void ImuAlgo_UpdateZuptAndBias(imu_algo_t *algo, float acc_filt_x, float acc_filt_y)
{
    uint16_t i;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float mean_x, mean_y;
    float var_x = 0.0f;
    float var_y = 0.0f;
    float std_x, std_y;

    if (algo == 0)
    {
        return;
    }

    algo->acc_history_x[algo->acc_history_idx] = acc_filt_x;
    algo->acc_history_y[algo->acc_history_idx] = acc_filt_y;
    algo->acc_history_idx++;
    if (algo->acc_history_idx >= IMU_ALGO_HISTORY_SIZE)
    {
        algo->acc_history_idx = 0U;
    }

    if (algo->acc_history_count < IMU_ALGO_HISTORY_SIZE)
    {
        algo->acc_history_count++;
        return;
    }

    for (i = 0U; i < IMU_ALGO_HISTORY_SIZE; i++)
    {
        sum_x += algo->acc_history_x[i];
        sum_y += algo->acc_history_y[i];
    }
    mean_x = sum_x / (float)IMU_ALGO_HISTORY_SIZE;
    mean_y = sum_y / (float)IMU_ALGO_HISTORY_SIZE;

    for (i = 0U; i < IMU_ALGO_HISTORY_SIZE; i++)
    {
        float dx = algo->acc_history_x[i] - mean_x;
        float dy = algo->acc_history_y[i] - mean_y;
        var_x += dx * dx;
        var_y += dy * dy;
    }
    var_x /= (float)IMU_ALGO_HISTORY_SIZE;
    var_y /= (float)IMU_ALGO_HISTORY_SIZE;
    std_x = sqrtf(var_x);
    std_y = sqrtf(var_y);

    /* 只有当角速度判断为静止、且加速度滑动窗口标准差均合格，才触发 ZUPT */
    if ((fabsf(algo->gyro_filtered) < IMU_ALGO_ZUPT_GYRO_THRESHOLD) &&
        (std_x < IMU_ALGO_ZUPT_ACC_STD_THRESHOLD) &&
        (std_y < IMU_ALGO_ZUPT_ACC_STD_THRESHOLD))
    {
        algo->zupt_active = 1U;

        /* 第 2 层：静止时加速度纯来自零漂与温漂，慢慢跟踪收敛 a_bias */
        algo->acc_bias_x += IMU_ALGO_ACC_BIAS_ALPHA * (acc_filt_x - algo->acc_bias_x);
        algo->acc_bias_y += IMU_ALGO_ACC_BIAS_ALPHA * (acc_filt_y - algo->acc_bias_y);

        /* 第 5 层：ZUPT 零速度触发卡尔曼观测校正，同时把线速度归零根除累计漂移 */
        ImuAlgo_UpdateZuptKalman(algo);
        algo->vel_world_x = 0.0f;
        algo->vel_world_y = 0.0f;
    }
    else
    {
        algo->zupt_active = 0U;
    }
}

/**
 * @brief  第 3 层：姿态变换与重力补偿 —— 减去在线估算零飘，根据官方偏航角将机体加速度向世界坐标系正交投影
 * @param  algo       算法实例指针
 * @param  acc_filt_x 机体滤波 X 加速度
 * @param  acc_filt_y 机体滤波 Y 加速度
 * @param  yaw_deg    当前官方平滑偏航角 (deg)
 * @retval None
 */
void ImuAlgo_RotateAndCompensateAcc(imu_algo_t *algo, float acc_filt_x, float acc_filt_y, float yaw_deg)
{
    float a_body_x;
    float a_body_y;
    float yaw_rad;
    float cos_y, sin_y;

    if (algo == 0)
    {
        return;
    }

    /* 去掉在线自校准的加速度零漂 */
    a_body_x = acc_filt_x - algo->acc_bias_x;
    a_body_y = acc_filt_y - algo->acc_bias_y;

    /* 二维坐标旋转：abody -> aworld */
    yaw_rad = yaw_deg * MATH_DEG_TO_RAD;
    cos_y = cosf(yaw_rad);
    sin_y = sinf(yaw_rad);

    algo->acc_world_x = a_body_x * cos_y - a_body_y * sin_y;
    algo->acc_world_y = a_body_x * sin_y + a_body_y * cos_y;
}

/**
 * @brief  第 11 层：地面差速/直行机器人约束运动模型 (侧滑速度阻尼限幅)
 * @param  algo    算法实例指针
 * @param  yaw_deg 当前偏航角 (deg)
 * @retval None
 */
void ImuAlgo_ApplyNonHolonomicConstraint(imu_algo_t *algo, float yaw_deg)
{
#if (IMU_ALGO_NONHOLONOMIC_CONSTRAINT == 1U)
    float yaw_rad;
    float cos_y, sin_y;
    float v_body_x, v_body_y;

    if (algo == 0)
    {
        return;
    }

    yaw_rad = yaw_deg * MATH_DEG_TO_RAD;
    cos_y = cosf(yaw_rad);
    sin_y = sinf(yaw_rad);

    /* 世界速度转化至机体坐标系 */
    v_body_x = algo->vel_world_x * cos_y + algo->vel_world_y * sin_y;
    v_body_y = -algo->vel_world_x * sin_y + algo->vel_world_y * cos_y;

    /* 差速/直行轮在垂直轮轴的侧滑侧偏应受到极大阻尼衰减 */
    v_body_y *= 0.10f;

    /* 转回世界速度 */
    algo->vel_world_x = v_body_x * cos_y - v_body_y * sin_y;
    algo->vel_world_y = v_body_x * sin_y + v_body_y * cos_y;
#else
    (void)algo;
    (void)yaw_deg;
#endif
}

/**
 * @brief  第 8 层 & 第 9 层：梯形二次双重积分 (a -> v -> pos) + 三状态 [pos, vel, a_bias] 协方差预测
 * @param  algo 算法实例指针
 * @param  dt   当前时间差 (秒)
 * @retval None
 */
void ImuAlgo_PredictDoubleIntegral(imu_algo_t *algo, float dt)
{
    float a_last_x, a_last_y;
    float v_last_x, v_last_y;
    uint8_t r, c, k;
    float f_mat[3][3];
    float fp[3][3];
    float p_new_x[3][3];
    float p_new_y[3][3];

    if (algo == 0)
    {
        return;
    }

    if (algo->has_last_acc_world == 0U)
    {
        algo->acc_world_last_x = algo->acc_world_x;
        algo->acc_world_last_y = algo->acc_world_y;
        algo->has_last_acc_world = 1U;
    }
    a_last_x = algo->acc_world_last_x;
    a_last_y = algo->acc_world_last_y;

    if (algo->has_last_vel_world == 0U)
    {
        algo->vel_world_last_x = algo->vel_world_x;
        algo->vel_world_last_y = algo->vel_world_y;
        algo->has_last_vel_world = 1U;
    }
    v_last_x = algo->vel_world_last_x;
    v_last_y = algo->vel_world_last_y;

    /* 第一次梯形积分：加速度 -> 速度 */
    algo->vel_world_x += 0.5f * (algo->acc_world_x + a_last_x) * dt;
    algo->vel_world_y += 0.5f * (algo->acc_world_y + a_last_y) * dt;
    algo->acc_world_last_x = algo->acc_world_x;
    algo->acc_world_last_y = algo->acc_world_y;

    /* 第二次梯形积分：速度 -> 位置 */
    algo->pos_world_x += 0.5f * (algo->vel_world_x + v_last_x) * dt;
    algo->pos_world_y += 0.5f * (algo->vel_world_y + v_last_y) * dt;
    algo->vel_world_last_x = algo->vel_world_x;
    algo->vel_world_last_y = algo->vel_world_y;

    /* 状态转移矩阵 F:
     * [ 1   dt  -0.5*dt^2 ]
     * [ 0    1        -dt ]
     * [ 0    0          1 ]
     */
    f_mat[0][0] = 1.0f; f_mat[0][1] = dt;   f_mat[0][2] = -0.5f * dt * dt;
    f_mat[1][0] = 0.0f; f_mat[1][1] = 1.0f; f_mat[1][2] = -dt;
    f_mat[2][0] = 0.0f; f_mat[2][1] = 0.0f; f_mat[2][2] = 1.0f;

    /* 计算 FP = F * P_x */
    for (r = 0U; r < 3U; r++)
    {
        for (c = 0U; c < 3U; c++)
        {
            fp[r][c] = 0.0f;
            for (k = 0U; k < 3U; k++)
            {
                fp[r][c] += f_mat[r][k] * algo->p_acc_x[k][c];
            }
        }
    }
    /* 计算 P_new_x = FP * F^T + Q */
    for (r = 0U; r < 3U; r++)
    {
        for (c = 0U; c < 3U; c++)
        {
            p_new_x[r][c] = 0.0f;
            for (k = 0U; k < 3U; k++)
            {
                p_new_x[r][c] += fp[r][k] * f_mat[c][k];
            }
        }
    }
    p_new_x[0][0] += IMU_ALGO_Q_POS;
    p_new_x[1][1] += IMU_ALGO_Q_VEL;
    p_new_x[2][2] += IMU_ALGO_Q_ACC_BIAS;

    /* 计算 FP = F * P_y */
    for (r = 0U; r < 3U; r++)
    {
        for (c = 0U; c < 3U; c++)
        {
            fp[r][c] = 0.0f;
            for (k = 0U; k < 3U; k++)
            {
                fp[r][c] += f_mat[r][k] * algo->p_acc_y[k][c];
            }
        }
    }
    /* 计算 P_new_y = FP * F^T + Q */
    for (r = 0U; r < 3U; r++)
    {
        for (c = 0U; c < 3U; c++)
        {
            p_new_y[r][c] = 0.0f;
            for (k = 0U; k < 3U; k++)
            {
                p_new_y[r][c] += fp[r][k] * f_mat[c][k];
            }
        }
    }
    p_new_y[0][0] += IMU_ALGO_Q_POS;
    p_new_y[1][1] += IMU_ALGO_Q_VEL;
    p_new_y[2][2] += IMU_ALGO_Q_ACC_BIAS;

    for (r = 0U; r < 3U; r++)
    {
        for (c = 0U; c < 3U; c++)
        {
            algo->p_acc_x[r][c] = p_new_x[r][c];
            algo->p_acc_y[r][c] = p_new_y[r][c];
        }
    }
}

/**
 * @brief  第 5/9 层：ZUPT 零速度触发时的 3 状态卡尔曼测量校正 (以 z=0 更新速度、坐标趋势与零偏)
 * @param  algo 算法实例指针
 * @retval None
 */
void ImuAlgo_UpdateZuptKalman(imu_algo_t *algo)
{
    float s_x, s_y;
    float k_x[3], k_y[3];
    float y_innov_x, y_innov_y;
    uint8_t r, c;

    if (algo == 0)
    {
        return;
    }

    /* 观测方程 H = [0, 1, 0]，观测 z_vel = 0.0 */
    y_innov_x = 0.0f - algo->vel_world_x;
    y_innov_y = 0.0f - algo->vel_world_y;

    s_x = algo->p_acc_x[1][1] + IMU_ALGO_R_ZUPT;
    s_y = algo->p_acc_y[1][1] + IMU_ALGO_R_ZUPT;

    if (s_x < 0.0001f) s_x = 0.0001f;
    if (s_y < 0.0001f) s_y = 0.0001f;

    for (r = 0U; r < 3U; r++)
    {
        k_x[r] = algo->p_acc_x[r][1] / s_x;
        k_y[r] = algo->p_acc_y[r][1] / s_y;
    }

    /* 状态向量更新 [pos, vel, bias]^T */
    algo->pos_world_x += k_x[0] * y_innov_x;
    algo->vel_world_x += k_x[1] * y_innov_x;
    algo->acc_bias_x  += k_x[2] * y_innov_x;

    algo->pos_world_y += k_y[0] * y_innov_y;
    algo->vel_world_y += k_y[1] * y_innov_y;
    algo->acc_bias_y  += k_y[2] * y_innov_y;

    /* 协方差 P = (I - K*H) * P */
    for (r = 0U; r < 3U; r++)
    {
        for (c = 0U; c < 3U; c++)
        {
            algo->p_acc_x[r][c] -= k_x[r] * algo->p_acc_x[1][c];
            algo->p_acc_y[r][c] -= k_y[r] * algo->p_acc_y[1][c];
        }
    }
}

/**
 * @brief  获取姿态变换去偏后的世界坐标系 X 轴线性加速度
 * @retval 加速度值(m/s²)
 */
float ImuAlgo_GetAccWorldX(const imu_algo_t *algo)
{
    if (algo == 0) return 0.0f;
    return algo->acc_world_x;
}

/**
 * @brief  获取姿态变换去偏后的世界坐标系 Y 轴线性加速度
 * @retval 加速度值(m/s²)
 */
float ImuAlgo_GetAccWorldY(const imu_algo_t *algo)
{
    if (algo == 0) return 0.0f;
    return algo->acc_world_y;
}

/**
 * @brief  获取梯形积分与 ZUPT 修正后的世界坐标系 X 轴速度
 * @retval 速度值(m/s)
 */
float ImuAlgo_GetVelX(const imu_algo_t *algo)
{
    if (algo == 0) return 0.0f;
    return algo->vel_world_x;
}

/**
 * @brief  获取梯形积分与 ZUPT 修正后的世界坐标系 Y 轴速度
 * @retval 速度值(m/s)
 */
float ImuAlgo_GetVelY(const imu_algo_t *algo)
{
    if (algo == 0) return 0.0f;
    return algo->vel_world_y;
}

/**
 * @brief  获取梯形二次双重积分得到的世界坐标系 X 轴位置坐标 (Position X)
 * @retval 位置值(m)
 */
float ImuAlgo_GetPosX(const imu_algo_t *algo)
{
    if (algo == 0) return 0.0f;
    return algo->pos_world_x;
}

/**
 * @brief  获取梯形二次双重积分得到的世界坐标系 Y 轴位置坐标 (Position Y)
 * @retval 位置值(m)
 */
float ImuAlgo_GetPosY(const imu_algo_t *algo)
{
    if (algo == 0) return 0.0f;
    return algo->pos_world_y;
}

/**
 * @brief  获取实时在线跟踪收敛出的 X 轴加速度零漂
 * @retval 零漂值(m/s²)
 */
float ImuAlgo_GetAccBiasX(const imu_algo_t *algo)
{
    if (algo == 0) return 0.0f;
    return algo->acc_bias_x;
}

/**
 * @brief  获取实时在线跟踪收敛出的 Y 轴加速度零漂
 * @retval 零漂值(m/s²)
 */
float ImuAlgo_GetAccBiasY(const imu_algo_t *algo)
{
    if (algo == 0) return 0.0f;
    return algo->acc_bias_y;
}

/**
 * @brief  获取当前是否处于 ZUPT 零速度驻停模式
 * @retval 1:触发ZUPT, 0:运动
 */
uint8_t ImuAlgo_IsZuptActive(const imu_algo_t *algo)
{
    if (algo == 0) return 0U;
    return algo->zupt_active;
}

/**
 * @brief  一键清空复位底盘二次积分积累的位置与速度坐标 (PosX/PosY/VelX/VelY 清零)
 * @param  algo 算法实例指针
 * @retval None
 */
void ImuAlgo_ResetPosition(imu_algo_t *algo)
{
    if (algo == 0)
    {
        return;
    }
    algo->pos_world_x = 0.0f;
    algo->pos_world_y = 0.0f;
    algo->vel_world_x = 0.0f;
    algo->vel_world_y = 0.0f;
    algo->acc_world_last_x = 0.0f;
    algo->acc_world_last_y = 0.0f;
    algo->vel_world_last_x = 0.0f;
    algo->vel_world_last_y = 0.0f;
}
