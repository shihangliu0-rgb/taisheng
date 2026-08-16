/**
 * @file    imu_algo.h
 * @brief   IMU 传感器混合工程架构：原工程官方姿态平滑 + 11 层加速度/速度/位置双重积分惯导算法头文件
 * @note    独立算法模块，包含标量 Yaw 卡尔曼、自适应低通、姿态正交旋转、Butterworth、ZUPT 与梯形双重积分
 */

#ifndef IMU_ALGO_H
#define IMU_ALGO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"   /* 移植自 F405 工程，平台头由 stm32f4xx_hal.h 改为 H7 */
#include <stdint.h>
#include <math.h>


/* ==========================================================================
 *            IMU 算法调参专区 (原位于自有 imu.h，现随算法模块自带)
 *  说明：驱动层 imu.h 已改用抬升分支版本，故把本算法所需的调参宏集中到这里，
 *        改参数只动这一处，不影响驱动与上层任务。
 * ========================================================================== */

/* ---- Yaw 标量卡尔曼 ---- */
#define IMU_ALGO_Q_YAW                  0.001f   /* 过程噪声：越大越信任陀螺积分，响应快 */
#define IMU_ALGO_R_YAW                  3.0f     /* 测量噪声：越大越压制传感器抖动 */

/* ---- 角速度错帧保护与自适应一阶低通 ---- */
#define IMU_ALGO_GYRO_ABS_LIMIT         2000.0f  /* 绝对量程保护 deg/s，超过判错帧 */
#define IMU_ALGO_GYRO_STEP_LIMIT        500.0f   /* 单帧最大阶跃 deg/s，防尖峰 */
#define IMU_ALGO_GYRO_MOTION_THRESHOLD  5.0f     /* 静止/运动切换阈值 deg/s */
#define IMU_ALGO_GYRO_ALPHA_STATIC      0.02f    /* 静止强滤波系数 */
#define IMU_ALGO_GYRO_ALPHA_MOTION      0.30f    /* 运动弱滤波系数(低延迟) */

/* ---- 加速度二阶 Butterworth 低通 (fs=200Hz, fc=25Hz) ---- */
#define IMU_ALGO_ACC_LPF_B0             0.097631f
#define IMU_ALGO_ACC_LPF_B1             0.195262f
#define IMU_ALGO_ACC_LPF_B2             0.097631f
#define IMU_ALGO_ACC_LPF_A1             -0.942809f
#define IMU_ALGO_ACC_LPF_A2             0.333333f
#define IMU_ALGO_ACC_ABS_LIMIT          100.0f   /* 加速度绝对上限 m/s^2 */

/* ---- ZUPT 零速更新与加速度零偏在线估计 ---- */
#define IMU_ALGO_ZUPT_GYRO_THRESHOLD    0.25f    /* 判静止的角速度阈值 deg/s */
#define IMU_ALGO_ZUPT_ACC_STD_THRESHOLD 0.08f    /* 判静止的加速度标准差阈值 m/s^2 */
#define IMU_ALGO_ACC_BIAS_ALPHA         0.001f   /* 加速度零偏跟踪系数 */

/* ---- 三状态 [pos, vel, acc_bias] 卡尔曼 ---- */
#define IMU_ALGO_Q_POS                  0.0001f
#define IMU_ALGO_Q_VEL                  0.005f
#define IMU_ALGO_Q_ACC_BIAS             0.00002f
#define IMU_ALGO_R_ZUPT                 0.01f

/* ---- 地面机器人非全向约束：麦轮/全向轮请设为 0U ---- */
#define IMU_ALGO_NONHOLONOMIC_CONSTRAINT 0U

#define IMU_ALGO_HISTORY_SIZE   100U


/**
 * @brief  二阶 Butterworth 低通滤波器状态结构体
 */
typedef struct
{
    float x1, x2;               // 滤波历史输入项
    float y1, y2;               // 滤波历史输出项
} biquad_lpf_t;

/**
 * @brief  IMU 核心算法状态与历史缓存结构体
 */
typedef struct
{
    /* =================== 原工程融合路线：官方绝对角度首帧对齐与标量卡尔曼 =================== */
    float x_yaw;                // 官方偏航角经卡尔曼去杂波平滑后的数值 (deg)
    float p_yaw;                // 卡尔曼估测误差方差 P
    uint8_t kalman_yaw_inited;  // 卡尔曼是否已用首帧有效测量值初始化

    /* 自适应角速度一阶低通滤波 */
    float gyro_filtered;        // 经自适应一阶低通去噪后的角速度 (deg/s)
    float gyro_last_raw;        // 上一帧原始角速度值 (deg/s)
    uint8_t has_last_raw;       // 是否有上一帧原始值记录

    /* =================== 加速度双重积分、二阶滤波、ZUPT 与三状态卡尔曼 =================== */
    /* 二阶 Butterworth 低通滤波状态 (X 与 Y 轴) */
    biquad_lpf_t lpf_acc_x;
    biquad_lpf_t lpf_acc_y;
    float acc_x_filt;           // 机体坐标系滤波后 X 轴加速度 (m/s²)
    float acc_y_filt;           // 机体坐标系滤波后 Y 轴加速度 (m/s²)

    /* 在线零偏跟踪估计 (X 与 Y 轴) */
    float acc_bias_x;           // X 轴加速度在线零漂 (m/s²)
    float acc_bias_y;           // Y 轴加速度在线零漂 (m/s²)

    /* 世界坐标系线性加速度 (经过 Yaw 旋转矩阵正交投影变换与去零漂) */
    float acc_world_x;          // 世界坐标系 X 轴线性加速度 (m/s²)
    float acc_world_y;          // 世界坐标系 Y 轴线性加速度 (m/s²)
    float acc_world_last_x;     // 上一帧参与积分的世界坐标加速度 X
    float acc_world_last_y;     // 上一帧参与积分的世界坐标加速度 Y
    uint8_t has_last_acc_world;
    float acc_last_raw_x;       // 上一帧采样 X 加速度
    float acc_last_raw_y;       // 上一帧采样 Y 加速度
    uint8_t has_last_acc_raw;

    /* 速度梯形积分与 ZUPT 控制 */
    float vel_world_x;          // 世界坐标系 X 轴速度 (m/s)
    float vel_world_y;          // 世界坐标系 Y 轴速度 (m/s)
    float vel_world_last_x;     // 上一帧速度 X
    float vel_world_last_y;     // 上一帧速度 Y
    uint8_t has_last_vel_world;
    uint8_t zupt_active;        // ZUPT 静止驻停触发标记 (1:触发ZUPT, 0:运动中)

    /* 位置梯形双重积分 (X 与 Y 轴) */
    float pos_world_x;          // 世界坐标系 X 轴累计位置 (m)
    float pos_world_y;          // 世界坐标系 Y 轴累计位置 (m)

    /* 三状态 [pos, vel, acc_bias]^T 卡尔曼滤波误差协方差矩阵 P */
    float p_acc_x[3][3];        // X 轴三状态卡尔曼协方差矩阵
    float p_acc_y[3][3];        // Y 轴三状态卡尔曼协方差矩阵
    uint8_t pos_kalman_inited;  // 位置卡尔曼是否初始化

    /* 滑动窗口标准差与振动判断 (100帧窗) */
    float acc_history_x[IMU_ALGO_HISTORY_SIZE];
    float acc_history_y[IMU_ALGO_HISTORY_SIZE];
    uint16_t acc_history_idx;
    uint16_t acc_history_count;

    /* 微秒级高精度 dt (基于 DWT 周期计数器) */
    uint32_t last_dwt_cycles;   // 上一次调用的 DWT 周期值
    uint8_t dwt_inited;         // DWT 是否初始化
} imu_algo_t;

/**
 * @brief  初始化算法模块各个参数与结构体
 * @param  algo 算法实例指针
 * @retval None
 */
void ImuAlgo_Init(imu_algo_t *algo);

/**
 * @brief  异常角速度检验保护 (超过上限或单帧阶跃过大视为错帧)
 * @param  algo     算法实例指针
 * @param  gyro_raw 当前读到的原始角速度 (deg/s)
 * @retval 1:有效正常数据, 0:异常错帧需丢弃
 */
uint8_t ImuAlgo_CheckGyroValid(imu_algo_t *algo, float gyro_raw);

/**
 * @brief  角速度噪声自适应一阶滤波 (静止强滤波 alpha=0.02, 运动弱滤波 alpha=0.3)
 * @param  algo     算法实例指针
 * @param  gyro_raw 当前去完零偏的有效角速度 (deg/s)
 * @retval 滤波计算后得到的平滑角速度 (deg/s)
 */
float ImuAlgo_AdaptiveFilterGyro(imu_algo_t *algo, float gyro_raw);

/**
 * @brief  获取基于 DWT 微秒级时间计数的帧时间差 dt
 * @param  algo 算法实例指针
 * @retval dt (秒)，有效范围 [0.00001, 0.02]，超时默认 0.005
 */
float ImuAlgo_GetDtSeconds(imu_algo_t *algo);

/**
 * @brief  原工程路线：对官方传回的对零参考后偏航角执行标量一阶卡尔曼去杂波平滑
 * @param  algo     算法实例指针
 * @param  yaw_meas 官方 0x03 帧去首帧对齐后的偏航角 (deg)
 * @retval 平滑去杂波后的最优 Yaw 角度 (deg)
 */
float ImuAlgo_ApplyYawKalmanDeg(imu_algo_t *algo, float yaw_meas);

/**
 * @brief  获取最终最优偏航角输出 (Yaw)
 * @param  algo 算法实例指针
 * @retval 最优偏航角 (-180.0 ~ 180.0 deg)
 */
float ImuAlgo_GetYaw(const imu_algo_t *algo);

/* =================== 加速度处理链、二阶滤波、ZUPT与梯形二次积分 =================== */

/**
 * @brief  第 4 层：异常线性加速度检测保护 (判断采样是否超过最大额定量程)
 * @param  algo      算法实例指针
 * @param  acc_raw_x 采样 X 加速度 (m/s²)
 * @param  acc_raw_y 采样 Y 加速度 (m/s²)
 * @retval 1:有效数据, 0:错报尖峰包需抛弃
 */
uint8_t ImuAlgo_CheckAccValid(imu_algo_t *algo, float acc_raw_x, float acc_raw_y);

/**
 * @brief  第 4 层：二阶 Butterworth 低通滤波 (20~30Hz 截止频率，强力滤除电机机械振动)
 * @param  algo       算法实例指针
 * @param  acc_raw_x  原始有效 X 轴加速度
 * @param  acc_raw_y  原始有效 Y 轴加速度
 * @param  acc_filt_x 输出二阶滤波后的 X 轴加速度
 * @param  acc_filt_y 输出二阶滤波后的 Y 轴加速度
 * @retval None
 */
void ImuAlgo_BiquadFilterAcc(imu_algo_t *algo, float acc_raw_x, float acc_raw_y, float *acc_filt_x, float *acc_filt_y);

/**
 * @brief  第 7 层 & 第 5/2 层：滑动窗口振动判断、ZUPT 零速驻停与加速度零漂在线估计
 * @param  algo       算法实例指针
 * @param  acc_filt_x 滤波后 X 轴加速度 (m/s²)
 * @param  acc_filt_y 滤波后 Y 轴加速度 (m/s²)
 * @retval None
 */
void ImuAlgo_UpdateZuptAndBias(imu_algo_t *algo, float acc_filt_x, float acc_filt_y);

/**
 * @brief  第 3 层：姿态变换与重力补偿 —— 减去在线估算零漂，根据官方偏航角将机体加速度向世界坐标系正交投影
 * @param  algo       算法实例指针
 * @param  acc_filt_x 机体滤波 X 加速度
 * @param  acc_filt_y 机体滤波 Y 加速度
 * @param  yaw_deg    当前官方平滑偏航角 (deg)
 * @retval None
 */
void ImuAlgo_RotateAndCompensateAcc(imu_algo_t *algo, float acc_filt_x, float acc_filt_y, float yaw_deg);

/**
 * @brief  第 11 层：地面差速/直行机器人约束运动模型 (侧滑速度阻尼限幅)
 * @param  algo    算法实例指针
 * @param  yaw_deg 当前偏航角 (deg)
 * @retval None
 */
void ImuAlgo_ApplyNonHolonomicConstraint(imu_algo_t *algo, float yaw_deg);

/**
 * @brief  第 8 层 & 第 9 层：微秒 dt 梯形二次双重积分 (a -> v -> pos) + 三状态 [pos, vel, a_bias] 协方差预测
 * @param  algo 算法实例指针
 * @param  dt   当前时间差 (秒)
 * @retval None
 */
void ImuAlgo_PredictDoubleIntegral(imu_algo_t *algo, float dt);

/**
 * @brief  第 5/9 层：ZUPT 零速度触发时的 3 状态卡尔曼测量校正 (以 z=0 更新速度、坐标趋势与零偏)
 * @param  algo 算法实例指针
 * @retval None
 */
void ImuAlgo_UpdateZuptKalman(imu_algo_t *algo);

/**
 * @brief  获取姿态变换去偏后的世界坐标系 X 轴线性加速度
 * @retval 加速度值(m/s²)
 */
float ImuAlgo_GetAccWorldX(const imu_algo_t *algo);

/**
 * @brief  获取姿态变换去偏后的世界坐标系 Y 轴线性加速度
 * @retval 加速度值(m/s²)
 */
float ImuAlgo_GetAccWorldY(const imu_algo_t *algo);

/**
 * @brief  获取梯形积分与 ZUPT 修正后的世界坐标系 X 轴速度
 * @retval 速度值(m/s)
 */
float ImuAlgo_GetVelX(const imu_algo_t *algo);

/**
 * @brief  获取梯形积分与 ZUPT 修正后的世界坐标系 Y 轴速度
 * @retval 速度值(m/s)
 */
float ImuAlgo_GetVelY(const imu_algo_t *algo);

/**
 * @brief  获取梯形二次双重积分得到的世界坐标系 X 轴位置坐标 (Position X)
 * @retval 位置值(m)
 */
float ImuAlgo_GetPosX(const imu_algo_t *algo);

/**
 * @brief  获取梯形二次双重积分得到的世界坐标系 Y 轴位置坐标 (Position Y)
 * @retval 位置值(m)
 */
float ImuAlgo_GetPosY(const imu_algo_t *algo);

/**
 * @brief  获取实时在线跟踪收敛出的 X 轴加速度零漂
 * @retval 零漂值(m/s²)
 */
float ImuAlgo_GetAccBiasX(const imu_algo_t *algo);

/**
 * @brief  获取实时在线跟踪收敛出的 Y 轴加速度零漂
 * @retval 零漂值(m/s²)
 */
float ImuAlgo_GetAccBiasY(const imu_algo_t *algo);

/**
 * @brief  获取当前是否处于 ZUPT 零速度驻停模式
 * @retval 1:静止触发ZUPT, 0:运动
 */
uint8_t ImuAlgo_IsZuptActive(const imu_algo_t *algo);

/**
 * @brief  一键复位清零位置坐标和线速度
 * @param  algo 算法实例指针
 * @retval None
 */
void ImuAlgo_ResetPosition(imu_algo_t *algo);

#ifdef __cplusplus
}
#endif

#endif /* IMU_ALGO_H */
