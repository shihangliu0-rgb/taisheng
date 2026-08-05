/**
 * @file    imu.h
 * @brief   IMU 传感器驱动与卡尔曼姿态/双重积分位置解算头文件
 * @note    包含偏航角、二阶低通滤波、姿态旋转变换、ZUPT/ZARU 与梯形二次积分位置解算的调参专区
 */

#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define IMU_RX_DMA_BUFFER_SIZE  256U

/* ========================================================================== */
/*                   IMU 姿态解算核心算法参数配置 (调参专区)                     */
/* ========================================================================== */

/**
 * @brief  二状态卡尔曼过程噪声方差 —— Yaw 角度变化信度
 * @note   影响什么：增大该值表示更信任陀螺仪积分，角度响应更快但可能残留轻微高频抖动；
 *                  减小该值表示更信任测量值，平滑性更好但急转弯响应可能稍慢。
 */
#define IMU_ALGO_Q_YAW                  0.001f

/**
 * @brief  二状态卡尔曼过程噪声方差 —— 陀螺仪零漂 (Bias) 跟踪速度 (ZARU 零角速度更新)
 * @note   影响什么：控制系统判定零偏变化的快慢。数值越小（如 0.00001），零偏变化越缓慢平稳，
 *                  适用于缓慢的温度漂移；数值增大则零偏估计变化剧烈，易被短时运动误导。
 */
#define IMU_ALGO_Q_BIAS                 0.00001f

/**
 * @brief  二状态卡尔曼测量噪声方差 —— 传感器绝对偏航角噪声度量
 * @note   影响什么：增大该值表示传感器读数抖动大（颠簸/强磁场），滤波器会大力压制测量噪声；
 *                  减小该值则对硬件角度反馈更加灵敏。
 */
#define IMU_ALGO_R_YAW                  3.0f

/**
 * @brief  异常角速度绝对量程保护阈值 (°/s)
 * @note   影响什么：单帧读数超过该值即视为串口错包直接剔除；车体极限飞转超过该值时需调大。
 */
#define IMU_ALGO_GYRO_ABS_LIMIT         2000.0f

/**
 * @brief  连续两帧角速度最大允许阶跃跳变阈值 (°/s)
 * @note   影响什么：防突发尖峰脉冲 (Spike) 干扰。设太小会把急刹转向误判为错帧；设太大抗干扰能力降低。
 */
#define IMU_ALGO_GYRO_STEP_LIMIT        500.0f

/**
 * @brief  静止与快转状态切换的角速度阈值 (°/s)
 * @note   影响什么：低于此速度应用“静止强滤波”抑制底噪；高于此速度切换“运动弱滤波”消除延迟。
 */
#define IMU_ALGO_GYRO_MOTION_THRESHOLD  5.0f

/**
 * @brief  静止/微动状态下角速度一阶滤波系数 alpha
 * @note   影响什么：alpha 越小滤波越重（如 0.02），静止数据更加纯净平直；偏大则去噪能力变弱。
 */
#define IMU_ALGO_GYRO_ALPHA_STATIC      0.02f

/**
 * @brief  运动/快转状态下角速度一阶滤波系数 alpha
 * @note   影响什么：alpha 越接近 1.0（如 0.30），系统跟踪急拐弯的实时性越高、延迟越低。
 */
#define IMU_ALGO_GYRO_ALPHA_MOTION      0.30f

/**
 * @brief  100 帧滑动窗口静止在线自检 —— 角速度平均值阈值 (°/s)
 * @note   影响什么：平均角速度绝对值小于此阈值，且标准差合格时，才认定为静止并自动更新零偏。
 */
#define IMU_ALGO_STAT_MEAN_THRESHOLD    0.20f

/**
 * @brief  100 帧滑动窗口静止在线自检 —— 角速度标准差 (std) 阈值 (°/s)
 * @note   影响什么：标准差越小（如 0.05），要求车体必须极度静止不动才触发零漂校对；调大能容忍轻微震动。
 */
#define IMU_ALGO_STAT_STD_THRESHOLD     0.05f

/**
 * @brief  静止状态下角速度零偏在线跟踪平滑指数 alpha (ZARU)
 * @note   影响什么：bias = (1-alpha)*bias + alpha*mean。数值越小（如 0.01），零偏自适应修正越稳定不震荡。
 */
#define IMU_ALGO_BIAS_DYNAMIC_ALPHA     0.01f

/* ========================================================================== */
/*                IMU 加速度二阶滤波、姿态变换、ZUPT与二次积分调参专区              */
/* ========================================================================== */

/**
 * @brief  第一层：二阶 Butterworth 低通滤波器系数 (默认对应 fs=200Hz, fc=25Hz)
 * @note   影响什么：相比一阶 LPF，二阶 Butterworth 低通滤波能够在 20~30Hz 截止频率以下保持平坦通带，
 *                  并对高频电机振动进行 40dB/dec 的极强衰减。
 */
#define IMU_ALGO_ACC_LPF_B0             0.097631f
#define IMU_ALGO_ACC_LPF_B1             0.195262f
#define IMU_ALGO_ACC_LPF_B2             0.097631f
#define IMU_ALGO_ACC_LPF_A1             -0.942809f
#define IMU_ALGO_ACC_LPF_A2             0.333333f

/**
 * @brief  异常线性加速度绝对上限阈值 (m/s²)
 * @note   影响什么：单帧读数绝对值超过此上限值时，判定为串口传输错乱干扰包立即抛弃。
 */
#define IMU_ALGO_ACC_ABS_LIMIT          100.0f

/**
 * @brief  第二层：零速度更新 (ZUPT) —— 角速度静止限制阈值 (°/s)
 * @note   影响什么：判断车体是否停止的前提条件。只有角速度绝对值低于此阈值、且加速度平稳时才触动 ZUPT。
 */
#define IMU_ALGO_ZUPT_GYRO_THRESHOLD    0.25f

/**
 * @brief  第二层：零速度更新 (ZUPT) —— 加速度 100 帧标准差阈值 (m/s²)
 * @note   影响什么：标准差小于该阈值时判定加速度无波动、机器处于驻停状态，立即触发 vx=0 / vy=0，
 *                  直接根除任何速度与位置积分漂移。
 */
#define IMU_ALGO_ZUPT_ACC_STD_THRESHOLD 0.08f

/**
 * @brief  第三层：加速度零偏估计平滑跟踪系数 alpha
 * @note   影响什么：在驻停状态 (ZUPT) 下在线收敛加速度零偏 a_bias。alpha 越小（如 0.001），
 *                  零偏估算越细腻无波动。
 */
#define IMU_ALGO_ACC_BIAS_ALPHA         0.001f

/**
 * @brief  三状态卡尔曼 [pos, vel, acc_bias]^T 过程噪声方差 Q_pos, Q_vel, Q_bias
 * @note   影响什么：调节系统对加速度二重积分位置与速度预测的信任度；在 ZUPT 触动时融合速度观测。
 */
#define IMU_ALGO_Q_POS                  0.0001f
#define IMU_ALGO_Q_VEL                  0.005f
#define IMU_ALGO_Q_ACC_BIAS             0.00002f

/**
 * @brief  三状态卡尔曼 ZUPT 零测量噪声方差 R_zupt
 * @note   影响什么：当判定车体静止进行 v=0 测量反馈时的置信力度。
 */
#define IMU_ALGO_R_ZUPT                 0.01f

/**
 * @brief  第十一层：是否开启地面机器人非全向约束 (Non-Holonomic Constraint)
 * @note   影响什么：设为 1U 时在机体坐标系下阻尼侧向速度 vy_body ≈ 0，适用于差速与普通轮直行底盘；
 *                  如使用麦克纳姆轮或全向轮横向平移，请设为 0U。
 */
#define IMU_ALGO_NONHOLONOMIC_CONSTRAINT 1U

/* ========================================================================== */
/*                             数据结构与外部函数声明                         */
/* ========================================================================== */

/**
 * @brief  IMU 姿态解算与双重积分位置/速度输出数据结构体
 */
typedef struct
{
    float yaw_deg;              // 修正点与二状态卡尔曼滤波后的偏航角 Yaw (度)
    float gyro_z_deg_s;         // 校正零漂误差后的 Z 轴角速度 Gyro Z (度/秒)
    float gyro_bias_deg_s;      // 实时在线跟踪评估的最优角速度零漂均值 (度/秒)
    float acc_world_x_mps2;     // 姿态变换与去偏后的世界坐标系 X 轴线性加速度 (m/s²)
    float acc_world_y_mps2;     // 姿态变换与去偏后的世界坐标系 Y 轴线性加速度 (m/s²)
    float vel_world_x_mps;      // 梯形积分与 ZUPT 修正的世界坐标系 X 轴速度 (m/s)
    float vel_world_y_mps;      // 梯形积分与 ZUPT 修正的世界坐标系 Y 轴速度 (m/s)
    float pos_world_x_m;        // 梯形二次双重积分的世界坐标系 X 轴累计位置 (m)
    float pos_world_y_m;        // 梯形二次双重积分的世界坐标系 Y 轴累计位置 (m)
    float acc_bias_x_mps2;      // 实时在线跟踪估算出的 X 轴加速度零漂 (m/s²)
    float acc_bias_y_mps2;      // 实时在线跟踪估算出的 Y 轴加速度零漂 (m/s²)
    uint8_t gyro_bias_ready;    // 陀螺仪零漂标定完成状态 (1:标定成功, 0:未完成)
    uint8_t is_connected;       // 传感器状态标记
    uint8_t is_stationary;      // 当前是否判断为角速度静止状态 (1:静止, 0:运动)
    uint8_t zupt_active;        // 当前是否触发了 ZUPT 零速度更新 (1:驻停零速, 0:运动)
} imu_data_t;

/**
 * @brief  初始化 IMU 模块与卡尔曼滤波器，并启动 DMA 接收
 * @retval None
 */
void Imu_Init(void);

/**
 * @brief  主循环中持续调用的 IMU 任务更新函数（执行指令序列调度等）
 * @retval None
 */
void Imu_Update(void);

/**
 * @brief  开启串口 6 DMA 接收监听
 * @retval None
 */
void Imu_StartReceive(void);

/**
 * @brief  处理通过串口 6 接收到的原始数据帧
 * @param  data 接收缓冲区首地址
 * @param  len  当前收到的字节长度
 * @retval None
 */
void Imu_ProcessRxData(const uint8_t *data, uint16_t len);

/**
 * @brief  直接获取当前解算得到的最优偏航角 (Yaw)
 * @retval 偏航角角度值 (-180.0 ~ 180.0 度)
 */
float Imu_GetYaw(void);

/**
 * @brief  直接获取消除零漂与自适应滤波后的 Z 轴角速度 (Gyro Z)
 * @retval Z轴角速度(度/秒)
 */
float Imu_GetGyroZ(void);

/**
 * @brief  获取实时在线跟踪评估的最优角速度零偏 (Bias)
 * @retval 零漂值(度/秒)
 */
float Imu_GetBias(void);

/**
 * @brief  获取姿态旋转与去零差后的世界坐标系 X 轴线性加速度
 * @retval 加速度值(m/s²)
 */
float Imu_GetAccWorldX(void);

/**
 * @brief  获取姿态旋转与去零差后的世界坐标系 Y 轴线性加速度
 * @retval 加速度值(m/s²)
 */
float Imu_GetAccWorldY(void);

/**
 * @brief  获取梯形积分与 ZUPT 修正后的世界坐标系 X 轴速度
 * @retval 速度值(m/s)
 */
float Imu_GetVelX(void);

/**
 * @brief  获取梯形积分与 ZUPT 修正后的世界坐标系 Y 轴速度
 * @retval 速度值(m/s)
 */
float Imu_GetVelY(void);

/**
 * @brief  获取梯形二次积分解算得到的世界坐标系 X 轴累计位置 (Position X)
 * @retval 位置值(m)
 */
float Imu_GetPosX(void);

/**
 * @brief  获取梯形二次积分解算得到的世界坐标系 Y 轴累计位置 (Position Y)
 * @retval 位置值(m)
 */
float Imu_GetPosY(void);

/**
 * @brief  获取实时在线跟踪评估的 X 轴加速度零漂 (Acc Bias X)
 * @retval 零偏值(m/s²)
 */
float Imu_GetAccBiasX(void);

/**
 * @brief  获取实时在线跟踪评估的 Y 轴加速度零漂 (Acc Bias Y)
 * @retval 零偏值(m/s²)
 */
float Imu_GetAccBiasY(void);

/**
 * @brief  获取当前是否触发 ZUPT 零速度驻停
 * @retval 1:触发ZUPT, 0:运动
 */
uint8_t Imu_IsZuptActive(void);

/**
 * @brief  获取当前是否判定为车体静止
 * @retval 1:静止, 0:运动中
 */
uint8_t Imu_IsStationary(void);

/**
 * @brief  一键清空复位底盘二次积分积累的位置与速度坐标 (PosX/PosY/VelX/VelY 清零)
 * @retval None
 */
void Imu_ResetPosition(void);

/**
 * @brief  获取完整的 IMU 输出状态数据
 * @param  data 输出保存结构体指针
 * @retval None
 */
void Imu_GetData(imu_data_t *data);

/**
 * @brief  获取内部 DMA 接收缓冲区首地址
 * @retval 缓冲区首地址
 */
uint8_t *Imu_GetRxBuffer(void);

/**
 * @brief  启动陀螺仪零漂校准序列
 * @retval None
 */
void Imu_CalibrateGyro(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
