#ifndef IMU_MAIN_H
#define IMU_MAIN_H

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    IMU_STATE_UNINITIALIZED,  // 尚未初始化
    IMU_STATE_CALIBRATING,    // 传感器内部陀螺仪校准
    IMU_STATE_CONFIGURING,    // 配置串口主动输出内容
    IMU_STATE_BIAS_SAMPLING,  // 采集软件零偏
    IMU_STATE_READY,          // 数据可供底盘使用
    IMU_STATE_ERROR           // 初始化或命令发送失败
} imu_state_t;

typedef struct
{
    float yaw_deg;                 // 归零并滤波后的偏航角，单位为 deg
    float gyro_z_deg_s;            // 零偏修正后的 Z 轴角速度，单位为 deg/s
    float gyro_bias_deg_s;         // 上电采集得到的 Z 轴角速度零偏
    imu_state_t state;             // 当前初始化或运行状态
    uint32_t last_rx_ms;           // 最近有效帧的系统时间
    uint32_t valid_frame_count;    // 有效帧累计数量
    uint32_t invalid_frame_count;  // 错误帧累计数量
    uint32_t rx_overflow_count;    // 软件接收队列溢出次数
    uint32_t uart_error_count;     // HAL 串口错误次数
    float target_yaw_deg;          // 航向保持目标角，单位为 deg
    float yaw_error_deg;           // 航向闭环最短角度误差，单位为 deg
    int16_t omega_output;          // 输出给底盘混控的旋转指令
    bool yaw_valid;                // 偏航角是否已完成归零
    bool gyro_valid;               // 角速度零偏是否已完成
    bool online;                   // 最近 100 ms 内两类控制数据是否都有效
    bool yaw_hold_enabled;         // 航向保持功能是否使能
    bool yaw_hold_active;          // 当前是否由航向闭环控制旋转
} imu_data_t;

//航向环
typedef struct
{
    uint32_t boot_delay_ms;           // 上电后等待 IMU 启动的时间
    uint32_t cal_cmd_delay_ms;        // 发送校准命令前的间隔
    uint32_t gyro_cal_wait_ms;        // 等待 IMU 硬件陀螺仪校准完成的时间
    uint32_t config_start_delay_ms;   // 校准完成后，开始发配置命令前的等待
    uint32_t config_cmd_delay_ms;     // 每条配置命令之间的发送间隔
    uint16_t gyro_bias_samples;       // 软件零偏采样采集的帧数
    uint32_t online_timeout_ms;       // 判定 IMU 离线的超时时间
    uint32_t recovery_timeout_ms;     // 持续离线多久后启动自动恢复
    uint32_t recovery_retry_ms;       // 两次自动恢复之间的最小间隔
    uint32_t yaw_tx_period_ms;        // 向上位机发送偏航角的周期
    uint32_t yaw_control_period_ms;   // 航向保持控制环的执行周期
    int16_t yaw_cmd_threshold;        // 手动旋转指令阈值（判定是否手动旋转）
    int16_t yaw_linear_threshold;     // 底盘平移速度阈值（判定静止/运动）
    float kalman_q;                   // 偏航角卡尔曼滤波的过程噪声
    float kalman_r;                   // 偏航角卡尔曼滤波的测量噪声
    float gyro_filter_q;              // 角速度卡尔曼滤波的过程噪声
    float gyro_filter_r;              // 角速度卡尔曼滤波的测量噪声
    float yaw_tx_scale;               // 偏航角上报时的放大倍数
    float yaw_deadzone_deg;           // 航向保持死区（误差小于此值不输出）
    float yaw_i_active_deg;           // 积分分离阈值（误差在此范围内才积分）
    float yaw_i_decay;                // 积分分离时的积分衰减系数
    float yaw_gyro_k;                 // 角速度前馈阻尼系数
} imu_config_t;
/**
 * @brief 初始化 IMU 上层逻辑并启动 USART1 接收
 * @retval HAL 状态
 */
HAL_StatusTypeDef ImuMain_Init(void);

/**
 * @brief 执行 IMU 初始化状态机、数据解析和零偏修正
 * @retval None
 */
void ImuMain_Run1ms(void);

/**
 * @brief 将下一帧欧拉角设为新的偏航角零点
 * @retval HAL 状态
 */
HAL_StatusTypeDef ImuMain_ZeroYaw(void);

/**
 * @brief 根据当前航向计算底盘旋转输出
 * @param vx X 方向速度指令
 * @param vy Y 方向速度指令
 * @param omega 手动旋转指令
 * @retval 修正后的底盘旋转指令
 */
int16_t ImuMain_CalcOmega(int16_t vx, int16_t vy, int16_t omega);

/**
 * @brief 设置航向保持目标角
 * @param target_yaw_deg 目标偏航角，单位为 deg
 * @retval HAL 状态
 */
HAL_StatusTypeDef ImuMain_SetTargetYaw(float target_yaw_deg);

/**
 * @brief 设置航向保持使能状态
 * @param enabled 是否使能
 * @retval None
 */
void ImuMain_EnableYawHold(bool enabled);

/**
 * @brief 获取 IMU 上层输出数据
 * @param data 数据输出地址
 * @retval 是否获取成功
 */
bool ImuMain_GetData(imu_data_t *data);

/**
 * @brief 通过上位机串口回传当前 yaw 角
 * @param uart 上位机串口句柄
 * @retval HAL 状态
 */
HAL_StatusTypeDef ImuMain_SendYaw(UART_HandleTypeDef *uart);

/**
 * @brief 转发 HAL 串口空闲事件
 * @param uart 触发回调的串口
 * @param size HAL 返回的当前 DMA 写入位置
 * @retval None
 */
void ImuMain_HandleRxEvent(UART_HandleTypeDef *uart, uint16_t size);

/**
 * @brief 转发 HAL 串口错误事件
 * @param uart 触发错误的串口
 * @retval None
 */
void ImuMain_HandleUartError(UART_HandleTypeDef *uart);

#endif
