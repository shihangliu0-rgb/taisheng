#ifndef IMU_H
#define IMU_H

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float gyro_z_deg_s;       // Z 轴原始角速度
    float yaw_deg;            // 原始偏航角
    uint32_t gyro_sequence;   // 角速度有效帧序号
    uint32_t yaw_sequence;    // 偏航角有效帧序号
    bool gyro_valid;          // 是否收到过有效角速度帧
    bool yaw_valid;           // 是否收到过有效欧拉角帧
} imu_raw_data_t;

typedef struct
{
    uint32_t valid_frame_count;    // 完整且数据有效的帧数
    uint32_t invalid_frame_count;  // 帧格式或数值错误次数
    uint32_t rx_overflow_count;    // 软件接收队列溢出次数
    uint32_t uart_error_count;     // HAL 串口错误次数
    uint32_t last_valid_ms;        // 最近有效帧的系统时间
} imu_stats_t;

/**
 * @brief 初始化 DM-IMU L1 串口驱动
 * @param uart 已配置为 921600 波特率和循环接收 DMA 的串口
 * @retval HAL 状态
 */
HAL_StatusTypeDef Imu_Init(UART_HandleTypeDef *uart);

/**
 * @brief 处理接收队列中的数据，并在串口错误后恢复 DMA
 * @retval None
 */
void Imu_Process(void);

/**
 * @brief 向 DM-IMU L1 发送配置命令
 * @param data 命令数据
 * @param length 命令长度
 * @retval HAL 状态
 */
HAL_StatusTypeDef Imu_Send(const uint8_t *data, uint16_t length);

/**
 * @brief 获取最近一次有效的原始数据
 * @param data 原始数据输出地址
 * @retval 是否获取成功
 */
bool Imu_GetRawData(imu_raw_data_t *data);

/**
 * @brief 获取驱动通信统计信息
 * @param stats 统计信息输出地址
 * @retval 是否获取成功
 */
bool Imu_GetStats(imu_stats_t *stats);

/**
 * @brief 处理 HAL 串口空闲事件，搬运循环 DMA 新数据
 * @param uart 触发回调的串口
 * @param size HAL 返回的当前 DMA 写入位置
 * @retval None
 */
void Imu_HandleRxEvent(UART_HandleTypeDef *uart, uint16_t size);

/**
 * @brief 记录串口错误并请求任务上下文恢复 DMA
 * @param uart 触发错误的串口
 * @retval None
 */
void Imu_HandleUartError(UART_HandleTypeDef *uart);

#endif
