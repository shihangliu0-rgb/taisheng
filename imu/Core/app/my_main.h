/**
 * @file    my_main.h
 * @brief   用户自定义主应用层头文件
 * @note    存放全系统自写的主循环调用、调试打印宏及串口回调抽象
 */

#ifndef MY_MAIN_H
#define MY_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/**
 * @brief  用户自定义主程序初始化（初始化 IMU 与串口日志等）
 * @retval None
 */
void MyMain_Init(void);

/**
 * @brief  用户自定义主循环函数（周期更新 IMU 状态并打印姿态数据）
 * @retval None
 */
void MyMain_Loop(void);

/**
 * @brief  串口 DMA 或空闲中断接收回调处理
 * @param  huart 串口外设句柄
 * @param  size  接收数据长度
 * @retval None
 */
void MyMain_RxCallback(UART_HandleTypeDef *huart, uint16_t size);

/**
 * @brief  串口发送完成中断回调处理
 * @param  huart 串口外设句柄
 * @retval None
 */
void MyMain_TxCallback(UART_HandleTypeDef *huart);

/**
 * @brief  串口通信错误异常回调处理
 * @param  huart 串口外设句柄
 * @retval None
 */
void MyMain_ErrorCallback(UART_HandleTypeDef *huart);

/**
 * @brief  通过 USART1 串口发送格式化字符串
 * @param  format 格式化字符串
 * @param  ...    可变参数
 * @retval None
 */
void MyMain_PrintLog(const char *format, ...);

#define LOG_INFO(format, ...)  MyMain_PrintLog("[INFO] " format "\r\n", ##__VA_ARGS__)
#define LOG_WARN(format, ...)  MyMain_PrintLog("[WARN] " format "\r\n", ##__VA_ARGS__)
#define LOG_ERROR(format, ...) MyMain_PrintLog("[ERROR] " format "\r\n", ##__VA_ARGS__)
#define LOG_DEBUG(format, ...) MyMain_PrintLog("[DEBUG] " format "\r\n", ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* MY_MAIN_H */
