/**
 * @file    my_main.c
 * @brief   用户自定义主应用实现（初始化、调试串口输出及 IMU 调度循环）
 */

#include "my_main.h"
#include "imu.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>

#define LOG_TX_BUFFER_SIZE      128U
#define LOG_TX_TIMEOUT_MS       20U

static char log_tx_buf[LOG_TX_BUFFER_SIZE];
static volatile uint8_t log_tx_busy = 0U;

/**
 * @brief  通过 USART1 串口发送格式化字符串
 * @param  format 格式化字符串
 * @param  ...    可变参数
 * @retval None
 */
void MyMain_PrintLog(const char *format, ...)
{
    char temp_buf[LOG_TX_BUFFER_SIZE];
    va_list args;
    int32_t len;

    if (format == 0)
    {
        return;
    }

    va_start(args, format);
    len = vsnprintf(temp_buf, sizeof(temp_buf), format, args);
    va_end(args);

    if (len <= 0)
    {
        return;
    }

    if (len >= (int32_t)sizeof(temp_buf))
    {
        len = (int32_t)sizeof(temp_buf) - 1;
    }

    if ((log_tx_busy == 0U) && (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY))
    {
        if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)temp_buf, (uint16_t)len) == HAL_OK)
        {
            log_tx_busy = 1U;
        }
        else
        {
            HAL_UART_Transmit(&huart1, (uint8_t *)temp_buf, (uint16_t)len, LOG_TX_TIMEOUT_MS);
        }
    }
    else
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)temp_buf, (uint16_t)len, LOG_TX_TIMEOUT_MS);
    }
}

/**
 * @brief  用户自定义主程序初始化（初始化 IMU 与串口日志等）
 * @retval None
 */
void MyMain_Init(void)
{
    log_tx_busy = 0U;
    Imu_Init();
    LOG_INFO("MyMain application initialized");
}

/**
 * @brief  用户自定义主循环函数（周期更新 IMU 状态并打印姿态数据）
 * @retval None
 */
void MyMain_Loop(void)
{
    Imu_Update();
}

/**
 * @brief  串口 DMA 或空闲中断接收回调处理
 * @param  huart 串口外设句柄
 * @param  size  接收数据长度
 * @retval None
 */
void MyMain_RxCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if ((huart->Instance == USART6) && (size > 0U))
    {
        Imu_ProcessRxData(Imu_GetRxBuffer(), size);
    }

    if (huart->Instance == USART6)
    {
        Imu_StartReceive();
    }
}

/**
 * @brief  串口发送完成中断回调处理
 * @param  huart 串口外设句柄
 * @retval None
 */
void MyMain_TxCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        log_tx_busy = 0U;
    }
}

/**
 * @brief  串口通信错误异常回调处理
 * @param  huart 串口外设句柄
 * @retval None
 */
void MyMain_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        Imu_StartReceive();
    }
}
