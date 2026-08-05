/**
 * @file    imu_main.c
 * @brief   IMU 在 723 主板上的任务接入层实现
 * @note    按 723 工程风格(Init + Run1ms + 回调)，内部调用 user/imu 驱动与算法；
 *          板级 main/日志/串口回调(F405 my_main)已剥离。
 */

#include "imu_main.h"
#include "usart.h"          /* 引用 H7 既有串口句柄声明，仅使用、不修改 usart.c */

/* USART3(PD8/PD9) 由用户在 CubeMX 开启后，huart3 的定义在 usart.c 中生成。
 * 在那之前用 extern 前向声明保证可编译；与 usart.h 的声明兼容(可重复 extern)。 */
extern UART_HandleTypeDef huart3;

HAL_StatusTypeDef ImuMain_Init(void)
{
    Imu_AttachUart(&IMU_UART_HANDLE);   /* 绑定 IMU 串口(由 imu_main.h 的宏选择，不修改 usart.c) */
    Imu_Init();                         /* 算法初始化 + 启动 DMA 接收 + 零漂校准调度 */
    return HAL_OK;
}

void ImuMain_Run1ms(void)
{
    Imu_Update();                       /* 驱动调度器：IMU 指令序列 / 延时回调 */
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
