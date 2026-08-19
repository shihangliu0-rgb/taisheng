#ifndef DT35_PNP_LINK_H
#define DT35_PNP_LINK_H

#include "stm32h7xx_hal.h"

/* DT35 使用 F/L 命名，PNP 使用 F/B 命名；两者地址保持一致。 */
#define SENSOR_LINK_ADDR_F     0x40U  /* 前方传感器地址 */
#define SENSOR_LINK_ADDR_L     0x41U  /* 左侧传感器地址 */
#define PNP_LINK_ADDR_F        SENSOR_LINK_ADDR_F
#define PNP_LINK_ADDR_B        SENSOR_LINK_ADDR_L
#define SENSOR_LINK_F_INDEX    0U     /* DT35_F / PNP_F 数组下标 */
#define SENSOR_LINK_L_B_INDEX  1U     /* DT35_L / PNP_B 数组下标 */
#define SENSOR_LINK_COUNT      2U     /* 每类传感器数量 */

/*
 * DT35 frame: AA, address, distance_cm low, high, XOR checksum.
 * PNP frame:  AB, address, trigger low, 0, XOR checksum.
 */

typedef struct
{
    uint32_t last_rx_ms;
    uint16_t distance_cm;
    uint8_t online;
    uint8_t frame_pending;
} dt35_link_t;

typedef struct
{
    uint32_t last_rx_ms;
    uint8_t trigger;
    uint8_t online;
    uint8_t frame_pending;
} pnp_link_t;

extern volatile dt35_link_t dt35_link[SENSOR_LINK_COUNT];
extern volatile pnp_link_t pnp_link[SENSOR_LINK_COUNT];

/**
 * @brief 初始化 DT35 和 PNP 串口链路
 * @param uart 传感器串口
 * @retval HAL 状态
 */
HAL_StatusTypeDef DT35PnpLink_Init(UART_HandleTypeDef *uart);

/**
 * @brief 处理串口重启和传感器离线状态
 * @param None
 * @retval None
 */
void DT35PnpLink_Run(void);

/**
 * @brief 将有变化的 DT35 和 PNP 数据发送到上位机
 * @param uart 上位机串口
 * @retval None
 */
void DT35PnpLink_Send(UART_HandleTypeDef *uart);

/**
 * @brief 处理传感器串口单字节接收完成
 * @param uart 触发回调的串口
 * @retval None
 */
void DT35PnpLink_RxCplt(UART_HandleTypeDef *uart);

/**
 * @brief 记录传感器串口错误并请求重启接收
 * @param uart 触发回调的串口
 * @retval None
 */
void DT35PnpLink_Error(UART_HandleTypeDef *uart);

#endif
