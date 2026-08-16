#ifndef RS_BUS_H
#define RS_BUS_H

#include "fdcan.h"

#include <stdbool.h>
#include <stdint.h>

/* 队列长度属于中断与任务共享环形缓冲区布局。 */
#define RS_BUS_RX_QUEUE_SIZE 32U

typedef void (*rs_bus_rx_handler_t)(void *context, uint32_t id,
                                    const uint8_t data[8], uint32_t tick_ms);

typedef struct
{
    uint32_t id;
    uint32_t tick_ms;
    uint8_t data[8];
} rs_bus_frame_t;

typedef struct
{
    FDCAN_HandleTypeDef *device;
    rs_bus_rx_handler_t rx_handler;
    void *rx_context;
    rs_bus_frame_t rx_queue[RS_BUS_RX_QUEUE_SIZE];
    volatile uint8_t rx_head; /* ISR 写入。 */
    volatile uint8_t rx_tail; /* 任务读取。 */
    volatile bool bus_off;
    uint8_t host_id;
    bool ready;
} rs_bus_t;

/** @brief 初始化 RS00 使用的扩展帧 CAN 总线。 */
HAL_StatusTypeDef RsBus_Init(rs_bus_t *bus, FDCAN_HandleTypeDef *device,
                             uint8_t host_id);
/** @brief 停止 RS00 CAN 总线。 */
void RsBus_Stop(rs_bus_t *bus);
/** @brief 从 Bus-Off 状态重启控制器，保留应用层控制状态。 */
HAL_StatusTypeDef RsBus_Recover(rs_bus_t *bus);
/** @brief 设置主循环中的唯一接收处理函数。 */
HAL_StatusTypeDef RsBus_SetHandler(rs_bus_t *bus,
                                   rs_bus_rx_handler_t handler,
                                   void *context);
/** @brief 发送一帧 8 字节扩展 CAN 数据。 */
HAL_StatusTypeDef RsBus_Send(rs_bus_t *bus, uint32_t id,
                             const uint8_t data[8]);
/** @brief 在 CAN 中断中取帧入固定接收队列。 */
void RsBus_HandleRxIsr(rs_bus_t *bus, uint32_t interrupt_flags);
/** @brief 在 CAN 中断中记录总线错误。 */
void RsBus_HandleErrorIsr(rs_bus_t *bus, uint32_t interrupt_flags);
/** @brief 在主循环中解析接收队列。 */
void RsBus_ProcessRx(rs_bus_t *bus);
/** @brief 查询总线当前是否处于 Bus-Off 状态。 */
bool RsBus_BusOff(const rs_bus_t *bus);

#endif
