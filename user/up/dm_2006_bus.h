#ifndef DM_2006_BUS_H
#define DM_2006_BUS_H

#include "fdcan.h"

#include <stdbool.h>
#include <stdint.h>

/* 队列和回调容量属于编译期内存布局参数。 */
#define STD_CAN_RX_QUEUE_SIZE 32U
#define STD_CAN_MAX_HANDLERS  2U

typedef void (*std_can_rx_handler_t)(void *context, uint16_t id,
                                     const uint8_t data[8],
                                     uint32_t tick_ms);

typedef struct
{
    uint16_t id;
    uint32_t tick_ms;
    uint8_t data[8];
} std_can_frame_t;

typedef struct
{
    std_can_rx_handler_t callback;
    void *context;
} std_can_handler_t;

typedef struct
{
    FDCAN_HandleTypeDef *device;
    std_can_handler_t handlers[STD_CAN_MAX_HANDLERS];
    std_can_frame_t rx_queue[STD_CAN_RX_QUEUE_SIZE];
    volatile uint8_t rx_head; /* ISR 写入。 */
    volatile uint8_t rx_tail; /* 任务读取。 */
    volatile bool bus_off;
    uint8_t handler_count;
    bool ready;
} std_can_t;

/** @brief 初始化标准 CAN 总线、过滤器和接收中断。 */
HAL_StatusTypeDef StdCan_Init(std_can_t *bus, FDCAN_HandleTypeDef *device,
                              const uint16_t *rx_ids, uint8_t rx_id_count);
/** @brief 停止标准 CAN 总线。 */
void StdCan_Stop(std_can_t *bus);
/** @brief 注册一个主循环中执行的接收处理函数。 */
HAL_StatusTypeDef StdCan_AddHandler(std_can_t *bus,
                                    std_can_rx_handler_t callback,
                                    void *context);
/** @brief 发送一帧 8 字节标准 CAN 数据。 */
HAL_StatusTypeDef StdCan_Send(std_can_t *bus, uint16_t id,
                              const uint8_t data[8]);
/** @brief 在 CAN 中断中取帧入固定接收队列。 */
void StdCan_HandleRxIsr(std_can_t *bus, uint32_t interrupt_flags);
/** @brief 在 CAN 中断中记录总线错误。 */
void StdCan_HandleErrorIsr(std_can_t *bus, uint32_t interrupt_flags);
/** @brief 在主循环中解析接收队列。 */
void StdCan_ProcessRx(std_can_t *bus);
/** @brief 查询总线是否进入闭锁状态。 */
bool StdCan_BusOff(const std_can_t *bus);

#endif
