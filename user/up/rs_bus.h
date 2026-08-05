#ifndef RS_BUS_H
#define RS_BUS_H

#include "fdcan.h"

#include <stdbool.h>
#include <stdint.h>

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
    volatile uint8_t rx_head; // ISR 写入
    volatile uint8_t rx_tail; // 任务读取
    uint8_t host_id;
    bool ready;
} rs_bus_t;

HAL_StatusTypeDef RsBus_Init(rs_bus_t *bus, FDCAN_HandleTypeDef *device,
                             uint8_t host_id);
void RsBus_Stop(rs_bus_t *bus);
HAL_StatusTypeDef RsBus_SetHandler(rs_bus_t *bus,
                                   rs_bus_rx_handler_t handler,
                                   void *context);
HAL_StatusTypeDef RsBus_Send(rs_bus_t *bus, uint32_t id,
                             const uint8_t data[8]);
void RsBus_HandleRxIsr(rs_bus_t *bus, uint32_t interrupt_flags);
void RsBus_ProcessRx(rs_bus_t *bus);

#endif
