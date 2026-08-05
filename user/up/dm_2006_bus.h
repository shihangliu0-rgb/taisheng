#ifndef DM_2006_BUS_H
#define DM_2006_BUS_H

#include "fdcan.h"

#include <stdbool.h>
#include <stdint.h>

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
    uint32_t rx_count;
    uint32_t rx_overflow_count;
    uint32_t rx_error_count;
    uint32_t tx_count;
    uint32_t tx_busy_count;
    uint32_t tx_error_count;
    uint32_t error_warning_count;
    uint32_t error_passive_count;
    uint32_t bus_off_count;
} std_can_diag_t;

typedef struct
{
    FDCAN_HandleTypeDef *device;
    std_can_handler_t handlers[STD_CAN_MAX_HANDLERS];
    std_can_frame_t rx_queue[STD_CAN_RX_QUEUE_SIZE];
    volatile uint8_t rx_head; // ISR 写入
    volatile uint8_t rx_tail; // 任务读取
    uint8_t handler_count;
    bool ready;
    std_can_diag_t diag;
} std_can_t;

HAL_StatusTypeDef StdCan_Init(std_can_t *bus, FDCAN_HandleTypeDef *device,
                              const uint16_t *rx_ids, uint8_t rx_id_count);
void StdCan_Stop(std_can_t *bus);
HAL_StatusTypeDef StdCan_AddHandler(std_can_t *bus,
                                    std_can_rx_handler_t callback,
                                    void *context);
HAL_StatusTypeDef StdCan_Send(std_can_t *bus, uint16_t id,
                              const uint8_t data[8]);
void StdCan_HandleRxIsr(std_can_t *bus, uint32_t interrupt_flags);
void StdCan_HandleErrorIsr(std_can_t *bus, uint32_t interrupt_flags);
void StdCan_ProcessRx(std_can_t *bus);
void StdCan_GetDiag(const std_can_t *bus, std_can_diag_t *diag);

#endif
