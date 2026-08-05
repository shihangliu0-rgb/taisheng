#ifndef FDCAN_TASK_H
#define FDCAN_TASK_H

#include "dm_2006_bus.h"
#include "rs_bus.h"

#include <stdint.h>

typedef struct
{
    uint8_t rs_host_id;
    const uint16_t *std_rx_ids;
    uint8_t std_rx_id_count;
} fdcan_config_t;

HAL_StatusTypeDef Fdcan_Init(const fdcan_config_t *config);
void Fdcan_Run1ms(void);
void Fdcan_Stop(void);
rs_bus_t *Fdcan_GetRsBus(void);
std_can_t *Fdcan_GetStdBus(void);
void Fdcan_HandleRxIsr(FDCAN_HandleTypeDef *hfdcan,
                       uint32_t interrupt_flags);
void Fdcan_HandleErrorIsr(FDCAN_HandleTypeDef *hfdcan,
                          uint32_t interrupt_flags);

#endif
