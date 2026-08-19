#ifndef FDCAN_TASK_H
#define FDCAN_TASK_H

#include "rs_bus.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t rs_host_id;
} fdcan_config_t;

HAL_StatusTypeDef Fdcan_Init(const fdcan_config_t *config);
void Fdcan_Run1ms(void);
void Fdcan_Stop(void);
rs_bus_t *Fdcan_GetRsBus(void);
void Fdcan_HandleRxIsr(FDCAN_HandleTypeDef *hfdcan,
                       uint32_t interrupt_flags);
void Fdcan_HandleErrorIsr(FDCAN_HandleTypeDef *hfdcan,
                          uint32_t interrupt_flags);
bool Fdcan_BusOff(void);

#endif
