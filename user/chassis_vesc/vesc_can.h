#ifndef VESC_CAN_H
#define VESC_CAN_H

#include "fdcan.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    bool ready;
} vesc_can_t;

typedef struct
{
    uint8_t motor_id;
    uint8_t packet_id;
    uint8_t length;
    uint8_t data[8];
} vesc_can_msg_t;

HAL_StatusTypeDef VescCan_Init(vesc_can_t *bus,
                               FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef VescCan_Send(vesc_can_t *bus, uint8_t motor_id,
                               uint8_t packet_id, const uint8_t *data,
                               uint8_t length);
HAL_StatusTypeDef VescCan_Read(vesc_can_t *bus, vesc_can_msg_t *msg);
void VescCan_Recover(vesc_can_t *bus);
void VescCan_Stop(vesc_can_t *bus);

#endif
