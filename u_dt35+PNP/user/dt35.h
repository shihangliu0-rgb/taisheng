#ifndef DT35_H
#define DT35_H

#include "stm32f1xx_hal.h"

/* 两个距离传感器的固定 I2C 地址。 */
#define DT35_ADDR_F 0x40U
#define DT35_ADDR_L 0x41U
/* Keep numeric aliases for the existing read API. */
#define DT35_ADDR_40 DT35_ADDR_F
#define DT35_ADDR_41 DT35_ADDR_L

typedef struct
{
    uint16_t raw;
    uint16_t voltage_mv;
    uint16_t distance_cm;
} dt35_data_t;

HAL_StatusTypeDef DT35_Init(I2C_HandleTypeDef *i2c);
HAL_StatusTypeDef DT35_Read_41(dt35_data_t *data);
HAL_StatusTypeDef DT35_Read_40(dt35_data_t *data);

#endif
