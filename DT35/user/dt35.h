#ifndef DT35_H
#define DT35_H

#include "stm32f1xx_hal.h"

/* 两个距离传感器的固定 I2C 地址。 */
#define DT35_ADDR_41 0x41U
#define DT35_ADDR_40 0x40U

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
