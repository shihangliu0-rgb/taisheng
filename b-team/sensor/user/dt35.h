#ifndef DT35_H
#define DT35_H

#include "stm32f1xx_hal.h"

/* 两个距离传感器的固定 I2C 地址。 */
#define DT35_ADDR_F 0x40U
#define DT35_ADDR_L 0x41U
/* Keep numeric aliases for the existing read API. */
#define DT35_ADDR_40 DT35_ADDR_F
#define DT35_ADDR_41 DT35_ADDR_L

/*
 * 示教窗口（0–10 V 线性映射，整 cm 截断）：
 * 前光 0x40：5–140 cm；左光 0x41：5–240 cm。
 * 必须与主控 path.c 的 PATH_FRONT_LASER_MAX_CM / PATH_LEFT_LASER_MAX_CM
 * 一致，并且 DT35 本体模拟量示教窗口也要设成同样的近/远点，
 * 否则空地会被报成 20 cm，自动起步冲一下就被前光硬停。
 */
#define DT35_DIST_NEAR_CM        5U
#define DT35_DIST_FRONT_FAR_CM   140U
#define DT35_DIST_LEFT_FAR_CM    240U

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
