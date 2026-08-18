#ifndef SIM_DT35_PNP_LINK_H
#define SIM_DT35_PNP_LINK_H

#include <stdint.h>

#define SENSOR_LINK_F_INDEX   0U
#define SENSOR_LINK_L_B_INDEX 1U
#define SENSOR_LINK_COUNT     2U

typedef struct
{
    uint32_t last_rx_ms;
    uint16_t distance_cm;
    uint8_t online;
    uint8_t frame_pending;
} dt35_link_t;

extern volatile dt35_link_t dt35_link[SENSOR_LINK_COUNT];

#endif
