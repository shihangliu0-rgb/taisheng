#ifndef TEST_CHASSIS_MAIN_H
#define TEST_CHASSIS_MAIN_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CHASSIS_WHEEL_LF,
    CHASSIS_WHEEL_RF,
    CHASSIS_WHEEL_LR,
    CHASSIS_WHEEL_RR,
    CHASSIS_WHEEL_COUNT
} chassis_wheel_t;

typedef struct
{
    int32_t target_rpm;
    int32_t actual_rpm;
    float current_a;
    float duty;
    bool online;
    uint32_t last_rx_ms;
} vesc_motor_status_t;

bool Chassis_GetStatus(chassis_wheel_t wheel,
                       vesc_motor_status_t *status);

#endif
