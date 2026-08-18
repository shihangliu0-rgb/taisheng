#ifndef SIM_CHASSIS_MAIN_H
#define SIM_CHASSIS_MAIN_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR = 1
} HAL_StatusTypeDef;

#define __DMB() do { } while (0)
#define __get_PRIMASK() 0U
#define __disable_irq() do { } while (0)
#define __enable_irq() do { } while (0)

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

HAL_StatusTypeDef Chassis_SetVelocity(int16_t vx, int16_t vy, int16_t z);
void Chassis_StopAll(void);
bool Chassis_GetStatus(chassis_wheel_t wheel, vesc_motor_status_t *status);

#endif
