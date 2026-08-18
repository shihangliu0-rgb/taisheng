#ifndef TEST_CHASSIS_MAIN_H
#define TEST_CHASSIS_MAIN_H

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

HAL_StatusTypeDef Chassis_SetVelocity(int16_t vx, int16_t vy, int16_t z);
void Chassis_StopAll(void);

#endif
