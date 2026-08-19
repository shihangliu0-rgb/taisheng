#ifndef AUTO_CHASSIS_H
#define AUTO_CHASSIS_H

#include "stm32h7xx_hal.h"

#include <stdint.h>

typedef enum
{
    AUTO_CHASSIS_IDLE = 0U,
    AUTO_CHASSIS_MANUAL = 1U,
    AUTO_CHASSIS_NAVIGATING = 2U,
    AUTO_CHASSIS_ALIGN_BLOCK = 3U,
    AUTO_CHASSIS_ALIGN_BALL = 4U,
    AUTO_CHASSIS_ARRIVED = 5U,
    AUTO_CHASSIS_FAULT = 6U
} auto_chassis_state_t;

typedef enum
{
    AUTO_CHASSIS_ERROR_NONE = 0U,
    AUTO_CHASSIS_ERROR_POSE_INVALID = 1U,
    AUTO_CHASSIS_ERROR_BLOCK_INVALID = 2U,
    AUTO_CHASSIS_ERROR_BALL_INVALID = 3U,
    AUTO_CHASSIS_ERROR_CHASSIS = 4U,
    AUTO_CHASSIS_ERROR_BAD_TARGET = 5U
} auto_chassis_error_t;

extern volatile auto_chassis_state_t auto_chassis_state;
extern volatile auto_chassis_error_t auto_chassis_error;

void AutoChassis_Init(void);
void AutoChassis_Run(void);

/* Field targets use exactly the coordinate values carried by the 0x11 frame. */
HAL_StatusTypeDef AutoChassis_SetFieldTarget(float field_x_m,
                                             float field_y_m,
                                             float field_yaw);

/* Perception X points forward and Y points left in the robot frame. */
HAL_StatusTypeDef AutoChassis_AlignBlock(float stop_distance_m);
HAL_StatusTypeDef AutoChassis_AlignBall(float stop_distance_m);

void AutoChassis_Stop(void);
auto_chassis_state_t AutoChassis_GetState(void);

#endif
