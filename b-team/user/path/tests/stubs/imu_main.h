#ifndef TEST_IMU_MAIN_H
#define TEST_IMU_MAIN_H

#include "chassis_main.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    IMU_STATE_UNINITIALIZED,
    IMU_STATE_CALIBRATING,
    IMU_STATE_CONFIGURING,
    IMU_STATE_BIAS_SAMPLING,
    IMU_STATE_READY,
    IMU_STATE_ERROR
} imu_state_t;

typedef struct
{
    float yaw_deg;
    imu_state_t state;
    float target_yaw_deg;
    bool yaw_valid;
    bool online;
    bool yaw_hold_enabled;
} imu_data_t;

bool ImuMain_GetData(imu_data_t *data);
void ImuMain_EnableYawHold(bool enabled);
HAL_StatusTypeDef ImuMain_SetTargetYaw(float target_yaw_deg);

#endif
