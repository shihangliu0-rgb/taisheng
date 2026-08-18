#ifndef SIM_IMU_MAIN_H
#define SIM_IMU_MAIN_H

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
    float gyro_z_deg_s;
    float gyro_bias_deg_s;
    imu_state_t state;
    uint32_t last_rx_ms;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t rx_overflow_count;
    uint32_t uart_error_count;
    float target_yaw_deg;
    float yaw_error_deg;
    int16_t omega_output;
    bool yaw_valid;
    bool gyro_valid;
    bool online;
    bool yaw_hold_enabled;
    bool yaw_hold_active;
} imu_data_t;

bool ImuMain_GetData(imu_data_t *data);
void ImuMain_EnableYawHold(bool enabled);
HAL_StatusTypeDef ImuMain_SetTargetYaw(float target_yaw_deg);

#endif
