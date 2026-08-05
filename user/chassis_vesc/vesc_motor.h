#ifndef VESC_MOTOR_H
#define VESC_MOTOR_H

#include "vesc_can.h"

#include <stdbool.h>
#include <stdint.h>

#define VESC_STATUS_TIMEOUT_MS 200U

typedef struct
{
    uint8_t id;
    uint8_t pole_pairs;
    int32_t min_rpm;
    int32_t max_rpm;
    float brake_current_a;
} vesc_motor_config_t;

typedef struct
{
    int32_t target_rpm;
    int32_t actual_rpm;
    float current_a;
    float duty;
    bool online;
    uint32_t last_rx_ms;
} vesc_motor_status_t;

typedef struct
{
    vesc_can_t *bus;
    vesc_motor_config_t config;
    vesc_motor_status_t status;
} vesc_motor_t;

HAL_StatusTypeDef VescMotor_Init(vesc_motor_t *motor, vesc_can_t *bus,
                                 const vesc_motor_config_t *config);
HAL_StatusTypeDef VescMotor_SetRpm(vesc_motor_t *motor,
                                   int32_t target_rpm);
HAL_StatusTypeDef VescMotor_SendRpm(vesc_motor_t *motor);
HAL_StatusTypeDef VescMotor_Brake(vesc_motor_t *motor);
bool VescMotor_Parse(vesc_motor_t *motor, const vesc_can_msg_t *msg,
                     uint32_t now_ms);
void VescMotor_Update(vesc_motor_t *motor, uint32_t now_ms);
bool VescMotor_GetStatus(const vesc_motor_t *motor,
                         vesc_motor_status_t *status);

#endif
