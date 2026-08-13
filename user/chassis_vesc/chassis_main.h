#ifndef CHASSIS_MAIN_H
#define CHASSIS_MAIN_H

#include "vesc_motor.h"

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

extern volatile int16_t chassis_target_vx;
extern volatile int16_t chassis_target_vy;
extern volatile int16_t chassis_target_z;

/**
 * @brief 初始化 FDCAN1 和四台底盘 VESC
 * @retval HAL 状态
 */
HAL_StatusTypeDef Chassis_Init(void);

/**
 * @brief 解算并设置四个车轮的目标转速
 * @param vx X 方向目标速度，正方向向右
 * @param vy Y 方向目标速度，正方向向前
 * @param z Z 轴目标旋转速度，正方向为逆时针
 * @retval HAL 状态
 */
HAL_StatusTypeDef Chassis_SetVelocity(int16_t vx, int16_t vy, int16_t z);

void Chassis_Run1ms(void);
void Chassis_StopAll(void);
bool Chassis_GetStatus(chassis_wheel_t wheel,
                       vesc_motor_status_t *status);

#endif
