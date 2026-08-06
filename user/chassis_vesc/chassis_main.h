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

/* 底盘状态机 */
typedef enum
{
    CHASSIS_INIT,         /* 底盘初始化状态 */
    CHASSIS_STOP,         /* 底盘停止，所有电机输出为 0 */
    CHASSIS_MANUAL,       /* 底盘手动遥控模式，遥控器直接控制底盘运动 */
    CHASSIS_MOVE,         /* 底盘普通运动模式，速度指令直接输出 */
    CHASSIS_PATH_FOLLOW,  /* 底盘路径跟踪模式(未实现) */
    CHASSIS_ALIGN,        /* 底盘对位对齐模式(未实现) */
    CHASSIS_OBSTACLE,     /* 底盘避障状态(未实现) */
    CHASSIS_SUPPORT,      /* 底盘支撑模式(未实现) */
    CHASSIS_ERROR,        /* 底盘故障错误状态，异常保护 */
} Chassis_State_t;

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

/* ====== 状态机 ====== */
/* 设置底盘状态(状态转换入口，由上层/遥控/上位机调用) */
void Chassis_SetState(Chassis_State_t state);
/* 获取当前底盘状态 */
Chassis_State_t Chassis_GetState(void);

/* 普通停止：速度目标清零，【保持当前状态】(如路径规划暂停后可恢复) */
void Chassis_Stop(void);
/* 安全/急停：立即停转所有电机 + 切到 CHASSIS_STOP(需显式切回才能恢复)。
 * 触发场景：急停、CAN/电机掉线、看门狗超时等 */
void Chassis_EmergencyStop(void);

#endif
