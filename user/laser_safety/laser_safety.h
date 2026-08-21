#ifndef LASER_SAFETY_H
#define LASER_SAFETY_H

#include "chassis_main.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 激光测距安全限速模块。
 *
 * 只有前方(DT35 F)和左侧(DT35 L)两个激光测距，用于保障底盘不撞墙：
 *  - 前方距离只约束向前速度 vy(正方向向前)，后退不受影响；
 *  - 左侧距离只约束向左平移 vx(负方向向左)，向右平移和旋转不受影响。
 *
 * 预测模型(刹车距离)：给定剩余距离 d - STOP_DISTANCE 内必须能刹停，
 * 允许速度上限 v_max = sqrt(2 * a * (d - STOP_DISTANCE))。
 * 距离 <= STOP_DISTANCE 时 v_max = 0，对应方向完全禁止。
 */

/* 硬停止距离(cm)：前方/左侧距离小于等于该值时对应方向速度强制为 0。 */
#define LASER_SAFETY_STOP_DISTANCE_CM   10.0f
/* 预测模型使用的最大减速度(mm/s^2)，默认 2 m/s^2。调小更保守，调大放得更开。 */
#define LASER_SAFETY_BRAKE_ACCEL_MM_S2  2000.0f
/* 允许速度达到该值(mm/s)时视为无限制，防止误压开放场地的全速。 */
#define LASER_SAFETY_FULL_SPEED_MM_S    2000
/* 周期纠偏时重发命令的超时(ms)。 */
#define LASER_SAFETY_ENFORCE_TIMEOUT_MS 100U

typedef struct
{
    int16_t vx;
    int16_t vy;
    int16_t z;
    bool front_limited;   /* vy 被压低 */
    bool front_blocked;   /* 前方距离 <= 硬停距离，vy 被压到 0 */
    bool left_limited;    /* 向左速度被压低 */
    bool left_blocked;    /* 左侧距离 <= 硬停距离，vx 被压到 0 */
    bool front_offline;   /* 前方传感器离线，本次未限制 */
    bool left_offline;    /* 左侧传感器离线，本次未限制 */
} laser_safety_clamp_t;

/**
 * @brief 对速度命令做激光限速后转发给底盘（替代直接调用
 *        Chassis_RequestVelocity，参数完全相同，z 不做限制）。
 */
HAL_StatusTypeDef LaserSafety_RequestVelocity(chassis_cmd_source_t source,
                                              int16_t vx, int16_t vy,
                                              int16_t z, uint32_t timeout_ms);

/**
 * @brief 对速度命令做激光限速后转发给底盘（替代直接调用
 *        Chassis_SetVelocity）。
 */
HAL_StatusTypeDef LaserSafety_SetVelocity(int16_t vx, int16_t vy, int16_t z);

/**
 * @brief 纯限速计算，不访问底盘：读前方/左侧激光距离，输出钳制后的命令。
 *        可在命令源上下文(包括中断)中调用。
 */
void LaserSafety_ClampCommand(int16_t vx, int16_t vy, int16_t z,
                              laser_safety_clamp_t *out);

/**
 * @brief 周期纠偏：底盘当前生效目标超过当前限速时，用限速值在活动命令源上
 *        重发一次短超时命令。距离持续变小时让机器人边靠近边减速，
 *        距离 <= 硬停距离时目标被压到 0，保证不会越过停止线。
 *        每 1 ms 调用一次(底盘任务里 Chassis_Run1ms 之后)。
 */
void LaserSafety_Run1ms(void);

/**
 * @brief 读取最近一次限速结果，用于调试观察。
 */
bool LaserSafety_GetStatus(laser_safety_clamp_t *status);

#endif
