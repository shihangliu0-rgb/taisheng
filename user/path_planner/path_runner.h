/**
 ******************************************************************************
 * @file    path_runner.h
 * @brief   在线跟踪总控(对应 profile_runner.py,运行在 STM32 上)
 *
 * 对接本仓库已有模块(数据全部来自真实外设,无仿真):
 *   上位机位姿  0x11 位置帧 -> user/pc_link       (PcLink_GetPosition)
 *   IMU 陀螺/yaw           -> user/imu           (ImuMain_GetData)
 *   前/左激光(DT35)        -> user/com_link      (dt35_link[].distance_cm,
 *                                 由 UART9 的 DT35 帧解析得到)
 *   底盘执行               -> user/chassis_vesc  (Chassis_SetVelocity)
 *
 * 控制周期 PATH_CONTROL_PERIOD_MS(默认 5ms),放在 commTask 里以 1ms
 * 周期调用 PathRunner_Run(),内部自行分频。
 *
 * 安全行为:
 *   - 上位机位姿丢失 >500ms    -> 主动 STOP
 *   - IMU 离线                 -> 主动 STOP
 *   - 前激光 <12cm             -> 强制 vx=0(仅保留航向锁定)
 *   - 前激光离线(可配置)      -> 主动 STOP
 *   - 全程超时 PATH_MAX_RUN_MS -> 主动 STOP
 ******************************************************************************
 */
#ifndef PATH_RUNNER_H
#define PATH_RUNNER_H

#include "path_types.h"

#if PATH_DEBUG && defined(PATH_DEBUG_UART_HANDLE)
#include "stm32h7xx_hal.h"
#endif

/**
 * @brief 初始化规划器(进入 INIT 状态;同时关闭 IMU 模块自带航向保持,
 *        由本模块的 yaw-lock 接管底盘 z 指令)
 */
void PathRunner_Init(void);

/**
 * @brief 主循环调用(1ms 周期,内部按控制周期分频)
 */
void PathRunner_Run(void);

/**
 * @brief 读取调试信息(每 400ms 采样一次即可)
 */
void PathRunner_GetDebug(path_debug_t *debug);

/**
 * @brief 读取离线生成的参考轨迹(构建完成后有效;用于 CSV 导出/上位机可视化)
 * @param count 输出:轨迹点数
 * @retval 轨迹数组指针;未构建完成返回 NULL
 */
const path_point_t *PathRunner_GetTrajectory(uint16_t *count);

#if PATH_DEBUG && defined(PATH_DEBUG_UART_HANDLE)
/**
 * @brief 每 400ms 向调试串口输出一行运行状态(调试用)
 */
void PathRunner_DebugDump(UART_HandleTypeDef *uart);
#endif

#endif /* PATH_RUNNER_H */
