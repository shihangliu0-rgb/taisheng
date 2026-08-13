#ifndef UP_MAIN_H
#define UP_MAIN_H

#include "c610_2006.h"
#include "dm_app.h"
#include "rs_app.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float rs_l_deg;
    float rs_r_deg;
    float dm_l_deg;
    float dm_r_deg;
} up_motor_angles_t;

/* 调试时可直接观察的抬升机构状态。 */
extern up_motor_angles_t up_target_angles;
extern up_motor_angles_t up_command_angles;
extern HAL_StatusTypeDef up_last_result;
extern bool up_curve_running;

/**
 * @brief 初始化总线和三类电机，并启动回零动作
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_Init(void);

/**
 * @brief 使能 RS/DM 并运动到已保存的零点，不改写机械零点
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_HomeMotors(void);

/**
 * @brief 使用五次多项式同时规划两台 RS 和两台 DM 的输出轴目标角度
 * @param rs_l_deg RS 左输出轴目标角度(deg)
 * @param rs_r_deg RS 右输出轴目标角度(deg)
 * @param dm_l_deg DM 左输出轴目标角度(deg)
 * @param dm_r_deg DM 右输出轴目标角度(deg)
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_SetMotorPos(float rs_l_deg, float rs_r_deg,
                                 float dm_l_deg, float dm_r_deg);

/**
 * @brief 查询四台 RS/DM 电机是否结束规划并到达目标位置
 * @retval true 已到位，false 仍在运动、电机离线或存在故障
 */
bool Up_MotorMoveDone(void);

/**
 * @brief 设置两台 RS00 的输出轴目标角度
 * @param angle_deg 两台 RS00 的输出轴目标角度(deg)
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_SetRsPos(float angle_deg);

/**
 * @brief 设置两台达妙电机的输出轴目标角度
 * @param angle_deg 两台达妙电机的输出轴目标角度(deg)
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_SetDmPos(float angle_deg);

/**
 * @brief 设置并使能一台 M2006 的位置控制
 * @param id 电机 ID
 * @param position_deg 输出轴目标角度(deg)
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_SetM2006Pos(uint8_t id, float position_deg);

/**
 * @brief 两台 M2006 输出轴分别相对当前位置转动指定角度
 * @param offset_deg 输出轴相对转动角度(deg)
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_MoveM2006(float offset_deg);

/**
 * @brief 执行抬升机构 1 ms 周期任务
 * @retval None
 */
void Up_Run1ms(void);

/**
 * @brief 停止全部抬升电机并关闭自动恢复
 * @retval None
 */
void Up_StopAll(void);

/**
 * @brief 读取一台 RS00 的应用层状态
 * @param id 电机 ID
 * @param status 状态输出
 * @retval true 读取成功，false 参数错误或模块未初始化
 */
bool Up_GetRsStatus(uint8_t id, rs_app_status_t *status);

/**
 * @brief 读取一台 DM 电机的应用层状态
 * @param id 电机 ID
 * @param status 状态输出
 * @retval true 读取成功，false 参数错误或模块未初始化
 */
bool Up_GetDmStatus(uint16_t id, dm_app_status_t *status);

/**
 * @brief 读取一台 M2006 的应用层状态
 * @param id 电机 ID
 * @param status 状态输出
 * @retval true 读取成功，false 参数错误或模块未初始化
 */
bool Up_GetM2006Status(uint8_t id, m2006_status_t *status);

#endif
