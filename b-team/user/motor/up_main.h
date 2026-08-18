#ifndef UP_MAIN_H
#define UP_MAIN_H

#include "c610_2006.h"
#include "dm_app.h"
#include "rs_app.h"

#include <stdbool.h>
#include <stdint.h>

/* 第一次标零后沿负方向运动的二次标零角度，后续动作共用。 */
#define UP_SECOND_ZERO_OFFSET_DEG 62.0f

typedef struct
{
    float rs_l_deg;
    float rs_r_deg;
    float dm_l_deg;
    float dm_r_deg;
} up_motor_angles_t;

typedef enum
{
    UP_STATE_INIT,
    UP_STATE_FIRST_ZERO,
    UP_STATE_MOVE_SECOND_ZERO,
    UP_STATE_SECOND_ZERO,
    UP_STATE_READY,
    UP_STATE_STOPPED,
    UP_STATE_ERROR
} up_state_t;

typedef enum
{
    UP_ZERO_DISABLE_RS,
    UP_ZERO_DISABLE_DM,
    UP_ZERO_WAIT_DISABLED,
    UP_ZERO_CLEAR_RS_FAULT,
    UP_ZERO_SET_RS,
    UP_ZERO_SET_DM,
    UP_ZERO_ENABLE_DM,
    UP_ZERO_ENABLE_RS,
    UP_ZERO_WAIT_READY
} up_zero_step_t;

/* 调试时可直接观察的抬升机构状态。 */
extern up_motor_angles_t up_target_angles;
extern up_motor_angles_t up_command_angles;
extern HAL_StatusTypeDef up_last_result;
extern bool up_curve_running;
extern up_state_t up_state;
extern up_zero_step_t up_zero_step;
extern float up_rs_feedforward_nm;
extern float up_dm_feedforward_nm;

/**
 * @brief 初始化总线并启动四台电机的二次标零流程
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_Init(void);

/** @brief 查询四台电机是否完成二次标零、停在零度待机位并保持使能。 */
bool Up_IsReady(void);
bool Up_IsM2006Ready(void);

/**
 * @brief 重新执行四台电机的二次标零流程
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
 * @brief 设置两台 RS00 和两台达妙电机的输出轴前馈转矩
 * @param rs_torque_nm 两台 RS00 的前馈转矩(Nm)
 * @param dm_torque_nm 两台达妙电机的前馈转矩(Nm)
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_SetFeedforward(float rs_torque_nm, float dm_torque_nm);

/**
 * @brief 读取四台抬升电机的输出轴角度
 * @param angles 四台电机角度
 * @retval true 四台电机均在线、使能且无故障
 */
bool Up_GetMotorAngles(up_motor_angles_t *angles);

/**
 * @brief 查询四台 RS/DM 电机是否结束规划并到达目标位置
 * @retval true 已到位，false 仍在运动、电机离线或存在故障
 */
bool Up_MotorMoveDone(void);

/**
 * @brief 查询两台 RS00 是否结束规划并到达目标位置
 * @retval true 已到位，false 仍在运动、电机离线或存在故障
 */
bool Up_RsMoveDone(void);

/**
 * @brief 查询两台达妙电机是否结束规划并到达目标位置
 * @retval true 已到位，false 仍在运动、电机离线或存在故障
 */
bool Up_DmMoveDone(void);

/**
 * @brief 设置两台 RS00 的输出轴目标角度
 * @param left_deg 左侧 RS00 输出轴目标角度(deg)
 * @param right_deg 右侧 RS00 输出轴目标角度(deg)
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_SetRsPos(float left_deg, float right_deg);

/**
 * @brief 设置两台达妙电机的输出轴目标角度
 * @param left_deg 左侧达妙输出轴目标角度(deg)
 * @param right_deg 右侧达妙输出轴目标角度(deg)
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_SetDmPos(float left_deg, float right_deg);

/** @brief 将两台 RS00 的当前位置重新设为输出轴零点。 */
HAL_StatusTypeDef Up_RebaseRsPosition(void);

/** @brief 将两台达妙电机的当前位置重新设为输出轴零点。 */
HAL_StatusTypeDef Up_RebaseDmPosition(void);

/** @brief 查询位置重置状态，执行中返回 HAL_BUSY。 */
HAL_StatusTypeDef Up_GetPositionRebaseStatus(void);

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
 * @brief 关闭两台 M2006 的位置控制并输出零电流
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_CoastM2006(void);

/**
 * @brief 执行抬升机构 1 ms 周期任务
 * @retval None
 */
void Up_Run1ms(void);

/**
 * @brief 失能全部抬升电机，调用 Up_HomeMotors 后才能恢复
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
