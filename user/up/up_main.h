#ifndef UP_MAIN_H
#define UP_MAIN_H

#include "c610_2006.h"
#include "dm_app.h"
#include "rs_app.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化总线和三类电机，RS/DM 自动运动到已保存的零点
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_Init(void);

/**
 * @brief 使能 RS/DM 并运动到已保存的零点，不改写机械零点
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_HomeMotors(void);

/**
 * @brief 使用五次多项式同时规划两台 RS 和两台 DM 的目标角度
 * @param rs_l_deg RS 左电机目标角度(deg)
 * @param rs_f_deg RS 前电机目标角度(deg)
 * @param dm_l_deg DM 左电机目标角度(deg)
 * @param dm_f_deg DM 前电机目标角度(deg)
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_SetMotorPos(float rs_l_deg, float rs_f_deg,
                                 float dm_l_deg, float dm_f_deg);

/**
 * @brief 设置并使能一台 M2006 的位置控制
 * @param id 电机 ID
 * @param position_deg 输出轴目标角度(deg)
 * @retval HAL 状态
 */
HAL_StatusTypeDef Up_SetM2006Pos(uint8_t id, float position_deg);

void Up_Run1ms(void);
void Up_StopAll(void);
bool Up_GetRsStatus(uint8_t id, rs_app_status_t *status);
bool Up_GetDmStatus(uint16_t id, dm_app_status_t *status);
bool Up_GetM2006Status(uint8_t id, m2006_status_t *status);

#endif
