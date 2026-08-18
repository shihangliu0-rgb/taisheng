#ifndef ACTION_API_H
#define ACTION_API_H

#include "stm32h7xx_hal.h"

#include <stdint.h>

typedef enum
{
    ACTION_CMD_LOWER = 0U,        /* 四台电机沿撑起反方向回待机位。 */
    ACTION_CMD_LIFT = 1U,         /* 四台抬升电机运动到支撑角度。 */
    ACTION_CMD_M2006_FORWARD = 2U, /* 两台 M2006 向前转一圈。 */
    ACTION_CMD_FRONT_FLAT = 3U,   /* 前部 RS 绕转、标零并放平。 */
    ACTION_CMD_FRONT_DOWN = 4U,   /* 前部 RS 从放平回到支撑。 */
    ACTION_CMD_REAR_FLAT = 5U,    /* 后部 DM 反向绕转、标零并放平。 */
    ACTION_CMD_REAR_DOWN = 6U,    /* 后部 DM 继续反向回到支撑。 */
    ACTION_CMD_M2006_COAST = 7U,  /* 两台 M2006 失去位置保持。 */
    ACTION_CMD_ALIGN = 8U,        /* 四台抬升电机在第二次零点与 +45 度对准位之间切换。 */
    ACTION_CMD_NONE = 0xFFU
} action_cmd_t;

#define ACTION_CMD_MAX ACTION_CMD_ALIGN

extern volatile action_cmd_t action_pending;
extern volatile uint8_t action_pnp_f_trigger;
extern volatile uint8_t action_pnp_b_trigger;
extern HAL_StatusTypeDef action_last_result;

/**
 * @brief 更新动作层使用的两个 PNP 触发状态
 * @param trigger_f PNP_F 触发状态
 * @param trigger_b PNP_B 触发状态
 * @retval None
 */
void Action_UpdatePnp(uint8_t trigger_f, uint8_t trigger_b);

/**
 * @brief 更新遥控 PD6，并在有效电平边沿请求一次对准动作
 * @param switch_state PD6 当前状态，1 表示闭合
 * @param online 遥控链路是否在线
 * @retval None
 */
void Action_UpdateAlignSwitch(uint8_t switch_state, uint8_t online);

/**
 * @brief 更新遥控 PE0，并在有效电平边沿交替请求抬升/下沉
 * @param switch_state PE0 当前状态，1 表示闭合
 * @param online 遥控链路是否在线
 * @retval None
 */
void Action_UpdateLiftSwitch(uint8_t switch_state, uint8_t online);

/**
 * @brief Update the six local remote buttons and request actions on press edges.
 * @param buttons KEY_1 through KEY_6 in bits 0 through 5
 * @param online Nonzero while the remote link is online
 * @retval None
 */
void Action_UpdateRemoteButtons(uint8_t buttons, uint8_t online);

/**
 * @brief 提交一个机构动作
 * @param action 动作命令
 * @retval HAL 状态
 */
HAL_StatusTypeDef Action_Request(action_cmd_t action);

/**
 * @brief 在抬升任务中执行待处理动作
 * @retval None
 */
void Action_Run1ms(void);

#endif
