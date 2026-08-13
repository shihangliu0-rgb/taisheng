#ifndef ACTION_API_H
#define ACTION_API_H

#include "stm32h7xx_hal.h"

#include <stdint.h>

typedef enum
{
    ACTION_CMD_LOWER = 0U,
    ACTION_CMD_LIFT = 1U,
    ACTION_CMD_M2006_FORWARD = 2U,
    ACTION_CMD_FRONT_FOLD = 3U,
    ACTION_CMD_FRONT_FLAT = 4U,
    ACTION_CMD_FRONT_DOWN = 5U,
    ACTION_CMD_REAR_FOLD = 6U,
    ACTION_CMD_REAR_FLAT = 7U,
    ACTION_CMD_REAR_DOWN = 8U,
    ACTION_CMD_NONE = 0xFFU
} action_cmd_t;

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
