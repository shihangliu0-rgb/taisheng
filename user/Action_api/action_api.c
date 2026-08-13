#include "action_api.h"

#include "up_main.h"

/* 动作命令使用的输出轴固定位置，单位为度。 */
#define LEG_HOME_ANGLE_DEG      0.0f
#define LEG_LIFT_ANGLE_DEG      180.0f
#define LEG_FOLD_ANGLE_DEG      270.0f
#define LEG_FLAT_ANGLE_DEG      90.0f
#define LEG_DOWN_ANGLE_DEG      180.0f
#define M2006_FORWARD_ANGLE_DEG 360.0f

typedef enum
{
    ACTION_STEP_IDLE,
    ACTION_STEP_WAIT_FRONT_HOME,
    ACTION_STEP_WAIT_FRONT_FLAT,
    ACTION_STEP_WAIT_REAR_HOME,
    ACTION_STEP_WAIT_REAR_FLAT
} action_step_t;

static action_step_t action_step;

volatile action_cmd_t action_pending = ACTION_CMD_NONE;
volatile uint8_t action_pnp_f_trigger;
volatile uint8_t action_pnp_b_trigger;
HAL_StatusTypeDef action_last_result = HAL_OK;

static HAL_StatusTypeDef set_all_leg_pos(float angle_deg)
{
    return Up_SetMotorPos(angle_deg, angle_deg, angle_deg, angle_deg);
}

static void run_action_sequence(void)
{
    switch (action_step)
    {
    case ACTION_STEP_WAIT_FRONT_HOME:
        if (Up_MotorMoveDone())
        {
            action_last_result = Up_SetRsPos(LEG_FLAT_ANGLE_DEG);
            action_step = (action_last_result == HAL_OK)
                              ? ACTION_STEP_WAIT_FRONT_FLAT
                              : ACTION_STEP_IDLE;
        }
        break;

    case ACTION_STEP_WAIT_FRONT_FLAT:
        if (Up_MotorMoveDone())
        {
            action_step = ACTION_STEP_IDLE;
        }
        break;

    case ACTION_STEP_WAIT_REAR_HOME:
        if (Up_MotorMoveDone())
        {
            action_last_result = Up_SetDmPos(LEG_FLAT_ANGLE_DEG);
            action_step = (action_last_result == HAL_OK)
                              ? ACTION_STEP_WAIT_REAR_FLAT
                              : ACTION_STEP_IDLE;
        }
        break;

    case ACTION_STEP_WAIT_REAR_FLAT:
        if (Up_MotorMoveDone())
        {
            action_step = ACTION_STEP_IDLE;
        }
        break;

    case ACTION_STEP_IDLE:
    default:
        break;
    }
}

/**
 * @brief 更新动作层使用的两个 PNP 触发状态
 * @param trigger_f PNP_F 触发状态
 * @param trigger_b PNP_B 触发状态
 * @retval None
 */
void Action_UpdatePnp(uint8_t trigger_f, uint8_t trigger_b)
{
    action_pnp_f_trigger = (trigger_f != 0U) ? 1U : 0U;
    action_pnp_b_trigger = (trigger_b != 0U) ? 1U : 0U;
}

/**
 * @brief 提交一个机构动作
 * @param action 动作命令
 * @retval HAL 状态
 */
HAL_StatusTypeDef Action_Request(action_cmd_t action)
{
    if (action > ACTION_CMD_REAR_DOWN)
    {
        return HAL_ERROR;
    }

    action_pending = action;
    return HAL_OK;
}

/**
 * @brief 在抬升任务中执行待处理动作
 * @retval None
 */
void Action_Run1ms(void)
{
    action_cmd_t action;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    action = action_pending;
    action_pending = ACTION_CMD_NONE;
    if (primask == 0U)
    {
        __enable_irq();
    }

    if (action == ACTION_CMD_NONE)
    {
        run_action_sequence();
        return;
    }

    action_step = ACTION_STEP_IDLE;
    switch (action)
    {
    case ACTION_CMD_LOWER:
        action_last_result = set_all_leg_pos(LEG_HOME_ANGLE_DEG);
        break;

    case ACTION_CMD_LIFT:
        action_last_result = set_all_leg_pos(LEG_LIFT_ANGLE_DEG);
        break;

    case ACTION_CMD_M2006_FORWARD:
        action_last_result = Up_MoveM2006(M2006_FORWARD_ANGLE_DEG);
        break;

    case ACTION_CMD_FRONT_FOLD:
        action_last_result = Up_SetRsPos(LEG_FOLD_ANGLE_DEG);
        break;

    case ACTION_CMD_FRONT_FLAT:
        action_last_result = Up_SetRsPos(LEG_HOME_ANGLE_DEG);
        if (action_last_result == HAL_OK)
        {
            action_step = ACTION_STEP_WAIT_FRONT_HOME;
        }
        break;

    case ACTION_CMD_FRONT_DOWN:
        action_last_result = Up_SetRsPos(LEG_DOWN_ANGLE_DEG);
        break;

    case ACTION_CMD_REAR_FOLD:
        action_last_result = Up_SetDmPos(LEG_FOLD_ANGLE_DEG);
        break;

    case ACTION_CMD_REAR_FLAT:
        action_last_result = Up_SetDmPos(LEG_HOME_ANGLE_DEG);
        if (action_last_result == HAL_OK)
        {
            action_step = ACTION_STEP_WAIT_REAR_HOME;
        }
        break;

    case ACTION_CMD_REAR_DOWN:
        action_last_result = Up_SetDmPos(LEG_DOWN_ANGLE_DEG);
        break;

    case ACTION_CMD_NONE:
    default:
        break;
    }
}
