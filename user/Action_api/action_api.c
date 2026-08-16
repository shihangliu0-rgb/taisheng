#include "action_api.h"

#include "up_main.h"

/* 动作位置使用输出轴角度；前机构为 RS，后机构为 DM。 */
#define LEG_RS_L_REBASED_SUPPORT_DEG    183.0f
#define LEG_RS_R_REBASED_SUPPORT_DEG    181.0f
#define LEG_RS_L_LIFT_TRAVEL_DEG \
    (LEG_RS_L_REBASED_SUPPORT_DEG - UP_SECOND_ZERO_OFFSET_DEG)
#define LEG_RS_R_LIFT_TRAVEL_DEG \
    (LEG_RS_R_REBASED_SUPPORT_DEG - UP_SECOND_ZERO_OFFSET_DEG)
#define LEG_RS_L_WRAP_TRAVEL_DEG \
    (360.0f - LEG_RS_L_REBASED_SUPPORT_DEG)
#define LEG_RS_R_WRAP_TRAVEL_DEG \
    (360.0f - LEG_RS_R_REBASED_SUPPORT_DEG)
#define LEG_RS_FLAT_TARGET_DEG           90.0f

/* DM 撑起行程（正向，左右可独立调整）。 */
#define LEG_DM_L_SUPPORT_REFERENCE_DEG 178.0f
#define LEG_DM_R_SUPPORT_REFERENCE_DEG 178.0f
#define LEG_DM_L_LIFT_TRAVEL_DEG \
    (LEG_DM_L_SUPPORT_REFERENCE_DEG - UP_SECOND_ZERO_OFFSET_DEG)
#define LEG_DM_R_LIFT_TRAVEL_DEG \
    (LEG_DM_R_SUPPORT_REFERENCE_DEG - UP_SECOND_ZERO_OFFSET_DEG)

/* DM 放平第一段行程（反向）。 */
#define LEG_DM_L_FLAT_FIRST_TRAVEL_DEG LEG_DM_L_SUPPORT_REFERENCE_DEG
#define LEG_DM_R_FLAT_FIRST_TRAVEL_DEG LEG_DM_R_SUPPORT_REFERENCE_DEG

/* DM 放平第一段结束并标零后的目标（反向）。 */
#define LEG_DM_L_FLAT_TARGET_DEG      (-107.0f) /* M2006 到达蹬台阶位置。 */
#define LEG_DM_R_FLAT_TARGET_DEG      (-107.0f)

/* DM 从放平位置回到等效支撑位置的行程（反向）。 */
#define LEG_DM_L_DOWN_TRAVEL_DEG        75.0f /* -182 度等效于正向 178 度。 */
#define LEG_DM_R_DOWN_TRAVEL_DEG        75.0f

#define LEG_LIFT_FF_REFERENCE_DEG 96.0f
#define LEG_LIFT_FF_TRAVEL_DEG \
    (LEG_LIFT_FF_REFERENCE_DEG - UP_SECOND_ZERO_OFFSET_DEG)
#define LEG_LIFT_RS_TORQUE_NM   4.0f
#define LEG_LIFT_DM_TORQUE_NM   3.0f

#define M2006_FORWARD_ANGLE_DEG 360.0f
#define ACTION_TIMEOUT_MS        5000U

typedef enum
{
    LEG_INITIAL,
    LEG_LIFTED,
    LEG_FLAT
} leg_state_t;

typedef enum
{
    ACTION_IDLE,
    ACTION_WAIT_LOWER,
    ACTION_WAIT_LIFT,
    ACTION_WAIT_FRONT_WRAP,
    ACTION_WAIT_FRONT_ZERO,
    ACTION_WAIT_FRONT_FLAT,
    ACTION_WAIT_FRONT_DOWN,
    ACTION_WAIT_REAR_REVERSE,
    ACTION_WAIT_REAR_ZERO,
    ACTION_WAIT_REAR_FLAT,
    ACTION_WAIT_REAR_DOWN
} action_step_t;

static action_step_t action_step;
static leg_state_t front_state;
static leg_state_t rear_state;
static uint32_t action_due_ms;
static float lift_rs_torque_nm;
static float lift_dm_torque_nm;
static up_motor_angles_t lift_ff_angles;
static bool motors_ready_latched;

volatile action_cmd_t action_pending = ACTION_CMD_NONE;
volatile uint8_t action_pnp_f_trigger;
volatile uint8_t action_pnp_b_trigger;
HAL_StatusTypeDef action_last_result = HAL_OK;

static bool time_reached(uint32_t now_ms, uint32_t due_ms)
{
    return (int32_t)(now_ms - due_ms) >= 0;
}

static void set_action_step(action_step_t next_step)
{
    action_step = next_step;
    action_due_ms = HAL_GetTick() + ACTION_TIMEOUT_MS;
}

static void complete_action(void)
{
    action_last_result = HAL_OK;
    action_step = ACTION_IDLE;
}

static void begin_action_step(HAL_StatusTypeDef status,
                              action_step_t next_step)
{
    action_last_result = status;
    if (status == HAL_OK)
    {
        set_action_step(next_step);
    }
}

static void reset_leg_state(void)
{
    front_state = LEG_INITIAL;
    rear_state = LEG_INITIAL;
}

static void fail_action(HAL_StatusTypeDef status)
{
    action_last_result = status;
    action_step = ACTION_IDLE;
    Up_StopAll();
}

static bool action_allowed(action_cmd_t action)
{
    switch (action)
    {
    case ACTION_CMD_LOWER:
        return (front_state == LEG_LIFTED) && (rear_state == LEG_LIFTED);

    case ACTION_CMD_LIFT:
        return (front_state == LEG_INITIAL) &&
               (rear_state == LEG_INITIAL);

    case ACTION_CMD_FRONT_FLAT:
        return front_state == LEG_LIFTED;

    case ACTION_CMD_FRONT_DOWN:
        return front_state == LEG_FLAT;

    case ACTION_CMD_REAR_FLAT:
        return rear_state == LEG_LIFTED;

    case ACTION_CMD_REAR_DOWN:
        return rear_state == LEG_FLAT;

    case ACTION_CMD_M2006_FORWARD:
    case ACTION_CMD_M2006_COAST:
    case ACTION_CMD_NONE:
    default:
        return true;
    }
}

static void run_action_step(void)
{
    HAL_StatusTypeDef rebase_status;
    uint32_t now_ms = HAL_GetTick();

    switch (action_step)
    {
    case ACTION_WAIT_LOWER:
        if (Up_MotorMoveDone())
        {
            reset_leg_state();
            complete_action();
        }
        break;

    case ACTION_WAIT_LIFT:
    {
        up_motor_angles_t angles;
        bool torque_changed = false;

        if (!Up_GetMotorAngles(&angles))
        {
            break;
        }
        if ((lift_rs_torque_nm == 0.0f) &&
            (angles.rs_l_deg >= lift_ff_angles.rs_l_deg) &&
            (angles.rs_r_deg >= lift_ff_angles.rs_r_deg))
        {
            lift_rs_torque_nm = LEG_LIFT_RS_TORQUE_NM;
            torque_changed = true;
        }
        if ((lift_dm_torque_nm == 0.0f) &&
            (angles.dm_l_deg >= lift_ff_angles.dm_l_deg) &&
            (angles.dm_r_deg >= lift_ff_angles.dm_r_deg))
        {
            lift_dm_torque_nm = LEG_LIFT_DM_TORQUE_NM;
            torque_changed = true;
        }
        if (torque_changed &&
            (Up_SetFeedforward(lift_rs_torque_nm,
                               lift_dm_torque_nm) != HAL_OK))
        {
            fail_action(HAL_ERROR);
            return;
        }
        if (Up_MotorMoveDone() &&
            (lift_rs_torque_nm == LEG_LIFT_RS_TORQUE_NM) &&
            (lift_dm_torque_nm == LEG_LIFT_DM_TORQUE_NM))
        {
            front_state = LEG_LIFTED;
            rear_state = LEG_LIFTED;
            complete_action();
        }
        break;
    }

    case ACTION_WAIT_FRONT_WRAP:
        if (Up_RsMoveDone())
        {
            action_last_result = Up_RebaseRsPosition();
            if (action_last_result != HAL_OK)
            {
                fail_action(action_last_result);
                return;
            }
            set_action_step(ACTION_WAIT_FRONT_ZERO);
        }
        break;

    case ACTION_WAIT_FRONT_ZERO:
        rebase_status = Up_GetPositionRebaseStatus();
        if (rebase_status == HAL_OK)
        {
            action_last_result = Up_SetRsPos(LEG_RS_FLAT_TARGET_DEG,
                                             LEG_RS_FLAT_TARGET_DEG);
            if (action_last_result != HAL_OK)
            {
                fail_action(action_last_result);
                return;
            }
            set_action_step(ACTION_WAIT_FRONT_FLAT);
        }
        else if (rebase_status != HAL_BUSY)
        {
            fail_action(rebase_status);
        }
        break;

    case ACTION_WAIT_FRONT_FLAT:
        if (Up_RsMoveDone())
        {
            front_state = LEG_FLAT;
            complete_action();
        }
        break;

    case ACTION_WAIT_FRONT_DOWN:
        if (Up_RsMoveDone())
        {
            front_state = LEG_LIFTED;
            complete_action();
        }
        break;

    case ACTION_WAIT_REAR_REVERSE:
        if (Up_DmMoveDone())
        {
            action_last_result = Up_RebaseDmPosition();
            if (action_last_result != HAL_OK)
            {
                fail_action(action_last_result);
                return;
            }
            set_action_step(ACTION_WAIT_REAR_ZERO);
        }
        break;

    case ACTION_WAIT_REAR_ZERO:
        rebase_status = Up_GetPositionRebaseStatus();
        if (rebase_status == HAL_OK)
        {
            action_last_result = Up_SetDmPos(LEG_DM_L_FLAT_TARGET_DEG,
                                             LEG_DM_R_FLAT_TARGET_DEG);
            if (action_last_result != HAL_OK)
            {
                fail_action(action_last_result);
                return;
            }
            set_action_step(ACTION_WAIT_REAR_FLAT);
        }
        else if (rebase_status != HAL_BUSY)
        {
            fail_action(rebase_status);
        }
        break;

    case ACTION_WAIT_REAR_FLAT:
        if (Up_DmMoveDone())
        {
            rear_state = LEG_FLAT;
            complete_action();
        }
        break;

    case ACTION_WAIT_REAR_DOWN:
        if (Up_DmMoveDone())
        {
            rear_state = LEG_LIFTED;
            complete_action();
        }
        break;

    case ACTION_IDLE:
    default:
        break;
    }

    if ((action_step != ACTION_IDLE) && time_reached(now_ms, action_due_ms))
    {
        fail_action(HAL_TIMEOUT);
    }
}

static bool action_ready(action_cmd_t action)
{
    if (action == ACTION_CMD_M2006_FORWARD)
    {
        return Up_IsM2006Ready();
    }
    if ((action == ACTION_CMD_M2006_COAST) || (action == ACTION_CMD_NONE))
    {
        return true;
    }
    return Up_IsReady();
}

static void defer_action(action_cmd_t action)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (action_pending == ACTION_CMD_NONE)
    {
        action_pending = action;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static action_cmd_t take_pending_action(void)
{
    action_cmd_t action;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    action = action_pending;
    action_pending = ACTION_CMD_NONE;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return action;
}

static void start_action(action_cmd_t action)
{
    switch (action)
    {
    case ACTION_CMD_LOWER:
        /* 四台电机沿撑起反方向回到待机位置。 */
        begin_action_step(
            Up_SetMotorPos(
                up_target_angles.rs_l_deg - LEG_RS_L_LIFT_TRAVEL_DEG,
                up_target_angles.rs_r_deg - LEG_RS_R_LIFT_TRAVEL_DEG,
                up_target_angles.dm_l_deg - LEG_DM_L_LIFT_TRAVEL_DEG,
                up_target_angles.dm_r_deg - LEG_DM_R_LIFT_TRAVEL_DEG),
            ACTION_WAIT_LOWER);
        break;

    case ACTION_CMD_LIFT:
        /* 四台电机沿各自撑起方向运动到支撑位置。 */
        lift_rs_torque_nm = 0.0f;
        lift_dm_torque_nm = 0.0f;
        lift_ff_angles.rs_l_deg = up_target_angles.rs_l_deg +
                                  LEG_LIFT_FF_TRAVEL_DEG;
        lift_ff_angles.rs_r_deg = up_target_angles.rs_r_deg +
                                  LEG_LIFT_FF_TRAVEL_DEG;
        lift_ff_angles.dm_l_deg = up_target_angles.dm_l_deg +
                                  LEG_LIFT_FF_TRAVEL_DEG;
        lift_ff_angles.dm_r_deg = up_target_angles.dm_r_deg +
                                   LEG_LIFT_FF_TRAVEL_DEG;
        begin_action_step(
            Up_SetMotorPos(
                up_target_angles.rs_l_deg + LEG_RS_L_LIFT_TRAVEL_DEG,
                up_target_angles.rs_r_deg + LEG_RS_R_LIFT_TRAVEL_DEG,
                up_target_angles.dm_l_deg + LEG_DM_L_LIFT_TRAVEL_DEG,
                up_target_angles.dm_r_deg + LEG_DM_R_LIFT_TRAVEL_DEG),
            ACTION_WAIT_LIFT);
        break;

    case ACTION_CMD_M2006_FORWARD:
        /* 两台 M2006 沿前进方向转一圈。 */
        action_last_result = Up_MoveM2006(M2006_FORWARD_ANGLE_DEG);
        break;

    case ACTION_CMD_FRONT_FLAT:
        /* 两台 RS 沿撑起方向继续绕转，重设零点后再到 90 度。 */
        begin_action_step(
            Up_SetRsPos(
                up_target_angles.rs_l_deg + LEG_RS_L_WRAP_TRAVEL_DEG,
                up_target_angles.rs_r_deg + LEG_RS_R_WRAP_TRAVEL_DEG),
            ACTION_WAIT_FRONT_WRAP);
        break;

    case ACTION_CMD_FRONT_DOWN:
        /* 两台 RS 从放平位置回到各自支撑角度。 */
        begin_action_step(
            Up_SetRsPos(LEG_RS_L_REBASED_SUPPORT_DEG,
                        LEG_RS_R_REBASED_SUPPORT_DEG),
            ACTION_WAIT_FRONT_DOWN);
        break;

    case ACTION_CMD_REAR_FLAT:
        /* DM 反向 178 度，重设零点后继续反向 107 度。 */
        begin_action_step(
            Up_SetDmPos(
                up_target_angles.dm_l_deg -
                    LEG_DM_L_FLAT_FIRST_TRAVEL_DEG,
                up_target_angles.dm_r_deg -
                    LEG_DM_R_FLAT_FIRST_TRAVEL_DEG),
            ACTION_WAIT_REAR_REVERSE);
        break;

    case ACTION_CMD_REAR_DOWN:
        /* DM 从放平位置继续沿反方向回到等效支撑位置。 */
        begin_action_step(
            Up_SetDmPos(
                up_target_angles.dm_l_deg - LEG_DM_L_DOWN_TRAVEL_DEG,
                up_target_angles.dm_r_deg - LEG_DM_R_DOWN_TRAVEL_DEG),
            ACTION_WAIT_REAR_DOWN);
        break;

    case ACTION_CMD_M2006_COAST:
        /* 两台 M2006 关闭位置控制并进入滑行状态。 */
        action_last_result = Up_CoastM2006();
        break;

    case ACTION_CMD_NONE:
    default:
        break;
    }
}

void Action_UpdatePnp(uint8_t trigger_f, uint8_t trigger_b)
{
    action_pnp_f_trigger = (trigger_f != 0U) ? 1U : 0U;
    action_pnp_b_trigger = (trigger_b != 0U) ? 1U : 0U;
}

HAL_StatusTypeDef Action_Request(action_cmd_t action)
{
    if (action > ACTION_CMD_MAX)
    {
        return HAL_ERROR;
    }

    action_pending = action;
    return HAL_OK;
}

void Action_Run1ms(void)
{
    action_cmd_t action;
    bool motors_ready;

    motors_ready = Up_IsReady();
    if (!motors_ready)
    {
        motors_ready_latched = false;
    }
    else if (!motors_ready_latched)
    {
        reset_leg_state();
        motors_ready_latched = true;
    }

    if (action_step != ACTION_IDLE)
    {
        run_action_step();
        return;
    }

    action = take_pending_action();

    if (action == ACTION_CMD_NONE)
    {
        return;
    }
    if (!action_ready(action))
    {
        defer_action(action);
        return;
    }
    if (!action_allowed(action))
    {
        action_last_result = HAL_ERROR;
        return;
    }

    start_action(action);
}
